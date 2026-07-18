using System.Diagnostics;
using Ref = ArcCollision.Ref;
using Wrapper = ArcCollision.Wrapper;

namespace ArcCollision.Benchmarks;

// Measures ArcWorld.QueryBatch. Three timings are compared for the same query
// set, so the packet's contribution is separable from the managed/native gap:
//   * Ref-loop     - reference backend, one Query per shape (managed).
//   * Native-loop  - native backend, one arc_world_query per shape (scalar path,
//                    N P/Invokes, no packet).
//   * Native-batch - native backend, arc_world_query_batch: four queries at a time
//                    through a 4-wide SIMD packet descent, one P/Invoke.
// So Native-batch/Native-loop isolates the batch+packet win inside the native
// backend, while Native-batch/Ref-loop is the end-to-end speedup a managed caller
// sees. Colliders are held fixed; only the queries are timed. Sparse queries are
// scattered uniformly (mostly empty -> traversal-bound); dense queries are centred
// on real colliders (many hits -> result-bound and spatially coherent).
internal static class QueryBatchBenchmark
{
    private const float SparseHalfExtent = 4f;
    private const float DenseHalfExtent = 40f;
    private static readonly int[] BatchSizes = { 256, 1024, 4096, 16384 };

    // The query workload uses its own fixed seed, independent of --seed, so the
    // generated boxes (and therefore the batch-query rows) are identical on every
    // run and comparable across runs. --seed still varies the collider scene.
    private const ulong QuerySeed = 0xA11CE5EEDUL;

    public static void Run(
        BenchmarkScenario scenario,
        RefPreparedScene refScene,
        WrapperPreparedScene wrapperScene,
        BenchmarkOptions options,
        int threadId)
    {
        Console.WriteLine();
        Console.WriteLine("Batch query benchmark (ArcWorld.QueryBatch)");
        Console.WriteLine($"World: static={options.StaticCount}, dynamic={options.DynamicCount} "
            + "(colliders held fixed; only the query is timed)");
        Console.WriteLine("Sparse=scattered; Dense=per-query collider; Coherent=4-query bundles share a collider");
        Console.WriteLine("Mode      N        Ref-loop  Nat-loop  Nat-batch  batch/loop  batch/ref  Hits/query");

        int[] centers = ColliderCenters(scenario);
        using Ref.ArcWorld refWorld = BuildRefWorld(refScene, options);
        using Wrapper.ArcWorld wrapperWorld = BuildWrapperWorld(wrapperScene, options);

        foreach (QueryMode mode in new[] { QueryMode.Sparse, QueryMode.Dense, QueryMode.Coherent })
        {
            float half = mode == QueryMode.Sparse ? SparseHalfExtent : DenseHalfExtent;
            foreach (int n in BatchSizes)
                Measure(refWorld, wrapperWorld, centers, options, threadId, mode, half, n);
        }
    }

    // Sparse: boxes scattered uniformly (mostly empty). Dense: each box on a random
    // collider. Coherent: each group of four boxes shares a collider, so a packet's
    // four queries descend overlapping node sets -- the case the packet is built for.
    private enum QueryMode { Sparse, Dense, Coherent }

    private static void Measure(
        Ref.ArcWorld refWorld,
        Wrapper.ArcWorld wrapperWorld,
        int[] centers,
        BenchmarkOptions options,
        int threadId,
        QueryMode mode,
        float half,
        int n)
    {
        (Ref.Shape[] refQueries, Wrapper.Shape[] wrapperQueries) =
            BuildQueries(centers, mode, half, n);

        var refResults = new List<Ref.ArcHandle>(n * 4);
        var refCounts = new List<int>(n);
        var wrapperResults = new List<Wrapper.ArcHandle>(n * 4);
        var wrapperCounts = new List<int>(n);
        var scratch = new List<Wrapper.ArcHandle>(64);

        // Warm all three paths and confirm the backends agree before timing.
        refWorld.QueryBatch(refQueries, refResults, refCounts);
        wrapperWorld.QueryBatch(wrapperQueries, wrapperResults, wrapperCounts);
        ValidateEquivalent(refResults, refCounts, wrapperResults, wrapperCounts, mode, n);
        long totalHits = refResults.Count;

        double refLoopMs = BestMedianMs(options, threadId,
            () => refWorld.QueryBatch(refQueries, refResults, refCounts));
        double nativeLoopMs = BestMedianMs(options, threadId, () =>
        {
            wrapperResults.Clear();
            for (int i = 0; i < wrapperQueries.Length; i++)
            {
                wrapperWorld.Query(wrapperQueries[i], scratch);
                wrapperResults.AddRange(scratch);
            }
        });
        double nativeBatchMs = BestMedianMs(options, threadId,
            () => wrapperWorld.QueryBatch(wrapperQueries, wrapperResults, wrapperCounts));

        double hitsPerQuery = (double)totalHits / n;
        Console.WriteLine($"{mode,-8}  {n,-7}  "
            + $"{refLoopMs,7:0.000}  {nativeLoopMs,8:0.000}  {nativeBatchMs,9:0.000}  "
            + $"{nativeLoopMs / nativeBatchMs,9:0.00}x  {refLoopMs / nativeBatchMs,8:0.00}x  {hitsPerQuery,10:0.00}");
    }

    private static double BestMedianMs(
        BenchmarkOptions options, int threadId, Action action)
    {
        for (int i = 0; i < options.WarmupIterations; i++) action();
        var samples = new double[options.Iterations];
        for (int i = 0; i < options.Iterations; i++)
        {
            EnsureThread(threadId);
            long start = Stopwatch.GetTimestamp();
            action();
            samples[i] = Stopwatch.GetElapsedTime(start).TotalMilliseconds;
        }
        Array.Sort(samples);
        return samples[samples.Length / 2];
    }

    // Raw 24.8 centres of every collider, so dense queries can be placed on them.
    private static int[] ColliderCenters(BenchmarkScenario scenario)
    {
        int staticCount = scenario.StaticShapes.Length;
        int dynamicCount = scenario.DynamicInitialShapes.Length;
        var centers = new int[(staticCount + dynamicCount) * 2];
        int at = 0;
        foreach (ShapeSpec spec in scenario.StaticShapes)
        {
            centers[at++] = spec.X;
            centers[at++] = spec.Y;
        }
        foreach (ShapeSpec spec in scenario.DynamicInitialShapes)
        {
            centers[at++] = spec.X;
            centers[at++] = spec.Y;
        }
        return centers;
    }

    private static (Ref.Shape[], Wrapper.Shape[]) BuildQueries(
        int[] centers, QueryMode mode, float half, int n)
    {
        var random = new StableRandom(QuerySeed ^ (ulong)mode * 0x9E3779B9UL ^ (ulong)n);
        int colliderCount = centers.Length / 2;
        var refQueries = new Ref.Shape[n];
        var wrapperQueries = new Wrapper.Shape[n];
        int groupCollider = 0;
        for (int i = 0; i < n; i++)
        {
            float x, y;
            if (mode == QueryMode.Sparse)
            {
                x = random.NextInt(-5200 * 256, 5200 * 256 + 1) * (1f / 256f);
                y = random.NextInt(-5200 * 256, 5200 * 256 + 1) * (1f / 256f);
            }
            else
            {
                // Dense: a fresh collider per query. Coherent: one collider shared by
                // each group of four, so a packet's four queries stay near each other.
                if (mode == QueryMode.Dense || (i & 3) == 0)
                    groupCollider = random.NextInt(colliderCount);
                x = (centers[groupCollider * 2] + random.NextInt(-8 * 256, 8 * 256 + 1)) * (1f / 256f);
                y = (centers[groupCollider * 2 + 1] + random.NextInt(-8 * 256, 8 * 256 + 1)) * (1f / 256f);
            }
            refQueries[i] = new Ref.Aabb(new Ref.Vec2(x, y), new Ref.Vec2(half, half));
            wrapperQueries[i] = new Wrapper.Aabb(new Wrapper.Vec2(x, y), new Wrapper.Vec2(half, half));
        }
        return (refQueries, wrapperQueries);
    }

    private static Ref.ArcWorld BuildRefWorld(RefPreparedScene scene, BenchmarkOptions options)
    {
        int total = options.StaticCount + options.DynamicCount;
        var world = new Ref.ArcWorld(new Ref.ArcWorldOptions(
            options.FatMargin, total, Math.Max(1024, total * 8)));
        for (int i = 0; i < scene.StaticShapes.Length; i++)
            world.AddStatic(i, scene.StaticShapes[i]);
        for (int i = 0; i < scene.DynamicInitialShapes.Length; i++)
            world.Add(options.StaticCount + i, scene.DynamicInitialShapes[i]);
        world.BuildStatic();
        return world;
    }

    private static Wrapper.ArcWorld BuildWrapperWorld(
        WrapperPreparedScene scene, BenchmarkOptions options)
    {
        int total = options.StaticCount + options.DynamicCount;
        var world = new Wrapper.ArcWorld(new Wrapper.ArcWorldOptions(
            options.FatMargin, total, Math.Max(1024, total * 8)));
        for (int i = 0; i < scene.StaticShapes.Length; i++)
            world.AddStatic(i, scene.StaticShapes[i]);
        for (int i = 0; i < scene.DynamicInitialShapes.Length; i++)
            world.Add(options.StaticCount + i, scene.DynamicInitialShapes[i]);
        world.BuildStatic();
        return world;
    }

    private static void ValidateEquivalent(
        List<Ref.ArcHandle> refResults, List<int> refCounts,
        List<Wrapper.ArcHandle> wrapperResults, List<int> wrapperCounts,
        QueryMode mode, int n)
    {
        bool ok = refResults.Count == wrapperResults.Count
            && refCounts.Count == wrapperCounts.Count;
        for (int i = 0; ok && i < refCounts.Count; i++)
            ok = refCounts[i] == wrapperCounts[i];
        for (int i = 0; ok && i < refResults.Count; i++)
            ok = refResults[i].EntityId == wrapperResults[i].EntityId;
        if (!ok)
        {
            throw new InvalidOperationException(
                $"Batch query backends disagree (mode={mode}, N={n}): "
                + $"Ref {refResults.Count} handles / {refCounts.Count} counts, "
                + $"Native {wrapperResults.Count} handles / {wrapperCounts.Count} counts.");
        }
    }

    private static void EnsureThread(int expectedThreadId)
    {
        if (Environment.CurrentManagedThreadId != expectedThreadId)
            throw new InvalidOperationException("Benchmark execution changed threads.");
    }
}
