using System.Diagnostics;
using Ref = ArcCollision.Ref;
using Wrapper = ArcCollision.Wrapper;

namespace ArcCollision.Benchmarks;

internal readonly record struct TrialResult(
    TimeSpan BuildTime,
    TimeSpan SimulationTime,
    long CandidateCount,
    long CollisionCount,
    ulong Checksum);

internal static class BackendRunners
{
    private const ulong HashOffset = 14695981039346656037UL;
    private const ulong HashPrime = 1099511628211UL;

    public static TrialResult RunRef(
        RefPreparedScene scene, BenchmarkOptions options, int expectedThreadId)
    {
        EnsureThread(expectedThreadId);
        int pairCapacity = PairCapacity(options);
        var dynamicHandles = new Ref.ArcHandle[options.DynamicCount];
        var pairs = new List<Ref.CandidatePair>(pairCapacity);

        long buildStart = Stopwatch.GetTimestamp();
        using var world = new Ref.ArcWorld(new Ref.ArcWorldOptions(
            options.FatMargin,
            options.StaticCount + options.DynamicCount,
            pairCapacity));
        for (int i = 0; i < scene.StaticShapes.Length; i++)
            world.AddStatic(i, scene.StaticShapes[i]);
        for (int i = 0; i < scene.DynamicInitialShapes.Length; i++)
        {
            dynamicHandles[i] = world.Add(
                options.StaticCount + i, scene.DynamicInitialShapes[i]);
        }
        world.BuildStatic();
        long buildEnd = Stopwatch.GetTimestamp();

        ulong checksum = HashOffset;
        long candidateCount = 0;
        long collisionCount = 0;
        long simulationStart = Stopwatch.GetTimestamp();
        for (int frame = 0; frame < options.Frames; frame++)
        {
            int shapeOffset = frame * options.DynamicCount;
            for (int i = 0; i < dynamicHandles.Length; i++)
                world.Update(dynamicHandles[i], scene.DynamicFrameShapes[shapeOffset + i]);

            world.ComputePairs(pairs);
            candidateCount += pairs.Count;
            checksum = Add(checksum, unchecked((uint)frame));
            checksum = Add(checksum, unchecked((uint)pairs.Count));
            for (int i = 0; i < pairs.Count; i++)
            {
                Ref.CandidatePair pair = pairs[i];
                checksum = Add(checksum, unchecked((uint)pair.A.EntityId));
                checksum = Add(checksum, unchecked((uint)pair.B.EntityId));
                bool colliding = world.TryComputeContact(pair, out Ref.ContactPair contact);
                checksum = Add(checksum, colliding ? 1u : 0u);
                if (!colliding) continue;
                collisionCount++;
                Ref.Manifold manifold = contact.Manifold;
                checksum = AddManifold(checksum, manifold.Normal.X, manifold.Normal.Y,
                    manifold.Depth, manifold.Contact.X, manifold.Contact.Y);
            }
        }
        long simulationEnd = Stopwatch.GetTimestamp();
        EnsureThread(expectedThreadId);
        return new TrialResult(
            Stopwatch.GetElapsedTime(buildStart, buildEnd),
            Stopwatch.GetElapsedTime(simulationStart, simulationEnd),
            candidateCount, collisionCount, checksum);
    }

    public static TrialResult RunWrapper(
        WrapperPreparedScene scene, BenchmarkOptions options, int expectedThreadId)
    {
        EnsureThread(expectedThreadId);
        int pairCapacity = PairCapacity(options);
        var dynamicHandles = new Wrapper.ArcHandle[options.DynamicCount];
        var pairs = new List<Wrapper.CandidatePair>(pairCapacity);

        long buildStart = Stopwatch.GetTimestamp();
        using var world = new Wrapper.ArcWorld(new Wrapper.ArcWorldOptions(
            options.FatMargin,
            options.StaticCount + options.DynamicCount,
            pairCapacity));
        for (int i = 0; i < scene.StaticShapes.Length; i++)
            world.AddStatic(i, scene.StaticShapes[i]);
        for (int i = 0; i < scene.DynamicInitialShapes.Length; i++)
        {
            dynamicHandles[i] = world.Add(
                options.StaticCount + i, scene.DynamicInitialShapes[i]);
        }
        world.BuildStatic();
        long buildEnd = Stopwatch.GetTimestamp();

        ulong checksum = HashOffset;
        long candidateCount = 0;
        long collisionCount = 0;
        long simulationStart = Stopwatch.GetTimestamp();
        for (int frame = 0; frame < options.Frames; frame++)
        {
            int shapeOffset = frame * options.DynamicCount;
            for (int i = 0; i < dynamicHandles.Length; i++)
                world.Update(dynamicHandles[i], scene.DynamicFrameShapes[shapeOffset + i]);

            world.ComputePairs(pairs);
            candidateCount += pairs.Count;
            checksum = Add(checksum, unchecked((uint)frame));
            checksum = Add(checksum, unchecked((uint)pairs.Count));
            for (int i = 0; i < pairs.Count; i++)
            {
                Wrapper.CandidatePair pair = pairs[i];
                checksum = Add(checksum, unchecked((uint)pair.A.EntityId));
                checksum = Add(checksum, unchecked((uint)pair.B.EntityId));
                bool colliding = world.TryComputeContact(pair, out Wrapper.ContactPair contact);
                checksum = Add(checksum, colliding ? 1u : 0u);
                if (!colliding) continue;
                collisionCount++;
                Wrapper.Manifold manifold = contact.Manifold;
                checksum = AddManifold(checksum, manifold.Normal.X, manifold.Normal.Y,
                    manifold.Depth, manifold.Contact.X, manifold.Contact.Y);
            }
        }
        long simulationEnd = Stopwatch.GetTimestamp();
        EnsureThread(expectedThreadId);
        return new TrialResult(
            Stopwatch.GetElapsedTime(buildStart, buildEnd),
            Stopwatch.GetElapsedTime(simulationStart, simulationEnd),
            candidateCount, collisionCount, checksum);
    }

    private static int PairCapacity(BenchmarkOptions options) =>
        Math.Max(1024, checked((options.StaticCount + options.DynamicCount) * 8));

    private static ulong Add(ulong hash, uint value) =>
        unchecked((hash ^ value) * HashPrime);

    private static ulong AddManifold(
        ulong hash, float normalX, float normalY,
        float depth, float contactX, float contactY)
    {
        hash = Add(hash, BitConverter.SingleToUInt32Bits(normalX));
        hash = Add(hash, BitConverter.SingleToUInt32Bits(normalY));
        hash = Add(hash, BitConverter.SingleToUInt32Bits(depth));
        hash = Add(hash, BitConverter.SingleToUInt32Bits(contactX));
        return Add(hash, BitConverter.SingleToUInt32Bits(contactY));
    }

    private static void EnsureThread(int expectedThreadId)
    {
        if (Environment.CurrentManagedThreadId != expectedThreadId)
            throw new InvalidOperationException("Benchmark execution changed threads.");
    }
}
