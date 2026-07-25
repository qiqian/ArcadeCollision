/*
 * BackendRunners.cs
 * ArcCollision.Benchmarks - deterministic performance benchmark suite.
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System.Diagnostics;
using Ref = ArcCollision.Ref;
using Wrapper = ArcCollision.Wrapper;

namespace ArcCollision.Benchmarks;

internal readonly record struct TrialResult(
    TimeSpan BuildTime,
    TimeSpan SimulationTime,
    long PairCount,
    long CollisionCount,
    long AllocatedBytes,
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
        var confirmed = new List<Ref.ContactPair>(pairCapacity);
        var candidates = new List<Ref.CandidatePair>(pairCapacity);
        var resolved = new List<Ref.ContactPair>(pairCapacity);

        long allocatedStart = GC.GetAllocatedBytesForCurrentThread();
        long buildStart = Stopwatch.GetTimestamp();
        using var world = new Ref.ArcWorld(new Ref.ArcWorldOptions(
            options.FatMargin,
            options.StaticCount + options.DynamicCount,
            pairCapacity));
        for (int i = 0; i < scene.StaticShapes.Length; i++)
            world.AddStatic(i, scene.StaticShapes[i], Ref.CollisionFilter.Default);
        for (int i = 0; i < scene.DynamicInitialShapes.Length; i++)
        {
            dynamicHandles[i] = world.Add(
                options.StaticCount + i, scene.DynamicInitialShapes[i], Ref.CollisionFilter.Default);
        }
        world.BuildStatic();
        long buildEnd = Stopwatch.GetTimestamp();

        ulong checksum = HashOffset;
        long pairTotal = 0;
        long collisionCount = 0;
        long simulationStart = Stopwatch.GetTimestamp();
        for (int frame = 0; frame < options.Frames; frame++)
        {
            int shapeOffset = frame * options.DynamicCount;
            for (int i = 0; i < dynamicHandles.Length; i++)
                world.UpdateTransform(
                    dynamicHandles[i], scene.DynamicFrameTransforms[shapeOffset + i]);

            world.ComputePairs(confirmed, candidates, Ref.ManifoldFields.All);
            world.TryComputeContacts(candidates, resolved, Ref.ManifoldFields.All);
            int pairCount = confirmed.Count + candidates.Count;
            pairTotal += pairCount;
            collisionCount += confirmed.Count + resolved.Count;
            checksum = Add(checksum, unchecked((uint)frame));
            checksum = Add(checksum, unchecked((uint)pairCount));
            for (int i = 0; i < confirmed.Count; i++)
            {
                Ref.ContactPair contact = confirmed[i];
                checksum = Add(checksum, unchecked((uint)contact.A.EntityId));
                checksum = Add(checksum, unchecked((uint)contact.B.EntityId));
                checksum = Add(checksum, 1u);
                Ref.Manifold manifold = contact.Manifold;
                checksum = AddManifold(checksum, manifold.Normal.X, manifold.Normal.Y,
                    manifold.Depth, manifold.Contact.X, manifold.Contact.Y);
            }
            for (int i = 0; i < candidates.Count; i++)
            {
                Ref.CandidatePair pair = candidates[i];
                checksum = Add(checksum, unchecked((uint)pair.A.EntityId));
                checksum = Add(checksum, unchecked((uint)pair.B.EntityId));
            }
            checksum = Add(checksum, unchecked((uint)resolved.Count));
            for (int i = 0; i < resolved.Count; i++)
            {
                Ref.ContactPair contact = resolved[i];
                checksum = Add(checksum, unchecked((uint)contact.A.EntityId));
                checksum = Add(checksum, unchecked((uint)contact.B.EntityId));
                Ref.Manifold manifold = contact.Manifold;
                checksum = AddManifold(checksum, manifold.Normal.X, manifold.Normal.Y,
                    manifold.Depth, manifold.Contact.X, manifold.Contact.Y);
            }
        }
        long simulationEnd = Stopwatch.GetTimestamp();
        long allocatedBytes = GC.GetAllocatedBytesForCurrentThread() - allocatedStart;
        EnsureThread(expectedThreadId);
        return new TrialResult(
            Stopwatch.GetElapsedTime(buildStart, buildEnd),
            Stopwatch.GetElapsedTime(simulationStart, simulationEnd),
            pairTotal, collisionCount, allocatedBytes, checksum);
    }

    public static TrialResult RunWrapper(
        WrapperPreparedScene scene, BenchmarkOptions options, int expectedThreadId)
    {
        EnsureThread(expectedThreadId);
        int pairCapacity = PairCapacity(options);
        var dynamicHandles = new Wrapper.ArcHandle[options.DynamicCount];
        var confirmed = new List<Wrapper.ContactPair>(pairCapacity);
        var candidates = new List<Wrapper.CandidatePair>(pairCapacity);
        var resolved = new List<Wrapper.ContactPair>(pairCapacity);

        long allocatedStart = GC.GetAllocatedBytesForCurrentThread();
        long buildStart = Stopwatch.GetTimestamp();
        using var world = new Wrapper.ArcWorld(new Wrapper.ArcWorldOptions(
            options.FatMargin,
            options.StaticCount + options.DynamicCount,
            pairCapacity));
        for (int i = 0; i < scene.StaticShapes.Length; i++)
            world.AddStatic(i, scene.StaticShapes[i], Wrapper.CollisionFilter.Default);
        for (int i = 0; i < scene.DynamicInitialShapes.Length; i++)
        {
            dynamicHandles[i] = world.Add(
                options.StaticCount + i, scene.DynamicInitialShapes[i], Wrapper.CollisionFilter.Default);
        }
        world.BuildStatic();
        long buildEnd = Stopwatch.GetTimestamp();

        ulong checksum = HashOffset;
        long pairTotal = 0;
        long collisionCount = 0;
        long simulationStart = Stopwatch.GetTimestamp();
        for (int frame = 0; frame < options.Frames; frame++)
        {
            int shapeOffset = frame * options.DynamicCount;
            for (int i = 0; i < dynamicHandles.Length; i++)
                world.UpdateTransform(
                    dynamicHandles[i], scene.DynamicFrameTransforms[shapeOffset + i]);

            world.ComputePairs(
                confirmed, candidates, Wrapper.ManifoldFields.All);
            world.TryComputeContacts(
                candidates, resolved, Wrapper.ManifoldFields.All);
            int pairCount = confirmed.Count + candidates.Count;
            pairTotal += pairCount;
            collisionCount += confirmed.Count + resolved.Count;
            checksum = Add(checksum, unchecked((uint)frame));
            checksum = Add(checksum, unchecked((uint)pairCount));
            for (int i = 0; i < confirmed.Count; i++)
            {
                Wrapper.ContactPair contact = confirmed[i];
                checksum = Add(checksum, unchecked((uint)contact.A.EntityId));
                checksum = Add(checksum, unchecked((uint)contact.B.EntityId));
                checksum = Add(checksum, 1u);
                Wrapper.Manifold manifold = contact.Manifold;
                checksum = AddManifold(checksum, manifold.Normal.X, manifold.Normal.Y,
                    manifold.Depth, manifold.Contact.X, manifold.Contact.Y);
            }
            for (int i = 0; i < candidates.Count; i++)
            {
                Wrapper.CandidatePair pair = candidates[i];
                checksum = Add(checksum, unchecked((uint)pair.A.EntityId));
                checksum = Add(checksum, unchecked((uint)pair.B.EntityId));
            }
            checksum = Add(checksum, unchecked((uint)resolved.Count));
            for (int i = 0; i < resolved.Count; i++)
            {
                Wrapper.ContactPair contact = resolved[i];
                checksum = Add(checksum, unchecked((uint)contact.A.EntityId));
                checksum = Add(checksum, unchecked((uint)contact.B.EntityId));
                Wrapper.Manifold manifold = contact.Manifold;
                checksum = AddManifold(checksum, manifold.Normal.X, manifold.Normal.Y,
                    manifold.Depth, manifold.Contact.X, manifold.Contact.Y);
            }
        }
        long simulationEnd = Stopwatch.GetTimestamp();
        long allocatedBytes = GC.GetAllocatedBytesForCurrentThread() - allocatedStart;
        EnsureThread(expectedThreadId);
        return new TrialResult(
            Stopwatch.GetElapsedTime(buildStart, buildEnd),
            Stopwatch.GetElapsedTime(simulationStart, simulationEnd),
            pairTotal, collisionCount, allocatedBytes, checksum);
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
