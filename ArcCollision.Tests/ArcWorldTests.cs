using Xunit;

namespace ArcCollision.Tests;

[CollectionDefinition("ArcWorld lifecycle", DisableParallelization = true)]
public sealed class ArcWorldLifecycleCollection { }

[Collection("ArcWorld lifecycle")]
public class ArcWorldTests
{
    [Fact]
    public void HandlesCarryEntityIdAndRemainCompact()
    {
        Assert.Equal(8, System.Runtime.CompilerServices.Unsafe.SizeOf<ArcHandle>());
        Assert.Equal(20, ArcHandle.IndexBits);
        Assert.Equal(12, ArcHandle.GenerationBits);
        Assert.Equal(0x000F_FFFF, ArcHandle.MaxIndex);
        Assert.Equal(0x0000_0FFF, ArcHandle.MaxGeneration);
        Assert.Equal(1_048_576, ArcWorld.MaxColliderCount);
        Assert.Equal(0x0FFF_FFFF, ArcHandle.MaxEntityId);
        Assert.Equal(15, ArcWorld.MaxWorldCount);
    }

    [Fact]
    public void HandlesFromAnotherWorldAreRejected()
    {
        using var first = new ArcWorld();
        using var second = new ArcWorld();
        ArcHandle foreign = first.Add(1, new Circle(Vec2.Zero, 1));
        ArcHandle local = second.Add(2, new Circle(Vec2.Zero, 1));

        Assert.False(second.IsValid(foreign));
        Assert.True(second.IsValid(local));
        Assert.Throws<ArgumentException>(() =>
            second.Update(foreign, new Circle(new Vec2(10, 0), 1)));
        Assert.Throws<ArgumentException>(() => second.Remove(foreign));
        Assert.True(second.IsValid(local));
    }

    [Fact]
    public void EntityIdUsesLow28BitsAndRejectsOutOfRangeValues()
    {
        using var world = new ArcWorld();
        ArcHandle maximum = world.Add(ArcHandle.MaxEntityId,
            new Circle(Vec2.Zero, 1));

        Assert.Equal(ArcHandle.MaxEntityId, maximum.EntityId);
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            world.Add(-1, new Circle(Vec2.Zero, 1)));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            world.Add(ArcHandle.MaxEntityId + 1, new Circle(Vec2.Zero, 1)));
    }

    [Fact]
    public void DisposedWorldIdIsReusedWithoutRevivingOldHandles()
    {
        CollectDeadWorlds();
        var first = new ArcWorld();
        ArcHandle stale = first.Add(1, new Circle(Vec2.Zero, 1));
        uint releasedWorldId = stale.WorldId;
        first.Dispose();
        first.Dispose();

        using var replacement = new ArcWorld();
        ArcHandle current = replacement.Add(2, new Circle(Vec2.Zero, 1));

        Assert.Equal(releasedWorldId, current.WorldId);
        Assert.NotEqual(stale.Generation, current.Generation);
        Assert.False(replacement.IsValid(stale));
        Assert.True(replacement.IsValid(current));
        Assert.Throws<ObjectDisposedException>(() =>
            first.Add(3, new Circle(Vec2.Zero, 1)));
    }

    [Fact]
    public void WorldIdPoolLimitsLiveWorldsToFourBitCapacity()
    {
        CollectDeadWorlds();
        var worlds = new ArcWorld[ArcWorld.MaxWorldCount];
        try
        {
            for (int i = 0; i < worlds.Length; i++)
                worlds[i] = new ArcWorld();

            Assert.Throws<InvalidOperationException>(() => new ArcWorld());
            var worldIds = new HashSet<uint>();
            for (int i = 0; i < worlds.Length; i++)
            {
                ArcHandle handle = worlds[i].Add(i, new Circle(Vec2.Zero, 0));
                Assert.Equal(i, handle.EntityId);
                Assert.True(worldIds.Add(handle.WorldId));
            }
            Assert.Equal(ArcWorld.MaxWorldCount, worldIds.Count);
        }
        finally
        {
            foreach (ArcWorld? world in worlds) world?.Dispose();
        }
    }

    [Fact]
    public void EveryPackedWorldNibblePreservesEntityIdBits()
    {
        CollectDeadWorlds();
        var worlds = new ArcWorld[ArcWorld.MaxWorldCount];
        int[] entityIds =
        {
            0, 1, 0x07FF_FFFF, 0x0800_0000, ArcHandle.MaxEntityId,
        };
        try
        {
            for (int i = 0; i < worlds.Length; i++)
            {
                worlds[i] = new ArcWorld();
                int entityId = entityIds[i % entityIds.Length];
                ArcHandle handle = worlds[i].Add(entityId, new Circle(Vec2.Zero, 0));
                Assert.Equal((uint)(i + 1), handle.WorldId);
                Assert.Equal(entityId, handle.EntityId);
            }
        }
        finally
        {
            foreach (ArcWorld? world in worlds) world?.Dispose();
        }
    }

    [Fact]
    public void ArbitraryReleasedWorldSlotsAreReused()
    {
        CollectDeadWorlds();
        var worlds = new ArcWorld[ArcWorld.MaxWorldCount];
        try
        {
            for (int i = 0; i < worlds.Length; i++) worlds[i] = new ArcWorld();
            uint[] released = { 3, 8, 15 };
            foreach (uint id in released)
            {
                worlds[id - 1].Dispose();
                worlds[id - 1] = null!;
            }

            var replacements = new List<ArcWorld>();
            try
            {
                for (int i = 0; i < released.Length; i++) replacements.Add(new ArcWorld());
                uint[] actual = replacements
                    .Select(world => world.Add(0, new Circle(Vec2.Zero, 0)).WorldId)
                    .OrderBy(id => id).ToArray();
                Assert.Equal(released, actual);
            }
            finally
            {
                foreach (ArcWorld world in replacements) world.Dispose();
            }
        }
        finally
        {
            foreach (ArcWorld? world in worlds) world?.Dispose();
        }
    }

    [Fact]
    public void RepeatedWorldRebuildNeverRevivesAStaleHandle()
    {
        CollectDeadWorlds();
        ArcHandle previous = default;
        uint worldId = 0;
        var generations = new HashSet<uint>();

        for (int cycle = 0; cycle < 512; cycle++)
        {
            using var world = new ArcWorld();
            ArcHandle current = world.Add(cycle, new Circle(Vec2.Zero, 1));
            if (cycle == 0) worldId = current.WorldId;
            Assert.Equal(worldId, current.WorldId);
            Assert.True(generations.Add(current.Generation));
            if (cycle != 0) Assert.False(world.IsValid(previous));
            previous = current;
        }
    }

    [Fact]
    public void TwelveBitGenerationWrapsWithoutRetiringTheSlot()
    {
        CollectDeadWorlds();
        int reusedIndex;
        ArcHandle oldest;
        using (var world = new ArcWorld())
        {
            ArcHandle current = world.Add(1, new Circle(Vec2.Zero, 1));
            oldest = current;
            reusedIndex = current.Index;
            uint oldestGeneration = oldest.Generation;

            while (current.Generation < ArcHandle.MaxGeneration)
            {
                world.Remove(current);
                current = world.Add(1, new Circle(Vec2.Zero, 1));
                Assert.Equal(reusedIndex, current.Index);
            }

            world.Remove(current);
            ArcHandle replacement = world.Add(2, new Circle(Vec2.Zero, 1));
            Assert.Equal(reusedIndex, replacement.Index);
            Assert.Equal(1u, replacement.Generation);

            while (replacement.Generation < oldestGeneration)
            {
                world.Remove(replacement);
                replacement = world.Add(2, new Circle(Vec2.Zero, 1));
                Assert.Equal(reusedIndex, replacement.Index);
            }

            Assert.Equal(oldestGeneration, replacement.Generation);
            Assert.True(world.IsValid(oldest));
        }
    }

    [Fact]
    public void PackedIndexRejectsOutOfRangeIndexAndGeneration()
    {
        _ = new ArcHandle(ArcHandle.MaxIndex, ArcHandle.MaxGeneration, 1, 0);
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new ArcHandle(ArcHandle.MaxIndex + 1, 1, 1, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new ArcHandle(0, 0, 1, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new ArcHandle(0, ArcHandle.MaxGeneration + 1u, 1, 0));
    }

    private static void CollectDeadWorlds()
    {
        GC.Collect();
        GC.WaitForPendingFinalizers();
        GC.Collect();
    }

    [Fact]
    public void BroadphasePairsCanBeFilteredBeforeNarrowphase()
    {
        var world = new ArcWorld(2f);
        ArcHandle a = world.Add(101, new Circle(Vec2.Zero, 1f));
        ArcHandle b = world.Add(202, new Circle(new Vec2(1.5f, 0), 1f));
        world.Add(303, new Circle(new Vec2(20, 0), 1f));

        var candidates = new List<CandidatePair>();
        world.ComputePairs(candidates);

        CandidatePair pair = Assert.Single(candidates);
        Assert.Equal(101, pair.A.EntityId);
        Assert.Equal(202, pair.B.EntityId);
        Assert.True(world.TryComputeContact(pair, out ContactPair contact));
        Assert.True(contact.Manifold.Colliding);
        Assert.Equal(a, contact.A);
        Assert.Equal(b, contact.B);
    }

    [Fact]
    public void DefaultFiltersPreserveExistingPairBehaviour()
    {
        using var world = new ArcWorld();
        world.Add(1, new Circle(Vec2.Zero, 1));
        world.Add(2, new Circle(new Vec2(1, 0), 1));
        var candidates = new List<CandidatePair>();

        world.ComputePairs(candidates);

        Assert.Single(candidates);
    }

    [Fact]
    public void PairFilteringRequiresMutualAcceptance()
    {
        const uint attack = 1u << 1;
        const uint hurtbox = 1u << 2;
        using var world = new ArcWorld();
        world.Add(1, new Circle(Vec2.Zero, 1),
            new CollisionFilter(attack, hurtbox));
        ArcHandle target = world.Add(2, new Circle(new Vec2(1, 0), 1),
            new CollisionFilter(hurtbox, 0));
        var candidates = new List<CandidatePair>();

        Assert.Equal(new CollisionFilter(hurtbox, 0), world.GetFilter(target));

        world.ComputePairs(candidates);
        Assert.Empty(candidates);

        world.SetFilter(target, new CollisionFilter(hurtbox, attack));
        world.ComputePairs(candidates);
        Assert.Single(candidates);
    }

    [Fact]
    public void SettingSameFilterDoesNotInvalidateCollectedPair()
    {
        using var world = new ArcWorld();
        ArcHandle first = world.Add(1, new Circle(Vec2.Zero, 1));
        world.Add(2, new Circle(new Vec2(1, 0), 1));
        var candidates = new List<CandidatePair>();
        world.ComputePairs(candidates);
        CandidatePair pair = Assert.Single(candidates);

        world.SetFilter(first, CollisionFilter.Default);

        Assert.True(world.TryComputeContact(pair, out _));
    }

    [Fact]
    public void ChangingFilterInvalidatesCollectedPairs()
    {
        using var world = new ArcWorld();
        ArcHandle first = world.Add(1, new Circle(Vec2.Zero, 1));
        world.Add(2, new Circle(new Vec2(1, 0), 1));
        var candidates = new List<CandidatePair>();
        world.ComputePairs(candidates);
        CandidatePair stale = Assert.Single(candidates);

        world.SetFilter(first, new CollisionFilter(
            CollisionCategories.Default, collidesWith: 0));

        Assert.False(world.TryComputeContact(stale, out _));
        world.ComputePairs(candidates);
        Assert.Empty(candidates);
    }

    [Fact]
    public void FilteredQueryUsesTransientShapeFilter()
    {
        const uint attack = 1u << 1;
        const uint hurtbox = 1u << 2;
        const uint scenery = 1u << 3;
        using var world = new ArcWorld();
        world.AddStatic(10, new Circle(Vec2.Zero, 1),
            new CollisionFilter(hurtbox, attack));
        world.AddStatic(20, new Circle(Vec2.Zero, 1),
            new CollisionFilter(scenery, CollisionCategories.All));
        world.BuildStatic();
        Shape query = new Circle(Vec2.Zero, 2);
        var results = new List<ArcHandle>();

        world.Query(query, new CollisionFilter(attack, hurtbox), results);

        Assert.Equal(10, Assert.Single(results).EntityId);

        world.Query(query, results);
        Assert.Equal(2, results.Count);
    }

    [Fact]
    public void MutationInvalidatesPreviouslyCollectedPairs()
    {
        var world = new ArcWorld(2f);
        var half = new Vec2(1, 1);
        world.Add(1, new Aabb(Vec2.Zero, half));
        ArcHandle moving = world.Add(2, new Aabb(new Vec2(1, 0), half));
        var candidates = new List<CandidatePair>();
        world.ComputePairs(candidates);
        CandidatePair stale = Assert.Single(candidates);

        world.Update(moving, new Aabb(new Vec2(20, 0), half));

        Assert.False(world.TryComputeContact(stale, out _));
        world.ComputePairs(candidates);
        Assert.Empty(candidates);
    }

    [Fact]
    public void RemovedHandleCannotAddressReusedSlot()
    {
        var world = new ArcWorld();
        ArcHandle removed = world.Add(7, new Circle(Vec2.Zero, 1));
        world.Remove(removed);
        ArcHandle replacement = world.Add(8, new Circle(Vec2.Zero, 1));

        Assert.False(world.IsValid(removed));
        Assert.True(world.IsValid(replacement));
        Assert.NotEqual(removed, replacement);
        Assert.Throws<ArgumentException>(() =>
            world.Update(removed, new Circle(Vec2.Zero, 2)));
    }

    [Fact]
    public void ClearInvalidatesHandlesBeforeSlotReuse()
    {
        var world = new ArcWorld();
        ArcHandle old = world.Add(11, new Circle(Vec2.Zero, 1));

        world.Clear();
        ArcHandle current = world.Add(12, new Circle(Vec2.Zero, 1));

        Assert.False(world.IsValid(old));
        Assert.True(world.IsValid(current));
        Assert.NotEqual(old, current);
    }

    [Fact]
    public void PairCollectionIncludesDynamicStaticButOmitsStaticStatic()
    {
        var world = new ArcWorld();
        world.AddStatic(1, new Aabb(Vec2.Zero, new Vec2(2, 2)));
        world.AddStatic(2, new Aabb(Vec2.Zero, new Vec2(2, 2)));
        world.Add(3, new Circle(Vec2.Zero, 1));
        world.BuildStatic();
        var candidates = new List<CandidatePair>();

        world.ComputePairs(candidates);

        Assert.Equal(2, candidates.Count);
        Assert.All(candidates, pair =>
            Assert.True(pair.A.EntityId == 3 || pair.B.EntityId == 3));
    }

    [Fact]
    public void BuildStaticIsExplicitAndIdempotent()
    {
        var world = new ArcWorld();
        world.AddStatic(42, new Aabb(Vec2.Zero, new Vec2(2, 2)));

        world.BuildStatic();
        world.BuildStatic();

        var candidates = new List<ArcHandle>();
        world.Query(new Circle(Vec2.Zero, 1), candidates);
        Assert.Equal(42, Assert.Single(candidates).EntityId);
    }

    [Fact]
    public void OptimizedPairTraversalMatchesBruteForce()
    {
        var world = new ArcWorld(8f);
        var dynamic = new List<(int Id, Aabb Box)>();
        var stationary = new List<(int Id, Aabb Box)>();
        var random = new Random(90421);
        static Aabb NextBox(Random random) => new(
            new Vec2(random.Next(-100, 101), random.Next(-100, 101)),
            new Vec2(random.Next(2, 20), random.Next(2, 20)));

        for (int id = 0; id < 80; id++)
        {
            Aabb box = NextBox(random);
            dynamic.Add((id, box));
            world.Add(id, box);
        }
        for (int id = 100; id < 140; id++)
        {
            Aabb box = NextBox(random);
            stationary.Add((id, box));
            world.AddStatic(id, box);
        }
        world.BuildStatic();

        var actualPairs = new List<CandidatePair>();
        world.ComputePairs(actualPairs);
        var actual = new HashSet<(int, int)>();
        foreach (CandidatePair pair in actualPairs)
            actual.Add(pair.A.EntityId < pair.B.EntityId
                ? (pair.A.EntityId, pair.B.EntityId)
                : (pair.B.EntityId, pair.A.EntityId));

        var expected = new HashSet<(int, int)>();
        for (int a = 0; a < dynamic.Count; a++)
        {
            for (int b = a + 1; b < dynamic.Count; b++)
                if (dynamic[a].Box.Overlaps(dynamic[b].Box))
                    expected.Add((dynamic[a].Id, dynamic[b].Id));
            for (int b = 0; b < stationary.Count; b++)
                if (dynamic[a].Box.Overlaps(stationary[b].Box))
                    expected.Add((dynamic[a].Id, stationary[b].Id));
        }

        Assert.True(expected.SetEquals(actual));
        Assert.Equal(actual.Count, actualPairs.Count);
    }

    [Fact]
    public void QueryReturnsHandlesThenComputesOnlySelectedContact()
    {
        var world = new ArcWorld();
        world.AddStatic(10, new Aabb(Vec2.Zero, new Vec2(2, 2)));
        world.Add(20, new Circle(new Vec2(10, 0), 1));
        world.BuildStatic();
        Shape probe = new Circle(new Vec2(1, 0), 1.5f);
        var candidates = new List<ArcHandle>();

        world.Query(probe, candidates);

        ArcHandle target = Assert.Single(candidates);
        Assert.Equal(10, target.EntityId);
        Assert.True(world.TryComputeContact(probe, target, out Manifold manifold));
        Assert.True(manifold.Colliding);
    }
}
