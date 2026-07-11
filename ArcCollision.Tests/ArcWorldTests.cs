using ArcCollision;
using Xunit;

namespace ArcCollision.Tests;

public class ArcWorldTests
{
    [Fact]
    public void SpatialHashIsInternalImplementationDetail()
    {
        Type? type = typeof(ArcWorld).Assembly.GetType("ArcCollision.SpatialHash");
        Assert.NotNull(type);
        Assert.False(type!.IsPublic);
    }

    [Fact]
    public void HandlesCarryEntityIdAndRemainCompact()
    {
        Assert.Equal(12, System.Runtime.CompilerServices.Unsafe.SizeOf<ArcHandle>());
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
