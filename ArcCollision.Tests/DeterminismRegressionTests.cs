using System.Runtime.CompilerServices;
using Xunit;

namespace ArcCollision.Tests;

[Collection("ArcWorld lifecycle")]
public class DeterminismRegressionTests
{
    [Fact]
    public void ObbAuthoritativeAngleSurvivesMovesAndShapeRoundTrips()
    {
        var angle = new Angle32(0xFEDCBA98u);
        var box = new Obb(new Vec2(1, 2), new Vec2(3, 4), angle);
        Obb moved = box.Moved(new Vec2(100, -50));
        Shape shape = moved;

        Assert.Equal(angle.Raw, box.Angle.Raw);
        Assert.Equal(angle.Raw, moved.Angle.Raw);
        Assert.True(shape.TryGetObb(out Obb roundTripped));
        Assert.Equal(angle.Raw, roundTripped.Angle.Raw);
        Assert.Equal(20, Unsafe.SizeOf<Obb>());
        Assert.Equal(32, Unsafe.SizeOf<Shape>());
    }

    [Fact]
    public void PublicValueHashesHaveLockedProcessIndependentValues()
    {
        Assert.Equal(1046910925, new Vec2(1.5f, -2.25f).GetHashCode());
        Assert.Equal(new Vec2(0f, float.NaN).GetHashCode(),
            new Vec2(-0f, BitConverter.UInt32BitsToSingle(0x7FA12345u)).GetHashCode());
        // World id 5 (the old 9 no longer fits the 3-bit field). The hash is
        // still Combine(index, generation, worldId) over an unchanged algorithm,
        // so only the input moved -- the locked value is recomputed, not relaxed.
        var handle = new ArcHandle(7, 11, 5, 123456);
        Assert.Equal(-1827787050, handle.GetHashCode());

        var sameIdentityDifferentMetadata = new ArcHandle(7, 11, 5, 999999);
        Assert.Equal(handle, sameIdentityDifferentMetadata);
        Assert.Equal(handle.GetHashCode(), sameIdentityDifferentMetadata.GetHashCode());
    }

    [Fact]
    public void StaticBvhTraversalIsIndependentOfDictionaryInsertionOrder()
    {
        var entries = new (int Id, BpBounds Bounds)[]
        {
            (7, new BpBounds(-10, -10, 10, 10)),
            (3, new BpBounds(-10, -10, 10, 10)),
            (11, new BpBounds(20, -5, 30, 5)),
            (2, new BpBounds(-30, -5, -20, 5)),
            (19, new BpBounds(0, 20, 5, 30)),
            (5, new BpBounds(0, -30, 5, -20)),
        };
        var forward = new Dictionary<int, BpBounds>();
        var reverse = new Dictionary<int, BpBounds>();
        foreach ((int id, BpBounds bounds) in entries) forward.Add(id, bounds);
        for (int i = entries.Length - 1; i >= 0; i--)
            reverse.Add(entries[i].Id, entries[i].Bounds);

        int[] first = QueryBvh(forward);
        int[] second = QueryBvh(reverse);

        Assert.Equal(first, second);
    }

    [Fact]
    public void ArcWorldOutputsHaveStableEntityOrderingAcrossInsertionOrders()
    {
        int[] forward = { 50, 10, 40, 20, 30 };
        int[] reverse = forward.Reverse().ToArray();

        (int[] Query, (int A, int B)[] Pairs) first = RunWorld(forward);
        (int[] Query, (int A, int B)[] Pairs) second = RunWorld(reverse);

        Assert.Equal(new[] { 10, 20, 30, 40, 50 }, first.Query);
        Assert.Equal(first.Query, second.Query);
        Assert.Equal(first.Pairs, second.Pairs);
        Assert.All(first.Pairs, pair => Assert.True(pair.A <= pair.B));
    }

    private static (int[] Query, (int A, int B)[] Pairs) RunWorld(int[] order)
    {
        using var world = new ArcWorld(1);
        foreach (int entityId in order)
        {
            Shape shape = new Aabb(Vec2.Zero, new Vec2(10, 10));
            if (entityId is 20 or 40) world.AddStatic(entityId, shape);
            else world.Add(entityId, shape);
        }
        world.BuildStatic();

        var query = new List<ArcHandle>();
        world.Query(new Aabb(Vec2.Zero, new Vec2(20, 20)), query);
        var pairs = new List<CandidatePair>();
        world.ComputePairs(pairs);
        return (query.Select(handle => handle.EntityId).ToArray(),
            pairs.Select(pair => (pair.A.EntityId, pair.B.EntityId)).ToArray());
    }

    private static int[] QueryBvh(Dictionary<int, BpBounds> source)
    {
        var bvh = new StaticBvh();
        bvh.Build(source);
        var results = new List<int>();
        bvh.Query(new BpBounds(-100, -100, 100, 100), results);
        return results.ToArray();
    }

}
