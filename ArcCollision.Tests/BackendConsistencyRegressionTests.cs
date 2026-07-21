using Xunit;

namespace ArcCollision.Tests;

[Collection("ArcWorld lifecycle")]
public class BackendConsistencyRegressionTests
{
    [Fact]
    public void FailedAddDoesNotConsumeAHandleSlot()
    {
        using var world = new ArcWorld();
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            world.Add(1, new Circle(new Vec2(float.PositiveInfinity, 0), 1), CollisionFilter.Default));

        ArcHandle handle = world.Add(2, new Circle(Vec2.Zero, 1), CollisionFilter.Default);
        Assert.Equal(0, handle.Index);
    }

    [Fact]
    public void StandaloneBroadphasesRejectCallsAfterDispose()
    {
        var tree = new DynamicAabbTree();
        tree.Dispose();
        Assert.Throws<ObjectDisposedException>(() => tree.Clear());
        Assert.Throws<ObjectDisposedException>(() => _ = tree.Count);

        var bvh = new StaticBvh();
        bvh.Dispose();
        Assert.Throws<ObjectDisposedException>(() => bvh.Clear());
        Assert.Throws<ObjectDisposedException>(() =>
            bvh.Build(new Dictionary<int, BpBounds>()));
    }
}
