using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Fixtures adapted from box2d/test. These exercise the equivalent ArcCollision
/// primitives; Box2D features without an ArcCollision.Ref counterpart are not
/// reproduced here.
/// </summary>
public class Box2DPortedDistanceAndShapeTests
{
    private const float FixedEpsilon = 1f / 256f;

    // box2d/test/test_distance.c: SegmentDistanceTest
    [Fact]
    public void SegmentDistance_Box2DFixture_ReturnsExpectedClosestPoints()
    {
        float distanceSquared = Distance.ClosestPointsSegmentSegment(
            new Vec2(-1, -1), new Vec2(-1, 1),
            new Vec2(2, 0), new Vec2(1, 0),
            out Vec2 first, out Vec2 second);

        Assert.Equal(4f, distanceSquared, FixedEpsilon);
        AssertVec(new Vec2(-1, 0), first);
        AssertVec(new Vec2(1, 0), second);
    }

    // box2d/test/test_shape.c: ShapeAABBTest and PointInShapeTest
    [Fact]
    public void ShapeBoundsAndPointPredicates_MatchBox2DFixtures()
    {
        var circle = new Circle(new Vec2(1, 0), 1);
        Aabb circleBounds = circle.Bounds;
        AssertVec(new Vec2(0, -1), circleBounds.Min);
        AssertVec(new Vec2(2, 1), circleBounds.Max);
        Assert.True(Collide.PointInCircle(new Vec2(0.5f, 0.5f), circle));
        Assert.False(Collide.PointInCircle(new Vec2(4, -4), circle));

        var box = new Aabb(Vec2.Zero, new Vec2(1, 1));
        AssertVec(new Vec2(-1, -1), box.Min);
        AssertVec(new Vec2(1, 1), box.Max);
        Assert.True(Collide.PointInAabb(new Vec2(0.5f, 0.5f), box));
        Assert.False(Collide.PointInAabb(new Vec2(4, -4), box));
    }

    // box2d/test/test_collision.c: AABBRayCastTest. Box2D treats a ray
    // starting inside as no hit, while ArcCollision intentionally reports an
    // initial-overlap hit, so only common external-ray semantics are ported.
    [Theory]
    [InlineData(-3, 0, 6, 0, true, 1f / 3f, -1, 0, -1, 0)]
    [InlineData(3, 0, -6, 0, true, 1f / 3f, 1, 0, 1, 0)]
    [InlineData(0, -3, 0, 6, true, 1f / 3f, 0, -1, 0, -1)]
    [InlineData(0, 3, 0, -6, true, 1f / 3f, 0, 1, 0, 1)]
    [InlineData(-3, 2, 6, 0, false, 0, 0, 0, 0, 0)]
    [InlineData(2, -3, 0, 6, false, 0, 0, 0, 0, 0)]
    [InlineData(-2, 1.5f, 4, 0, false, 0, 0, 0, 0, 0)]
    [InlineData(-2, 1, 4, 0, true, 0.25f, -1, 0, -1, 1)]
    [InlineData(-3, 0, 0.5f, 0, false, 0, 0, 0, 0, 0)]
    [InlineData(-2, 0, 1, 0, true, 1, -1, 0, -1, 0)]
    public void RayVsAabb_Box2DExternalFixtures(
        float ox, float oy, float dx, float dy, bool expectedHit,
        float expectedTime, float nx, float ny, float px, float py)
    {
        SweepHit hit = Sweep.RayVsAabb(
            new Vec2(ox, oy), new Vec2(dx, dy),
            new Aabb(Vec2.Zero, new Vec2(1, 1)));

        Assert.Equal(expectedHit, hit.Hit);
        if (!expectedHit) return;

        Assert.Equal(expectedTime, hit.Time, 1f / 65536f);
        AssertVec(new Vec2(nx, ny), hit.Normal);
        AssertVec(new Vec2(px, py), hit.Point);
    }

    // box2d/test/test_collision.c: diagonal corner case. Either incident face
    // is valid in Box2D; ArcCollision must likewise return one axial normal.
    [Fact]
    public void RayVsAabb_Box2DDiagonalCornerFixture_ReturnsIncidentFace()
    {
        SweepHit hit = Sweep.RayVsAabb(
            new Vec2(-2, -2), new Vec2(4, 4),
            new Aabb(Vec2.Zero, new Vec2(1, 1)));

        Assert.True(hit.Hit);
        Assert.Equal(0.25f, hit.Time, 1f / 65536f);
        AssertVec(new Vec2(-1, -1), hit.Point);
        Assert.True(
            (hit.Normal.X == -1f && hit.Normal.Y == 0f)
            || (hit.Normal.X == 0f && hit.Normal.Y == -1f));
    }

    // box2d/test/test_collision.c: offset AABB case
    [Fact]
    public void RayVsAabb_Box2DOffsetFixture_ReportsWorldContactPoint()
    {
        SweepHit hit = Sweep.RayVsAabb(
            new Vec2(0, 4), new Vec2(6, 0),
            Aabb.FromMinMax(new Vec2(2, 3), new Vec2(4, 5)));

        Assert.True(hit.Hit);
        Assert.Equal(1f / 3f, hit.Time, 1f / 65536f);
        AssertVec(new Vec2(-1, 0), hit.Normal);
        AssertVec(new Vec2(2, 4), hit.Point);
    }

    private static void AssertVec(Vec2 expected, Vec2 actual)
    {
        Assert.Equal(expected.X, actual.X, FixedEpsilon);
        Assert.Equal(expected.Y, actual.Y, FixedEpsilon);
    }
}

public class Box2DPortedDynamicTreeTests
{
    // box2d/test/test_dynamic_tree.c: TreeCreateDestroy and
    // TreeMultipleProxiesTest, adapted to ArcCollision's id-only proxies.
    [Fact]
    public void DynamicTree_CreateDestroyAndClear_TrackProxyCount()
    {
        var tree = new DynamicAabbTree();
        int first = tree.CreateProxy(42, Bounds(-5, -1, -3, 1));
        int second = tree.CreateProxy(43, Bounds(-1, -1, 1, 1));
        int third = tree.CreateProxy(44, Bounds(3, -1, 5, 1));

        Assert.Equal(3, tree.Count);
        Assert.NotEqual(first, second);
        Assert.NotEqual(second, third);

        tree.DestroyProxy(second);
        Assert.Equal(2, tree.Count);

        tree.Clear();
        Assert.Equal(0, tree.Count);
        //Assert.Equal(-1, tree.RootIndex);
    }

    // box2d/test/test_dynamic_tree.c: TreeQueryTest
    [Fact]
    public void DynamicTree_Query_Box2DFixture_ReturnsOnlyMiddleProxy()
    {
        var tree = new DynamicAabbTree();
        tree.CreateProxy(42, Bounds(-5, -1, -3, 1));
        tree.CreateProxy(43, Bounds(-1, -1, 1, 1));
        tree.CreateProxy(44, Bounds(3, -1, 5, 1));

        var found = new List<int>();
        tree.Query(Bounds(-2, -2, 2, 2), found);

        Assert.Equal(new[] { 43 }, found);
    }

    // box2d/test/test_dynamic_tree.c: TreeMoveAndEnlargeTest. ArcCollision
    // combines the tight-bounds containment check and fat-bounds replacement
    // in MoveProxy rather than exposing separate Move/Enlarge operations.
    [Fact]
    public void DynamicTree_MoveProxy_UpdatesQueriesAndSkipsContainedMoves()
    {
        var tree = new DynamicAabbTree();
        int proxy = tree.CreateProxy(100, Bounds(0, 0, 1, 1));

        Assert.True(tree.MoveProxy(proxy,
            Bounds(10, 10, 11, 11), Bounds(9.5f, 9.5f, 11.5f, 11.5f)));

        var oldArea = new List<int>();
        tree.Query(Bounds(-1, -1, 2, 2), oldArea);
        Assert.Empty(oldArea);

        var newArea = new List<int>();
        tree.Query(Bounds(10, 10, 11, 11), newArea);
        Assert.Equal(new[] { 100 }, newArea);

        Assert.False(tree.MoveProxy(proxy,
            Bounds(10.25f, 10.25f, 10.75f, 10.75f),
            Bounds(10, 10, 11, 11)));
    }

    private static BpBounds Bounds(float minX, float minY, float maxX, float maxY) =>
        new(Aabb.FromMinMax(new Vec2(minX, minY), new Vec2(maxX, maxY)));
}
