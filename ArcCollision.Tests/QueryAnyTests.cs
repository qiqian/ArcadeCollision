using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Pins <c>ArcWorld.QueryAny</c> to <c>Query</c>. QueryAny exists only to answer
/// the same question faster (it stops at the first match instead of collecting
/// and sorting), so the one invariant that matters is that it never disagrees
/// with the equivalently filtered query about emptiness -- across shape kinds,
/// filters, disabled colliders, and both broadphase trees.
/// </summary>
[Collection("ArcWorld lifecycle")]
public class QueryAnyTests
{
    // Accepts every category, so it is the closest stand-in for "no filter".
    private static readonly CollisionFilter Any = CollisionFilter.Default;

    private static Shape BoxAt(float x, float y, float half = 1f) =>
        new Aabb(new Vec2(x, y), new Vec2(half, half));

    private static void AssertAgreesWithQuery(
        ArcWorld world, in Shape query, in CollisionFilter filter)
    {
        var results = new List<ArcHandle>();
        world.Query(query, filter, results);
        Assert.Equal(results.Count > 0, world.QueryAny(query, filter));
    }

    [Fact]
    public void MatchesQueryEmptinessForHitAndMiss()
    {
        using var world = new ArcWorld();
        world.Add(1, BoxAt(0f, 0f), CollisionFilter.Default);
        world.AddStatic(2, BoxAt(100f, 0f), CollisionFilter.Default);
        world.BuildStatic();

        Assert.True(world.QueryAny(BoxAt(0f, 0f), Any));     // dynamic tree
        Assert.True(world.QueryAny(BoxAt(100f, 0f), Any));   // static tree
        Assert.False(world.QueryAny(BoxAt(-500f, -500f), Any));

        AssertAgreesWithQuery(world, BoxAt(0f, 0f), Any);
        AssertAgreesWithQuery(world, BoxAt(100f, 0f), Any);
        AssertAgreesWithQuery(world, BoxAt(-500f, -500f), Any);
    }

    [Fact]
    public void EmptyWorldReportsNothing()
    {
        using var world = new ArcWorld();
        Assert.False(world.QueryAny(BoxAt(0f, 0f), Any));
        world.BuildStatic();
        Assert.False(world.QueryAny(BoxAt(0f, 0f), Any));
    }

    /// <summary>Touching only at the boundary counts, exactly as Query does.</summary>
    [Fact]
    public void BoundaryTouchAgreesWithQuery()
    {
        using var world = new ArcWorld();
        world.Add(1, BoxAt(0f, 0f), CollisionFilter.Default);

        AssertAgreesWithQuery(world, BoxAt(2f, 0f), Any);     // edges just meet
        AssertAgreesWithQuery(world, BoxAt(2.5f, 0f), Any);   // clearly apart
    }

    [Fact]
    public void RespectsCollisionFilter()
    {
        var wall = new CollisionFilter(categories: 0b0001, collidesWith: 0b0001);
        var probe = new CollisionFilter(categories: 0b0010, collidesWith: 0b0010);
        var matching = new CollisionFilter(categories: 0b0001, collidesWith: 0b0001);

        using var world = new ArcWorld();
        world.Add(1, BoxAt(0f, 0f), wall);

        Assert.False(world.QueryAny(BoxAt(0f, 0f), probe));   // mutually rejected
        Assert.True(world.QueryAny(BoxAt(0f, 0f), matching));

        AssertAgreesWithQuery(world, BoxAt(0f, 0f), probe);
        AssertAgreesWithQuery(world, BoxAt(0f, 0f), matching);
    }

    /// <summary>
    /// A filter that rejects everything must short-circuit to false even where
    /// the bounds do overlap -- the early-out must not skip the filter test.
    /// </summary>
    [Fact]
    public void RejectingFilterFindsNothingOverOverlappingBounds()
    {
        var none = new CollisionFilter(categories: 0u, collidesWith: 0u);

        using var world = new ArcWorld();
        world.Add(1, BoxAt(0f, 0f), CollisionFilter.Default);
        world.AddStatic(2, BoxAt(0f, 0f), CollisionFilter.Default);
        world.BuildStatic();

        Assert.True(world.QueryAny(BoxAt(0f, 0f), Any));
        Assert.False(world.QueryAny(BoxAt(0f, 0f), none));
        AssertAgreesWithQuery(world, BoxAt(0f, 0f), none);
    }

    /// <summary>
    /// Disabled colliders are filtered after the broadphase, and a disabled
    /// static stays in the BVH, so the early-out path must apply that test too.
    /// </summary>
    [Fact]
    public void IgnoresDisabledColliders()
    {
        using var world = new ArcWorld();
        ArcHandle dynamicHandle = world.Add(1, BoxAt(0f, 0f), CollisionFilter.Default);
        ArcHandle staticHandle = world.AddStatic(2, BoxAt(100f, 0f), CollisionFilter.Default);
        world.BuildStatic();

        world.SetEnabled(dynamicHandle, false);
        world.SetEnabled(staticHandle, false);
        Assert.False(world.QueryAny(BoxAt(0f, 0f), Any));
        Assert.False(world.QueryAny(BoxAt(100f, 0f), Any));
        AssertAgreesWithQuery(world, BoxAt(100f, 0f), Any);

        world.SetEnabled(dynamicHandle, true);
        world.SetEnabled(staticHandle, true);
        Assert.True(world.QueryAny(BoxAt(0f, 0f), Any));
        Assert.True(world.QueryAny(BoxAt(100f, 0f), Any));
    }

    [Fact]
    public void RemovedColliderIsNoLongerReported()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.Add(1, BoxAt(0f, 0f), CollisionFilter.Default);
        Assert.True(world.QueryAny(BoxAt(0f, 0f), Any));

        world.Remove(handle);
        Assert.False(world.QueryAny(BoxAt(0f, 0f), Any));
    }

    [Fact]
    public void FollowsColliderAfterTransformUpdate()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.Add(1, BoxAt(0f, 0f), CollisionFilter.Default);

        world.UpdateTransform(handle, new Transform(new Vec2(40f, 0f)));
        Assert.False(world.QueryAny(BoxAt(0f, 0f), Any));
        Assert.True(world.QueryAny(BoxAt(40f, 0f), Any));
    }

    /// <summary>Every query shape kind resolves through the same bounds path.</summary>
    [Fact]
    public void SupportsEveryQueryShapeKind()
    {
        using var world = new ArcWorld();
        world.Add(1, BoxAt(0f, 0f, 2f), CollisionFilter.Default);

        Shape[] hits =
        {
            new Circle(new Vec2(0f, 0f), 1f),
            new Aabb(new Vec2(0f, 0f), new Vec2(1f, 1f)),
            new Capsule(new Vec2(-1f, 0f), new Vec2(1f, 0f), 0.5f),
            new Obb(new Vec2(0f, 0f), new Vec2(1f, 1f), 0.3f),
        };
        foreach (Shape shape in hits)
        {
            Assert.True(world.QueryAny(shape, Any));
            AssertAgreesWithQuery(world, shape, Any);
        }

        Shape[] misses =
        {
            new Circle(new Vec2(500f, 500f), 1f),
            new Aabb(new Vec2(500f, 500f), new Vec2(1f, 1f)),
            new Capsule(new Vec2(499f, 500f), new Vec2(501f, 500f), 0.5f),
            new Obb(new Vec2(500f, 500f), new Vec2(1f, 1f), 0.3f),
        };
        foreach (Shape shape in misses)
        {
            Assert.False(world.QueryAny(shape, Any));
            AssertAgreesWithQuery(world, shape, Any);
        }
    }

    /// <summary>
    /// Sweeps a populated world so the early-out path is exercised against many
    /// tree shapes and both hit and miss regions, never disagreeing with Query.
    /// </summary>
    [Fact]
    public void AgreesWithQueryAcrossAPopulatedWorld()
    {
        using var world = new ArcWorld();
        int id = 0;
        for (int x = 0; x < 8; x++)
            for (int y = 0; y < 8; y++)
                world.AddStatic(id++, BoxAt(x * 10f, y * 10f), CollisionFilter.Default);
        for (int i = 0; i < 16; i++)
            world.Add(1000 + i, new Circle(new Vec2(i * 7f, i * 5f), 3f), CollisionFilter.Default);
        world.BuildStatic();

        for (int x = -20; x <= 100; x += 3)
            for (int y = -20; y <= 100; y += 3)
                AssertAgreesWithQuery(world, BoxAt(x, y, 1.5f), Any);
    }
}
