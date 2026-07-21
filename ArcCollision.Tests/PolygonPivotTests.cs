using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Locks the polygon pivot rule and <c>Polygon.Centered()</c>. A polygon's local
/// origin is where its vertices were authored relative to (0, 0), unlike the
/// primitives which pivot about their own centre -- that asymmetry is deliberate
/// (it is the only way to express an off-centre pivot), so it is pinned here
/// rather than left to be rediscovered.
/// </summary>
[Collection("ArcWorld lifecycle")]
public class PolygonPivotTests
{
    // Authored far from its own origin: bounds centre is (102, 101).
    private static Polygon FarFromOrigin() => new(
        new Vec2(100f, 100f), new Vec2(104f, 100f),
        new Vec2(104f, 102f), new Vec2(100f, 102f));

    private static Vec2 BoundsCenter(ArcWorld world, ArcHandle handle) =>
        world.GetShape(handle).Bounds.Center;

    [Fact]
    public void CenteredMovesBoundsCentreToTheOrigin()
    {
        Polygon far = FarFromOrigin();
        Assert.Equal(102f, far.Bounds.Center.X, 3);
        Assert.Equal(101f, far.Bounds.Center.Y, 3);

        Polygon centered = far.Centered();
        Assert.Equal(0f, centered.Bounds.Center.X, 3);
        Assert.Equal(0f, centered.Bounds.Center.Y, 3);

        // Same geometry, just shifted: extents are untouched.
        Assert.Equal(far.Count, centered.Count);
        Assert.Equal(far.Bounds.HalfExtents.X, centered.Bounds.HalfExtents.X, 3);
        Assert.Equal(far.Bounds.HalfExtents.Y, centered.Bounds.HalfExtents.Y, 3);
    }

    [Fact]
    public void CenteringAnAlreadyCentredPolygonIsANoOp()
    {
        var already = new Polygon(
            new Vec2(-2f, -1f), new Vec2(2f, -1f),
            new Vec2(2f, 1f), new Vec2(-2f, 1f));

        Polygon centered = already.Centered();
        Assert.Equal(0f, centered.Bounds.Center.X, 3);
        Assert.Equal(0f, centered.Bounds.Center.Y, 3);
        Assert.Equal(already.Count, centered.Count);
    }

    /// <summary>
    /// The authored origin is the pivot: rotating a polygon whose geometry sits
    /// far from (0, 0) swings it around the origin rather than spinning in place.
    /// </summary>
    [Fact]
    public void AuthoredOriginIsThePivot()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.Add(1, new Shape(FarFromOrigin()), CollisionFilter.Default);
        Assert.Equal(102f, BoundsCenter(world, handle).X, 2);
        Assert.Equal(101f, BoundsCenter(world, handle).Y, 2);

        // Quarter turn about the authored origin maps (102, 101) to (-101, 102).
        world.UpdateTransform(handle, new Transform(
            Vec2.Zero, Angle32.FromRadians(MathF.PI / 2f), 1f));
        Assert.Equal(-101f, BoundsCenter(world, handle).X, 1);
        Assert.Equal(102f, BoundsCenter(world, handle).Y, 1);
    }

    /// <summary>
    /// After Centered() the polygon spins in place, matching how the primitives
    /// behave -- the whole point of the helper.
    /// </summary>
    [Fact]
    public void CenteredPolygonSpinsInPlace()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.Add(1, new Shape(FarFromOrigin().Centered()), CollisionFilter.Default);
        Assert.Equal(0f, BoundsCenter(world, handle).X, 2);
        Assert.Equal(0f, BoundsCenter(world, handle).Y, 2);

        world.UpdateTransform(handle, new Transform(
            new Vec2(30f, 40f), Angle32.FromRadians(MathF.PI / 2f), 1f));
        Assert.Equal(30f, BoundsCenter(world, handle).X, 1);
        Assert.Equal(40f, BoundsCenter(world, handle).Y, 1);
    }

    /// <summary>
    /// The primitives all pivot about their own centre, so a rotation about the
    /// spot they already occupy leaves them there. This is the behaviour
    /// Centered() brings polygons in line with.
    /// </summary>
    [Fact]
    public void PrimitivesPivotAboutTheirOwnCentre()
    {
        using var world = new ArcWorld();
        ArcHandle circle = world.Add(1, new Circle(new Vec2(100f, 50f), 2f), CollisionFilter.Default);
        ArcHandle capsule = world.Add(
            2, new Capsule(new Vec2(-5f, 0f), new Vec2(5f, 0f), 1f),
            CollisionFilter.Default);

        // A circle's centre is its local origin: re-placing it is absolute.
        world.UpdateTransform(circle, new Transform(Vec2.Zero));
        Assert.Equal(0f, BoundsCenter(world, circle).X, 3);
        Assert.Equal(0f, BoundsCenter(world, circle).Y, 3);

        // A capsule pivots about the midpoint of its endpoints.
        world.UpdateTransform(capsule, new Transform(
            Vec2.Zero, Angle32.FromRadians(MathF.PI / 2f), 1f));
        world.GetShape(capsule).TryGetCapsule(out Capsule spun);
        Assert.Equal(0f, spun.A.X, 1);
        Assert.Equal(-5f, spun.A.Y, 1);
        Assert.Equal(0f, spun.B.X, 1);
        Assert.Equal(5f, spun.B.Y, 1);
    }

    /// <summary>
    /// An OBB's authored angle is a base angle the transform rotation adds to,
    /// so rotation 0 restores it instead of straightening the box.
    /// </summary>
    [Fact]
    public void AuthoredAngleIsABaseTheTransformComposesWith()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.Add(
            1, new Obb(new Vec2(10f, 0f), new Vec2(2f, 1f), 0.5f),
            CollisionFilter.Default);

        world.UpdateTransform(handle, new Transform(new Vec2(10f, 0f)));
        world.GetShape(handle).TryGetObb(out Obb kept);
        Assert.Equal(0.5f, kept.Angle.Radians, 3);

        world.UpdateTransform(handle, new Transform(
            new Vec2(10f, 0f), Angle32.FromRadians(0.25f), 1f));
        world.GetShape(handle).TryGetObb(out Obb composed);
        Assert.Equal(0.75f, composed.Angle.Radians, 3);
    }
}
