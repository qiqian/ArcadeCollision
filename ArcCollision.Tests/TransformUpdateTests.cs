using System;
using System.Collections.Generic;
using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Validates transform-based world updates against both managed and native
/// backends. A collider keeps its immutable
/// base shape and is re-placed by a rigid transform (absolute world position,
/// rotation, uniform scale). Observed through the broadphase
/// <c>Query</c> (bounds overlap), which is enough to pin placement/scale/rotation.
/// </summary>
public class TransformUpdateTests
{
    private static bool BoundsHit(ArcWorld world, ArcHandle handle, Vec2 point)
    {
        var results = new List<ArcHandle>();
        world.Query(new Aabb(point, new Vec2(0.01f, 0.01f)), results);
        return results.Contains(handle);
    }

    [Fact]
    public void TransformRejectsNonFiniteScale()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new Transform(Vec2.Zero, new Angle32(0), float.PositiveInfinity));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new Transform(Vec2.Zero, new Angle32(0), float.NaN));
    }

    [Fact]
    public void TranslationOnly_PlacesBaseAtAbsolutePosition()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.Add(1, new Circle(new Vec2(0, 0), 2f));

        world.UpdateTransform(handle, new Transform(new Vec2(100f, 50f)));

        Assert.True(BoundsHit(world, handle, new Vec2(100f, 50f)));
        Assert.False(BoundsHit(world, handle, new Vec2(0f, 0f)));
    }

    [Fact]
    public void AbsoluteAndDeltaUpdatesReachTheSameTransform()
    {
        using var world = new ArcWorld();
        var initial = new Vec2(4f, 7f);
        var baseCircle = new Circle(initial, 2f);
        ArcHandle viaAbsolute = world.Add(1, baseCircle);
        ArcHandle viaDelta = world.Add(2, baseCircle);

        var target = new Vec2(30f, -12f);
        world.UpdateTransform(viaAbsolute, new Transform(target));
        world.UpdateTransformDelta(viaDelta, new Transform(target - initial));

        foreach (Vec2 probe in ProbeGrid(target, 5f))
            Assert.Equal(
                BoundsHit(world, viaAbsolute, probe),
                BoundsHit(world, viaDelta, probe));
    }

    [Fact]
    public void Scale_GrowsShapeUniformly()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.Add(1, new Circle(new Vec2(0f, 0f), 2f));

        // r=2: a probe 5 units out misses. Scale x3 -> r=6: now inside the bounds.
        Assert.False(BoundsHit(world, handle, new Vec2(5f, 0f)));
        world.UpdateTransform(handle, new Transform(new Vec2(0f, 0f), new Angle32(0), 3f));
        Assert.True(BoundsHit(world, handle, new Vec2(5f, 0f)));
    }

    [Fact]
    public void Rotation_IgnoredForAabb_StaysAxisAligned()
    {
        using var world = new ArcWorld();
        // Wide-short box: 10 in X, 1 in Y.
        ArcHandle handle = world.Add(1, new Aabb(new Vec2(0f, 0f), new Vec2(10f, 1f)));

        world.UpdateTransform(handle, new Transform(
            new Vec2(0f, 0f), Angle32.FromRadians(MathF.PI / 2f), 1f));

        Assert.True(BoundsHit(world, handle, new Vec2(8f, 0f)));   // still wide in X
        Assert.False(BoundsHit(world, handle, new Vec2(0f, 8f)));  // never tall in Y
    }

    [Fact]
    public void Rotation_OrientsObb()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.Add(1, new Obb(new Vec2(0f, 0f), new Vec2(10f, 1f), 0f));

        Assert.False(BoundsHit(world, handle, new Vec2(0f, 8f)));  // wide in X initially
        world.UpdateTransform(handle, new Transform(
            new Vec2(0f, 0f), Angle32.FromRadians(MathF.PI / 2f), 1f));
        Assert.True(BoundsHit(world, handle, new Vec2(0f, 8f)));   // now tall in Y
    }

    [Fact]
    public void Capsule_TranslationOnly_MovesEndpoints()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.Add(1,
            new Capsule(new Vec2(-5f, 0f), new Vec2(5f, 0f), 1f));

        world.UpdateTransform(handle, new Transform(new Vec2(20f, 20f)));

        Assert.True(BoundsHit(world, handle, new Vec2(25f, 20f)));  // endpoint moved
        Assert.False(BoundsHit(world, handle, new Vec2(5f, 0f)));   // left the origin
    }

    [Fact]
    public void Polygon_ScalesAroundItsLocalOrigin()
    {
        using var world = new ArcWorld();
        var polygon = new Polygon(
            new Vec2(-2f, -1f), new Vec2(2f, -1f), new Vec2(0f, 2f));
        ArcHandle handle = world.Add(1, polygon);

        Assert.False(BoundsHit(world, handle, new Vec2(5f, 0f)));
        world.UpdateTransform(handle, new Transform(
            new Vec2(10f, 0f), new Angle32(0), 3f));

        Assert.True(BoundsHit(world, handle, new Vec2(15f, 0f)));
        Assert.False(BoundsHit(world, handle, new Vec2(3f, 0f)));
    }

    [Fact]
    public void Delta_ComposesPositionRotationAndScale()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.Add(1,
            new Obb(Vec2.Zero, new Vec2(4f, 1f), 0f));
        Angle32 quarterTurn = Angle32.FromRadians(MathF.PI / 2f);

        world.UpdateTransform(handle, new Transform(
            new Vec2(10f, 20f), quarterTurn, 2f));
        world.UpdateTransformDelta(handle, new Transform(
            new Vec2(5f, -2f), quarterTurn, .5f));

        Assert.True(BoundsHit(world, handle, new Vec2(18f, 18f)));
        Assert.False(BoundsHit(world, handle, new Vec2(15f, 22f)));
    }

    [Fact]
    public void RemovedSlotReuseUsesReplacementLocalGeometry()
    {
        using var world = new ArcWorld();
        ArcHandle removed = world.Add(1, new Circle(Vec2.Zero, 8f));
        world.Remove(removed);
        ArcHandle replacement = world.Add(2,
            new Aabb(Vec2.Zero, new Vec2(1f, 3f)));

        world.UpdateTransform(replacement, new Transform(
            new Vec2(20f, 10f), new Angle32(0), 2f));

        Assert.True(BoundsHit(world, replacement, new Vec2(21f, 15f)));
        Assert.False(BoundsHit(world, replacement, new Vec2(25f, 10f)));
    }

    private static IEnumerable<Vec2> ProbeGrid(Vec2 center, float radius)
    {
        for (float dx = -radius; dx <= radius; dx += 1f)
            for (float dy = -radius; dy <= radius; dy += 1f)
                yield return new Vec2(center.X + dx, center.Y + dy);
    }
}
