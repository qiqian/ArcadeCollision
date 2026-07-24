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
[Collection("ArcWorld lifecycle")]
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
        ArcHandle handle = world.Add(1, new Circle(new Vec2(0, 0), 2f), CollisionFilter.Default);

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
        ArcHandle viaAbsolute = world.Add(1, baseCircle, CollisionFilter.Default);
        ArcHandle viaDelta = world.Add(2, baseCircle, CollisionFilter.Default);

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
        ArcHandle handle = world.Add(1, new Circle(new Vec2(0f, 0f), 2f), CollisionFilter.Default);

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
        ArcHandle handle = world.Add(1, new Aabb(new Vec2(0f, 0f), new Vec2(10f, 1f)), CollisionFilter.Default);

        world.UpdateTransform(handle, new Transform(
            new Vec2(0f, 0f), Angle32.FromRadians(MathF.PI / 2f), 1f));

        Assert.True(BoundsHit(world, handle, new Vec2(8f, 0f)));   // still wide in X
        Assert.False(BoundsHit(world, handle, new Vec2(0f, 8f)));  // never tall in Y
    }

    [Fact]
    public void Rotation_OrientsObb()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.Add(1, new Obb(new Vec2(0f, 0f), new Vec2(10f, 1f), 0f), CollisionFilter.Default);

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
            new Capsule(new Vec2(-5f, 0f), new Vec2(5f, 0f), 1f),
            CollisionFilter.Default);

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
        ArcHandle handle = world.Add(1, polygon, CollisionFilter.Default);

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
            new Obb(Vec2.Zero, new Vec2(4f, 1f), 0f), CollisionFilter.Default);
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
        ArcHandle removed = world.Add(1, new Circle(Vec2.Zero, 8f), CollisionFilter.Default);
        world.Remove(removed);
        ArcHandle replacement = world.Add(2,
            new Aabb(Vec2.Zero, new Vec2(1f, 3f)), CollisionFilter.Default);

        world.UpdateTransform(replacement, new Transform(
            new Vec2(20f, 10f), new Angle32(0), 2f));

        Assert.True(BoundsHit(world, replacement, new Vec2(21f, 15f)));
        Assert.False(BoundsHit(world, replacement, new Vec2(25f, 10f)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SameQuantizedRotationAndScaleTranslationMatchesFullMaterialization(
        bool isStatic)
    {
        const float scale = 1.375f;
        float equivalentScale = float.BitIncrement(scale);
        Assert.Equal(QuantizeScale(scale), QuantizeScale(equivalentScale));

        var rotation = new Angle32(0x2A17C39Du);
        var previousRotation = new Angle32(rotation.Raw + 1u);
        var firstPosition = new Vec2(-123.125f, 84.375f);
        var finalPosition = new Vec2(211.75f, -177.625f);

        int entityId = 1;
        foreach (Shape baseShape in TranslationFastPathShapes())
        {
            using var translated = new ArcWorld();
            using var rematerialized = new ArcWorld();
            ArcHandle translatedHandle = Add(
                translated, entityId, baseShape, isStatic);
            ArcHandle rematerializedHandle = Add(
                rematerialized, entityId, baseShape, isStatic);

            // The second update has the same fixed rotation and scale as the
            // first, even though the public scale uses different float bits.
            translated.UpdateTransform(translatedHandle, new Transform(
                firstPosition, rotation, scale));
            translated.UpdateTransform(translatedHandle, new Transform(
                finalPosition, rotation, equivalentScale));

            // Change rotation immediately before the final pose so this world's
            // final update must take the complete materialization path.
            rematerialized.UpdateTransform(rematerializedHandle, new Transform(
                firstPosition, previousRotation, equivalentScale));
            rematerialized.UpdateTransform(rematerializedHandle, new Transform(
                finalPosition, rotation, equivalentScale));

            AssertShapeBits(
                rematerialized.GetShape(rematerializedHandle),
                translated.GetShape(translatedHandle));

            // Also observe the dynamic tree / lazily rebuilt static BVH, so a
            // shape-only update cannot conceal stale broadphase bounds.
            Assert.True(BoundsHit(
                translated, translatedHandle, finalPosition));
            Assert.False(BoundsHit(
                translated, translatedHandle, firstPosition));
            Assert.Equal(
                BoundsHit(rematerialized, rematerializedHandle, finalPosition),
                BoundsHit(translated, translatedHandle, finalPosition));
            entityId++;
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SamePoseTranslationPreservesLargeCoordinateCapsuleEndpoints(
        bool isStatic)
    {
        const float grid = 1f / 256f;
        var capsule = new Capsule(
            Vec2.Zero, new Vec2(2f * grid, 0f), grid);
        var firstPosition = new Vec2(65_536f, 64f);
        var finalPosition = new Vec2(65_536f + 2f * grid, 64f);

        using var translated = new ArcWorld();
        using var rematerialized = new ArcWorld();
        ArcHandle translatedHandle = Add(
            translated, 1, capsule, isStatic);
        ArcHandle rematerializedHandle = Add(
            rematerialized, 1, capsule, isStatic);

        // The local endpoint offsets are one fixed-point unit. At X=65536 a
        // float ULP is two units, so translating the public endpoint floats
        // would collapse them and produce a bit-different result.
        translated.UpdateTransform(
            translatedHandle, new Transform(firstPosition));
        translated.UpdateTransform(
            translatedHandle, new Transform(finalPosition));

        rematerialized.UpdateTransform(rematerializedHandle, new Transform(
            firstPosition, new Angle32(1u), 1f));
        rematerialized.UpdateTransform(
            rematerializedHandle, new Transform(finalPosition));

        Shape expected = rematerialized.GetShape(rematerializedHandle);
        Shape actual = translated.GetShape(translatedHandle);
        AssertShapeBits(expected, actual);
        Assert.True(actual.TryGetCapsule(out Capsule actualCapsule));
        Assert.NotEqual(
            BitConverter.SingleToUInt32Bits(actualCapsule.A.X),
            BitConverter.SingleToUInt32Bits(actualCapsule.B.X));
        Assert.True(BoundsHit(translated, translatedHandle, finalPosition));
    }

    [Fact]
    public void ScaledPolygonTranslationReusesMaterializedGeometry()
    {
        var polygon = new Polygon(
            new Vec2(-2.25f, -1.5f),
            new Vec2(2.75f, -1.25f),
            new Vec2(1.5f, 2.5f),
            new Vec2(-1.75f, 2.25f));
        var rotation = new Angle32(0x13579BDFu);
        const float scale = 1.625f;

        using var translated = new ArcWorld();
        using var rematerialized = new ArcWorld();
        ArcHandle translatedHandle = translated.Add(
            1, polygon, CollisionFilter.Default);
        ArcHandle rematerializedHandle = rematerialized.Add(
            1, polygon, CollisionFilter.Default);

        translated.UpdateTransform(translatedHandle, new Transform(
            new Vec2(-20.25f, 31.5f), rotation, scale));
        Shape beforeTranslation = translated.GetShape(translatedHandle);
        Assert.True(beforeTranslation.TryGetPolygon(
            out Polygon? beforeGeometry, out _, out _));

        translated.UpdateTransform(translatedHandle, new Transform(
            new Vec2(140.75f, -93.125f),
            rotation,
            float.BitIncrement(scale)));
        rematerialized.UpdateTransform(rematerializedHandle, new Transform(
            new Vec2(-20.25f, 31.5f),
            new Angle32(rotation.Raw + 1u),
            scale));
        rematerialized.UpdateTransform(rematerializedHandle, new Transform(
            new Vec2(140.75f, -93.125f),
            rotation,
            float.BitIncrement(scale)));

        Shape afterTranslation = translated.GetShape(translatedHandle);
        AssertShapeBits(
            rematerialized.GetShape(rematerializedHandle), afterTranslation);
        Assert.True(afterTranslation.TryGetPolygon(
            out Polygon? afterGeometry, out _, out _));
#if REFERENCE_BACKEND
        // The managed backend can expose the optimization directly: translating
        // a scaled polygon retains the already-materialized immutable geometry.
        Assert.Same(beforeGeometry, afterGeometry);
#else
        // Native GetShape returns a fresh managed owner for the retained native
        // polygon, so object identity is intentionally not observable here.
        Assert.NotNull(beforeGeometry);
        Assert.NotNull(afterGeometry);
#endif
    }

    private static ArcHandle Add(
        ArcWorld world, int entityId, Shape shape, bool isStatic) =>
        isStatic
            ? world.AddStatic(entityId, shape, CollisionFilter.Default)
            : world.Add(entityId, shape, CollisionFilter.Default);

    private static IEnumerable<Shape> TranslationFastPathShapes()
    {
        yield return new Circle(new Vec2(.25f, -.375f), 1.125f);
        yield return new Aabb(
            new Vec2(-.5f, .75f), new Vec2(2.25f, 1.375f));
        yield return new Capsule(
            new Vec2(-2.125f, -.625f),
            new Vec2(2.375f, .875f),
            .8125f);
        yield return new Obb(
            new Vec2(.625f, -.875f),
            new Vec2(2.5f, .75f),
            new Angle32(0x10203040u));
        yield return new Polygon(
            new Vec2(-2.25f, -1.5f),
            new Vec2(2.75f, -1.25f),
            new Vec2(1.5f, 2.5f),
            new Vec2(-1.75f, 2.25f));
    }

    private static long QuantizeScale(float scale) =>
        (long)Math.Round(
            (double)scale * 65_536.0, MidpointRounding.ToEven);

    private static void AssertShapeBits(Shape expected, Shape actual)
    {
        Assert.Equal(expected.Kind, actual.Kind);
        switch (expected.Kind)
        {
            case ShapeKind.Circle:
                Assert.True(expected.TryGetCircle(out Circle expectedCircle));
                Assert.True(actual.TryGetCircle(out Circle actualCircle));
                AssertVecBits(expectedCircle.Center, actualCircle.Center);
                AssertFloatBits(expectedCircle.Radius, actualCircle.Radius);
                break;
            case ShapeKind.Aabb:
                Assert.True(expected.TryGetAabb(out Aabb expectedAabb));
                Assert.True(actual.TryGetAabb(out Aabb actualAabb));
                AssertVecBits(expectedAabb.Center, actualAabb.Center);
                AssertVecBits(
                    expectedAabb.HalfExtents, actualAabb.HalfExtents);
                break;
            case ShapeKind.Capsule:
                Assert.True(expected.TryGetCapsule(
                    out Capsule expectedCapsule));
                Assert.True(actual.TryGetCapsule(out Capsule actualCapsule));
                AssertVecBits(expectedCapsule.A, actualCapsule.A);
                AssertVecBits(expectedCapsule.B, actualCapsule.B);
                AssertFloatBits(
                    expectedCapsule.Radius, actualCapsule.Radius);
                break;
            case ShapeKind.Obb:
                Assert.True(expected.TryGetObb(out Obb expectedObb));
                Assert.True(actual.TryGetObb(out Obb actualObb));
                AssertVecBits(expectedObb.Center, actualObb.Center);
                AssertVecBits(expectedObb.HalfExtents, actualObb.HalfExtents);
                Assert.Equal(expectedObb.Angle.Raw, actualObb.Angle.Raw);
                break;
            case ShapeKind.Polygon:
                Assert.True(expected.TryGetPolygon(
                    out Polygon? expectedPolygon,
                    out Vec2 expectedTranslation,
                    out Angle32 expectedRotation));
                Assert.True(actual.TryGetPolygon(
                    out Polygon? actualPolygon,
                    out Vec2 actualTranslation,
                    out Angle32 actualRotation));
                AssertVecBits(expectedTranslation, actualTranslation);
                Assert.Equal(expectedRotation.Raw, actualRotation.Raw);
                Assert.Equal(expectedPolygon!.Count, actualPolygon!.Count);
                for (int i = 0; i < expectedPolygon.Count; i++)
                    AssertVecBits(expectedPolygon[i], actualPolygon[i]);
                break;
            default:
                throw new InvalidOperationException("Invalid shape kind.");
        }
    }

    private static void AssertVecBits(Vec2 expected, Vec2 actual)
    {
        AssertFloatBits(expected.X, actual.X);
        AssertFloatBits(expected.Y, actual.Y);
    }

    private static void AssertFloatBits(float expected, float actual) =>
        Assert.Equal(
            BitConverter.SingleToUInt32Bits(expected),
            BitConverter.SingleToUInt32Bits(actual));

    private static IEnumerable<Vec2> ProbeGrid(Vec2 center, float radius)
    {
        for (float dx = -radius; dx <= radius; dx += 1f)
            for (float dy = -radius; dy <= radius; dy += 1f)
                yield return new Vec2(center.X + dx, center.Y + dy);
    }
}
