using System;
using Xunit;

namespace ArcCollision.Tests;

public class BoundaryRegressionTests
{
    private const float Grid = 1f / 256f;

    [Fact]
    public void InitialOverlap_AllFastSweepOrdersReturnUnitNormals()
    {
        Shape circle = new Circle(new Vec2(0.25f, 0), 1);
        Shape aabb = new Aabb(Vec2.Zero, new Vec2(2, 2));
        Shape capsule = new Capsule(new Vec2(-2, 0), new Vec2(2, 0), 0.75f);
        Shape obb = new Obb(Vec2.Zero, new Vec2(2, 1), 0.43f);
        (Shape Mover, Shape Target)[] pairs =
        {
            (circle, new Circle(Vec2.Zero, 1)),
            (circle, aabb), (aabb, circle),
            (circle, capsule), (capsule, circle),
            (circle, obb), (obb, circle),
            (aabb, new Aabb(new Vec2(0.5f, 0), new Vec2(2, 2))),
        };

        foreach ((Shape mover, Shape target) in pairs)
        {
            AssertInitialHit(Sweep.MovingShapeVsShape(mover, Vec2.Zero, target),
                $"{mover.Kind}/{target.Kind} zero motion");
            AssertInitialHit(Sweep.MovingShapeVsShape(
                mover, new Vec2(3, 0.25f), target),
                $"{mover.Kind}/{target.Kind} moving");
        }
    }

    [Fact]
    public void RayVsAabb_StartInsideOrOnEveryFaceReturnsUnitNormal()
    {
        var box = new Aabb(Vec2.Zero, new Vec2(4, 3));
        Vec2[] origins =
        {
            Vec2.Zero, new(-3.5f, 0), new(3.5f, 0), new(0, -2.5f), new(0, 2.5f),
            new(-4, 0), new(4, 0), new(0, -3), new(0, 3),
        };
        Vec2[] motions = { Vec2.Zero, new(10, 1), new(-7, -2) };

        foreach (Vec2 origin in origins)
        foreach (Vec2 motion in motions)
            AssertInitialHit(Sweep.RayVsAabb(origin, motion, box),
                $"origin={origin}, motion={motion}");
    }

    [Fact]
    public void InitialOverlapFastSweepsRemainValidAtFarCoordinates()
    {
        foreach (float offset in new[] { -1_800_000f, 1_800_000f })
        {
            Vec2 at = new(offset, -offset);
            AssertInitialHit(Sweep.MovingCircleVsAabb(
                new Circle(at, 2), new Vec2(100, 1),
                new Aabb(at, new Vec2(5, 5))), $"circle/aabb at {offset}");
            AssertInitialHit(Sweep.MovingAabbVsAabb(
                new Aabb(at, new Vec2(2, 2)), new Vec2(-100, 3),
                new Aabb(at + new Vec2(1, 0), new Vec2(2, 2))),
                $"aabb/aabb at {offset}");
        }
    }

    [Fact]
    public void PolygonRejectsSeveralSelfIntersectionTopologies()
    {
        var invalid = new List<Vec2[]>
        {
            new[] { new Vec2(0, 0), new Vec2(2, 2), new Vec2(0, 2), new Vec2(2, 0) },
            new[] { new Vec2(0, 0), new Vec2(3, 0), new Vec2(3, 3), new Vec2(0, 3), new Vec2(3, 0) },
            new[] { new Vec2(0, 0), new Vec2(4, 0), new Vec2(4, 4), new Vec2(2, 0), new Vec2(0, 4) },
            new[] { new Vec2(0, 0), new Vec2(3, 0), new Vec2(1, 0), new Vec2(1, 2), new Vec2(0, 2) },
            new[] { new Vec2(0, 0), new Vec2(1f / 1024f, 0), new Vec2(1, 1), new Vec2(0, 1) },
            Pentagram(5),
        };

        foreach (Vec2[] vertices in invalid)
            Assert.Throws<ArgumentException>(() => new Polygon(vertices));
    }

    [Fact]
    public void PolygonAcceptsForwardCollinearEdgesAndBothWindings()
    {
        Vec2[] withForwardCollinear =
        {
            new(0, 0), new(1, 0), new(2, 0), new(2, 2), new(0, 2),
        };
        Vec2[] concave =
        {
            new(0, 0), new(4, 0), new(4, 1), new(1, 1), new(1, 4), new(0, 4),
        };

        _ = new Polygon(withForwardCollinear);
        _ = new Polygon(concave);
        Array.Reverse(concave);
        _ = new Polygon(concave);
    }

    [Fact]
    public void DeepCapsuleCrossingsAreSwapSymmetricAndSeparateInOneStep()
    {
        foreach (float angle in new[] { 0f, 0.31f, 0.79f, 1.5707963f })
        {
            Vec2 direction = new(MathF.Cos(angle), MathF.Sin(angle));
            Shape a = new Capsule(new Vec2(-6, 0), new Vec2(6, 0), 1);
            Shape b = new Capsule(direction * -5, direction * 5, 0.75f);
            AssertSymmetricSeparation(a, b, $"cross angle {angle:R}");
        }

        AssertSymmetricSeparation(
            new Capsule(new Vec2(-5, 0), new Vec2(5, 0), 1),
            new Capsule(new Vec2(-2, 0), new Vec2(8, 0), 0.5f),
            "collinear overlap");
        AssertSymmetricSeparation(
            new Capsule(Vec2.Zero, Vec2.Zero, 2),
            new Capsule(new Vec2(1, 0), new Vec2(1, 0), 1),
            "degenerate capsules");
    }

    [Fact]
    public void CircleCenteredOnSlantedCapsuleSpineUsesPerpendicularMtv()
    {
        var capsule = new Capsule(new Vec2(-4, -2), new Vec2(4, 2), 1);
        var circle = new Circle(Vec2.Zero, 0.75f);
        Manifold manifold = Collide.CircleVsCapsule(circle, capsule);
        Vec2 spine = capsule.B - capsule.A;

        Assert.True(manifold.Colliding);
        Assert.InRange(MathF.Abs(manifold.Normal.Dot(spine.Normalized())), 0, 2e-6f);
        Circle separated = circle.Moved(
            manifold.SeparationForA - manifold.Normal * (2 * Grid));
        Assert.False(Collide.Overlaps(separated, capsule));
    }

    [Fact]
    public void DeepContainmentPairsReturnOneStepSeparationsInBothOrders()
    {
        Polygon largePolygon = new(
            new Vec2(-8, -6), new Vec2(9, -5), new Vec2(8, 7), new Vec2(-7, 8));
        Polygon smallPolygon = new(
            new Vec2(-1, -1), new Vec2(1.5f, -0.5f), new Vec2(0.5f, 1.5f));
        (Shape A, Shape B)[] pairs =
        {
            (new Aabb(new Vec2(0.5f, 0), new Vec2(1, 1)),
                new Aabb(Vec2.Zero, new Vec2(6, 5))),
            (new Obb(new Vec2(0.4f, -0.2f), new Vec2(1.5f, 0.75f), 0.6f),
                new Aabb(Vec2.Zero, new Vec2(7, 6))),
            (new Capsule(new Vec2(-2, 0), new Vec2(2, 1), 0.5f), largePolygon),
            (new Circle(new Vec2(0.25f, -0.25f), 0.75f), largePolygon),
            (smallPolygon, new Obb(new Vec2(0.3f, 0.2f), new Vec2(6, 5), 0.2f)),
        };

        foreach ((Shape a, Shape b) in pairs)
            AssertSymmetricSeparation(a, b, $"{a.Kind}/{b.Kind} containment");
    }

    [Fact]
    public void ConcaveUnionsReturnOneStepVerifiedSeparations()
    {
        Polygon[] polygons =
        {
            new(new Vec2(0, 0), new Vec2(4, 0), new Vec2(4, 1),
                new Vec2(1, 1), new Vec2(1, 4), new Vec2(0, 4)),
            new(new Vec2(0, 0), new Vec2(4, 0), new Vec2(4, 5),
                new Vec2(3, 5), new Vec2(3, 1), new Vec2(1, 1),
                new Vec2(1, 5), new Vec2(0, 5)),
            new(new Vec2(0, 0), new Vec2(6, 0), new Vec2(6, 3),
                new Vec2(5, 3), new Vec2(5, 1), new Vec2(4, 1),
                new Vec2(4, 3), new Vec2(3, 3), new Vec2(3, 1),
                new Vec2(2, 1), new Vec2(2, 3), new Vec2(1, 3),
                new Vec2(1, 1), new Vec2(0, 1)),
        };
        Circle[] probes =
        {
            new(new Vec2(0.6f, 2.5f), 0.75f),
            new(new Vec2(0.5f, 2.5f), 0.75f),
            new(new Vec2(4.5f, 2.4f), 0.75f),
        };

        for (int i = 0; i < polygons.Length; i++)
            AssertSymmetricSeparation(polygons[i], probes[i], $"concave {i}");
    }

    [Fact]
    public void BoxAndSatContactsStayInsideOverlappingWorldBounds()
    {
        (Shape A, Shape B)[] pairs =
        {
            (new Aabb(Vec2.Zero, new Vec2(1, 1)),
                new Aabb(new Vec2(0.5f, 5), new Vec2(1, 10))),
            (new Aabb(Vec2.Zero, new Vec2(8, 7)),
                new Aabb(new Vec2(1, -2), new Vec2(1, 1))),
            (new Obb(Vec2.Zero, new Vec2(4, 1), 0.4f),
                new Aabb(new Vec2(2, 0), new Vec2(3, 2))),
            (new Polygon(new Vec2(-4, -2), new Vec2(4, -2),
                    new Vec2(3, 3), new Vec2(-3, 3)),
                new Obb(new Vec2(2, 0), new Vec2(3, 1), -0.3f)),
        };

        foreach ((Shape a, Shape b) in pairs)
        {
            Manifold manifold = Collide.ShapeVsShape(a, b);
            Assert.True(manifold.Colliding);
            AssertInBoundsIntersection(manifold.Contact, a.Bounds, b.Bounds,
                $"{a.Kind}/{b.Kind}");
        }
    }

    private static void AssertInitialHit(SweepHit hit, string context)
    {
        Assert.True(hit.Hit, context);
        Assert.Equal(0f, hit.Time);
        Assert.InRange(hit.Normal.Length, 1f - 2e-6f, 1f + 2e-6f);
        Assert.True(float.IsFinite(hit.Point.X) && float.IsFinite(hit.Point.Y), context);
    }

    private static void AssertSymmetricSeparation(Shape a, Shape b, string context)
    {
        Manifold ab = Collide.ShapeVsShape(a, b);
        Manifold ba = Collide.ShapeVsShape(b, a);
        Assert.True(ab.Colliding && ba.Colliding, context);
        Assert.Equal(ab.Depth, ba.Depth, 3 * Grid);

        Shape movedA = a.Moved(ab.SeparationForA - ab.Normal * (2 * Grid));
        Shape movedB = b.Moved(ba.SeparationForA - ba.Normal * (2 * Grid));
        Assert.False(Collide.Overlaps(movedA, b), $"A separation failed: {context}");
        Assert.False(Collide.Overlaps(movedB, a), $"B separation failed: {context}");
    }

    private static void AssertInBoundsIntersection(
        Vec2 point, Aabb a, Aabb b, string context)
    {
        float minX = MathF.Max(a.Min.X, b.Min.X) - Grid;
        float maxX = MathF.Min(a.Max.X, b.Max.X) + Grid;
        float minY = MathF.Max(a.Min.Y, b.Min.Y) - Grid;
        float maxY = MathF.Min(a.Max.Y, b.Max.Y) + Grid;
        Assert.True(point.X >= minX && point.X <= maxX
            && point.Y >= minY && point.Y <= maxY,
            $"contact {point} outside bounds intersection: {context}");
    }

    private static Vec2[] Pentagram(float radius)
    {
        var result = new Vec2[5];
        for (int i = 0; i < result.Length; i++)
        {
            int vertex = i * 2 % 5;
            float angle = MathF.PI * 0.5f + vertex * MathF.PI * 2f / 5f;
            result[i] = new Vec2(MathF.Cos(angle) * radius, MathF.Sin(angle) * radius);
        }
        return result;
    }
}
