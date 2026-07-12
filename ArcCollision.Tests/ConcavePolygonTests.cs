using System;
using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Hand-crafted concave polygon tests. Each test defines a specific concave
/// polygon topology and places probe shapes at strategic positions: inside
/// arms, in concavity gaps, at reflex vertices, etc.
///
/// These complement the fuzzing/oracle tests by covering specific geometric
/// configurations that random generation is unlikely to produce.
/// </summary>
public class ConcavePolygonTests
{
    // ============================================================ L-shape

    // An L-shape occupying the bottom and left of a 4×4 square:
    //   (0,4)──(1,4)
    //     │      │
    //   (0,0)──(4,0)──(4,1)
    //                  │
    //           (1,1)──┘
    private static readonly Polygon LShape = new(
        new Vec2(0, 0), new Vec2(4, 0), new Vec2(4, 1),
        new Vec2(1, 1), new Vec2(1, 4), new Vec2(0, 4));

    [Fact]
    public void LShape_CircleInGap_DoesNotCollide()
    {
        // (2.5, 2.5) is in the concavity gap (upper-right void)
        Assert.False(Collide.Overlaps(LShape, new Circle(new Vec2(2.5f, 2.5f), 0.3f)));
    }

    [Fact]
    public void LShape_CircleInBottomArm_Collides()
    {
        // (2.0, 0.5) is inside the bottom arm
        Assert.True(Collide.Overlaps(LShape, new Circle(new Vec2(2f, 0.5f), 0.3f)));
    }

    [Fact]
    public void LShape_CircleInLeftArm_Collides()
    {
        // (0.5, 3.0) is inside the left arm
        Assert.True(Collide.Overlaps(LShape, new Circle(new Vec2(0.5f, 3f), 0.3f)));
    }

    [Fact]
    public void LShape_CircleAtReflexCorner_Collides()
    {
        // Just inside the reflex vertex at (1,1)
        Assert.True(Collide.Overlaps(LShape, new Circle(new Vec2(0.8f, 0.8f), 0.3f)));
    }

    [Fact]
    public void LShape_ManifoldDepthAndSeparation()
    {
        var circle = new Circle(new Vec2(2f, 0.5f), 0.3f);
        Manifold m = Collide.ShapeVsShape(LShape, circle);
        Assert.True(m.Colliding);
        Assert.True(m.Depth > 0f);

        Shape moved = ((Shape)LShape).Moved(
            m.SeparationForA - m.Normal * (2f / 256f));
        Manifold after = Collide.ShapeVsShape(moved, circle);
        Assert.True(!after.Colliding,
            $"separation did not resolve overlap: final depth {after.Depth:R}");
    }

    // ============================================================ U-shape

    // U-shape: open at the top, walls on left and right with a channel
    private static readonly Polygon UShape = new(
        new Vec2(0, 0), new Vec2(3, 0), new Vec2(3, 4),
        new Vec2(2, 4), new Vec2(2, 1), new Vec2(1, 1),
        new Vec2(1, 4), new Vec2(0, 4));

    [Fact]
    public void UShape_CircleInChannel_DoesNotCollide()
    {
        // (1.5, 2.5) is in the U-channel gap
        Assert.False(Collide.Overlaps(UShape, new Circle(new Vec2(1.5f, 2.5f), 0.3f)));
    }

    [Fact]
    public void UShape_CircleInLeftWall_Collides()
    {
        Assert.True(Collide.Overlaps(UShape, new Circle(new Vec2(0.5f, 2f), 0.3f)));
    }

    [Fact]
    public void UShape_CircleInRightWall_Collides()
    {
        Assert.True(Collide.Overlaps(UShape, new Circle(new Vec2(2.5f, 2f), 0.3f)));
    }

    [Fact]
    public void UShape_CircleInBottom_Collides()
    {
        Assert.True(Collide.Overlaps(UShape, new Circle(new Vec2(1.5f, 0.5f), 0.3f)));
    }

    // ============================================================ T-shape

    // T-shape: horizontal top bar + vertical stem
    private static readonly Polygon TShape = new(
        new Vec2(0, 3), new Vec2(4, 3), new Vec2(4, 4),
        new Vec2(2.5f, 4), new Vec2(2.5f, 6),
        new Vec2(1.5f, 6), new Vec2(1.5f, 4), new Vec2(0, 4));

    [Fact]
    public void TShape_CircleInGapBesideStem_DoesNotCollide()
    {
        // (0.5, 5) is beside the stem but above the bar
        Assert.False(Collide.Overlaps(TShape, new Circle(new Vec2(0.5f, 5f), 0.3f)));
    }

    [Fact]
    public void TShape_CircleInStem_Collides()
    {
        Assert.True(Collide.Overlaps(TShape, new Circle(new Vec2(2f, 5f), 0.3f)));
    }

    [Fact]
    public void TShape_CircleInBar_Collides()
    {
        Assert.True(Collide.Overlaps(TShape, new Circle(new Vec2(3f, 3.5f), 0.3f)));
    }

    // ============================================================ Star (5-pointed)

    private static Polygon MakeStar(Vec2 center, float outerR, float innerR)
    {
        var verts = new Vec2[10];
        for (int i = 0; i < 10; i++)
        {
            double angle = Math.PI / 2 + i * Math.PI / 5;
            float r = i % 2 == 0 ? outerR : innerR;
            verts[i] = new Vec2(
                center.X + (float)Math.Cos(angle) * r,
                center.Y + (float)Math.Sin(angle) * r);
        }
        return new Polygon(verts);
    }

    [Fact]
    public void Star_CircleInConcavity_DoesNotCollide()
    {
        Polygon star = MakeStar(new Vec2(10, 10), 5f, 2f);
        // Point between two arms (concavity) at angle halfway between arms
        double angle = Math.PI / 2 + Math.PI / 5 * 0.5;
        float probeR = 3.5f;
        var probeCenter = new Vec2(
            10f + (float)Math.Cos(angle) * probeR,
            10f + (float)Math.Sin(angle) * probeR);
        Assert.False(Collide.Overlaps(star, new Circle(probeCenter, 0.3f)),
            $"expected miss in star concavity at {probeCenter.X}, {probeCenter.Y}");
    }

    [Fact]
    public void Star_CircleAtTip_Collides()
    {
        Polygon star = MakeStar(new Vec2(10, 10), 5f, 2f);
        // Top tip is at (10, 15)
        Assert.True(Collide.Overlaps(star, new Circle(new Vec2(10f, 14.5f), 0.3f)));
    }

    [Fact]
    public void Star_CircleAtCenter_Collides()
    {
        Polygon star = MakeStar(new Vec2(10, 10), 5f, 2f);
        Assert.True(Collide.Overlaps(star, new Circle(new Vec2(10f, 10f), 0.3f)));
    }

    // ============================================================ Comb

    // Comb: base with 3 teeth pointing up, gaps between teeth
    private static readonly Polygon Comb = new(
        new Vec2(0, 0), new Vec2(6, 0), new Vec2(6, 3), new Vec2(5, 3),
        new Vec2(5, 1), new Vec2(4, 1), new Vec2(4, 3), new Vec2(3, 3),
        new Vec2(3, 1), new Vec2(2, 1), new Vec2(2, 3), new Vec2(1, 3),
        new Vec2(1, 1), new Vec2(0, 1));

    [Fact]
    public void Comb_CircleInGap_DoesNotCollide()
    {
        // (2.5, 2) is in the gap between tooth 2 (x=[1,2]) and tooth 3 (x=[3,4])
        Assert.False(Collide.Overlaps(Comb, new Circle(new Vec2(2.5f, 2f), 0.3f)));
    }

    [Fact]
    public void Comb_CircleInTooth_Collides()
    {
        // (1.5, 2) is inside the third tooth (x=[1,2])
        Assert.True(Collide.Overlaps(Comb, new Circle(new Vec2(1.5f, 2f), 0.3f)));
    }

    [Fact]
    public void Comb_CircleInBase_Collides()
    {
        // (3, 0.5) is in the base
        Assert.True(Collide.Overlaps(Comb, new Circle(new Vec2(3f, 0.5f), 0.3f)));
    }

    // ============================================================ Cross-shape tests

    [Fact]
    public void ConcaveVsAabb_GapMiss_ArmHit()
    {
        Assert.False(Collide.ShapeVsShape(
            LShape, new Aabb(new Vec2(2.5f, 2.5f), new Vec2(0.3f, 0.3f))).Colliding);
        Assert.True(Collide.ShapeVsShape(
            LShape, new Aabb(new Vec2(2f, 0.5f), new Vec2(0.3f, 0.3f))).Colliding);
    }

    [Fact]
    public void ConcaveVsCapsule_GapMiss_ArmHit()
    {
        var capGap = new Capsule(new Vec2(2.5f, 2.5f), new Vec2(2.5f, 2.5f), 0.3f);
        var capArm = new Capsule(new Vec2(2f, 0.5f), new Vec2(2f, 0.5f), 0.3f);
        Assert.False(Collide.Overlaps(LShape, capGap));
        Assert.True(Collide.Overlaps(LShape, capArm));
    }

    [Fact]
    public void ConcaveVsObb_GapMiss_ArmHit()
    {
        var obbGap = new Obb(new Vec2(2.5f, 2.5f), new Vec2(0.3f, 0.3f), 0.5f);
        var obbArm = new Obb(new Vec2(2f, 0.5f), new Vec2(0.3f, 0.3f), 0.5f);
        Assert.False(Collide.Overlaps(LShape, obbGap));
        Assert.True(Collide.Overlaps(LShape, obbArm));
    }

    [Fact]
    public void ConcaveVsConcave_OverlapAndGap()
    {
        // Two L-shapes: one original, one shifted to overlap
        Shape shifted = ((Shape)LShape).Moved(new Vec2(0.5f, 0.5f));
        Assert.True(Collide.Overlaps(LShape, shifted));

        // Shifted far: verify no crash even if gap-vs-gap
        Shape farShifted = ((Shape)LShape).Moved(new Vec2(2f, 2f));
        _ = Collide.ShapeVsShape(LShape, farShifted);
        _ = Collide.Overlaps(LShape, farShifted);
    }

    // ============================================================ Depth consistency

    [Fact]
    public void AllConcaveShapes_DepthIsPositiveWhenColliding()
    {
        Polygon[] concaves = { LShape, UShape, TShape, Comb };
        foreach (Polygon poly in concaves)
        {
            Vec2 center = poly.Bounds.Center;
            var centeredProbe = new Circle(center, 0.3f);
            Manifold m = Collide.ShapeVsShape(poly, centeredProbe);
            if (m.Colliding)
            {
                Assert.True(m.Depth > 0f,
                    $"negative depth for probe at {center.X},{center.Y} on concave polygon");
            }
        }
    }
}
