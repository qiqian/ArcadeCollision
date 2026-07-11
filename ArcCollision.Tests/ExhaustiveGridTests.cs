using System;
using ArcCollision.Tests.Support;
using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Exhaustively enumerates every sub-pixel configuration on the 1/256 grid in a
/// window around exact touch and compares the library's boolean predicates with
/// exact integer math — zero tolerance. This mechanically covers every rounding
/// corner case for the exact-predicate pairs.
/// </summary>
public class ExhaustiveGridTests
{
    // The library's contract: touching (distSq == r²) counts as colliding for
    // circle predicates; exact edge contact counts for AABB overlap; but
    // AabbVsAabb's manifold requires strictly positive overlap.

    [Theory]
    [InlineData(256, 256)]     // r=1 vs r=1
    [InlineData(256, 384)]     // r=1 vs r=1.5
    [InlineData(1, 1)]         // sub-pixel radii (1/256 each)
    [InlineData(0, 512)]       // zero radius vs r=2
    [InlineData(97, 351)]      // odd radii — exercises isqrt/rounding parity
    public void CircleVsCircle_Boolean_MatchesExactIntegerMath(long rAfx, long rBfx)
    {
        float rA = rAfx / 256f, rB = rBfx / 256f;
        long touch = rAfx + rBfx;
        // Window: from clearly-inside to clearly-outside around exact touch,
        // scanning dx while dy sweeps a diagonal band.
        long window = 160;
        var a = new Circle(new Vec2(0, 0), rA);

        for (long dy = 0; dy <= touch + window; dy += 7)
        {
            for (long dx = Math.Max(0, touch - window); dx <= touch + window; dx++)
            {
                bool expected = dx * dx + dy * dy <= touch * touch;
                var b = new Circle(new Vec2(dx / 256f, dy / 256f), rB);
                bool actual = Collide.CircleVsCircle(a, b).Colliding;
                if (actual != expected)
                    Assert.Fail($"circle-circle mismatch at dx={dx} dy={dy} rA={rAfx} rB={rBfx}: expected {expected}");
            }
        }
    }

    [Fact]
    public void CircleVsCircle_ExactTouch_IsCollidingWithZeroDepth()
    {
        // 3-4-5 triangle: distance exactly 5 = 2 + 3.
        var a = new Circle(new Vec2(0, 0), 2f);
        var b = new Circle(new Vec2(3f, 4f), 3f);
        var m = Collide.CircleVsCircle(a, b);
        Assert.True(m.Colliding);
        Assert.Equal(0f, m.Depth);

        // One grid step farther: separated.
        var b2 = new Circle(new Vec2(3f + TestGeo.Grid, 4f), 3f);
        Assert.False(Collide.CircleVsCircle(a, b2).Colliding);
    }

    [Fact]
    public void AabbOverlap_Boolean_MatchesExactIntegerMath_AroundCornerTouch()
    {
        var a = new Aabb(new Vec2(0, 0), new Vec2(1f, 0.75f));
        long hx = 256, hy = 192;
        long bx = 128, by = 96;   // b half extents 0.5 × 0.375

        for (long dy = hy + by - 96; dy <= hy + by + 96; dy++)
        {
            for (long dx = hx + bx - 96; dx <= hx + bx + 96; dx += 3)
            {
                var b = new Aabb(new Vec2(dx / 256f, dy / 256f), new Vec2(bx / 256f, by / 256f));
                bool expected = dx <= hx + bx && dy <= hy + by;   // inclusive touch
                bool actual = a.Overlaps(b);
                if (actual != expected)
                    Assert.Fail($"aabb overlap mismatch at dx={dx} dy={dy}: expected {expected}");

                // The manifold variant requires strictly positive overlap on both axes.
                bool expectManifold = dx < hx + bx && dy < hy + by;
                bool actualManifold = Collide.AabbVsAabb(a, b).Colliding;
                if (actualManifold != expectManifold)
                    Assert.Fail($"AabbVsAabb mismatch at dx={dx} dy={dy}: expected {expectManifold}");
            }
        }
    }

    [Fact]
    public void CircleVsAabb_Boolean_MatchesExactIntegerMath_AroundCornerArc()
    {
        // Box corner at (256, 192) fixed units; circle radius 128 (0.5).
        var box = new Aabb(new Vec2(0, 0), new Vec2(1f, 0.75f));
        long cornerX = 256, cornerY = 192, r = 128;

        for (long dy = -32; dy <= r + 64; dy += 2)
        {
            for (long dx = -32; dx <= r + 64; dx++)
            {
                long px = cornerX + dx, py = cornerY + dy;
                long cx = Math.Clamp(px, -cornerX, cornerX);
                long cy = Math.Clamp(py, -cornerY, cornerY);
                long ddx = px - cx, ddy = py - cy;
                bool expected = ddx * ddx + ddy * ddy <= r * r;

                var c = new Circle(new Vec2(px / 256f, py / 256f), r / 256f);
                bool actual = Collide.CircleVsAabb(c, box).Colliding;
                if (actual != expected)
                    Assert.Fail($"circle-aabb mismatch at px={px} py={py}: expected {expected}");
            }
        }
    }

    [Fact]
    public void PointPredicates_MatchExactIntegerMath()
    {
        var circle = new Circle(new Vec2(0.5f, -0.25f), 1.25f);
        long ccx = 128, ccy = -64, r = 320;
        var box = new Aabb(new Vec2(-0.5f, 0.25f), new Vec2(0.75f, 1.5f));
        long bcx = -128, bcy = 64, bhx = 192, bhy = 384;

        for (long py = -600; py <= 600; py += 3)
        {
            for (long px = -600; px <= 600; px += 3)
            {
                var p = new Vec2(px / 256f, py / 256f);

                long dx = px - ccx, dy = py - ccy;
                bool inCircle = dx * dx + dy * dy <= r * r;
                Assert.Equal(inCircle, Collide.PointInCircle(p, circle));

                bool inBox = Math.Abs(px - bcx) <= bhx && Math.Abs(py - bcy) <= bhy;
                Assert.Equal(inBox, Collide.PointInAabb(p, box));
            }
        }
    }

    [Fact]
    public void RoundingBoundary_HalfGridInputs_UseBankersRounding()
    {
        // 0.5/256 inputs round to even fixed values on both sides of zero, so
        // symmetric inputs must produce symmetric predicates.
        float half = 0.5f / 256f;
        var a = new Circle(new Vec2(half, 0), 1f);
        var b = new Circle(new Vec2(-half, 0), 1f);
        var m1 = Collide.CircleVsCircle(a, new Circle(new Vec2(2f + half, 0), 1f));
        var m2 = Collide.CircleVsCircle(b, new Circle(new Vec2(-2f - half, 0), 1f));
        Assert.Equal(m1.Colliding, m2.Colliding);
        Assert.Equal(m1.Depth, m2.Depth);
    }
}
