using System;
using ArcCollision.Tests.Support;
using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Validates that Manifold.Contact coordinates match a double-precision oracle.
/// Only tested for collision pairs with well-defined contact semantics:
///
///  - Circle vs Circle: midpoint of overlap along centre line
///  - Circle vs AABB (centre outside): closest point on box surface
///  - Circle vs Capsule: midpoint of overlap along circle-to-spine-projection line
///
/// Contact tolerance includes fixed-grid rounding plus the small Q1.30 direction
/// error amplified by the witness offset.
/// </summary>
public class ContactOracleTests
{
    private const int Iterations = 2000;
    private static readonly Regime[] Regimes = { Regime.Origin, Regime.Mid };

    /// <summary>
    /// SAT paths (OBB / polygon) return an approximate contact, but it must at
    /// least lie inside both operands' bounding boxes now that it is clamped to
    /// their overlap. The reviewer's example is the anchor case: an unclamped
    /// midpoint landed at y≈1.25, outside the OBB's [-0.5, 0.5] band.
    /// </summary>
    [Fact]
    public void SatContact_LiesWithinBothOperandBounds()
    {
        var box = new Aabb(new Vec2(0, 0), new Vec2(2, 2));
        var obb = new Obb(new Vec2(3, 0), new Vec2(2, 0.5f), 0f);
        Manifold m = Collide.ShapeVsShape(new Shape(box), new Shape(obb));
        Assert.True(m.Colliding);
        Assert.True(m.Contact.Y is >= -0.5f - 1f / 256f and <= 0.5f + 1f / 256f,
            $"contact y={m.Contact.Y:R} outside the OBB band");

        // Fuzz: any colliding OBB / polygon pair's contact must sit inside the
        // intersection of the two bounding boxes.
        var gen = new FuzzGen(555, Regime.Mid);
        for (int i = 0; i < 3000; i++)
        {
            Shape a = i % 2 == 0 ? new Shape(TestGeo.Q(gen.Obb(gen.Position())))
                                 : new Shape(TestGeo.Q(gen.Aabb(gen.Position())));
            Obb bo = TestGeo.Q(gen.Obb(gen.Near(a.Bounds.Center, FuzzGen.Reach(new Aabb(a.Bounds.Center, a.Bounds.HalfExtents)) + gen.Size())));
            Shape b = new Shape(bo);
            Manifold m2 = Collide.ShapeVsShape(a, b);
            if (!m2.Colliding) continue;

            Aabb ba = a.Bounds, bb = b.Bounds;
            float loX = MathF.Max(ba.Min.X, bb.Min.X) - 1f / 256f;
            float hiX = MathF.Min(ba.Max.X, bb.Max.X) + 1f / 256f;
            float loY = MathF.Max(ba.Min.Y, bb.Min.Y) - 1f / 256f;
            float hiY = MathF.Min(ba.Max.Y, bb.Max.Y) + 1f / 256f;
            Assert.True(m2.Contact.X >= loX && m2.Contact.X <= hiX
                && m2.Contact.Y >= loY && m2.Contact.Y <= hiY,
                $"[i={i}] contact {TestGeo.Dump(m2.Contact)} outside overlap [{loX:F3},{hiX:F3}]x[{loY:F3},{hiY:F3}]: "
                + $"{TestGeo.Dump(new Aabb(a.Bounds.Center, a.Bounds.HalfExtents))} vs {TestGeo.Dump(bo)}");
        }
    }

    // Base tolerance: 3 grid cells for sub-pixel rounding.
    private const double BaseTol = 3.0 / 256.0;

    private static double ScaledTol(float maxRadius)
    {
        // Q1.30 direction rounding is amplified by the witness offset.
        return BaseTol + Math.Abs(maxRadius) * 0.0000002;
    }

    private static void CheckContact(
        Manifold m, double expectedX, double expectedY, double tolerance, string repro)
    {
        if (!m.Colliding) return;
        double dx = Math.Abs(m.Contact.X - expectedX);
        double dy = Math.Abs(m.Contact.Y - expectedY);
        Assert.True(dx <= tolerance && dy <= tolerance,
            $"contact ({m.Contact.X:F4}, {m.Contact.Y:F4}) vs oracle ({expectedX:F4}, {expectedY:F4}), " +
            $"delta ({dx:F6}, {dy:F6}), tol={tolerance:F4}: {repro}");
    }

    [Theory]
    [InlineData(101)]
    [InlineData(102)]
    public void CircleVsCircle_ContactMatchesOracle(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            int verified = 0;
            for (int i = 0; i < Iterations; i++)
            {
                Circle a = TestGeo.Q(gen.Circle(gen.Position()));
                Circle b = TestGeo.Q(gen.Circle(gen.Near(a.Center, FuzzGen.Reach(a) + gen.Size())));
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(a)}, {TestGeo.Dump(b)}";

                Manifold m = Collide.CircleVsCircle(a, b);
                if (!m.Colliding || m.Depth <= 2f / 256f) continue;

                (double ex, double ey) = TestGeo.ContactCircleCircle(a, b);
                double tol = ScaledTol(MathF.Max(MathF.Abs(a.Radius), MathF.Abs(b.Radius)));
                CheckContact(m, ex, ey, tol, repro);
                verified++;
            }
            Assert.True(verified > Iterations / 6,
                $"too few verified contacts ({verified}) at {regime}");
        }
    }

    [Theory]
    [InlineData(103)]
    [InlineData(104)]
    public void CircleVsAabb_ContactMatchesOracle_CentreOutside(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            int verified = 0;
            for (int i = 0; i < Iterations; i++)
            {
                Aabb box = TestGeo.Q(gen.Aabb(gen.Position()));
                Circle c = TestGeo.Q(gen.Circle(gen.Near(box.Center, FuzzGen.Reach(box) + gen.Size())));
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(c)}, {TestGeo.Dump(box)}";

                Manifold m = Collide.CircleVsAabb(c, box);
                if (!m.Colliding || m.Depth <= 2f / 256f) continue;

                // Only check when centre is outside the box (inside has face-ejection semantics)
                bool centreInside =
                    Math.Abs(c.Center.X - box.Center.X) <= Math.Abs(box.HalfExtents.X)
                    && Math.Abs(c.Center.Y - box.Center.Y) <= Math.Abs(box.HalfExtents.Y);
                if (centreInside) continue;

                (double ex, double ey) = TestGeo.ContactCircleAabb(c, box);
                double tol = ScaledTol(MathF.Abs(c.Radius));
                CheckContact(m, ex, ey, tol, repro);
                verified++;
            }
            Assert.True(verified > Iterations / 10,
                $"too few verified contacts ({verified}) at {regime}");
        }
    }

    [Theory]
    [InlineData(105)]
    [InlineData(106)]
    public void CircleVsCapsule_ContactMatchesOracle(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            int verified = 0;
            for (int i = 0; i < Iterations; i++)
            {
                Capsule cap = TestGeo.Q(gen.Capsule(gen.Position()));
                Circle c = TestGeo.Q(gen.Circle(gen.Near(cap.A, FuzzGen.Reach(cap) + gen.Size())));
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(c)}, {TestGeo.Dump(cap)}";

                Manifold m = Collide.CircleVsCapsule(c, cap);
                if (!m.Colliding || m.Depth <= 4f / 256f) continue;

                (double ex, double ey) = TestGeo.ContactCircleCapsule(c, cap);
                // The contact is the overlap midpoint (circle-circle reduction),
                // which sits ~depth/2 off the surface, so the tolerance carries a
                // depth term in addition to the size-relative rounding term.
                double tol = ScaledTol(MathF.Max(MathF.Abs(c.Radius), MathF.Abs(cap.Radius)))
                    + m.Depth * 0.6;
                CheckContact(m, ex, ey, tol, repro);
                verified++;
            }
            Assert.True(verified > Iterations / 8,
                $"too few verified contacts ({verified}) at {regime}");
        }
    }
}
