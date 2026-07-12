using System;
using ArcCollision.Tests.Support;
using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Randomized differential testing against the double-precision oracle. Inputs
/// are snapped to the 1/256 grid first so the oracle and the integer core see
/// identical geometry; agreement rules then depend on how much internal
/// rounding each pair's algorithm performs:
///
///  - circle/circle, circle/aabb: the integer predicate is exact → the boolean
///    must match the oracle with zero tolerance.
///  - capsule and OBB paths: 16.16 parameters and snapped closest points allow
///    a small gray zone around exact touch; outside it the boolean must match.
///
/// Every failure message contains a paste-ready repro.
/// </summary>
public class OracleDifferentialTests
{
    private const int Iterations = 4000;
    private static readonly Regime[] Regimes = { Regime.Origin, Regime.Mid, Regime.Far };

    // Gray zones (world units): bound on internal rounding of each algorithm.
    private const double ExactZone = 0.0;
    private const double CapsuleZone = 4.0 / 256.0;
    private const double BoxZone = 6.0 / 256.0;

    // Q1.30 axes make direction error negligible; retain only a tiny relative
    // term on top of the absolute 24.8 position/vertex quantization envelope.
    private static double Extent(Circle c) => Math.Abs(c.Radius);
    private static double Extent(Obb o) => Math.Abs(o.HalfExtents.X) + Math.Abs(o.HalfExtents.Y);
    private static double Extent(Capsule c) => (c.B - c.A).Length * 0.5 + Math.Abs(c.Radius);
    private static double SatGray(double extent) => 6.0 / 256.0 + extent * 0.0000001;

    private static void CheckBoolean(
        bool actual, double clearance, double grayZone, string repro)
    {
        if (clearance < -grayZone && !actual)
            Assert.Fail($"missed collision (clearance {clearance:F6}): {repro}");
        if (clearance > grayZone && actual)
            Assert.Fail($"phantom collision (clearance {clearance:F6}): {repro}");
    }

    private static void CheckDepth(
        Manifold m, double clearance, double grayZone, double depthTol, string repro)
    {
        if (!m.Colliding || clearance > -grayZone)
            return;   // only judge depth when the overlap itself is unambiguous
        Assert.True(m.Depth >= 0f, $"negative depth: {repro}");
        double expected = -clearance;
        if (Math.Abs(m.Depth - expected) > depthTol)
            Assert.Fail($"depth {m.Depth:F6} vs oracle {expected:F6}: {repro}");
        double lenSq = m.Normal.X * (double)m.Normal.X + m.Normal.Y * (double)m.Normal.Y;
        Assert.True(Math.Abs(Math.Sqrt(lenSq) - 1.0) < 2e-6,
            $"normal not unit ({Math.Sqrt(lenSq):F6}): {repro}");
    }

    [Theory]
    [InlineData(11)]
    [InlineData(12)]
    [InlineData(13)]
    public void CircleVsCircle_AgreesWithOracle(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            for (int i = 0; i < Iterations; i++)
            {
                Circle a = TestGeo.Q(gen.Circle(gen.Position()));
                Circle b = TestGeo.Q(gen.Circle(gen.Near(a.Center, FuzzGen.Reach(a) + gen.Size())));
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(a)}, {TestGeo.Dump(b)}";

                double clearance = TestGeo.Clearance(TestGeo.DShape.From(a), TestGeo.DShape.From(b));
                Manifold m = Collide.CircleVsCircle(a, b);

                // Exact predicate — also verify the touching boundary itself.
                bool expected = clearance <= 1e-9;
                if (Math.Abs(clearance) > 1e-7)
                    Assert.True(m.Colliding == expected, $"boolean mismatch (clearance {clearance:F8}): {repro}");
                CheckDepth(m, clearance, ExactZone + 1e-7, 2.5 / 256.0, repro);
            }
        }
    }

    [Theory]
    [InlineData(21)]
    [InlineData(22)]
    public void CircleVsAabb_AgreesWithOracle(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            for (int i = 0; i < Iterations; i++)
            {
                Aabb box = TestGeo.Q(gen.Aabb(gen.Position()));
                Circle c = TestGeo.Q(gen.Circle(gen.Near(box.Center, FuzzGen.Reach(box) + gen.Size())));
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(c)}, {TestGeo.Dump(box)}";

                double clearance = TestGeo.Clearance(TestGeo.DShape.From(c), TestGeo.DShape.From(box));
                Manifold m = Collide.CircleVsAabb(c, box);

                if (Math.Abs(clearance) > 1e-7)
                    Assert.True(m.Colliding == (clearance <= 0), $"boolean mismatch (clearance {clearance:F8}): {repro}");
                // Depth semantics differ when the centre is inside the box
                // (face ejection + radius), so only compare depth for shallow hits.
                if (clearance < 0 && -clearance < c.Radius)
                    CheckDepth(m, clearance, 1e-7, 2.5 / 256.0, repro);
            }
        }
    }

    [Theory]
    [InlineData(31)]
    [InlineData(32)]
    public void CircleVsCapsule_AgreesWithOracle(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            for (int i = 0; i < Iterations; i++)
            {
                Capsule cap = TestGeo.Q(gen.Capsule(gen.Position()));
                Circle c = TestGeo.Q(gen.Circle(gen.Near(cap.A, FuzzGen.Reach(cap) + gen.Size())));
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(c)}, {TestGeo.Dump(cap)}";

                double clearance = TestGeo.Clearance(TestGeo.DShape.From(c), TestGeo.DShape.From(cap));
                Manifold m = Collide.CircleVsCapsule(c, cap);

                CheckBoolean(m.Colliding, clearance, CapsuleZone, repro);
                CheckDepth(m, clearance, CapsuleZone, 6.0 / 256.0, repro);
            }
        }
    }

    [Theory]
    [InlineData(41)]
    [InlineData(42)]
    public void CapsuleVsCapsule_AgreesWithOracle(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            for (int i = 0; i < Iterations; i++)
            {
                Capsule a = TestGeo.Q(gen.Capsule(gen.Position()));
                Capsule b = TestGeo.Q(gen.Capsule(gen.Near(a.A, FuzzGen.Reach(a) + gen.Size())));
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(a)}, {TestGeo.Dump(b)}";

                double clearance = TestGeo.Clearance(TestGeo.DShape.From(a), TestGeo.DShape.From(b));
                Manifold m = Collide.CapsuleVsCapsule(a, b);

                CheckBoolean(m.Colliding, clearance, CapsuleZone, repro);
                // Depth mirrors the closest-point reduction only while the spines
                // do not cross; crossing spines make the reduction saturate. The
                // depth tolerance grows with radius (isqrt of a larger distance).
                double spineDistance = clearance + a.Radius + b.Radius;
                if (clearance < 0 && spineDistance > CapsuleZone)
                    CheckDepth(m, clearance, CapsuleZone,
                        6.0 / 256.0 + (a.Radius + b.Radius) * 0.0004, repro);
            }
        }
    }

    [Theory]
    [InlineData(51)]
    [InlineData(52)]
    public void CapsuleVsAabb_AgreesWithOracle(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            for (int i = 0; i < Iterations; i++)
            {
                Aabb box = TestGeo.Q(gen.Aabb(gen.Position()));
                Capsule cap = TestGeo.Q(gen.Capsule(gen.Near(box.Center, FuzzGen.Reach(box) + gen.Size())));
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(cap)}, {TestGeo.Dump(box)}";

                double clearance = TestGeo.Clearance(TestGeo.DShape.From(cap), TestGeo.DShape.From(box));
                bool overlap = Collide.Overlaps(new Shape(cap), new Shape(box));
                CheckBoolean(overlap, clearance, BoxZone, repro);

                Manifold m = Collide.CapsuleVsAabb(cap, box);
                CheckBoolean(m.Colliding, clearance, BoxZone, repro);
            }
        }
    }

    [Theory]
    [InlineData(61)]
    [InlineData(62)]
    public void CircleVsObb_AgreesWithOracle(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            for (int i = 0; i < Iterations; i++)
            {
                Obb box = TestGeo.Q(gen.Obb(gen.Position()));
                Circle c = TestGeo.Q(gen.Circle(gen.Near(box.Center, FuzzGen.Reach(box) + gen.Size())));
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(c)}, {TestGeo.Dump(box)}";

                // Dedicated oracle replicating the local-frame reduction, so the
                // gray zone stays a few grid cells at every world scale.
                double clearance = TestGeo.ClearanceCircleObb(c, box);
                Manifold m = Collide.ShapeVsShape(new Shape(c), new Shape(box));
                bool overlap = Collide.Overlaps(new Shape(c), new Shape(box));

                double gray = SatGray(Extent(c) + Extent(box));
                CheckBoolean(m.Colliding, clearance, gray, repro);
                CheckBoolean(overlap, clearance, gray, repro);
                Assert.True(m.Colliding == overlap
                    || Math.Abs(clearance) <= gray,
                    $"manifold/overlap disagree away from touch: {repro}");
            }
        }
    }

    [Theory]
    [InlineData(71)]
    [InlineData(72)]
    public void ObbVsObb_AgreesWithOracle(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            for (int i = 0; i < Iterations; i++)
            {
                Obb a = TestGeo.Q(gen.Obb(gen.Position()));
                Obb b = TestGeo.Q(gen.Obb(gen.Near(a.Center, FuzzGen.Reach(a) + gen.Size())));
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(a)}, {TestGeo.Dump(b)}";

                double clearance = TestGeo.Clearance(TestGeo.DShape.From(a), TestGeo.DShape.From(b));
                Manifold m = Collide.ShapeVsShape(new Shape(a), new Shape(b));

                // SAT boolean on quantized axes; corner-corner distance is not a
                // SAT axis, so allow the gray zone in both directions.
                double gray = SatGray(Extent(a) + Extent(b));
                CheckBoolean(m.Colliding, clearance, gray, repro);
                CheckDepth(m, clearance, gray, gray, repro);
            }
        }
    }
}
