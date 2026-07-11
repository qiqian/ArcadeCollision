using System;
using ArcCollision.Tests.Support;
using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Validates swept tests against a sampled double-precision oracle:
///
///  1. No tunneling — if the oracle proves the mover definitely passes through
///     the target (a sample penetrates beyond the gray zone), the sweep must hit.
///  2. No phantom hits — if the oracle proves the path never comes within the
///     gray zone of touching, the sweep must miss.
///  3. TOI accuracy — the reported time, converted to distance along the motion,
///     must sit within a few grid cells of the oracle's first-touch time.
///
/// Sample counts adapt to shape thickness so thin fast crossings cannot slip
/// between samples: clearance is |motion|-Lipschitz in t.
/// </summary>
public class SweepAccuracyTests
{
    private const int Iterations = 1200;

    private static TestGeo.DShape Translate(TestGeo.DShape s, double dx, double dy)
    {
        var xs = new double[s.Count];
        var ys = new double[s.Count];
        for (int i = 0; i < s.Count; i++) { xs[i] = s.Xs[i] + dx; ys[i] = s.Ys[i] + dy; }
        return new TestGeo.DShape { Xs = xs, Ys = ys, R = s.R };
    }

    private sealed record OracleSweep(bool DefiniteHit, bool DefiniteMiss, double TouchTime);

    /// <summary>Sampled + bisected first-touch analysis of clearance(t).</summary>
    private static OracleSweep Analyze(
        TestGeo.DShape mover, TestGeo.DShape target, Vec2 motion, double grayZone)
    {
        double len = Math.Sqrt((double)motion.X * motion.X + (double)motion.Y * motion.Y);
        int samples = Math.Clamp((int)(len / Math.Max(grayZone, 1.0 / 256.0)) + 8, 64, 4096);

        double firstBelow = -1;
        double prevT = 0;
        double minClear = double.MaxValue;
        for (int i = 0; i <= samples; i++)
        {
            double t = (double)i / samples;
            double c = TestGeo.Clearance(Translate(mover, motion.X * t, motion.Y * t), target);
            if (c < minClear) minClear = c;
            if (c < -grayZone && firstBelow < 0)
            {
                firstBelow = t;
                break;
            }
            prevT = t;
        }

        if (firstBelow >= 0)
        {
            // Bisect the zero crossing between the last clear sample and the
            // first definitely-penetrating one.
            double lo = prevT, hi = firstBelow;
            for (int i = 0; i < 50; i++)
            {
                double mid = (lo + hi) * 0.5;
                double c = TestGeo.Clearance(Translate(mover, motion.X * mid, motion.Y * mid), target);
                if (c > 0) lo = mid; else hi = mid;
            }
            return new OracleSweep(true, false, (lo + hi) * 0.5);
        }

        // Definite miss requires margin covering the sampling gap:
        // |clearance(t±Δt/2)| can dip at most len·Δt/2 below the sample.
        double sampleGapSlack = len / samples * 0.5;
        bool definiteMiss = minClear > grayZone + sampleGapSlack;
        return new OracleSweep(false, definiteMiss, 1.0);
    }

    private static void CheckSweep(
        SweepHit hit, OracleSweep oracle, Vec2 motion, double grayZone, string repro)
    {
        double len = Math.Sqrt((double)motion.X * motion.X + (double)motion.Y * motion.Y);
        if (oracle.DefiniteHit)
        {
            Assert.True(hit.Hit, $"sweep missed a definite hit (touch t={oracle.TouchTime:F6}): {repro}");
            // Convert time error to distance along the motion.
            double posError = Math.Abs(hit.Time - oracle.TouchTime) * len;
            Assert.True(posError <= grayZone + 14.0 / 256.0,
                $"TOI off by {posError:F6} units (impl t={hit.Time:R}, oracle t={oracle.TouchTime:F6}): {repro}");
        }
        else if (oracle.DefiniteMiss)
        {
            Assert.True(!hit.Hit, $"phantom sweep hit at t={hit.Time:R}: {repro}");
        }
        // Otherwise: grazing band — either answer is acceptable.
    }

    [Theory]
    [InlineData(91, Regime.Origin)]
    [InlineData(92, Regime.Mid)]
    [InlineData(93, Regime.Far)]
    public void MovingCircleVsCircle_MatchesSampledOracle(int seed, Regime regime)
    {
        var gen = new FuzzGen(seed, regime);
        for (int i = 0; i < Iterations; i++)
        {
            Circle mover = TestGeo.Q(gen.Circle(gen.Position()));
            Circle target = TestGeo.Q(gen.Circle(gen.Near(mover.Center, gen.SizeMax * 2f)));
            Vec2 motion = TestGeo.Q(new Vec2(
                gen.NextFloat(-gen.SizeMax * 4, gen.SizeMax * 4),
                gen.NextFloat(-gen.SizeMax * 4, gen.SizeMax * 4)));
            string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(mover)} motion {TestGeo.Dump(motion)} vs {TestGeo.Dump(target)}";

            SweepHit hit = Sweep.MovingCircleVsCircle(mover, motion, target);
            OracleSweep oracle = Analyze(
                TestGeo.DShape.From(mover), TestGeo.DShape.From(target), motion, 3.0 / 256.0);
            CheckSweep(hit, oracle, motion, 3.0 / 256.0, repro);
        }
    }

    [Theory]
    [InlineData(101, Regime.Origin)]
    [InlineData(102, Regime.Mid)]
    [InlineData(103, Regime.Far)]
    public void MovingCircleVsAabb_MatchesSampledOracle(int seed, Regime regime)
    {
        var gen = new FuzzGen(seed, regime);
        for (int i = 0; i < Iterations; i++)
        {
            Aabb target = TestGeo.Q(gen.Aabb(gen.Position()));
            Circle mover = TestGeo.Q(gen.Circle(gen.Near(target.Center, FuzzGen.Reach(target) + gen.SizeMax)));
            Vec2 motion = TestGeo.Q(new Vec2(
                gen.NextFloat(-gen.SizeMax * 4, gen.SizeMax * 4),
                gen.NextFloat(-gen.SizeMax * 4, gen.SizeMax * 4)));
            string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(mover)} motion {TestGeo.Dump(motion)} vs {TestGeo.Dump(target)}";

            SweepHit hit = Sweep.MovingCircleVsAabb(mover, motion, target);
            OracleSweep oracle = Analyze(
                TestGeo.DShape.From(mover), TestGeo.DShape.From(target), motion, 4.0 / 256.0);
            CheckSweep(hit, oracle, motion, 4.0 / 256.0, repro);
        }
    }

    [Theory]
    [InlineData(111, Regime.Origin)]
    [InlineData(112, Regime.Mid)]
    public void MovingCircleVsCapsule_MatchesSampledOracle(int seed, Regime regime)
    {
        var gen = new FuzzGen(seed, regime);
        for (int i = 0; i < Iterations; i++)
        {
            Capsule target = TestGeo.Q(gen.Capsule(gen.Position()));
            Circle mover = TestGeo.Q(gen.Circle(gen.Near(target.A, FuzzGen.Reach(target) + gen.SizeMax)));
            Vec2 motion = TestGeo.Q(new Vec2(
                gen.NextFloat(-gen.SizeMax * 4, gen.SizeMax * 4),
                gen.NextFloat(-gen.SizeMax * 4, gen.SizeMax * 4)));
            string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(mover)} motion {TestGeo.Dump(motion)} vs {TestGeo.Dump(target)}";

            SweepHit hit = Sweep.MovingCircleVsCapsule(mover, motion, target);
            OracleSweep oracle = Analyze(
                TestGeo.DShape.From(mover), TestGeo.DShape.From(target), motion, 4.0 / 256.0);
            CheckSweep(hit, oracle, motion, 4.0 / 256.0, repro);
        }
    }

    [Theory]
    [InlineData(121, Regime.Origin)]
    [InlineData(122, Regime.Mid)]
    public void MovingShapeVsShape_BoxPairs_MatchSampledOracle(int seed, Regime regime)
    {
        var gen = new FuzzGen(seed, regime);
        for (int i = 0; i < Iterations / 2; i++)
        {
            Aabb mover = TestGeo.Q(gen.Aabb(gen.Position()));
            Obb target = TestGeo.Q(gen.Obb(gen.Near(mover.Center, FuzzGen.Reach(mover) + gen.SizeMax)));
            Vec2 motion = TestGeo.Q(new Vec2(
                gen.NextFloat(-gen.SizeMax * 4, gen.SizeMax * 4),
                gen.NextFloat(-gen.SizeMax * 4, gen.SizeMax * 4)));
            string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(mover)} motion {TestGeo.Dump(motion)} vs {TestGeo.Dump(target)}";

            SweepHit hit = Sweep.MovingShapeVsShape(new Shape(mover), motion, new Shape(target));
            OracleSweep oracle = Analyze(
                TestGeo.DShape.From(mover), TestGeo.DShape.From(target), motion, 6.0 / 256.0);
            CheckSweep(hit, oracle, motion, 6.0 / 256.0, repro);
        }
    }

    [Fact]
    public void FastThinCrossings_NeverTunnel_AtAnyWorldOffset()
    {
        // A razor-thin wall crossed by a fast mover — the classic tunneling
        // setup — replicated near the origin and at ±1.8M units.
        float[] offsets = { 0f, 1_800_000f, -1_800_000f };
        foreach (float off in offsets)
        {
            var wall = new Aabb(new Vec2(off, off), new Vec2(2f / 256f, 500f));
            var mover = new Circle(new Vec2(off - 900f, off + 10f), 1f / 256f);
            var motion = new Vec2(1800f, 0f);

            SweepHit hit = Sweep.MovingCircleVsAabb(mover, motion, wall);
            Assert.True(hit.Hit, $"tunneled through thin wall at offset {off}");
            Assert.True(hit.Time is > 0.49f and < 0.51f, $"TOI {hit.Time:R} out of band at offset {off}");

            SweepHit generic = Sweep.MovingShapeVsShape(
                new Shape(mover), motion, new Shape(wall));
            Assert.True(generic.Hit, $"generic sweep tunneled at offset {off}");
        }
    }

    [Fact]
    public void SweepEdgeCases_ZeroMotion_Tangent_StartTouching()
    {
        var circle = new Circle(new Vec2(0, 0), 1f);

        // Zero motion, separated → miss; zero motion, contained → immediate hit.
        Assert.False(Sweep.RayVsCircle(new Vec2(5, 0), Vec2.Zero, circle).Hit);
        SweepHit inside = Sweep.RayVsCircle(new Vec2(0.5f, 0), Vec2.Zero, circle);
        Assert.True(inside.Hit);
        Assert.Equal(0f, inside.Time);

        // Exact tangent ray (grazes y = 1): accepting or rejecting is legal,
        // but it must not crash and any hit must be within the motion range.
        SweepHit tangent = Sweep.RayVsCircle(new Vec2(-5f, 1f), new Vec2(10f, 0f), circle);
        if (tangent.Hit) Assert.InRange(tangent.Time, 0f, 1f);

        // Ray starting exactly on the surface, moving inward → t = 0.
        SweepHit onSurface = Sweep.RayVsCircle(new Vec2(-1f, 0f), new Vec2(2f, 0f), circle);
        Assert.True(onSurface.Hit);
        Assert.Equal(0f, onSurface.Time);

        // Motion ending exactly at the surface → hit at t = 1 (touch counts).
        SweepHit endsAtSurface = Sweep.RayVsCircle(new Vec2(-3f, 0f), new Vec2(2f, 0f), circle);
        Assert.True(endsAtSurface.Hit);
        Assert.Equal(1f, endsAtSurface.Time, 2f / 65536f);

        // Slab-parallel ray exactly on the box boundary must not divide by zero.
        var box = new Aabb(new Vec2(0, 0), new Vec2(1, 1));
        SweepHit parallel = Sweep.RayVsAabb(new Vec2(-5f, 1f), new Vec2(10f, 0f), box);
        if (parallel.Hit) Assert.InRange(parallel.Time, 0f, 1f);
    }
}
