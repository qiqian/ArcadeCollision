using System;
using ArcCollision.Tests.Support;
using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Oracle differential tests for all collision pairs involving Polygon.
/// Uses the double-precision SAT oracle (TestGeo.Clearance with DShape.From(Polygon))
/// to verify the integer SAT implementation's boolean and depth accuracy.
/// </summary>
public class PolygonOracleTests
{
    private const int Iterations = 4000;
    private static readonly Regime[] Regimes = { Regime.Origin, Regime.Mid };
    private const double PolygonZone = 8.0 / 256.0;

    private static void CheckBoolean(
        bool actual, double clearance, double grayZone, string repro)
    {
        if (clearance < -grayZone && !actual)
            Assert.Fail($"missed collision (clearance {clearance:F6}): {repro}");
        if (clearance > grayZone && actual)
            Assert.Fail($"phantom collision (clearance {clearance:F6}): {repro}");
    }

    [Theory]
    [InlineData(81)]
    [InlineData(82)]
    public void CircleVsPolygon_AgreesWithOracle(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            for (int i = 0; i < Iterations; i++)
            {
                Polygon poly;
                try { poly = TestGeo.Q(gen.ConvexPolygon(gen.Position())); }
                catch (ArgumentException) { continue; }
                if (Tol.IsSliver(poly)) continue;
                Circle c = TestGeo.Q(gen.Circle(gen.Near(poly.Bounds.Center, FuzzGen.Reach(poly) + gen.Size())));
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(c)}, {TestGeo.Dump(poly)}";

                double clearance = TestGeo.Clearance(TestGeo.DShape.From(c), TestGeo.DShape.From(poly));
                Manifold m = Collide.ShapeVsShape(new Shape(c), new Shape(poly));
                bool overlaps = Collide.Overlaps(new Shape(c), new Shape(poly));

                double gray = Tol.SatGray(Tol.Extent(c) + Tol.Extent(poly));
                CheckBoolean(m.Colliding, clearance, gray, repro);
                CheckBoolean(overlaps, clearance, gray, repro);
                if (m.Colliding && clearance < -gray)
                    Assert.True(m.Depth >= 0f, $"negative depth: {repro}");
            }
        }
    }

    [Theory]
    [InlineData(83)]
    [InlineData(84)]
    public void AabbVsPolygon_AgreesWithOracle(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            for (int i = 0; i < Iterations; i++)
            {
                Polygon poly;
                try { poly = TestGeo.Q(gen.ConvexPolygon(gen.Position())); }
                catch (ArgumentException) { continue; }
                if (Tol.IsSliver(poly)) continue;
                Aabb box = TestGeo.Q(gen.Aabb(gen.Near(poly.Bounds.Center, FuzzGen.Reach(poly) + gen.Size())));
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(box)}, {TestGeo.Dump(poly)}";

                double clearance = TestGeo.Clearance(TestGeo.DShape.From(box), TestGeo.DShape.From(poly));
                Manifold m = Collide.ShapeVsShape(new Shape(box), new Shape(poly));
                bool overlaps = Collide.Overlaps(new Shape(box), new Shape(poly));

                double gray = Tol.SatGray(Tol.Extent(box) + Tol.Extent(poly));
                CheckBoolean(m.Colliding, clearance, gray, repro);
                CheckBoolean(overlaps, clearance, gray, repro);
                if (m.Colliding && clearance < -gray)
                    Assert.True(m.Depth >= 0f, $"negative depth: {repro}");
            }
        }
    }

    [Theory]
    [InlineData(85)]
    [InlineData(86)]
    public void CapsuleVsPolygon_AgreesWithOracle(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            for (int i = 0; i < Iterations; i++)
            {
                Polygon poly;
                try { poly = TestGeo.Q(gen.ConvexPolygon(gen.Position())); }
                catch (ArgumentException) { continue; }
                if (Tol.IsSliver(poly)) continue;
                Capsule cap = TestGeo.Q(gen.Capsule(gen.Near(poly.Bounds.Center, FuzzGen.Reach(poly) + gen.Size())));
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(cap)}, {TestGeo.Dump(poly)}";

                double clearance = TestGeo.Clearance(TestGeo.DShape.From(cap), TestGeo.DShape.From(poly));
                Manifold m = Collide.ShapeVsShape(new Shape(cap), new Shape(poly));
                bool overlaps = Collide.Overlaps(new Shape(cap), new Shape(poly));

                double gray = Tol.SatGray(Tol.Extent(cap) + Tol.Extent(poly));
                CheckBoolean(m.Colliding, clearance, gray, repro);
                CheckBoolean(overlaps, clearance, gray, repro);
                if (m.Colliding && clearance < -gray)
                    Assert.True(m.Depth >= 0f, $"negative depth: {repro}");
            }
        }
    }

    [Theory]
    [InlineData(87)]
    [InlineData(88)]
    public void ObbVsPolygon_AgreesWithOracle(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            for (int i = 0; i < Iterations; i++)
            {
                Polygon poly;
                try { poly = TestGeo.Q(gen.ConvexPolygon(gen.Position())); }
                catch (ArgumentException) { continue; }
                if (Tol.IsSliver(poly)) continue;
                Obb obb = TestGeo.Q(gen.Obb(gen.Near(poly.Bounds.Center, FuzzGen.Reach(poly) + gen.Size())));
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(obb)}, {TestGeo.Dump(poly)}";

                double clearance = TestGeo.Clearance(TestGeo.DShape.From(obb), TestGeo.DShape.From(poly));
                Manifold m = Collide.ShapeVsShape(new Shape(obb), new Shape(poly));
                bool overlaps = Collide.Overlaps(new Shape(obb), new Shape(poly));

                double gray = Tol.SatGray(Tol.Extent(obb) + Tol.Extent(poly));
                CheckBoolean(m.Colliding, clearance, gray, repro);
                CheckBoolean(overlaps, clearance, gray, repro);
                if (m.Colliding && clearance < -gray)
                    Assert.True(m.Depth >= 0f, $"negative depth: {repro}");
            }
        }
    }

    [Theory]
    [InlineData(89)]
    [InlineData(90)]
    public void PolygonVsPolygon_AgreesWithOracle(int seed)
    {
        foreach (Regime regime in Regimes)
        {
            var gen = new FuzzGen(seed * 100 + (int)regime, regime);
            for (int i = 0; i < Iterations; i++)
            {
                Polygon a, b;
                try { a = TestGeo.Q(gen.ConvexPolygon(gen.Position())); }
                catch (ArgumentException) { continue; }
                try { b = TestGeo.Q(gen.ConvexPolygon(gen.Near(a.Bounds.Center, FuzzGen.Reach(a) + gen.Size()))); }
                catch (ArgumentException) { continue; }
                if (Tol.IsSliver(a) || Tol.IsSliver(b)) continue;
                string repro = $"[{regime} seed={seed} i={i}] {TestGeo.Dump(a)}, {TestGeo.Dump(b)}";

                double clearance = TestGeo.Clearance(TestGeo.DShape.From(a), TestGeo.DShape.From(b));
                Manifold m = Collide.ShapeVsShape(new Shape(a), new Shape(b));
                bool overlaps = Collide.Overlaps(new Shape(a), new Shape(b));

                double gray = Tol.SatGray(Tol.Extent(a) + Tol.Extent(b));
                CheckBoolean(m.Colliding, clearance, gray, repro);
                CheckBoolean(overlaps, clearance, gray, repro);
                if (m.Colliding && clearance < -gray)
                    Assert.True(m.Depth >= 0f, $"negative depth: {repro}");
            }
        }
    }
}
