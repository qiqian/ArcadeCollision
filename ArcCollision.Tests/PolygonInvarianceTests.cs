using ArcCollision.Tests.Support;
using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Metamorphic invariants for Polygon shapes. These test structural properties
/// that hold regardless of the specific geometry, ensuring the SAT
/// implementation maintains fundamental collision semantics.
///
/// Only convex polygons are tested here — concave polygons have triangulation-
/// dependent behaviour that breaks simple mirror/rotation invariants.
/// </summary>
public class PolygonInvarianceTests
{
    private const int Cases = 1500;

    private static readonly Vec2[] Offsets =
    {
        new(0f, 0f),
        new(1000.125f, -777.875f),
        new(250_000.5f, 99_999.25f),
        new(-1_200_000.25f, 900_000.125f),
        new(1_898_000f, -1_898_000f),
    };

    private static float S8(float v) => MathF.Round(v * 8f) / 8f;

    /// <summary>
    /// Generate a polygon vs other-shape pair for invariance testing.
    /// The polygon is always convex and snapped to 1/8 grid.
    /// </summary>
    private static (Shape polyShape, Shape other, string repro)? MakePair(Random rng, FuzzGen gen, int i)
    {
        Vec2 p = new(S8(rng.Next(-16000, 16000) / 8f), S8(rng.Next(-16000, 16000) / 8f));
        Polygon poly;
        try { poly = gen.ConvexPolygonGrid(p); }
        catch (ArgumentException) { return null; }

        float size = MathF.Max(0.125f, rng.Next(1, 960) / 8f);
        double ang = rng.NextDouble() * Math.PI * 2;
        float d = rng.Next(0, 2400) / 8f;
        Vec2 at = new(S8(p.X + (float)Math.Cos(ang) * d), S8(p.Y + (float)Math.Sin(ang) * d));

        int kind = rng.Next(5);
        Shape other;
        switch (kind)
        {
            case 0: other = new Circle(at, size); break;
            case 1: other = new Aabb(at, new Vec2(size, size)); break;
            case 2:
                Vec2 capEnd = new(S8(at.X + size), S8(at.Y + size * 0.5f));
                other = new Capsule(at, capEnd, size * 0.5f);
                break;
            case 3: other = new Obb(at, new Vec2(size, size * 0.5f), rng.Next(-314, 314) / 100f); break;
            default:
                try { other = gen.ConvexPolygonGrid(at); }
                catch (ArgumentException) { return null; }
                break;
        }

        string repro = $"case {i}: poly={TestGeo.Dump(poly)} vs {other.Kind}";
        return (poly, other, repro);
    }

    [Fact]
    public void Polygon_TranslationInvariance_BitExactAcrossTheWorld()
    {
        var rng = new Random(901);
        var gen = new FuzzGen(901, Regime.Origin);
        int collidingChecked = 0;

        for (int i = 0; i < Cases; i++)
        {
            var pair = MakePair(rng, gen, i);
            if (pair == null) continue;
            var (polyShape, other, repro) = pair.Value;

            Manifold baseline = Collide.ShapeVsShape(polyShape, other);

            foreach (Vec2 offset in Offsets)
            {
                Manifold moved = Collide.ShapeVsShape(
                    polyShape.Moved(offset), other.Moved(offset));

                Assert.True(baseline.Colliding == moved.Colliding,
                    $"colliding changed at offset {TestGeo.Dump(offset)}: {repro}");
                if (!baseline.Colliding) continue;

                Assert.True(baseline.Depth == moved.Depth,
                    $"depth {baseline.Depth:R} -> {moved.Depth:R} at {TestGeo.Dump(offset)}: {repro}");
                Assert.True(baseline.Normal.X == moved.Normal.X && baseline.Normal.Y == moved.Normal.Y,
                    $"normal changed at {TestGeo.Dump(offset)}: {repro}");
            }
            if (baseline.Colliding) collidingChecked++;
        }

        Assert.True(collidingChecked > Cases / 8,
            $"suspiciously few colliding cases ({collidingChecked}) — generator broken?");
    }

    [Fact]
    public void Polygon_SwapSymmetry_DepthEqualNormalOpposed()
    {
        var rng = new Random(902);
        var gen = new FuzzGen(902, Regime.Origin);

        for (int i = 0; i < Cases; i++)
        {
            var pair = MakePair(rng, gen, i);
            if (pair == null) continue;
            var (polyShape, other, repro) = pair.Value;

            Manifold ab = Collide.ShapeVsShape(polyShape, other);
            Manifold ba = Collide.ShapeVsShape(other, polyShape);

            bool grazing = (ab.Colliding && ab.Depth <= 2f / 256f)
                || (ba.Colliding && ba.Depth <= 2f / 256f);
            if (ab.Colliding != ba.Colliding)
            {
                Assert.True(grazing, $"swap asymmetry beyond grazing: {repro}");
                continue;
            }
            if (!ab.Colliding) continue;

            Assert.True(MathF.Abs(ab.Depth - ba.Depth) <= 2f / 256f,
                $"swap depth mismatch {ab.Depth:R} vs {ba.Depth:R}: {repro}");
            if (ab.Depth > 4f / 256f)
            {
                float dot = ab.Normal.X * ba.Normal.X + ab.Normal.Y * ba.Normal.Y;
                Assert.True(dot <= -(1f - 6f / 256f),
                    $"swap normals not opposed (dot {dot:R}): {repro}");
            }
        }
    }

    [Fact]
    public void Polygon_SeparationVector_ResolvesTheOverlap()
    {
        var rng = new Random(903);
        var gen = new FuzzGen(903, Regime.Origin);
        int resolved = 0;

        for (int i = 0; i < Cases; i++)
        {
            var pair = MakePair(rng, gen, i);
            if (pair == null) continue;
            var (polyShape, other, repro) = pair.Value;

            Manifold m = Collide.ShapeVsShape(polyShape, other);
            if (!m.Colliding || m.Depth <= 0f) continue;

            Shape moved = polyShape.Moved(
                m.SeparationForA - m.Normal * (2f / 256f));
            Manifold after = Collide.ShapeVsShape(moved, other);
            Assert.True(!after.Colliding,
                $"separation did not resolve overlap (depth {m.Depth:R} -> {after.Depth:R}): {repro}");
            resolved++;
        }

        Assert.True(resolved > Cases / 8, $"too few colliding polygon cases resolved ({resolved})");
    }

    [Fact]
    public void Polygon_OverlapsAgreesWithManifold()
    {
        var rng = new Random(904);
        var gen = new FuzzGen(904, Regime.Origin);

        for (int i = 0; i < Cases; i++)
        {
            var pair = MakePair(rng, gen, i);
            if (pair == null) continue;
            var (polyShape, other, repro) = pair.Value;

            Manifold m = Collide.ShapeVsShape(polyShape, other);
            bool overlaps = Collide.Overlaps(polyShape, other);

            if (m.Colliding && m.Depth > 4f / 256f)
                Assert.True(overlaps, $"Overlaps missed deep collision (depth {m.Depth:R}): {repro}");
            if (!overlaps)
                Assert.True(!m.Colliding || m.Depth <= 4f / 256f,
                    $"manifold reports collision but Overlaps does not (depth {m.Depth:R}): {repro}");
        }
    }
}
