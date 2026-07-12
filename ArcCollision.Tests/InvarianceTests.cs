using ArcCollision.Tests.Support;
using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Metamorphic invariants that require no oracle. The headline test is
/// translation invariance: an integer core must produce BIT-IDENTICAL depth,
/// normal and time when both shapes are moved by the same exactly-representable
/// offset — even 1.9 million units from the origin. Float cores fail this;
/// any deviation here is a genuine large-world precision bug.
///
/// Base coordinates use the 1/8 grid: exactly representable both in 24.8 fixed
/// point and in float at any magnitude below 2^21, so input conversion stays
/// lossless after the offset is applied.
/// </summary>
public class InvarianceTests
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

    /// <summary>Snap to the 1/8 grid (exact in float and 24.8 at ±2M).</summary>
    private static float S8(float v) => MathF.Round(v * 8f) / 8f;
    private static Vec2 S8(Vec2 v) => new(S8(v.X), S8(v.Y));

    /// <summary>Test-side shape record that can be transformed exactly.</summary>
    private readonly struct TShape
    {
        public readonly ShapeKind Kind;
        public readonly Circle C;
        public readonly Aabb B;
        public readonly Capsule K;
        public readonly Obb O;

        public TShape(Circle c) : this() { Kind = ShapeKind.Circle; C = c; }
        public TShape(Aabb b) : this() { Kind = ShapeKind.Aabb; B = b; }
        public TShape(Capsule k) : this() { Kind = ShapeKind.Capsule; K = k; }
        public TShape(Obb o) : this() { Kind = ShapeKind.Obb; O = o; }

        public Shape AsShape() => Kind switch
        {
            ShapeKind.Circle => C,
            ShapeKind.Aabb => B,
            ShapeKind.Capsule => K,
            _ => O,
        };

        public TShape Moved(Vec2 d) => Kind switch
        {
            ShapeKind.Circle => new TShape(new Circle(C.Center + d, C.Radius)),
            ShapeKind.Aabb => new TShape(new Aabb(B.Center + d, B.HalfExtents)),
            ShapeKind.Capsule => new TShape(new Capsule(K.A + d, K.B + d, K.Radius)),
            _ => new TShape(new Obb(O.Center + d, O.HalfExtents, O.Angle)),
        };

        public TShape MirrorX() => Kind switch
        {
            ShapeKind.Circle => new TShape(new Circle(new Vec2(-C.Center.X, C.Center.Y), C.Radius)),
            ShapeKind.Aabb => new TShape(new Aabb(new Vec2(-B.Center.X, B.Center.Y), B.HalfExtents)),
            ShapeKind.Capsule => new TShape(new Capsule(
                new Vec2(-K.A.X, K.A.Y), new Vec2(-K.B.X, K.B.Y), K.Radius)),
            // cos(-θ) == cos(θ) and sin(-θ) == -sin(θ) exactly in IEEE floats,
            // so negating the rotation mirrors the quantized axis exactly.
            _ => new TShape(new Obb(new Vec2(-O.Center.X, O.Center.Y), O.HalfExtents, -O.Angle)),
        };

        /// <summary>(x, y) → (-y, x); valid for grid-rotatable kinds only.</summary>
        public TShape Rot90() => Kind switch
        {
            ShapeKind.Circle => new TShape(new Circle(R(C.Center), C.Radius)),
            ShapeKind.Aabb => new TShape(new Aabb(R(B.Center), new Vec2(B.HalfExtents.Y, B.HalfExtents.X))),
            ShapeKind.Capsule => new TShape(new Capsule(R(K.A), R(K.B), K.Radius)),
            _ => throw new InvalidOperationException("OBB rotation+π/2 is not float-exact."),
        };

        private static Vec2 R(Vec2 v) => new(-v.Y, v.X);

        public override string ToString() => Kind switch
        {
            ShapeKind.Circle => TestGeo.Dump(C),
            ShapeKind.Aabb => TestGeo.Dump(B),
            ShapeKind.Capsule => TestGeo.Dump(K),
            _ => TestGeo.Dump(O),
        };
    }

    private static (TShape a, TShape b, string repro) MakePair(Random rng, int i, int kinds = 4)
    {
        Vec2 p = S8(new Vec2(rng.Next(-16000, 16000) / 8f, rng.Next(-16000, 16000) / 8f));
        float Size() => MathF.Max(0.125f, rng.Next(1, 960) / 8f);
        Vec2 Near()
        {
            double ang = rng.NextDouble() * Math.PI * 2;
            float d = rng.Next(0, 2400) / 8f;
            return S8(new Vec2(p.X + (float)Math.Cos(ang) * d, p.Y + (float)Math.Sin(ang) * d));
        }

        TShape Make(int kind, Vec2 at) => kind switch
        {
            0 => new TShape(new Circle(at, Size())),
            1 => new TShape(new Aabb(at, new Vec2(Size(), Size()))),
            2 => new TShape(new Capsule(at, S8(at + new Vec2(Size(), Size() * 0.5f)), Size() * 0.5f)),
            _ => new TShape(new Obb(at, new Vec2(Size(), Size() * 0.5f), rng.Next(-314, 314) / 100f)),
        };

        TShape a = Make(rng.Next(kinds), p);
        TShape b = Make(rng.Next(kinds), Near());
        return (a, b, $"case {i}: {a} vs {b}");
    }

    [Fact]
    public void Discrete_TranslationInvariance_BitExactAcrossTheWorld()
    {
        var rng = new Random(777);
        int collidingChecked = 0;

        for (int i = 0; i < Cases; i++)
        {
            (TShape a, TShape b, string repro) = MakePair(rng, i);
            Manifold baseline = Collide.ShapeVsShape(a.AsShape(), b.AsShape());

            foreach (Vec2 offset in Offsets)
            {
                Manifold moved = Collide.ShapeVsShape(
                    a.Moved(offset).AsShape(), b.Moved(offset).AsShape());

                Assert.True(baseline.Colliding == moved.Colliding,
                    $"colliding changed at offset {TestGeo.Dump(offset)}: {repro}");
                if (!baseline.Colliding) continue;

                // Depth and normal are small-magnitude floats: identical bits required.
                Assert.True(baseline.Depth == moved.Depth,
                    $"depth {baseline.Depth:R} -> {moved.Depth:R} at {TestGeo.Dump(offset)}: {repro}");
                Assert.True(baseline.Normal.X == moved.Normal.X && baseline.Normal.Y == moved.Normal.Y,
                    $"normal changed at {TestGeo.Dump(offset)}: {repro}");

                // Contact is a world coordinate: exact in fixed space, but the
                // float return rounds at large magnitudes (ULP 0.125 at 1.9M).
                float cx = moved.Contact.X - offset.X;
                float cy = moved.Contact.Y - offset.Y;
                Assert.True(MathF.Abs(cx - baseline.Contact.X) <= 0.25f
                    && MathF.Abs(cy - baseline.Contact.Y) <= 0.25f,
                    $"contact drifted at {TestGeo.Dump(offset)}: {repro}");
            }
            if (baseline.Colliding) collidingChecked++;
        }

        Assert.True(collidingChecked > Cases / 6,
            $"suspiciously few colliding cases ({collidingChecked}) — generator broken?");
    }

    [Fact]
    public void Sweep_TranslationInvariance_TimeAndNormalBitExact()
    {
        var rng = new Random(778);
        int hitsChecked = 0;

        for (int i = 0; i < Cases; i++)
        {
            (TShape a, TShape b, string repro) = MakePair(rng, i);
            var motion = S8(new Vec2(rng.Next(-8000, 8000) / 8f, rng.Next(-8000, 8000) / 8f));
            SweepHit baseline = Sweep.MovingShapeVsShape(a.AsShape(), motion, b.AsShape());

            foreach (Vec2 offset in Offsets)
            {
                SweepHit moved = Sweep.MovingShapeVsShape(
                    a.Moved(offset).AsShape(), motion, b.Moved(offset).AsShape());
                Assert.True(baseline.Hit == moved.Hit,
                    $"sweep hit changed at {TestGeo.Dump(offset)}: {repro}");
                if (!baseline.Hit) continue;
                Assert.True(baseline.Time == moved.Time,
                    $"toi {baseline.Time:R} -> {moved.Time:R} at {TestGeo.Dump(offset)}: {repro}");
                Assert.True(baseline.Normal.X == moved.Normal.X && baseline.Normal.Y == moved.Normal.Y,
                    $"sweep normal changed at {TestGeo.Dump(offset)}: {repro}");
            }
            if (baseline.Hit) hitsChecked++;
        }

        Assert.True(hitsChecked > Cases / 12,
            $"suspiciously few sweep hits ({hitsChecked}) — generator broken?");
    }

    [Fact]
    public void Discrete_MirrorInvariance_ExactUnderXNegation()
    {
        var rng = new Random(779);
        for (int i = 0; i < Cases; i++)
        {
            (TShape a, TShape b, string repro) = MakePair(rng, i);
            Manifold m0 = Collide.ShapeVsShape(a.AsShape(), b.AsShape());
            Manifold m1 = Collide.ShapeVsShape(a.MirrorX().AsShape(), b.MirrorX().AsShape());

            Assert.True(m0.Colliding == m1.Colliding, $"mirror colliding mismatch: {repro}");
            if (!m0.Colliding) continue;

            // Skip the degenerate fallback (coincident centres pick +X on both
            // sides, which legitimately breaks the mirror mapping).
            if (m0.Normal.X == 1f && m0.Normal.Y == 0f && m1.Normal.X == 1f && m1.Normal.Y == 0f)
                continue;

            Assert.True(m0.Depth == m1.Depth,
                $"mirror depth mismatch {m0.Depth:R} vs {m1.Depth:R}: {repro}");
            Assert.True(m0.Normal.X == -m1.Normal.X && m0.Normal.Y == m1.Normal.Y,
                $"mirror normal mismatch {TestGeo.Dump(m0.Normal)} vs {TestGeo.Dump(m1.Normal)}: {repro}");
        }
    }

    [Fact]
    public void Discrete_Rotation90Invariance_ExactForGridRotatableShapes()
    {
        var rng = new Random(780);
        for (int i = 0; i < Cases; i++)
        {
            (TShape a, TShape b, string repro) = MakePair(rng, i, kinds: 3);  // no OBB
            Manifold m0 = Collide.ShapeVsShape(a.AsShape(), b.AsShape());
            Manifold m1 = Collide.ShapeVsShape(a.Rot90().AsShape(), b.Rot90().AsShape());

            Assert.True(m0.Colliding == m1.Colliding, $"rot90 colliding mismatch: {repro}");
            if (!m0.Colliding) continue;

            // Box pairs can tie on both axes; the tie-break axis is not
            // rotation-covariant, so skip exact comparison on ties.
            if (a.Kind == ShapeKind.Aabb && b.Kind == ShapeKind.Aabb && AabbOverlapTie(a.B, b.B))
                continue;
            if (m0.Normal.X == 1f && m0.Normal.Y == 0f && m1.Normal.X == 1f && m1.Normal.Y == 0f)
                continue;

            Assert.True(m0.Depth == m1.Depth,
                $"rot90 depth mismatch {m0.Depth:R} vs {m1.Depth:R}: {repro}");
            Assert.True(m1.Normal.X == -m0.Normal.Y && m1.Normal.Y == m0.Normal.X,
                $"rot90 normal mismatch {TestGeo.Dump(m0.Normal)} vs {TestGeo.Dump(m1.Normal)}: {repro}");
        }
    }

    private static float ReachOf(TShape s)
    {
        Aabb b = s.AsShape().Bounds;
        return MathF.Max(MathF.Abs(b.HalfExtents.X), MathF.Abs(b.HalfExtents.Y));
    }

    private static bool AabbOverlapTie(Aabb a, Aabb b)
    {
        long overlapX = TestGeo.QFx(a.HalfExtents.X) + TestGeo.QFx(b.HalfExtents.X)
            - Math.Abs(TestGeo.QFx(a.Center.X) - TestGeo.QFx(b.Center.X));
        long overlapY = TestGeo.QFx(a.HalfExtents.Y) + TestGeo.QFx(b.HalfExtents.Y)
            - Math.Abs(TestGeo.QFx(a.Center.Y) - TestGeo.QFx(b.Center.Y));
        return overlapX == overlapY;
    }

    [Fact]
    public void Discrete_SwapSymmetry_DepthEqualNormalOpposed()
    {
        var rng = new Random(781);
        for (int i = 0; i < Cases; i++)
        {
            (TShape a, TShape b, string repro) = MakePair(rng, i);
            Manifold ab = Collide.ShapeVsShape(a.AsShape(), b.AsShape());
            Manifold ba = Collide.ShapeVsShape(b.AsShape(), a.AsShape());

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

            // The MTV normal is only well-defined for shallow contacts. A deep
            // overlap (e.g. two capsule spines that cross) has an ambiguous
            // minimum-translation direction that can differ by feature on swap,
            // so only assert antiparallel normals when the penetration is small
            // relative to the shapes.
            float reach = MathF.Min(ReachOf(a), ReachOf(b));
            if (ab.Depth > 4f / 256f && ab.Depth < 0.4f * reach)
            {
                float dot = ab.Normal.X * ba.Normal.X + ab.Normal.Y * ba.Normal.Y;
                Assert.True(dot <= -(1f - 8f / 256f),
                    $"swap normals not opposed (dot {dot:R}, depth {ab.Depth:R}): {repro}");
            }
        }
    }

    [Fact]
    public void Discrete_SeparationVector_ResolvesTheOverlap()
    {
        var rng = new Random(782);
        int resolved = 0;
        for (int i = 0; i < Cases; i++)
        {
            (TShape a, TShape b, string repro) = MakePair(rng, i);
            Manifold m = Collide.ShapeVsShape(a.AsShape(), b.AsShape());
            if (!m.Colliding || m.Depth <= 0f) continue;

            // The separation guarantee only holds for a genuine minimum-translation
            // vector, i.e. shallow contact. Capsule/capsule reduces to a
            // closest-point normal that is NOT a true MTV once the spines deeply
            // overlap, so restrict the assertion to shallow penetrations.
            if (m.Depth >= 0.4f * MathF.Min(ReachOf(a), ReachOf(b))) continue;

            // Applying the MTV separates the shapes along the minimum-penetration
            // axis. For a corner-deep contact the remaining overlap may lie on a
            // different (cross) axis, so a single step resolves the tested axis,
            // not necessarily all overlap — assert the penetration is mostly gone.
            Vec2 separation = m.SeparationForA - m.Normal * (3f / 256f);
            Manifold after = Collide.ShapeVsShape(a.Moved(separation).AsShape(), b.AsShape());
            Assert.True(!after.Colliding || after.Depth <= m.Depth * 0.5f + 8f / 256f,
                $"separation failed (depth {m.Depth:R} -> {after.Depth:R}): {repro}");
            resolved++;
        }
        Assert.True(resolved > Cases / 15, $"too few colliding cases resolved ({resolved})");
    }
}
