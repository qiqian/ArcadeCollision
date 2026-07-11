using System;

namespace ArcCollision.Tests.Support;

/// <summary>Where in the world (and at what size) a fuzz case is generated.</summary>
public enum Regime
{
    Origin,     // positions within ±60, sizes 0.05..6 — sub-pixel behaviour
    Mid,        // positions within ±100k, sizes 0.5..300 — gameplay scale
    Far,        // positions within ±1.8M, sizes 1..2000 — large-world precision
}

/// <summary>
/// Deterministic shape generator. Every case is reproducible from (seed, index);
/// shapes are biased so roughly half of all pairs are near-touching, which is
/// where the interesting rounding behaviour lives.
/// </summary>
internal sealed class FuzzGen
{
    private readonly Random _rng;
    public Regime Regime { get; }

    public FuzzGen(int seed, Regime regime)
    {
        _rng = new Random(seed);
        Regime = regime;
    }

    public float PositionSpan => Regime switch
    {
        Regime.Origin => 60f,
        Regime.Mid => 100_000f,
        _ => 1_800_000f,
    };

    public float SizeMin => Regime switch
    {
        Regime.Origin => 0.05f,
        Regime.Mid => 0.5f,
        _ => 1f,
    };

    public float SizeMax => Regime switch
    {
        Regime.Origin => 6f,
        Regime.Mid => 300f,
        _ => 2000f,
    };

    public float NextFloat(float min, float max) =>
        min + (float)_rng.NextDouble() * (max - min);

    public int NextInt(int maxExclusive) => _rng.Next(maxExclusive);

    public Vec2 Position() =>
        new(NextFloat(-PositionSpan, PositionSpan), NextFloat(-PositionSpan, PositionSpan));

    /// <summary>A second position biased to be near-touching distance from the first.</summary>
    public Vec2 Near(Vec2 anchor, float touchDistance)
    {
        double angle = _rng.NextDouble() * Math.PI * 2;
        // Concentrate around the touch distance: ±15% plus a sub-pixel jitter band.
        float dist = _rng.Next(3) switch
        {
            0 => touchDistance * NextFloat(0.2f, 0.95f),           // clearly inside
            1 => touchDistance + NextFloat(-6f, 6f) * TestGeo.Grid, // grazing band
            _ => touchDistance * NextFloat(1.05f, 1.8f),           // clearly outside
        };
        return new Vec2(
            anchor.X + (float)Math.Cos(angle) * dist,
            anchor.Y + (float)Math.Sin(angle) * dist);
    }

    public float Size() => NextFloat(SizeMin, SizeMax);

    public Circle Circle(Vec2 at) => new(at, Size());

    public Aabb Aabb(Vec2 at) => new(at, new Vec2(Size(), Size()));

    public Capsule Capsule(Vec2 at)
    {
        // Include degenerate (A == B) spines ~10% of the time.
        if (_rng.Next(10) == 0)
            return new Capsule(at, at, Size());
        float half = Size();
        double angle = _rng.NextDouble() * Math.PI * 2;
        var d = new Vec2((float)Math.Cos(angle) * half, (float)Math.Sin(angle) * half);
        return new Capsule(at - d, at + d, Size() * 0.5f);
    }

    public Obb Obb(Vec2 at) =>
        new(at, new Vec2(Size(), Size() * NextFloat(0.2f, 1f)), NextFloat(-MathF.PI, MathF.PI));

    /// <summary>Approximate reach of a shape from its anchor, used for touch-distance biasing.</summary>
    public static float Reach(Circle c) => MathF.Abs(c.Radius);
    public static float Reach(Aabb b) => MathF.Max(MathF.Abs(b.HalfExtents.X), MathF.Abs(b.HalfExtents.Y));
    public static float Reach(Capsule c) =>
        (c.B - c.A).Length * 0.5f + MathF.Abs(c.Radius);
    public static float Reach(Obb o) => MathF.Max(MathF.Abs(o.HalfExtents.X), MathF.Abs(o.HalfExtents.Y)) * 1.5f;
}
