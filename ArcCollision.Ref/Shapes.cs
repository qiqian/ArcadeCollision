using System;

namespace ArcCollision;

/// <summary>A circle defined by a center point and radius.</summary>
public readonly struct Circle
{
    public readonly Vec2 Center;
    public readonly float Radius;

    public Circle(Vec2 center, float radius)
    {
        Center = center;
        Radius = radius;
    }

    public Aabb Bounds => new(Center, new Vec2(Radius, Radius));
    public Circle Moved(Vec2 delta) => new(Center + delta, Radius);
}

/// <summary>
/// Axis-aligned bounding box stored as center + half-extents. This form keeps
/// the collision math branch-light and matches the C implementation.
/// </summary>
public readonly struct Aabb
{
    public readonly Vec2 Center;
    public readonly Vec2 HalfExtents;

    public Aabb(Vec2 center, Vec2 halfExtents)
    {
        Center = center;
        HalfExtents = halfExtents;
    }

    public Vec2 Min => new(Center.X - HalfExtents.X, Center.Y - HalfExtents.Y);
    public Vec2 Max => new(Center.X + HalfExtents.X, Center.Y + HalfExtents.Y);

    public static Aabb FromMinMax(Vec2 min, Vec2 max)
    {
        var center = (min + max) * 0.5f;
        var half = (max - min) * 0.5f;
        return new Aabb(center, half);
    }

    public Aabb Moved(Vec2 delta) => new(Center + delta, HalfExtents);

    /// <summary>Expand the box by <paramref name="amount"/> on every side.</summary>
    public Aabb Expanded(float amount) => new(Center, new Vec2(HalfExtents.X + amount, HalfExtents.Y + amount));

    /// <summary>Smallest box containing both inputs (broadphase union).</summary>
    public static Aabb Union(Aabb a, Aabb b) => FromMinMax(Vec2.Min(a.Min, b.Min), Vec2.Max(a.Max, b.Max));

    public bool Overlaps(Aabb b)
    {
        // Integer core: quantize to 24.8 fixed point and compare exactly.
        return Math.Abs(Fx.From(Center.X) - Fx.From(b.Center.X))
                <= Fx.From(HalfExtents.X) + Fx.From(b.HalfExtents.X)
            && Math.Abs(Fx.From(Center.Y) - Fx.From(b.Center.Y))
                <= Fx.From(HalfExtents.Y) + Fx.From(b.HalfExtents.Y);
    }
}

/// <summary>
/// A capsule: the set of points within <see cref="Radius"/> of the segment
/// A-B. Great for characters, sword swings and thick projectiles.
/// </summary>
public readonly struct Capsule
{
    public readonly Vec2 A;
    public readonly Vec2 B;
    public readonly float Radius;

    public Capsule(Vec2 a, Vec2 b, float radius)
    {
        A = a;
        B = b;
        Radius = radius;
    }

    public Aabb Bounds
    {
        get
        {
            var min = Vec2.Min(A, B) - new Vec2(Radius, Radius);
            var max = Vec2.Max(A, B) + new Vec2(Radius, Radius);
            return Aabb.FromMinMax(min, max);
        }
    }

    public Capsule Moved(Vec2 delta) => new(A + delta, B + delta, Radius);
}
