using System;
using System.Runtime.CompilerServices;

namespace ArcCollision;

/// <summary>
/// A 2D vector using single-precision floats.
/// The reference library mirrors the memory layout and semantics of the C
/// implementation (<c>arc_vec2</c>) so results can be compared 1:1.
/// </summary>
public readonly struct Vec2 : IEquatable<Vec2>
{
    public readonly float X;
    public readonly float Y;

    public Vec2(float x, float y)
    {
        X = x;
        Y = y;
    }

    public static readonly Vec2 Zero = new(0f, 0f);
    public static readonly Vec2 UnitX = new(1f, 0f);
    public static readonly Vec2 UnitY = new(0f, 1f);

    public static Vec2 operator +(Vec2 a, Vec2 b) => new(a.X + b.X, a.Y + b.Y);
    public static Vec2 operator -(Vec2 a, Vec2 b) => new(a.X - b.X, a.Y - b.Y);
    public static Vec2 operator -(Vec2 a) => new(-a.X, -a.Y);
    public static Vec2 operator *(Vec2 a, float s) => new(a.X * s, a.Y * s);
    public static Vec2 operator *(float s, Vec2 a) => new(a.X * s, a.Y * s);
    public static Vec2 operator /(Vec2 a, float s) => new(a.X / s, a.Y / s);

    public float Dot(Vec2 b) => X * b.X + Y * b.Y;

    /// <summary>2D scalar cross product (z-component of the 3D cross).</summary>
    public float Cross(Vec2 b) => X * b.Y - Y * b.X;

    public float LengthSquared => X * X + Y * Y;

    public float Length => MathF.Sqrt(X * X + Y * Y);

    /// <summary>Left-hand perpendicular (rotate +90 degrees).</summary>
    public Vec2 Perp => new(-Y, X);

    public float DistanceSquared(Vec2 b)
    {
        float dx = X - b.X;
        float dy = Y - b.Y;
        return dx * dx + dy * dy;
    }

    public float Distance(Vec2 b) => MathF.Sqrt(DistanceSquared(b));

    /// <summary>
    /// Returns the unit-length version of this vector. For a (near) zero
    /// vector the supplied fallback is returned instead so callers never get
    /// a NaN direction.
    /// </summary>
    public Vec2 Normalized(Vec2 fallback = default)
    {
        float lenSq = X * X + Y * Y;
        if (lenSq < 1e-12f)
            return fallback;
        float inv = 1f / MathF.Sqrt(lenSq);
        return new Vec2(X * inv, Y * inv);
    }

    public static Vec2 Lerp(Vec2 a, Vec2 b, float t) => new(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec2 Min(Vec2 a, Vec2 b) => new(MathF.Min(a.X, b.X), MathF.Min(a.Y, b.Y));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec2 Max(Vec2 a, Vec2 b) => new(MathF.Max(a.X, b.X), MathF.Max(a.Y, b.Y));

    public bool Equals(Vec2 other) => X.Equals(other.X) && Y.Equals(other.Y);
    public override bool Equals(object? obj) => obj is Vec2 v && Equals(v);
    public override int GetHashCode() =>
        DeterministicHash.Combine(
            DeterministicHash.Float(X), DeterministicHash.Float(Y));
    public override string ToString() => $"({X:0.###}, {Y:0.###})";
}
