using System;

namespace ArcCollision.Wrapper;

/// <summary>
/// A placement of a collider's immutable base shape: the world position of
/// the shape's local origin, a rotation applied to its authored orientation, and
/// a uniform scale. Identity (position=0, rotation=0, scale=1) reproduces the
/// authored pose. Circles and axis-aligned boxes ignore rotation; OBB, capsule
/// and polygon respond to it. ArcWorld quantizes position to Q24.8 and scale to
/// Q16.16 before composing or materializing it. Most updates are translation only.
/// </summary>
public readonly struct Transform : IEquatable<Transform>
{
    public readonly Vec2 Position;
    public readonly Angle32 Rotation;
    public readonly float Scale;

    public Transform(Vec2 position, Angle32 rotation, float scale)
    {
        if (!float.IsFinite(scale) || scale < 0f)
            throw new ArgumentOutOfRangeException(nameof(scale), scale,
                "Scale must be finite and non-negative.");
        Position = position;
        Rotation = rotation;
        Scale = scale;
    }

    public Transform(Vec2 position) : this(position, new Angle32(0), 1f)
    {
    }

    public static Transform Identity => new(Vec2.Zero, new Angle32(0), 1f);

    /// <summary>The cheap path: no rotation and unit scale, so the collider just
    /// translates to <see cref="Position"/>.</summary>
    public bool IsTranslationOnly => Rotation.Raw == 0 && Scale == 1f;

    public bool Equals(Transform other) =>
        Position.Equals(other.Position) && Rotation.Raw == other.Rotation.Raw
        && Scale.Equals(other.Scale);

    public override bool Equals(object? obj) => obj is Transform other && Equals(other);
    public override int GetHashCode() => HashCode.Combine(Position, Rotation.Raw, Scale);
}
