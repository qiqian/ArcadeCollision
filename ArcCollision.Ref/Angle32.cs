using System;

namespace ArcCollision.Ref;

/// <summary>
/// Deterministic binary angle: one full turn maps to the complete UInt32 range.
/// Construct synchronized state from <see cref="Raw"/>; the radians conversion
/// exists for API compatibility at non-authoritative boundaries.
/// </summary>
public readonly struct Angle32 : IEquatable<Angle32>
{
    private const double UnitsPerRadian = 4294967296.0 / (Math.PI * 2.0);
    private const double RadiansPerUnit = (Math.PI * 2.0) / 4294967296.0;

    public readonly uint Raw;

    public Angle32(uint raw) => Raw = raw;

    public static Angle32 FromRadians(float radians)
    {
        if (!float.IsFinite(radians))
            throw new ArgumentOutOfRangeException(nameof(radians), radians,
                "Rotation must be finite.");
        double wrapped = radians % (Math.PI * 2.0);
        long units = (long)Math.Round(wrapped * UnitsPerRadian,
            MidpointRounding.ToEven);
        return new Angle32(unchecked((uint)units));
    }

    public float Radians => (float)(unchecked((int)Raw) * RadiansPerUnit);

    public static Angle32 operator -(Angle32 angle) =>
        new(unchecked(0u - angle.Raw));

    public bool Equals(Angle32 other) => Raw == other.Raw;
    public override bool Equals(object? obj) => obj is Angle32 other && Equals(other);
    public override int GetHashCode() => unchecked((int)Raw);
    public static bool operator ==(Angle32 left, Angle32 right) => left.Raw == right.Raw;
    public static bool operator !=(Angle32 left, Angle32 right) => left.Raw != right.Raw;
}
