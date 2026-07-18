using System;
using System.Numerics;
using Xunit;

namespace ArcCollision.Ref;

/// <summary>
/// Internal fixed-point layer for the integer collision core.
///
/// Spatial quantities (positions, radii, depths) use 24.8 fixed point: floats
/// crossing the public API boundary are multiplied by 256 on the way in and
/// divided by 256 on the way out. Dimensionless parameters (segment/sweep
/// times) use 16.16 so a time like 0.4 survives the round trip with ~1.5e-5
/// error instead of 1/256.
///
/// Products of two fixed values carry scale 2^16 ("squared" scale). Degree-four
/// expressions are scaled down before multiplication so the hot path stays in
/// 64-bit integer arithmetic.
/// </summary>
internal static class Fx
{
    public const int Shift = 8;
    public const long One = 1L << Shift;              // 256: one world unit
    public const int TShift = 16;
    public const long TOne = 1L << TShift;            // 65536: parameter 1.0

    // Keeps every derived delta, squared length and radius sum inside Int64.
    // Public coordinates remain usable up to roughly +/-1.95 million units.
    private const long MaxInputRaw = 500_000_000L;
    public const float MaxInput = CollisionLimits.MaxCoordinate;

    public static long From(float v)
    {
        if (!float.IsFinite(v) || v < -MaxInput || v > MaxInput)
            throw new ArgumentOutOfRangeException(nameof(v),
                $"Fixed-point input must be finite and within +/-{MaxInput}.");
        return (long)MathF.Round(v * One);
    }
    public static float To(long fx) => fx / (float)One;

    /// <summary>Convert a 16.16 parameter back to float.</summary>
    public static float ToT(long t16) => t16 / (float)TOne;

    /// <summary>Convert a squared fixed quantity (scale 2^16) back to float.</summary>
    public static float ToSq(long sq) => sq / (float)(One * One);

    /// <summary>Rounded division, symmetric around zero.</summary>
    public static long RoundDiv(long a, long b)
    {
        if (b == 0) throw new DivideByZeroException();
        if (b < 0) { a = -a; b = -b; }
        long half = b >> 1;
        return a >= 0 ? (a + half) / b : -((-a + half) / b);
    }

    /// <summary>Floor division (toward negative infinity), for grid bucketing.</summary>
    public static long FloorDiv(long a, long b)
    {
        if (b == 0) throw new DivideByZeroException();
        long q = a / b;
        if ((a ^ b) < 0 && a % b != 0) q--;
        return q;
    }

    public static long CeilDivPositive(long numerator, long denominator)
    {
        if (numerator < 0 || denominator <= 0)
            throw new ArgumentOutOfRangeException();
        return numerator == 0 ? 0 : 1 + (numerator - 1) / denominator;
    }

    /// <summary>(a * t16) &gt;&gt; 16 with rounding — apply a 16.16 parameter to a fixed value.</summary>
    public static long MulT(long a, long t16) => RoundShift(a * t16, TShift);

    /// <summary>
    /// Clamped 16.16 parameter num/den where both carry the same scale.
    /// </summary>
    public static long ClampedParam(long num, long den)
    {
        if (den <= 0) throw new ArgumentOutOfRangeException(nameof(den));
        if (num <= 0) return 0;
        if (num >= den) return TOne;

        ulong remainder = (ulong)num;
        ulong divisor = (ulong)den;
        long result = 0;
        for (int i = 0; i < TShift; i++)
        {
            result <<= 1;
            // Equivalent to remainder *= 2, without overflowing UInt64.
            if (remainder >= divisor - remainder)
            {
                remainder -= divisor - remainder;
                result |= 1;
            }
            else
            {
                remainder += remainder;
            }
        }
        return result;
    }

    public static long RatioT(long num, long den)
    {
        if (den == 0) throw new DivideByZeroException();
        bool negative = (num < 0) != (den < 0);
        ulong numerator = Magnitude(num);
        ulong denominator = Magnitude(den);
        ulong whole = numerator / denominator;
        ulong remainder = numerator % denominator;
        ulong fraction = 0;
        for (int i = 0; i < TShift; i++)
        {
            fraction <<= 1;
            if (remainder >= denominator - remainder)
            {
                remainder -= denominator - remainder;
                fraction |= 1;
            }
            else
            {
                remainder += remainder;
            }
        }
        long result = checked((long)(whole * (ulong)TOne + fraction));
        return negative ? -result : result;
    }

    /// <summary>(unit * fx) &gt;&gt; 8 with rounding — scale a fixed value by a 24.8 unit component.</summary>
    public static long MulUnit(long unit, long fx) => RoundShift(unit * fx, Shift);

    private static long RoundShift(long v, int shift)
    {
        long half = 1L << (shift - 1);
        return v >= 0 ? (v + half) >> shift : -((-v + half) >> shift);
    }

    /// <summary>
    /// Common right shift that keeps the difference of two products in Int64.
    /// Inputs are reduced to at most 30 magnitude bits.
    /// </summary>
    public static int ProductShift(long a, long b, long c) =>
        ProductShiftFromMax(Math.Max(Magnitude(a), Math.Max(Magnitude(b), Magnitude(c))));

    public static int ProductShift(long a, long b, long c, long d, long e) =>
        ProductShiftFromMax(Math.Max(
            Math.Max(Magnitude(a), Magnitude(b)),
            Math.Max(Magnitude(c), Math.Max(Magnitude(d), Magnitude(e)))));

    private static int ProductShiftFromMax(ulong max)
    {
        int bits = max == 0 ? 0 : BitOperations.Log2(max) + 1;
        return Math.Max(0, bits - 30);
    }

    internal static ulong Magnitude(long value) =>
        value < 0 ? (ulong)(-(value + 1)) + 1 : (ulong)value;

    public static long ScaleProductOperand(long value, int shift) =>
        shift == 0 ? value : value >> shift;

    /// <summary>Integer square root (restoring method). sqrt of a squared-scale
    /// (2^16) value yields a fixed-scale (2^8) value.</summary>
    public static long Sqrt(long v)
    {
        if (v <= 0) return 0;
        ulong x = (ulong)v;
        ulong r = 0;
        ulong bit = 1UL << 62;
        while (bit > x) bit >>= 2;
        while (bit != 0)
        {
            if (x >= r + bit) { x -= r + bit; r = (r >> 1) + bit; }
            else r >>= 1;
            bit >>= 2;
        }
        return (long)r;
    }

}

/// <summary>A 2D vector in 24.8 fixed point (all-integer).</summary>
internal readonly struct FxVec2
{
    public readonly long X, Y;

    public FxVec2(long x, long y) { X = x; Y = y; }

    public static FxVec2 From(Vec2 v) => new(Fx.From(v.X), Fx.From(v.Y));
    public Vec2 ToVec2() => new(Fx.To(X), Fx.To(Y));

    public static readonly FxVec2 Zero = new(0, 0);
    public static readonly FxVec2 UnitX = new(Fx.One, 0);

    public static FxVec2 operator +(FxVec2 a, FxVec2 b) => new(a.X + b.X, a.Y + b.Y);
    public static FxVec2 operator -(FxVec2 a, FxVec2 b) => new(a.X - b.X, a.Y - b.Y);
    public static FxVec2 operator -(FxVec2 a) => new(-a.X, -a.Y);

    /// <summary>Dot product; result carries squared scale (2^16).</summary>
    public long Dot(FxVec2 b) => X * b.X + Y * b.Y;

    public long LengthSq => X * X + Y * Y;

    public long DistSq(FxVec2 b)
    {
        long dx = X - b.X, dy = Y - b.Y;
        return dx * dx + dy * dy;
    }

    /// <summary>Length in fixed scale: isqrt of the squared-scale value.</summary>
    public long Length => Fx.Sqrt(LengthSq);

    /// <summary>Advance by a 16.16 parameter along this vector.</summary>
    public FxVec2 MulT(long t16) => new(Fx.MulT(X, t16), Fx.MulT(Y, t16));

    /// <summary>Scale a fixed vector by a 24.8 unit vector's magnitude factor.</summary>
    public FxVec2 MulUnit(long fx) => new(Fx.MulUnit(X, fx), Fx.MulUnit(Y, fx));

    /// <summary>
    /// Unit direction in 24.8 (components in [-256, 256]). Falls back when the
    /// vector is exactly zero, mirroring <see cref="Vec2.Normalized"/>.
    ///
    /// The length is taken at adaptive extra precision: the squared length is
    /// shifted up by the largest even amount that stays inside Int64 before the
    /// square root, so short (sub-pixel) vectors still yield a near-unit normal
    /// (length error &lt; ~0.15%) instead of the ~1.5% a plain 24.8 isqrt of a
    /// small value would give. Large vectors already have ample length precision,
    /// so their shift is small and the result is unchanged.
    /// </summary>
    public FxVec2 NormalizedFx(FxVec2 fallback)
    {
        long lenSq = LengthSq;
        if (lenSq == 0) return fallback;

        int shift = 16;
        while (shift > 0 && (lenSq >> (62 - shift)) != 0) shift -= 2;
        long hiLen = Fx.Sqrt(lenSq << shift);          // = worldLen * 2^(8 + shift/2)
        if (hiLen == 0) return fallback;

        int numShift = Fx.Shift + (shift >> 1);        // component numerator shift
        return new FxVec2(Fx.RoundDiv(X << numShift, hiLen), Fx.RoundDiv(Y << numShift, hiLen));
    }

}

/// <summary>
/// High-precision unit direction in signed Q1.30. Spatial vectors remain Q24.8;
/// keeping axes separate prevents angular quantization error from growing with
/// shape extent during projection.
/// </summary>
internal readonly struct FxAxis
{
    public const int Shift = 30;
    public const long One = 1L << Shift;

    public readonly long X;
    public readonly long Y;

    public FxAxis(long x, long y) { X = x; Y = y; }

    public static readonly FxAxis Zero = new(0, 0);
    public static readonly FxAxis UnitX = new(One, 0);
    public static readonly FxAxis UnitY = new(0, One);

    private static readonly int[] CordicAngles =
    {
        0x20000000, 0x12E4051E, 0x09FB385B, 0x051111D4,
        0x028B0D43, 0x0145D7E1, 0x00A2F61E, 0x00517C55,
        0x0028BE53, 0x00145F2F, 0x000A2F98, 0x000517CC,
        0x00028BE6, 0x000145F3, 0x0000A2FA, 0x0000517D,
        0x000028BE, 0x0000145F, 0x00000A30, 0x00000518,
        0x0000028C, 0x00000146, 0x000000A3, 0x00000051,
        0x00000029, 0x00000014, 0x0000000A, 0x00000005,
        0x00000003, 0x00000001, 0x00000001,
    };
    private const long CordicGainInverse = 652032874;

    public bool IsZero => X == 0 && Y == 0;
    public FxAxis Perpendicular => new(-Y, X);

    public static FxAxis FromVector(FxVec2 vector, FxAxis fallback) =>
        FromComponents(vector.X, vector.Y, fallback);

    public static FxAxis FromAngle(Angle32 angle)
    {
        if (angle.Raw == 0) return UnitX;
        if (angle.Raw == 0x40000000u) return UnitY;
        if (angle.Raw == 0x80000000u) return -UnitX;
        if (angle.Raw == 0xC0000000u) return -UnitY;

        uint quadrant = angle.Raw >> 30;
        long phase = angle.Raw & 0x3FFFFFFFu;
        bool swapInQuadrant = phase > 0x20000000L;
        long z = swapInQuadrant ? 0x40000000L - phase : phase;

        long x = CordicGainInverse;
        long y = 0;
        for (int i = 0; i < CordicAngles.Length; i++)
        {
            long direction = z >= 0 ? 1 : -1;
            long nextX = x - direction * (y >> i);
            long nextY = y + direction * (x >> i);
            z -= direction * CordicAngles[i];
            x = nextX;
            y = nextY;
        }
        long cosine = swapInQuadrant ? y : x;
        long sine = swapInQuadrant ? x : y;
        return quadrant switch
        {
            0 => FromComponents(cosine, sine, UnitX),
            1 => FromComponents(-sine, cosine, UnitY),
            2 => FromComponents(-cosine, -sine, -UnitX),
            _ => FromComponents(sine, -cosine, -UnitY),
        };
    }

    public static FxAxis FromComponents(long x, long y, FxAxis fallback)
    {
        long lengthSq = x * x + y * y;
        if (lengthSq == 0) return fallback;

        // Increase sqrt precision before normalization. For a tiny vector such
        // as (1,1), shifting lengthSq by 60 yields a ~Q30 denominator instead of
        // isqrt(2)==1. The shift is even so components use the matching scale.
        int extraShift = 60;
        while (extraShift > 0 && (lengthSq >> (62 - extraShift)) != 0)
            extraShift -= 2;

        long highPrecisionLength = Fx.Sqrt(lengthSq << extraShift);
        if (highPrecisionLength == 0) return fallback;
        int componentShift = extraShift >> 1;
        long scaledX = x << componentShift;
        long scaledY = y << componentShift;
        return new FxAxis(
            RatioQ30(scaledX, highPrecisionLength),
            RatioQ30(scaledY, highPrecisionLength));
    }

    /// <summary>Q24.8 position dot Q1.30 axis, returning Q?.38.</summary>
    public long Dot(FxVec2 position) => position.X * X + position.Y * Y;

    /// <summary>Q1.30 dot Q1.30, rounded back to Q1.30.</summary>
    public long Dot(FxAxis other) =>
        RoundShift(X * other.X + Y * other.Y, Shift);

    /// <summary>Scale this unit axis by a Q24.8 distance, returning Q24.8.</summary>
    public FxVec2 Scale(long distance) => new(
        RoundShift(X * distance, Shift),
        RoundShift(Y * distance, Shift));

    public FxVec2 ToFxVec2() => new(
        RoundShift(X, Shift - Fx.Shift),
        RoundShift(Y, Shift - Fx.Shift));

    public Vec2 ToVec2() => new(X / (float)One, Y / (float)One);

    public static FxAxis operator -(FxAxis axis) => new(-axis.X, -axis.Y);

    public static FxAxis Transform(FxAxis basisX, FxAxis basisY, FxAxis local)
    {
        long x = RoundShift(basisX.X * local.X + basisY.X * local.Y, Shift);
        long y = RoundShift(basisX.Y * local.X + basisY.Y * local.Y, Shift);
        return FromComponents(x, y, UnitX);
    }

    private static long RatioQ30(long numerator, long denominator)
    {
        if (denominator <= 0) throw new ArgumentOutOfRangeException(nameof(denominator));
        bool negative = numerator < 0;
        ulong remainder = Fx.Magnitude(numerator);
        ulong divisor = (ulong)denominator;
        if (remainder >= divisor) return negative ? -One : One;

        long result = 0;
        for (int i = 0; i < Shift; i++)
        {
            result <<= 1;
            if (remainder >= divisor - remainder)
            {
                remainder -= divisor - remainder;
                result |= 1;
            }
            else
            {
                remainder += remainder;
            }
        }
        return negative ? -result : result;
    }

    private static long RoundShift(long value, int shift)
    {
        long half = 1L << (shift - 1);
        return value >= 0
            ? (value + half) >> shift
            : -((-value + half) >> shift);
    }
}

internal readonly struct FxCircle
{
    public readonly FxVec2 Center;
    public readonly long Radius;
    public FxCircle(FxVec2 center, long radius) { Center = center; Radius = radius; }
    public static FxCircle From(Circle c) => new(FxVec2.From(c.Center), Fx.From(c.Radius));
}

internal readonly struct FxAabb
{
    public readonly FxVec2 Center;
    public readonly FxVec2 Half;
    public FxAabb(FxVec2 center, FxVec2 half) { Center = center; Half = half; }
    public static FxAabb From(Aabb b) => new(FxVec2.From(b.Center), FxVec2.From(b.HalfExtents));
    public FxVec2 Min => new(Center.X - Half.X, Center.Y - Half.Y);
    public FxVec2 Max => new(Center.X + Half.X, Center.Y + Half.Y);
}

/// <summary>Integer manifold used internally before the boundary conversion.
/// Normal is Q1.30; Depth and Contact are Q24.8.</summary>
internal readonly struct FxManifold
{
    public readonly bool Colliding;
    public readonly FxAxis Normal;
    public readonly long Depth;
    public readonly FxVec2 Contact;

    public FxManifold(bool colliding, FxAxis normal, long depth, FxVec2 contact)
    {
        Colliding = colliding;
        Normal = normal;
        Depth = depth;
        Contact = contact;
    }

    public FxManifold(bool colliding, FxVec2 normal, long depth, FxVec2 contact)
        : this(colliding,
            colliding ? FxAxis.FromVector(normal, FxAxis.UnitX) : FxAxis.Zero,
            depth, contact)
    {
    }

    public static readonly FxManifold None = new(false, FxAxis.Zero, 0, FxVec2.Zero);

    public Manifold ToManifold() => Colliding
        ? new Manifold(true, Normal.ToVec2(), Fx.To(Depth), Contact.ToVec2())
        : Manifold.None;
}

/// <summary>Integer sweep result. Time is 16.16; Normal is Q1.30.</summary>
internal readonly struct FxSweep
{
    public readonly bool Hit;
    public readonly long Time16;
    public readonly FxAxis Normal;
    public readonly FxVec2 Point;

    public FxSweep(bool hit, long time16, FxAxis normal, FxVec2 point)
    {
        Hit = hit;
        Time16 = time16;
        Normal = normal;
        Point = point;
    }

    public FxSweep(bool hit, long time16, FxVec2 normal, FxVec2 point)
        : this(hit,
            time16,
            hit ? FxAxis.FromVector(normal, FxAxis.UnitX) : FxAxis.Zero,
            point)
    {
    }

    public static readonly FxSweep Miss = new(false, Fx.TOne, FxAxis.Zero, FxVec2.Zero);

    public SweepHit ToSweepHit() => Hit
        ? new SweepHit(true, Fx.ToT(Time16), Normal.ToVec2(), Point.ToVec2())
        : SweepHit.Miss;


}
