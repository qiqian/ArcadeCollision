using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;

namespace ArcCollision.Wrapper;

[StructLayout(LayoutKind.Sequential)]
public readonly struct Vec2 : IEquatable<Vec2>
{
    public readonly float X, Y;
    public Vec2(float x, float y) { X = x; Y = y; }
    public static readonly Vec2 Zero = new(0, 0);
    public static readonly Vec2 UnitX = new(1, 0);
    public static readonly Vec2 UnitY = new(0, 1);
    public static Vec2 operator +(Vec2 a, Vec2 b) => new(a.X + b.X, a.Y + b.Y);
    public static Vec2 operator -(Vec2 a, Vec2 b) => new(a.X - b.X, a.Y - b.Y);
    public static Vec2 operator -(Vec2 a) => new(-a.X, -a.Y);
    public static Vec2 operator *(Vec2 a, float s) => new(a.X * s, a.Y * s);
    public static Vec2 operator *(float s, Vec2 a) => a * s;
    public static Vec2 operator /(Vec2 a, float s) => new(a.X / s, a.Y / s);
    public float Dot(Vec2 b) => X * b.X + Y * b.Y;
    public float Cross(Vec2 b) => X * b.Y - Y * b.X;
    public float LengthSquared => X * X + Y * Y;
    public float Length => MathF.Sqrt(LengthSquared);
    public Vec2 Perp => new(-Y, X);
    public float DistanceSquared(Vec2 b) => (this - b).LengthSquared;
    public float Distance(Vec2 b) => MathF.Sqrt(DistanceSquared(b));
    public Vec2 Normalized(Vec2 fallback = default)
    {
        float lengthSquared = LengthSquared;
        if (lengthSquared < 1e-12f)
            return fallback;
        float inverseLength = 1f / MathF.Sqrt(lengthSquared);
        return new Vec2(X * inverseLength, Y * inverseLength);
    }
    public static Vec2 Lerp(Vec2 a, Vec2 b, float t) => a + (b - a) * t;
    public static Vec2 Min(Vec2 a, Vec2 b) => new(MathF.Min(a.X, b.X), MathF.Min(a.Y, b.Y));
    public static Vec2 Max(Vec2 a, Vec2 b) => new(MathF.Max(a.X, b.X), MathF.Max(a.Y, b.Y));
    public bool Equals(Vec2 other) => X.Equals(other.X) && Y.Equals(other.Y);
    public override bool Equals(object? obj) => obj is Vec2 other && Equals(other);
    public override int GetHashCode() => DeterministicHash.Combine(
        DeterministicHash.Float(X), DeterministicHash.Float(Y));
    public override string ToString() => $"({X:0.###}, {Y:0.###})";
}

[StructLayout(LayoutKind.Sequential)]
public readonly struct Angle32 : IEquatable<Angle32>
{
    public readonly uint Raw;
    public Angle32(uint raw) => Raw = raw;
    public static Angle32 FromRadians(float radians)
    {
        if (!float.IsFinite(radians))
            throw new ArgumentOutOfRangeException(nameof(radians), radians,
                "Rotation must be finite.");
        const double units = 4294967296.0 / (Math.PI * 2.0);
        long value = (long)Math.Round(radians % (Math.PI * 2.0) * units, MidpointRounding.ToEven);
        return new Angle32(unchecked((uint)value));
    }
    public float Radians => (float)(unchecked((int)Raw) * ((Math.PI * 2.0) / 4294967296.0));
    public static Angle32 operator -(Angle32 angle) => new(unchecked(0u - angle.Raw));
    public bool Equals(Angle32 other) => Raw == other.Raw;
    public override bool Equals(object? obj) => obj is Angle32 other && Equals(other);
    public override int GetHashCode() => unchecked((int)Raw);
    public static bool operator ==(Angle32 left, Angle32 right) => left.Raw == right.Raw;
    public static bool operator !=(Angle32 left, Angle32 right) => left.Raw != right.Raw;
}

[StructLayout(LayoutKind.Sequential)]
public readonly struct Circle
{
    public readonly Vec2 Center;
    public readonly float Radius;
    public Circle(Vec2 center, float radius)
    {
        if (!(radius >= 0f))
            throw new ArgumentOutOfRangeException(nameof(radius), radius,
                "Radius must be non-negative (NaN rejected).");
        Center = center; Radius = radius;
    }
    public Aabb Bounds => new(Center, new Vec2(Radius, Radius));
    public Circle Moved(Vec2 delta) => new(Center + delta, Radius);
}

[StructLayout(LayoutKind.Sequential)]
public readonly struct Aabb
{
    public readonly Vec2 Center, HalfExtents;
    public Aabb(Vec2 center, Vec2 halfExtents)
    {
        if (!(halfExtents.X >= 0f) || !(halfExtents.Y >= 0f))
            throw new ArgumentOutOfRangeException(nameof(halfExtents), halfExtents,
                "Half-extents must be non-negative (NaN rejected).");
        Center = center; HalfExtents = halfExtents;
    }
    public Vec2 Min => Center - HalfExtents;
    public Vec2 Max => Center + HalfExtents;
    public static Aabb FromMinMax(Vec2 min, Vec2 max) => new((min + max) * .5f, (max - min) * .5f);
    public Aabb Moved(Vec2 delta) => new(Center + delta, HalfExtents);
    public Aabb Expanded(float amount) => new(Center, HalfExtents + new Vec2(amount, amount));
    public static Aabb Union(Aabb a, Aabb b) => FromMinMax(Vec2.Min(a.Min, b.Min), Vec2.Max(a.Max, b.Max));
    public bool Overlaps(Aabb b)
    {
        return Math.Abs(FixedValidation.From(Center.X) - FixedValidation.From(b.Center.X))
                <= FixedValidation.From(HalfExtents.X) + FixedValidation.From(b.HalfExtents.X)
            && Math.Abs(FixedValidation.From(Center.Y) - FixedValidation.From(b.Center.Y))
                <= FixedValidation.From(HalfExtents.Y) + FixedValidation.From(b.HalfExtents.Y);
    }
}

[StructLayout(LayoutKind.Sequential)]
public readonly struct Capsule
{
    public readonly Vec2 A, B;
    public readonly float Radius;
    public Capsule(Vec2 a, Vec2 b, float radius)
    {
        if (!(radius >= 0f))
            throw new ArgumentOutOfRangeException(nameof(radius), radius,
                "Radius must be non-negative (NaN rejected).");
        A = a; B = b; Radius = radius;
    }
    public Aabb Bounds => Aabb.FromMinMax(Vec2.Min(A, B) - new Vec2(Radius, Radius), Vec2.Max(A, B) + new Vec2(Radius, Radius));
    public Capsule Moved(Vec2 delta) => new(A + delta, B + delta, Radius);
}

[StructLayout(LayoutKind.Sequential)]
public readonly struct Obb
{
    public readonly Vec2 Center, HalfExtents;
    public readonly Angle32 Angle;
    public float Rotation => Angle.Radians;
    public Obb(Vec2 center, Vec2 halfExtents, float rotation = 0) : this(center, halfExtents, Angle32.FromRadians(rotation)) { }
    public Obb(Vec2 center, Vec2 halfExtents, Angle32 angle)
    {
        if (!(halfExtents.X >= 0f) || !(halfExtents.Y >= 0f))
            throw new ArgumentOutOfRangeException(nameof(halfExtents), halfExtents,
                "Half-extents must be non-negative (NaN rejected).");
        Center = center; HalfExtents = halfExtents; Angle = angle;
    }
    public Aabb Bounds
    {
        get
        {
            _ = FixedValidation.From(HalfExtents.X);
            _ = FixedValidation.From(HalfExtents.Y);
            NativeShape shape = default;
            shape.Kind = (int)ShapeKind.Obb;
            shape.Obb = this;
            return NativeMethods.ShapeBounds(shape);
        }
    }
    public Obb Moved(Vec2 delta) => new(Center + delta, HalfExtents, Angle);
}

internal sealed class PolygonHandle : SafeHandleZeroOrMinusOneIsInvalid
{
    public PolygonHandle() : base(true) { }
    internal PolygonHandle(IntPtr pointer, bool owns = true) : base(owns) => SetHandle(pointer);
    protected override bool ReleaseHandle() { NativeMethods.PolygonRelease(handle); return true; }
}

public sealed class Polygon
{
    private readonly Vec2[] _vertices;
    private readonly Aabb _bounds;
    internal PolygonHandle Handle { get; }

    public Polygon(params Vec2[] vertices) : this(vertices.AsSpan()) { }
    public Polygon(ReadOnlySpan<Vec2> vertices)
    {
        if (vertices.Length < 3) throw new ArgumentException("A polygon requires at least three vertices.", nameof(vertices));
        for (int i = 0; i < vertices.Length; i++)
            FixedValidation.Vec2(vertices[i]);
        _vertices = vertices.ToArray();
        _bounds = ComputeBounds(_vertices);
        IntPtr pointer = NativeMethods.PolygonCreate(_vertices, _vertices.Length);
        if (pointer == IntPtr.Zero) NativeMethods.ThrowLastError("Polygon creation failed.");
        Handle = new PolygonHandle(pointer);
    }
    internal Polygon(IntPtr retainedPointer)
        : this(retainedPointer, NativeMethods.PolygonGetBounds(retainedPointer))
    {
    }
    internal Polygon(IntPtr retainedPointer, Aabb bounds)
    {
        Handle = new PolygonHandle(retainedPointer);
        int count = NativeMethods.PolygonGetCount(retainedPointer);
        _vertices = new Vec2[count];
        NativeMethods.Check(NativeMethods.PolygonGetVertices(retainedPointer, _vertices, count, out _));
        _bounds = bounds;
    }
    public int Count => _vertices.Length;
    public Vec2 this[int index] => _vertices[index];
    public ReadOnlySpan<Vec2> Vertices => _vertices;
    public Aabb Bounds => _bounds;

    private static Aabb ComputeBounds(Vec2[] vertices)
    {
        Vec2 min = vertices[0], max = min;
        for (int i = 1; i < vertices.Length; i++)
        {
            min = Vec2.Min(min, vertices[i]);
            max = Vec2.Max(max, vertices[i]);
        }
        return Aabb.FromMinMax(min, max);
    }
    public Polygon Moved(Vec2 delta)
    {
        FixedValidation.Vec2(delta);
        IntPtr pointer = NativeMethods.PolygonMoved(Handle.DangerousGetHandle(), delta);
        if (pointer == IntPtr.Zero) NativeMethods.ThrowLastError("Polygon movement failed.");
        GC.KeepAlive(this);
        return new Polygon(pointer, _bounds.Moved(delta));
    }
}

public enum ShapeKind { Circle, Aabb, Capsule, Obb, Polygon }

[StructLayout(LayoutKind.Explicit, Size = 32)]
public readonly struct Shape
{
    [FieldOffset(0)] private readonly Circle _circle;
    [FieldOffset(0)] private readonly Aabb _aabb;
    [FieldOffset(0)] private readonly Capsule _capsule;
    [FieldOffset(0)] private readonly Obb _obb;
    [FieldOffset(0)] private readonly Vec2 _polygonTranslation;
    [FieldOffset(8)] private readonly Angle32 _polygonRotation;
    [FieldOffset(20)] public readonly ShapeKind Kind;
    [FieldOffset(24)] private readonly Polygon? _polygon;
    public Shape(Circle value) { this = default; Kind = ShapeKind.Circle; _circle = value; }
    public Shape(Aabb value) { this = default; Kind = ShapeKind.Aabb; _aabb = value; }
    public Shape(Capsule value) { this = default; Kind = ShapeKind.Capsule; _capsule = value; }
    public Shape(Obb value) { this = default; Kind = ShapeKind.Obb; _obb = value; }
    public Shape(Polygon value) : this(value, Vec2.Zero, new Angle32(0)) { }
    public Shape(Polygon value, Vec2 translation, Angle32 rotation)
    { ArgumentNullException.ThrowIfNull(value); this = default; Kind = ShapeKind.Polygon; _polygon = value; _polygonTranslation = translation; _polygonRotation = rotation; }
    public Shape(Polygon value, Vec2 translation, float rotation) : this(value, translation, Angle32.FromRadians(rotation)) { }
    public Aabb Bounds => Kind switch { ShapeKind.Circle => _circle.Bounds, ShapeKind.Aabb => _aabb, ShapeKind.Capsule => _capsule.Bounds, ShapeKind.Obb => _obb.Bounds, ShapeKind.Polygon => NativeBounds(), _ => throw new InvalidOperationException() };
    public Shape Moved(Vec2 delta) => Kind switch { ShapeKind.Circle => _circle.Moved(delta), ShapeKind.Aabb => _aabb.Moved(delta), ShapeKind.Capsule => _capsule.Moved(delta), ShapeKind.Obb => _obb.Moved(delta), ShapeKind.Polygon => new Shape(_polygon!, _polygonTranslation + delta, _polygonRotation), _ => throw new InvalidOperationException() };
    public Shape WithPolygonTransform(Vec2 translation, Angle32 rotation) => Kind == ShapeKind.Polygon ? new Shape(_polygon!, translation, rotation) : throw new InvalidOperationException();
    public Vec2 PolygonTranslation => Kind == ShapeKind.Polygon ? _polygonTranslation : throw new InvalidOperationException();
    public Angle32 PolygonRotation => Kind == ShapeKind.Polygon ? _polygonRotation : throw new InvalidOperationException();
    public bool TryGetCircle(out Circle value) { value = Kind == ShapeKind.Circle ? _circle : default; return Kind == ShapeKind.Circle; }
    public bool TryGetAabb(out Aabb value) { value = Kind == ShapeKind.Aabb ? _aabb : default; return Kind == ShapeKind.Aabb; }
    public bool TryGetCapsule(out Capsule value) { value = Kind == ShapeKind.Capsule ? _capsule : default; return Kind == ShapeKind.Capsule; }
    public bool TryGetObb(out Obb value) { value = Kind == ShapeKind.Obb ? _obb : default; return Kind == ShapeKind.Obb; }
    public bool TryGetPolygon(out Polygon? geometry, out Vec2 translation, out Angle32 rotation) { bool yes = Kind == ShapeKind.Polygon; geometry = yes ? _polygon : null; translation = yes ? _polygonTranslation : default; rotation = yes ? _polygonRotation : default; return yes; }
    internal Polygon? PolygonObject => _polygon;
    internal NativeShape ToNative() => new(this);
    private Aabb NativeBounds()
    {
        if (_polygonRotation.Raw == 0)
            return _polygon!.Bounds.Moved(_polygonTranslation);
        NativeShape native = ToNative();
        FixedValidation.PolygonTransform(this);
        Aabb result = NativeMethods.ShapeBounds(native);
        GC.KeepAlive(_polygon);
        return result;
    }
    public static implicit operator Shape(Circle value) => new(value);
    public static implicit operator Shape(Aabb value) => new(value);
    public static implicit operator Shape(Capsule value) => new(value);
    public static implicit operator Shape(Obb value) => new(value);
    public static implicit operator Shape(Polygon value) => new(value);
}

[StructLayout(LayoutKind.Sequential)]
public readonly struct Manifold
{
    [MarshalAs(UnmanagedType.Bool)] public readonly bool Colliding;
    public readonly Vec2 Normal;
    public readonly float Depth;
    public readonly Vec2 Contact;
    public Manifold(bool colliding, Vec2 normal, float depth, Vec2 contact) { Colliding = colliding; Normal = normal; Depth = depth; Contact = contact; }
    public static readonly Manifold None = new(false, Vec2.Zero, 0, Vec2.Zero);
    public Vec2 SeparationForA => Normal * -Depth;
    public Vec2 SeparationForB => Normal * Depth;
}

[StructLayout(LayoutKind.Sequential)]
public readonly struct SweepHit
{
    [MarshalAs(UnmanagedType.Bool)] public readonly bool Hit;
    public readonly float Time;
    public readonly Vec2 Normal, Point;
    public SweepHit(bool hit, float time, Vec2 normal, Vec2 point) { Hit = hit; Time = time; Normal = normal; Point = point; }
    public static readonly SweepHit Miss = new(false, 1, Vec2.Zero, Vec2.Zero);
}

public static class CollisionCategories { public const uint Default = 1; public const uint All = uint.MaxValue; }
[StructLayout(LayoutKind.Sequential)]
public readonly struct CollisionFilter : IEquatable<CollisionFilter>
{
    public readonly uint Categories, CollidesWith;
    public static readonly CollisionFilter Disabled = default;
    public static readonly CollisionFilter Default = new(CollisionCategories.Default, CollisionCategories.All);
    public CollisionFilter(uint categories, uint collidesWith = CollisionCategories.All) { Categories = categories; CollidesWith = collidesWith; }
    public bool IsDisabled => Categories == 0 || CollidesWith == 0;
    public bool CanCollideWith(in CollisionFilter other) => (Categories & other.CollidesWith) != 0 && (other.Categories & CollidesWith) != 0;
    public bool Allows(in CollisionFilter other) => CanCollideWith(other);
    public bool Equals(CollisionFilter other) => Categories == other.Categories && CollidesWith == other.CollidesWith;
    public override bool Equals(object? obj) => obj is CollisionFilter other && Equals(other);
    public override int GetHashCode() => DeterministicHash.Combine(
        unchecked((int)Categories), unchecked((int)CollidesWith));
    public static bool operator ==(CollisionFilter left, CollisionFilter right) => left.Equals(right);
    public static bool operator !=(CollisionFilter left, CollisionFilter right) => !left.Equals(right);
}

public static class CollisionLimits { public const float GridSize = 1f / 256f; public const float MaxCoordinate = 1_953_125f; }
public readonly struct ArcWorldOptions
{
    public readonly float FatMargin;
    public readonly int InitialColliderCapacity, InitialPairCapacity;
    public ArcWorldOptions() : this(16, 16, 16) { }
    public ArcWorldOptions(float fatMargin = 16, int initialColliderCapacity = 16, int initialPairCapacity = 16)
    {
        if (!float.IsFinite(fatMargin) || fatMargin < 0f)
            throw new ArgumentOutOfRangeException(nameof(fatMargin));
        if (initialColliderCapacity is < 0 or > ArcWorld.MaxColliderCount)
            throw new ArgumentOutOfRangeException(nameof(initialColliderCapacity));
        if (initialPairCapacity < 0)
            throw new ArgumentOutOfRangeException(nameof(initialPairCapacity));
        FatMargin = fatMargin;
        InitialColliderCapacity = initialColliderCapacity;
        InitialPairCapacity = initialPairCapacity;
    }
}

public enum SweepAlgorithm { AnalyticCircle, RoundedAabb, RoundedSegment, LocalSpaceRoundedAabb, SweptAabb, ContinuousSat, FeatureCast }

internal static class DeterministicHash
{
    private const uint Offset = 2166136261u, Prime = 16777619u;
    private static uint Add(uint hash, uint value) => unchecked((hash ^ value) * Prime);
    public static int Combine(int a, int b) => unchecked((int)Add(Add(Offset, unchecked((uint)a)), unchecked((uint)b)));
    public static int Combine(int a, uint b, uint c) => unchecked((int)Add(Add(Add(Offset, unchecked((uint)a)), b), c));
    public static int Float(float value)
    {
        uint bits = BitConverter.SingleToUInt32Bits(value);
        if ((bits & 0x7fffffffu) == 0) return 0;
        if ((bits & 0x7f800000u) == 0x7f800000u && (bits & 0x007fffffu) != 0) bits = 0x7fc00000u;
        return unchecked((int)bits);
    }
}
