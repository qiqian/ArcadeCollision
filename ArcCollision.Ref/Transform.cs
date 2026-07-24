using System;
using System.Runtime.InteropServices;

namespace ArcCollision.Ref;

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

    /// <summary>The cheap path: no rotation and unit scale, so materialize is a
    /// pure translation of the base shape to <see cref="Position"/>.</summary>
    public bool IsTranslationOnly => Rotation.Raw == 0 && Scale == 1f;

    public bool Equals(Transform other) =>
        Position.Equals(other.Position) && Rotation == other.Rotation
        && Scale.Equals(other.Scale);

    public override bool Equals(object? obj) => obj is Transform other && Equals(other);
    public override int GetHashCode() => HashCode.Combine(Position, Rotation.Raw, Scale);
}

/// <summary>World-owned transform after the public floats have crossed the API
/// boundary: position is Q24.8, scale is Q16.16, and rotation is Angle32.</summary>
internal readonly struct FxTransform
{
    public readonly FxVec2 Position;
    public readonly uint Rotation;
    public readonly long Scale16;

    public FxTransform(FxVec2 position, uint rotation, long scale16)
    {
        Position = position;
        Rotation = rotation;
        Scale16 = scale16;
    }

    public static FxTransform From(in Transform value) => new(
        new FxVec2(
            Fx.FromTransformPosition(value.Position.X),
            Fx.FromTransformPosition(value.Position.Y)),
        value.Rotation.Raw,
        Fx.FromScale(value.Scale));

    public static FxTransform Compose(in FxTransform current, in FxTransform delta) => new(
        current.Position + delta.Position,
        unchecked(current.Rotation + delta.Rotation),
        Fx.MulScales(current.Scale16, delta.Scale16));
}

/// <summary>
/// A collider's shape reduced to its local canonical form, precomputed once on
/// Add: geometry relative to the shape's own origin and orientation, with
/// placement (centre / segment midpoint / polygon translation) factored out into
/// the seed <see cref="Transform"/>. Because the base is immutable, each transform
/// update materializes the world shape directly from this form without re-deriving
/// it (e.g. the capsule endpoints are already relative to the midpoint).
/// </summary>
[StructLayout(LayoutKind.Explicit, Size = 56)]
internal readonly struct LocalShape
{
    // Fixed primitive payloads form a tagged union. Capsule is the largest at
    // 40 bytes: A @0, B @16, radius @32. As with public Shape, the managed
    // Polygon reference must live in a separate non-overlapping slot so the GC
    // can identify it reliably.
    [FieldOffset(0)] private readonly long _circleRadius;
    [FieldOffset(0)] private readonly FxVec2 _boxHalfExtents;
    [FieldOffset(16)] private readonly uint _obbBaseAngle;
    [FieldOffset(0)] private readonly FxVec2 _capsuleA;
    [FieldOffset(16)] private readonly FxVec2 _capsuleB;
    [FieldOffset(32)] private readonly long _capsuleRadius;
    [FieldOffset(0)] private readonly uint _polygonBaseAngle;

    [FieldOffset(40)] public readonly ShapeKind Kind;
    [FieldOffset(48)] private readonly Polygon? _polygon;

    private LocalShape(long circleRadius)
    {
        this = default;
        Kind = ShapeKind.Circle;
        _circleRadius = circleRadius;
    }

    private LocalShape(ShapeKind kind, FxVec2 halfExtents, uint baseAngle)
    {
        this = default;
        Kind = kind;
        _boxHalfExtents = halfExtents;
        _obbBaseAngle = baseAngle;
    }

    private LocalShape(FxVec2 a, FxVec2 b, long radius)
    {
        this = default;
        Kind = ShapeKind.Capsule;
        _capsuleA = a;
        _capsuleB = b;
        _capsuleRadius = radius;
    }

    private LocalShape(Polygon polygon, uint baseAngle)
    {
        this = default;
        Kind = ShapeKind.Polygon;
        _polygon = polygon;
        _polygonBaseAngle = baseAngle;
    }

    internal long CircleRadius => _circleRadius;
    internal FxVec2 BoxHalfExtents => _boxHalfExtents;
    internal uint ObbBaseAngle => _obbBaseAngle;
    internal FxVec2 CapsuleA => _capsuleA;
    internal FxVec2 CapsuleB => _capsuleB;
    internal long CapsuleRadius => _capsuleRadius;
    internal uint PolygonBaseAngle => _polygonBaseAngle;
    internal Polygon Polygon => _polygon!;

    // Reduce a world-placed shape to its local form plus the transform that
    // reproduces it (Materialize(result, initial) == shape). The initial rotation
    // is always zero: the authored orientation lives in the local form, and the
    // transform's rotation is applied on top of it.
    public static LocalShape From(in Shape shape, out FxTransform initial)
    {
        switch (shape.Kind)
        {
            case ShapeKind.Circle:
            {
                Circle circle = shape.Circle;
                FxVec2 center = FxVec2.From(circle.Center);
                initial = new FxTransform(center, 0, Fx.ScaleOne);
                return new LocalShape(Fx.From(circle.Radius));
            }
            case ShapeKind.Aabb:
            {
                Aabb aabb = shape.Aabb;
                initial = new FxTransform(FxVec2.From(aabb.Center), 0, Fx.ScaleOne);
                return new LocalShape(
                    ShapeKind.Aabb, FxVec2.From(aabb.HalfExtents), 0);
            }
            case ShapeKind.Obb:
            {
                Obb obb = shape.Obb;
                initial = new FxTransform(FxVec2.From(obb.Center), 0, Fx.ScaleOne);
                return new LocalShape(
                    ShapeKind.Obb, FxVec2.From(obb.HalfExtents), obb.Angle.Raw);
            }
            case ShapeKind.Capsule:
            {
                Capsule capsule = shape.Capsule;
                FxVec2 a = FxVec2.From(capsule.A);
                FxVec2 b = FxVec2.From(capsule.B);
                var mid = new FxVec2((a.X + b.X) / 2, (a.Y + b.Y) / 2);
                initial = new FxTransform(mid, 0, Fx.ScaleOne);
                return new LocalShape(a - mid, b - mid, Fx.From(capsule.Radius));
            }
            case ShapeKind.Polygon:
            {
                initial = new FxTransform(
                    FxVec2.From(shape.PolygonTranslation), 0, Fx.ScaleOne);
                return new LocalShape(shape.Polygon, shape.PolygonRotation.Raw);
            }
            default:
                throw new InvalidOperationException("Invalid shape kind.");
        }
    }
}

/// <summary>Places a precomputed <see cref="LocalShape"/> into world space under a
/// <see cref="Transform"/>.</summary>
internal static class ShapeTransform
{
    internal readonly struct Result
    {
        public readonly Shape Shape;
        public readonly BpBounds Bounds;
        // Translation-invariant capsule endpoint offsets after the current
        // rotation and scale. Other shape kinds leave these at zero.
        public readonly FxVec2 CapsuleAOffset;
        public readonly FxVec2 CapsuleBOffset;

        public Result(
            Shape shape,
            BpBounds bounds,
            FxVec2 capsuleAOffset = default,
            FxVec2 capsuleBOffset = default)
        {
            Shape = shape;
            Bounds = bounds;
            CapsuleAOffset = capsuleAOffset;
            CapsuleBOffset = capsuleBOffset;
        }
    }

    /// <summary>
    /// Re-places an already posed shape when only its fixed-point position
    /// changed. Rotation/scale work and scaled-polygon allocation are skipped;
    /// no public float is ever used as the source of the translation.
    /// </summary>
    public static Result Translate(
        in LocalShape local,
        in Shape current,
        in BpBounds currentBounds,
        in FxTransform currentTransform,
        in FxTransform transform,
        FxVec2 capsuleAOffset,
        FxVec2 capsuleBOffset)
    {
        FxVec2 position = transform.Position;
        FxVec2 delta = position - currentTransform.Position;
        BpBounds bounds = currentBounds.Translated(delta.X, delta.Y);
        switch (local.Kind)
        {
            case ShapeKind.Circle:
            {
                long radius = Fx.MulScale(local.CircleRadius, transform.Scale16);
                return new Result(
                    new Circle(position.ToVec2(), Fx.To(radius)), bounds);
            }
            case ShapeKind.Aabb:
            {
                FxVec2 half = Scale(local.BoxHalfExtents, transform.Scale16);
                return new Result(
                    new Aabb(position.ToVec2(), half.ToVec2()), bounds);
            }
            case ShapeKind.Obb:
            {
                FxVec2 half = Scale(local.BoxHalfExtents, transform.Scale16);
                var angle = new Angle32(
                    unchecked(local.ObbBaseAngle + transform.Rotation));
                return new Result(
                    new Obb(position.ToVec2(), half.ToVec2(), angle), bounds);
            }
            case ShapeKind.Capsule:
            {
                FxVec2 a = position + capsuleAOffset;
                FxVec2 b = position + capsuleBOffset;
                long radius = Fx.MulScale(
                    local.CapsuleRadius, transform.Scale16);
                return new Result(
                    new Capsule(a.ToVec2(), b.ToVec2(), Fx.To(radius)),
                    bounds,
                    capsuleAOffset,
                    capsuleBOffset);
            }
            case ShapeKind.Polygon:
            {
                // A non-unit scale can only have entered the current Shape via a
                // full materialization, so it already owns the correctly scaled
                // immutable geometry. Unit scale reuses the authored polygon.
                Polygon polygon = transform.Scale16 == Fx.ScaleOne
                    ? local.Polygon
                    : current.Polygon;
                var angle = new Angle32(
                    unchecked(local.PolygonBaseAngle + transform.Rotation));
                return new Result(
                    new Shape(polygon, position.ToVec2(), angle), bounds);
            }
            default:
                throw new InvalidOperationException("Invalid shape kind.");
        }
    }

    public static Result Materialize(in LocalShape local, in FxTransform transform)
    {
        FxVec2 position = transform.Position;
        switch (local.Kind)
        {
            case ShapeKind.Circle:
            {
                long radius = Fx.MulScale(local.CircleRadius, transform.Scale16);
                return new Result(
                    new Circle(position.ToVec2(), Fx.To(radius)),
                    BpBounds.FromFixedCircle(position, radius));
            }
            case ShapeKind.Aabb:
            {
                FxVec2 half = Scale(local.BoxHalfExtents, transform.Scale16);
                return new Result(
                    new Aabb(position.ToVec2(), half.ToVec2()),
                    BpBounds.FromFixedAabb(position, half));
            }
            case ShapeKind.Obb:
            {
                FxVec2 half = Scale(local.BoxHalfExtents, transform.Scale16);
                var angle = new Angle32(unchecked(local.ObbBaseAngle + transform.Rotation));
                return new Result(
                    new Obb(position.ToVec2(), half.ToVec2(), angle),
                    BpBounds.FromFixedObb(position, half, angle));
            }
            case ShapeKind.Capsule:
            {
                FxVec2 aOffset = RotateScale(local.CapsuleA, transform);
                FxVec2 bOffset = RotateScale(local.CapsuleB, transform);
                FxVec2 a = position + aOffset;
                FxVec2 b = position + bOffset;
                long radius = Fx.MulScale(local.CapsuleRadius, transform.Scale16);
                return new Result(
                    new Capsule(a.ToVec2(), b.ToVec2(), Fx.To(radius)),
                    BpBounds.FromFixedCapsule(a, b, radius),
                    aOffset,
                    bOffset);
            }
            case ShapeKind.Polygon:
            {
                Polygon polygon = transform.Scale16 == Fx.ScaleOne
                    ? local.Polygon : Scaled(local.Polygon, transform.Scale16);
                var angle = new Angle32(
                    unchecked(local.PolygonBaseAngle + transform.Rotation));
                return new Result(
                    new Shape(polygon, position.ToVec2(), angle),
                    BpBounds.FromFixedPolygon(polygon, position, angle));
            }
            default:
                throw new InvalidOperationException("Invalid shape kind.");
        }
    }

    private static FxVec2 Scale(FxVec2 value, long scale16) => new(
        Fx.MulScale(value.X, scale16), Fx.MulScale(value.Y, scale16));

    private static FxVec2 RotateScale(FxVec2 local, in FxTransform transform)
    {
        FxVec2 scaled = Scale(local, transform.Scale16);
        if (transform.Rotation == 0) return scaled;
        FxAxis axisX = FxAxis.FromAngle(new Angle32(transform.Rotation));
        FxAxis axisY = axisX.Perpendicular;
        return axisX.Scale(scaled.X) + axisY.Scale(scaled.Y);
    }

    private static Polygon Scaled(Polygon polygon, long scale16)
    {
        var vertices = new FxVec2[polygon.Count];
        for (int i = 0; i < vertices.Length; i++)
        {
            FxVec2 source = polygon.FixedVertices[i];
            vertices[i] = Scale(source, scale16);
        }
        return Polygon.FromFixed(vertices);
    }
}
