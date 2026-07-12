using System;
using System.Runtime.InteropServices;

namespace ArcCollision;

public enum ShapeKind
{
    Circle,
    Aabb,
    Capsule,
    Obb,
    Polygon,
}

/// <summary>
/// Allocation-free discriminated union for generic collision dispatch. Polygon
/// instances remain references to their immutable cached vertex data.
/// </summary>
[StructLayout(LayoutKind.Explicit, Size = 32)]
public readonly struct Shape
{
    // C-style tagged union. Primitive members are unmanaged value types and
    // safely share the same storage; the managed Polygon reference stays in a
    // separate, pointer-aligned slot so the GC can track it correctly.
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
    public Shape(Polygon value)
        : this(value, Vec2.Zero, new Angle32(0))
    {
    }

    /// <summary>
    /// Creates a polygon instance that shares immutable geometry and applies a
    /// translation and rotation about the geometry origin without allocating.
    /// </summary>
    public Shape(Polygon value, Vec2 translation, Angle32 rotation)
    {
        ArgumentNullException.ThrowIfNull(value);
        this = default;
        Kind = ShapeKind.Polygon;
        _polygon = value;
        _polygonTranslation = translation;
        _polygonRotation = rotation;
    }

    public Shape(Polygon value, Vec2 translation, float rotation)
        : this(value, translation, Angle32.FromRadians(rotation))
    {
    }

    public Aabb Bounds => Kind switch
    {
        ShapeKind.Circle => _circle.Bounds,
        ShapeKind.Aabb => _aabb,
        ShapeKind.Capsule => _capsule.Bounds,
        ShapeKind.Obb => _obb.Bounds,
        ShapeKind.Polygon => PolygonBounds(),
        _ => throw new InvalidOperationException("Invalid shape kind."),
    };

    public Shape Moved(Vec2 delta) => Kind switch
    {
        ShapeKind.Circle => _circle.Moved(delta),
        ShapeKind.Aabb => _aabb.Moved(delta),
        ShapeKind.Capsule => _capsule.Moved(delta),
        ShapeKind.Obb => _obb.Moved(delta),
        ShapeKind.Polygon => new Shape(
            _polygon!, _polygonTranslation + delta, _polygonRotation),
        _ => throw new InvalidOperationException("Invalid shape kind."),
    };

    public Shape WithPolygonTransform(Vec2 translation, Angle32 rotation)
    {
        if (Kind != ShapeKind.Polygon)
            throw new InvalidOperationException("Only polygon shapes have an instance transform.");
        return new Shape(_polygon!, translation, rotation);
    }

    public bool TryGetCircle(out Circle value)
    {
        if (Kind == ShapeKind.Circle)
        {
            value = _circle;
            return true;
        }
        value = default;
        return false;
    }

    public bool TryGetAabb(out Aabb value)
    {
        if (Kind == ShapeKind.Aabb)
        {
            value = _aabb;
            return true;
        }
        value = default;
        return false;
    }

    public bool TryGetCapsule(out Capsule value)
    {
        if (Kind == ShapeKind.Capsule)
        {
            value = _capsule;
            return true;
        }
        value = default;
        return false;
    }

    public bool TryGetObb(out Obb value)
    {
        if (Kind == ShapeKind.Obb)
        {
            value = _obb;
            return true;
        }
        value = default;
        return false;
    }

    public bool TryGetPolygon(
        out Polygon? geometry, out Vec2 translation, out Angle32 rotation)
    {
        if (Kind == ShapeKind.Polygon)
        {
            geometry = _polygon;
            translation = _polygonTranslation;
            rotation = _polygonRotation;
            return true;
        }
        geometry = null;
        translation = default;
        rotation = default;
        return false;
    }

    internal Circle Circle => _circle;
    internal Aabb Aabb => _aabb;
    internal Capsule Capsule => _capsule;
    internal Obb Obb => _obb;
    internal Polygon Polygon => _polygon!;
    public Vec2 PolygonTranslation => Kind == ShapeKind.Polygon
        ? _polygonTranslation
        : throw new InvalidOperationException("Shape is not a polygon.");

    public Angle32 PolygonRotation => Kind == ShapeKind.Polygon
        ? _polygonRotation
        : throw new InvalidOperationException("Shape is not a polygon.");

    private Aabb PolygonBounds() => _polygonRotation.Raw == 0
        ? _polygon!.Bounds.Moved(_polygonTranslation)
        : new BpBounds(_polygon!, _polygonTranslation, _polygonRotation).ToAabb();

    public static implicit operator Shape(Circle value) => new(value);
    public static implicit operator Shape(Aabb value) => new(value);
    public static implicit operator Shape(Capsule value) => new(value);
    public static implicit operator Shape(Obb value) => new(value);
    public static implicit operator Shape(Polygon value) => new(value);
}
