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

    [FieldOffset(20)] public readonly ShapeKind Kind;
    [FieldOffset(24)] private readonly Polygon? _polygon;

    public Shape(Circle value) { this = default; Kind = ShapeKind.Circle; _circle = value; }
    public Shape(Aabb value) { this = default; Kind = ShapeKind.Aabb; _aabb = value; }
    public Shape(Capsule value) { this = default; Kind = ShapeKind.Capsule; _capsule = value; }
    public Shape(Obb value) { this = default; Kind = ShapeKind.Obb; _obb = value; }
    public Shape(Polygon value)
    {
        ArgumentNullException.ThrowIfNull(value);
        this = default;
        Kind = ShapeKind.Polygon;
        _polygon = value;
    }

    public Aabb Bounds => Kind switch
    {
        ShapeKind.Circle => _circle.Bounds,
        ShapeKind.Aabb => _aabb,
        ShapeKind.Capsule => _capsule.Bounds,
        ShapeKind.Obb => _obb.Bounds,
        ShapeKind.Polygon => _polygon!.Bounds,
        _ => throw new InvalidOperationException("Invalid shape kind."),
    };

    public Shape Moved(Vec2 delta) => Kind switch
    {
        ShapeKind.Circle => _circle.Moved(delta),
        ShapeKind.Aabb => _aabb.Moved(delta),
        ShapeKind.Capsule => _capsule.Moved(delta),
        ShapeKind.Obb => _obb.Moved(delta),
        ShapeKind.Polygon => _polygon!.Moved(delta),
        _ => throw new InvalidOperationException("Invalid shape kind."),
    };

    internal Circle Circle => _circle;
    internal Aabb Aabb => _aabb;
    internal Capsule Capsule => _capsule;
    internal Obb Obb => _obb;
    internal Polygon Polygon => _polygon!;

    public static implicit operator Shape(Circle value) => new(value);
    public static implicit operator Shape(Aabb value) => new(value);
    public static implicit operator Shape(Capsule value) => new(value);
    public static implicit operator Shape(Obb value) => new(value);
    public static implicit operator Shape(Polygon value) => new(value);
}
