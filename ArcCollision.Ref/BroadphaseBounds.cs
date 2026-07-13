using System;

namespace ArcCollision;

/// <summary>
/// Broadphase axis-aligned bounds in the 24.8 fixed-point grid, stored as int32
/// min/max. int32 (16 bytes) keeps <see cref="DynamicAabbTree"/> / <see cref="StaticBvh"/>
/// nodes cache-dense — the real broadphase win, since traversal is memory-bound.
/// This holds because the ±<see cref="Fx.MaxInput"/> coordinate limit keeps every
/// bound within int32 (position ± extent, ×256). Derived spans (max − min, up to
/// ~2^31) widen to long for <see cref="Perimeter"/> / <see cref="CenterX"/> so they
/// never overflow. Kept exact so tree structure and pair enumeration stay
/// deterministic; mirrors the native arc_bp_bounds.
/// </summary>
public readonly struct BpBounds
{
    public readonly int MinX, MinY, MaxX, MaxY;

    public BpBounds(long minX, long minY, long maxX, long maxY)
    {
        MinX = (int)minX;
        MinY = (int)minY;
        MaxX = (int)maxX;
        MaxY = (int)maxY;
    }

    public BpBounds(Aabb box)
    {
        long x = Fx.From(box.Center.X);
        long y = Fx.From(box.Center.Y);
        long hx = Math.Abs(Fx.From(box.HalfExtents.X));
        long hy = Math.Abs(Fx.From(box.HalfExtents.Y));
        MinX = (int)(x - hx);
        MinY = (int)(y - hy);
        MaxX = (int)(x + hx);
        MaxY = (int)(y + hy);
    }

    public BpBounds(Circle circle)
    {
        long x = Fx.From(circle.Center.X);
        long y = Fx.From(circle.Center.Y);
        long r = Math.Abs(Fx.From(circle.Radius));
        MinX = (int)(x - r);
        MinY = (int)(y - r);
        MaxX = (int)(x + r);
        MaxY = (int)(y + r);
    }

    public BpBounds(Capsule capsule)
    {
        long ax = Fx.From(capsule.A.X);
        long ay = Fx.From(capsule.A.Y);
        long bx = Fx.From(capsule.B.X);
        long by = Fx.From(capsule.B.Y);
        long r = Math.Abs(Fx.From(capsule.Radius));
        MinX = (int)(Math.Min(ax, bx) - r);
        MinY = (int)(Math.Min(ay, by) - r);
        MaxX = (int)(Math.Max(ax, bx) + r);
        MaxY = (int)(Math.Max(ay, by) + r);
    }

    public BpBounds(Obb box)
    {
        long x = Fx.From(box.Center.X);
        long y = Fx.From(box.Center.Y);
        long halfX = Math.Abs(Fx.From(box.HalfExtents.X));
        long halfY = Math.Abs(Fx.From(box.HalfExtents.Y));
        FxAxis axisX = FxAxis.FromAngle(box.Angle);
        FxAxis axisY = axisX.Perpendicular;
        long extentX = Fx.CeilDivPositive(
            Math.Abs(axisX.X) * halfX + Math.Abs(axisY.X) * halfY, FxAxis.One);
        long extentY = Fx.CeilDivPositive(
            Math.Abs(axisX.Y) * halfX + Math.Abs(axisY.Y) * halfY, FxAxis.One);
        MinX = (int)(x - extentX);
        MinY = (int)(y - extentY);
        MaxX = (int)(x + extentX);
        MaxY = (int)(y + extentY);
    }

    public BpBounds(Polygon polygon)
        : this(polygon, Vec2.Zero, new Angle32(0))
    {
    }

    public BpBounds(Polygon polygon, Vec2 translation, Angle32 rotation)
    {
        ArgumentNullException.ThrowIfNull(polygon);
        FxVec2 offset = FxVec2.From(translation);
        if (rotation.Raw == 0)
        {
            MinX = (int)(polygon.MinXFx + offset.X);
            MinY = (int)(polygon.MinYFx + offset.Y);
            MaxX = (int)(polygon.MaxXFx + offset.X);
            MaxY = (int)(polygon.MaxYFx + offset.Y);
            return;
        }

        FxAxis axisX = FxAxis.FromAngle(rotation);
        FxAxis axisY = axisX.Perpendicular;
        FxVec2 first = Transform(polygon.FixedVertices[0], offset, axisX, axisY);
        long minX = first.X, minY = first.Y, maxX = first.X, maxY = first.Y;
        for (int i = 1; i < polygon.FixedVertices.Length; i++)
        {
            FxVec2 vertex = Transform(polygon.FixedVertices[i], offset, axisX, axisY);
            minX = Math.Min(minX, vertex.X);
            minY = Math.Min(minY, vertex.Y);
            maxX = Math.Max(maxX, vertex.X);
            maxY = Math.Max(maxY, vertex.Y);
        }
        MinX = (int)minX;
        MinY = (int)minY;
        MaxX = (int)maxX;
        MaxY = (int)maxY;
    }

    public BpBounds(Shape shape)
    {
        this = shape.Kind switch
        {
            ShapeKind.Circle => new BpBounds(shape.Circle),
            ShapeKind.Aabb => new BpBounds(shape.Aabb),
            ShapeKind.Capsule => new BpBounds(shape.Capsule),
            ShapeKind.Obb => new BpBounds(shape.Obb),
            ShapeKind.Polygon => new BpBounds(
                shape.Polygon, shape.PolygonTranslation, shape.PolygonRotation),
            _ => throw new ArgumentOutOfRangeException(nameof(shape)),
        };
    }

    // max − min can reach ~2^31, so widen before halving/summing.
    public int CenterX => (int)(MinX + (((long)MaxX - MinX) >> 1));
    public int CenterY => (int)(MinY + (((long)MaxY - MinY) >> 1));
    public long Perimeter => 2 * (((long)MaxX - MinX) + ((long)MaxY - MinY));

    public bool Overlaps(in BpBounds other) =>
        MinX <= other.MaxX && other.MinX <= MaxX
        && MinY <= other.MaxY && other.MinY <= MaxY;

    public bool Contains(in BpBounds other) =>
        MinX <= other.MinX && MinY <= other.MinY
        && MaxX >= other.MaxX && MaxY >= other.MaxY;

    public BpBounds Expanded(long margin) =>
        new(MinX - margin, MinY - margin, MaxX + margin, MaxY + margin);

    public BpBounds Translated(long x, long y) =>
        new(MinX + x, MinY + y, MaxX + x, MaxY + y);

    public static BpBounds Union(in BpBounds a, in BpBounds b) => new(
        Math.Min(a.MinX, b.MinX), Math.Min(a.MinY, b.MinY),
        Math.Max(a.MaxX, b.MaxX), Math.Max(a.MaxY, b.MaxY));

    public Aabb ToAabb() => Aabb.FromMinMax(
        new Vec2(Fx.To(MinX), Fx.To(MinY)),
        new Vec2(Fx.To(MaxX), Fx.To(MaxY)));

    private static FxVec2 Transform(
        FxVec2 vertex, FxVec2 offset, FxAxis axisX, FxAxis axisY) =>
        offset + axisX.Scale(vertex.X) + axisY.Scale(vertex.Y);
}
