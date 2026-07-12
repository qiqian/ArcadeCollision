using System;

namespace ArcCollision;

/// <summary>
/// Broadphase axis-aligned bounds in the 24.8 fixed-point grid (integer min/max),
/// the currency of <see cref="DynamicAabbTree"/> and <see cref="StaticBvh"/>. Kept
/// as exact integers so tree structure and pair enumeration are deterministic.
/// </summary>
public readonly struct BpBounds
{
    public readonly long MinX, MinY, MaxX, MaxY;

    public BpBounds(long minX, long minY, long maxX, long maxY)
    {
        MinX = minX;
        MinY = minY;
        MaxX = maxX;
        MaxY = maxY;
    }

    public BpBounds(Aabb box)
    {
        long x = Fx.From(box.Center.X);
        long y = Fx.From(box.Center.Y);
        long hx = Math.Abs(Fx.From(box.HalfExtents.X));
        long hy = Math.Abs(Fx.From(box.HalfExtents.Y));
        MinX = x - hx;
        MinY = y - hy;
        MaxX = x + hx;
        MaxY = y + hy;
    }

    public BpBounds(Circle circle)
    {
        long x = Fx.From(circle.Center.X);
        long y = Fx.From(circle.Center.Y);
        long r = Math.Abs(Fx.From(circle.Radius));
        MinX = x - r;
        MinY = y - r;
        MaxX = x + r;
        MaxY = y + r;
    }

    public BpBounds(Capsule capsule)
    {
        long ax = Fx.From(capsule.A.X);
        long ay = Fx.From(capsule.A.Y);
        long bx = Fx.From(capsule.B.X);
        long by = Fx.From(capsule.B.Y);
        long r = Math.Abs(Fx.From(capsule.Radius));
        MinX = Math.Min(ax, bx) - r;
        MinY = Math.Min(ay, by) - r;
        MaxX = Math.Max(ax, bx) + r;
        MaxY = Math.Max(ay, by) + r;
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
        MinX = x - extentX;
        MinY = y - extentY;
        MaxX = x + extentX;
        MaxY = y + extentY;
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
            MinX = polygon.MinXFx + offset.X;
            MinY = polygon.MinYFx + offset.Y;
            MaxX = polygon.MaxXFx + offset.X;
            MaxY = polygon.MaxYFx + offset.Y;
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
        MinX = minX;
        MinY = minY;
        MaxX = maxX;
        MaxY = maxY;
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

    public long CenterX => MinX + ((MaxX - MinX) >> 1);
    public long CenterY => MinY + ((MaxY - MinY) >> 1);
    public long Perimeter => 2 * ((MaxX - MinX) + (MaxY - MinY));

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
