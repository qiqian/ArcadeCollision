using System;

namespace ArcCollision;

/// <summary>A circle defined by a center point and radius.</summary>
public readonly struct Circle
{
    public readonly Vec2 Center;
    public readonly float Radius;

    public Circle(Vec2 center, float radius)
    {
        Center = center;
        Radius = radius;
    }

    public Aabb Bounds => new(Center, new Vec2(Radius, Radius));
    public Circle Moved(Vec2 delta) => new(Center + delta, Radius);
}

/// <summary>
/// Axis-aligned bounding box stored as center + half-extents. This form keeps
/// the collision math branch-light and matches the C implementation.
/// </summary>
public readonly struct Aabb
{
    public readonly Vec2 Center;
    public readonly Vec2 HalfExtents;

    public Aabb(Vec2 center, Vec2 halfExtents)
    {
        Center = center;
        HalfExtents = halfExtents;
    }

    public Vec2 Min => new(Center.X - HalfExtents.X, Center.Y - HalfExtents.Y);
    public Vec2 Max => new(Center.X + HalfExtents.X, Center.Y + HalfExtents.Y);

    public static Aabb FromMinMax(Vec2 min, Vec2 max)
    {
        var center = (min + max) * 0.5f;
        var half = (max - min) * 0.5f;
        return new Aabb(center, half);
    }

    public Aabb Moved(Vec2 delta) => new(Center + delta, HalfExtents);

    /// <summary>Expand the box by <paramref name="amount"/> on every side.</summary>
    public Aabb Expanded(float amount) => new(Center, new Vec2(HalfExtents.X + amount, HalfExtents.Y + amount));

    /// <summary>Smallest box containing both inputs (broadphase union).</summary>
    public static Aabb Union(Aabb a, Aabb b) => FromMinMax(Vec2.Min(a.Min, b.Min), Vec2.Max(a.Max, b.Max));

    public bool Overlaps(Aabb b)
    {
        // Integer core: quantize to 24.8 fixed point and compare exactly.
        return Math.Abs(Fx.From(Center.X) - Fx.From(b.Center.X))
                <= Fx.From(HalfExtents.X) + Fx.From(b.HalfExtents.X)
            && Math.Abs(Fx.From(Center.Y) - Fx.From(b.Center.Y))
                <= Fx.From(HalfExtents.Y) + Fx.From(b.HalfExtents.Y);
    }
}

/// <summary>An oriented box defined by center, half-extents and radians.</summary>
public readonly struct Obb
{
    public readonly Vec2 Center;
    public readonly Vec2 HalfExtents;
    public readonly float Rotation;

    public Obb(Vec2 center, Vec2 halfExtents, float rotation = 0f)
    {
        Center = center;
        HalfExtents = halfExtents;
        Rotation = rotation;
    }

    public Aabb Bounds
    {
        get
        {
            float c = MathF.Cos(Rotation);
            float s = MathF.Sin(Rotation);
            float hx = MathF.Abs(HalfExtents.X);
            float hy = MathF.Abs(HalfExtents.Y);
            var half = new Vec2(
                MathF.Abs(c) * hx + MathF.Abs(s) * hy,
                MathF.Abs(s) * hx + MathF.Abs(c) * hy);
            return new Aabb(Center, half);
        }
    }

    public Obb Moved(Vec2 delta) => new(Center + delta, HalfExtents, Rotation);
}

/// <summary>
/// Immutable polygon with cached floating-point and fixed-point bounds. The
/// broadphase accepts convex or concave polygons; narrowphase interpretation is
/// intentionally outside this shape's responsibility.
/// </summary>
public sealed class Polygon
{
    private readonly Vec2[] _vertices;
    private readonly FxVec2[] _fixedVertices;
    private readonly Aabb _bounds;

    internal readonly long MinXFx, MinYFx, MaxXFx, MaxYFx;
    internal readonly int[] TriangleIndices;
    internal readonly bool IsConvex;

    public Polygon(params Vec2[] vertices)
        : this(vertices.AsSpan())
    {
    }

    public Polygon(ReadOnlySpan<Vec2> vertices)
    {
        if (vertices.Length < 3)
            throw new ArgumentException("A polygon requires at least three vertices.", nameof(vertices));

        _vertices = vertices.ToArray();
        _fixedVertices = new FxVec2[vertices.Length];
        Vec2 first = vertices[0];
        float minX = first.X, minY = first.Y, maxX = first.X, maxY = first.Y;
        long minXFx = Fx.From(first.X), minYFx = Fx.From(first.Y);
        long maxXFx = minXFx, maxYFx = minYFx;
        _fixedVertices[0] = new FxVec2(minXFx, minYFx);

        for (int i = 1; i < vertices.Length; i++)
        {
            Vec2 vertex = vertices[i];
            minX = MathF.Min(minX, vertex.X);
            minY = MathF.Min(minY, vertex.Y);
            maxX = MathF.Max(maxX, vertex.X);
            maxY = MathF.Max(maxY, vertex.Y);
            long x = Fx.From(vertex.X);
            long y = Fx.From(vertex.Y);
            _fixedVertices[i] = new FxVec2(x, y);
            minXFx = Math.Min(minXFx, x);
            minYFx = Math.Min(minYFx, y);
            maxXFx = Math.Max(maxXFx, x);
            maxYFx = Math.Max(maxYFx, y);
        }

        MinXFx = minXFx;
        MinYFx = minYFx;
        MaxXFx = maxXFx;
        MaxYFx = maxYFx;
        _bounds = Aabb.FromMinMax(new Vec2(minX, minY), new Vec2(maxX, maxY));
        IsConvex = ComputeConvexity(_fixedVertices);
        TriangleIndices = IsConvex ? Array.Empty<int>() : Triangulate(_fixedVertices);
    }

    public int Count => _vertices.Length;
    public Vec2 this[int index] => _vertices[index];
    public ReadOnlySpan<Vec2> Vertices => _vertices;
    public Aabb Bounds => _bounds;
    internal FxVec2[] FixedVertices => _fixedVertices;

    public Polygon Moved(Vec2 delta)
    {
        var moved = new Vec2[_vertices.Length];
        for (int i = 0; i < moved.Length; i++)
            moved[i] = _vertices[i] + delta;
        return new Polygon(moved);
    }

    private static bool ComputeConvexity(FxVec2[] vertices)
    {
        int sign = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            long cross = Cross(vertices[i], vertices[(i + 1) % vertices.Length],
                vertices[(i + 2) % vertices.Length]);
            if (cross == 0) continue;
            int current = cross > 0 ? 1 : -1;
            if (sign != 0 && current != sign) return false;
            sign = current;
        }
        if (sign == 0)
            throw new ArgumentException("Polygon vertices must enclose a non-zero area.", nameof(vertices));
        return true;
    }

    private static int[] Triangulate(FxVec2[] vertices)
    {
        long signedArea = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            FxVec2 a = vertices[i];
            FxVec2 b = vertices[(i + 1) % vertices.Length];
            signedArea += a.X * b.Y - a.Y * b.X;
        }
        int winding = signedArea >= 0 ? 1 : -1;

        int[] remaining = new int[vertices.Length];
        for (int i = 0; i < remaining.Length; i++) remaining[i] = i;
        int remainingCount = remaining.Length;
        int[] triangles = new int[(vertices.Length - 2) * 3];
        int output = 0;

        while (remainingCount > 3)
        {
            bool clipped = false;
            for (int i = 0; i < remainingCount; i++)
            {
                int previous = remaining[(i + remainingCount - 1) % remainingCount];
                int current = remaining[i];
                int next = remaining[(i + 1) % remainingCount];
                long corner = Cross(vertices[previous], vertices[current], vertices[next]);
                if ((winding > 0 && corner <= 0) || (winding < 0 && corner >= 0))
                    continue;

                bool containsVertex = false;
                for (int j = 0; j < remainingCount; j++)
                {
                    int candidate = remaining[j];
                    if (candidate == previous || candidate == current || candidate == next)
                        continue;
                    if (PointInTriangle(vertices[candidate], vertices[previous],
                            vertices[current], vertices[next], winding))
                    {
                        containsVertex = true;
                        break;
                    }
                }
                if (containsVertex) continue;

                triangles[output++] = previous;
                triangles[output++] = current;
                triangles[output++] = next;
                Array.Copy(remaining, i + 1, remaining, i, remainingCount - i - 1);
                remainingCount--;
                clipped = true;
                break;
            }

            if (!clipped)
                throw new ArgumentException("Polygon must be simple and non-self-intersecting.", nameof(vertices));
        }

        triangles[output++] = remaining[0];
        triangles[output++] = remaining[1];
        triangles[output] = remaining[2];
        return triangles;
    }

    private static bool PointInTriangle(
        FxVec2 point, FxVec2 a, FxVec2 b, FxVec2 c, int winding)
    {
        long ab = Cross(a, b, point);
        long bc = Cross(b, c, point);
        long ca = Cross(c, a, point);
        return winding > 0
            ? ab >= 0 && bc >= 0 && ca >= 0
            : ab <= 0 && bc <= 0 && ca <= 0;
    }

    private static long Cross(FxVec2 a, FxVec2 b, FxVec2 c)
    {
        long abX = b.X - a.X;
        long abY = b.Y - a.Y;
        long acX = c.X - a.X;
        long acY = c.Y - a.Y;
        return abX * acY - abY * acX;
    }
}

/// <summary>
/// A capsule: the set of points within <see cref="Radius"/> of the segment
/// A-B. Great for characters, sword swings and thick projectiles.
/// </summary>
public readonly struct Capsule
{
    public readonly Vec2 A;
    public readonly Vec2 B;
    public readonly float Radius;

    public Capsule(Vec2 a, Vec2 b, float radius)
    {
        A = a;
        B = b;
        Radius = radius;
    }

    public Aabb Bounds
    {
        get
        {
            var min = Vec2.Min(A, B) - new Vec2(Radius, Radius);
            var max = Vec2.Max(A, B) + new Vec2(Radius, Radius);
            return Aabb.FromMinMax(min, max);
        }
    }

    public Capsule Moved(Vec2 delta) => new(A + delta, B + delta, Radius);
}
