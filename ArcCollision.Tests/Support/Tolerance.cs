using System;

namespace ArcCollision.Tests.Support;

/// <summary>
/// Shared, documented tolerance model for the accuracy suite.
///
/// The integer core is exact for the primitive predicate paths (circle/circle,
/// circle/aabb, aabb/aabb). SAT axes use Q1.30, so direction error is negligible;
/// the remaining envelope is primarily the 24.8 position/vertex grid plus a
/// tiny size-relative term for Q30 projection rounding.
/// </summary>
internal static class Tol
{
    public const double Grid = 1.0 / 256.0;

    // Half-extent "reach" used to scale SAT tolerances. For SAT the relevant
    // quantity is how far a face projects, so the sum of both operands' extents.
    public static double Extent(Circle c) => Math.Abs(c.Radius);
    public static double Extent(Aabb b) => Math.Abs(b.HalfExtents.X) + Math.Abs(b.HalfExtents.Y);
    public static double Extent(Obb o) => Math.Abs(o.HalfExtents.X) + Math.Abs(o.HalfExtents.Y);
    public static double Extent(Capsule c) => (c.B - c.A).Length * 0.5 + Math.Abs(c.Radius);
    public static double Extent(Polygon p)
    {
        Aabb b = p.Bounds;
        return Math.Abs(b.HalfExtents.X) + Math.Abs(b.HalfExtents.Y);
    }

    /// <summary>Size-relative gray zone for a SAT / rotated-shape pair.</summary>
    public static double SatGray(double extentSum) => 6.0 / 256.0 + extentSum * 0.0000001;

    // -------------------------------------------------------- sliver filter

    /// <summary>
    /// A convex polygon's thinnest dimension: the minimum, over every edge
    /// normal, of the projection span of all vertices. A sliver (near-collinear
    /// vertices) has a min-width near zero, where both the integer SAT and the
    /// oracle become unreliable within the 1/256 quantization, so such shapes are
    /// excluded from the differential tests rather than papered over with a huge
    /// tolerance.
    /// </summary>
    public static double MinWidth(Polygon p)
    {
        double min = double.MaxValue;
        int n = p.Count;
        for (int i = 0; i < n; i++)
        {
            Vec2 a = p[i], b = p[(i + 1) % n];
            double ex = b.X - a.X, ey = b.Y - a.Y;
            double len = Math.Sqrt(ex * ex + ey * ey);
            if (len < 1e-9) continue;
            double nx = -ey / len, ny = ex / len;
            double mn = double.MaxValue, mx = double.MinValue;
            for (int j = 0; j < n; j++)
            {
                double d = p[j].X * nx + p[j].Y * ny;
                if (d < mn) mn = d;
                if (d > mx) mx = d;
            }
            if (mx - mn < min) min = mx - mn;
        }
        return min == double.MaxValue ? 0.0 : min;
    }

    /// <summary>True when a polygon is too thin for reliable SAT (a few grid cells).</summary>
    public static bool IsSliver(Polygon p) => MinWidth(p) < 8.0 / 256.0;
}
