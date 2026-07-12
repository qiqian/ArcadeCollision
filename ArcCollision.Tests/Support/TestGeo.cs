using System;
using System.Globalization;

namespace ArcCollision.Tests.Support;

/// <summary>
/// Double-precision reference geometry ("oracle") plus quantization helpers.
///
/// All oracle math operates on inputs that were first snapped to the library's
/// 1/256 grid, so the oracle and the integer implementation see identical
/// geometry. Remaining disagreement is bounded by the library's internal
/// rounding, which each test encodes as an explicit gray zone.
/// </summary>
internal static class TestGeo
{
    public const float Grid = 1f / 256f;

    /// <summary>Snap a float to the 24.8 grid exactly like Fx.From (round-half-even).</summary>
    public static float Q(float v) => (float)Math.Round(v * 256.0, MidpointRounding.ToEven) / 256f;

    /// <summary>The fixed-point integer a float converts to (test-side replica).</summary>
    public static long QFx(float v) => (long)Math.Round(v * 256.0, MidpointRounding.ToEven);

    public static Vec2 Q(Vec2 v) => new(Q(v.X), Q(v.Y));

    public static Circle Q(Circle c) => new(Q(c.Center), Q(c.Radius));
    public static Aabb Q(Aabb b) => new(Q(b.Center), Q(b.HalfExtents));
    public static Capsule Q(Capsule c) => new(Q(c.A), Q(c.B), Q(c.Radius));
    // OBB rotation is converted directly to a Q1.30 basis by the library, so the
    // angle itself does not use the 24.8 position grid.
    public static Obb Q(Obb o) => new(Q(o.Center), Q(o.HalfExtents), o.Angle);
    public static Polygon Q(Polygon p)
    {
        var verts = new Vec2[p.Count];
        for (int i = 0; i < p.Count; i++) verts[i] = Q(p[i]);
        return new Polygon(verts);
    }

    public static bool IsConvex(Polygon polygon)
    {
        int sign = 0;
        for (int i = 0; i < polygon.Count; i++)
        {
            Vec2 a = polygon[i];
            Vec2 b = polygon[(i + 1) % polygon.Count];
            Vec2 c = polygon[(i + 2) % polygon.Count];
            double cross = (double)(b.X - a.X) * (c.Y - a.Y)
                - (double)(b.Y - a.Y) * (c.X - a.X);
            if (cross == 0) continue;
            int current = cross > 0 ? 1 : -1;
            if (sign != 0 && sign != current) return false;
            sign = current;
        }
        return sign != 0;
    }

    // ------------------------------------------------------------ repro dump

    public static string Dump(Circle c) => $"new Circle(new Vec2({R(c.Center.X)}, {R(c.Center.Y)}), {R(c.Radius)})";
    public static string Dump(Aabb b) => $"new Aabb(new Vec2({R(b.Center.X)}, {R(b.Center.Y)}), new Vec2({R(b.HalfExtents.X)}, {R(b.HalfExtents.Y)}))";
    public static string Dump(Capsule c) => $"new Capsule(new Vec2({R(c.A.X)}, {R(c.A.Y)}), new Vec2({R(c.B.X)}, {R(c.B.Y)}), {R(c.Radius)})";
    public static string Dump(Obb o) => $"new Obb(new Vec2({R(o.Center.X)}, {R(o.Center.Y)}), new Vec2({R(o.HalfExtents.X)}, {R(o.HalfExtents.Y)}), new Angle32(0x{o.Angle.Raw:X8}u))";
    public static string Dump(Vec2 v) => $"new Vec2({R(v.X)}, {R(v.Y)})";
    public static string Dump(Polygon p)
    {
        var parts = new string[p.Count];
        for (int i = 0; i < p.Count; i++) parts[i] = Dump(p[i]);
        return $"new Polygon({string.Join(", ", parts)})";
    }
    private static string R(float v) => v.ToString("R", CultureInfo.InvariantCulture) + "f";

    // ---------------------------------------------------- double convex proxy

    /// <summary>
    /// A convex shape in double precision: a hull of 1..N points plus a radius
    /// (circle = 1 point, capsule = 2, boxes/convex polygons = N, radius 0).
    /// </summary>
    public sealed class DShape
    {
        public double[] Xs = Array.Empty<double>();
        public double[] Ys = Array.Empty<double>();
        public double R;

        public int Count => Xs.Length;

        public static DShape From(Circle c) => new()
        {
            Xs = new double[] { c.Center.X },
            Ys = new double[] { c.Center.Y },
            R = Math.Abs(c.Radius),
        };

        public static DShape From(Capsule c) => new()
        {
            Xs = new double[] { c.A.X, c.B.X },
            Ys = new double[] { c.A.Y, c.B.Y },
            R = Math.Abs(c.Radius),
        };

        public static DShape From(Aabb b)
        {
            double cx = b.Center.X, cy = b.Center.Y;
            double hx = Math.Abs(b.HalfExtents.X), hy = Math.Abs(b.HalfExtents.Y);
            return new DShape
            {
                Xs = new[] { cx - hx, cx + hx, cx + hx, cx - hx },
                Ys = new[] { cy - hy, cy - hy, cy + hy, cy + hy },
            };
        }

        /// <summary>
        /// Builds the independent double-precision OBB used by the oracle.
        /// </summary>
        public static DShape From(Obb o)
        {
            (double ax, double ay) = QuantizedAxis(o.Rotation);
            double px = -ay, py = ax;
            double cx = o.Center.X, cy = o.Center.Y;
            double hx = Math.Abs(o.HalfExtents.X), hy = Math.Abs(o.HalfExtents.Y);
            return new DShape
            {
                Xs = new[]
                {
                    cx - ax * hx - px * hy, cx + ax * hx - px * hy,
                    cx + ax * hx + px * hy, cx - ax * hx + px * hy,
                },
                Ys = new[]
                {
                    cy - ay * hx - py * hy, cy + ay * hx - py * hy,
                    cy + ay * hx + py * hy, cy - ay * hx + py * hy,
                },
            };
        }

        public static DShape From(Polygon p)
        {
            var xs = new double[p.Count];
            var ys = new double[p.Count];
            for (int i = 0; i < p.Count; i++)
            {
                xs[i] = Q(p[i].X);
                ys[i] = Q(p[i].Y);
            }
            return new DShape { Xs = xs, Ys = ys };
        }
    }

    /// <summary>The ideal double-precision OBB axis used by the oracle.</summary>
    public static (double X, double Y) QuantizedAxis(float rotation)
    {
        return (Math.Cos(rotation), Math.Sin(rotation));
    }

    /// <summary>The Q1.30 axis components produced at the rotation boundary.</summary>
    public static (long X, long Y) QuantizedAxisFx(float rotation)
    {
        const long one = 1L << 30;
        return ((long)Math.Round(Math.Cos(rotation) * one),
            (long)Math.Round(Math.Sin(rotation) * one));
    }

    /// <summary>
    /// Clearance oracle for CircleVsObb using the Q1.30 local-frame transform.
    /// </summary>
    public static double ClearanceCircleObb(Circle c, Obb o)
    {
        (long axX, long axY) = QuantizedAxisFx(o.Rotation);
        long dx = QFx(c.Center.X) - QFx(o.Center.X);
        long dy = QFx(c.Center.Y) - QFx(o.Center.Y);
        // Keep the local coordinate unrounded here; the caller's gray zone
        // accounts for the implementation's final 24.8 rounding.
        const double axisOne = 1L << 30;
        double lx = (dx * (double)axX + dy * (double)axY) / axisOne / 256.0;
        double ly = (dx * (double)-axY + dy * (double)axX) / axisOne / 256.0;
        double hx = Math.Abs(Q(o.HalfExtents.X));
        double hy = Math.Abs(Q(o.HalfExtents.Y));
        double ex = Math.Max(0, Math.Abs(lx) - hx);
        double ey = Math.Max(0, Math.Abs(ly) - hy);
        double outside = Math.Sqrt(ex * ex + ey * ey);
        double inside = Math.Min(hx - Math.Abs(lx), hy - Math.Abs(ly));
        double signed = outside > 0 ? outside : -inside;
        return signed - Math.Abs(Q(c.Radius));
    }

    private static long RoundDivD(long a, long b)
    {
        if (b < 0) { a = -a; b = -b; }
        long half = b >> 1;
        return a >= 0 ? (a + half) / b : -((-a + half) / b);
    }

    public static long ISqrt(long v)
    {
        if (v <= 0) return 0;
        long r = (long)Math.Sqrt(v);
        while (r > 0 && r * r > v) r--;
        while ((r + 1) * (r + 1) <= v) r++;
        return r;
    }

    // -------------------------------------------------------------- clearance

    /// <summary>
    /// Signed surface clearance between two convex shapes: positive = separated
    /// by that distance, negative = penetrating by that depth, zero = touching.
    /// For radius-carrying pairs (circle/capsule) this matches the library's
    /// closest-point reduction semantics; for hull pairs it is SAT depth.
    /// </summary>
    public static double Clearance(DShape a, DShape b)
    {
        // Pure radius pairs: exact segment distance, no SAT needed.
        if (a.Count <= 2 && b.Count <= 2)
        {
            double d = SegSegDistance(
                a.Xs[0], a.Ys[0], a.Xs[a.Count - 1], a.Ys[a.Count - 1],
                b.Xs[0], b.Ys[0], b.Xs[b.Count - 1], b.Ys[b.Count - 1]);
            return d - a.R - b.R;
        }

        bool overlapping = HullsOverlap(a, b, out double penetration);
        double signed = overlapping ? -penetration : BoundaryDistance(a, b);
        return signed - a.R - b.R;
    }

    private static double BoundaryDistance(DShape a, DShape b)
    {
        double best = double.MaxValue;
        int edgesA = EdgeCount(a), edgesB = EdgeCount(b);
        for (int i = 0; i < edgesA; i++)
        {
            (double p1x, double p1y, double q1x, double q1y) = Edge(a, i);
            for (int j = 0; j < edgesB; j++)
            {
                (double p2x, double p2y, double q2x, double q2y) = Edge(b, j);
                double d = SegSegDistance(p1x, p1y, q1x, q1y, p2x, p2y, q2x, q2y);
                if (d < best) best = d;
            }
        }
        return best;
    }

    private static int EdgeCount(DShape s) => s.Count switch { 1 => 1, 2 => 1, _ => s.Count };

    private static (double, double, double, double) Edge(DShape s, int i)
    {
        if (s.Count == 1) return (s.Xs[0], s.Ys[0], s.Xs[0], s.Ys[0]);
        if (s.Count == 2) return (s.Xs[0], s.Ys[0], s.Xs[1], s.Ys[1]);
        int j = (i + 1) % s.Count;
        return (s.Xs[i], s.Ys[i], s.Xs[j], s.Ys[j]);
    }

    /// <summary>SAT overlap + minimum translation depth for convex hulls
    /// (degenerate segments contribute their normal and direction axes).</summary>
    private static bool HullsOverlap(DShape a, DShape b, out double penetration)
    {
        penetration = double.MaxValue;

        if (a.Count == 1 && b.Count == 1)
        {
            bool same = a.Xs[0] == b.Xs[0] && a.Ys[0] == b.Ys[0];
            penetration = 0;
            return same;
        }

        Span<double> axes = stackalloc double[(a.Count + b.Count) * 4];
        int axisCount = 0;
        CollectAxes(a, axes, ref axisCount);
        CollectAxes(b, axes, ref axisCount);
        if (axisCount == 0) { penetration = 0; return false; }

        for (int i = 0; i < axisCount; i += 2)
        {
            double ax = axes[i], ay = axes[i + 1];
            Project(a, ax, ay, out double minA, out double maxA);
            Project(b, ax, ay, out double minB, out double maxB);
            double overlap = Math.Min(maxA - minB, maxB - minA);
            if (overlap < 0)
            {
                penetration = 0;
                return false;
            }
            if (overlap < penetration) penetration = overlap;
        }
        return true;
    }

    private static void CollectAxes(DShape s, Span<double> axes, ref int count)
    {
        int n = s.Count;
        if (n < 2) return;
        int edges = n == 2 ? 1 : n;
        for (int i = 0; i < edges; i++)
        {
            int j = (i + 1) % n;
            double ex = s.Xs[j] - s.Xs[i];
            double ey = s.Ys[j] - s.Ys[i];
            double len = Math.Sqrt(ex * ex + ey * ey);
            if (len < 1e-12) continue;
            ex /= len; ey /= len;
            axes[count++] = -ey; axes[count++] = ex;    // edge normal
            if (n == 2)
            {
                axes[count++] = ex; axes[count++] = ey; // segment direction too
            }
        }
    }

    private static void Project(DShape s, double ax, double ay, out double min, out double max)
    {
        min = double.MaxValue;
        max = double.MinValue;
        for (int i = 0; i < s.Count; i++)
        {
            double d = s.Xs[i] * ax + s.Ys[i] * ay;
            if (d < min) min = d;
            if (d > max) max = d;
        }
    }

    public static double SegSegDistance(
        double p1x, double p1y, double q1x, double q1y,
        double p2x, double p2y, double q2x, double q2y)
    {
        double d1x = q1x - p1x, d1y = q1y - p1y;
        double d2x = q2x - p2x, d2y = q2y - p2y;
        double rx = p1x - p2x, ry = p1y - p2y;
        double a = d1x * d1x + d1y * d1y;
        double e = d2x * d2x + d2y * d2y;
        double f = d2x * rx + d2y * ry;
        double s, t;
        const double eps = 1e-14;

        if (a <= eps && e <= eps) { s = t = 0; }
        else if (a <= eps) { s = 0; t = Math.Clamp(f / e, 0, 1); }
        else
        {
            double c = d1x * rx + d1y * ry;
            if (e <= eps) { t = 0; s = Math.Clamp(-c / a, 0, 1); }
            else
            {
                double bb = d1x * d2x + d1y * d2y;
                double denom = a * e - bb * bb;
                s = denom > eps ? Math.Clamp((bb * f - c * e) / denom, 0, 1) : 0;
                t = (bb * s + f) / e;
                if (t < 0) { t = 0; s = Math.Clamp(-c / a, 0, 1); }
                else if (t > 1) { t = 1; s = Math.Clamp((bb - c) / a, 0, 1); }
            }
        }

        double c1x = p1x + d1x * s, c1y = p1y + d1y * s;
        double c2x = p2x + d2x * t, c2y = p2y + d2y * t;
        double dx = c1x - c2x, dy = c1y - c2y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    // --------------------------------------------------------- contact oracle

    /// <summary>
    /// Double-precision contact point for Circle vs Circle.
    /// Contact sits on the line between centres at (radiusA - depth/2) from A.
    /// </summary>
    public static (double X, double Y) ContactCircleCircle(Circle a, Circle b)
    {
        double ax = Q(a.Center.X), ay = Q(a.Center.Y);
        double bx = Q(b.Center.X), by = Q(b.Center.Y);
        double ra = Math.Abs(Q(a.Radius)), rb = Math.Abs(Q(b.Radius));
        double dx = bx - ax, dy = by - ay;
        double dist = Math.Sqrt(dx * dx + dy * dy);
        double nx, ny;
        if (dist > 1e-12) { nx = dx / dist; ny = dy / dist; }
        else { nx = 1; ny = 0; }
        double depth = ra + rb - dist;
        double offset = ra - depth / 2.0;
        return (ax + nx * offset, ay + ny * offset);
    }

    /// <summary>
    /// Double-precision contact point for Circle vs AABB (centre outside box).
    /// Contact is the closest point on the AABB surface.
    /// </summary>
    public static (double X, double Y) ContactCircleAabb(Circle c, Aabb box)
    {
        double cx = Q(c.Center.X), cy = Q(c.Center.Y);
        double bcx = Q(box.Center.X), bcy = Q(box.Center.Y);
        double hx = Math.Abs(Q(box.HalfExtents.X)), hy = Math.Abs(Q(box.HalfExtents.Y));
        double closestX = Math.Clamp(cx, bcx - hx, bcx + hx);
        double closestY = Math.Clamp(cy, bcy - hy, bcy + hy);
        return (closestX, closestY);
    }

    /// <summary>
    /// Double-precision contact point for Circle vs Capsule.
    /// Closest point on the capsule spine to the circle centre, then
    /// midpoint of the two closest surface points (like CircleVsCircle).
    /// </summary>
    public static (double X, double Y) ContactCircleCapsule(Circle c, Capsule cap)
    {
        double cx = Q(c.Center.X), cy = Q(c.Center.Y);
        double ax = Q(cap.A.X), ay = Q(cap.A.Y);
        double bx = Q(cap.B.X), by = Q(cap.B.Y);
        double rc = Math.Abs(Q(c.Radius)), rk = Math.Abs(Q(cap.Radius));
        // Project centre onto spine
        double dx = bx - ax, dy = by - ay;
        double lenSq = dx * dx + dy * dy;
        double t = 0;
        if (lenSq > 1e-14)
            t = Math.Clamp(((cx - ax) * dx + (cy - ay) * dy) / lenSq, 0, 1);
        double spineX = ax + dx * t, spineY = ay + dy * t;
        // Now it's circle-vs-circle between (cx,cy,rc) and (spineX,spineY,rk)
        double ddx = spineX - cx, ddy = spineY - cy;
        double dist = Math.Sqrt(ddx * ddx + ddy * ddy);
        double nx, ny;
        if (dist > 1e-12) { nx = ddx / dist; ny = ddy / dist; }
        else { nx = 1; ny = 0; }
        double depth = rc + rk - dist;
        double offset = rc - depth / 2.0;
        return (cx + nx * offset, cy + ny * offset);
    }

    // --------------------------------------------------- BpBounds replicas

    public static (long MinX, long MinY, long MaxX, long MaxY) BoundsOf(Circle c)
    {
        long x = QFx(c.Center.X), y = QFx(c.Center.Y), r = Math.Abs(QFx(c.Radius));
        return (x - r, y - r, x + r, y + r);
    }

    public static (long MinX, long MinY, long MaxX, long MaxY) BoundsOf(Aabb b)
    {
        long x = QFx(b.Center.X), y = QFx(b.Center.Y);
        long hx = Math.Abs(QFx(b.HalfExtents.X)), hy = Math.Abs(QFx(b.HalfExtents.Y));
        return (x - hx, y - hy, x + hx, y + hy);
    }

    public static (long MinX, long MinY, long MaxX, long MaxY) BoundsOf(Capsule c)
    {
        long ax = QFx(c.A.X), ay = QFx(c.A.Y), bx = QFx(c.B.X), by = QFx(c.B.Y);
        long r = Math.Abs(QFx(c.Radius));
        return (Math.Min(ax, bx) - r, Math.Min(ay, by) - r,
                Math.Max(ax, bx) + r, Math.Max(ay, by) + r);
    }

    public static (long MinX, long MinY, long MaxX, long MaxY) BoundsOf(Obb o) => BoundsOf(o.Bounds);

    public static (long MinX, long MinY, long MaxX, long MaxY) BoundsOf(Polygon p)
    {
        long minX = long.MaxValue, minY = long.MaxValue, maxX = long.MinValue, maxY = long.MinValue;
        for (int i = 0; i < p.Count; i++)
        {
            long x = QFx(p[i].X), y = QFx(p[i].Y);
            minX = Math.Min(minX, x); minY = Math.Min(minY, y);
            maxX = Math.Max(maxX, x); maxY = Math.Max(maxY, y);
        }
        return (minX, minY, maxX, maxY);
    }

    public static bool BoundsOverlap(
        (long MinX, long MinY, long MaxX, long MaxY) a,
        (long MinX, long MinY, long MaxX, long MaxY) b) =>
        a.MinX <= b.MaxX && b.MinX <= a.MaxX && a.MinY <= b.MaxY && b.MinY <= a.MaxY;
}
