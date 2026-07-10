using System;

namespace ArcCollision;

/// <summary>Geometric distance / closest-point helpers shared by the tests.</summary>
public static class Distance
{
    /// <summary>
    /// Closest point on segment [a, b] to <paramref name="p"/>. Also outputs the
    /// clamped parameter <paramref name="t"/> in [0, 1] such that the result is
    /// <c>a + (b - a) * t</c>.
    /// </summary>
    public static Vec2 ClosestPointOnSegment(Vec2 p, Vec2 a, Vec2 b, out float t)
    {
        Vec2 ab = b - a;
        float lenSq = ab.LengthSquared;
        if (lenSq < 1e-12f)
        {
            t = 0f;
            return a;
        }
        t = (p - a).Dot(ab) / lenSq;
        t = Math.Clamp(t, 0f, 1f);
        return a + ab * t;
    }

    public static Vec2 ClosestPointOnSegment(Vec2 p, Vec2 a, Vec2 b) => ClosestPointOnSegment(p, a, b, out _);

    /// <summary>Closest point on (or inside) an AABB to <paramref name="p"/>.</summary>
    public static Vec2 ClosestPointOnAabb(Vec2 p, Aabb box)
    {
        Vec2 min = box.Min;
        Vec2 max = box.Max;
        return new Vec2(
            Math.Clamp(p.X, min.X, max.X),
            Math.Clamp(p.Y, min.Y, max.Y));
    }

    /// <summary>
    /// Shortest distance (squared) between two segments. Outputs the closest
    /// point on each. Used by capsule-vs-capsule.
    /// </summary>
    public static float ClosestPointsSegmentSegment(
        Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2, out Vec2 c1, out Vec2 c2)
    {
        Vec2 d1 = q1 - p1;
        Vec2 d2 = q2 - p2;
        Vec2 r = p1 - p2;
        float a = d1.LengthSquared;
        float e = d2.LengthSquared;
        float f = d2.Dot(r);

        float s, t;
        const float eps = 1e-12f;

        if (a <= eps && e <= eps)
        {
            s = t = 0f;
        }
        else if (a <= eps)
        {
            s = 0f;
            t = Math.Clamp(f / e, 0f, 1f);
        }
        else
        {
            float c = d1.Dot(r);
            if (e <= eps)
            {
                t = 0f;
                s = Math.Clamp(-c / a, 0f, 1f);
            }
            else
            {
                float b = d1.Dot(d2);
                float denom = a * e - b * b;
                s = denom > eps ? Math.Clamp((b * f - c * e) / denom, 0f, 1f) : 0f;
                t = (b * s + f) / e;
                if (t < 0f)
                {
                    t = 0f;
                    s = Math.Clamp(-c / a, 0f, 1f);
                }
                else if (t > 1f)
                {
                    t = 1f;
                    s = Math.Clamp((b - c) / a, 0f, 1f);
                }
            }
        }

        c1 = p1 + d1 * s;
        c2 = p2 + d2 * t;
        return c1.DistanceSquared(c2);
    }
}
