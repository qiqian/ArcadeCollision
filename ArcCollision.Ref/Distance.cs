using System;

namespace ArcCollision.Ref;

/// <summary>
/// Geometric distance / closest-point helpers shared by the tests. Float
/// arguments are converted to 24.8 fixed point at the boundary and all interior
/// math is integer; segment parameters are computed in 16.16.
/// </summary>
public static class Distance
{
    /// <summary>
    /// Closest point on segment [a, b] to <paramref name="p"/>. Also outputs the
    /// clamped parameter <paramref name="t"/> in [0, 1] such that the result is
    /// <c>a + (b - a) * t</c>.
    /// </summary>
    public static Vec2 ClosestPointOnSegment(Vec2 p, Vec2 a, Vec2 b, out float t)
    {
        FxVec2 closest = ClosestPointOnSegmentFx(
            FxVec2.From(p), FxVec2.From(a), FxVec2.From(b), out long t16);
        t = Fx.ToT(t16);
        return closest.ToVec2();
    }

    public static Vec2 ClosestPointOnSegment(Vec2 p, Vec2 a, Vec2 b) => ClosestPointOnSegment(p, a, b, out _);

    internal static FxVec2 ClosestPointOnSegmentFx(FxVec2 p, FxVec2 a, FxVec2 b, out long t16)
    {
        FxVec2 ab = b - a;
        long lenSq = ab.LengthSq;              // squared scale (2^16)
        if (lenSq == 0)
        {
            t16 = 0;
            return a;
        }
        // t = (p-a)·ab / |ab|² as a 16.16 parameter.
        long dot = (p - a).Dot(ab);
        t16 = Fx.ClampedParam(dot, lenSq);
        return a + ab.MulT(t16);
    }

    /// <summary>Closest point on (or inside) an AABB to <paramref name="p"/>.</summary>
    public static Vec2 ClosestPointOnAabb(Vec2 p, Aabb box) =>
        ClosestPointOnAabbFx(FxVec2.From(p), FxAabb.From(box)).ToVec2();

    internal static FxVec2 ClosestPointOnAabbFx(FxVec2 p, FxAabb box)
    {
        FxVec2 min = box.Min;
        FxVec2 max = box.Max;
        return new FxVec2(
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
        long distSq = ClosestPointsSegmentSegmentFx(
            FxVec2.From(p1), FxVec2.From(q1), FxVec2.From(p2), FxVec2.From(q2),
            out FxVec2 f1, out FxVec2 f2);
        c1 = f1.ToVec2();
        c2 = f2.ToVec2();
        return Fx.ToSq(distSq);
    }

    internal static long ClosestPointsSegmentSegmentFx(
        FxVec2 p1, FxVec2 q1, FxVec2 p2, FxVec2 q2, out FxVec2 c1, out FxVec2 c2)
    {
        FxVec2 d1 = q1 - p1;
        FxVec2 d2 = q2 - p2;
        FxVec2 r = p1 - p2;
        long a = d1.LengthSq;                  // squared scale
        long e = d2.LengthSq;
        long f = d2.Dot(r);

        long s16, t16;

        if (a == 0 && e == 0)
        {
            s16 = t16 = 0;
        }
        else if (a == 0)
        {
            s16 = 0;
            t16 = Fx.ClampedParam(f, e);
        }
        else
        {
            long c = d1.Dot(r);
            if (e == 0)
            {
                t16 = 0;
                s16 = Fx.ClampedParam(-c, a);
            }
            else
            {
                long b = d1.Dot(d2);
                // Degree-4 products (squared × squared scale) exceed 64 bits for
                // large coordinates, so operands are uniformly scaled first.
                int shift = Fx.ProductShift(a, b, c, e, f);
                long scaledA = Fx.ScaleProductOperand(a, shift);
                long scaledB = Fx.ScaleProductOperand(b, shift);
                long scaledC = Fx.ScaleProductOperand(c, shift);
                long scaledE = Fx.ScaleProductOperand(e, shift);
                long scaledF = Fx.ScaleProductOperand(f, shift);
                long denom = scaledA * scaledE - scaledB * scaledB;
                if (denom > 0)
                {
                    long numerator = scaledB * scaledF - scaledC * scaledE;
                    s16 = Fx.ClampedParam(numerator, denom);
                }
                else
                {
                    s16 = 0;
                }

                long projected = Fx.MulT(b, s16) + f;
                if (projected < 0)
                {
                    t16 = 0;
                    s16 = Fx.ClampedParam(-c, a);
                }
                else if (projected > e)
                {
                    t16 = Fx.TOne;
                    s16 = Fx.ClampedParam(b - c, a);
                }
                else
                {
                    t16 = Fx.ClampedParam(projected, e);
                }
            }
        }

        c1 = p1 + d1.MulT(s16);
        c2 = p2 + d2.MulT(t16);
        return c1.DistSq(c2);
    }
}
