using System;

namespace ArcCollision;

/// <summary>
/// Continuous (swept) collision tests. These prevent fast movers from
/// tunnelling through thin geometry by finding the first time-of-impact.
///
/// Ray/segment functions take an origin and a full displacement vector; the
/// returned <see cref="SweepHit.Time"/> is the fraction of that displacement at
/// first contact.
/// </summary>
public static class Sweep
{
    /// <summary>Ray (origin + d) versus circle. <paramref name="d"/> is the full motion.</summary>
    public static SweepHit RayVsCircle(Vec2 origin, Vec2 d, Circle circle)
    {
        // Solve |origin + t*d - center|^2 = r^2 for smallest t in [0,1].
        Vec2 m = origin - circle.Center;
        float a = d.LengthSquared;
        if (a < 1e-12f)
        {
            // No motion: treat as a static containment check.
            return m.LengthSquared <= circle.Radius * circle.Radius
                ? new SweepHit(true, 0f, m.Normalized(Vec2.UnitX), origin)
                : SweepHit.Miss;
        }

        float b = m.Dot(d);
        float c = m.LengthSquared - circle.Radius * circle.Radius;

        // Origin already inside and moving: contact at t=0.
        if (c <= 0f)
        {
            Vec2 n0 = m.Normalized(-d.Normalized(Vec2.UnitX));
            return new SweepHit(true, 0f, n0, origin);
        }

        float disc = b * b - a * c;
        if (disc < 0f)
            return SweepHit.Miss;

        float t = (-b - MathF.Sqrt(disc)) / a;
        if (t < 0f || t > 1f)
            return SweepHit.Miss;

        Vec2 point = origin + d * t;
        Vec2 normal = (point - circle.Center).Normalized(Vec2.UnitX);
        return new SweepHit(true, t, normal, point);
    }

    /// <summary>Ray (origin + d) versus AABB using the slab method.</summary>
    public static SweepHit RayVsAabb(Vec2 origin, Vec2 d, Aabb box)
    {
        Vec2 min = box.Min;
        Vec2 max = box.Max;

        float tMin = 0f;
        float tMax = 1f;
        Vec2 normal = Vec2.Zero;

        // X slab
        if (!Slab(origin.X, d.X, min.X, max.X, new Vec2(-1f, 0f), new Vec2(1f, 0f),
                  ref tMin, ref tMax, ref normal))
            return SweepHit.Miss;
        // Y slab
        if (!Slab(origin.Y, d.Y, min.Y, max.Y, new Vec2(0f, -1f), new Vec2(0f, 1f),
                  ref tMin, ref tMax, ref normal))
            return SweepHit.Miss;

        Vec2 point = origin + d * tMin;
        return new SweepHit(true, tMin, normal, point);
    }

    private static bool Slab(
        float origin, float dir, float slabMin, float slabMax,
        Vec2 nMin, Vec2 nMax, ref float tMin, ref float tMax, ref Vec2 normal)
    {
        const float eps = 1e-9f;
        if (MathF.Abs(dir) < eps)
        {
            // Parallel to the slab: miss if the origin is outside it.
            return origin >= slabMin && origin <= slabMax;
        }

        float inv = 1f / dir;
        float t1 = (slabMin - origin) * inv;
        float t2 = (slabMax - origin) * inv;
        Vec2 n1 = nMin;
        Vec2 n2 = nMax;
        if (t1 > t2)
        {
            (t1, t2) = (t2, t1);
            (n1, n2) = (n2, n1);
        }

        if (t1 > tMin)
        {
            tMin = t1;
            normal = n1;
        }
        if (t2 < tMax)
            tMax = t2;

        return tMin <= tMax;
    }

    /// <summary>
    /// A moving circle versus a static circle. Reduces to a ray-vs-circle test
    /// against a circle grown by the mover's radius (Minkowski sum).
    /// </summary>
    public static SweepHit MovingCircleVsCircle(Circle mover, Vec2 motion, Circle target)
    {
        var expanded = new Circle(target.Center, target.Radius + mover.Radius);
        SweepHit hit = RayVsCircle(mover.Center, motion, expanded);
        if (!hit.Hit)
            return SweepHit.Miss;

        // Report the contact point on the target surface, not the expanded one.
        Vec2 moverAt = mover.Center + motion * hit.Time;
        Vec2 point = moverAt - hit.Normal * mover.Radius;
        return new SweepHit(true, hit.Time, hit.Normal, point);
    }

    /// <summary>
    /// A moving circle versus a static AABB. The exact Minkowski sum is a
    /// rounded rectangle; we test the box expanded by the radius (faces) and the
    /// four corner circles, keeping the earliest impact. This is conservative and
    /// tunnel-free.
    /// </summary>
    public static SweepHit MovingCircleVsAabb(Circle mover, Vec2 motion, Aabb box)
    {
        float r = mover.Radius;
        Aabb expanded = box.Expanded(r);
        SweepHit best = SweepHit.Miss;

        SweepHit faceHit = RayVsAabb(mover.Center, motion, expanded);
        if (faceHit.Hit)
        {
            // Only accept the face hit when contact lies on an actual face region
            // (not the rounded corner zone), otherwise defer to the corner tests.
            Vec2 at = mover.Center + motion * faceHit.Time;
            Vec2 min = box.Min;
            Vec2 max = box.Max;
            bool cornerZone = (at.X < min.X || at.X > max.X) && (at.Y < min.Y || at.Y > max.Y);
            if (!cornerZone)
                best = faceHit;
        }

        // Corner circles.
        Span<Vec2> corners = stackalloc Vec2[4]
        {
            box.Min,
            new Vec2(box.Max.X, box.Min.Y),
            box.Max,
            new Vec2(box.Min.X, box.Max.Y),
        };
        foreach (Vec2 corner in corners)
        {
            SweepHit h = RayVsCircle(mover.Center, motion, new Circle(corner, r));
            if (h.Hit && (!best.Hit || h.Time < best.Time))
                best = h;
        }

        return best;
    }
}
