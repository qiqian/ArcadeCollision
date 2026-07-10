using System;

namespace ArcCollision;

/// <summary>
/// Discrete (static) overlap tests. Every function returns a <see cref="Manifold"/>
/// whose normal points from the first shape towards the second.
///
/// This is the correctness reference: implementations favour clarity and well
/// defined degenerate-case behaviour over raw speed.
/// </summary>
public static class Collide
{
    // ---------------------------------------------------------------- Point

    public static bool PointInCircle(Vec2 p, Circle c) => p.DistanceSquared(c.Center) <= c.Radius * c.Radius;

    public static bool PointInAabb(Vec2 p, Aabb box)
    {
        Vec2 d = p - box.Center;
        return MathF.Abs(d.X) <= box.HalfExtents.X && MathF.Abs(d.Y) <= box.HalfExtents.Y;
    }

    public static bool PointInCapsule(Vec2 p, Capsule cap)
    {
        Vec2 closest = Distance.ClosestPointOnSegment(p, cap.A, cap.B);
        return p.DistanceSquared(closest) <= cap.Radius * cap.Radius;
    }

    // -------------------------------------------------------- Circle / Circle

    public static Manifold CircleVsCircle(Circle a, Circle b)
    {
        Vec2 delta = b.Center - a.Center;
        float r = a.Radius + b.Radius;
        float distSq = delta.LengthSquared;
        if (distSq > r * r)
            return Manifold.None;

        float dist = MathF.Sqrt(distSq);
        // Degenerate: concentric circles -> pick an arbitrary but stable axis.
        Vec2 normal = dist > 1e-6f ? delta * (1f / dist) : Vec2.UnitX;
        float depth = r - dist;
        Vec2 contact = a.Center + normal * (a.Radius - depth * 0.5f);
        return new Manifold(true, normal, depth, contact);
    }

    // ------------------------------------------------------------ Aabb / Aabb

    public static Manifold AabbVsAabb(Aabb a, Aabb b)
    {
        Vec2 delta = b.Center - a.Center;
        float overlapX = (a.HalfExtents.X + b.HalfExtents.X) - MathF.Abs(delta.X);
        if (overlapX <= 0f)
            return Manifold.None;
        float overlapY = (a.HalfExtents.Y + b.HalfExtents.Y) - MathF.Abs(delta.Y);
        if (overlapY <= 0f)
            return Manifold.None;

        // Resolve along the axis of least penetration.
        if (overlapX < overlapY)
        {
            float sign = delta.X < 0f ? -1f : 1f;
            var normal = new Vec2(sign, 0f);
            float contactX = a.Center.X + sign * a.HalfExtents.X;
            return new Manifold(true, normal, overlapX, new Vec2(contactX, b.Center.Y));
        }
        else
        {
            float sign = delta.Y < 0f ? -1f : 1f;
            var normal = new Vec2(0f, sign);
            float contactY = a.Center.Y + sign * a.HalfExtents.Y;
            return new Manifold(true, normal, overlapY, new Vec2(b.Center.X, contactY));
        }
    }

    // ---------------------------------------------------------- Circle / Aabb

    public static Manifold CircleVsAabb(Circle c, Aabb box)
    {
        Vec2 closest = Distance.ClosestPointOnAabb(c.Center, box);
        Vec2 delta = closest - c.Center;
        float distSq = delta.LengthSquared;

        if (distSq > c.Radius * c.Radius)
            return Manifold.None;

        if (distSq > 1e-12f)
        {
            // Center is outside the box: normal points to the nearest face/corner.
            float dist = MathF.Sqrt(distSq);
            Vec2 normal = delta * (1f / dist);
            float depth = c.Radius - dist;
            return new Manifold(true, normal, depth, closest);
        }

        // Center is inside the box: eject along the nearest face. `out` is the
        // outward face direction; the normal stays A->B (towards the box centre)
        // so SeparationForA = -normal*depth pushes the circle out that face,
        // matching the outside branch above.
        Vec2 d = c.Center - box.Center;
        float overlapX = box.HalfExtents.X - MathF.Abs(d.X);
        float overlapY = box.HalfExtents.Y - MathF.Abs(d.Y);
        if (overlapX < overlapY)
        {
            float outSign = d.X < 0f ? -1f : 1f;
            var normal = new Vec2(-outSign, 0f);
            float depth = overlapX + c.Radius;
            Vec2 contact = new(box.Center.X + outSign * box.HalfExtents.X, c.Center.Y);
            return new Manifold(true, normal, depth, contact);
        }
        else
        {
            float outSign = d.Y < 0f ? -1f : 1f;
            var normal = new Vec2(0f, -outSign);
            float depth = overlapY + c.Radius;
            Vec2 contact = new(c.Center.X, box.Center.Y + outSign * box.HalfExtents.Y);
            return new Manifold(true, normal, depth, contact);
        }
    }

    // ------------------------------------------------------- Capsule variants

    public static Manifold CircleVsCapsule(Circle c, Capsule cap)
    {
        Vec2 closest = Distance.ClosestPointOnSegment(c.Center, cap.A, cap.B);
        // Treat the closest point on the spine as a circle of radius cap.Radius.
        return CircleVsCircle(c, new Circle(closest, cap.Radius));
    }

    public static Manifold CapsuleVsCapsule(Capsule a, Capsule b)
    {
        Distance.ClosestPointsSegmentSegment(a.A, a.B, b.A, b.B, out Vec2 c1, out Vec2 c2);
        return CircleVsCircle(new Circle(c1, a.Radius), new Circle(c2, b.Radius));
    }

    public static Manifold CapsuleVsAabb(Capsule cap, Aabb box)
    {
        // Sample the closest point on the spine to the box centre, then reduce
        // to a circle-vs-box test. Adequate for the shapes arcade games use.
        Vec2 spinePoint = Distance.ClosestPointOnSegment(box.Center, cap.A, cap.B);
        return CircleVsAabb(new Circle(spinePoint, cap.Radius), box);
    }
}
