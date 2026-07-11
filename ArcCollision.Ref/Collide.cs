using System;

namespace ArcCollision;

/// <summary>
/// Discrete (static) overlap tests. Every function returns a <see cref="Manifold"/>
/// whose normal points from the first shape towards the second.
///
/// The math runs on an all-integer 24.8 fixed-point core: float arguments are
/// scaled by 256 at the boundary, everything inside is integer arithmetic
/// (64-bit, with scaled long products for degree-four expressions), and results are scaled back by
/// 1/256 on return. This is the correctness reference: implementations favour
/// clarity and well defined degenerate-case behaviour over raw speed.
/// </summary>
public static class Collide
{
    // ---------------------------------------------------------------- Point

    public static bool PointInCircle(Vec2 p, Circle c)
    {
        FxVec2 pf = FxVec2.From(p);
        FxCircle cf = FxCircle.From(c);
        return pf.DistSq(cf.Center) <= cf.Radius * cf.Radius;
    }

    public static bool PointInAabb(Vec2 p, Aabb box)
    {
        FxVec2 pf = FxVec2.From(p);
        FxAabb bf = FxAabb.From(box);
        return Math.Abs(pf.X - bf.Center.X) <= bf.Half.X
            && Math.Abs(pf.Y - bf.Center.Y) <= bf.Half.Y;
    }

    public static bool PointInCapsule(Vec2 p, Capsule cap)
    {
        FxVec2 pf = FxVec2.From(p);
        FxVec2 closest = Distance.ClosestPointOnSegmentFx(pf, FxVec2.From(cap.A), FxVec2.From(cap.B), out _);
        long r = Fx.From(cap.Radius);
        return pf.DistSq(closest) <= r * r;
    }

    // -------------------------------------------------------- Circle / Circle

    public static Manifold CircleVsCircle(Circle a, Circle b) =>
        CircleVsCircleFx(FxCircle.From(a), FxCircle.From(b)).ToManifold();

    internal static FxManifold CircleVsCircleFx(FxCircle a, FxCircle b)
    {
        FxVec2 delta = b.Center - a.Center;
        long r = a.Radius + b.Radius;
        long distSq = delta.LengthSq;
        if (distSq > r * r)
            return FxManifold.None;

        long dist = Fx.Sqrt(distSq);
        // Degenerate: concentric circles -> pick an arbitrary but stable axis.
        FxVec2 normal = dist > 0 ? delta.NormalizedFx(FxVec2.UnitX) : FxVec2.UnitX;
        long depth = r - dist;
        FxVec2 contact = a.Center + normal.MulUnit(a.Radius - depth / 2);
        return new FxManifold(true, normal, depth, contact);
    }

    // ------------------------------------------------------------ Aabb / Aabb

    public static Manifold AabbVsAabb(Aabb a, Aabb b) =>
        AabbVsAabbFx(FxAabb.From(a), FxAabb.From(b)).ToManifold();

    internal static FxManifold AabbVsAabbFx(FxAabb a, FxAabb b)
    {
        FxVec2 delta = b.Center - a.Center;
        long overlapX = (a.Half.X + b.Half.X) - Math.Abs(delta.X);
        if (overlapX <= 0)
            return FxManifold.None;
        long overlapY = (a.Half.Y + b.Half.Y) - Math.Abs(delta.Y);
        if (overlapY <= 0)
            return FxManifold.None;

        // Resolve along the axis of least penetration.
        if (overlapX < overlapY)
        {
            long sign = delta.X < 0 ? -1 : 1;
            var normal = new FxVec2(sign * Fx.One, 0);
            long contactX = a.Center.X + sign * a.Half.X;
            return new FxManifold(true, normal, overlapX, new FxVec2(contactX, b.Center.Y));
        }
        else
        {
            long sign = delta.Y < 0 ? -1 : 1;
            var normal = new FxVec2(0, sign * Fx.One);
            long contactY = a.Center.Y + sign * a.Half.Y;
            return new FxManifold(true, normal, overlapY, new FxVec2(b.Center.X, contactY));
        }
    }

    // ---------------------------------------------------------- Circle / Aabb

    public static Manifold CircleVsAabb(Circle c, Aabb box) =>
        CircleVsAabbFx(FxCircle.From(c), FxAabb.From(box)).ToManifold();

    internal static FxManifold CircleVsAabbFx(FxCircle c, FxAabb box)
    {
        FxVec2 closest = Distance.ClosestPointOnAabbFx(c.Center, box);
        FxVec2 delta = closest - c.Center;
        long distSq = delta.LengthSq;

        if (distSq > c.Radius * c.Radius)
            return FxManifold.None;

        if (distSq > 0)
        {
            // Center is outside the box: normal points to the nearest face/corner.
            long dist = Fx.Sqrt(distSq);
            FxVec2 normal = delta.NormalizedFx(FxVec2.UnitX);
            long depth = c.Radius - dist;
            return new FxManifold(true, normal, depth, closest);
        }

        // Center is inside the box: eject along the nearest face. `out` is the
        // outward face direction; the normal stays A->B (towards the box centre)
        // so SeparationForA = -normal*depth pushes the circle out that face,
        // matching the outside branch above.
        FxVec2 d = c.Center - box.Center;
        long overlapX = box.Half.X - Math.Abs(d.X);
        long overlapY = box.Half.Y - Math.Abs(d.Y);
        if (overlapX < overlapY)
        {
            long outSign = d.X < 0 ? -1 : 1;
            var normal = new FxVec2(-outSign * Fx.One, 0);
            long depth = overlapX + c.Radius;
            var contact = new FxVec2(box.Center.X + outSign * box.Half.X, c.Center.Y);
            return new FxManifold(true, normal, depth, contact);
        }
        else
        {
            long outSign = d.Y < 0 ? -1 : 1;
            var normal = new FxVec2(0, -outSign * Fx.One);
            long depth = overlapY + c.Radius;
            var contact = new FxVec2(c.Center.X, box.Center.Y + outSign * box.Half.Y);
            return new FxManifold(true, normal, depth, contact);
        }
    }

    // ------------------------------------------------------- Capsule variants

    public static Manifold CircleVsCapsule(Circle c, Capsule cap)
    {
        FxCircle cf = FxCircle.From(c);
        FxVec2 closest = Distance.ClosestPointOnSegmentFx(
            cf.Center, FxVec2.From(cap.A), FxVec2.From(cap.B), out _);
        // Treat the closest point on the spine as a circle of radius cap.Radius.
        return CircleVsCircleFx(cf, new FxCircle(closest, Fx.From(cap.Radius))).ToManifold();
    }

    public static Manifold CapsuleVsCapsule(Capsule a, Capsule b)
    {
        Distance.ClosestPointsSegmentSegmentFx(
            FxVec2.From(a.A), FxVec2.From(a.B), FxVec2.From(b.A), FxVec2.From(b.B),
            out FxVec2 c1, out FxVec2 c2);
        return CircleVsCircleFx(
            new FxCircle(c1, Fx.From(a.Radius)),
            new FxCircle(c2, Fx.From(b.Radius))).ToManifold();
    }

    public static Manifold CapsuleVsAabb(Capsule cap, Aabb box)
    {
        // Sample the closest point on the spine to the box centre, then reduce
        // to a circle-vs-box test. Adequate for the shapes arcade games use.
        FxAabb bf = FxAabb.From(box);
        FxVec2 spinePoint = Distance.ClosestPointOnSegmentFx(
            bf.Center, FxVec2.From(cap.A), FxVec2.From(cap.B), out _);
        return CircleVsAabbFx(new FxCircle(spinePoint, Fx.From(cap.Radius)), bf).ToManifold();
    }
}
