namespace ArcCollision.Wrapper;

public static class Distance
{
    public static Vec2 ClosestPointOnSegment(Vec2 p, Vec2 a, Vec2 b, out float t)
    {
        FixedValidation.Vec2(p);
        FixedValidation.Vec2(a);
        FixedValidation.Vec2(b);
        return NativeMethods.ClosestPointSegment(p, a, b, out t);
    }
    public static Vec2 ClosestPointOnSegment(Vec2 p, Vec2 a, Vec2 b) =>
        ClosestPointOnSegment(p, a, b, out _);
    public static Vec2 ClosestPointOnAabb(Vec2 p, Aabb box)
    {
        FixedValidation.Vec2(p);
        FixedValidation.Aabb(box);
        return NativeMethods.ClosestPointAabb(p, box);
    }
    public static float ClosestPointsSegmentSegment(
        Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2, out Vec2 c1, out Vec2 c2)
    {
        FixedValidation.Vec2(p1);
        FixedValidation.Vec2(q1);
        FixedValidation.Vec2(p2);
        FixedValidation.Vec2(q2);
        return NativeMethods.ClosestSegments(p1, q1, p2, q2, out c1, out c2);
    }
}

public static class Collide
{
    public static bool PointInCircle(Vec2 p, Circle c)
    {
        FixedValidation.Vec2(p);
        FixedValidation.Circle(c);
        return NativeMethods.PointCircle(p, c) != 0;
    }
    public static bool PointInAabb(Vec2 p, Aabb box)
    {
        FixedValidation.Vec2(p);
        FixedValidation.Aabb(box);
        return NativeMethods.PointAabb(p, box) != 0;
    }
    public static bool PointInCapsule(Vec2 p, Capsule cap)
    {
        FixedValidation.Vec2(p);
        FixedValidation.Capsule(cap);
        return NativeMethods.PointCapsule(p, cap) != 0;
    }
    public static Manifold CircleVsCircle(Circle a, Circle b)
    {
        FixedValidation.Circle(a);
        FixedValidation.Circle(b);
        return NativeMethods.CircleCircle(a, b);
    }
    public static Manifold AabbVsAabb(Aabb a, Aabb b)
    {
        FixedValidation.Aabb(a);
        FixedValidation.Aabb(b);
        return NativeMethods.AabbAabb(a, b);
    }
    public static Manifold CircleVsAabb(Circle c, Aabb box)
    {
        FixedValidation.Circle(c);
        FixedValidation.Aabb(box);
        return NativeMethods.CircleAabb(c, box);
    }
    public static Manifold CircleVsCapsule(Circle c, Capsule cap)
    {
        FixedValidation.Circle(c);
        FixedValidation.Capsule(cap);
        return NativeMethods.CircleCapsule(c, cap);
    }
    public static Manifold CapsuleVsCapsule(Capsule a, Capsule b)
    {
        FixedValidation.Capsule(a);
        FixedValidation.Capsule(b);
        return NativeMethods.CapsuleCapsule(a, b);
    }
    public static Manifold CapsuleVsAabb(Capsule cap, Aabb box)
    {
        FixedValidation.Capsule(cap);
        FixedValidation.Aabb(box);
        return NativeMethods.CapsuleAabb(cap, box);
    }
    public static Manifold ShapeVsShape(
        in Shape a, in Shape b, ManifoldFields fields = ManifoldFields.All)
    {
        FixedValidation.ManifoldMode(fields);
        FixedValidation.Shape(a);
        FixedValidation.Shape(b);
        NativeShape na = a.ToNative(), nb = b.ToNative();
        Manifold result = NativeMethods.ShapeShape(na, nb, fields);
        GC.KeepAlive(a.PolygonObject); GC.KeepAlive(b.PolygonObject);
        return result;
    }
    public static bool Overlaps(in Shape a, in Shape b)
    {
        FixedValidation.Shape(a);
        FixedValidation.Shape(b);
        NativeShape na = a.ToNative(), nb = b.ToNative();
        bool result = NativeMethods.ShapesOverlap(na, nb) != 0;
        GC.KeepAlive(a.PolygonObject); GC.KeepAlive(b.PolygonObject);
        return result;
    }
}

public static class Sweep
{
    public static SweepHit RayVsCircle(Vec2 origin, Vec2 d, Circle circle)
    {
        FixedValidation.Vec2(origin);
        FixedValidation.Vec2(d);
        FixedValidation.Circle(circle);
        return NativeMethods.RayCircle(origin, d, circle);
    }
    public static SweepHit RayVsAabb(Vec2 origin, Vec2 d, Aabb box)
    {
        FixedValidation.Vec2(origin);
        FixedValidation.Vec2(d);
        FixedValidation.Aabb(box);
        return NativeMethods.RayAabb(origin, d, box);
    }
    public static SweepHit MovingCircleVsCircle(Circle mover, Vec2 motion, Circle target)
    {
        FixedValidation.Circle(mover);
        FixedValidation.Circle(target);
        FixedValidation.Vec2(motion);
        return NativeMethods.MovingCircleCircle(mover, motion, target);
    }
    public static SweepHit MovingCircleVsAabb(Circle mover, Vec2 motion, Aabb box)
    {
        FixedValidation.Circle(mover);
        FixedValidation.Vec2(motion);
        FixedValidation.Aabb(box);
        return NativeMethods.MovingCircleAabb(mover, motion, box);
    }
    public static SweepHit MovingCircleVsCapsule(Circle mover, Vec2 motion, Capsule target)
    {
        FixedValidation.Circle(mover);
        FixedValidation.Vec2(motion);
        _ = FixedValidation.From(target.Radius);
        FixedValidation.Vec2(target.A);
        FixedValidation.Vec2(target.B);
        return NativeMethods.MovingCircleCapsule(mover, motion, target);
    }
    public static SweepHit MovingCircleVsObb(Circle mover, Vec2 motion, Obb target)
    {
        FixedValidation.Vec2(target.Center);
        FixedValidation.Circle(mover);
        FixedValidation.Vec2(motion);
        FixedValidation.Vec2(target.HalfExtents);
        return NativeMethods.MovingCircleObb(mover, motion, target);
    }
    public static SweepHit MovingAabbVsAabb(Aabb mover, Vec2 motion, Aabb target)
    {
        FixedValidation.Aabb(mover);
        FixedValidation.Aabb(target);
        FixedValidation.Vec2(motion);
        return NativeMethods.MovingAabbAabb(mover, motion, target);
    }
    public static SweepHit MovingShapeVsShape(in Shape mover, Vec2 motion, in Shape target)
    {
        FixedValidation.Shape(mover);
        FixedValidation.Vec2(motion);
        FixedValidation.Shape(target);
        NativeShape a = mover.ToNative(), b = target.ToNative();
        SweepHit result = NativeMethods.MovingShapeShape(a, motion, b);
        GC.KeepAlive(mover.PolygonObject); GC.KeepAlive(target.PolygonObject);
        return result;
    }
    public static SweepAlgorithm GetAlgorithm(in Shape mover, in Shape target)
    {
        NativeShape a = mover.ToNative(), b = target.ToNative();
        var result = (SweepAlgorithm)NativeMethods.GetSweepAlgorithm(a, b);
        GC.KeepAlive(mover.PolygonObject); GC.KeepAlive(target.PolygonObject);
        return result;
    }
}
