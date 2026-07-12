using System;

namespace ArcCollision;

/// <summary>
/// Continuous (swept) collision tests. These prevent fast movers from
/// tunnelling through thin geometry by finding the first time-of-impact.
///
/// Ray/segment functions take an origin and a full displacement vector; the
/// returned <see cref="SweepHit.Time"/> is the fraction of that displacement at
/// first contact. Like the discrete tests, the math is all-integer: positions
/// in 24.8 fixed point and times in 16.16. Degree-four discriminants are
/// uniformly scaled before multiplication to keep arithmetic in Int64.
/// </summary>
public static partial class Sweep
{
    /// <summary>Ray (origin + d) versus circle. <paramref name="d"/> is the full motion.</summary>
    public static SweepHit RayVsCircle(Vec2 origin, Vec2 d, Circle circle) =>
        RayVsCircleFx(FxVec2.From(origin), FxVec2.From(d), FxCircle.From(circle)).ToSweepHit();

    internal static FxSweep RayVsCircleFx(FxVec2 origin, FxVec2 d, FxCircle circle)
    {
        // Solve |origin + t*d - center|^2 = r^2 for smallest t in [0,1].
        FxVec2 m = origin - circle.Center;
        long a = d.LengthSq;                       // squared scale
        if (a == 0)
        {
            // No motion: treat as a static containment check.
            return m.LengthSq <= circle.Radius * circle.Radius
                ? new FxSweep(true, 0, FxAxis.FromVector(m, FxAxis.UnitX), origin)
                : FxSweep.Miss;
        }

        long b = m.Dot(d);
        long c = m.LengthSq - circle.Radius * circle.Radius;

        // Origin already inside and moving: contact at t=0.
        if (c <= 0)
        {
            FxAxis n0 = FxAxis.FromVector(m,
                FxAxis.FromVector(-d, FxAxis.UnitX));
            return new FxSweep(true, 0, n0, origin);
        }

        int shift = Fx.ProductShift(a, b, c);
        long scaledA = Fx.ScaleProductOperand(a, shift);
        long scaledB = Fx.ScaleProductOperand(b, shift);
        long scaledC = Fx.ScaleProductOperand(c, shift);
        if (scaledA == 0)
            return FxSweep.Miss;
        long disc = scaledB * scaledB - scaledA * scaledC;
        if (disc < 0)
            return FxSweep.Miss;

        long sqrtDisc = Fx.Sqrt(disc);
        long t16 = Fx.RatioT(-scaledB - sqrtDisc, scaledA);
        if (t16 < 0 || t16 > Fx.TOne)
            return FxSweep.Miss;

        FxVec2 point = origin + d.MulT(t16);
        FxAxis normal = FxAxis.FromVector(point - circle.Center, FxAxis.UnitX);
        return new FxSweep(true, t16, normal, point);
    }

    /// <summary>Ray (origin + d) versus AABB using the slab method.</summary>
    public static SweepHit RayVsAabb(Vec2 origin, Vec2 d, Aabb box) =>
        RayVsAabbFx(FxVec2.From(origin), FxVec2.From(d), FxAabb.From(box)).ToSweepHit();

    internal static FxSweep RayVsAabbFx(FxVec2 origin, FxVec2 d, FxAabb box)
    {
        FxVec2 min = box.Min;
        FxVec2 max = box.Max;

        long tMin = 0;
        long tMax = Fx.TOne;
        FxAxis normal = FxAxis.Zero;

        // X slab
        if (!Slab(origin.X, d.X, min.X, max.X, -FxAxis.UnitX, FxAxis.UnitX,
                  ref tMin, ref tMax, ref normal))
            return FxSweep.Miss;
        // Y slab
        if (!Slab(origin.Y, d.Y, min.Y, max.Y, -FxAxis.UnitY, FxAxis.UnitY,
                  ref tMin, ref tMax, ref normal))
            return FxSweep.Miss;

        if (normal.IsZero)
        {
            long best = origin.X - min.X;
            normal = -FxAxis.UnitX;
            long distance = max.X - origin.X;
            if (distance < best) { best = distance; normal = FxAxis.UnitX; }
            distance = origin.Y - min.Y;
            if (distance < best) { best = distance; normal = -FxAxis.UnitY; }
            distance = max.Y - origin.Y;
            if (distance < best) normal = FxAxis.UnitY;
        }

        FxVec2 point = origin + d.MulT(tMin);
        return new FxSweep(true, tMin, normal, point);
    }

    private static bool Slab(
        long origin, long dir, long slabMin, long slabMax,
        FxAxis nMin, FxAxis nMax, ref long tMin, ref long tMax, ref FxAxis normal)
    {
        if (dir == 0)
        {
            // Parallel to the slab: miss if the origin is outside it.
            return origin >= slabMin && origin <= slabMax;
        }

        // Slab entry/exit times as 16.16; deltas are degree-1 so a 64-bit shift
        // is safe for any sane world size.
        long t1 = Fx.RatioT(slabMin - origin, dir);
        long t2 = Fx.RatioT(slabMax - origin, dir);
        FxAxis n1 = nMin;
        FxAxis n2 = nMax;
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
        var moverFx = FxCircle.From(mover);
        var targetFx = FxCircle.From(target);
        var expanded = new FxCircle(targetFx.Center, targetFx.Radius + moverFx.Radius);
        FxSweep hit = RayVsCircleFx(moverFx.Center, FxVec2.From(motion), expanded);
        if (!hit.Hit)
            return SweepHit.Miss;

        if (hit.Time16 == 0)
        {
            FxManifold initial = Collide.CircleVsCircleFx(moverFx, targetFx);
            if (initial.Colliding)
                return new FxSweep(true, 0, -initial.Normal, initial.Contact).ToSweepHit();
        }

        // Report the contact point on the target surface, not the expanded one.
        FxVec2 moverAt = moverFx.Center + FxVec2.From(motion).MulT(hit.Time16);
        FxVec2 point = moverAt - hit.Normal.Scale(moverFx.Radius);
        return new FxSweep(true, hit.Time16, hit.Normal, point).ToSweepHit();
    }

    /// <summary>
    /// A moving circle versus a static AABB. The exact Minkowski sum is a
    /// rounded rectangle; we test the box expanded by the radius (faces) and the
    /// four corner circles, keeping the earliest impact. This is conservative and
    /// tunnel-free.
    /// </summary>
    public static SweepHit MovingCircleVsAabb(Circle mover, Vec2 motion, Aabb box)
    {
        return MovingCircleVsAabbFx(
            FxCircle.From(mover), FxVec2.From(motion), FxAabb.From(box)).ToSweepHit();
    }

    internal static FxSweep MovingCircleVsAabbFx(
        FxCircle moverFx, FxVec2 motionFx, FxAabb boxFx)
    {
        long r = moverFx.Radius;
        var expanded = new FxAabb(boxFx.Center, new FxVec2(boxFx.Half.X + r, boxFx.Half.Y + r));
        FxSweep best = FxSweep.Miss;

        FxSweep faceHit = RayVsAabbFx(moverFx.Center, motionFx, expanded);
        if (faceHit.Hit)
        {
            // Only accept the face hit when contact lies on an actual face region
            // (not the rounded corner zone), otherwise defer to the corner tests.
            FxVec2 at = moverFx.Center + motionFx.MulT(faceHit.Time16);
            FxVec2 min = boxFx.Min;
            FxVec2 max = boxFx.Max;
            bool cornerZone = (at.X < min.X || at.X > max.X) && (at.Y < min.Y || at.Y > max.Y);
            if (!cornerZone)
                best = faceHit;
        }

        // Corner circles.
        Span<FxVec2> corners = stackalloc FxVec2[4]
        {
            boxFx.Min,
            new FxVec2(boxFx.Max.X, boxFx.Min.Y),
            boxFx.Max,
            new FxVec2(boxFx.Min.X, boxFx.Max.Y),
        };
        foreach (FxVec2 corner in corners)
        {
            FxSweep h = RayVsCircleFx(moverFx.Center, motionFx, new FxCircle(corner, r));
            if (h.Hit && (!best.Hit || h.Time16 < best.Time16))
                best = h;
        }

        if (!best.Hit)
            return FxSweep.Miss;

        if (best.Time16 == 0)
        {
            FxManifold initial = Collide.CircleVsAabbFx(moverFx, boxFx);
            if (initial.Colliding)
                return new FxSweep(true, 0, -initial.Normal, initial.Contact);
        }

        FxVec2 moverAtImpact = moverFx.Center + motionFx.MulT(best.Time16);
        FxVec2 contact = moverAtImpact - best.Normal.Scale(r);
        return new FxSweep(true, best.Time16, best.Normal, contact);
    }

    public static SweepHit MovingCircleVsCapsule(
        Circle mover, Vec2 motion, Capsule target)
    {
        FxCircle circle = FxCircle.From(mover);
        FxVec2 motionFx = FxVec2.From(motion);
        long radius = Math.Abs(circle.Radius) + Math.Abs(Fx.From(target.Radius));
        FxSweep hit = RayVsCapsuleCore(circle.Center, motionFx,
            FxVec2.From(target.A), FxVec2.From(target.B), radius);
        if (!hit.Hit) return SweepHit.Miss;
        FxVec2 contact = hit.Point - hit.Normal.Scale(Math.Abs(circle.Radius));
        return new FxSweep(true, hit.Time16, hit.Normal, contact).ToSweepHit();
    }

    public static SweepHit MovingCircleVsObb(Circle mover, Vec2 motion, Obb target)
    {
        FxAxis axisX = FxAxis.FromAngle(target.Angle);
        FxAxis axisY = axisX.Perpendicular;
        FxVec2 targetCenter = FxVec2.From(target.Center);
        FxCircle moverFx = FxCircle.From(mover);
        FxVec2 relative = moverFx.Center - targetCenter;
        var localCircle = new FxCircle(new FxVec2(
            Fx.RoundDiv(axisX.Dot(relative), FxAxis.One),
            Fx.RoundDiv(axisY.Dot(relative), FxAxis.One)), Math.Abs(moverFx.Radius));
        FxVec2 motionFx = FxVec2.From(motion);
        var localMotion = new FxVec2(
            Fx.RoundDiv(axisX.Dot(motionFx), FxAxis.One),
            Fx.RoundDiv(axisY.Dot(motionFx), FxAxis.One));
        var localBox = new FxAabb(FxVec2.Zero, new FxVec2(
            Math.Abs(Fx.From(target.HalfExtents.X)),
            Math.Abs(Fx.From(target.HalfExtents.Y))));
        FxSweep local = MovingCircleVsAabbFx(localCircle, localMotion, localBox);
        if (!local.Hit) return SweepHit.Miss;
        FxAxis normal = FxAxis.Transform(axisX, axisY, local.Normal);
        FxVec2 point = targetCenter
            + axisX.Scale(local.Point.X)
            + axisY.Scale(local.Point.Y);
        return new FxSweep(true, local.Time16, normal, point).ToSweepHit();
    }

    public static SweepHit MovingAabbVsAabb(Aabb mover, Vec2 motion, Aabb target)
    {
        FxAabb moving = FxAabb.From(mover);
        FxAabb stationary = FxAabb.From(target);
        var expanded = new FxAabb(stationary.Center,
            new FxVec2(stationary.Half.X + moving.Half.X,
                stationary.Half.Y + moving.Half.Y));
        FxVec2 motionFx = FxVec2.From(motion);
        FxSweep hit = RayVsAabbFx(moving.Center, motionFx, expanded);
        if (!hit.Hit) return SweepHit.Miss;
        if (hit.Time16 == 0)
        {
            FxManifold initial = Collide.AabbVsAabbFx(moving, stationary);
            if (initial.Colliding)
                return new FxSweep(true, 0, -initial.Normal, initial.Contact).ToSweepHit();
        }
        FxVec2 center = moving.Center + motionFx.MulT(hit.Time16);
        long extent = Fx.RoundDiv(
            Math.Abs(hit.Normal.X) * moving.Half.X
            + Math.Abs(hit.Normal.Y) * moving.Half.Y,
            FxAxis.One);
        FxVec2 contact = center - hit.Normal.Scale(extent);
        return new FxSweep(true, hit.Time16, hit.Normal, contact).ToSweepHit();
    }
}
