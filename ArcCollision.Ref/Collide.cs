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
///
/// <para><b>Touch semantics.</b> Contact is inclusive and consistent across
/// shapes: shapes that exactly touch (zero separation after 1/256 quantization)
/// report <c>Colliding == true</c> with <c>Depth == 0</c>, matching the boolean
/// <see cref="Overlaps(in Shape, in Shape)"/> and the broadphase predicates.</para>
///
/// <para><b>Manifold accuracy.</b> Depth and normal are exact for the primitive
/// paths (circle/circle, circle/aabb, aabb/aabb) and accurate to a small,
/// size-relative bound for the SAT / rotated paths (any OBB or polygon).
/// <see cref="Manifold.Contact"/> is exact only for the circle-reduction paths;
/// on SAT paths it is an approximate point within the operands' overlapping
/// bounds — see <see cref="Manifold"/>. <see cref="Manifold.SeparationForA"/>
/// resolves the reported contact feature but is not guaranteed to fully separate
/// deeply-overlapping capsules or concave polygons in a single step; apply it
/// iteratively until <c>Colliding</c> is false.</para>
/// </summary>
public static partial class Collide
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
        // Inclusive touch (overlap == 0 counts as colliding with Depth 0), to
        // match CircleVsCircle and the boolean Overlaps / broadphase predicates.
        long overlapX = (a.Half.X + b.Half.X) - Math.Abs(delta.X);
        if (overlapX < 0)
            return FxManifold.None;
        long overlapY = (a.Half.Y + b.Half.Y) - Math.Abs(delta.Y);
        if (overlapY < 0)
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
        => CapsuleVsBox(cap, CreateBox(box));

    // ======================================================== Generic Shape dispatch

    /// <summary>Computes collision details for any supported shape pair.</summary>
    public static Manifold ShapeVsShape(in Shape a, in Shape b)
    {
        return (a.Kind, b.Kind) switch
        {
            (ShapeKind.Circle, ShapeKind.Circle) => CircleVsCircle(a.Circle, b.Circle),
            (ShapeKind.Circle, ShapeKind.Aabb) => CircleVsAabb(a.Circle, b.Aabb),
            (ShapeKind.Aabb, ShapeKind.Circle) => Reverse(CircleVsAabb(b.Circle, a.Aabb)),
            (ShapeKind.Circle, ShapeKind.Capsule) => CircleVsCapsule(a.Circle, b.Capsule),
            (ShapeKind.Capsule, ShapeKind.Circle) => Reverse(CircleVsCapsule(b.Circle, a.Capsule)),
            (ShapeKind.Circle, ShapeKind.Obb) => CircleVsObb(a.Circle, b.Obb),
            (ShapeKind.Obb, ShapeKind.Circle) => Reverse(CircleVsObb(b.Circle, a.Obb)),
            (ShapeKind.Aabb, ShapeKind.Aabb) => AabbVsAabb(a.Aabb, b.Aabb),
            (ShapeKind.Aabb, ShapeKind.Capsule) => Reverse(CapsuleVsBox(b.Capsule, CreateBox(a.Aabb))),
            (ShapeKind.Capsule, ShapeKind.Aabb) => CapsuleVsBox(a.Capsule, CreateBox(b.Aabb)),
            (ShapeKind.Aabb, ShapeKind.Obb) => BoxVsBox(CreateBox(a.Aabb), CreateBox(b.Obb)),
            (ShapeKind.Obb, ShapeKind.Aabb) => BoxVsBox(CreateBox(a.Obb), CreateBox(b.Aabb)),
            (ShapeKind.Capsule, ShapeKind.Obb) => CapsuleVsBox(a.Capsule, CreateBox(b.Obb)),
            (ShapeKind.Obb, ShapeKind.Capsule) => Reverse(CapsuleVsBox(b.Capsule, CreateBox(a.Obb))),
            (ShapeKind.Obb, ShapeKind.Obb) => BoxVsBox(CreateBox(a.Obb), CreateBox(b.Obb)),
            (ShapeKind.Capsule, ShapeKind.Capsule) => CapsuleVsCapsule(a.Capsule, b.Capsule),
            _ when a.Kind == ShapeKind.Polygon || b.Kind == ShapeKind.Polygon => SatShapeVsShape(a, b),
            _ => throw new InvalidOperationException("Missing primitive collision dispatch."),
        };
    }

    /// <summary>
    /// Boolean-only collision test. Primitive pairs avoid manifold, square-root
    /// and contact calculations; complex rounded/rotated pairs use SAT.
    /// </summary>
    public static bool Overlaps(in Shape a, in Shape b)
    {
        return (a.Kind, b.Kind) switch
        {
            (ShapeKind.Circle, ShapeKind.Circle) => CircleCircleOverlap(a.Circle, b.Circle),
            (ShapeKind.Circle, ShapeKind.Aabb) => CircleAabbOverlap(a.Circle, b.Aabb),
            (ShapeKind.Aabb, ShapeKind.Circle) => CircleAabbOverlap(b.Circle, a.Aabb),
            (ShapeKind.Circle, ShapeKind.Capsule) => CircleCapsuleOverlap(a.Circle, b.Capsule),
            (ShapeKind.Capsule, ShapeKind.Circle) => CircleCapsuleOverlap(b.Circle, a.Capsule),
            (ShapeKind.Circle, ShapeKind.Obb) => CircleObbOverlap(a.Circle, b.Obb),
            (ShapeKind.Obb, ShapeKind.Circle) => CircleObbOverlap(b.Circle, a.Obb),
            (ShapeKind.Aabb, ShapeKind.Aabb) => BoundsOverlap(a.Aabb, b.Aabb),
            (ShapeKind.Aabb, ShapeKind.Capsule) => CapsuleBoxOverlap(b.Capsule, CreateBox(a.Aabb)),
            (ShapeKind.Capsule, ShapeKind.Aabb) => CapsuleBoxOverlap(a.Capsule, CreateBox(b.Aabb)),
            (ShapeKind.Aabb, ShapeKind.Obb) => BoxesOverlap(CreateBox(a.Aabb), CreateBox(b.Obb)),
            (ShapeKind.Obb, ShapeKind.Aabb) => BoxesOverlap(CreateBox(a.Obb), CreateBox(b.Aabb)),
            (ShapeKind.Capsule, ShapeKind.Obb) => CapsuleBoxOverlap(a.Capsule, CreateBox(b.Obb)),
            (ShapeKind.Obb, ShapeKind.Capsule) => CapsuleBoxOverlap(b.Capsule, CreateBox(a.Obb)),
            (ShapeKind.Obb, ShapeKind.Obb) => BoxesOverlap(CreateBox(a.Obb), CreateBox(b.Obb)),
            (ShapeKind.Capsule, ShapeKind.Capsule) => CapsuleCapsuleOverlap(a.Capsule, b.Capsule),
            _ when a.Kind == ShapeKind.Polygon || b.Kind == ShapeKind.Polygon => SatShapesOverlap(a, b),
            _ => throw new InvalidOperationException("Missing primitive overlap dispatch."),
        };
    }

    private static bool SatShapesOverlap(in Shape a, in Shape b)
    {
        int piecesA = PieceCount(a);
        int piecesB = PieceCount(b);
        for (int pieceA = 0; pieceA < piecesA; pieceA++)
        {
            ConvexProxy proxyA = CreateProxy(a, pieceA);
            for (int pieceB = 0; pieceB < piecesB; pieceB++)
                if (SatOverlaps(proxyA, CreateProxy(b, pieceB))) return true;
        }
        return false;
    }

    private static Manifold SatShapeVsShape(in Shape a, in Shape b)
    {
        int piecesA = PieceCount(a);
        int piecesB = PieceCount(b);
        FxManifold best = FxManifold.None;

        for (int pieceA = 0; pieceA < piecesA; pieceA++)
        {
            ConvexProxy proxyA = CreateProxy(a, pieceA);
            for (int pieceB = 0; pieceB < piecesB; pieceB++)
            {
                FxManifold candidate = Sat(proxyA, CreateProxy(b, pieceB));
                if (candidate.Colliding && (!best.Colliding || candidate.Depth > best.Depth))
                    best = candidate;
            }
        }
        if (best.Colliding && (ShapeCenter(b) - ShapeCenter(a)).Dot(best.Normal) < 0)
            best = new FxManifold(true, -best.Normal, best.Depth, best.Contact);
        return best.ToManifold();
    }

    private static Manifold Reverse(Manifold manifold) => manifold.Colliding
        ? new Manifold(true, -manifold.Normal, manifold.Depth, manifold.Contact)
        : Manifold.None;

    private static bool CircleCircleOverlap(Circle a, Circle b)
    {
        FxCircle first = FxCircle.From(a);
        FxCircle second = FxCircle.From(b);
        long radius = first.Radius + second.Radius;
        return first.Center.DistSq(second.Center) <= radius * radius;
    }

    private static bool CircleAabbOverlap(Circle circle, Aabb box)
    {
        FxCircle fixedCircle = FxCircle.From(circle);
        FxAabb fixedBox = FxAabb.From(box);
        FxVec2 closest = Distance.ClosestPointOnAabbFx(fixedCircle.Center, fixedBox);
        return fixedCircle.Center.DistSq(closest) <= fixedCircle.Radius * fixedCircle.Radius;
    }

    private static bool CircleCapsuleOverlap(Circle circle, Capsule capsule)
    {
        FxVec2 center = FxVec2.From(circle.Center);
        FxVec2 closest = Distance.ClosestPointOnSegmentFx(center,
            FxVec2.From(capsule.A), FxVec2.From(capsule.B), out _);
        long radius = Math.Abs(Fx.From(circle.Radius)) + Math.Abs(Fx.From(capsule.Radius));
        return center.DistSq(closest) <= radius * radius;
    }

    private static bool CapsuleCapsuleOverlap(Capsule a, Capsule b)
    {
        long distanceSq = Distance.ClosestPointsSegmentSegmentFx(
            FxVec2.From(a.A), FxVec2.From(a.B), FxVec2.From(b.A), FxVec2.From(b.B),
            out _, out _);
        long radius = Math.Abs(Fx.From(a.Radius)) + Math.Abs(Fx.From(b.Radius));
        return distanceSq <= radius * radius;
    }

    private static bool BoundsOverlap(Aabb a, Aabb b) =>
        new BpBounds(a).Overlaps(new BpBounds(b));

    private readonly struct BoxProxy
    {
        public readonly FxVec2 Center;
        public readonly FxVec2 AxisX;
        public readonly FxVec2 AxisY;
        public readonly long HalfX;
        public readonly long HalfY;

        public BoxProxy(FxVec2 center, FxVec2 axisX, FxVec2 axisY, long halfX, long halfY)
        {
            Center = center;
            AxisX = axisX;
            AxisY = axisY;
            HalfX = halfX;
            HalfY = halfY;
        }
    }

    private static BoxProxy CreateBox(Aabb box) => new(
        FxVec2.From(box.Center), new FxVec2(Fx.One, 0), new FxVec2(0, Fx.One),
        Math.Abs(Fx.From(box.HalfExtents.X)), Math.Abs(Fx.From(box.HalfExtents.Y)));

    private static BoxProxy CreateBox(Obb box)
    {
        float c = MathF.Cos(box.Rotation);
        float s = MathF.Sin(box.Rotation);
        var axisX = new FxVec2(Fx.From(c), Fx.From(s)).NormalizedFx(FxVec2.UnitX);
        var axisY = new FxVec2(-axisX.Y, axisX.X);
        return new BoxProxy(FxVec2.From(box.Center), axisX, axisY,
            Math.Abs(Fx.From(box.HalfExtents.X)), Math.Abs(Fx.From(box.HalfExtents.Y)));
    }

    private static Manifold CircleVsObb(Circle circle, Obb box)
    {
        BoxProxy target = CreateBox(box);
        FxCircle source = FxCircle.From(circle);
        FxVec2 delta = source.Center - target.Center;
        var localCenter = new FxVec2(
            Fx.RoundDiv(delta.Dot(target.AxisX), Fx.One),
            Fx.RoundDiv(delta.Dot(target.AxisY), Fx.One));
        FxManifold local = CircleVsAabbFx(
            new FxCircle(localCenter, Math.Abs(source.Radius)),
            new FxAabb(FxVec2.Zero, new FxVec2(target.HalfX, target.HalfY)));
        if (!local.Colliding)
            return Manifold.None;

        FxVec2 normal = target.AxisX.MulUnit(local.Normal.X)
            + target.AxisY.MulUnit(local.Normal.Y);
        FxVec2 contact = target.Center
            + target.AxisX.MulUnit(local.Contact.X)
            + target.AxisY.MulUnit(local.Contact.Y);
        return new FxManifold(true, normal.NormalizedFx(FxVec2.UnitX),
            local.Depth, contact).ToManifold();
    }

    private static bool CircleObbOverlap(Circle circle, Obb box)
    {
        BoxProxy target = CreateBox(box);
        FxCircle source = FxCircle.From(circle);
        FxVec2 delta = source.Center - target.Center;
        long localX = Fx.RoundDiv(delta.Dot(target.AxisX), Fx.One);
        long localY = Fx.RoundDiv(delta.Dot(target.AxisY), Fx.One);
        long closestX = Math.Clamp(localX, -target.HalfX, target.HalfX);
        long closestY = Math.Clamp(localY, -target.HalfY, target.HalfY);
        long dx = localX - closestX;
        long dy = localY - closestY;
        long radius = Math.Abs(source.Radius);
        return dx * dx + dy * dy <= radius * radius;
    }

    private static Manifold BoxVsBox(in BoxProxy a, in BoxProxy b)
    {
        long depth = long.MaxValue;
        FxVec2 axis = FxVec2.UnitX;
        if (!TestBoxAxis(a.AxisX, a, b, ref depth, ref axis)
            || !TestBoxAxis(a.AxisY, a, b, ref depth, ref axis)
            || !TestBoxAxis(b.AxisX, a, b, ref depth, ref axis)
            || !TestBoxAxis(b.AxisY, a, b, ref depth, ref axis))
            return Manifold.None;

        FxVec2 normal = axis.NormalizedFx(FxVec2.UnitX);
        if ((b.Center - a.Center).Dot(normal) < 0) normal = -normal;
        FxVec2 contact = ClampContact(
            Midpoint(BoxSupport(a, normal), BoxSupport(b, -normal)), BoxBounds(a), BoxBounds(b));
        return new FxManifold(true, normal, depth, contact).ToManifold();
    }

    private static bool BoxesOverlap(in BoxProxy a, in BoxProxy b) =>
        BoxAxisOverlaps(a.AxisX, a, b)
        && BoxAxisOverlaps(a.AxisY, a, b)
        && BoxAxisOverlaps(b.AxisX, a, b)
        && BoxAxisOverlaps(b.AxisY, a, b);

    private static bool TestBoxAxis(
        FxVec2 testAxis, in BoxProxy a, in BoxProxy b,
        ref long bestDepth, ref FxVec2 bestAxis)
    {
        ProjectBox(a, testAxis, out long minA, out long maxA);
        ProjectBox(b, testAxis, out long minB, out long maxB);
        long overlap = Math.Min(maxA - minB, maxB - minA);
        if (overlap < 0) return false;
        long depth = Fx.RoundDiv(overlap, testAxis.Length);
        if (depth < bestDepth)
        {
            bestDepth = depth;
            bestAxis = testAxis;
        }
        return true;
    }

    private static bool BoxAxisOverlaps(FxVec2 axis, in BoxProxy a, in BoxProxy b)
    {
        ProjectBox(a, axis, out long minA, out long maxA);
        ProjectBox(b, axis, out long minB, out long maxB);
        return maxA >= minB && maxB >= minA;
    }

    private static void ProjectBox(in BoxProxy box, FxVec2 axis, out long min, out long max)
    {
        long center = box.Center.Dot(axis);
        long radius = Fx.MulUnit(Math.Abs(box.AxisX.Dot(axis)), box.HalfX)
            + Fx.MulUnit(Math.Abs(box.AxisY.Dot(axis)), box.HalfY);
        min = center - radius;
        max = center + radius;
    }

    private static FxVec2 BoxSupport(in BoxProxy box, FxVec2 direction)
    {
        FxVec2 result = box.Center;
        result += box.AxisX.MulUnit(box.AxisX.Dot(direction) >= 0 ? box.HalfX : -box.HalfX);
        result += box.AxisY.MulUnit(box.AxisY.Dot(direction) >= 0 ? box.HalfY : -box.HalfY);
        return result;
    }

    private static Manifold CapsuleVsBox(Capsule capsule, in BoxProxy box)
    {
        FxVec2 a = FxVec2.From(capsule.A);
        FxVec2 b = FxVec2.From(capsule.B);
        long radius = Math.Abs(Fx.From(capsule.Radius));
        long depth = long.MaxValue;
        FxVec2 axis = FxVec2.UnitX;

        if (!TestCapsuleBoxAxis(box.AxisX, a, b, radius, box, ref depth, ref axis)
            || !TestCapsuleBoxAxis(box.AxisY, a, b, radius, box, ref depth, ref axis))
            return Manifold.None;

        FxVec2 spine = b - a;
        if (!TestCapsuleBoxAxis(new FxVec2(-spine.Y, spine.X),
                a, b, radius, box, ref depth, ref axis))
            return Manifold.None;

        for (int corner = 0; corner < 4; corner++)
        {
            FxVec2 vertex = BoxVertex(box, corner);
            FxVec2 closest = Distance.ClosestPointOnSegmentFx(vertex, a, b, out _);
            if (!TestCapsuleBoxAxis(closest - vertex,
                    a, b, radius, box, ref depth, ref axis))
                return Manifold.None;
        }

        FxVec2 normal = axis.NormalizedFx(FxVec2.UnitX);
        FxVec2 center = Midpoint(a, b);
        if ((box.Center - center).Dot(normal) < 0) normal = -normal;
        FxVec2 contact = ClampContact(
            Midpoint(CapsuleSupport(a, b, radius, normal), BoxSupport(box, -normal)),
            SegmentBounds(a, b, radius), BoxBounds(box));
        return new FxManifold(true, normal, depth, contact).ToManifold();
    }

    private static bool CapsuleBoxOverlap(Capsule capsule, in BoxProxy box)
    {
        FxVec2 a = FxVec2.From(capsule.A);
        FxVec2 b = FxVec2.From(capsule.B);
        long radius = Math.Abs(Fx.From(capsule.Radius));
        if (!CapsuleBoxAxisOverlaps(box.AxisX, a, b, radius, box)
            || !CapsuleBoxAxisOverlaps(box.AxisY, a, b, radius, box))
            return false;

        FxVec2 spine = b - a;
        if (!CapsuleBoxAxisOverlaps(new FxVec2(-spine.Y, spine.X), a, b, radius, box))
            return false;

        for (int corner = 0; corner < 4; corner++)
        {
            FxVec2 vertex = BoxVertex(box, corner);
            FxVec2 closest = Distance.ClosestPointOnSegmentFx(vertex, a, b, out _);
            if (!CapsuleBoxAxisOverlaps(closest - vertex, a, b, radius, box))
                return false;
        }
        return true;
    }

    private static bool TestCapsuleBoxAxis(
        FxVec2 testAxis, FxVec2 a, FxVec2 b, long radius, in BoxProxy box,
        ref long bestDepth, ref FxVec2 bestAxis)
    {
        long lengthSq = testAxis.LengthSq;
        if (lengthSq == 0) return true;
        long length = Fx.Sqrt(lengthSq);
        ProjectCapsule(a, b, radius, testAxis, length, out long minA, out long maxA);
        ProjectBox(box, testAxis, out long minB, out long maxB);
        long overlap = Math.Min(maxA - minB, maxB - minA);
        if (overlap < 0) return false;
        long depth = Fx.RoundDiv(overlap, length);
        if (depth < bestDepth)
        {
            bestDepth = depth;
            bestAxis = testAxis;
        }
        return true;
    }

    private static bool CapsuleBoxAxisOverlaps(
        FxVec2 axis, FxVec2 a, FxVec2 b, long radius, in BoxProxy box)
    {
        long lengthSq = axis.LengthSq;
        if (lengthSq == 0) return true;
        long length = Fx.Sqrt(lengthSq);
        ProjectCapsule(a, b, radius, axis, length, out long minA, out long maxA);
        ProjectBox(box, axis, out long minB, out long maxB);
        return maxA >= minB && maxB >= minA;
    }

    private static void ProjectCapsule(
        FxVec2 a, FxVec2 b, long radius, FxVec2 axis, long axisLength,
        out long min, out long max)
    {
        long first = a.Dot(axis);
        long second = b.Dot(axis);
        long radiusProjection = radius * axisLength;
        min = Math.Min(first, second) - radiusProjection;
        max = Math.Max(first, second) + radiusProjection;
    }

    private static FxVec2 CapsuleSupport(
        FxVec2 a, FxVec2 b, long radius, FxVec2 direction)
    {
        FxVec2 endpoint = a.Dot(direction) >= b.Dot(direction) ? a : b;
        return endpoint + direction.MulUnit(radius);
    }

    private static FxVec2 BoxVertex(in BoxProxy box, int index)
    {
        long x = index is 1 or 2 ? box.HalfX : -box.HalfX;
        long y = index >= 2 ? box.HalfY : -box.HalfY;
        return box.Center + box.AxisX.MulUnit(x) + box.AxisY.MulUnit(y);
    }

    private readonly struct ConvexProxy
    {
        private readonly FxVec2 _v0, _v1, _v2, _v3;
        private readonly FxVec2[]? _vertices;
        private readonly int[]? _indices;
        private readonly int _indexOffset;

        public readonly int Count;
        public readonly long Radius;
        public readonly FxVec2 Center;

        public ConvexProxy(FxVec2 v0, long radius)
        {
            this = default;
            _v0 = v0;
            Count = 1;
            Radius = radius;
            Center = v0;
        }

        public ConvexProxy(FxVec2 v0, FxVec2 v1, long radius)
        {
            this = default;
            _v0 = v0;
            _v1 = v1;
            Count = 2;
            Radius = radius;
            Center = Midpoint(v0, v1);
        }

        public ConvexProxy(FxVec2 v0, FxVec2 v1, FxVec2 v2, FxVec2 v3)
        {
            this = default;
            _v0 = v0;
            _v1 = v1;
            _v2 = v2;
            _v3 = v3;
            Count = 4;
            Center = new FxVec2((v0.X + v1.X + v2.X + v3.X) / 4,
                (v0.Y + v1.Y + v2.Y + v3.Y) / 4);
        }

        public ConvexProxy(FxVec2[] vertices)
        {
            this = default;
            _vertices = vertices;
            Count = vertices.Length;
            Center = Average(vertices, null, 0, Count);
        }

        public ConvexProxy(FxVec2[] vertices, int[] indices, int indexOffset)
        {
            this = default;
            _vertices = vertices;
            _indices = indices;
            _indexOffset = indexOffset;
            Count = 3;
            Center = Average(vertices, indices, indexOffset, Count);
        }

        public FxVec2 Vertex(int index)
        {
            if (_vertices != null)
                return _vertices[_indices == null ? index : _indices[_indexOffset + index]];
            return index switch
            {
                0 => _v0,
                1 => _v1,
                2 => _v2,
                3 => _v3,
                _ => throw new ArgumentOutOfRangeException(nameof(index)),
            };
        }

        public int EdgeCount => Count <= 1 ? 0 : Count == 2 ? 1 : Count;

        public void Edge(int index, out FxVec2 a, out FxVec2 b)
        {
            a = Vertex(index);
            b = Vertex(Count == 2 ? 1 : (index + 1) % Count);
        }

        private static FxVec2 Average(FxVec2[] vertices, int[]? indices, int offset, int count)
        {
            long x = 0, y = 0;
            for (int i = 0; i < count; i++)
            {
                FxVec2 vertex = vertices[indices == null ? i : indices[offset + i]];
                x += vertex.X;
                y += vertex.Y;
            }
            return new FxVec2(x / count, y / count);
        }
    }

    private struct SatState
    {
        public long Depth;
        public FxVec2 Axis;
        public bool HasAxis;
    }

    private static int PieceCount(in Shape shape) =>
        shape.Kind == ShapeKind.Polygon && !shape.Polygon.IsConvex
            ? shape.Polygon.TriangleIndices.Length / 3
            : 1;

    private static ConvexProxy CreateProxy(in Shape shape, int piece)
    {
        switch (shape.Kind)
        {
            case ShapeKind.Circle:
            {
                Circle circle = shape.Circle;
                return new ConvexProxy(FxVec2.From(circle.Center), Math.Abs(Fx.From(circle.Radius)));
            }
            case ShapeKind.Capsule:
            {
                Capsule capsule = shape.Capsule;
                return new ConvexProxy(FxVec2.From(capsule.A), FxVec2.From(capsule.B),
                    Math.Abs(Fx.From(capsule.Radius)));
            }
            case ShapeKind.Aabb:
            {
                Aabb source = shape.Aabb;
                FxVec2 center = FxVec2.From(source.Center);
                var half = new FxVec2(Math.Abs(Fx.From(source.HalfExtents.X)),
                    Math.Abs(Fx.From(source.HalfExtents.Y)));
                FxVec2 min = center - half;
                FxVec2 max = center + half;
                return new ConvexProxy(min, new FxVec2(max.X, min.Y), max,
                    new FxVec2(min.X, max.Y));
            }
            case ShapeKind.Obb:
            {
                Obb box = shape.Obb;
                float c = MathF.Cos(box.Rotation);
                float s = MathF.Sin(box.Rotation);
                float hx = MathF.Abs(box.HalfExtents.X);
                float hy = MathF.Abs(box.HalfExtents.Y);
                Vec2 x = new(c * hx, s * hx);
                Vec2 y = new(-s * hy, c * hy);
                return new ConvexProxy(
                    FxVec2.From(box.Center - x - y), FxVec2.From(box.Center + x - y),
                    FxVec2.From(box.Center + x + y), FxVec2.From(box.Center - x + y));
            }
            case ShapeKind.Polygon:
            {
                Polygon polygon = shape.Polygon;
                return polygon.IsConvex
                    ? new ConvexProxy(polygon.FixedVertices)
                    : new ConvexProxy(polygon.FixedVertices, polygon.TriangleIndices, piece * 3);
            }
            default:
                throw new ArgumentOutOfRangeException(nameof(shape));
        }
    }

    private static FxManifold Sat(in ConvexProxy a, in ConvexProxy b)
    {
        var state = new SatState { Depth = long.MaxValue };
        if (!TestEdgeAxes(a, a, b, ref state) || !TestEdgeAxes(b, a, b, ref state))
            return FxManifold.None;

        if (a.Radius != 0 || b.Radius != 0)
        {
            if (!TestVertexEdgeAxes(a, b, a, b, ref state)
                || !TestVertexEdgeAxes(b, a, a, b, ref state))
                return FxManifold.None;
        }

        if (!state.HasAxis)
        {
            FxVec2 fallback = b.Center - a.Center;
            if (fallback.LengthSq == 0) fallback = FxVec2.UnitX;
            if (!TestAxis(fallback, a, b, ref state))
                return FxManifold.None;
        }

        FxVec2 normal = state.Axis.NormalizedFx(FxVec2.UnitX);
        if ((b.Center - a.Center).Dot(normal) < 0)
            normal = -normal;
        FxVec2 pointA = Support(a, normal);
        FxVec2 pointB = Support(b, -normal);
        FxVec2 contact = ClampContact(Midpoint(pointA, pointB), ProxyBounds(a), ProxyBounds(b));
        return new FxManifold(true, normal, state.Depth, contact);
    }

    private static bool SatOverlaps(in ConvexProxy a, in ConvexProxy b)
    {
        bool hasAxis = false;
        if (!EdgeAxesOverlap(a, a, b, ref hasAxis)
            || !EdgeAxesOverlap(b, a, b, ref hasAxis))
            return false;

        if (a.Radius != 0 || b.Radius != 0)
        {
            if (!VertexEdgeAxesOverlap(a, b, a, b, ref hasAxis)
                || !VertexEdgeAxesOverlap(b, a, a, b, ref hasAxis))
                return false;
        }

        if (!hasAxis)
        {
            FxVec2 fallback = b.Center - a.Center;
            if (fallback.LengthSq == 0) fallback = FxVec2.UnitX;
            return AxisOverlaps(fallback, a, b, ref hasAxis);
        }
        return true;
    }

    private static bool EdgeAxesOverlap(
        in ConvexProxy source, in ConvexProxy a, in ConvexProxy b, ref bool hasAxis)
    {
        for (int edge = 0; edge < source.EdgeCount; edge++)
        {
            source.Edge(edge, out FxVec2 p0, out FxVec2 p1);
            FxVec2 delta = p1 - p0;
            if (!AxisOverlaps(new FxVec2(-delta.Y, delta.X), a, b, ref hasAxis))
                return false;
        }
        return true;
    }

    private static bool VertexEdgeAxesOverlap(
        in ConvexProxy vertices, in ConvexProxy edges,
        in ConvexProxy a, in ConvexProxy b, ref bool hasAxis)
    {
        for (int vertex = 0; vertex < vertices.Count; vertex++)
        {
            FxVec2 point = vertices.Vertex(vertex);
            for (int edge = 0; edge < edges.EdgeCount; edge++)
            {
                edges.Edge(edge, out FxVec2 e0, out FxVec2 e1);
                FxVec2 closest = Distance.ClosestPointOnSegmentFx(point, e0, e1, out _);
                if (!AxisOverlaps(closest - point, a, b, ref hasAxis))
                    return false;
            }
        }
        return true;
    }

    private static bool AxisOverlaps(
        FxVec2 axis, in ConvexProxy a, in ConvexProxy b, ref bool hasAxis)
    {
        // Round shapes need the radius projected along a correctly-scaled axis,
        // so normalize to a 24.8 unit axis (exact length) — see TestAxis. Purely
        // polygonal pairs carry no radius, so the raw axis suffices there.
        if (a.Radius != 0 || b.Radius != 0)
        {
            FxVec2 unit = axis.NormalizedFx(FxVec2.Zero);
            if (unit.X == 0 && unit.Y == 0)
                return true;
            hasAxis = true;
            Project(a, unit, Fx.One, out long uMinA, out long uMaxA);
            Project(b, unit, Fx.One, out long uMinB, out long uMaxB);
            return uMaxA >= uMinB && uMaxB >= uMinA;
        }

        long lengthSq = axis.LengthSq;
        if (lengthSq == 0)
            return true;
        hasAxis = true;
        Project(a, axis, 0, out long minA, out long maxA);
        Project(b, axis, 0, out long minB, out long maxB);
        return maxA >= minB && maxB >= minA;
    }

    private static bool TestEdgeAxes(
        in ConvexProxy source, in ConvexProxy a, in ConvexProxy b, ref SatState state)
    {
        for (int edge = 0; edge < source.EdgeCount; edge++)
        {
            source.Edge(edge, out FxVec2 p0, out FxVec2 p1);
            FxVec2 delta = p1 - p0;
            if (!TestAxis(new FxVec2(-delta.Y, delta.X), a, b, ref state))
                return false;
        }
        return true;
    }

    private static bool TestVertexEdgeAxes(
        in ConvexProxy vertices, in ConvexProxy edges,
        in ConvexProxy a, in ConvexProxy b, ref SatState state)
    {
        for (int vertex = 0; vertex < vertices.Count; vertex++)
        {
            FxVec2 point = vertices.Vertex(vertex);
            for (int edge = 0; edge < edges.EdgeCount; edge++)
            {
                edges.Edge(edge, out FxVec2 e0, out FxVec2 e1);
                FxVec2 closest = Distance.ClosestPointOnSegmentFx(point, e0, e1, out _);
                if (!TestAxis(closest - point, a, b, ref state))
                    return false;
            }
        }
        return true;
    }

    private static bool TestAxis(
        FxVec2 axis, in ConvexProxy a, in ConvexProxy b, ref SatState state)
    {
        // Normalize to a 24.8 unit axis first. A near-degenerate polygon edge
        // yields a tiny raw axis whose integer length (isqrt of a very small
        // value) is far too imprecise to scale the radius projection by — that
        // under-count manufactures false separating axes and misses collisions.
        // A unit axis has an exact known length (Fx.One), so the projection and
        // depth are computed against it with no length-precision loss.
        FxVec2 unit = axis.NormalizedFx(FxVec2.Zero);
        if (unit.X == 0 && unit.Y == 0)
            return true;
        Project(a, unit, Fx.One, out long minA, out long maxA);
        Project(b, unit, Fx.One, out long minB, out long maxB);
        long towardPositive = maxA - minB;
        long towardNegative = maxB - minA;
        if (towardPositive < 0 || towardNegative < 0)
            return false;

        long overlap = Math.Min(towardPositive, towardNegative);
        long depth = Fx.RoundDiv(overlap, Fx.One);
        if (!state.HasAxis || depth < state.Depth)
        {
            state.HasAxis = true;
            state.Depth = depth;
            state.Axis = unit;
        }
        return true;
    }

    private static void Project(
        in ConvexProxy proxy, FxVec2 axis, long axisLength, out long min, out long max)
    {
        long projection = proxy.Vertex(0).Dot(axis);
        min = max = projection;
        for (int i = 1; i < proxy.Count; i++)
        {
            projection = proxy.Vertex(i).Dot(axis);
            min = Math.Min(min, projection);
            max = Math.Max(max, projection);
        }
        long radiusProjection = proxy.Radius * axisLength;
        min -= radiusProjection;
        max += radiusProjection;
    }

    private static FxVec2 Support(in ConvexProxy proxy, FxVec2 direction)
    {
        FxVec2 best = proxy.Vertex(0);
        long bestProjection = best.Dot(direction);
        for (int i = 1; i < proxy.Count; i++)
        {
            FxVec2 candidate = proxy.Vertex(i);
            long projection = candidate.Dot(direction);
            if (projection > bestProjection)
            {
                best = candidate;
                bestProjection = projection;
            }
        }
        return best + direction.MulUnit(proxy.Radius);
    }

    private static FxVec2 Midpoint(FxVec2 a, FxVec2 b) =>
        new(a.X + ((b.X - a.X) >> 1), a.Y + ((b.Y - a.Y) >> 1));

    // The midpoint of two support points can fall outside the shapes' overlap;
    // clamp it into the intersection of the operands' world AABBs so the reported
    // SAT contact is always at least within both bounding boxes.
    private static FxVec2 ClampContact(FxVec2 contact, in FxAabb a, in FxAabb b)
    {
        long minX = Math.Max(a.Min.X, b.Min.X), maxX = Math.Min(a.Max.X, b.Max.X);
        long minY = Math.Max(a.Min.Y, b.Min.Y), maxY = Math.Min(a.Max.Y, b.Max.Y);
        if (minX > maxX || minY > maxY) return contact;   // not overlapping (shouldn't happen)
        return new FxVec2(Math.Clamp(contact.X, minX, maxX), Math.Clamp(contact.Y, minY, maxY));
    }

    private static FxAabb BoxBounds(in BoxProxy box)
    {
        long hx = Fx.MulUnit(Math.Abs(box.AxisX.X), box.HalfX) + Fx.MulUnit(Math.Abs(box.AxisY.X), box.HalfY);
        long hy = Fx.MulUnit(Math.Abs(box.AxisX.Y), box.HalfX) + Fx.MulUnit(Math.Abs(box.AxisY.Y), box.HalfY);
        return new FxAabb(box.Center, new FxVec2(hx, hy));
    }

    private static FxAabb SegmentBounds(FxVec2 a, FxVec2 b, long radius)
    {
        long minX = Math.Min(a.X, b.X) - radius, maxX = Math.Max(a.X, b.X) + radius;
        long minY = Math.Min(a.Y, b.Y) - radius, maxY = Math.Max(a.Y, b.Y) + radius;
        return new FxAabb(new FxVec2((minX + maxX) / 2, (minY + maxY) / 2),
            new FxVec2((maxX - minX) / 2, (maxY - minY) / 2));
    }

    private static FxAabb ProxyBounds(in ConvexProxy p)
    {
        FxVec2 v0 = p.Vertex(0);
        long minX = v0.X, maxX = v0.X, minY = v0.Y, maxY = v0.Y;
        for (int i = 1; i < p.Count; i++)
        {
            FxVec2 v = p.Vertex(i);
            minX = Math.Min(minX, v.X); maxX = Math.Max(maxX, v.X);
            minY = Math.Min(minY, v.Y); maxY = Math.Max(maxY, v.Y);
        }
        minX -= p.Radius; maxX += p.Radius; minY -= p.Radius; maxY += p.Radius;
        return new FxAabb(new FxVec2((minX + maxX) / 2, (minY + maxY) / 2),
            new FxVec2((maxX - minX) / 2, (maxY - minY) / 2));
    }

    private static FxVec2 ShapeCenter(in Shape shape)
    {
        BpBounds bounds = new(shape);
        return new FxVec2(bounds.CenterX, bounds.CenterY);
    }
}
