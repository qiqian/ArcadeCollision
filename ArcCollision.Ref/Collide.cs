using System;

namespace ArcCollision.Ref;

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
/// <para><b>Manifold accuracy.</b> Depth and normal are exact for primitive and
/// convex fixed-grid paths, within projection rounding for rotated geometry.
/// Deep capsule crossings use their rounded Minkowski difference. Concave unions
/// accumulate convex-piece separations and return a verified one-step separation;
/// that separation is not necessarily globally minimal. Contact is a stable hint
/// rather than a guaranteed clipped surface anchor — see <see cref="Manifold"/>.</para>
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

    internal static FxManifold CircleVsCircleFx(
        FxCircle a, FxCircle b, bool computeContact = true)
    {
        FxVec2 delta = b.Center - a.Center;
        long r = a.Radius + b.Radius;
        long distSq = delta.LengthSq;
        if (distSq > r * r)
            return FxManifold.None;

        long dist = Fx.Sqrt(distSq);
        // Degenerate: concentric circles -> pick an arbitrary but stable axis.
        FxAxis normal = dist > 0
            ? FxAxis.FromVector(delta, FxAxis.UnitX)
            : FxAxis.UnitX;
        long depth = r - dist;
        FxVec2 contact = computeContact
            ? a.Center + normal.Scale(a.Radius - depth / 2)
            : FxVec2.Zero;
        return new FxManifold(true, normal, depth, contact);
    }

    // ------------------------------------------------------------ Aabb / Aabb

    public static Manifold AabbVsAabb(Aabb a, Aabb b) =>
        AabbVsAabbFx(FxAabb.From(a), FxAabb.From(b)).ToManifold();

    internal static FxManifold AabbVsAabbFx(
        FxAabb a, FxAabb b, bool computeContact = true)
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
            FxAxis normal = sign < 0 ? -FxAxis.UnitX : FxAxis.UnitX;
            long contactX = a.Center.X + sign * a.Half.X;
            FxVec2 contact = computeContact
                ? ClampContact(new FxVec2(contactX, b.Center.Y), a, b)
                : FxVec2.Zero;
            return new FxManifold(true, normal, overlapX, contact);
        }
        else
        {
            long sign = delta.Y < 0 ? -1 : 1;
            FxAxis normal = sign < 0 ? -FxAxis.UnitY : FxAxis.UnitY;
            long contactY = a.Center.Y + sign * a.Half.Y;
            FxVec2 contact = computeContact
                ? ClampContact(new FxVec2(b.Center.X, contactY), a, b)
                : FxVec2.Zero;
            return new FxManifold(true, normal, overlapY, contact);
        }
    }

    // ---------------------------------------------------------- Circle / Aabb

    public static Manifold CircleVsAabb(Circle c, Aabb box) =>
        CircleVsAabbFx(FxCircle.From(c), FxAabb.From(box)).ToManifold();

    internal static FxManifold CircleVsAabbFx(
        FxCircle c, FxAabb box, bool computeContact = true)
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
            FxAxis normal = FxAxis.FromVector(delta, FxAxis.UnitX);
            long depth = c.Radius - dist;
            return new FxManifold(true, normal, depth,
                computeContact ? closest : FxVec2.Zero);
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
            FxAxis normal = outSign < 0 ? FxAxis.UnitX : -FxAxis.UnitX;
            long depth = overlapX + c.Radius;
            FxVec2 contact = computeContact
                ? new FxVec2(box.Center.X + outSign * box.Half.X, c.Center.Y)
                : FxVec2.Zero;
            return new FxManifold(true, normal, depth, contact);
        }
        else
        {
            long outSign = d.Y < 0 ? -1 : 1;
            FxAxis normal = outSign < 0 ? FxAxis.UnitY : -FxAxis.UnitY;
            long depth = overlapY + c.Radius;
            FxVec2 contact = computeContact
                ? new FxVec2(c.Center.X, box.Center.Y + outSign * box.Half.Y)
                : FxVec2.Zero;
            return new FxManifold(true, normal, depth, contact);
        }
    }

    // ------------------------------------------------------- Capsule variants

    public static Manifold CircleVsCapsule(Circle c, Capsule cap) =>
        CircleVsCapsule(c, cap, computeContact: true);

    private static Manifold CircleVsCapsule(
        Circle c, Capsule cap, bool computeContact)
    {
        FxCircle cf = FxCircle.From(c);
        FxVec2 a = FxVec2.From(cap.A);
        FxVec2 b = FxVec2.From(cap.B);
        FxVec2 closest = Distance.ClosestPointOnSegmentFx(cf.Center, a, b, out _);
        FxCircle spinePoint = new(closest, Math.Abs(Fx.From(cap.Radius)));
        FxVec2 delta = closest - cf.Center;
        if (delta.LengthSq != 0 || a.X == b.X && a.Y == b.Y)
            return CircleVsCircleFx(cf, spinePoint, computeContact).ToManifold();

        long depth = Math.Abs(cf.Radius) + spinePoint.Radius;
        FxVec2 spine = b - a;
        FxAxis normal = FxAxis.FromVector(
            new FxVec2(-spine.Y, spine.X), FxAxis.UnitX);
        FxVec2 contact = computeContact
            ? cf.Center + normal.Scale(Math.Abs(cf.Radius) - depth / 2)
            : FxVec2.Zero;
        return new FxManifold(true, normal, depth, contact).ToManifold();
    }

    public static Manifold CapsuleVsCapsule(Capsule a, Capsule b) =>
        CapsuleVsCapsule(a, b, computeContact: true);

    private static Manifold CapsuleVsCapsule(
        Capsule a, Capsule b, bool computeContact)
    {
        FxVec2 a0 = FxVec2.From(a.A);
        FxVec2 a1 = FxVec2.From(a.B);
        FxVec2 b0 = FxVec2.From(b.A);
        FxVec2 b1 = FxVec2.From(b.B);
        long radiusA = Math.Abs(Fx.From(a.Radius));
        long radiusB = Math.Abs(Fx.From(b.Radius));

        // The common shallow-contact case has disjoint spines and reduces
        // exactly to two circles at their closest points. Keep intersecting,
        // touching and fixed-grid ambiguous spines on the Minkowski SAT path:
        // closest-point reduction cannot produce their correct MTV.
        if (!SpinesIntersect(a0, a1, b0, b1))
        {
            long spineDistanceSq = Distance.ClosestPointsSegmentSegmentFx(
                a0, a1, b0, b1, out FxVec2 closestA, out FxVec2 closestB);
            if (spineDistanceSq != 0)
            {
                FxManifold reduced = CircleVsCircleFx(
                    new FxCircle(closestA, radiusA),
                    new FxCircle(closestB, radiusB),
                    computeContact: false);
                if (!reduced.Colliding) return Manifold.None;
                FxVec2 reducedContact = computeContact
                    ? ClampContact(
                        SupportFeatureContact(
                            new ConvexProxy(a0, a1, radiusA),
                            new ConvexProxy(b0, b1, radiusB),
                            reduced.Normal),
                        SegmentBounds(a0, a1, radiusA),
                        SegmentBounds(b0, b1, radiusB))
                    : FxVec2.Zero;
                return new FxManifold(
                    true, reduced.Normal, reduced.Depth, reducedContact).ToManifold();
            }
        }

        var difference = new ConvexProxy(
            a0 - b0, a1 - b0, a1 - b1, a0 - b1, radiusA + radiusB);
        // This intermediate manifold contributes only its normal/depth.
        FxManifold configuration = Sat(
            difference, new ConvexProxy(FxVec2.Zero, 0), computeContact: false);
        if (!configuration.Colliding) return Manifold.None;

        FxAxis normal = configuration.Normal;
        FxVec2 contact = computeContact
            ? ClampContact(
                SupportFeatureContact(
                    new ConvexProxy(a0, a1, radiusA),
                    new ConvexProxy(b0, b1, radiusB),
                    normal),
                SegmentBounds(a0, a1, radiusA), SegmentBounds(b0, b1, radiusB))
            : FxVec2.Zero;
        return new FxManifold(true, normal, configuration.Depth, contact).ToManifold();
    }

    private static bool SpinesIntersect(FxVec2 a, FxVec2 b, FxVec2 c, FxVec2 d)
    {
        long abC = Cross(a, b, c);
        long abD = Cross(a, b, d);
        long cdA = Cross(c, d, a);
        long cdB = Cross(c, d, b);
        if (abC == 0 && OnSegment(a, b, c)) return true;
        if (abD == 0 && OnSegment(a, b, d)) return true;
        if (cdA == 0 && OnSegment(c, d, a)) return true;
        if (cdB == 0 && OnSegment(c, d, b)) return true;
        return (abC < 0) != (abD < 0) && (cdA < 0) != (cdB < 0);
    }

    private static long Cross(FxVec2 a, FxVec2 b, FxVec2 c)
    {
        FxVec2 ab = b - a;
        FxVec2 ac = c - a;
        return ab.X * ac.Y - ab.Y * ac.X;
    }

    private static bool OnSegment(FxVec2 a, FxVec2 b, FxVec2 point) =>
        point.X >= Math.Min(a.X, b.X) && point.X <= Math.Max(a.X, b.X)
        && point.Y >= Math.Min(a.Y, b.Y) && point.Y <= Math.Max(a.Y, b.Y);

    public static Manifold CapsuleVsAabb(Capsule cap, Aabb box)
        => CapsuleVsBox(cap, CreateBox(box));

    // ======================================================== Generic Shape dispatch

    /// <summary>
    /// Computes the requested collision details for any supported shape pair.
    /// <see cref="ManifoldFields.None"/> uses the boolean-only early-out path;
    /// <see cref="ManifoldFields.NormalDepth"/> skips all contact-point work.
    /// Fields that were not requested are returned as zero.
    /// </summary>
    public static Manifold ShapeVsShape(
        in Shape a, in Shape b, ManifoldFields fields = ManifoldFields.All)
    {
        if (fields is < ManifoldFields.None or > ManifoldFields.All)
            throw new ArgumentOutOfRangeException(nameof(fields));
        if (fields == ManifoldFields.None)
            return new Manifold(Overlaps(a, b), Vec2.Zero, 0f, Vec2.Zero);

        bool computeContact = fields == ManifoldFields.All;
        return (a.Kind, b.Kind) switch
        {
            (ShapeKind.Circle, ShapeKind.Circle) =>
                CircleVsCircleFx(FxCircle.From(a.Circle), FxCircle.From(b.Circle),
                    computeContact).ToManifold(),
            (ShapeKind.Circle, ShapeKind.Aabb) =>
                CircleVsAabbFx(FxCircle.From(a.Circle), FxAabb.From(b.Aabb),
                    computeContact).ToManifold(),
            (ShapeKind.Aabb, ShapeKind.Circle) => Reverse(
                CircleVsAabbFx(FxCircle.From(b.Circle), FxAabb.From(a.Aabb),
                    computeContact).ToManifold()),
            (ShapeKind.Circle, ShapeKind.Capsule) =>
                CircleVsCapsule(a.Circle, b.Capsule, computeContact),
            (ShapeKind.Capsule, ShapeKind.Circle) => Reverse(
                CircleVsCapsule(b.Circle, a.Capsule, computeContact)),
            (ShapeKind.Circle, ShapeKind.Obb) => CircleVsObb(a.Circle, b.Obb, computeContact),
            (ShapeKind.Obb, ShapeKind.Circle) => Reverse(CircleVsObb(b.Circle, a.Obb, computeContact)),
            (ShapeKind.Aabb, ShapeKind.Aabb) =>
                AabbVsAabbFx(FxAabb.From(a.Aabb), FxAabb.From(b.Aabb),
                    computeContact).ToManifold(),
            (ShapeKind.Aabb, ShapeKind.Capsule) => Reverse(
                CapsuleVsBox(b.Capsule, CreateBox(a.Aabb), computeContact)),
            (ShapeKind.Capsule, ShapeKind.Aabb) =>
                CapsuleVsBox(a.Capsule, CreateBox(b.Aabb), computeContact),
            (ShapeKind.Aabb, ShapeKind.Obb) =>
                BoxVsBox(CreateBox(a.Aabb), CreateBox(b.Obb), computeContact),
            (ShapeKind.Obb, ShapeKind.Aabb) =>
                BoxVsBox(CreateBox(a.Obb), CreateBox(b.Aabb), computeContact),
            (ShapeKind.Capsule, ShapeKind.Obb) =>
                CapsuleVsBox(a.Capsule, CreateBox(b.Obb), computeContact),
            (ShapeKind.Obb, ShapeKind.Capsule) => Reverse(
                CapsuleVsBox(b.Capsule, CreateBox(a.Obb), computeContact)),
            (ShapeKind.Obb, ShapeKind.Obb) =>
                BoxVsBox(CreateBox(a.Obb), CreateBox(b.Obb), computeContact),
            (ShapeKind.Capsule, ShapeKind.Capsule) =>
                CapsuleVsCapsule(a.Capsule, b.Capsule, computeContact),
            _ when a.Kind == ShapeKind.Polygon || b.Kind == ShapeKind.Polygon =>
                SatShapeVsShape(a, b, computeContact),
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

    private static Manifold SatShapeVsShape(
        in Shape a, in Shape b, bool computeContact)
    {
        int piecesA = PieceCount(a);
        int piecesB = PieceCount(b);
        if (piecesA > 1 || piecesB > 1)
            return ConcaveShapeVsShape(a, b, piecesA, piecesB, computeContact);

        FxManifold best = FxManifold.None;

        for (int pieceA = 0; pieceA < piecesA; pieceA++)
        {
            ConvexProxy proxyA = CreateProxy(a, pieceA);
            for (int pieceB = 0; pieceB < piecesB; pieceB++)
            {
                FxManifold candidate = Sat(
                    proxyA, CreateProxy(b, pieceB), computeContact);
                if (candidate.Colliding && (!best.Colliding || candidate.Depth > best.Depth))
                    best = candidate;
            }
        }
        return best.ToManifold();
    }

    private static Manifold ConcaveShapeVsShape(
        in Shape a, in Shape b, int piecesA, int piecesB, bool computeContact)
    {
        FxVec2 offset = FxVec2.Zero;
        FxManifold first = FindDeepestPieceContact(
            a, b, piecesA, piecesB, offset, computeContact);
        if (!first.Colliding) return Manifold.None;

        int iterationLimit = Math.Min(64, 8 + 2 * (piecesA + piecesB));
        for (int iteration = 0; iteration < iterationLimit; iteration++)
        {
            // Iteration zero has the same offset as `first`, so reuse it. Later
            // candidates contribute only their normal/depth to the accumulated
            // push (the contact is only ever taken from `first`), so skip their
            // otherwise-discarded support-feature contact work.
            FxManifold candidate = iteration == 0
                ? first
                : FindDeepestPieceContact(
                    a, b, piecesA, piecesB, offset, computeContact: false);
            if (!candidate.Colliding)
            {
                long depth = offset.Length + 2;
                FxAxis normal = FxAxis.FromVector(-offset, first.Normal);
                return new FxManifold(true, normal, depth, first.Contact).ToManifold();
            }

            long stepDepth = candidate.Depth + 2;
            offset -= candidate.Normal.Scale(stepDepth);
        }

        BpBounds boundsA = new(a);
        BpBounds boundsB = new(b);
        FxVec2 separation = GuaranteedBoundsSeparation(boundsA, boundsB);
        FxAxis fallbackNormal = FxAxis.FromVector(-separation, first.Normal);
        return new FxManifold(true, fallbackNormal, separation.Length, first.Contact).ToManifold();
    }

    private static FxVec2 GuaranteedBoundsSeparation(in BpBounds a, in BpBounds b)
    {
        long left = b.MinX - a.MaxX - 2;
        long right = b.MaxX - a.MinX + 2;
        long down = b.MinY - a.MaxY - 2;
        long up = b.MaxY - a.MinY + 2;

        FxVec2 best = new(left, 0);
        ulong magnitude = Fx.Magnitude(left);
        if (Fx.Magnitude(right) < magnitude)
        {
            best = new FxVec2(right, 0);
            magnitude = Fx.Magnitude(right);
        }
        if (Fx.Magnitude(down) < magnitude)
        {
            best = new FxVec2(0, down);
            magnitude = Fx.Magnitude(down);
        }
        if (Fx.Magnitude(up) < magnitude)
            best = new FxVec2(0, up);
        return best;
    }

    private static FxManifold FindDeepestPieceContact(
        in Shape a, in Shape b, int piecesA, int piecesB,
        FxVec2 offsetA, bool computeContact)
    {
        FxManifold best = FxManifold.None;
        for (int pieceA = 0; pieceA < piecesA; pieceA++)
        {
            ConvexProxy proxyA = CreateProxy(a, pieceA).Translated(offsetA);
            for (int pieceB = 0; pieceB < piecesB; pieceB++)
            {
                FxManifold candidate = Sat(
                    proxyA, CreateProxy(b, pieceB), computeContact);
                if (candidate.Colliding && IsBetterPieceContact(candidate, best))
                    best = candidate;
            }
        }
        return best;
    }

    private static bool IsBetterPieceContact(FxManifold candidate, FxManifold best)
    {
        if (!best.Colliding || candidate.Depth > best.Depth) return true;
        if (candidate.Depth < best.Depth) return false;

        FxAxis candidateAxis = CanonicalAxis(candidate.Normal);
        FxAxis bestAxis = CanonicalAxis(best.Normal);
        return candidateAxis.X < bestAxis.X
            || candidateAxis.X == bestAxis.X && candidateAxis.Y < bestAxis.Y;
    }

    private static FxAxis CanonicalAxis(FxAxis axis) =>
        axis.X < 0 || axis.X == 0 && axis.Y < 0 ? -axis : axis;

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
        public readonly FxAxis AxisX;
        public readonly FxAxis AxisY;
        public readonly long HalfX;
        public readonly long HalfY;

        public BoxProxy(FxVec2 center, FxAxis axisX, FxAxis axisY, long halfX, long halfY)
        {
            Center = center;
            AxisX = axisX;
            AxisY = axisY;
            HalfX = halfX;
            HalfY = halfY;
        }
    }

    // A box's four corners and its enclosing bounds, materialized once so corner
    // SAT, support-feature contact, and contact clamping share them instead of
    // re-deriving each vertex on demand. Axis scaling uses symmetric fixed
    // rounding, so Scale(-h) == -Scale(h); the corners are therefore bit-identical
    // to the former per-index BoxVertex, and the bounds to the former BoxBounds.
    private readonly struct BoxGeometry
    {
        public readonly FxVec2 V0, V1, V2, V3;
        public readonly FxAabb Bounds;

        public BoxGeometry(in BoxProxy box)
        {
            FxVec2 x = box.AxisX.Scale(box.HalfX);
            FxVec2 y = box.AxisY.Scale(box.HalfY);
            V0 = box.Center - x - y;
            V1 = box.Center + x - y;
            V2 = box.Center + x + y;
            V3 = box.Center - x + y;
            long hx = Math.Abs(x.X) + Math.Abs(y.X);
            long hy = Math.Abs(x.Y) + Math.Abs(y.Y);
            Bounds = new FxAabb(box.Center, new FxVec2(hx, hy));
        }

        public FxVec2 Vertex(int index) => index switch
        {
            0 => V0,
            1 => V1,
            2 => V2,
            3 => V3,
            _ => throw new ArgumentOutOfRangeException(nameof(index)),
        };
    }

    private static BoxProxy CreateBox(Aabb box) => new(
        FxVec2.From(box.Center), FxAxis.UnitX, FxAxis.UnitY,
        Math.Abs(Fx.From(box.HalfExtents.X)), Math.Abs(Fx.From(box.HalfExtents.Y)));

    private static BoxProxy CreateBox(Obb box)
    {
        FxAxis axisX = FxAxis.FromAngle(box.Angle);
        FxAxis axisY = axisX.Perpendicular;
        return new BoxProxy(FxVec2.From(box.Center), axisX, axisY,
            Math.Abs(Fx.From(box.HalfExtents.X)), Math.Abs(Fx.From(box.HalfExtents.Y)));
    }

    private static Manifold CircleVsObb(
        Circle circle, Obb box, bool computeContact)
    {
        BoxProxy target = CreateBox(box);
        FxCircle source = FxCircle.From(circle);
        FxVec2 delta = source.Center - target.Center;
        var localCenter = new FxVec2(
            Fx.RoundDiv(target.AxisX.Dot(delta), FxAxis.One),
            Fx.RoundDiv(target.AxisY.Dot(delta), FxAxis.One));
        FxManifold local = CircleVsAabbFx(
            new FxCircle(localCenter, Math.Abs(source.Radius)),
            new FxAabb(FxVec2.Zero, new FxVec2(target.HalfX, target.HalfY)),
            computeContact);
        if (!local.Colliding)
            return Manifold.None;

        FxAxis normal = FxAxis.Transform(target.AxisX, target.AxisY, local.Normal);
        FxVec2 contact = computeContact
            ? target.Center
                + target.AxisX.Scale(local.Contact.X)
                + target.AxisY.Scale(local.Contact.Y)
            : FxVec2.Zero;
        return new FxManifold(true, normal, local.Depth, contact).ToManifold();
    }

    private static bool CircleObbOverlap(Circle circle, Obb box)
    {
        BoxProxy target = CreateBox(box);
        FxCircle source = FxCircle.From(circle);
        FxVec2 delta = source.Center - target.Center;
        long localX = Fx.RoundDiv(target.AxisX.Dot(delta), FxAxis.One);
        long localY = Fx.RoundDiv(target.AxisY.Dot(delta), FxAxis.One);
        long closestX = Math.Clamp(localX, -target.HalfX, target.HalfX);
        long closestY = Math.Clamp(localY, -target.HalfY, target.HalfY);
        long dx = localX - closestX;
        long dy = localY - closestY;
        long radius = Math.Abs(source.Radius);
        return dx * dx + dy * dy <= radius * radius;
    }

    private static Manifold BoxVsBox(
        in BoxProxy a, in BoxProxy b, bool computeContact)
    {
        long depth = long.MaxValue;
        long overlap = long.MaxValue;
        FxAxis axis = FxAxis.UnitX;
        FxVec2 centerDelta = b.Center - a.Center;
        // A box basis is orthonormal, so each symmetric axis dot is worth
        // computing once and the cross self-dots (AxisX.AxisY) are exactly zero;
        // later-axis dots are deferred so an early separation stays cheap. This is
        // bit-identical to projecting each axis with the general ProjectBox.
        long aSelf = a.AxisX.Dot(a.AxisX);
        long axbx = a.AxisX.Dot(b.AxisX);
        long axby = a.AxisX.Dot(b.AxisY);
        if (!TestBoxAxis(a.AxisX, a, b, aSelf, 0, axbx, axby,
                centerDelta, ref overlap, ref depth, ref axis))
            return Manifold.None;
        long aybx = a.AxisY.Dot(b.AxisX);
        long ayby = a.AxisY.Dot(b.AxisY);
        if (!TestBoxAxis(a.AxisY, a, b, 0, aSelf, aybx, ayby,
                centerDelta, ref overlap, ref depth, ref axis))
            return Manifold.None;
        long bSelf = b.AxisX.Dot(b.AxisX);
        if (!TestBoxAxis(b.AxisX, a, b, axbx, aybx, bSelf, 0,
                centerDelta, ref overlap, ref depth, ref axis)
            || !TestBoxAxis(b.AxisY, a, b, axby, ayby, 0, bSelf,
                centerDelta, ref overlap, ref depth, ref axis))
            return Manifold.None;

        FxAxis normalAxis = axis;
        FxVec2 contact = FxVec2.Zero;
        if (computeContact)
        {
            BoxGeometry geometryA = new(a);
            BoxGeometry geometryB = new(b);
            contact = ClampContact(
                SupportFeatureContact(
                    new ConvexProxy(
                        geometryA.V0, geometryA.V1, geometryA.V2, geometryA.V3),
                    new ConvexProxy(
                        geometryB.V0, geometryB.V1, geometryB.V2, geometryB.V3),
                    normalAxis),
                geometryA.Bounds, geometryB.Bounds);
        }
        return new FxManifold(true, normalAxis, depth, contact).ToManifold();
    }

    private static bool BoxesOverlap(in BoxProxy a, in BoxProxy b)
    {
        long aSelf = a.AxisX.Dot(a.AxisX);
        long axbx = a.AxisX.Dot(b.AxisX);
        long axby = a.AxisX.Dot(b.AxisY);
        if (!BoxAxisOverlaps(a.AxisX, a, b, aSelf, 0, axbx, axby)) return false;
        long aybx = a.AxisY.Dot(b.AxisX);
        long ayby = a.AxisY.Dot(b.AxisY);
        if (!BoxAxisOverlaps(a.AxisY, a, b, 0, aSelf, aybx, ayby)) return false;
        long bSelf = b.AxisX.Dot(b.AxisX);
        return BoxAxisOverlaps(b.AxisX, a, b, axbx, aybx, bSelf, 0)
            && BoxAxisOverlaps(b.AxisY, a, b, axby, ayby, 0, bSelf);
    }

    private static bool TestBoxAxis(
        FxAxis testAxis, in BoxProxy a, in BoxProxy b,
        long aAxisXDot, long aAxisYDot, long bAxisXDot, long bAxisYDot,
        FxVec2 centerDelta,
        ref long bestOverlap, ref long bestDepth, ref FxAxis bestAxis)
    {
        ProjectBoxCached(a, testAxis, aAxisXDot, aAxisYDot, out long minA, out long maxA);
        ProjectBoxCached(b, testAxis, bAxisXDot, bAxisYDot, out long minB, out long maxB);
        long towardPositive = maxA - minB;
        long towardNegative = maxB - minA;
        long overlap = Math.Min(towardPositive, towardNegative);
        if (overlap < 0) return false;
        long depth = Fx.RoundDiv(overlap, FxAxis.One);
        FxAxis oriented = OrientAxis(
            testAxis, towardPositive, towardNegative, centerDelta);
        if (overlap < bestOverlap || overlap == bestOverlap &&
            IsCanonicalAxisBefore(oriented, bestAxis))
        {
            bestOverlap = overlap;
            bestDepth = depth;
            bestAxis = oriented;
        }
        return true;
    }

    private static bool BoxAxisOverlaps(
        FxAxis axis, in BoxProxy a, in BoxProxy b,
        long aAxisXDot, long aAxisYDot, long bAxisXDot, long bAxisYDot)
    {
        ProjectBoxCached(a, axis, aAxisXDot, aAxisYDot, out long minA, out long maxA);
        ProjectBoxCached(b, axis, bAxisXDot, bAxisYDot, out long minB, out long maxB);
        return maxA >= minB && maxB >= minA;
    }

    // General box projection onto an arbitrary axis (used by capsule/box, where
    // the test axes are not the box's own axes so no dot can be cached).
    private static void ProjectBox(in BoxProxy box, FxAxis axis, out long min, out long max)
    {
        long center = axis.Dot(box.Center);
        long radius = Math.Abs(box.AxisX.Dot(axis)) * box.HalfX
            + Math.Abs(box.AxisY.Dot(axis)) * box.HalfY;
        min = center - radius;
        max = center + radius;
    }

    // Same arithmetic as ProjectBox with the axis/box-axis dots supplied by the
    // box/box SAT, which computes each only once.
    private static void ProjectBoxCached(
        in BoxProxy box, FxAxis axis, long axisXDot, long axisYDot,
        out long min, out long max)
    {
        long center = axis.Dot(box.Center);
        long radius = Math.Abs(axisXDot) * box.HalfX + Math.Abs(axisYDot) * box.HalfY;
        min = center - radius;
        max = center + radius;
    }

    private static FxVec2 BoxSupport(in BoxProxy box, FxAxis direction)
    {
        FxVec2 result = box.Center;
        result += box.AxisX.Scale(box.AxisX.Dot(direction) >= 0 ? box.HalfX : -box.HalfX);
        result += box.AxisY.Scale(box.AxisY.Dot(direction) >= 0 ? box.HalfY : -box.HalfY);
        return result;
    }

    private static Manifold CapsuleVsBox(
        Capsule capsule, in BoxProxy box, bool computeContact = true)
    {
        FxVec2 a = FxVec2.From(capsule.A);
        FxVec2 b = FxVec2.From(capsule.B);
        long radius = Math.Abs(Fx.From(capsule.Radius));
        long depth = long.MaxValue;
        long overlap = long.MaxValue;
        FxAxis axis = FxAxis.UnitX;

        if (!TestCapsuleBoxAxis(box.AxisX, a, b, radius, box, ref overlap, ref depth, ref axis)
            || !TestCapsuleBoxAxis(box.AxisY, a, b, radius, box, ref overlap, ref depth, ref axis))
            return Manifold.None;

        FxVec2 spine = b - a;
        if (spine.LengthSq != 0 && !TestCapsuleBoxAxis(
                FxAxis.FromVector(new FxVec2(-spine.Y, spine.X), FxAxis.UnitY),
                a, b, radius, box, ref overlap, ref depth, ref axis))
            return Manifold.None;

        BoxGeometry geometry = new(box);
        for (int corner = 0; corner < 4; corner++)
        {
            FxVec2 vertex = geometry.Vertex(corner);
            FxVec2 closest = Distance.ClosestPointOnSegmentFx(vertex, a, b, out _);
            FxVec2 rawAxis = closest - vertex;
            if (rawAxis.LengthSq != 0 && !TestCapsuleBoxAxis(
                    FxAxis.FromVector(rawAxis, FxAxis.UnitX),
                    a, b, radius, box, ref overlap, ref depth, ref axis))
                return Manifold.None;
        }

        FxAxis normalAxis = axis;
        FxVec2 contact = computeContact
            ? ClampContact(
                SupportFeatureContact(
                    new ConvexProxy(a, b, radius),
                    new ConvexProxy(
                        geometry.V0, geometry.V1, geometry.V2, geometry.V3),
                    normalAxis),
                SegmentBounds(a, b, radius), geometry.Bounds)
            : FxVec2.Zero;
        return new FxManifold(true, normalAxis, depth, contact).ToManifold();
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
        if (spine.LengthSq != 0 && !CapsuleBoxAxisOverlaps(
                FxAxis.FromVector(new FxVec2(-spine.Y, spine.X), FxAxis.UnitY),
                a, b, radius, box))
            return false;

        BoxGeometry geometry = new(box);
        for (int corner = 0; corner < 4; corner++)
        {
            FxVec2 vertex = geometry.Vertex(corner);
            FxVec2 closest = Distance.ClosestPointOnSegmentFx(vertex, a, b, out _);
            FxVec2 rawAxis = closest - vertex;
            if (rawAxis.LengthSq != 0 && !CapsuleBoxAxisOverlaps(
                    FxAxis.FromVector(rawAxis, FxAxis.UnitX), a, b, radius, box))
                return false;
        }
        return true;
    }

    private static bool TestCapsuleBoxAxis(
        FxAxis testAxis, FxVec2 a, FxVec2 b, long radius, in BoxProxy box,
        ref long bestOverlap, ref long bestDepth, ref FxAxis bestAxis)
    {
        ProjectCapsule(a, b, radius, testAxis, out long minA, out long maxA);
        ProjectBox(box, testAxis, out long minB, out long maxB);
        long towardPositive = maxA - minB;
        long towardNegative = maxB - minA;
        long overlap = Math.Min(towardPositive, towardNegative);
        if (overlap < 0) return false;
        long depth = Fx.RoundDiv(overlap, FxAxis.One);
        FxAxis oriented = OrientAxis(testAxis, towardPositive, towardNegative,
            box.Center - Midpoint(a, b));
        if (overlap < bestOverlap || overlap == bestOverlap &&
            IsCanonicalAxisBefore(oriented, bestAxis))
        {
            bestOverlap = overlap;
            bestDepth = depth;
            bestAxis = oriented;
        }
        return true;
    }

    private static bool CapsuleBoxAxisOverlaps(
        FxAxis axis, FxVec2 a, FxVec2 b, long radius, in BoxProxy box)
    {
        ProjectCapsule(a, b, radius, axis, out long minA, out long maxA);
        ProjectBox(box, axis, out long minB, out long maxB);
        return maxA >= minB && maxB >= minA;
    }

    private static void ProjectCapsule(
        FxVec2 a, FxVec2 b, long radius, FxAxis axis,
        out long min, out long max)
    {
        long first = axis.Dot(a);
        long second = axis.Dot(b);
        long radiusProjection = radius * FxAxis.One;
        min = Math.Min(first, second) - radiusProjection;
        max = Math.Max(first, second) + radiusProjection;
    }


    private readonly struct ConvexProxy
    {
        private readonly FxVec2 _v0, _v1, _v2, _v3;
        private readonly FxVec2[]? _vertices;
        private readonly int[]? _indices;
        private readonly int _indexOffset;
        private readonly FxVec2 _offset;
        private readonly FxAxis _axisX;
        private readonly FxAxis _axisY;
        private readonly bool _rotated;

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

        public ConvexProxy(
            FxVec2 v0, FxVec2 v1, FxVec2 v2, FxVec2 v3, long radius)
            : this(v0, v1, v2, v3)
        {
            Radius = radius;
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

        private ConvexProxy(in ConvexProxy source, FxVec2 offset)
        {
            _v0 = source._v0;
            _v1 = source._v1;
            _v2 = source._v2;
            _v3 = source._v3;
            _vertices = source._vertices;
            _indices = source._indices;
            _indexOffset = source._indexOffset;
            _offset = source._offset + offset;
            _axisX = source._axisX;
            _axisY = source._axisY;
            _rotated = source._rotated;
            Count = source.Count;
            Radius = source.Radius;
            Center = source.Center + offset;
        }

        private ConvexProxy(
            in ConvexProxy source, FxVec2 translation, Angle32 rotation)
        {
            _v0 = source._v0;
            _v1 = source._v1;
            _v2 = source._v2;
            _v3 = source._v3;
            _vertices = source._vertices;
            _indices = source._indices;
            _indexOffset = source._indexOffset;
            _offset = translation;
            _axisX = FxAxis.FromAngle(rotation);
            _axisY = _axisX.Perpendicular;
            _rotated = true;
            Count = source.Count;
            Radius = source.Radius;
            Center = Transform(source.Center) + translation;
        }

        public ConvexProxy Translated(FxVec2 offset) =>
            offset.X == 0 && offset.Y == 0 ? this : new ConvexProxy(this, offset);

        public ConvexProxy Transformed(Vec2 translation, Angle32 rotation)
        {
            FxVec2 offset = FxVec2.From(translation);
            return rotation.Raw == 0
                ? Translated(offset)
                : new ConvexProxy(this, offset, rotation);
        }

        public FxVec2 Vertex(int index)
        {
            if (_vertices != null)
            {
                FxVec2 value = _vertices[
                    _indices == null ? index : _indices[_indexOffset + index]];
                return (_rotated ? Transform(value) : value) + _offset;
            }
            FxVec2 vertex = index switch
            {
                0 => _v0,
                1 => _v1,
                2 => _v2,
                3 => _v3,
                _ => throw new ArgumentOutOfRangeException(nameof(index)),
            };
            return (_rotated ? Transform(vertex) : vertex) + _offset;
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

        private FxVec2 Transform(FxVec2 vertex) =>
            _axisX.Scale(vertex.X) + _axisY.Scale(vertex.Y);
    }

    private struct SatState
    {
        public long Depth;
        public long Overlap;
        public FxAxis Axis;
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
                BoxProxy box = CreateBox(shape.Obb);
                FxVec2 x = box.AxisX.Scale(box.HalfX);
                FxVec2 y = box.AxisY.Scale(box.HalfY);
                return new ConvexProxy(box.Center - x - y, box.Center + x - y,
                    box.Center + x + y, box.Center - x + y);
            }
            case ShapeKind.Polygon:
            {
                Polygon polygon = shape.Polygon;
                ConvexProxy proxy = polygon.IsConvex
                    ? new ConvexProxy(polygon.FixedVertices)
                    : new ConvexProxy(polygon.FixedVertices, polygon.TriangleIndices, piece * 3);
                return proxy.Transformed(shape.PolygonTranslation, shape.PolygonRotation);
            }
            default:
                throw new ArgumentOutOfRangeException(nameof(shape));
        }
    }

    private static FxManifold Sat(
        in ConvexProxy a, in ConvexProxy b, bool computeContact = true)
    {
        var state = new SatState { Depth = long.MaxValue, Overlap = long.MaxValue };
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

        FxAxis normalAxis = state.Axis;
        FxVec2 contact = FxVec2.Zero;
        if (computeContact)
        {
            contact = SupportFeatureContact(a, b, normalAxis);
            contact = ClampContact(contact, ProxyBounds(a), ProxyBounds(b));
        }
        return new FxManifold(true, normalAxis, state.Depth, contact);
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
        // Radius projections require a unit axis. FxAxis normalizes to Q1.30
        // with adaptive input precision, including for very short edges.
        FxAxis unit = FxAxis.FromVector(axis, FxAxis.Zero);
        if (unit.IsZero) return true;
        hasAxis = true;
        Project(a, unit, out long minA, out long maxA);
        Project(b, unit, out long minB, out long maxB);
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
        // Normalize to Q1.30 before projecting. Adaptive pre-scaling preserves
        // precision for near-degenerate polygon edges before the integer sqrt.
        FxAxis unit = FxAxis.FromVector(axis, FxAxis.Zero);
        if (unit.IsZero) return true;
        Project(a, unit, out long minA, out long maxA);
        Project(b, unit, out long minB, out long maxB);
        long towardPositive = maxA - minB;
        long towardNegative = maxB - minA;
        if (towardPositive < 0 || towardNegative < 0)
            return false;

        long overlap = Math.Min(towardPositive, towardNegative);
        long depth = Fx.RoundDiv(overlap, FxAxis.One);
        FxAxis oriented = OrientAxis(unit, towardPositive, towardNegative,
            b.Center - a.Center);
        bool better = !state.HasAxis || overlap < state.Overlap;
        if (!better && state.HasAxis && overlap == state.Overlap)
        {
            better = IsCanonicalAxisBefore(oriented, state.Axis);
        }
        if (better)
        {
            state.HasAxis = true;
            state.Depth = depth;
            state.Overlap = overlap;
            state.Axis = oriented;
        }
        return true;
    }

    private static FxAxis OrientAxis(
        FxAxis axis, long towardPositive, long towardNegative, FxVec2 centerDelta)
    {
        if (towardPositive < towardNegative) return axis;
        if (towardNegative < towardPositive) return -axis;
        return axis.Dot(centerDelta) < 0 ? -axis : axis;
    }

    private static bool IsCanonicalAxisBefore(FxAxis candidate, FxAxis current)
    {
        FxAxis candidateAxis = CanonicalAxis(candidate);
        FxAxis currentAxis = CanonicalAxis(current);
        return candidateAxis.X < currentAxis.X
            || candidateAxis.X == currentAxis.X && candidateAxis.Y < currentAxis.Y;
    }

    private static void Project(
        in ConvexProxy proxy, FxAxis axis, out long min, out long max)
    {
        long projection = axis.Dot(proxy.Vertex(0));
        min = max = projection;
        for (int i = 1; i < proxy.Count; i++)
        {
            projection = axis.Dot(proxy.Vertex(i));
            min = Math.Min(min, projection);
            max = Math.Max(max, projection);
        }
        long radiusProjection = proxy.Radius * FxAxis.One;
        min -= radiusProjection;
        max += radiusProjection;
    }

    private static FxVec2 Support(in ConvexProxy proxy, FxAxis direction)
    {
        FxVec2 best = proxy.Vertex(0);
        long bestProjection = direction.Dot(best);
        for (int i = 1; i < proxy.Count; i++)
        {
            FxVec2 candidate = proxy.Vertex(i);
            long projection = direction.Dot(candidate);
            if (projection > bestProjection)
            {
                best = candidate;
                bestProjection = projection;
            }
        }
        return best + direction.Scale(proxy.Radius);
    }

    private const long SupportFeatureTolerance = 4 * FxAxis.One;

    private readonly struct SupportFeature
    {
        public readonly long Normal;
        public readonly long TangentMin;
        public readonly long TangentMax;

        public SupportFeature(long normal, long tangentMin, long tangentMax)
        {
            Normal = normal;
            TangentMin = tangentMin;
            TangentMax = tangentMax;
        }
    }

    // A face or capsule side is an entire support feature, not one arbitrary
    // vertex. Build both shapes' extreme features and choose the middle of their
    // tangential overlap (or the nearest feature endpoints for a corner contact).
    // This handles face/vertex box contacts and also prevents tiny Q1.30 capsule
    // projection noise from moving the contact to the far cap.
    private static FxVec2 SupportFeatureContact(
        in ConvexProxy a, in ConvexProxy b, FxAxis normal)
    {
        FxAxis tangent = normal.Perpendicular;
        SupportFeature featureA = FindSupportFeature(a, normal, tangent, maximum: true);
        SupportFeature featureB = FindSupportFeature(b, normal, tangent, maximum: false);

        long normalA = featureA.Normal + a.Radius * FxAxis.One;
        long normalB = featureB.Normal - b.Radius * FxAxis.One;
        long contactNormal = ProjectionMidpoint(normalA, normalB);

        long contactTangent;
        long overlapMin = Math.Max(featureA.TangentMin, featureB.TangentMin);
        long overlapMax = Math.Min(featureA.TangentMax, featureB.TangentMax);
        if (overlapMin <= overlapMax)
            contactTangent = ProjectionMidpoint(overlapMin, overlapMax);
        else if (featureA.TangentMax < featureB.TangentMin)
            contactTangent = ProjectionMidpoint(featureA.TangentMax, featureB.TangentMin);
        else
            contactTangent = ProjectionMidpoint(featureB.TangentMax, featureA.TangentMin);

        return normal.Scale(Fx.RoundDiv(contactNormal, FxAxis.One))
            + tangent.Scale(Fx.RoundDiv(contactTangent, FxAxis.One));
    }

    private static SupportFeature FindSupportFeature(
        in ConvexProxy proxy, FxAxis normal, FxAxis tangent, bool maximum)
    {
        long extreme = normal.Dot(proxy.Vertex(0));
        for (int i = 1; i < proxy.Count; i++)
        {
            long projection = normal.Dot(proxy.Vertex(i));
            extreme = maximum ? Math.Max(extreme, projection) : Math.Min(extreme, projection);
        }

        long tangentMin = long.MaxValue;
        long tangentMax = long.MinValue;
        for (int i = 0; i < proxy.Count; i++)
        {
            FxVec2 vertex = proxy.Vertex(i);
            long projection = normal.Dot(vertex);
            long distance = maximum ? extreme - projection : projection - extreme;
            if (distance > SupportFeatureTolerance) continue;
            long tangentProjection = tangent.Dot(vertex);
            tangentMin = Math.Min(tangentMin, tangentProjection);
            tangentMax = Math.Max(tangentMax, tangentProjection);
        }
        return new SupportFeature(extreme, tangentMin, tangentMax);
    }

    private static long ProjectionMidpoint(long a, long b) => a + ((b - a) >> 1);

    private static FxVec2 Midpoint(FxVec2 a, FxVec2 b) =>
        new(a.X + ((b.X - a.X) >> 1), a.Y + ((b.Y - a.Y) >> 1));

    // A SAT contact hint can fall just outside the shapes' overlap; clamp it into
    // the intersection of the operands' world AABBs so it is always at least
    // within both bounding boxes.
    private static FxVec2 ClampContact(FxVec2 contact, in FxAabb a, in FxAabb b)
    {
        long minX = Math.Max(a.Min.X, b.Min.X), maxX = Math.Min(a.Max.X, b.Max.X);
        long minY = Math.Max(a.Min.Y, b.Min.Y), maxY = Math.Min(a.Max.Y, b.Max.Y);
        if (minX > maxX || minY > maxY) return contact;   // not overlapping (shouldn't happen)
        return new FxVec2(Math.Clamp(contact.X, minX, maxX), Math.Clamp(contact.Y, minY, maxY));
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

}
