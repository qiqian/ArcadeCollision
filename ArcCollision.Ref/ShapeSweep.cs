using System;

namespace ArcCollision.Ref;

public enum SweepAlgorithm
{
    AnalyticCircle,
    RoundedAabb,
    RoundedSegment,
    LocalSpaceRoundedAabb,
    SweptAabb,
    ContinuousSat,
    FeatureCast,
}

public static partial class Sweep
{
    /// <summary>
    /// Sweeps any supported shape through a linear translation against a static
    /// target. Shape orientation remains fixed during the sweep. Concave
    /// polygons use their cached triangle decomposition and return the earliest
    /// piece contact.
    /// </summary>
    public static SweepHit MovingShapeVsShape(
        in Shape mover, Vec2 motion, in Shape target)
    {
        if (TryFastSweep(mover, motion, target, out SweepHit fastHit))
            return fastHit;

        Manifold initial = Collide.ShapeVsShape(mover, target);
        if (initial.Colliding)
            return new SweepHit(true, 0f, -initial.Normal, initial.Contact);

        FxVec2 motionFx = FxVec2.From(motion);
        int moverPieces = SweepPieceCount(mover);
        int targetPieces = SweepPieceCount(target);
        FxSweep best = FxSweep.Miss;
        for (int a = 0; a < moverPieces; a++)
        {
            SweepProxy proxyA = CreateSweepProxy(mover, a);
            for (int b = 0; b < targetPieces; b++)
                SweepConvex(proxyA, motionFx, CreateSweepProxy(target, b), ref best);
        }
        return best.ToSweepHit();
    }

    public static SweepAlgorithm GetAlgorithm(in Shape mover, in Shape target)
    {
        if (mover.Kind == ShapeKind.Circle && target.Kind == ShapeKind.Circle)
            return SweepAlgorithm.AnalyticCircle;
        if ((mover.Kind == ShapeKind.Circle && target.Kind == ShapeKind.Aabb)
            || (mover.Kind == ShapeKind.Aabb && target.Kind == ShapeKind.Circle))
            return SweepAlgorithm.RoundedAabb;
        if ((mover.Kind == ShapeKind.Circle && target.Kind == ShapeKind.Capsule)
            || (mover.Kind == ShapeKind.Capsule && target.Kind == ShapeKind.Circle))
            return SweepAlgorithm.RoundedSegment;
        if ((mover.Kind == ShapeKind.Circle && target.Kind == ShapeKind.Obb)
            || (mover.Kind == ShapeKind.Obb && target.Kind == ShapeKind.Circle))
            return SweepAlgorithm.LocalSpaceRoundedAabb;
        if (mover.Kind == ShapeKind.Aabb && target.Kind == ShapeKind.Aabb)
            return SweepAlgorithm.SweptAabb;
        if (IsPolygonal(mover.Kind) && IsPolygonal(target.Kind))
            return SweepAlgorithm.ContinuousSat;
        return SweepAlgorithm.FeatureCast;
    }

    private static bool TryFastSweep(
        in Shape mover, Vec2 motion, in Shape target, out SweepHit hit)
    {
        switch (mover.Kind, target.Kind)
        {
            case (ShapeKind.Circle, ShapeKind.Circle):
                hit = MovingCircleVsCircle(mover.Circle, motion, target.Circle);
                return true;
            case (ShapeKind.Circle, ShapeKind.Aabb):
                hit = MovingCircleVsAabb(mover.Circle, motion, target.Aabb);
                return true;
            case (ShapeKind.Circle, ShapeKind.Capsule):
                hit = MovingCircleVsCapsule(mover.Circle, motion, target.Capsule);
                return true;
            case (ShapeKind.Circle, ShapeKind.Obb):
                hit = MovingCircleVsObb(mover.Circle, motion, target.Obb);
                return true;
            case (ShapeKind.Aabb, ShapeKind.Aabb):
                hit = MovingAabbVsAabb(mover.Aabb, motion, target.Aabb);
                return true;
            case (ShapeKind.Aabb, ShapeKind.Circle):
                hit = ReverseRelative(
                    MovingCircleVsAabb(target.Circle, -motion, mover.Aabb), motion);
                return true;
            case (ShapeKind.Capsule, ShapeKind.Circle):
                hit = ReverseRelative(
                    MovingCircleVsCapsule(target.Circle, -motion, mover.Capsule), motion);
                return true;
            case (ShapeKind.Obb, ShapeKind.Circle):
                hit = ReverseRelative(
                    MovingCircleVsObb(target.Circle, -motion, mover.Obb), motion);
                return true;
            default:
                hit = SweepHit.Miss;
                return false;
        }
    }

    private static SweepHit ReverseRelative(SweepHit hit, Vec2 originalMotion)
    {
        if (!hit.Hit) return hit;
        return new SweepHit(true, hit.Time, -hit.Normal,
            hit.Point + originalMotion * hit.Time);
    }

    private static bool IsPolygonal(ShapeKind kind) =>
        kind is ShapeKind.Aabb or ShapeKind.Obb or ShapeKind.Polygon;

    private readonly struct SweepProxy
    {
        private readonly FxVec2 _v0, _v1, _v2, _v3;
        private readonly FxVec2[]? _vertices;
        private readonly int[]? _indices;
        private readonly int _indexOffset;
        private readonly FxVec2 _translation;
        private readonly FxAxis _axisX;
        private readonly FxAxis _axisY;
        private readonly bool _transformed;

        public readonly int Count;
        public readonly long Radius;

        public SweepProxy(FxVec2 point, long radius)
        {
            this = default;
            _v0 = point;
            Count = 1;
            Radius = radius;
        }

        public SweepProxy(FxVec2 a, FxVec2 b, long radius)
        {
            this = default;
            _v0 = a;
            _v1 = b;
            Count = 2;
            Radius = radius;
        }

        public SweepProxy(FxVec2 v0, FxVec2 v1, FxVec2 v2, FxVec2 v3)
        {
            this = default;
            _v0 = v0;
            _v1 = v1;
            _v2 = v2;
            _v3 = v3;
            Count = 4;
        }

        public SweepProxy(FxVec2[] vertices)
        {
            this = default;
            _vertices = vertices;
            Count = vertices.Length;
        }

        public SweepProxy(FxVec2[] vertices, int[] indices, int indexOffset)
        {
            this = default;
            _vertices = vertices;
            _indices = indices;
            _indexOffset = indexOffset;
            Count = 3;
        }

        private SweepProxy(
            FxVec2[] vertices,
            int[]? indices,
            int indexOffset,
            int count,
            Vec2 translation,
            Angle32 rotation)
        {
            this = default;
            _vertices = vertices;
            _indices = indices;
            _indexOffset = indexOffset;
            _translation = FxVec2.From(translation);
            _axisX = FxAxis.FromAngle(rotation);
            _axisY = _axisX.Perpendicular;
            _transformed = rotation.Raw != 0;
            Count = count;
        }

        public static SweepProxy Polygon(
            FxVec2[] vertices,
            int[]? indices,
            int indexOffset,
            int count,
            Vec2 translation,
            Angle32 rotation) =>
            new(vertices, indices, indexOffset, count, translation, rotation);

        public FxVec2 Vertex(int index)
        {
            if (_vertices != null)
            {
                FxVec2 value = _vertices[
                    _indices == null ? index : _indices[_indexOffset + index]];
                if (_transformed)
                    value = _axisX.Scale(value.X) + _axisY.Scale(value.Y);
                return value + _translation;
            }
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
    }

    private static void SweepConvex(
        in SweepProxy mover, FxVec2 motion, in SweepProxy target, ref FxSweep best)
    {
        if (mover.Radius == 0 && target.Radius == 0
            && mover.Count >= 3 && target.Count >= 3)
        {
            AddEarlier(SweptSat(mover, motion, target), ref best);
            return;
        }

        long radius = mover.Radius + target.Radius;

        if (mover.Count == 1 && target.Count == 1)
        {
            FxSweep hit = RayVsRoundedPoint(
                mover.Vertex(0), motion, target.Vertex(0), radius);
            AddMoverVertexHit(hit, mover.Radius, ref best);
            return;
        }

        for (int vertex = 0; vertex < mover.Count; vertex++)
        {
            FxVec2 point = mover.Vertex(vertex);
            for (int edge = 0; edge < target.EdgeCount; edge++)
            {
                target.Edge(edge, out FxVec2 e0, out FxVec2 e1);
                FxSweep hit = RayVsCapsuleCore(point, motion, e0, e1, radius);
                AddMoverVertexHit(hit, mover.Radius, ref best);
            }
        }

        FxVec2 reverseMotion = -motion;
        for (int vertex = 0; vertex < target.Count; vertex++)
        {
            FxVec2 point = target.Vertex(vertex);
            for (int edge = 0; edge < mover.EdgeCount; edge++)
            {
                mover.Edge(edge, out FxVec2 e0, out FxVec2 e1);
                FxSweep hit = RayVsCapsuleCore(point, reverseMotion, e0, e1, radius);
                if (!hit.Hit || (best.Hit && hit.Time16 >= best.Time16))
                    continue;

                FxAxis normal = -hit.Normal;
                FxVec2 contact = point + normal.Scale(target.Radius);
                best = new FxSweep(true, hit.Time16, normal, contact);
            }
        }
    }

    private static void AddMoverVertexHit(FxSweep hit, long moverRadius, ref FxSweep best)
    {
        if (!hit.Hit || (best.Hit && hit.Time16 >= best.Time16))
            return;
        FxVec2 contact = hit.Point - hit.Normal.Scale(moverRadius);
        best = new FxSweep(true, hit.Time16, hit.Normal, contact);
    }

    private static FxSweep SweptSat(
        in SweepProxy mover, FxVec2 motion, in SweepProxy target)
    {
        long enter = 0;
        long exit = Fx.TOne;
        FxAxis normal = FxAxis.Zero;
        if (!SweepAxes(mover, mover, motion, target, ref enter, ref exit, ref normal)
            || !SweepAxes(target, mover, motion, target, ref enter, ref exit, ref normal)
            || enter < 0 || enter > Fx.TOne)
            return FxSweep.Miss;

        if (normal.IsZero)
            normal = FxAxis.FromVector(-motion, FxAxis.UnitX);
        FxVec2 pointA = SweepSupport(mover, -normal) + motion.MulT(enter);
        FxVec2 pointB = SweepSupport(target, normal);
        FxVec2 contact = new(
            pointA.X + ((pointB.X - pointA.X) >> 1),
            pointA.Y + ((pointB.Y - pointA.Y) >> 1));
        return new FxSweep(true, enter, normal, contact);
    }

    private static bool SweepAxes(
        in SweepProxy source, in SweepProxy mover, FxVec2 motion,
        in SweepProxy target, ref long enter, ref long exit, ref FxAxis normal)
    {
        for (int edge = 0; edge < source.EdgeCount; edge++)
        {
            source.Edge(edge, out FxVec2 p0, out FxVec2 p1);
            FxVec2 delta = p1 - p0;
            FxAxis axis = FxAxis.FromVector(
                new FxVec2(-delta.Y, delta.X), FxAxis.UnitX);
            if (!SweepAxis(axis, mover, motion, target,
                    ref enter, ref exit, ref normal))
                return false;
        }
        return true;
    }

    private static bool SweepAxis(
        FxAxis axis, in SweepProxy mover, FxVec2 motion, in SweepProxy target,
        ref long enter, ref long exit, ref FxAxis normal)
    {
        ProjectSweep(mover, axis, out long minA, out long maxA);
        ProjectSweep(target, axis, out long minB, out long maxB);
        long velocity = axis.Dot(motion);
        if (velocity == 0)
            return maxA >= minB && maxB >= minA;

        long axisEnter = Fx.RatioT(minB - maxA, velocity);
        long axisExit = Fx.RatioT(maxB - minA, velocity);
        if (axisEnter > axisExit)
            (axisEnter, axisExit) = (axisExit, axisEnter);
        if (axisEnter > enter)
        {
            enter = axisEnter;
            normal = velocity > 0 ? -axis : axis;
        }
        exit = Math.Min(exit, axisExit);
        return enter <= exit;
    }

    private static void ProjectSweep(
        in SweepProxy proxy, FxAxis axis, out long min, out long max)
    {
        long projection = axis.Dot(proxy.Vertex(0));
        min = max = projection;
        for (int i = 1; i < proxy.Count; i++)
        {
            projection = axis.Dot(proxy.Vertex(i));
            min = Math.Min(min, projection);
            max = Math.Max(max, projection);
        }
    }

    private static FxVec2 SweepSupport(in SweepProxy proxy, FxAxis direction)
    {
        FxVec2 best = proxy.Vertex(0);
        long projection = direction.Dot(best);
        for (int i = 1; i < proxy.Count; i++)
        {
            FxVec2 candidate = proxy.Vertex(i);
            long candidateProjection = direction.Dot(candidate);
            if (candidateProjection > projection)
            {
                best = candidate;
                projection = candidateProjection;
            }
        }
        return best;
    }

    /// <summary>Ray point versus a rounded segment.</summary>
    private static FxSweep RayVsCapsuleCore(
        FxVec2 origin, FxVec2 motion, FxVec2 a, FxVec2 b, long radius)
    {
        FxVec2 segment = b - a;
        if (segment.LengthSq == 0)
            return RayVsCircleFx(origin, motion, new FxCircle(a, radius));

        FxSweep best = FxSweep.Miss;
        AddEarlier(RayVsRoundedPoint(origin, motion, a, radius), ref best);
        AddEarlier(RayVsRoundedPoint(origin, motion, b, radius), ref best);

        FxAxis unitNormal = FxAxis.FromVector(
            new FxVec2(-segment.Y, segment.X), FxAxis.UnitY);
        long position = Fx.RoundDiv(unitNormal.Dot(origin - a), FxAxis.One);
        long velocity = Fx.RoundDiv(unitNormal.Dot(motion), FxAxis.One);
        long entry;
        long exit;
        if (velocity == 0)
        {
            if (Math.Abs(position) > radius)
                return best;
            entry = 0;
            exit = Fx.TOne;
        }
        else
        {
            long t1 = Fx.RatioT(-radius - position, velocity);
            long t2 = Fx.RatioT(radius - position, velocity);
            entry = Math.Max(0, Math.Min(t1, t2));
            exit = Math.Min(Fx.TOne, Math.Max(t1, t2));
            if (entry > exit)
                return best;
        }

        FxVec2 point = origin + motion.MulT(entry);
        long projection = (point - a).Dot(segment);
        if (projection >= 0 && projection <= segment.LengthSq)
        {
            FxVec2 closest = Distance.ClosestPointOnSegmentFx(point, a, b, out _);
            FxAxis normal = FxAxis.FromVector(point - closest,
                FxAxis.FromVector(-motion, unitNormal));
            AddEarlier(new FxSweep(true, entry, normal, point), ref best);
        }
        return best;
    }

    private static FxSweep RayVsRoundedPoint(
        FxVec2 origin, FxVec2 motion, FxVec2 center, long radius)
    {
        FxSweep hit = RayVsCircleFx(origin, motion, new FxCircle(center, radius));
        if (!hit.Hit || (hit.Point - center).LengthSq != 0)
            return hit;
        FxAxis normal = FxAxis.FromVector(-motion, FxAxis.UnitX);
        return new FxSweep(true, hit.Time16, normal, hit.Point);
    }

    private static void AddEarlier(FxSweep candidate, ref FxSweep best)
    {
        if (candidate.Hit && (!best.Hit || candidate.Time16 < best.Time16))
            best = candidate;
    }

    private static int SweepPieceCount(in Shape shape) =>
        shape.Kind == ShapeKind.Polygon && !shape.Polygon.IsConvex
            ? shape.Polygon.TriangleIndices.Length / 3
            : 1;

    private static SweepProxy CreateSweepProxy(in Shape shape, int piece)
    {
        switch (shape.Kind)
        {
            case ShapeKind.Circle:
            {
                Circle circle = shape.Circle;
                return new SweepProxy(
                    FxVec2.From(circle.Center), Math.Abs(Fx.From(circle.Radius)));
            }
            case ShapeKind.Capsule:
            {
                Capsule capsule = shape.Capsule;
                return new SweepProxy(FxVec2.From(capsule.A), FxVec2.From(capsule.B),
                    Math.Abs(Fx.From(capsule.Radius)));
            }
            case ShapeKind.Aabb:
            {
                Aabb box = shape.Aabb;
                FxVec2 center = FxVec2.From(box.Center);
                var half = new FxVec2(Math.Abs(Fx.From(box.HalfExtents.X)),
                    Math.Abs(Fx.From(box.HalfExtents.Y)));
                FxVec2 min = center - half;
                FxVec2 max = center + half;
                return new SweepProxy(min, new FxVec2(max.X, min.Y), max,
                    new FxVec2(min.X, max.Y));
            }
            case ShapeKind.Obb:
            {
                Obb box = shape.Obb;
                FxAxis axisX = FxAxis.FromAngle(box.Angle);
                FxAxis axisY = axisX.Perpendicular;
                FxVec2 center = FxVec2.From(box.Center);
                FxVec2 x = axisX.Scale(Math.Abs(Fx.From(box.HalfExtents.X)));
                FxVec2 y = axisY.Scale(Math.Abs(Fx.From(box.HalfExtents.Y)));
                return new SweepProxy(
                    center - x - y, center + x - y,
                    center + x + y, center - x + y);
            }
            case ShapeKind.Polygon:
            {
                Polygon polygon = shape.Polygon;
                return polygon.IsConvex
                    ? SweepProxy.Polygon(
                        polygon.FixedVertices, null, 0, polygon.FixedVertices.Length,
                        shape.PolygonTranslation, shape.PolygonRotation)
                    : SweepProxy.Polygon(
                        polygon.FixedVertices, polygon.TriangleIndices, piece * 3, 3,
                        shape.PolygonTranslation, shape.PolygonRotation);
            }
            default:
                throw new ArgumentOutOfRangeException(nameof(shape));
        }
    }
}
