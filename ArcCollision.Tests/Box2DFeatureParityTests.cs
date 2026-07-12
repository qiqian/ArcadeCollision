using ArcCollision;
using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Direct ports of the supported portions of box2d/test/test_distance.c.
/// The original polygon-versus-segment proxy is represented by an AABB and a
/// zero-radius Capsule, which are the equivalent ArcCollision.Ref shapes.
/// </summary>
public class Box2DDistanceSweepToiParityTests
{
    private static readonly Aabb Box = new(Vec2.Zero, new Vec2(1, 1));
    private static readonly Capsule Segment = new(
        new Vec2(2, -1), new Vec2(2, 1), 0);

    // box2d/test/test_distance.c: ShapeDistanceTest
    [Fact]
    public void ShapeDistance_Box2DFixture_IsOne()
    {
        Vec2[] corners =
        {
            new(-1, -1), new(1, -1), new(1, 1), new(-1, 1),
        };
        float minimumSquared = float.PositiveInfinity;
        for (int i = 0; i < corners.Length; i++)
        {
            float distanceSquared = Distance.ClosestPointsSegmentSegment(
                corners[i], corners[(i + 1) % corners.Length],
                Segment.A, Segment.B, out _, out _);
            minimumSquared = MathF.Min(minimumSquared, distanceSquared);
        }

        Assert.Equal(1f, MathF.Sqrt(minimumSquared), 1f / 256f);
    }

    // box2d/test/test_distance.c: ShapeCastTest
    [Fact]
    public void ShapeCast_Box2DFixture_HitsAtHalfTranslation()
    {
        SweepHit output = Sweep.MovingShapeVsShape(
            Segment, new Vec2(-2, 0), Box);

        Assert.True(output.Hit);
        Assert.Equal(0.5f, output.Time, 0.005f);
    }

    // box2d/test/test_distance.c: TimeOfImpactTest. ArcCollision exposes the
    // linear, fixed-orientation TOI through the same MovingShapeVsShape API.
    [Fact]
    public void TimeOfImpact_Box2DFixture_HitsAtHalfTranslation()
    {
        SweepHit output = Sweep.MovingShapeVsShape(
            Segment, new Vec2(-2, 0), Box);

        Assert.True(output.Hit);
        Assert.Equal(0.5f, output.Time, 0.005f);
    }
}

/// <summary>
/// Ports of the supported collision tests in box2d/test/test_collision.c.
/// </summary>
public class Box2DCollisionParityTests
{
    // box2d/test/test_collision.c: AABBTest
    [Fact]
    public void Aabb_Box2DFixture_ValidatesOverlapAndContainment()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Aabb.FromMinMax(new Vec2(-1, -1), new Vec2(-2, -2)));

        var a = Aabb.FromMinMax(new Vec2(-1, -1), new Vec2(1, 1));
        var b = Aabb.FromMinMax(new Vec2(2, 2), new Vec2(4, 4));
        Assert.False(a.Overlaps(b));
        Assert.False(new BpBounds(a).Contains(new BpBounds(b)));
    }

    // Cases 7 and 12 of Box2D's AABBRayCastTest are initial-overlap cases.
    // ArcCollision's documented sweep contract reports those at time zero,
    // rather than filtering them as Box2D's low-level AABB ray helper does.
    [Theory]
    [InlineData(0, 0, 2, 0)]
    [InlineData(0, 0, 0, 0)]
    public void AabbRayCast_Box2DInitialOverlapFixtures_FollowSweepContract(
        float ox, float oy, float dx, float dy)
    {
        SweepHit output = Sweep.RayVsAabb(
            new Vec2(ox, oy), new Vec2(dx, dy),
            new Aabb(Vec2.Zero, new Vec2(1, 1)));

        Assert.True(output.Hit);
        Assert.Equal(0, output.Time);
    }

    // box2d/test/test_collision.c: LargeWorldManifoldTest. ArcCollision has a
    // single contact hint rather than Box2D's two-point local manifold, so the
    // equivalent assertions are collision, normal and 0.1 penetration depth.
    [Fact]
    public void LargeWorldManifold_Box2DFixture_IsTranslationInvariant()
    {
        var aOrigin = new Aabb(Vec2.Zero, new Vec2(0.5f, 0.5f));
        var bOrigin = new Aabb(new Vec2(0.9f, 0), new Vec2(0.5f, 0.5f));
        Manifold origin = Collide.AabbVsAabb(aOrigin, bOrigin);

        Assert.True(origin.Colliding);
        Assert.Equal(0.1f, origin.Depth, 0.01f);
        Assert.Equal(1f, origin.Normal.X);
        Assert.Equal(0f, origin.Normal.Y);

        // At +/-1.9M a float world coordinate has a 1/8 ULP. Use the closest
        // exactly representable separation for the actual large-world
        // invariance check, while retaining Box2D's original 0.9 fixture above.
        var gridA = new Aabb(Vec2.Zero, new Vec2(0.5f, 0.5f));
        var gridB = new Aabb(new Vec2(0.875f, 0), new Vec2(0.5f, 0.5f));
        Manifold gridOrigin = Collide.AabbVsAabb(gridA, gridB);
        Vec2 offset = Box2DLargeWorldParityTests.FarBase;
        Manifold large = Collide.AabbVsAabb(
            gridA.Moved(offset), gridB.Moved(offset));

        Assert.Equal(gridOrigin.Colliding, large.Colliding);
        Assert.Equal(gridOrigin.Depth, large.Depth);
        Assert.Equal(gridOrigin.Normal.X, large.Normal.X);
        Assert.Equal(gridOrigin.Normal.Y, large.Normal.Y);
        Assert.Equal(gridOrigin.Contact.X, large.Contact.X - offset.X, 0.125f);
        Assert.Equal(gridOrigin.Contact.Y, large.Contact.Y - offset.Y, 0.125f);
    }

    // box2d/test/test_collision.c: LargeWorldAABBTest. Box2D obtains the 0.6
    // extent from a 0.5 box plus 0.1 polygon radius. ArcCollision stores the
    // already-expanded shape bounds, then applies the fat margin explicitly.
    [Fact]
    public void LargeWorldAabb_Box2DFixture_PreservesTightAndFatExtents()
    {
        AssertBoundsAtBase(Vec2.Zero);
        AssertBoundsAtBase(Box2DLargeWorldParityTests.FarBase);
    }

    private static void AssertBoundsAtBase(Vec2 basePosition)
    {
        var roundedBoxBounds = new Aabb(basePosition, new Vec2(0.6f, 0.6f));
        var tight = new BpBounds(roundedBoxBounds);
        long centerX = Fx.From(basePosition.X);
        long centerY = Fx.From(basePosition.Y);
        long extent = Fx.From(0.6f);

        Assert.True(tight.MinX <= centerX - extent);
        Assert.True(tight.MinY <= centerY - extent);
        Assert.True(tight.MaxX >= centerX + extent);
        Assert.True(tight.MaxY >= centerY + extent);

        long extra = Fx.From(0.05f);
        BpBounds fat = tight.Expanded(extra);
        Assert.True(fat.MinX <= centerX - extent - extra);
        Assert.True(fat.MinY <= centerY - extent - extra);
        Assert.True(fat.MaxX >= centerX + extent + extra);
        Assert.True(fat.MaxY >= centerY + extent + extra);
    }
}

/// <summary>
/// Geometry-query equivalents of box2d/test/test_large_world.c. The fixed-point
/// API supports +/-1.95M units. The far base is deliberately near that limit;
/// local positions that are added to it use the 1/8 grid, matching the float
/// ULP in this range while still exercising the fixed-point large-world core.
/// </summary>
public class Box2DLargeWorldParityTests
{
    internal static readonly Vec2 FarBase = new(1_898_000, -1_898_000);

    // box2d/test/test_large_world.c: LargeWorldBulletTest
    [Fact]
    public void LargeWorldBullet_Box2DFixture_DoesNotTunnelAtEitherBase()
    {
        SweepHit origin = RunBullet(Vec2.Zero);
        SweepHit large = RunBullet(FarBase);

        Assert.True(origin.Hit);
        Assert.True(large.Hit);
        Assert.InRange(origin.Point.X, 0f, 0.5f);
        Assert.InRange(large.Point.X - FarBase.X, 0f, 0.5f);
        Assert.Equal(origin.Time, large.Time);
        Assert.Equal(origin.Normal.X, large.Normal.X);
        Assert.Equal(origin.Normal.Y, large.Normal.Y);
    }

    // box2d/test/test_large_world.c: LargeWorldRayCastTest
    [Fact]
    public void LargeWorldRayCast_Box2DFixture_HitsSameRelativeFace()
    {
        SweepHit origin = RunRayCast(Vec2.Zero);
        SweepHit large = RunRayCast(FarBase);

        Assert.True(origin.Hit);
        Assert.True(large.Hit);
        Assert.Equal(-0.5f, origin.Point.X, 1f / 256f);
        Assert.Equal(0f, origin.Point.Y, 1f / 256f);
        Assert.Equal(origin.Time, large.Time);
        Assert.Equal(origin.Point.X, large.Point.X - FarBase.X, 1f / 256f);
        Assert.Equal(origin.Point.Y, large.Point.Y - FarBase.Y, 1f / 256f);
    }

    // box2d/test/test_large_world.c: LargeWorldOriginQueryTest
    [Fact]
    public void LargeWorldOriginQueries_Box2DFixture_MatchAtEitherBase()
    {
        QueryResult origin = RunOriginQueries(Vec2.Zero);
        QueryResult large = RunOriginQueries(FarBase);

        Assert.Equal(1, origin.OverlapCount);
        Assert.True(origin.Cast.Hit);
        Assert.True(origin.MoverCast.Hit);
        Assert.True(origin.MoverColliding);
        Assert.True(origin.InsidePoint);
        Assert.True(origin.ShapeRay.Hit);
        Assert.Equal(-0.5f, origin.Cast.Point.X, 1f / 256f);
        Assert.Equal(-0.5f, origin.ShapeRay.Point.X, 1f / 256f);

        Assert.Equal(origin.OverlapCount, large.OverlapCount);
        Assert.Equal(origin.MoverColliding, large.MoverColliding);
        Assert.Equal(origin.InsidePoint, large.InsidePoint);
        Assert.Equal(origin.Cast.Time, large.Cast.Time);
        Assert.Equal(origin.MoverCast.Time, large.MoverCast.Time);
        Assert.Equal(origin.ShapeRay.Time, large.ShapeRay.Time);
        AssertRelativePoint(origin.Cast.Point, large.Cast.Point);
        AssertRelativePoint(origin.MoverCast.Point, large.MoverCast.Point);
        AssertRelativePoint(origin.ShapeRay.Point, large.ShapeRay.Point);
    }

    private static SweepHit RunBullet(Vec2 basePosition)
    {
        var wall = new Aabb(basePosition, new Vec2(0.05f, 5));
        var bullet = new Circle(basePosition + new Vec2(10, 0), 0.1f);
        return Sweep.MovingCircleVsAabb(bullet, new Vec2(-20, 0), wall);
    }

    private static SweepHit RunRayCast(Vec2 basePosition) =>
        Sweep.RayVsAabb(basePosition + new Vec2(-5, 0), new Vec2(10, 0),
            new Aabb(basePosition, new Vec2(0.5f, 0.5f)));

    private static QueryResult RunOriginQueries(Vec2 basePosition)
    {
        var box = new Aabb(basePosition, new Vec2(0.5f, 0.5f));
        using var world = new ArcWorld();
        world.AddStatic(1, box);
        world.BuildStatic();

        Shape overlap = new Circle(basePosition, 0.1f);
        var handles = new List<ArcHandle>();
        world.Query(overlap, handles);
        int overlapCount = handles.Count(handle =>
            world.TryComputeContact(overlap, handle, out _));

        var castCircle = new Circle(basePosition + new Vec2(-5, 0), 0.1f);
        SweepHit cast = Sweep.MovingCircleVsAabb(castCircle, new Vec2(10, 0), box);

        var mover = new Capsule(
            basePosition + new Vec2(-5, -0.25f),
            basePosition + new Vec2(-5, 0.25f), 0.3f);
        SweepHit moverCast = Sweep.MovingShapeVsShape(mover, new Vec2(10, 0), box);

        var touching = new Capsule(
            basePosition + new Vec2(-0.875f, -0.25f),
            basePosition + new Vec2(-0.875f, 0.25f), 0.5f);
        bool moverColliding = Collide.ShapeVsShape(touching, box).Colliding;
        bool insidePoint = Collide.PointInAabb(basePosition, box);
        SweepHit shapeRay = RunRayCast(basePosition);

        return new QueryResult(overlapCount, cast, moverCast,
            moverColliding, insidePoint, shapeRay);
    }

    private static void AssertRelativePoint(Vec2 origin, Vec2 large)
    {
        Assert.Equal(origin.X, large.X - FarBase.X, 0.125f);
        Assert.Equal(origin.Y, large.Y - FarBase.Y, 0.125f);
    }

    private readonly record struct QueryResult(
        int OverlapCount,
        SweepHit Cast,
        SweepHit MoverCast,
        bool MoverColliding,
        bool InsidePoint,
        SweepHit ShapeRay);
}

/// <summary>
/// ArcCollision equivalent of box2d/test/test_determinism.c. Box2D hashes a
/// falling-hinge simulation across worker counts. ArcCollision has no dynamics
/// or scheduler, so this hashes all supported deterministic outputs for a fixed
/// scene across repeated runs and opposite insertion orders.
/// </summary>
public class Box2DDeterminismParityTests
{
    [Fact]
    public void MultirunAndCrossPlatform_Box2DPattern_HasLockedHash()
    {
        int[] forward = { 10, 20, 30, 40, 50, 60 };
        int[] reverse = forward.Reverse().ToArray();
        uint? expected = null;

        for (int run = 0; run < 3; run++)
        {
            uint forwardHash = RunScenario(forward);
            uint reverseHash = RunScenario(reverse);
            Assert.Equal(forwardHash, reverseHash);
            expected ??= forwardHash;
            Assert.Equal(expected.Value, forwardHash);
        }

        // Cross-platform regression value for this fixed-point scene.
        Assert.Equal(2644972881u, expected!.Value);
    }

    private static uint RunScenario(int[] insertionOrder)
    {
        var shapes = new Dictionary<int, Shape>
        {
            [10] = new Circle(new Vec2(-2, 0), 1.25f),
            [20] = new Aabb(new Vec2(-0.5f, 0), new Vec2(1, 1)),
            [30] = new Capsule(new Vec2(1, -1), new Vec2(1, 1), 0.75f),
            [40] = new Obb(new Vec2(2.25f, 0), new Vec2(1, 0.5f), 0.25f),
            [50] = new Circle(new Vec2(10, 10), 0.5f),
            [60] = new Aabb(new Vec2(0, 3), new Vec2(4, 0.25f)),
        };

        using var world = new ArcWorld(1);
        foreach (int entityId in insertionOrder)
        {
            if (entityId is 40 or 60) world.AddStatic(entityId, shapes[entityId]);
            else world.Add(entityId, shapes[entityId]);
        }
        world.BuildStatic();

        uint hash = 2166136261u;
        var query = new List<ArcHandle>();
        world.Query(new Aabb(Vec2.Zero, new Vec2(20, 20)), query);
        foreach (ArcHandle handle in query) hash = Add(hash, unchecked((uint)handle.EntityId));

        var pairs = new List<CandidatePair>();
        world.ComputePairs(pairs);
        foreach (CandidatePair pair in pairs)
        {
            hash = Add(hash, unchecked((uint)pair.A.EntityId));
            hash = Add(hash, unchecked((uint)pair.B.EntityId));
            if (!world.TryComputeContact(pair, out ContactPair contact)) continue;
            hash = Add(hash, BitConverter.SingleToUInt32Bits(contact.Manifold.Depth));
            hash = Add(hash, BitConverter.SingleToUInt32Bits(contact.Manifold.Normal.X));
            hash = Add(hash, BitConverter.SingleToUInt32Bits(contact.Manifold.Normal.Y));
        }

        SweepHit sweep = Sweep.MovingShapeVsShape(
            new Circle(new Vec2(-5, 0), 0.5f), new Vec2(10, 0),
            new Aabb(Vec2.Zero, new Vec2(1, 1)));
        hash = Add(hash, BitConverter.SingleToUInt32Bits(sweep.Time));
        hash = Add(hash, BitConverter.SingleToUInt32Bits(sweep.Normal.X));
        hash = Add(hash, BitConverter.SingleToUInt32Bits(sweep.Normal.Y));
        return hash;
    }

    private static uint Add(uint hash, uint value) =>
        unchecked((hash ^ value) * 16777619u);
}
