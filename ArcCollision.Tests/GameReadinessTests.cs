using Xunit;

namespace ArcCollision.Tests;

[Collection("ArcWorld lifecycle")]
public class GameReadinessTests
{
    [Fact]
    public void UnrelatedMutationDoesNotInvalidateCandidate()
    {
        using var world = new ArcWorld();
        world.Add(1, new Circle(Vec2.Zero, 2));
        world.Add(2, new Circle(new Vec2(1, 0), 2));
        ArcHandle unrelated = world.Add(3, new Circle(new Vec2(100, 0), 1));
        var pairs = new List<CandidatePair>();
        world.ComputePairs(pairs);
        CandidatePair pair = Assert.Single(pairs);

        world.Update(unrelated, new Circle(new Vec2(200, 0), 1));

        Assert.True(world.TryComputeContact(pair, out _));
    }

    [Fact]
    public void DisabledColliderRetainsHandleAndLeavesBroadphase()
    {
        using var world = new ArcWorld();
        ArcHandle first = world.Add(1, new Circle(Vec2.Zero, 2));
        world.Add(2, new Circle(new Vec2(1, 0), 2));
        var pairs = new List<CandidatePair>();
        world.ComputePairs(pairs);
        CandidatePair oldPair = Assert.Single(pairs);

        world.SetEnabled(first, false);

        Assert.True(world.IsValid(first));
        Assert.False(world.IsEnabled(first));
        Assert.Equal(2, world.Count);
        Assert.Equal(1, world.EnabledCount);
        Assert.False(world.TryComputeContact(oldPair, out _));
        world.ComputePairs(pairs);
        Assert.Empty(pairs);

        world.Update(first, new Circle(new Vec2(1, 0), 2));
        world.SetEnabled(first, true);

        Assert.Equal(2, world.EnabledCount);
        world.ComputePairs(pairs);
        Assert.Single(pairs);
    }

    [Fact]
    public void ColliderCanBeCreatedDisabled()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.Add(1, new Circle(Vec2.Zero, 1),
            CollisionFilter.Default, enabled: false);
        var hits = new List<ArcHandle>();

        world.Query(new Circle(Vec2.Zero, 2), hits);

        Assert.True(world.IsValid(handle));
        Assert.False(world.IsEnabled(handle));
        Assert.Empty(hits);
    }

    [Fact]
    public void ShapeCastReturnsClosestAndCastAllIsDeterministic()
    {
        using var world = new ArcWorld();
        world.AddStatic(20, new Aabb(new Vec2(8, 0), new Vec2(1, 1)));
        world.AddStatic(10, new Aabb(new Vec2(4, 0), new Vec2(1, 1)));
        world.BuildStatic();
        Shape mover = new Circle(Vec2.Zero, 0.5f);

        Assert.True(world.ShapeCast(mover, new Vec2(10, 0), out WorldCastHit closest));
        Assert.Equal(10, closest.Handle.EntityId);
        Assert.InRange(closest.Hit.Time, 0.24f, 0.26f);

        var hits = new List<WorldCastHit>();
        world.ShapeCastAll(mover, new Vec2(10, 0), hits);
        Assert.Equal(new[] { 10, 20 }, hits.Select(h => h.Handle.EntityId));
        Assert.True(hits[0].Hit.Time < hits[1].Hit.Time);
    }

    [Fact]
    public void ShapeCastHonoursMutualFiltersAndDisabledState()
    {
        const uint attack = 1u << 1;
        const uint hurt = 1u << 2;
        const uint wall = 1u << 3;
        using var world = new ArcWorld();
        ArcHandle hurtHandle = world.AddStatic(1,
            new Circle(new Vec2(4, 0), 1), new CollisionFilter(hurt, attack));
        world.AddStatic(2, new Circle(new Vec2(2, 0), 1),
            new CollisionFilter(wall, CollisionCategories.All));
        world.BuildStatic();
        Shape mover = new Circle(Vec2.Zero, 0.5f);

        Assert.True(world.ShapeCast(mover, new Vec2(10, 0),
            new CollisionFilter(attack, hurt), out WorldCastHit hit));
        Assert.Equal(1, hit.Handle.EntityId);

        world.SetEnabled(hurtHandle, false);
        Assert.False(world.ShapeCast(mover, new Vec2(10, 0),
            new CollisionFilter(attack, hurt), out _));
    }

    [Fact]
    public void RayCastUsesPointSweepAgainstWorldShapes()
    {
        using var world = new ArcWorld();
        world.AddStatic(7, new Aabb(new Vec2(5, 0), new Vec2(1, 1)));
        world.BuildStatic();

        Assert.True(world.RayCast(Vec2.Zero, new Vec2(10, 0), out WorldCastHit hit));
        Assert.Equal(7, hit.Handle.EntityId);
        Assert.InRange(hit.Hit.Time, 0.39f, 0.41f);
    }

    [Fact]
    public void ShiftOriginRetainsHandlesFiltersAndRelativeContacts()
    {
        using var world = new ArcWorld();
        var filter = new CollisionFilter(2, 2);
        ArcHandle dynamic = world.Add(1, new Circle(new Vec2(1001, 500), 2), filter);
        ArcHandle stationary = world.AddStatic(
            2, new Circle(new Vec2(1002, 500), 2), filter);
        var pairs = new List<CandidatePair>();

        world.ShiftOrigin(new Vec2(1000, 500));
        world.ComputePairs(pairs);

        Assert.True(world.IsValid(dynamic));
        Assert.True(world.IsValid(stationary));
        Assert.Equal(filter, world.GetFilter(dynamic));
        Assert.Equal(new Vec2(1, 0), world.GetShape(dynamic).Bounds.Center);
        Assert.True(world.TryComputeContact(Assert.Single(pairs), out _));
    }

    [Fact]
    public void ShapeTryGetAndWorldTryGetApisAreNonThrowingForWrongOrStaleValues()
    {
        Shape shape = new Circle(Vec2.Zero, 1);
        Assert.True(shape.TryGetCircle(out Circle circle));
        Assert.Equal(1, circle.Radius);
        Assert.False(shape.TryGetAabb(out _));

        using var world = new ArcWorld();
        ArcHandle handle = world.Add(42, shape);
        Assert.True(world.TryGetShape(handle, out Shape stored));
        Assert.Equal(ShapeKind.Circle, stored.Kind);
        Assert.True(world.TryGetFilter(handle, out CollisionFilter filter));
        Assert.Equal(CollisionFilter.Default, filter);
        Assert.Equal(42, world.GetEntityId(handle));

        world.Remove(handle);
        Assert.False(world.TryGetShape(handle, out _));
        Assert.False(world.TryGetFilter(handle, out _));
    }

    [Fact]
    public void CollisionLimitsExposeFixedPointContract()
    {
        Assert.Equal(1f / 256f, CollisionLimits.GridSize);
        Assert.Equal(1_953_125f, CollisionLimits.MaxCoordinate);
    }

    [Fact]
    public void PreallocatedDynamicFrameLoopDoesNotAllocateAfterWarmup()
    {
        using var world = new ArcWorld(new ArcWorldOptions(
            fatMargin: 4,
            initialColliderCapacity: 32,
            initialPairCapacity: 256));
        var handles = new ArcHandle[32];
        for (int i = 0; i < handles.Length; i++)
            handles[i] = world.Add(i,
                new Circle(new Vec2(i * 1.5f, 0), 1));
        var pairs = new List<CandidatePair>(256);
        var query = new List<ArcHandle>(32);

        RunFrame(0);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int frame = 1; frame <= 20; frame++) RunFrame(frame);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(0, allocated);

        void RunFrame(int frame)
        {
            for (int i = 0; i < handles.Length; i++)
                world.Update(handles[i],
                    new Circle(new Vec2(i * 1.5f, frame & 1), 1));
            world.ComputePairs(pairs);
            world.Query(new Aabb(new Vec2(24, 0), new Vec2(30, 4)), query);
        }
    }

    [Fact]
    public void StaticCapacitySurvivesClearAndRebuildWithoutAllocating()
    {
        using var world = new ArcWorld(new ArcWorldOptions(
            initialColliderCapacity: 16,
            initialPairCapacity: 16));

        Rebuild();
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 10; i++) Rebuild();
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(0, allocated);

        void Rebuild()
        {
            world.Clear();
            for (int i = 0; i < 16; i++)
                world.AddStatic(i,
                    new Aabb(new Vec2(i * 4, 0), new Vec2(1, 1)));
            world.BuildStatic();
        }
    }

    [Fact]
    public void PolygonShapeMovedSharesGeometryWithoutAllocating()
    {
        var geometry = new Polygon(
            new Vec2(-1, -1), new Vec2(1, -1),
            new Vec2(1, 1), new Vec2(-1, 1));
        Shape shape = geometry;
        _ = shape.Moved(Vec2.UnitX);
        long before = GC.GetAllocatedBytesForCurrentThread();
        float checksum = 0;

        for (int i = 0; i < 10_000; i++)
        {
            Shape moved = shape.Moved(new Vec2(i, 2));
            checksum += moved.PolygonTranslation.X;
        }

        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.Equal(0, allocated);
        Assert.True(checksum > 0);
        Shape translated = shape.Moved(new Vec2(10, 20));
        Assert.True(translated.TryGetPolygon(
            out Polygon? shared, out Vec2 translation, out _));
        Assert.Same(geometry, shared);
        Assert.Equal(new Vec2(10, 20), translation);
    }

    [Fact]
    public void PolygonInstanceRotationMatchesEquivalentWorldGeometry()
    {
        var geometry = new Polygon(
            new Vec2(0, 0), new Vec2(2, 0), new Vec2(0, 1));
        Shape instance = new Shape(
            geometry, new Vec2(10, 20), new Angle32(1u << 30));
        Shape baked = new Polygon(
            new Vec2(10, 20), new Vec2(10, 22), new Vec2(9, 20));
        Shape probe = new Circle(new Vec2(9.75f, 20.75f), 0.25f);

        Assert.Equal(baked.Bounds.Min.X, instance.Bounds.Min.X, 3);
        Assert.Equal(baked.Bounds.Max.Y, instance.Bounds.Max.Y, 3);
        Assert.Equal(
            Collide.ShapeVsShape(instance, probe).Colliding,
            Collide.ShapeVsShape(baked, probe).Colliding);

        SweepHit transformedHit = Sweep.MovingShapeVsShape(
            new Circle(new Vec2(7, 20.5f), 0.25f), new Vec2(5, 0), instance);
        SweepHit bakedHit = Sweep.MovingShapeVsShape(
            new Circle(new Vec2(7, 20.5f), 0.25f), new Vec2(5, 0), baked);
        Assert.Equal(bakedHit.Hit, transformedHit.Hit);
        Assert.Equal(bakedHit.Time, transformedHit.Time, 3);
    }
}
