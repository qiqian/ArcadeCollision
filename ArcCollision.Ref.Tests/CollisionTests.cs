using ArcCollision;
using System.Reflection;
using Xunit;

namespace ArcCollision.Tests;

public class DiscreteTests
{
    private const float Eps = 1e-3f;

    [Fact]
    public void CircleVsCircle_Overlap_ReportsDepthAndNormal()
    {
        var a = new Circle(new Vec2(0, 0), 1f);
        var b = new Circle(new Vec2(1.5f, 0), 1f);
        var m = Collide.CircleVsCircle(a, b);

        Assert.True(m.Colliding);
        Assert.Equal(0.5f, m.Depth, Eps);
        Assert.Equal(1f, m.Normal.X, Eps);
        Assert.Equal(0f, m.Normal.Y, Eps);
    }

    [Fact]
    public void CircleVsCircle_Separated_NoCollision()
    {
        var a = new Circle(new Vec2(0, 0), 1f);
        var b = new Circle(new Vec2(3f, 0), 1f);
        Assert.False(Collide.CircleVsCircle(a, b).Colliding);
    }

    [Fact]
    public void CircleVsCircle_Concentric_UsesStableAxis()
    {
        var a = new Circle(new Vec2(5, 5), 2f);
        var b = new Circle(new Vec2(5, 5), 1f);
        var m = Collide.CircleVsCircle(a, b);
        Assert.True(m.Colliding);
        Assert.Equal(1f, m.Normal.Length, Eps); // normal is unit even when concentric
    }

    [Fact]
    public void AabbVsAabb_ResolvesLeastPenetrationAxis()
    {
        var a = new Aabb(new Vec2(0, 0), new Vec2(1, 1));
        var b = new Aabb(new Vec2(1.5f, 0.2f), new Vec2(1, 1));
        var m = Collide.AabbVsAabb(a, b);

        Assert.True(m.Colliding);
        Assert.Equal(0.5f, m.Depth, Eps);   // x overlap 0.5 < y overlap 1.8
        Assert.Equal(1f, m.Normal.X, Eps);
    }

    [Fact]
    public void CircleVsAabb_CenterInside_PushesOutNearestFace()
    {
        var box = new Aabb(new Vec2(0, 0), new Vec2(2, 2));
        var c = new Circle(new Vec2(1.5f, 0f), 0.5f); // inside, closest to +x face
        var m = Collide.CircleVsAabb(c, box);

        Assert.True(m.Colliding);
        // normal is A->B (towards box centre) => -x ...
        Assert.Equal(-1f, m.Normal.X, Eps);
        // ... and separating the circle ejects it out the +x face.
        Assert.True(m.SeparationForA.X > 0f);
    }

    [Fact]
    public void CircleVsAabb_Corner_NormalIsDiagonal()
    {
        var box = new Aabb(new Vec2(0, 0), new Vec2(1, 1));
        var c = new Circle(new Vec2(1.4f, 1.4f), 0.6f); // reaches the (1,1) corner
        var m = Collide.CircleVsAabb(c, box);

        Assert.True(m.Colliding);
        Assert.Equal(m.Normal.X, m.Normal.Y, Eps); // symmetric diagonal
        Assert.Equal(1f, m.Normal.Length, Eps);
    }

    [Fact]
    public void SeparationVectors_ActuallySeparate()
    {
        var a = new Circle(new Vec2(0, 0), 1f);
        var b = new Circle(new Vec2(1.2f, 0), 1f);
        var m = Collide.CircleVsCircle(a, b);

        var a2 = new Circle(a.Center + m.SeparationForA, a.Radius);
        var after = Collide.CircleVsCircle(a2, b);
        Assert.True(after.Depth <= Eps);
    }

    [Fact]
    public void ClosestSegments_LargeCrossing_DoesNotOverflowDegreeFourRatio()
    {
        float distance = Distance.ClosestPointsSegmentSegment(
            new Vec2(-900_000, -900_000), new Vec2(900_000, 900_000),
            new Vec2(-900_000, 900_000), new Vec2(900_000, -900_000),
            out Vec2 first, out Vec2 second);

        Assert.Equal(0f, distance, 1f / 256f);
        Assert.Equal(0f, first.X, 1f / 256f);
        Assert.Equal(0f, first.Y, 1f / 256f);
        Assert.Equal(first.X, second.X, 1f / 256f);
        Assert.Equal(first.Y, second.Y, 1f / 256f);
    }
}

public class SweepTests
{
    private const float Eps = 1e-3f;

    [Fact]
    public void RayVsCircle_DirectHit()
    {
        var hit = Sweep.RayVsCircle(new Vec2(-5, 0), new Vec2(10, 0), new Circle(new Vec2(0, 0), 1f));
        Assert.True(hit.Hit);
        Assert.Equal(0.4f, hit.Time, Eps); // reaches x=-1 at t=4/10
        Assert.Equal(-1f, hit.Normal.X, Eps);
    }

    [Fact]
    public void RayVsCircle_Misses()
    {
        var hit = Sweep.RayVsCircle(new Vec2(-5, 5), new Vec2(10, 0), new Circle(new Vec2(0, 0), 1f));
        Assert.False(hit.Hit);
    }

    [Fact]
    public void RayVsAabb_HitsNearFace()
    {
        var hit = Sweep.RayVsAabb(new Vec2(-5, 0), new Vec2(10, 0), new Aabb(new Vec2(0, 0), new Vec2(1, 1)));
        Assert.True(hit.Hit);
        Assert.Equal(0.4f, hit.Time, Eps);
        Assert.Equal(-1f, hit.Normal.X, Eps);
    }

    [Fact]
    public void MovingCircleVsAabb_NoTunnelForFastMover()
    {
        var mover = new Circle(new Vec2(-500, 0), 0.5f);
        var hit = Sweep.MovingCircleVsAabb(mover, new Vec2(1000, 0), new Aabb(new Vec2(0, 0), new Vec2(1, 1)));
        Assert.True(hit.Hit);
        Assert.Equal(-1f, hit.Normal.X, Eps);
    }

    [Fact]
    public void MovingCircleVsAabb_ReportsContactOnBoxSurface()
    {
        var mover = new Circle(new Vec2(-5, 0), 0.5f);
        var box = new Aabb(new Vec2(0, 0), new Vec2(1, 1));
        var hit = Sweep.MovingCircleVsAabb(mover, new Vec2(10, 0), box);

        Assert.True(hit.Hit);
        Assert.Equal(-1f, hit.Point.X, 1f / 256f);
        Assert.Equal(0f, hit.Point.Y, 1f / 256f);
    }

    [Fact]
    public void RayVsCircle_LargeCoordinates_DoNotOverflowLongDiscriminant()
    {
        var hit = Sweep.RayVsCircle(
            new Vec2(-900_000f, 0), new Vec2(1_800_000f, 0),
            new Circle(Vec2.Zero, 100f));

        Assert.True(hit.Hit);
        Assert.Equal((900_000f - 100f) / 1_800_000f, hit.Time, 2f / 65536f);
        Assert.Equal(-1f, hit.Normal.X, 1f / 256f);
    }

    [Fact]
    public void MovingCircleVsCircle_ContactTime()
    {
        var mover = new Circle(new Vec2(-5, 0), 0.5f);
        var hit = Sweep.MovingCircleVsCircle(mover, new Vec2(10, 0), new Circle(new Vec2(0, 0), 1f));
        Assert.True(hit.Hit);
        Assert.Equal(0.35f, hit.Time, Eps); // contact when centers 1.5 apart
    }
}

public class BroadphaseTests
{
    [Fact]
    public void SpatialHash_QueryFindsOverlappingEntities()
    {
        var hash = new SpatialHash(10f);
        hash.Insert(1, new Aabb(new Vec2(0, 0), new Vec2(2, 2)));
        hash.Insert(2, new Aabb(new Vec2(50, 50), new Vec2(2, 2)));

        var results = hash.Query(new Aabb(new Vec2(0, 0), new Vec2(3, 3)));
        Assert.Contains(1, results);
        Assert.DoesNotContain(2, results);
    }

    [Fact]
    public void SpatialHash_PairsAreUniqueAndOverlapping()
    {
        var hash = new SpatialHash(10f);
        hash.Insert(1, new Aabb(new Vec2(0, 0), new Vec2(3, 3)));
        hash.Insert(2, new Aabb(new Vec2(2, 0), new Vec2(3, 3))); // overlaps 1
        hash.Insert(3, new Aabb(new Vec2(500, 500), new Vec2(3, 3)));

        var pairs = new List<(int, int)>(hash.Pairs());
        Assert.Single(pairs);
        Assert.Equal((1, 2), pairs[0]);
    }

    [Fact]
    public void SpatialHash_NegativeFractionalCellsUseFloorDivision()
    {
        var hash = new SpatialHash(1f / 256f);
        var tiny = new Aabb(new Vec2(-1f / 256f, -1f / 256f), Vec2.Zero);
        hash.Insert(7, tiny);

        Assert.Contains(7, hash.Query(tiny));
        hash.Remove(7);
        Assert.Empty(hash.Query(tiny));
    }

    [Fact]
    public void SpatialHash_InternalStateContainsNoFloatingPointFields()
    {
        FieldInfo[] fields = typeof(SpatialHash).GetFields(BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.DoesNotContain(fields, f => f.FieldType == typeof(float) || f.FieldType == typeof(double));

        FieldInfo cells = Assert.Single(fields, f => f.Name == "_cells");
        Type cellType = cells.FieldType.GetGenericArguments()[0];
        Assert.All(cellType.GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic),
            f => Assert.Equal(typeof(long), f.FieldType));
    }

    [Fact]
    public void FixedBoundaryRejectsNonFiniteAndUnsafeCoordinates()
    {
        var unit = new Circle(Vec2.Zero, 1f);
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Collide.PointInCircle(new Vec2(float.NaN, 0), unit));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Collide.PointInCircle(new Vec2(2_000_000f, 0), unit));
    }
}
