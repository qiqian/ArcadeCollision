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

    [Fact]
    public void MovingShapeVsShape_SupportsEveryOrderedCombination()
    {
        Shape[] shapes =
        {
            new Circle(Vec2.Zero, 1f),
            new Aabb(Vec2.Zero, new Vec2(1.5f, 1f)),
            new Capsule(new Vec2(-1, 0), new Vec2(1, 0), .5f),
            new Obb(Vec2.Zero, new Vec2(1.5f, .75f), .4f),
            new Polygon(new Vec2(-1.5f, -1), new Vec2(1.5f, -1),
                new Vec2(1, 1), new Vec2(-1, 1)),
        };

        foreach (Shape moverSource in shapes)
        foreach (Shape target in shapes)
        {
            Shape mover = moverSource.Moved(new Vec2(-10, 0));
            SweepHit hit = Sweep.MovingShapeVsShape(mover, new Vec2(20, 0), target);
            Assert.True(hit.Hit, $"Expected {mover.Kind} vs {target.Kind} to hit.");
            Assert.InRange(hit.Time, 0f, 1f);

            SweepHit reverse = Sweep.MovingShapeVsShape(
                target, new Vec2(-20, 0), mover);
            Assert.True(reverse.Hit);
            Assert.Equal(hit.Time, reverse.Time, 2f / 65536f);
            Assert.True(hit.Normal.Dot(reverse.Normal) <= -.9f,
                $"Normal mismatch for {mover.Kind} vs {target.Kind}: "
                + $"{hit.Normal} / {reverse.Normal}");

            SweepHit miss = Sweep.MovingShapeVsShape(
                mover, new Vec2(20, 0), target.Moved(new Vec2(0, 100)));
            Assert.False(miss.Hit,
                $"Expected offset {mover.Kind} vs {target.Kind} to miss.");
        }
    }

    [Fact]
    public void MovingShapeVsShape_FastObbDoesNotTunnelThroughPolygon()
    {
        Shape mover = new Obb(new Vec2(-500, 0), new Vec2(2, 1), .35f);
        Shape target = new Polygon(
            new Vec2(-1, -20), new Vec2(1, -20),
            new Vec2(1, 20), new Vec2(-1, 20));

        SweepHit hit = Sweep.MovingShapeVsShape(mover, new Vec2(1000, 0), target);

        Assert.True(hit.Hit);
        Assert.Equal(-1f, hit.Normal.X, 2f / 256f);
    }

    [Fact]
    public void SpecializedRoundedAndBoxSweepsReportExpectedTimes()
    {
        var circle = new Circle(new Vec2(-5, 0), .5f);
        SweepHit capsule = Sweep.MovingCircleVsCapsule(circle, new Vec2(10, 0),
            new Capsule(new Vec2(0, -1), new Vec2(0, 1), .5f));
        SweepHit obb = Sweep.MovingCircleVsObb(circle, new Vec2(10, 0),
            new Obb(Vec2.Zero, new Vec2(1, 1)));
        SweepHit boxes = Sweep.MovingAabbVsAabb(
            new Aabb(new Vec2(-5, 0), new Vec2(.5f, .5f)), new Vec2(10, 0),
            new Aabb(Vec2.Zero, new Vec2(1, 1)));

        Assert.Equal(.4f, capsule.Time, 2f / 65536f);
        Assert.Equal(.35f, obb.Time, 2f / 65536f);
        Assert.Equal(.35f, boxes.Time, 2f / 65536f);
        Assert.Equal(SweepAlgorithm.RoundedSegment,
            Sweep.GetAlgorithm(circle, new Capsule(Vec2.Zero, Vec2.UnitY, 1)));
        Assert.Equal(SweepAlgorithm.ContinuousSat,
            Sweep.GetAlgorithm(new Obb(Vec2.Zero, new Vec2(1, 1)),
                new Polygon(new Vec2(-1, -1), new Vec2(1, -1), new Vec2(0, 1))));
    }
}

public class BroadphaseTests
{
    [Fact]
    public void ArcWorld_QueryFindsOverlappingEntities()
    {
        var world = new ArcWorld(10f);
        world.Add(1, new Aabb(new Vec2(0, 0), new Vec2(2, 2)));
        world.Add(2, new Aabb(new Vec2(50, 50), new Vec2(2, 2)));
        var results = new List<ArcHandle>();

        world.Query(new Aabb(new Vec2(0, 0), new Vec2(3, 3)), results);

        Assert.Contains(results, handle => handle.EntityId == 1);
        Assert.DoesNotContain(results, handle => handle.EntityId == 2);
    }

    [Fact]
    public void ArcWorld_PairsAreUniqueAndOverlapping()
    {
        var world = new ArcWorld(10f);
        world.Add(1, new Aabb(new Vec2(0, 0), new Vec2(3, 3)));
        world.Add(2, new Aabb(new Vec2(2, 0), new Vec2(3, 3)));
        world.Add(3, new Aabb(new Vec2(500, 500), new Vec2(3, 3)));
        var pairs = new List<CandidatePair>();

        world.ComputePairs(pairs);

        CandidatePair pair = Assert.Single(pairs);
        Assert.Equal(1, pair.A.EntityId);
        Assert.Equal(2, pair.B.EntityId);
        Assert.True(world.TryComputeContact(pair, out _));
    }

    [Fact]
    public void ArcWorld_QuerySupportsEveryShapeKind()
    {
        var world = new ArcWorld(8f);
        world.Add(1, new Circle(new Vec2(0, 0), 2));
        world.Add(2, new Capsule(new Vec2(-3, 1), new Vec2(3, 1), 1));
        world.Add(3, new Obb(new Vec2(1, -1), new Vec2(3, 1), 0.5f));
        world.Add(4, new Aabb(new Vec2(100, 100), new Vec2(2, 2)));
        var polygon = new Polygon(new Vec2(-2, -2), new Vec2(4, -1), new Vec2(0, 3));
        ArcHandle polygonHandle = world.Add(5, polygon);
        var results = new List<ArcHandle>();

        world.Query(new Circle(Vec2.Zero, 5), results);
        Assert.Contains(results, handle => handle.EntityId == 1);
        Assert.Contains(results, handle => handle.EntityId == 2);
        Assert.Contains(results, handle => handle.EntityId == 3);
        Assert.Contains(results, handle => handle.EntityId == 5);
        Assert.DoesNotContain(results, handle => handle.EntityId == 4);

        Polygon moved = polygon.Moved(new Vec2(200, 0));
        world.UpdateTransformDelta(
            polygonHandle, new Transform(new Vec2(200, 0)));
        world.Query(moved, results);
        Assert.Contains(results, handle => handle.EntityId == 5);
    }

    [Fact]
    public void ArcWorld_StaticPolygonUsesCachedDefensiveBounds()
    {
        Vec2[] vertices = { new(40, 40), new(60, 40), new(55, 65), new(48, 52) };
        var polygon = new Polygon(vertices);
        vertices[0] = new Vec2(-10_000, -10_000);
        var world = new ArcWorld(4f);
        world.AddStatic(77, polygon);
        world.BuildStatic();
        var results = new List<ArcHandle>();

        world.Query(polygon, results);
        Assert.Contains(results, handle => handle.EntityId == 77);
        world.Query(new Aabb(new Vec2(-10_000, -10_000), new Vec2(1, 1)), results);
        Assert.DoesNotContain(results, handle => handle.EntityId == 77);
        Assert.Throws<ArgumentException>(() =>
            new Polygon(new Vec2(0, 0), new Vec2(1, 1)));
    }

    [Fact]
    public void PolygonRejectsSelfIntersectionAndQuantizedZeroEdges()
    {
        var pentagram = new Vec2[5];
        for (int i = 0; i < pentagram.Length; i++)
        {
            int vertex = i * 2 % 5;
            double angle = Math.PI * 0.5 + vertex * Math.PI * 2.0 / 5.0;
            pentagram[i] = new Vec2((float)Math.Cos(angle) * 5,
                (float)Math.Sin(angle) * 5);
        }

        Assert.Throws<ArgumentException>(() => new Polygon(pentagram));
        Assert.Throws<ArgumentException>(() => new Polygon(
            new Vec2(0, 0), new Vec2(1f / 1024f, 0), new Vec2(1, 1), new Vec2(0, 1)));
    }

    [Fact]
    public void ArcWorld_HybridBroadphaseMatchesBruteForceAfterRandomMoves()
    {
        var world = new ArcWorld(12f);
        var handles = new Dictionary<int, ArcHandle>();
        var dynamic = new Dictionary<int, Aabb>();
        var stationary = new Dictionary<int, Aabb>();
        var random = new Random(12345);
        static Aabb RandomBox(Random random) => new(
            new Vec2(random.Next(-5_000, 5_001), random.Next(-5_000, 5_001)),
            new Vec2(random.Next(1, 80), random.Next(1, 80)));

        for (int id = 0; id < 100; id++)
        {
            Aabb box = RandomBox(random);
            dynamic.Add(id, box);
            handles.Add(id, world.Add(id, box));
        }
        for (int id = 1_000; id < 1_060; id++)
        {
            Aabb box = RandomBox(random);
            stationary.Add(id, box);
            world.AddStatic(id, box);
        }
        world.BuildStatic();

        var actualHandles = new List<ArcHandle>();
        for (int step = 0; step < 200; step++)
        {
            int id = random.Next(0, 100);
            Aabb randomPlacement = RandomBox(random);
            Aabb moved = new(randomPlacement.Center, dynamic[id].HalfExtents);
            dynamic[id] = moved;
            world.UpdateTransform(handles[id], new Transform(moved.Center));

            Aabb query = RandomBox(random).Expanded(150);
            world.Query(query, actualHandles);
            var actual = new HashSet<int>(actualHandles.Select(handle => handle.EntityId));
            var expected = new HashSet<int>();
            foreach (KeyValuePair<int, Aabb> item in dynamic)
                if (item.Value.Overlaps(query)) expected.Add(item.Key);
            foreach (KeyValuePair<int, Aabb> item in stationary)
                if (item.Value.Overlaps(query)) expected.Add(item.Key);

            Assert.True(expected.SetEquals(actual), $"Broadphase mismatch at move {step}.");
        }
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

    [Fact]
    public void Constructors_RejectNegativeAndNonFiniteSizes_ButAllowZero()
    {
        // Negative sizes previously produced divergent results between the raw
        // narrowphase and the Math.Abs broadphase; they are now rejected at the
        // boundary so every path sees the same non-negative geometry.
        Assert.Throws<ArgumentOutOfRangeException>(() => new Circle(Vec2.Zero, -1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Circle(Vec2.Zero, float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Aabb(Vec2.Zero, new Vec2(-1f, 1f)));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Obb(Vec2.Zero, new Vec2(1f, -1f), 0.3f));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Obb(Vec2.Zero, new Vec2(1f, 1f), float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Capsule(Vec2.Zero, Vec2.UnitX, -0.5f));

        // Zero is a valid degenerate size (point circle, zero-thickness box).
        _ = new Circle(Vec2.Zero, 0f);
        _ = new Aabb(Vec2.Zero, Vec2.Zero);
        _ = new Capsule(Vec2.Zero, Vec2.UnitX, 0f);
    }
}

public class GenericShapeTests
{
    [Fact]
    public void Shape_ExplicitUnion_Remains32Bytes()
    {
        Assert.Equal(32, System.Runtime.CompilerServices.Unsafe.SizeOf<Shape>());
    }

    [Fact]
    public void ShapeVsShape_SupportsEveryShapeCombination()
    {
        Shape[] shapes =
        {
            new Circle(Vec2.Zero, 2),
            new Aabb(Vec2.Zero, new Vec2(2, 2)),
            new Capsule(new Vec2(-2, 0), new Vec2(2, 0), 1),
            new Obb(Vec2.Zero, new Vec2(2, 1), 0.35f),
            new Polygon(new Vec2(-2, -1), new Vec2(2, -1),
                new Vec2(2, 1), new Vec2(0, 2), new Vec2(-2, 1)),
        };

        for (int a = 0; a < shapes.Length; a++)
        {
            for (int b = 0; b < shapes.Length; b++)
            {
                Assert.True(Collide.ShapeVsShape(shapes[a], shapes[b]).Colliding,
                    $"Expected {shapes[a].Kind} vs {shapes[b].Kind} to collide.");
                Shape distant = shapes[b].Moved(new Vec2(100, 100));
                Assert.False(Collide.Overlaps(shapes[a], distant),
                    $"Expected separated {shapes[a].Kind} vs {shapes[b].Kind} to miss.");
            }
        }
    }

    [Fact]
    public void ShapeVsShape_ReversesNormalAndPreservesDepth()
    {
        Shape circle = new Circle(Vec2.Zero, 2);
        Shape box = new Aabb(new Vec2(3, 0), new Vec2(2, 2));

        Manifold forward = Collide.ShapeVsShape(circle, box);
        Manifold reverse = Collide.ShapeVsShape(box, circle);

        Assert.True(forward.Colliding);
        Assert.True(reverse.Colliding);
        Assert.Equal(1f, forward.Normal.X, 1f / 256f);
        Assert.Equal(-1f, reverse.Normal.X, 1f / 256f);
        Assert.Equal(forward.Depth, reverse.Depth, 1f / 256f);
    }

    [Fact]
    public void ShapeVsShape_RespectsConcavePolygonGap()
    {
        Shape concave = new Polygon(
            new Vec2(0, 0), new Vec2(4, 0), new Vec2(4, 1),
            new Vec2(1, 1), new Vec2(1, 4), new Vec2(0, 4));

        Assert.False(Collide.Overlaps(concave, new Circle(new Vec2(3, 3), 0.4f)));
        Assert.True(Collide.Overlaps(concave, new Circle(new Vec2(0.5f, 3), 0.4f)));
    }

    [Fact]
    public void ShapeVsShape_UsesExactObbPrimitivePaths()
    {
        Shape box = new Obb(Vec2.Zero, new Vec2(2, 1), 0.4f);
        Shape touchingCircle = new Circle(new Vec2(2.4f, 1f), 1f);
        Shape nearBox = new Aabb(new Vec2(2.5f, 0), new Vec2(1, 1));
        Shape farBox = new Obb(new Vec2(20, 0), new Vec2(2, 1), -0.7f);

        Manifold circleToBox = Collide.ShapeVsShape(touchingCircle, box);
        Manifold boxToCircle = Collide.ShapeVsShape(box, touchingCircle);
        Assert.True(circleToBox.Colliding);
        Assert.True(boxToCircle.Colliding);
        Assert.Equal(circleToBox.Depth, boxToCircle.Depth, 1f / 256f);
        Assert.Equal(circleToBox.Normal.X, -boxToCircle.Normal.X, 1f / 256f);
        Assert.Equal(circleToBox.Normal.Y, -boxToCircle.Normal.Y, 1f / 256f);
        Assert.True(Collide.Overlaps(nearBox, box));
        Assert.False(Collide.Overlaps(box, farBox));
    }

    [Fact]
    public void ShapeVsShape_CapsuleBoxLightPathsRejectCornerFalsePositives()
    {
        Shape axisBox = new Aabb(Vec2.Zero, new Vec2(1, 1));
        Shape cornerCapsule = new Capsule(new Vec2(1.4f, 1.4f), new Vec2(1.4f, 1.4f), 0.5f);
        Assert.True(axisBox.Bounds.Overlaps(cornerCapsule.Bounds));
        Assert.False(Collide.Overlaps(cornerCapsule, axisBox));
        Assert.False(Collide.ShapeVsShape(axisBox, cornerCapsule).Colliding);
        Assert.False(Collide.CapsuleVsAabb(
            new Capsule(new Vec2(1.4f, 1.4f), new Vec2(1.4f, 1.4f), 0.5f),
            new Aabb(Vec2.Zero, new Vec2(1, 1))).Colliding);

        const float rotation = 0.6f;
        float c = MathF.Cos(rotation);
        float s = MathF.Sin(rotation);
        Vec2 rotatedCorner = new(c * 1.4f - s * 1.4f, s * 1.4f + c * 1.4f);
        Shape rotatedBox = new Obb(Vec2.Zero, new Vec2(1, 1), rotation);
        // 0.56 keeps the exact rounded-corner test separated while making the
        // conservative world-space AABBs overlap after 24.8 quantization.
        Shape rotatedCapsule = new Capsule(rotatedCorner, rotatedCorner, 0.56f);
        Assert.True(rotatedBox.Bounds.Overlaps(rotatedCapsule.Bounds));
        Assert.False(Collide.Overlaps(rotatedCapsule, rotatedBox));
        Assert.False(Collide.ShapeVsShape(rotatedBox, rotatedCapsule).Colliding);
    }

    [Fact]
    public void CrossingCapsules_ReportASeparatingMtv()
    {
        var horizontal = new Capsule(new Vec2(-5, 0), new Vec2(5, 0), 1);
        var vertical = new Capsule(new Vec2(0, -5), new Vec2(0, 5), 1);

        Manifold manifold = Collide.CapsuleVsCapsule(horizontal, vertical);

        Assert.True(manifold.Colliding);
        Assert.Equal(7f, manifold.Depth, 2f / 256f);
        Capsule separated = horizontal.Moved(
            manifold.SeparationForA - manifold.Normal * (2f / 256f));
        Assert.False(Collide.Overlaps(separated, vertical));
    }

    [Fact]
    public void AabbContactRemainsInsideBothOverlappingBounds()
    {
        var a = new Aabb(Vec2.Zero, new Vec2(1, 1));
        var b = new Aabb(new Vec2(0.5f, 5), new Vec2(1, 10));

        Manifold manifold = Collide.AabbVsAabb(a, b);

        Assert.True(manifold.Colliding);
        Assert.True(manifold.Contact.X >= MathF.Max(a.Min.X, b.Min.X)
            && manifold.Contact.X <= MathF.Min(a.Max.X, b.Max.X));
        Assert.True(manifold.Contact.Y >= MathF.Max(a.Min.Y, b.Min.Y)
            && manifold.Contact.Y <= MathF.Min(a.Max.Y, b.Max.Y));
    }
}
