using System;
using System.Collections.Generic;
using System.Linq;
using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Backend-neutral tests for <c>ArcWorld.QueryBatch</c>. A batch query must return
/// exactly what the single-shape <c>Query</c> returns for each query, in the same
/// order, with per-query counts that partition the flat result buffer. The shared
/// compile runs this against BOTH the reference backend (which loops) and the
/// native wrapper backend (whose 4-wide SIMD packet descent must agree lane for
/// lane), so it pins the packet traversal against the scalar path on each side.
/// (Ref-vs-native equivalence itself is covered by the parity tests/benchmark.)
/// </summary>
public class QueryBatchTests
{
    // Enough colliders, static and dynamic, that queries hit varied subsets.
    private static ArcWorld BuildPopulatedWorld()
    {
        var world = new ArcWorld();
        int id = 0;
        for (int x = 0; x < 6; x++)
            for (int y = 0; y < 6; y++)
                world.AddStatic(id++, new Aabb(new Vec2(x * 10, y * 10), new Vec2(2, 2)));
        for (int i = 0; i < 12; i++)
            world.Add(1000 + i, new Circle(new Vec2(i * 7, i * 5), 3));
        world.BuildStatic();
        return world;
    }

    // Seven queries: multiple full packet groups plus a partial tail (7 % 4 == 3),
    // including an empty one and one that covers the whole scene.
    private static Shape[] SampleQueries() => new Shape[]
    {
        new Aabb(new Vec2(0, 0), new Vec2(5, 5)),
        new Circle(new Vec2(30, 20), 8f),
        new Aabb(new Vec2(50, 50), new Vec2(15, 15)),
        new Circle(new Vec2(-500, -500), 1f),          // overlaps nothing
        new Aabb(new Vec2(21, 14), new Vec2(20, 20)),
        new Circle(new Vec2(7, 5), 6f),
        new Aabb(new Vec2(25, 25), new Vec2(100, 100)), // covers the whole scene
    };

    [Fact]
    public void UnfilteredBatch_MatchesPerQuerySingleQuery()
    {
        using ArcWorld world = BuildPopulatedWorld();
        Shape[] queries = SampleQueries();

        var batch = new List<ArcHandle>();
        var counts = new List<int>();
        world.QueryBatch(queries, batch, counts);

        Assert.Equal(queries.Length, counts.Count);
        Assert.Equal(batch.Count, counts.Sum());

        var single = new List<ArcHandle>();
        int offset = 0;
        for (int i = 0; i < queries.Length; i++)
        {
            world.Query(queries[i], single);
            List<ArcHandle> slice = batch.GetRange(offset, counts[i]);
            Assert.Equal(single, slice);
            offset += counts[i];
        }
    }

    [Fact]
    public void FilteredBatch_MatchesPerQueryFilteredSingleQuery()
    {
        const uint solid = 1u << 1;
        const uint ghost = 1u << 2;
        const uint probe = 1u << 3;

        using var world = new ArcWorld();
        int id = 0;
        // Interleave two categories so the filter actually excludes half of them.
        for (int x = 0; x < 6; x++)
            for (int y = 0; y < 6; y++)
            {
                uint category = ((x + y) & 1) == 0 ? solid : ghost;
                world.AddStatic(id, new Aabb(new Vec2(x * 10, y * 10), new Vec2(2, 2)),
                    new CollisionFilter(category));
                id++;
            }
        for (int i = 0; i < 12; i++)
        {
            uint category = (i & 1) == 0 ? solid : ghost;
            world.Add(1000 + i, new Circle(new Vec2(i * 7, i * 5), 3f),
                new CollisionFilter(category));
        }
        world.BuildStatic();

        // A probe that only collides with `solid`; `ghost` colliders must drop out.
        var filter = new CollisionFilter(probe, collidesWith: solid);
        Shape[] queries = SampleQueries();

        var batch = new List<ArcHandle>();
        var counts = new List<int>();
        world.QueryBatch(queries, filter, batch, counts);

        Assert.Equal(queries.Length, counts.Count);
        Assert.Equal(batch.Count, counts.Sum());

        var single = new List<ArcHandle>();
        int offset = 0;
        for (int i = 0; i < queries.Length; i++)
        {
            world.Query(queries[i], filter, single);
            List<ArcHandle> slice = batch.GetRange(offset, counts[i]);
            Assert.Equal(single, slice);
            offset += counts[i];
        }
    }

    [Fact]
    public void EmptyBatch_ClearsOutputsAndReturnsNothing()
    {
        using ArcWorld world = BuildPopulatedWorld();
        var batch = new List<ArcHandle> { default };
        var counts = new List<int> { 99 };

        world.QueryBatch(ReadOnlySpan<Shape>.Empty, batch, counts);

        Assert.Empty(batch);
        Assert.Empty(counts);
    }

    [Fact]
    public void SingleQueryBatch_MatchesSingleQuery()
    {
        using ArcWorld world = BuildPopulatedWorld();
        Shape[] queries = { new Aabb(new Vec2(25, 25), new Vec2(100, 100)) };

        var batch = new List<ArcHandle>();
        var counts = new List<int>();
        world.QueryBatch(queries, batch, counts);

        var single = new List<ArcHandle>();
        world.Query(queries[0], single);

        Assert.Single(counts);
        Assert.Equal(single.Count, counts[0]);
        Assert.Equal(single, batch);
    }
}
