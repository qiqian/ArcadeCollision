using System;
using System.Collections.Generic;
using ArcCollision.Tests.Support;
using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Large-world, many-object broadphase validation. Ground truth is a brute-force
/// O(n²) sweep over test-side replicas of the quantized bounds; ArcWorld must
/// return exactly that pair set (minus static-static, which the hybrid
/// broadphase intentionally never reports), stay exact through hundreds of
/// frames of add/remove/update churn, and behave deterministically.
/// </summary>
public class BroadphaseScaleTests
{
    private sealed class Tracked
    {
        public int Id;
        public ArcHandle Handle;
        public bool IsStatic;
        public ShapeKind Kind;
        public Circle C;
        public Aabb B;
        public Capsule K;
        public Obb O;
        public Polygon? P;

        public Shape AsShape() => Kind switch
        {
            ShapeKind.Circle => C,
            ShapeKind.Aabb => B,
            ShapeKind.Capsule => K,
            ShapeKind.Obb => O,
            _ => P!,
        };

        public (long, long, long, long) Bounds() => Kind switch
        {
            ShapeKind.Circle => TestGeo.BoundsOf(C),
            ShapeKind.Aabb => TestGeo.BoundsOf(B),
            ShapeKind.Capsule => TestGeo.BoundsOf(K),
            ShapeKind.Obb => TestGeo.BoundsOf(O),
            _ => TestGeo.BoundsOf(P!),
        };
    }

    private static Tracked MakeTracked(Random rng, int id, Vec2 at, float sizeMax)
    {
        float Size() => 0.5f + (float)rng.NextDouble() * sizeMax;
        var t = new Tracked { Id = id, Kind = (ShapeKind)rng.Next(5) };
        switch (t.Kind)
        {
            case ShapeKind.Circle:
                t.C = new Circle(at, Size());
                break;
            case ShapeKind.Aabb:
                t.B = new Aabb(at, new Vec2(Size(), Size()));
                break;
            case ShapeKind.Capsule:
                t.K = new Capsule(at, at + new Vec2(Size(), Size() * 0.5f), Size() * 0.4f);
                break;
            case ShapeKind.Obb:
                t.O = new Obb(at, new Vec2(Size(), Size() * 0.6f),
                    (float)(rng.NextDouble() * Math.PI * 2 - Math.PI));
                break;
            default:
                float s = Size();
                t.P = new Polygon(
                    at + new Vec2(-s, -s * 0.7f),
                    at + new Vec2(s, -s * 0.5f),
                    at + new Vec2(s * 0.6f, s),
                    at + new Vec2(-s * 0.4f, s * 0.8f));
                break;
        }
        return t;
    }

    private static Vec2 ClusteredPosition(Random rng, Vec2[] clusters, float span, float clusterRadius)
    {
        if (rng.Next(100) < 85)
        {
            Vec2 c = clusters[rng.Next(clusters.Length)];
            return new Vec2(
                c.X + (float)(rng.NextDouble() * 2 - 1) * clusterRadius,
                c.Y + (float)(rng.NextDouble() * 2 - 1) * clusterRadius);
        }
        return new Vec2(
            (float)(rng.NextDouble() * 2 - 1) * span,
            (float)(rng.NextDouble() * 2 - 1) * span);
    }

    private static HashSet<(int, int)> BruteForcePairs(List<Tracked> entities)
    {
        var bounds = new (long, long, long, long)[entities.Count];
        for (int i = 0; i < entities.Count; i++) bounds[i] = entities[i].Bounds();

        var expected = new HashSet<(int, int)>();
        for (int i = 0; i < entities.Count; i++)
        {
            for (int j = i + 1; j < entities.Count; j++)
            {
                if (entities[i].IsStatic && entities[j].IsStatic)
                    continue;   // the hybrid broadphase never reports static-static
                if (TestGeo.BoundsOverlap(bounds[i], bounds[j]))
                    expected.Add(Key(entities[i].Id, entities[j].Id));
            }
        }
        return expected;
    }

    private static (int, int) Key(int a, int b) => a < b ? (a, b) : (b, a);

    private static HashSet<(int, int)> WorldPairs(ArcWorld world, List<CandidatePair> scratch)
    {
        world.ComputePairs(scratch);
        var set = new HashSet<(int, int)>();
        foreach (CandidatePair pair in scratch)
        {
            (int, int) key = Key(pair.A.EntityId, pair.B.EntityId);
            Assert.True(set.Add(key), $"duplicate candidate pair {key}");
        }
        return set;
    }

    private static void AssertSetsEqual(
        HashSet<(int, int)> expected, HashSet<(int, int)> actual, string context)
    {
        if (expected.SetEquals(actual)) return;
        var missing = new List<(int, int)>();
        foreach (var p in expected) if (!actual.Contains(p)) missing.Add(p);
        var extra = new List<(int, int)>();
        foreach (var p in actual) if (!expected.Contains(p)) extra.Add(p);
        Assert.Fail($"{context}: {missing.Count} missing (e.g. {(missing.Count > 0 ? missing[0] : default)}), "
            + $"{extra.Count} phantom (e.g. {(extra.Count > 0 ? extra[0] : default)})");
    }

    [Fact]
    public void LargeWorld_4000MixedShapes_PairsMatchBruteForceExactly()
    {
        var rng = new Random(31337);
        var world = new ArcWorld(16f);
        var entities = new List<Tracked>();

        // 60 dense clusters scattered across ±1.8M plus uniform strays.
        var clusters = new Vec2[60];
        for (int i = 0; i < clusters.Length; i++)
            clusters[i] = new Vec2(
                (float)(rng.NextDouble() * 2 - 1) * 1_800_000f,
                (float)(rng.NextDouble() * 2 - 1) * 1_800_000f);

        for (int id = 0; id < 4000; id++)
        {
            Vec2 at = ClusteredPosition(rng, clusters, 1_800_000f, 2500f);
            Tracked t = MakeTracked(rng, id, at, 400f);
            t.IsStatic = id < 2500;
            t.Handle = t.IsStatic ? world.AddStatic(id, t.AsShape()) : world.Add(id, t.AsShape());
            entities.Add(t);
        }
        world.BuildStatic();

        HashSet<(int, int)> expected = BruteForcePairs(entities);
        HashSet<(int, int)> actual = WorldPairs(world, new List<CandidatePair>());

        Assert.True(expected.Count > 500,
            $"world too sparse ({expected.Count} pairs) — clustering broken?");
        AssertSetsEqual(expected, actual, "initial build");

        // Narrowphase completeness: anything that truly overlaps must have been
        // a candidate (catches bounds-computation bugs), and a random sample of
        // non-candidate pairs must be genuinely non-overlapping.
        foreach ((int a, int b) in actual)
        {
            // candidates are bounds-overlapping by construction; nothing to do —
            // the set equality above already proves the candidate side.
            _ = a; _ = b;
            break;
        }
        var byId = new Dictionary<int, Tracked>();
        foreach (Tracked t in entities) byId[t.Id] = t;
        int sampled = 0;
        while (sampled < 20000)
        {
            int a = rng.Next(4000), b = rng.Next(4000);
            if (a == b) continue;
            sampled++;
            if (actual.Contains(Key(a, b))) continue;
            if (byId[a].IsStatic && byId[b].IsStatic) continue;
            if (Collide.Overlaps(byId[a].AsShape(), byId[b].AsShape()))
                Assert.Fail($"non-candidate pair ({a},{b}) actually overlaps — bounds too tight");
        }
    }

    [Fact]
    public void Churn_300Frames_AddRemoveUpdateTeleport_StaysExact()
    {
        var rng = new Random(4242);
        var world = new ArcWorld(12f);
        var entities = new List<Tracked>();
        var scratch = new List<CandidatePair>();
        var queryResults = new List<ArcHandle>();
        int nextId = 0;

        Vec2 RandomPos() => new(
            (float)(rng.NextDouble() * 2 - 1) * 200_000f,
            (float)(rng.NextDouble() * 2 - 1) * 200_000f);

        // Seed the world: 300 dynamic + 100 static.
        for (int i = 0; i < 400; i++)
        {
            Tracked t = MakeTracked(rng, nextId++, RandomPos(), 3000f);
            t.IsStatic = i >= 300;
            t.Handle = t.IsStatic ? world.AddStatic(t.Id, t.AsShape()) : world.Add(t.Id, t.AsShape());
            entities.Add(t);
        }

        for (int frame = 0; frame < 300; frame++)
        {
            int op = rng.Next(100);
            if (op < 55 && entities.Count > 0)
            {
                // Move a dynamic entity: small jiggle (inside the fat margin) or teleport.
                Tracked t = entities[rng.Next(entities.Count)];
                bool teleport = rng.Next(4) == 0;
                Vec2 at = teleport
                    ? RandomPos()
                    : Anchor(t) + new Vec2(rng.Next(-8, 9), rng.Next(-8, 9));
                Tracked moved = MakeTracked(rng, t.Id, at, 3000f);
                moved.IsStatic = t.IsStatic;
                moved.Handle = t.Handle;
                world.Update(t.Handle, moved.AsShape());
                entities[entities.IndexOf(t)] = moved;
            }
            else if (op < 75)
            {
                Tracked t = MakeTracked(rng, nextId++, RandomPos(), 3000f);
                t.IsStatic = rng.Next(4) == 0;
                t.Handle = t.IsStatic ? world.AddStatic(t.Id, t.AsShape()) : world.Add(t.Id, t.AsShape());
                entities.Add(t);
            }
            else if (op < 90 && entities.Count > 50)
            {
                int index = rng.Next(entities.Count);
                ArcHandle stale = entities[index].Handle;
                world.Remove(stale);
                Assert.False(world.IsValid(stale), "handle stayed valid after removal");
                entities.RemoveAt(index);
            }
            // else: no structural op this frame.

            HashSet<(int, int)> expected = BruteForcePairs(entities);
            HashSet<(int, int)> actual = WorldPairs(world, scratch);
            AssertSetsEqual(expected, actual, $"frame {frame}");

            // Query equivalence for a random transient shape.
            if (frame % 10 == 0)
            {
                Tracked probe = MakeTracked(rng, -1, RandomPos(), 20000f);
                var probeBounds = probe.Bounds();
                world.Query(probe.AsShape(), queryResults);
                var actualIds = new HashSet<int>();
                foreach (ArcHandle h in queryResults)
                    Assert.True(actualIds.Add(h.EntityId), "duplicate query result");
                var expectedIds = new HashSet<int>();
                foreach (Tracked t in entities)
                    if (TestGeo.BoundsOverlap(probeBounds, t.Bounds()))
                        expectedIds.Add(t.Id);
                Assert.True(expectedIds.SetEquals(actualIds),
                    $"query mismatch at frame {frame}: expected {expectedIds.Count}, got {actualIds.Count}");
            }
        }

        Assert.Equal(entities.Count, world.Count);

        static Vec2 Anchor(Tracked t) => t.Kind switch
        {
            ShapeKind.Circle => t.C.Center,
            ShapeKind.Aabb => t.B.Center,
            ShapeKind.Capsule => t.K.A,
            ShapeKind.Obb => t.O.Center,
            _ => t.P!.Bounds.Center,
        };
    }

    [Fact]
    public void Determinism_SameSeedTwice_IdenticalPairSets()
    {
        HashSet<(int, int)> Run()
        {
            var rng = new Random(9001);
            var world = new ArcWorld(16f);
            var entities = new List<Tracked>();
            for (int id = 0; id < 800; id++)
            {
                Vec2 at = new(
                    (float)(rng.NextDouble() * 2 - 1) * 50_000f,
                    (float)(rng.NextDouble() * 2 - 1) * 50_000f);
                Tracked t = MakeTracked(rng, id, at, 800f);
                t.IsStatic = id % 3 == 0;
                t.Handle = t.IsStatic ? world.AddStatic(id, t.AsShape()) : world.Add(id, t.AsShape());
                entities.Add(t);
            }
            // Churn a little before comparing.
            for (int i = 0; i < 100; i++)
            {
                Tracked t = entities[rng.Next(entities.Count)];
                if (t.IsStatic) continue;
                Tracked moved = MakeTracked(rng, t.Id, new Vec2(
                    (float)(rng.NextDouble() * 2 - 1) * 50_000f,
                    (float)(rng.NextDouble() * 2 - 1) * 50_000f), 800f);
                moved.IsStatic = false;
                moved.Handle = t.Handle;
                world.Update(t.Handle, moved.AsShape());
                entities[entities.IndexOf(t)] = moved;
            }
            return WorldPairs(world, new List<CandidatePair>());
        }

        HashSet<(int, int)> first = Run();
        HashSet<(int, int)> second = Run();
        Assert.True(first.SetEquals(second), "same seed produced different pair sets");
    }

    [Fact]
    public void CandidateUsesCurrentStateAfterWorldChanges()
    {
        var world = new ArcWorld(8f);
        ArcHandle a = world.Add(1, new Circle(new Vec2(0, 0), 5f));
        world.Add(2, new Circle(new Vec2(4, 0), 5f));
        var pairs = new List<CandidatePair>();
        world.ComputePairs(pairs);
        CandidatePair pair = Assert.Single(pairs);
        Assert.True(world.TryComputeContact(pair, out _));

        // Candidates are broadphase hints. Current handle, filter and shape state
        // is rechecked instead of invalidating every pair for an unrelated update.
        world.Update(a, new Circle(new Vec2(0, 0), 5f));
        Assert.True(world.TryComputeContact(pair, out _));
    }
}
