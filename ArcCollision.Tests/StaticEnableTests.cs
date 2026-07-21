using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Locks the static-collider broadphase invariant: the static BVH holds every
/// <em>active</em> static slot, including disabled ones, and <c>Enabled</c> is
/// applied after the broadphase instead.
///
/// This exists so toggling a static never dirties the one-shot SAH tree (which
/// would rebuild every leaf on the next query). The trade is that a disabled
/// static stays in the tree as dead weight, and the post-broadphase
/// <c>Enabled</c> test becomes the only thing keeping it out of results -- so
/// that filter, and the slot-recycling edges around it, are pinned here.
/// </summary>
[Collection("ArcWorld lifecycle")]
public class StaticEnableTests
{
    private static Shape BoxAt(float x, float y) =>
        new Aabb(new Vec2(x, y), new Vec2(1f, 1f));

    /// <summary>A query centred on the collider must not see it while disabled.</summary>
    [Fact]
    public void DisabledStaticIsExcludedFromQuery()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.AddStatic(1, BoxAt(0f, 0f));
        world.BuildStatic();

        var results = new List<ArcHandle>();
        world.Query(BoxAt(0f, 0f), results);
        Assert.Single(results);

        world.SetEnabled(handle, false);
        world.Query(BoxAt(0f, 0f), results);
        Assert.Empty(results);

        world.SetEnabled(handle, true);
        world.Query(BoxAt(0f, 0f), results);
        Assert.Single(results);
        Assert.Equal(1, results[0].EntityId);
    }

    /// <summary>The dynamic-vs-static pair pass must honour the same filter.</summary>
    [Fact]
    public void DisabledStaticProducesNoCandidatePairs()
    {
        using var world = new ArcWorld();
        ArcHandle staticHandle = world.AddStatic(1, BoxAt(0f, 0f));
        world.Add(2, BoxAt(0.5f, 0f));
        world.BuildStatic();

        var pairs = new List<CandidatePair>();
        world.ComputePairs(pairs);
        Assert.Single(pairs);

        world.SetEnabled(staticHandle, false);
        world.ComputePairs(pairs);
        Assert.Empty(pairs);

        world.SetEnabled(staticHandle, true);
        world.ComputePairs(pairs);
        Assert.Single(pairs);
    }

    /// <summary>
    /// A static added disabled is still a BVH member, so enabling it later must
    /// make it visible. Before membership tracked Active, the enable path skipped
    /// the tree and such a collider stayed invisible forever.
    /// </summary>
    [Fact]
    public void StaticAddedDisabledBecomesVisibleWhenEnabled()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.AddStatic(
            7, BoxAt(0f, 0f), CollisionFilter.Default, enabled: false);
        world.BuildStatic();

        var results = new List<ArcHandle>();
        world.Query(BoxAt(0f, 0f), results);
        Assert.Empty(results);

        world.SetEnabled(handle, true);
        world.Query(BoxAt(0f, 0f), results);
        Assert.Single(results);
        Assert.Equal(7, results[0].EntityId);
    }

    /// <summary>
    /// Removing a disabled static must drop its leaf. A retained leaf is keyed by
    /// slot index, so once the slot is recycled it would alias the new occupant
    /// and report it from the static tree using the dead collider's bounds.
    /// </summary>
    [Fact]
    public void RemovingDisabledStaticLeavesNoStaleLeaf()
    {
        using var world = new ArcWorld();
        ArcHandle stale = world.AddStatic(1, BoxAt(0f, 0f));
        world.BuildStatic();

        world.SetEnabled(stale, false);
        world.Remove(stale);

        // Recycles the freed slot with a dynamic collider placed elsewhere.
        ArcHandle reused = world.Add(2, BoxAt(50f, 50f));
        Assert.Equal(stale.Index, reused.Index);

        // The removed static's old location must be empty.
        var results = new List<ArcHandle>();
        world.Query(BoxAt(0f, 0f), results);
        Assert.Empty(results);

        // The new occupant must be reported exactly once, from the dynamic tree.
        world.Query(BoxAt(50f, 50f), results);
        Assert.Single(results);
        Assert.Equal(2, results[0].EntityId);
    }

    /// <summary>
    /// Moving a disabled static must still refresh its leaf bounds, otherwise
    /// re-enabling would expose the collider at its pre-move position.
    /// </summary>
    [Fact]
    public void MovingDisabledStaticUpdatesItsBounds()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.AddStatic(3, BoxAt(0f, 0f));
        world.BuildStatic();

        world.SetEnabled(handle, false);
        world.UpdateTransform(handle, new Transform(new Vec2(20f, 0f)));
        world.SetEnabled(handle, true);

        var results = new List<ArcHandle>();
        world.Query(BoxAt(0f, 0f), results);
        Assert.Empty(results);

        world.Query(BoxAt(20f, 0f), results);
        Assert.Single(results);
        Assert.Equal(3, results[0].EntityId);
    }

    /// <summary>Casts resolve against the same filtered candidate set.</summary>
    [Fact]
    public void DisabledStaticIsExcludedFromCasts()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.AddStatic(4, BoxAt(10f, 0f));
        world.BuildStatic();

        Assert.True(world.RayCast(new Vec2(0f, 0f), new Vec2(20f, 0f), out _));

        world.SetEnabled(handle, false);
        Assert.False(world.RayCast(new Vec2(0f, 0f), new Vec2(20f, 0f), out _));

        world.SetEnabled(handle, true);
        Assert.True(world.RayCast(new Vec2(0f, 0f), new Vec2(20f, 0f), out _));
    }

    /// <summary>
    /// Toggling must not disturb the reported counts: EnabledCount tracks the
    /// flag while StaticCount stays slot-based.
    /// </summary>
    [Fact]
    public void TogglingStaticKeepsCountsConsistent()
    {
        using var world = new ArcWorld();
        ArcHandle handle = world.AddStatic(1, BoxAt(0f, 0f));
        world.Add(2, BoxAt(50f, 50f));
        world.BuildStatic();

        Assert.Equal(2, world.Count);
        Assert.Equal(2, world.EnabledCount);
        Assert.Equal(1, world.StaticCount);
        Assert.Equal(1, world.DynamicCount);

        world.SetEnabled(handle, false);
        Assert.Equal(2, world.Count);
        Assert.Equal(1, world.EnabledCount);
        Assert.Equal(1, world.StaticCount);
        Assert.Equal(1, world.DynamicCount);

        world.SetEnabled(handle, true);
        Assert.Equal(2, world.EnabledCount);
        Assert.Equal(1, world.StaticCount);
    }
}
