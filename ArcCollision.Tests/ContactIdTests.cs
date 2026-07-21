using System.Collections.Generic;
using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Backend-neutral tests for the persistent contact identity
/// (<see cref="ArcHandle.PairId"/> / <c>ContactPair.Id</c>): a manifold produced
/// this frame must carry the same id as the same colliding pair last frame, even
/// after the colliders are moved with <c>UpdateTransform</c>. Runs against both the
/// reference and the native wrapper backends via the shared compile.
/// </summary>
[Collection("ArcWorld lifecycle")]
public class ContactIdTests
{
    private static ContactPair FindContact(ArcWorld world, List<CandidatePair> pairs)
    {
        world.ComputePairs(pairs);
        foreach (CandidatePair pair in pairs)
            if (world.TryComputeContact(pair, out ContactPair contact))
                return contact;
        Assert.Fail("Expected a contact but found none.");
        return default;
    }

    [Fact]
    public void ContactId_StaysStableWhileMoving()
    {
        using var world = new ArcWorld();
        ArcHandle a = world.Add(1, new Circle(new Vec2(0f, 0f), 1f), CollisionFilter.Default);
        _ = world.Add(2, new Circle(new Vec2(1.5f, 0f), 1f), CollisionFilter.Default); // overlaps `a`

        var pairs = new List<CandidatePair>();
        ulong id0 = FindContact(world, pairs).Id;

        // Nudge `a` each frame; it stays overlapping. The id must not change.
        for (int frame = 1; frame <= 5; frame++)
        {
            world.UpdateTransform(a, new Transform(new Vec2(0.1f * frame, 0f)));
            Assert.Equal(id0, FindContact(world, pairs).Id);
        }
    }

    [Fact]
    public void PairId_IsOrderIndependent()
    {
        using var world = new ArcWorld();
        ArcHandle a = world.Add(1, new Circle(Vec2.Zero, 1f), CollisionFilter.Default);
        ArcHandle b = world.Add(2, new Circle(new Vec2(1.5f, 0f), 1f), CollisionFilter.Default);

        Assert.Equal(ArcHandle.PairId(a, b), ArcHandle.PairId(b, a));
    }

    [Fact]
    public void PairId_DistinguishesDifferentPairs()
    {
        using var world = new ArcWorld();
        ArcHandle a = world.Add(1, new Circle(Vec2.Zero, 1f), CollisionFilter.Default);
        ArcHandle b = world.Add(2, new Circle(new Vec2(1.5f, 0f), 1f), CollisionFilter.Default);
        ArcHandle c = world.Add(3, new Circle(new Vec2(3f, 0f), 1f), CollisionFilter.Default);

        var ids = new HashSet<ulong>
        {
            ArcHandle.PairId(a, b),
            ArcHandle.PairId(a, c),
            ArcHandle.PairId(b, c),
        };
        Assert.Equal(3, ids.Count);
    }

    [Fact]
    public void ContactFrame_IsZeroWhenTrackingDisabled()
    {
        using var world = new ArcWorld();
        world.Add(1, new Circle(Vec2.Zero, 1f), CollisionFilter.Default);
        world.Add(2, new Circle(new Vec2(1.5f, 0f), 1f), CollisionFilter.Default);

        var pairs = new List<CandidatePair>();
        Assert.Equal(0, FindContact(world, pairs).Frame);
    }

    [Fact]
    public void ContactFrame_CountsConsecutiveFrames_AndRestartsAfterSeparation()
    {
        using var world = new ArcWorld { TrackContacts = true };
        ArcHandle a = world.Add(1, new Circle(Vec2.Zero, 1f), CollisionFilter.Default);
        world.Add(2, new Circle(new Vec2(1.5f, 0f), 1f), CollisionFilter.Default);
        var pairs = new List<CandidatePair>();

        Assert.Equal(1, FindContact(world, pairs).Frame); // first collision frame
        Assert.Equal(2, FindContact(world, pairs).Frame);
        Assert.Equal(3, FindContact(world, pairs).Frame);

        // Separate for a frame: no contact resolved.
        world.UpdateTransform(a, new Transform(new Vec2(100f, 0f)));
        world.ComputePairs(pairs);
        bool anyContact = false;
        foreach (CandidatePair pair in pairs)
            if (world.TryComputeContact(pair, out _))
                anyContact = true;
        Assert.False(anyContact);

        // Touch again: the frame count starts over at 1 (a new collision).
        world.UpdateTransform(a, Transform.Identity);
        Assert.Equal(1, FindContact(world, pairs).Frame);
    }

    [Fact]
    public void PairId_IsPerCollider_NotPerEntity()
    {
        using var world = new ArcWorld();
        // One entity (id 7) owning two colliders, e.g. a body and a hurtbox.
        ArcHandle body = world.Add(7, new Circle(Vec2.Zero, 1f), CollisionFilter.Default);
        ArcHandle hurt = world.Add(7, new Circle(Vec2.Zero, 2f), CollisionFilter.Default);
        ArcHandle other = world.Add(8, new Circle(new Vec2(1f, 0f), 1f), CollisionFilter.Default);

        Assert.NotEqual(ArcHandle.PairId(body, other), ArcHandle.PairId(hurt, other));
    }
}
