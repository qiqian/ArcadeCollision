using System;
using System.Collections.Generic;

namespace ArcCollision;

/// <summary>
/// Lightweight collider token. Index and generation identify the ArcWorld slot;
/// EntityId is caller-owned metadata copied into the handle for immediate logic
/// lookup after broadphase queries.
/// </summary>
public readonly struct ArcHandle : IEquatable<ArcHandle>
{
    internal readonly int Index;
    internal readonly uint Generation;
    public readonly int EntityId;

    internal ArcHandle(int index, uint generation, int entityId)
    {
        Index = index;
        Generation = generation;
        EntityId = entityId;
    }

    public bool Equals(ArcHandle other) =>
        Index == other.Index && Generation == other.Generation;

    public override bool Equals(object? obj) => obj is ArcHandle other && Equals(other);
    public override int GetHashCode() => HashCode.Combine(Index, Generation);
    public static bool operator ==(ArcHandle left, ArcHandle right) => left.Equals(right);
    public static bool operator !=(ArcHandle left, ArcHandle right) => !left.Equals(right);
}

/// <summary>A broadphase-only pair. No narrowphase work has been performed.</summary>
public readonly struct CandidatePair
{
    public readonly ArcHandle A;
    public readonly ArcHandle B;
    internal readonly uint Revision;

    internal CandidatePair(ArcHandle a, ArcHandle b, uint revision)
    {
        A = a;
        B = b;
        Revision = revision;
    }
}

/// <summary>A candidate pair whose narrowphase produced a manifold.</summary>
public readonly struct ContactPair
{
    public readonly ArcHandle A;
    public readonly ArcHandle B;
    public readonly Manifold Manifold;

    internal ContactPair(ArcHandle a, ArcHandle b, Manifold manifold)
    {
        A = a;
        B = b;
        Manifold = manifold;
    }
}

/// <summary>
/// Owns collider shapes and their broadphase proxies. Logic submits shapes on
/// add/update, receives lightweight handles from broadphase, filters candidates,
/// then selectively asks the world to compute final manifolds.
/// </summary>
public sealed class ArcWorld
{
    private struct Slot
    {
        public Shape Shape;
        public BpBounds Bounds;
        public int EntityId;
        public int TreeProxy;
        public int NextFree;
        public uint Generation;
        public bool Active;
        public bool Static;
    }

    private readonly SpatialHash _broadphase;
    private readonly List<int> _candidates = new();
    private readonly List<(int A, int B)> _broadphasePairs = new();
    private Slot[] _slots = new Slot[16];
    private int _slotCount;
    private int _freeList = -1;
    private int _activeCount;
    private int _dynamicCount;
    private uint _revision = 1;

    public ArcWorld(float fatMargin = 16f)
    {
        _broadphase = new SpatialHash(fatMargin);
    }

    public int Count => _activeCount;
    public int DynamicCount => _dynamicCount;
    public int StaticCount => _activeCount - _dynamicCount;
    public float FatMargin => _broadphase.FatMargin;

    public ArcHandle Add(int entityId, in Shape shape) => AddCore(entityId, shape, isStatic: false);
    public ArcHandle AddStatic(int entityId, in Shape shape) => AddCore(entityId, shape, isStatic: true);

    /// <summary>
    /// Rebuilds the immutable static BVH immediately. Call after batching static
    /// additions/updates to keep the first gameplay query free of build work.
    /// Query and ComputePairs still rebuild lazily when this is omitted.
    /// </summary>
    public void BuildStatic() => _broadphase.BuildStatic();

    public void Update(ArcHandle handle, in Shape shape)
    {
        ref Slot slot = ref GetSlot(handle);
        BpBounds bounds = new(shape);
        slot.Shape = shape;
        slot.Bounds = bounds;
        if (slot.Static)
        {
            _broadphase.AddOrUpdateStatic(handle.Index, bounds);
        }
        else
        {
            _broadphase.UpdateDynamic(slot.TreeProxy, bounds);
        }
        AdvanceRevision();
    }

    public void Remove(ArcHandle handle)
    {
        ref Slot slot = ref GetSlot(handle);
        if (slot.Static)
        {
            _broadphase.RemoveStatic(handle.Index);
        }
        else
        {
            _broadphase.RemoveDynamic(slot.TreeProxy);
            _dynamicCount--;
        }

        slot.Shape = default;
        slot.Bounds = default;
        slot.EntityId = 0;
        slot.TreeProxy = -1;
        slot.Active = false;
        slot.Static = false;
        slot.Generation = NextGeneration(slot.Generation);
        slot.NextFree = _freeList;
        _freeList = handle.Index;
        _activeCount--;
        AdvanceRevision();
    }

    public bool IsValid(ArcHandle handle) =>
        (uint)handle.Index < (uint)_slotCount
        && _slots[handle.Index].Active
        && _slots[handle.Index].Generation == handle.Generation;

    public void Clear()
    {
        _broadphase.Clear();
        _candidates.Clear();
        _broadphasePairs.Clear();
        _activeCount = _dynamicCount = 0;

        _freeList = _slotCount == 0 ? -1 : 0;
        for (int i = 0; i < _slotCount; i++)
        {
            ref Slot slot = ref _slots[i];
            slot.Shape = default;
            slot.Bounds = default;
            slot.EntityId = 0;
            slot.TreeProxy = -1;
            slot.Active = false;
            slot.Static = false;
            slot.Generation = NextGeneration(slot.Generation);
            slot.NextFree = i + 1 < _slotCount ? i + 1 : -1;
        }
        AdvanceRevision();
    }

    /// <summary>Collects broadphase candidates only; no manifolds are computed.</summary>
    public void ComputePairs(List<CandidatePair> results)
    {
        ArgumentNullException.ThrowIfNull(results);
        results.Clear();
        _broadphase.ComputePairs(_broadphasePairs);
        for (int i = 0; i < _broadphasePairs.Count; i++)
        {
            (int a, int b) = _broadphasePairs[i];
            if (_slots[a].Active && _slots[b].Active
                && _slots[a].Bounds.Overlaps(_slots[b].Bounds))
                results.Add(CreatePair(a, b));
        }
    }

    /// <summary>Returns broadphase handles overlapping a transient query shape.</summary>
    public void Query(in Shape query, List<ArcHandle> results)
    {
        ArgumentNullException.ThrowIfNull(results);
        results.Clear();
        BpBounds bounds = new(query);

        _candidates.Clear();
        _broadphase.QueryDynamic(bounds, _candidates);
        AppendQueryResults(bounds, results);
        _candidates.Clear();
        _broadphase.QueryStatic(bounds, _candidates);
        AppendQueryResults(bounds, results);
    }

    /// <summary>
    /// Computes narrowphase only for the selected candidate. Returns false when
    /// the pair is stale, invalid, or no longer colliding.
    /// </summary>
    public bool TryComputeContact(in CandidatePair pair, out ContactPair contact)
    {
        if (pair.Revision != _revision || !IsValid(pair.A) || !IsValid(pair.B))
        {
            contact = default;
            return false;
        }

        Manifold manifold = Collide.ShapeVsShape(
            _slots[pair.A.Index].Shape, _slots[pair.B.Index].Shape);
        if (!manifold.Colliding)
        {
            contact = default;
            return false;
        }

        contact = new ContactPair(pair.A, pair.B, manifold);
        return true;
    }

    /// <summary>Computes a transient query shape against one selected handle.</summary>
    public bool TryComputeContact(
        in Shape query, ArcHandle target, out Manifold manifold)
    {
        if (!IsValid(target))
        {
            manifold = Manifold.None;
            return false;
        }
        manifold = Collide.ShapeVsShape(query, _slots[target.Index].Shape);
        return manifold.Colliding;
    }

    private ArcHandle AddCore(int entityId, in Shape shape, bool isStatic)
    {
        int index = AllocateSlot();
        ref Slot slot = ref _slots[index];
        slot.Shape = shape;
        slot.Bounds = new BpBounds(shape);
        slot.EntityId = entityId;
        slot.TreeProxy = -1;
        slot.Active = true;
        slot.Static = isStatic;
        if (slot.Generation == 0) slot.Generation = 1;

        if (isStatic)
        {
            _broadphase.AddOrUpdateStatic(index, slot.Bounds);
        }
        else
        {
            slot.TreeProxy = _broadphase.AddDynamic(index, slot.Bounds);
            _dynamicCount++;
        }

        _activeCount++;
        AdvanceRevision();
        return CreateHandle(index);
    }

    private int AllocateSlot()
    {
        if (_freeList != -1)
        {
            int index = _freeList;
            _freeList = _slots[index].NextFree;
            _slots[index].NextFree = -1;
            return index;
        }

        if (_slotCount == _slots.Length)
            Array.Resize(ref _slots, _slots.Length * 2);
        return _slotCount++;
    }

    private ref Slot GetSlot(ArcHandle handle)
    {
        if (!IsValid(handle))
            throw new ArgumentException("Handle is stale or does not belong to this world.", nameof(handle));
        return ref _slots[handle.Index];
    }

    private void AppendQueryResults(BpBounds bounds, List<ArcHandle> results)
    {
        for (int i = 0; i < _candidates.Count; i++)
        {
            int index = _candidates[i];
            if (_slots[index].Active && _slots[index].Bounds.Overlaps(bounds))
                results.Add(CreateHandle(index));
        }
    }

    private CandidatePair CreatePair(int a, int b) =>
        new(CreateHandle(a), CreateHandle(b), _revision);

    private ArcHandle CreateHandle(int index)
    {
        ref Slot slot = ref _slots[index];
        return new ArcHandle(index, slot.Generation, slot.EntityId);
    }

    private void AdvanceRevision()
    {
        _revision++;
        if (_revision == 0) _revision = 1;
    }

    private static uint NextGeneration(uint generation)
    {
        generation++;
        return generation == 0 ? 1u : generation;
    }
}
