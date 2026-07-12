using System;
using System.Collections.Generic;
using System.Threading;

namespace ArcCollision;

/// <summary>
/// Lightweight collider token. Index and generation identify the ArcWorld slot.
/// EntityId is caller-owned metadata in [0, <see cref="MaxEntityId"/>]; its low
/// 28 bits share storage with the handle's internal 4-bit world id.
/// </summary>
public readonly struct ArcHandle : IEquatable<ArcHandle>
{
    internal const int EntityIdBits = 28;
    public const int MaxEntityId = (1 << EntityIdBits) - 1;
    private const int EntityIdMask = MaxEntityId;

    internal readonly int Index;
    internal readonly uint Generation;
    private readonly int _packedEntityId;

    public int EntityId => _packedEntityId & EntityIdMask;
    internal uint WorldId => (uint)_packedEntityId >> EntityIdBits;

    internal ArcHandle(int index, uint generation, uint worldId, int entityId)
    {
        Index = index;
        Generation = generation;
        if (worldId is 0 or > ArcWorld.MaxWorldCount
            || (uint)entityId > MaxEntityId)
            throw new ArgumentOutOfRangeException();
        _packedEntityId = unchecked((int)(worldId << EntityIdBits)) | entityId;
    }

    public bool Equals(ArcHandle other) =>
        Index == other.Index && Generation == other.Generation && WorldId == other.WorldId;

    public override bool Equals(object? obj) => obj is ArcHandle other && Equals(other);
    public override int GetHashCode() => HashCode.Combine(Index, Generation, WorldId);
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
/// At most <see cref="MaxWorldCount"/> worlds may be alive concurrently. Dispose
/// worlds when finished so their 4-bit id can be reused immediately.
/// </summary>
public sealed class ArcWorld : IDisposable
{
    public const int MaxWorldCount = 15;
    private static readonly object s_worldIdGate = new();
    private static readonly WeakReference<ArcWorld>?[] s_worldOwners =
        new WeakReference<ArcWorld>?[MaxWorldCount + 1];
    private static readonly int[] s_nextHandleGenerations = new int[MaxWorldCount + 1];

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
    private readonly uint _worldId;
    private readonly List<int> _candidates = new();
    private readonly List<(int A, int B)> _broadphasePairs = new();
    private Slot[] _slots = new Slot[16];
    private int _slotCount;
    private int _freeList = -1;
    private int _activeCount;
    private int _dynamicCount;
    private int _disposed;
    private uint _revision = 1;

    public ArcWorld(float fatMargin = 16f)
    {
        _broadphase = new SpatialHash(fatMargin);
        _worldId = AllocateWorldId(this);
    }

    public int Count { get { ThrowIfDisposed(); return _activeCount; } }
    public int DynamicCount { get { ThrowIfDisposed(); return _dynamicCount; } }
    public int StaticCount { get { ThrowIfDisposed(); return _activeCount - _dynamicCount; } }
    public float FatMargin { get { ThrowIfDisposed(); return _broadphase.FatMargin; } }

    public ArcHandle Add(int entityId, in Shape shape) => AddCore(entityId, shape, isStatic: false);
    public ArcHandle AddStatic(int entityId, in Shape shape) => AddCore(entityId, shape, isStatic: true);

    /// <summary>
    /// Rebuilds the immutable static BVH immediately. Call after batching static
    /// additions/updates to keep the first gameplay query free of build work.
    /// Query and ComputePairs still rebuild lazily when this is omitted.
    /// </summary>
    public void BuildStatic()
    {
        ThrowIfDisposed();
        _broadphase.BuildStatic();
    }

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
        slot.Generation = 0;
        slot.NextFree = _freeList;
        _freeList = handle.Index;
        _activeCount--;
        AdvanceRevision();
    }

    public bool IsValid(ArcHandle handle) =>
        Volatile.Read(ref _disposed) == 0
        && handle.WorldId == _worldId
        && (uint)handle.Index < (uint)_slotCount
        && _slots[handle.Index].Active
        && _slots[handle.Index].Generation == handle.Generation;

    public void Clear()
    {
        ThrowIfDisposed();
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
            slot.Generation = 0;
            slot.NextFree = i + 1 < _slotCount ? i + 1 : -1;
        }
        AdvanceRevision();
    }

    /// <summary>Collects broadphase candidates only; no manifolds are computed.</summary>
    public void ComputePairs(List<CandidatePair> results)
    {
        ThrowIfDisposed();
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
        ThrowIfDisposed();
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
        ThrowIfDisposed();
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
        ThrowIfDisposed();
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
        ThrowIfDisposed();
        if ((uint)entityId > ArcHandle.MaxEntityId)
            throw new ArgumentOutOfRangeException(nameof(entityId), entityId,
                $"Entity id must be between 0 and {ArcHandle.MaxEntityId}.");

        int index = AllocateSlot();
        ref Slot slot = ref _slots[index];
        slot.Shape = shape;
        slot.Bounds = new BpBounds(shape);
        slot.EntityId = entityId;
        slot.TreeProxy = -1;
        slot.Active = true;
        slot.Static = isStatic;
        slot.Generation = AllocateHandleGeneration(_worldId);

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
        ThrowIfDisposed();
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
        return new ArcHandle(index, slot.Generation, _worldId, slot.EntityId);
    }

    public void Dispose()
    {
        if (Interlocked.Exchange(ref _disposed, 1) != 0) return;
        _broadphase.Clear();
        _candidates.Clear();
        _broadphasePairs.Clear();
        Array.Clear(_slots, 0, _slotCount);
        _slotCount = 0;
        _freeList = -1;
        _activeCount = 0;
        _dynamicCount = 0;
        ReleaseWorldId(this, _worldId);
    }

    private static uint AllocateWorldId(ArcWorld owner)
    {
        lock (s_worldIdGate)
        {
            for (uint id = 1; id <= MaxWorldCount; id++)
            {
                WeakReference<ArcWorld>? slot = s_worldOwners[id];
                if (slot != null && slot.TryGetTarget(out ArcWorld? existing)
                    && Volatile.Read(ref existing._disposed) == 0)
                    continue;

                s_worldOwners[id] = new WeakReference<ArcWorld>(owner);
                return id;
            }
        }
        throw new InvalidOperationException(
            $"At most {MaxWorldCount} ArcWorld instances may be alive at once.");
    }

    private static void ReleaseWorldId(ArcWorld owner, uint worldId)
    {
        lock (s_worldIdGate)
        {
            WeakReference<ArcWorld>? slot = s_worldOwners[worldId];
            if (slot != null && slot.TryGetTarget(out ArcWorld? existing)
                && ReferenceEquals(existing, owner))
                s_worldOwners[worldId] = null;
        }
    }

    private static uint AllocateHandleGeneration(uint worldId)
    {
        uint generation;
        do
        {
            generation = unchecked((uint)Interlocked.Increment(
                ref s_nextHandleGenerations[worldId]));
        } while (generation == 0);
        return generation;
    }

    private void ThrowIfDisposed()
    {
        ObjectDisposedException.ThrowIf(Volatile.Read(ref _disposed) != 0, this);
    }

    private void AdvanceRevision()
    {
        _revision++;
        if (_revision == 0) _revision = 1;
    }

}
