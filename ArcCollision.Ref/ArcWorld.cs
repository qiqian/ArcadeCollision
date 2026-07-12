using System;
using System.Collections.Generic;
using System.Threading;

namespace ArcCollision;

/// <summary>
/// Lightweight collider token. Its first 32 bits pack a 20-bit slot index and
/// 12-bit generation. Generation cycles through 1..4095 and then wraps to 1,
/// so callers must not retain handles across 4095 reuses of the same slot.
/// Index values are limited to [0, <see cref="MaxIndex"/>].
/// EntityId is caller-owned metadata in [0, <see cref="MaxEntityId"/>]; its low
/// 28 bits share storage with the handle's internal 4-bit world id.
/// </summary>
public readonly struct ArcHandle : IEquatable<ArcHandle>
{
    public const int IndexBits = 20;
    public const int GenerationBits = 12;
    public const int MaxIndex = (1 << IndexBits) - 1;
    public const int MaxGeneration = (1 << GenerationBits) - 1;
    internal const int EntityIdBits = 28;
    public const int MaxEntityId = (1 << EntityIdBits) - 1;
    private const int EntityIdMask = MaxEntityId;

    private readonly int _packedIndex;
    private readonly int _packedEntityId;

    internal int Index => _packedIndex & MaxIndex;
    internal uint Generation => (uint)_packedIndex >> IndexBits;
    public int EntityId => _packedEntityId & EntityIdMask;
    internal uint WorldId => (uint)_packedEntityId >> EntityIdBits;

    internal ArcHandle(int index, uint generation, uint worldId, int entityId)
    {
        if ((uint)index > MaxIndex || generation is 0 or > MaxGeneration
            || worldId is 0 or > ArcWorld.MaxWorldCount
            || (uint)entityId > MaxEntityId)
            throw new ArgumentOutOfRangeException();
        _packedIndex = (int)(generation << IndexBits) | index;
        _packedEntityId = unchecked((int)(worldId << EntityIdBits)) | entityId;
    }

    public bool Equals(ArcHandle other) =>
        _packedIndex == other._packedIndex && WorldId == other.WorldId;

    public override bool Equals(object? obj) => obj is ArcHandle other && Equals(other);
    public override int GetHashCode() =>
        DeterministicHash.Combine(Index, Generation, WorldId);
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
/// Each world supports at most <see cref="MaxColliderCount"/> slot indices.
/// </summary>
public sealed class ArcWorld : IDisposable
{
    public const int MaxWorldCount = 15;
    public const int MaxColliderCount = ArcHandle.MaxIndex + 1;
    private static readonly object s_worldIdGate = new();
    private static readonly WeakReference<ArcWorld>?[] s_worldOwners =
        new WeakReference<ArcWorld>?[MaxWorldCount + 1];
    private static readonly ushort[][] s_handleGenerations =
        new ushort[MaxWorldCount + 1][];

    private sealed class HandleComparer : IComparer<ArcHandle>
    {
        public static readonly HandleComparer Instance = new();

        public int Compare(ArcHandle x, ArcHandle y)
        {
            int comparison = x.EntityId.CompareTo(y.EntityId);
            if (comparison != 0) return comparison;
            comparison = x.Index.CompareTo(y.Index);
            if (comparison != 0) return comparison;
            return x.Generation.CompareTo(y.Generation);
        }
    }

    private sealed class CandidateComparer : IComparer<CandidatePair>
    {
        public static readonly CandidateComparer Instance = new();

        public int Compare(CandidatePair x, CandidatePair y)
        {
            int comparison = HandleComparer.Instance.Compare(x.A, y.A);
            return comparison != 0
                ? comparison
                : HandleComparer.Instance.Compare(x.B, y.B);
        }
    }

    private struct Slot
    {
        public Shape Shape;
        public BpBounds Bounds;
        public CollisionFilter Filter;
        public int EntityId;
        public int TreeProxy;
        public int NextFree;
        public ushort Generation;
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
    private ushort[] _generationTable;

    public ArcWorld(float fatMargin = 16f)
    {
        _broadphase = new SpatialHash(fatMargin);
        _worldId = AllocateWorldId(this);
        _generationTable = s_handleGenerations[_worldId] ?? Array.Empty<ushort>();
    }

    public int Count { get { ThrowIfDisposed(); return _activeCount; } }
    public int DynamicCount { get { ThrowIfDisposed(); return _dynamicCount; } }
    public int StaticCount { get { ThrowIfDisposed(); return _activeCount - _dynamicCount; } }
    public float FatMargin { get { ThrowIfDisposed(); return _broadphase.FatMargin; } }

    public ArcHandle Add(int entityId, in Shape shape) =>
        AddCore(entityId, shape, CollisionFilter.Default, isStatic: false);

    public ArcHandle Add(int entityId, in Shape shape, in CollisionFilter filter) =>
        AddCore(entityId, shape, filter, isStatic: false);

    public ArcHandle AddStatic(int entityId, in Shape shape) =>
        AddCore(entityId, shape, CollisionFilter.Default, isStatic: true);

    public ArcHandle AddStatic(
        int entityId, in Shape shape, in CollisionFilter filter) =>
        AddCore(entityId, shape, filter, isStatic: true);

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

    public CollisionFilter GetFilter(ArcHandle handle) => GetSlot(handle).Filter;

    /// <summary>
    /// Changes the collider's category membership and accepted categories.
    /// Previously collected candidate pairs become stale when the value changes.
    /// </summary>
    public void SetFilter(ArcHandle handle, in CollisionFilter filter)
    {
        ref Slot slot = ref GetSlot(handle);
        if (slot.Filter == filter) return;
        slot.Filter = filter;
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
        slot.Filter = default;
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
            slot.Filter = default;
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
                && _slots[a].Filter.Allows(_slots[b].Filter)
                && _slots[a].Bounds.Overlaps(_slots[b].Bounds))
                results.Add(CreatePair(a, b));
        }
        results.Sort(CandidateComparer.Instance);
    }

    /// <summary>Returns broadphase handles overlapping a transient query shape.</summary>
    public void Query(in Shape query, List<ArcHandle> results)
    {
        QueryCore(query, default, applyFilter: false, results);
    }

    /// <summary>
    /// Returns broadphase handles overlapping a transient query shape whose
    /// collision filter mutually accepts the target collider's filter.
    /// </summary>
    public void Query(
        in Shape query, in CollisionFilter filter, List<ArcHandle> results)
    {
        QueryCore(query, filter, applyFilter: true, results);
    }

    private void QueryCore(
        in Shape query,
        in CollisionFilter filter,
        bool applyFilter,
        List<ArcHandle> results)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(results);
        results.Clear();
        BpBounds bounds = new(query);

        _candidates.Clear();
        _broadphase.QueryDynamic(bounds, _candidates);
        AppendQueryResults(bounds, filter, applyFilter, results);
        _candidates.Clear();
        _broadphase.QueryStatic(bounds, _candidates);
        AppendQueryResults(bounds, filter, applyFilter, results);
        results.Sort(HandleComparer.Instance);
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

        if (!_slots[pair.A.Index].Filter.Allows(_slots[pair.B.Index].Filter))
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

    private ArcHandle AddCore(
        int entityId,
        in Shape shape,
        in CollisionFilter filter,
        bool isStatic)
    {
        ThrowIfDisposed();
        if ((uint)entityId > ArcHandle.MaxEntityId)
            throw new ArgumentOutOfRangeException(nameof(entityId), entityId,
                $"Entity id must be between 0 and {ArcHandle.MaxEntityId}.");

        int index = AllocateSlot();
        ref Slot slot = ref _slots[index];
        slot.Shape = shape;
        slot.Bounds = new BpBounds(shape);
        slot.Filter = filter;
        slot.EntityId = entityId;
        slot.TreeProxy = -1;
        slot.Active = true;
        slot.Static = isStatic;
        slot.Generation = AllocateHandleGeneration(index);

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

        if (_slotCount >= MaxColliderCount)
            throw new InvalidOperationException(
                $"ArcWorld supports at most {MaxColliderCount} collider slots.");

        EnsureSlotCapacity(_slotCount);
        return _slotCount++;
    }

    private ref Slot GetSlot(ArcHandle handle)
    {
        ThrowIfDisposed();
        if (!IsValid(handle))
            throw new ArgumentException("Handle is stale or does not belong to this world.", nameof(handle));
        return ref _slots[handle.Index];
    }

    private void AppendQueryResults(
        BpBounds bounds,
        in CollisionFilter filter,
        bool applyFilter,
        List<ArcHandle> results)
    {
        for (int i = 0; i < _candidates.Count; i++)
        {
            int index = _candidates[i];
            if (_slots[index].Active
                && (!applyFilter || filter.Allows(_slots[index].Filter))
                && _slots[index].Bounds.Overlaps(bounds))
                results.Add(CreateHandle(index));
        }
    }

    private CandidatePair CreatePair(int a, int b)
    {
        ArcHandle first = CreateHandle(a);
        ArcHandle second = CreateHandle(b);
        return HandleComparer.Instance.Compare(first, second) <= 0
            ? new CandidatePair(first, second, _revision)
            : new CandidatePair(second, first, _revision);
    }

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

    private ushort AllocateHandleGeneration(int index)
    {
        EnsureGenerationCapacity(index);
        ushort generation = _generationTable[index];
        generation++;
        if (generation > ArcHandle.MaxGeneration) generation = 1;
        _generationTable[index] = generation;
        return generation;
    }

    private void EnsureGenerationCapacity(int index)
    {
        if (index < _generationTable.Length) return;
        int newLength = Math.Max(16, _generationTable.Length);
        while (newLength <= index)
        {
            newLength = Math.Min(MaxColliderCount, newLength * 2);
            if (newLength <= index && newLength == MaxColliderCount) break;
        }
        Array.Resize(ref _generationTable, newLength);
        s_handleGenerations[_worldId] = _generationTable;
    }

    private void EnsureSlotCapacity(int index)
    {
        if (index < _slots.Length) return;
        int newLength = _slots.Length;
        while (newLength <= index)
            newLength = Math.Min(MaxColliderCount, newLength * 2);
        Array.Resize(ref _slots, newLength);
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
