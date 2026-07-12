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

    internal CandidatePair(ArcHandle a, ArcHandle b)
    {
        A = a;
        B = b;
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

/// <summary>A world collider hit by a ray or translating shape cast.</summary>
public readonly struct WorldCastHit
{
    public readonly ArcHandle Handle;
    public readonly SweepHit Hit;

    internal WorldCastHit(ArcHandle handle, SweepHit hit)
    {
        Handle = handle;
        Hit = hit;
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

    private sealed class CastHitComparer : IComparer<WorldCastHit>
    {
        public static readonly CastHitComparer Instance = new();

        public int Compare(WorldCastHit x, WorldCastHit y)
        {
            int comparison = x.Hit.Time.CompareTo(y.Hit.Time);
            return comparison != 0
                ? comparison
                : HandleComparer.Instance.Compare(x.Handle, y.Handle);
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
        public bool Enabled;
        public bool Static;
    }

    private readonly SpatialHash _broadphase;
    private readonly uint _worldId;
    private readonly List<int> _candidates;
    private readonly List<(int A, int B)> _broadphasePairs;
    private Slot[] _slots;
    private int _slotCount;
    private int _freeList = -1;
    private int _activeCount;
    private int _enabledCount;
    private int _dynamicCount;
    private int _disposed;
    private ushort[] _generationTable;

    public ArcWorld(float fatMargin = 16f)
        : this(new ArcWorldOptions(fatMargin))
    {
    }

    public ArcWorld(in ArcWorldOptions options)
    {
        int colliderCapacity = Math.Max(16, options.InitialColliderCapacity);
        _slots = new Slot[colliderCapacity];
        _candidates = new List<int>(colliderCapacity);
        _broadphasePairs = new List<(int A, int B)>(options.InitialPairCapacity);
        _broadphase = new SpatialHash(options.FatMargin);
        _broadphase.EnsureCapacity(colliderCapacity);
        _worldId = AllocateWorldId(this);
        _generationTable = s_handleGenerations[_worldId] ?? Array.Empty<ushort>();
        if (colliderCapacity != 0) EnsureGenerationCapacity(colliderCapacity - 1);
    }

    /// <summary>Total valid colliders, including disabled colliders.</summary>
    public int Count { get { ThrowIfDisposed(); return _activeCount; } }
    /// <summary>Colliders currently participating in broadphase queries.</summary>
    public int EnabledCount { get { ThrowIfDisposed(); return _enabledCount; } }
    /// <summary>Total valid dynamic colliders, including disabled colliders.</summary>
    public int DynamicCount { get { ThrowIfDisposed(); return _dynamicCount; } }
    /// <summary>Total valid static colliders, including disabled colliders.</summary>
    public int StaticCount { get { ThrowIfDisposed(); return _activeCount - _dynamicCount; } }
    public float FatMargin { get { ThrowIfDisposed(); return _broadphase.FatMargin; } }

    public ArcHandle Add(int entityId, in Shape shape) =>
        AddCore(entityId, shape, CollisionFilter.Default, isStatic: false, enabled: true);

    public ArcHandle Add(int entityId, in Shape shape, in CollisionFilter filter) =>
        AddCore(entityId, shape, filter, isStatic: false, enabled: true);

    public ArcHandle AddStatic(int entityId, in Shape shape) =>
        AddCore(entityId, shape, CollisionFilter.Default, isStatic: true, enabled: true);

    public ArcHandle AddStatic(
        int entityId, in Shape shape, in CollisionFilter filter) =>
        AddCore(entityId, shape, filter, isStatic: true, enabled: true);

    public ArcHandle Add(
        int entityId, in Shape shape, in CollisionFilter filter, bool enabled) =>
        AddCore(entityId, shape, filter, isStatic: false, enabled);

    public ArcHandle AddStatic(
        int entityId, in Shape shape, in CollisionFilter filter, bool enabled) =>
        AddCore(entityId, shape, filter, isStatic: true, enabled);

    /// <summary>Preallocates storage used by colliders and candidate pairs.</summary>
    public void EnsureCapacity(int colliderCapacity, int pairCapacity = 0)
    {
        ThrowIfDisposed();
        if (colliderCapacity is < 0 or > MaxColliderCount)
            throw new ArgumentOutOfRangeException(nameof(colliderCapacity));
        if (pairCapacity < 0)
            throw new ArgumentOutOfRangeException(nameof(pairCapacity));
        if (_slots.Length < colliderCapacity)
            Array.Resize(ref _slots, colliderCapacity);
        if (colliderCapacity != 0)
            EnsureGenerationCapacity(colliderCapacity - 1);
        _candidates.EnsureCapacity(colliderCapacity);
        _broadphasePairs.EnsureCapacity(pairCapacity);
        _broadphase.EnsureCapacity(colliderCapacity);
    }

    /// <summary>
    /// Subtracts <paramref name="originDelta"/> from every collider while
    /// retaining handles, filters and enabled state. Useful for origin rebasing.
    /// </summary>
    public void ShiftOrigin(Vec2 originDelta)
    {
        ThrowIfDisposed();
        _ = Fx.From(originDelta.X);
        _ = Fx.From(originDelta.Y);
        Vec2 motion = -originDelta;

        // Validate the complete shift before mutating the world.
        for (int i = 0; i < _slotCount; i++)
        {
            if (!_slots[i].Active) continue;
            Shape moved = _slots[i].Shape.Moved(motion);
            _ = new BpBounds(moved);
        }

        _broadphase.Clear();

        for (int i = 0; i < _slotCount; i++)
        {
            ref Slot slot = ref _slots[i];
            if (!slot.Active) continue;
            slot.Shape = slot.Shape.Moved(motion);
            slot.Bounds = new BpBounds(slot.Shape);
            slot.TreeProxy = -1;
            if (!slot.Enabled) continue;
            if (slot.Static)
                _broadphase.AddOrUpdateStatic(i, slot.Bounds);
            else
                slot.TreeProxy = _broadphase.AddDynamic(i, slot.Bounds);
        }
        _broadphase.BuildStatic();
    }

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
        if (!slot.Enabled)
            return;
        if (slot.Static)
        {
            _broadphase.AddOrUpdateStatic(handle.Index, bounds);
        }
        else
        {
            _broadphase.UpdateDynamic(slot.TreeProxy, bounds);
        }
    }

    public CollisionFilter GetFilter(ArcHandle handle) => GetSlot(handle).Filter;

    public bool TryGetFilter(ArcHandle handle, out CollisionFilter filter)
    {
        if (!IsValid(handle))
        {
            filter = default;
            return false;
        }
        filter = _slots[handle.Index].Filter;
        return true;
    }

    public Shape GetShape(ArcHandle handle) => GetSlot(handle).Shape;

    public bool TryGetShape(ArcHandle handle, out Shape shape)
    {
        if (!IsValid(handle))
        {
            shape = default;
            return false;
        }
        shape = _slots[handle.Index].Shape;
        return true;
    }

    public int GetEntityId(ArcHandle handle) => GetSlot(handle).EntityId;

    /// <summary>
    /// Changes the collider's category membership and accepted categories.
    /// Previously collected candidates are rechecked against the new value.
    /// </summary>
    public void SetFilter(ArcHandle handle, in CollisionFilter filter)
    {
        ref Slot slot = ref GetSlot(handle);
        if (slot.Filter == filter) return;
        slot.Filter = filter;
    }

    public bool IsEnabled(ArcHandle handle) => GetSlot(handle).Enabled;

    /// <summary>
    /// Enables or disables broadphase participation without invalidating the
    /// handle. Disabled colliders retain their shape, filter and static status.
    /// </summary>
    public void SetEnabled(ArcHandle handle, bool enabled)
    {
        ref Slot slot = ref GetSlot(handle);
        if (slot.Enabled == enabled) return;

        if (enabled)
        {
            if (slot.Static)
                _broadphase.AddOrUpdateStatic(handle.Index, slot.Bounds);
            else
                slot.TreeProxy = _broadphase.AddDynamic(handle.Index, slot.Bounds);
        }
        else if (slot.Static)
        {
            _broadphase.RemoveStatic(handle.Index);
        }
        else
        {
            _broadphase.RemoveDynamic(slot.TreeProxy);
            slot.TreeProxy = -1;
        }
        slot.Enabled = enabled;
        _enabledCount += enabled ? 1 : -1;
    }

    public void Remove(ArcHandle handle)
    {
        ref Slot slot = ref GetSlot(handle);
        if (slot.Enabled && slot.Static)
        {
            _broadphase.RemoveStatic(handle.Index);
        }
        else if (slot.Enabled)
        {
            _broadphase.RemoveDynamic(slot.TreeProxy);
        }
        if (!slot.Static) _dynamicCount--;
        if (slot.Enabled) _enabledCount--;

        slot.Shape = default;
        slot.Bounds = default;
        slot.Filter = default;
        slot.EntityId = 0;
        slot.TreeProxy = -1;
        slot.Active = false;
        slot.Enabled = false;
        slot.Static = false;
        slot.Generation = 0;
        slot.NextFree = _freeList;
        _freeList = handle.Index;
        _activeCount--;
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
        _activeCount = _enabledCount = _dynamicCount = 0;

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
            slot.Enabled = false;
            slot.Static = false;
            slot.Generation = 0;
            slot.NextFree = i + 1 < _slotCount ? i + 1 : -1;
        }
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
                && _slots[a].Enabled && _slots[b].Enabled
                && _slots[a].Filter.CanCollideWith(_slots[b].Filter)
                && _slots[a].Bounds.Overlaps(_slots[b].Bounds))
                results.Add(CreatePair(a, b));
        }
        InPlaceSort.Sort(results, CandidateComparer.Instance);
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

    /// <summary>Returns the earliest unfiltered hit along a translation.</summary>
    public bool ShapeCast(
        in Shape mover, Vec2 motion, out WorldCastHit closest) =>
        ShapeCastCore(mover, motion, default, applyFilter: false, out closest);

    /// <summary>Returns the earliest mutually filtered hit along a translation.</summary>
    public bool ShapeCast(
        in Shape mover,
        Vec2 motion,
        in CollisionFilter filter,
        out WorldCastHit closest) =>
        ShapeCastCore(mover, motion, filter, applyFilter: true, out closest);

    /// <summary>Collects every unfiltered hit, sorted by time then handle.</summary>
    public void ShapeCastAll(
        in Shape mover, Vec2 motion, List<WorldCastHit> results) =>
        ShapeCastAllCore(mover, motion, default, applyFilter: false, results);

    /// <summary>Collects every mutually filtered hit, sorted by time then handle.</summary>
    public void ShapeCastAll(
        in Shape mover,
        Vec2 motion,
        in CollisionFilter filter,
        List<WorldCastHit> results) =>
        ShapeCastAllCore(mover, motion, filter, applyFilter: true, results);

    public bool RayCast(Vec2 origin, Vec2 motion, out WorldCastHit closest)
    {
        Shape point = new Circle(origin, 0f);
        return ShapeCast(point, motion, out closest);
    }

    public bool RayCast(
        Vec2 origin,
        Vec2 motion,
        in CollisionFilter filter,
        out WorldCastHit closest)
    {
        Shape point = new Circle(origin, 0f);
        return ShapeCast(point, motion, filter, out closest);
    }

    public void RayCastAll(
        Vec2 origin, Vec2 motion, List<WorldCastHit> results)
    {
        Shape point = new Circle(origin, 0f);
        ShapeCastAll(point, motion, results);
    }

    public void RayCastAll(
        Vec2 origin,
        Vec2 motion,
        in CollisionFilter filter,
        List<WorldCastHit> results)
    {
        Shape point = new Circle(origin, 0f);
        ShapeCastAll(point, motion, filter, results);
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
        InPlaceSort.Sort(results, HandleComparer.Instance);
    }

    private bool ShapeCastCore(
        in Shape mover,
        Vec2 motion,
        in CollisionFilter filter,
        bool applyFilter,
        out WorldCastHit closest)
    {
        ThrowIfDisposed();
        BpBounds bounds = SweptBounds(mover, motion);
        bool found = false;
        closest = default;

        _candidates.Clear();
        _broadphase.QueryDynamic(bounds, _candidates);
        FindClosestCast(mover, motion, filter, applyFilter, ref found, ref closest);
        _candidates.Clear();
        _broadphase.QueryStatic(bounds, _candidates);
        FindClosestCast(mover, motion, filter, applyFilter, ref found, ref closest);
        return found;
    }

    private void ShapeCastAllCore(
        in Shape mover,
        Vec2 motion,
        in CollisionFilter filter,
        bool applyFilter,
        List<WorldCastHit> results)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(results);
        results.Clear();
        BpBounds bounds = SweptBounds(mover, motion);

        _candidates.Clear();
        _broadphase.QueryDynamic(bounds, _candidates);
        AppendCastHits(mover, motion, filter, applyFilter, results);
        _candidates.Clear();
        _broadphase.QueryStatic(bounds, _candidates);
        AppendCastHits(mover, motion, filter, applyFilter, results);
        InPlaceSort.Sort(results, CastHitComparer.Instance);
    }

    private static BpBounds SweptBounds(in Shape mover, Vec2 motion)
    {
        BpBounds start = new(mover);
        BpBounds end = start.Translated(Fx.From(motion.X), Fx.From(motion.Y));
        return BpBounds.Union(start, end);
    }

    private void FindClosestCast(
        in Shape mover,
        Vec2 motion,
        in CollisionFilter filter,
        bool applyFilter,
        ref bool found,
        ref WorldCastHit closest)
    {
        for (int i = 0; i < _candidates.Count; i++)
        {
            int index = _candidates[i];
            if (!CanQuerySlot(index, filter, applyFilter)) continue;
            SweepHit hit = Sweep.MovingShapeVsShape(mover, motion, _slots[index].Shape);
            if (!hit.Hit) continue;
            var candidate = new WorldCastHit(CreateHandle(index), hit);
            if (!found || CastHitComparer.Instance.Compare(candidate, closest) < 0)
            {
                closest = candidate;
                found = true;
            }
        }
    }

    private void AppendCastHits(
        in Shape mover,
        Vec2 motion,
        in CollisionFilter filter,
        bool applyFilter,
        List<WorldCastHit> results)
    {
        for (int i = 0; i < _candidates.Count; i++)
        {
            int index = _candidates[i];
            if (!CanQuerySlot(index, filter, applyFilter)) continue;
            SweepHit hit = Sweep.MovingShapeVsShape(mover, motion, _slots[index].Shape);
            if (hit.Hit) results.Add(new WorldCastHit(CreateHandle(index), hit));
        }
    }

    private bool CanQuerySlot(
        int index, in CollisionFilter filter, bool applyFilter) =>
        _slots[index].Active
        && _slots[index].Enabled
        && (!applyFilter || filter.CanCollideWith(_slots[index].Filter));

    /// <summary>
    /// Computes narrowphase only for the selected candidate using current slot
    /// state. Returns false when either handle is invalid or disabled, the
    /// current filters reject one another, or the shapes no longer collide.
    /// </summary>
    public bool TryComputeContact(in CandidatePair pair, out ContactPair contact)
    {
        ThrowIfDisposed();
        if (!IsValid(pair.A) || !IsValid(pair.B)
            || !_slots[pair.A.Index].Enabled || !_slots[pair.B.Index].Enabled)
        {
            contact = default;
            return false;
        }

        if (!_slots[pair.A.Index].Filter.CanCollideWith(_slots[pair.B.Index].Filter))
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
        if (!IsValid(target) || !_slots[target.Index].Enabled)
        {
            manifold = Manifold.None;
            return false;
        }
        manifold = Collide.ShapeVsShape(query, _slots[target.Index].Shape);
        return manifold.Colliding;
    }

    /// <summary>
    /// Computes a transient query contact only when its filter and the target's
    /// current filter mutually accept one another.
    /// </summary>
    public bool TryComputeContact(
        in Shape query,
        in CollisionFilter filter,
        ArcHandle target,
        out Manifold manifold)
    {
        ThrowIfDisposed();
        if (!IsValid(target)
            || !_slots[target.Index].Enabled
            || !filter.CanCollideWith(_slots[target.Index].Filter))
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
        bool isStatic,
        bool enabled)
    {
        ThrowIfDisposed();
        if ((uint)entityId > ArcHandle.MaxEntityId)
            throw new ArgumentOutOfRangeException(nameof(entityId), entityId,
                $"Entity id must be between 0 and {ArcHandle.MaxEntityId}.");

        // Validate/quantize the complete shape before consuming a slot. Failed
        // Add calls therefore leave subsequent handle indices unchanged, matching
        // the native world's validate-before-allocate contract.
        BpBounds bounds = new(shape);
        int index = AllocateSlot();
        ref Slot slot = ref _slots[index];
        slot.Shape = shape;
        slot.Bounds = bounds;
        slot.Filter = filter;
        slot.EntityId = entityId;
        slot.TreeProxy = -1;
        slot.Active = true;
        slot.Enabled = enabled;
        slot.Static = isStatic;
        slot.Generation = AllocateHandleGeneration(index);

        if (enabled && isStatic)
        {
            _broadphase.AddOrUpdateStatic(index, slot.Bounds);
        }
        else if (enabled)
        {
            slot.TreeProxy = _broadphase.AddDynamic(index, slot.Bounds);
        }

        if (!isStatic) _dynamicCount++;

        _activeCount++;
        if (enabled) _enabledCount++;
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
                && _slots[index].Enabled
                && (!applyFilter || filter.CanCollideWith(_slots[index].Filter))
                && _slots[index].Bounds.Overlaps(bounds))
                results.Add(CreateHandle(index));
        }
    }

    private CandidatePair CreatePair(int a, int b)
    {
        ArcHandle first = CreateHandle(a);
        ArcHandle second = CreateHandle(b);
        return HandleComparer.Instance.Compare(first, second) <= 0
            ? new CandidatePair(first, second)
            : new CandidatePair(second, first);
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
        _enabledCount = 0;
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

}
