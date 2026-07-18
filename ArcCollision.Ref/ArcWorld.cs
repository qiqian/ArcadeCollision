using System;
using System.Collections.Generic;
using System.Threading;

namespace ArcCollision.Ref;

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

    /// <summary>
    /// A stable, order-independent 64-bit identity for the pair of colliders
    /// <paramref name="a"/> and <paramref name="b"/>, built from their slot index
    /// and generation. It is the same every frame as long as both colliders remain
    /// in the world -- moving them (UpdateTransform) does not change it -- so a
    /// caller can tell that a contact this frame is the same pair as last frame.
    /// Removing and re-adding a collider yields a new identity. Two colliders that
    /// share an EntityId still get distinct ids (identity is per collider, not per
    /// entity).
    /// </summary>
    public static ulong PairId(ArcHandle a, ArcHandle b)
    {
        uint keyA = (uint)a._packedIndex;
        uint keyB = (uint)b._packedIndex;
        (uint lo, uint hi) = keyA <= keyB ? (keyA, keyB) : (keyB, keyA);
        return ((ulong)lo << 32) | hi;
    }
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

    /// <summary>Stable per-pair identity; see <see cref="ArcHandle.PairId"/>.</summary>
    public ulong Id => ArcHandle.PairId(A, B);
}

/// <summary>A candidate pair whose narrowphase produced a manifold.</summary>
public readonly struct ContactPair
{
    public readonly ArcHandle A;
    public readonly ArcHandle B;
    public readonly Manifold Manifold;
    /// <summary>
    /// How many consecutive frames this contact has been colliding, counting this
    /// one: 1 on the first frame of a contact (so <c>Frame == 1</c> means "new
    /// collision this frame"), incrementing while it persists, and starting over at
    /// 1 if the pair separates and touches again. A "frame" is one
    /// <see cref="ArcWorld.ComputePairs"/> call. It is 0 unless
    /// <see cref="ArcWorld.TrackContacts"/> is enabled on the world.
    /// </summary>
    public readonly int Frame;

    internal ContactPair(ArcHandle a, ArcHandle b, Manifold manifold, int frame)
    {
        A = a;
        B = b;
        Manifold = manifold;
        Frame = frame;
    }

    /// <summary>
    /// Stable per-pair identity, the same across frames while both colliders live,
    /// so this manifold can be matched to the same contact next frame. See
    /// <see cref="ArcHandle.PairId"/>.
    /// </summary>
    public ulong Id => ArcHandle.PairId(A, B);
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
        // The immutable local canonical form (precomputed on Add) plus the current
        // transform. Shape == Materialize(Local, Transform); the local form lets
        // a transform-only update re-place the collider without
        // re-supplying or re-deriving geometry.
        public LocalShape Local;
        public FxTransform Transform;
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

    // Opt-in per-contact frame counting (see TrackContacts). Keyed by pair id;
    // each ComputePairs call is one frame. Off by default so the common
    // ComputePairs/TryComputeContact path pays nothing.
    private long _contactTick;
    private readonly Dictionary<ulong, (long LastTick, int Frame)> _contactFrames = new();
    private readonly List<ulong> _staleContacts = new();

    /// <summary>
    /// When enabled, each contact returned by <see cref="TryComputeContact(in
    /// CandidatePair, out ContactPair, ManifoldFields)"/> carries a
    /// <see cref="ContactPair.Frame"/> count: 1 the first frame a pair collides,
    /// incrementing while it keeps colliding, restarting at 1 after it separates.
    /// A frame boundary is one <see cref="ComputePairs"/> call. Off by default;
    /// while off, <c>Frame</c> is 0 and no per-contact state is kept.
    /// </summary>
    public bool TrackContacts { get; set; }

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
        var fixedMotion = new FxVec2(
            -Fx.FromTransformPosition(originDelta.X),
            -Fx.FromTransformPosition(originDelta.Y));

        // Materialize the complete integer shift before mutating the world.
        // Besides preserving failure atomicity, this keeps origin rebasing on
        // exactly the same fixed transform path as UpdateTransform.
        var shiftedTransforms = new FxTransform[_slotCount];
        var shiftedShapes = new ShapeTransform.Result[_slotCount];
        for (int i = 0; i < _slotCount; i++)
        {
            if (!_slots[i].Active) continue;
            ref Slot slot = ref _slots[i];
            FxTransform shifted = new(
                slot.Transform.Position + fixedMotion,
                slot.Transform.Rotation,
                slot.Transform.Scale16);
            shiftedTransforms[i] = shifted;
            shiftedShapes[i] = ShapeTransform.Materialize(slot.Local, shifted);
        }

        _broadphase.Clear();

        for (int i = 0; i < _slotCount; i++)
        {
            ref Slot slot = ref _slots[i];
            if (!slot.Active) continue;
            slot.Shape = shiftedShapes[i].Shape;
            slot.Transform = shiftedTransforms[i];
            slot.Bounds = shiftedShapes[i].Bounds;
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

    /// <summary>
    /// Re-places the collider's immutable base shape at an absolute rigid transform
    /// (world position, rotation applied to the authored orientation, uniform
    /// scale) instead of re-supplying geometry. Translation-only transforms (the
    /// common case) just shift the base to the new position.
    /// </summary>
    public void UpdateTransform(ArcHandle handle, in Transform transform)
    {
        ref Slot slot = ref GetSlot(handle);
        ApplyTransform(handle, ref slot, FxTransform.From(transform));
    }

    /// <summary>
    /// Applies a transform relative to the collider's current one: the delta
    /// position is added, the delta rotation composed, and the delta scale
    /// multiplied. A delta of <see cref="Transform.Identity"/> is a no-op; a pure
    /// position delta moves the collider while preserving its orientation.
    /// </summary>
    public void UpdateTransformDelta(ArcHandle handle, in Transform delta)
    {
        ref Slot slot = ref GetSlot(handle);
        FxTransform fixedDelta = FxTransform.From(delta);
        ApplyTransform(handle, ref slot,
            FxTransform.Compose(slot.Transform, fixedDelta));
    }

    private void ApplyTransform(ArcHandle handle, ref Slot slot, in FxTransform transform)
    {
        ShapeTransform.Result world = ShapeTransform.Materialize(slot.Local, transform);
        slot.Shape = world.Shape;
        slot.Transform = transform;
        slot.Bounds = world.Bounds;
        RefreshProxy(handle, ref slot, world.Bounds);
    }

    private void RefreshProxy(ArcHandle handle, ref Slot slot, in BpBounds bounds)
    {
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
        slot.Local = default;
        slot.Transform = default;
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
        _contactFrames.Clear();
        _contactTick = 0;
        _activeCount = _enabledCount = _dynamicCount = 0;

        _freeList = _slotCount == 0 ? -1 : 0;
        for (int i = 0; i < _slotCount; i++)
        {
            ref Slot slot = ref _slots[i];
            slot.Shape = default;
            slot.Local = default;
            slot.Transform = default;
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
        results.Clear();
        if (TrackContacts) AdvanceContactFrame();
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

    /// <summary>
    /// Resolves several query shapes together, mirroring the native batch API.
    /// Results are concatenated into <paramref name="results"/>, and counts[k] is
    /// the number of handles belonging to queries[k] (its slice follows the sum of
    /// the earlier counts). This reference backend simply loops over the queries.
    /// </summary>
    /// <param name="queries">Query shapes in input order.</param>
    /// <param name="results">Cleared, then filled with every query's handles concatenated in input order.</param>
    /// <param name="counts">
    /// Cleared, then filled with one value per query. <c>counts[i]</c> is the
    /// number of handles produced by <c>queries[i]</c>; its slice in
    /// <paramref name="results"/> starts after the sum of
    /// <c>counts[0]</c> through <c>counts[i - 1]</c> (zero when <c>i == 0</c>).
    /// </param>
    public void QueryBatch(
        ReadOnlySpan<Shape> queries, List<ArcHandle> results, List<int> counts)
    {
        QueryBatchCore(queries, default, applyFilter: false, results, counts);
    }

    /// <summary>Mutually filtered batch query; result grouping matches the unfiltered overload.</summary>
    /// <param name="queries">Query shapes in input order.</param>
    /// <param name="filter">Filter applied mutually between every query and candidate collider.</param>
    /// <param name="results">Cleared, then filled with every query's handles concatenated in input order.</param>
    /// <param name="counts">
    /// Cleared, then filled with one value per query. <c>counts[i]</c> is the
    /// number of handles produced by <c>queries[i]</c>; its slice in
    /// <paramref name="results"/> starts after the sum of
    /// <c>counts[0]</c> through <c>counts[i - 1]</c> (zero when <c>i == 0</c>).
    /// </param>
    public void QueryBatch(
        ReadOnlySpan<Shape> queries,
        in CollisionFilter filter,
        List<ArcHandle> results,
        List<int> counts)
    {
        QueryBatchCore(queries, filter, applyFilter: true, results, counts);
    }

    private readonly List<ArcHandle> _queryBatchScratch = new();

    private void QueryBatchCore(
        ReadOnlySpan<Shape> queries,
        in CollisionFilter filter,
        bool applyFilter,
        List<ArcHandle> results,
        List<int> counts)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(counts);
        results.Clear();
        counts.Clear();
        for (int i = 0; i < queries.Length; i++)
        {
            QueryCore(queries[i], filter, applyFilter, _queryBatchScratch);
            results.AddRange(_queryBatchScratch);
            counts.Add(_queryBatchScratch.Count);
        }
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
    /// Unrequested manifold fields are zero.
    /// </summary>
    public bool TryComputeContact(
        in CandidatePair pair,
        out ContactPair contact,
        ManifoldFields fields = ManifoldFields.All)
    {
        ValidateManifoldFields(fields);
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
            _slots[pair.A.Index].Shape, _slots[pair.B.Index].Shape, fields);
        if (!manifold.Colliding)
        {
            contact = default;
            return false;
        }

        int frame = TrackContacts ? RecordContactFrame(ArcHandle.PairId(pair.A, pair.B)) : 0;
        contact = new ContactPair(pair.A, pair.B, manifold, frame);
        return true;
    }

    // Begin a new contact-tracking frame: drop contacts that were not resolved in
    // the frame that just ended, then advance the tick.
    private void AdvanceContactFrame()
    {
        if (_contactFrames.Count != 0)
        {
            _staleContacts.Clear();
            foreach (KeyValuePair<ulong, (long LastTick, int Frame)> entry in _contactFrames)
                if (entry.Value.LastTick < _contactTick)
                    _staleContacts.Add(entry.Key);
            for (int i = 0; i < _staleContacts.Count; i++)
                _contactFrames.Remove(_staleContacts[i]);
        }
        _contactTick++;
    }

    // Frame count for a pair colliding this frame: continue the count if it also
    // collided last frame, otherwise start a new contact at 1.
    private int RecordContactFrame(ulong id)
    {
        int frame = 1;
        if (_contactFrames.TryGetValue(id, out (long LastTick, int Frame) record))
        {
            if (record.LastTick == _contactTick) frame = record.Frame;
            else if (record.LastTick == _contactTick - 1) frame = record.Frame + 1;
        }
        _contactFrames[id] = (_contactTick, frame);
        return frame;
    }

    /// <summary>
    /// Computes a transient query shape against one selected handle.
    /// Unrequested manifold fields are zero.
    /// </summary>
    public bool TryComputeContact(
        in Shape query,
        ArcHandle target,
        out Manifold manifold,
        ManifoldFields fields = ManifoldFields.All)
    {
        ValidateManifoldFields(fields);
        ThrowIfDisposed();
        if (!IsValid(target) || !_slots[target.Index].Enabled)
        {
            manifold = Manifold.None;
            return false;
        }
        manifold = Collide.ShapeVsShape(
            query, _slots[target.Index].Shape, fields);
        return manifold.Colliding;
    }

    /// <summary>
    /// Computes a transient query contact only when its filter and the target's
    /// current filter mutually accept one another. Unrequested manifold fields
    /// are zero.
    /// </summary>
    public bool TryComputeContact(
        in Shape query,
        in CollisionFilter filter,
        ArcHandle target,
        out Manifold manifold,
        ManifoldFields fields = ManifoldFields.All)
    {
        ValidateManifoldFields(fields);
        ThrowIfDisposed();
        if (!IsValid(target)
            || !_slots[target.Index].Enabled
            || !filter.CanCollideWith(_slots[target.Index].Filter))
        {
            manifold = Manifold.None;
            return false;
        }
        manifold = Collide.ShapeVsShape(
            query, _slots[target.Index].Shape, fields);
        return manifold.Colliding;
    }

    private static void ValidateManifoldFields(ManifoldFields fields)
    {
        if (fields is < ManifoldFields.None or > ManifoldFields.All)
            throw new ArgumentOutOfRangeException(nameof(fields));
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
        slot.Local = LocalShape.From(shape, out FxTransform initial);
        slot.Transform = initial;
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
