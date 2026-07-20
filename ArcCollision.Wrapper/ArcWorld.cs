using System.Runtime.InteropServices;

namespace ArcCollision.Wrapper;

[StructLayout(LayoutKind.Sequential)]
public readonly struct ArcHandle : IEquatable<ArcHandle>
{
    public const int IndexBits = 20, GenerationBits = 12;
    public const int MaxIndex = (1 << IndexBits) - 1;
    public const int MaxGeneration = (1 << GenerationBits) - 1;
    public const int MaxEntityId = (1 << 28) - 1;
    private readonly uint _packedIndex, _packedEntityId;
    internal ArcHandle(int index, uint generation, uint worldId, int entityId)
    {
        if ((uint)index > MaxIndex || generation is 0 or > MaxGeneration
            || worldId is 0 or > ArcWorld.MaxWorldCount
            || (uint)entityId > MaxEntityId)
            throw new ArgumentOutOfRangeException();
        _packedIndex = (generation << IndexBits) | (uint)index;
        _packedEntityId = (worldId << 28) | (uint)entityId;
    }
    internal int Index => (int)(_packedIndex & MaxIndex);
    internal uint Generation => _packedIndex >> IndexBits;
    internal uint WorldId => _packedEntityId >> 28;
    public int EntityId => (int)(_packedEntityId & MaxEntityId);
    public bool Equals(ArcHandle other) => _packedIndex == other._packedIndex && (_packedEntityId >> 28) == (other._packedEntityId >> 28);
    public override bool Equals(object? obj) => obj is ArcHandle other && Equals(other);
    public override int GetHashCode() => DeterministicHash.Combine(
        (int)(_packedIndex & MaxIndex), _packedIndex >> IndexBits, _packedEntityId >> 28);
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
        uint keyA = a._packedIndex;
        uint keyB = b._packedIndex;
        (uint lo, uint hi) = keyA <= keyB ? (keyA, keyB) : (keyB, keyA);
        return ((ulong)lo << 32) | hi;
    }
}

public readonly struct CandidatePair
{
    public readonly ArcHandle A, B;

    /// <summary>Stable per-pair identity; see <see cref="ArcHandle.PairId"/>.</summary>
    public ulong Id => ArcHandle.PairId(A, B);
}
public readonly struct ContactPair
{
    public readonly ArcHandle A, B;
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
    internal ContactPair(NativeContact value, int frame) { A = value.Pair.A; B = value.Pair.B; Manifold = value.Manifold; Frame = frame; }

    /// <summary>
    /// Stable per-pair identity, the same across frames while both colliders live,
    /// so this manifold can be matched to the same contact next frame. See
    /// <see cref="ArcHandle.PairId"/>.
    /// </summary>
    public ulong Id => ArcHandle.PairId(A, B);
}
public readonly struct WorldCastHit
{
    public readonly ArcHandle Handle;
    public readonly SweepHit Hit;
}

public sealed unsafe class ArcWorld : IDisposable
{
    public const int MaxWorldCount = 15;
    public const int MaxColliderCount = ArcHandle.MaxIndex + 1;
    private NativeWorldHandle? _handle;
    // QueryBatch is world-buffered and ArcWorld is not concurrently callable.
    // Keep blittable conversion scratch outside the GC heap.
    private NativeBuffer<NativeShape>? _queryBatchNative;

    public ArcWorld(float fatMargin = 16) : this(new ArcWorldOptions(fatMargin)) { }
    public ArcWorld(in ArcWorldOptions options)
    {
        _ = FixedValidation.From(options.FatMargin);
        if (NativeMethods.GetAbiVersion() != 5) throw new InvalidOperationException("ArcCollision native ABI version mismatch.");
        _handle = NativeMethods.WorldCreate(options);
        if (_handle.IsInvalid)
            NativeMethods.ThrowLastOperationError("Native world creation failed.");
    }

    private NativeWorldHandle Handle => _handle is { IsClosed: false } value ? value : throw new ObjectDisposedException(nameof(ArcWorld));
    public int Count => NativeMethods.WorldCount(Handle);
    public int EnabledCount => NativeMethods.WorldEnabledCount(Handle);
    public int DynamicCount => NativeMethods.WorldDynamicCount(Handle);
    public int StaticCount => NativeMethods.WorldStaticCount(Handle);
    public float FatMargin => NativeMethods.WorldFatMargin(Handle);

    public ArcHandle Add(int entityId, in Shape shape) => Add(entityId, shape, CollisionFilter.Default, true, false);
    public ArcHandle Add(int entityId, in Shape shape, in CollisionFilter filter) => Add(entityId, shape, filter, true, false);
    public ArcHandle Add(int entityId, in Shape shape, in CollisionFilter filter, bool enabled) => Add(entityId, shape, filter, enabled, false);
    public ArcHandle AddStatic(int entityId, in Shape shape) => Add(entityId, shape, CollisionFilter.Default, true, true);
    public ArcHandle AddStatic(int entityId, in Shape shape, in CollisionFilter filter) => Add(entityId, shape, filter, true, true);
    public ArcHandle AddStatic(int entityId, in Shape shape, in CollisionFilter filter, bool enabled) => Add(entityId, shape, filter, enabled, true);
    private ArcHandle Add(int entityId, in Shape shape, in CollisionFilter filter, bool enabled, bool isStatic)
    {
        _ = Handle;
        if ((uint)entityId > ArcHandle.MaxEntityId)
            throw new ArgumentOutOfRangeException(nameof(entityId), entityId,
                $"Entity id must be between 0 and {ArcHandle.MaxEntityId}.");
        // Validate managed-side before the native call so invalid shape values
        // throw ArgumentOutOfRangeException exactly like the Ref backend, and a
        // failed Add never reaches the native world (validate-before-allocate:
        // no handle slot is consumed). Add is a cold path, so unlike the query
        // routes this duplicate of the native check costs nothing that matters.
        FixedValidation.Shape(shape);
        NativeShape native = shape.ToNative();
        NativeMethods.Check(NativeMethods.WorldAdd(Handle, entityId, native, filter, isStatic ? 1 : 0, enabled ? 1 : 0, out ArcHandle result), nameof(entityId));
        GC.KeepAlive(shape.PolygonObject);
        return result;
    }

    public void EnsureCapacity(int colliderCapacity, int pairCapacity = 0)
    {
        _ = Handle;
        if (colliderCapacity is < 0 or > MaxColliderCount)
            throw new ArgumentOutOfRangeException(nameof(colliderCapacity));
        if (pairCapacity < 0)
            throw new ArgumentOutOfRangeException(nameof(pairCapacity));
        NativeMethods.Check(NativeMethods.WorldEnsureCapacity(Handle, colliderCapacity, pairCapacity));
    }
    public void ShiftOrigin(Vec2 originDelta)
    {
        _ = Handle;
        FixedValidation.Vec2(originDelta);
        NativeMethods.Check(NativeMethods.WorldShiftOrigin(Handle, originDelta));
    }
    public void BuildStatic() => NativeMethods.Check(NativeMethods.WorldBuildStatic(Handle));
    /// <summary>
    /// Re-places the collider's immutable base shape at an absolute rigid transform.
    /// </summary>
    public void UpdateTransform(ArcHandle handle, in Transform transform)
    {
        if (!IsValid(handle))
            throw new ArgumentException(
                "Handle is stale or does not belong to this world.", nameof(handle));
        FixedValidation.Vec2(transform.Position);
        NativeMethods.Check(
            NativeMethods.WorldUpdateTransform(Handle, handle, transform),
            nameof(transform));
    }

    /// <summary>
    /// Composes a relative transform onto the collider's current transform.
    /// </summary>
    public void UpdateTransformDelta(ArcHandle handle, in Transform delta)
    {
        if (!IsValid(handle))
            throw new ArgumentException(
                "Handle is stale or does not belong to this world.", nameof(handle));
        FixedValidation.Vec2(delta.Position);
        NativeMethods.Check(
            NativeMethods.WorldUpdateTransformDelta(Handle, handle, delta),
            nameof(delta));
    }
    public void Remove(ArcHandle handle) => NativeMethods.Check(NativeMethods.WorldRemove(Handle, handle), nameof(handle));
    public bool IsValid(ArcHandle handle) => NativeMethods.WorldIsValid(Handle, handle) != 0;
    public Shape GetShape(ArcHandle handle)
    {
        NativeMethods.Check(NativeMethods.WorldGetShape(Handle, handle, out NativeShape shape), nameof(handle));
        return shape.ToManagedOwned();
    }
    public bool TryGetShape(ArcHandle handle, out Shape shape)
    {
        if (!IsValid(handle)) { shape = default; return false; }
        shape = GetShape(handle); return true;
    }
    public CollisionFilter GetFilter(ArcHandle handle) { NativeMethods.Check(NativeMethods.WorldGetFilter(Handle, handle, out CollisionFilter filter), nameof(handle)); return filter; }
    public bool TryGetFilter(ArcHandle handle, out CollisionFilter filter)
    {
        if (!IsValid(handle)) { filter = default; return false; }
        filter = GetFilter(handle); return true;
    }
    public int GetEntityId(ArcHandle handle)
    {
        if (!IsValid(handle)) throw new ArgumentException("Handle is stale or belongs to another world.", nameof(handle));
        return handle.EntityId;
    }
    public void SetFilter(ArcHandle handle, in CollisionFilter filter) => NativeMethods.Check(NativeMethods.WorldSetFilter(Handle, handle, filter), nameof(handle));
    public bool IsEnabled(ArcHandle handle) { NativeMethods.Check(NativeMethods.WorldGetEnabled(Handle, handle, out int enabled), nameof(handle)); return enabled != 0; }
    public void SetEnabled(ArcHandle handle, bool enabled) => NativeMethods.Check(NativeMethods.WorldSetEnabled(Handle, handle, enabled ? 1 : 0), nameof(handle));
    public void Clear()
    {
        NativeMethods.Check(NativeMethods.WorldClear(Handle));
        _contactFrames.Clear();
        _contactTick = 0;
    }

    // Opt-in per-contact frame counting (see TrackContacts), tracked in managed
    // over the native narrowphase so the native ABI stays pure geometry. Keyed by
    // pair id; each ComputePairs call is one frame.
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

    public void ComputePairs(List<CandidatePair> results)
    {
        results.Clear();
        if (TrackContacts) AdvanceContactFrame();
        NativeMethods.Check(NativeMethods.WorldComputePairs(Handle, out IntPtr data, out int count));
        CopyToList(results, data, count);
    }

    // Publishes a borrowed native result array into the caller's list with one
    // bulk span copy; per-element List.Add calls dominate the managed side of
    // large result sets.
    private static void CopyToList<T>(List<T> results, IntPtr data, int count)
        where T : unmanaged
    {
        CollectionsMarshal.SetCount(results, count);
        new ReadOnlySpan<T>((void*)data, count).CopyTo(CollectionsMarshal.AsSpan(results));
    }

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

    public void Query(in Shape query, List<ArcHandle> results) => QueryCore(query, null, results);
    public void Query(in Shape query, in CollisionFilter filter, List<ArcHandle> results) { CollisionFilter copy = filter; QueryCore(query, &copy, results); }
    private void QueryCore(in Shape query, CollisionFilter* filter, List<ArcHandle> results)
    {
        _ = Handle;
        results.Clear();
        NativeShape native = query.ToNative();
        NativeMethods.Check(NativeMethods.WorldQuery(Handle, native, filter, out IntPtr data, out int count));
        CopyToList(results, data, count);
        GC.KeepAlive(query.PolygonObject);
    }

    /// <summary>Queries several shapes in one native call.</summary>
    /// <param name="queries">Query shapes in input order.</param>
    /// <param name="results">Cleared, then filled with every query's handles concatenated in input order.</param>
    /// <param name="counts">
    /// Cleared, then filled with one value per query. <c>counts[i]</c> is the
    /// number of handles produced by <c>queries[i]</c>; its slice in
    /// <paramref name="results"/> starts after the sum of
    /// <c>counts[0]</c> through <c>counts[i - 1]</c> (zero when <c>i == 0</c>).
    /// </param>
    public void QueryBatch(ReadOnlySpan<Shape> queries, List<ArcHandle> results, List<int> counts) => QueryBatchCore(queries, null, results, counts);

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
    public void QueryBatch(ReadOnlySpan<Shape> queries, in CollisionFilter filter, List<ArcHandle> results, List<int> counts) { CollisionFilter copy = filter; QueryBatchCore(queries, &copy, results, counts); }
    private void QueryBatchCore(ReadOnlySpan<Shape> queries, CollisionFilter* filter, List<ArcHandle> results, List<int> counts)
    {
        _ = Handle;
        results.Clear();
        counts.Clear();
        int n = queries.Length;
        if (n == 0) return;
        NativeShape* native = (_queryBatchNative ??= new()).EnsureCapacity(n);
        bool hasPolygons = false;
        for (int i = 0; i < n; i++)
        {
            native[i] = queries[i].ToNative();
            hasPolygons |= queries[i].Kind == ShapeKind.Polygon;
        }
        NativeMethods.Check(NativeMethods.WorldQueryBatch(
            Handle, native, n, filter,
            out IntPtr handleData, out IntPtr countData, out int total));
        CopyToList(results, handleData, total);
        CopyToList(counts, countData, n);
        // Primitive-only batches contain no managed objects referenced solely by
        // the native scratch array, so avoid another full O(N) scan. Polygon
        // batches keep every geometry handle owner alive through the C call.
        if (hasPolygons)
            for (int i = 0; i < n; i++)
                GC.KeepAlive(queries[i].PolygonObject);
    }

    public bool TryComputeContact(
        in CandidatePair pair,
        out ContactPair contact,
        ManifoldFields fields = ManifoldFields.All)
    {
        NativeStatus status = NativeMethods.WorldContactPair(
            Handle, pair, fields, out NativeContact native, out int colliding);
        if (status == NativeStatus.InvalidHandle) { contact = default; return false; }
        NativeMethods.Check(status, nameof(fields));
        if (colliding == 0) { contact = default; return false; }
        int frame = TrackContacts ? RecordContactFrame(ArcHandle.PairId(pair.A, pair.B)) : 0;
        contact = new ContactPair(native, frame);
        return true;
    }
    public bool TryComputeContact(
        in Shape query, ArcHandle target, out Manifold manifold,
        ManifoldFields fields = ManifoldFields.All) =>
        ContactShape(query, null, target, out manifold, fields);
    public bool TryComputeContact(
        in Shape query, in CollisionFilter filter, ArcHandle target,
        out Manifold manifold, ManifoldFields fields = ManifoldFields.All)
    {
        CollisionFilter copy = filter;
        return ContactShape(query, &copy, target, out manifold, fields);
    }
    private bool ContactShape(
        in Shape query, CollisionFilter* filter, ArcHandle target,
        out Manifold manifold, ManifoldFields fields)
    {
        _ = Handle;
        NativeShape native = query.ToNative();
        NativeStatus status = NativeMethods.WorldContactShape(
            Handle, native, filter, target, fields,
            out manifold, out int colliding);
        GC.KeepAlive(query.PolygonObject);
        if (status == NativeStatus.InvalidHandle)
        {
            manifold = Manifold.None;
            return false;
        }
        NativeMethods.Check(status, nameof(fields));
        return colliding != 0;
    }

    public bool ShapeCast(in Shape mover, Vec2 motion, out WorldCastHit closest) => ShapeCastCore(mover, motion, null, out closest);
    public bool ShapeCast(in Shape mover, Vec2 motion, in CollisionFilter filter, out WorldCastHit closest) { CollisionFilter copy = filter; return ShapeCastCore(mover, motion, &copy, out closest); }
    private bool ShapeCastCore(in Shape mover, Vec2 motion, CollisionFilter* filter, out WorldCastHit closest)
    {
        _ = Handle;
        FixedValidation.Vec2(motion);
        NativeShape native = mover.ToNative(); NativeMethods.Check(NativeMethods.WorldShapeCast(Handle, native, motion, filter, out WorldCastHit hit, out int found));
        GC.KeepAlive(mover.PolygonObject); closest = found != 0 ? hit : default; return found != 0;
    }
    public void ShapeCastAll(in Shape mover, Vec2 motion, List<WorldCastHit> results) => ShapeCastAllCore(mover, motion, null, results);
    public void ShapeCastAll(in Shape mover, Vec2 motion, in CollisionFilter filter, List<WorldCastHit> results) { CollisionFilter copy = filter; ShapeCastAllCore(mover, motion, &copy, results); }
    private void ShapeCastAllCore(in Shape mover, Vec2 motion, CollisionFilter* filter, List<WorldCastHit> results)
    {
        _ = Handle;
        results.Clear();
        FixedValidation.Vec2(motion);
        NativeShape native = mover.ToNative();
        NativeMethods.Check(NativeMethods.WorldShapeCastAll(Handle, native, motion, filter, out IntPtr data, out int count));
        CopyToList(results, data, count);
        GC.KeepAlive(mover.PolygonObject);
    }
    public bool RayCast(Vec2 origin, Vec2 motion, out WorldCastHit closest) => RayCastCore(origin, motion, null, out closest);
    public bool RayCast(Vec2 origin, Vec2 motion, in CollisionFilter filter, out WorldCastHit closest) { CollisionFilter copy = filter; return RayCastCore(origin, motion, &copy, out closest); }
    private bool RayCastCore(Vec2 origin, Vec2 motion, CollisionFilter* filter, out WorldCastHit closest)
    {
        _ = Handle;
        FixedValidation.Vec2(origin);
        FixedValidation.Vec2(motion);
        NativeMethods.Check(NativeMethods.WorldRayCast(Handle, origin, motion, filter, out WorldCastHit hit, out int found));
        closest = found != 0 ? hit : default;
        return found != 0;
    }
    public void RayCastAll(Vec2 origin, Vec2 motion, List<WorldCastHit> results) => RayCastAllCore(origin, motion, null, results);
    public void RayCastAll(Vec2 origin, Vec2 motion, in CollisionFilter filter, List<WorldCastHit> results) { CollisionFilter copy = filter; RayCastAllCore(origin, motion, &copy, results); }
    private void RayCastAllCore(Vec2 origin, Vec2 motion, CollisionFilter* filter, List<WorldCastHit> results)
    {
        _ = Handle;
        results.Clear();
        FixedValidation.Vec2(origin);
        FixedValidation.Vec2(motion);
        NativeMethods.Check(NativeMethods.WorldRayCastAll(Handle, origin, motion, filter, out IntPtr data, out int count));
        CopyToList(results, data, count);
    }

    public void Dispose()
    {
        _handle?.Dispose();
        _handle = null;
        _queryBatchNative?.Dispose();
        _queryBatchNative = null;
    }
}
