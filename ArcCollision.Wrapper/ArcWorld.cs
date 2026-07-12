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
    internal ArcHandle(NativeHandle h) { _packedIndex = h.PackedIndex; _packedEntityId = h.PackedEntityId; }
    internal ArcHandle(int index, uint generation, uint worldId, int entityId)
    {
        if ((uint)index > MaxIndex || generation is 0 or > MaxGeneration
            || worldId is 0 or > ArcWorld.MaxWorldCount
            || (uint)entityId > MaxEntityId)
            throw new ArgumentOutOfRangeException();
        _packedIndex = (generation << IndexBits) | (uint)index;
        _packedEntityId = (worldId << 28) | (uint)entityId;
    }
    internal NativeHandle Native => new(_packedIndex, _packedEntityId);
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
}

public readonly struct CandidatePair
{
    public readonly ArcHandle A, B;
    internal CandidatePair(NativePair pair) { A = new(pair.A); B = new(pair.B); }
    internal NativePair Native => new(A.Native, B.Native);
}
public readonly struct ContactPair
{
    public readonly ArcHandle A, B;
    public readonly Manifold Manifold;
    internal ContactPair(NativeContact value) { A = new(value.A); B = new(value.B); Manifold = value.Manifold; }
}
public readonly struct WorldCastHit
{
    public readonly ArcHandle Handle;
    public readonly SweepHit Hit;
    internal WorldCastHit(NativeCastHit value) { Handle = new(value.Handle); Hit = value.Hit; }
}

public sealed unsafe class ArcWorld : IDisposable
{
    public const int MaxWorldCount = 15;
    public const int MaxColliderCount = ArcHandle.MaxIndex + 1;
    private NativeWorldHandle? _handle;
    private NativePair[] _pairs = Array.Empty<NativePair>();
    private NativeHandle[] _handles = Array.Empty<NativeHandle>();
    private NativeCastHit[] _hits = Array.Empty<NativeCastHit>();

    public ArcWorld(float fatMargin = 16) : this(new ArcWorldOptions(fatMargin)) { }
    public ArcWorld(in ArcWorldOptions options)
    {
        _ = FixedValidation.From(options.FatMargin);
        if (NativeMethods.GetAbiVersion() != 2) throw new InvalidOperationException("ArcCollision native ABI version mismatch.");
        NativeOptions native = new(options);
        _handle = NativeMethods.WorldCreate(native);
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
        FixedValidation.Shape(shape);
        NativeShape native = shape.ToNative();
        NativeMethods.Check(NativeMethods.WorldAdd(Handle, entityId, native, filter, isStatic ? 1 : 0, enabled ? 1 : 0, out NativeHandle result), nameof(entityId));
        GC.KeepAlive(shape.PolygonObject);
        return new ArcHandle(result);
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
    public void Update(ArcHandle handle, in Shape shape)
    {
        if (!IsValid(handle))
            throw new ArgumentException(
                "Handle is stale or does not belong to this world.", nameof(handle));
        FixedValidation.Shape(shape);
        NativeShape native = shape.ToNative();
        NativeMethods.Check(NativeMethods.WorldUpdate(Handle, handle.Native, native), nameof(handle));
        GC.KeepAlive(shape.PolygonObject);
    }
    public void Remove(ArcHandle handle) => NativeMethods.Check(NativeMethods.WorldRemove(Handle, handle.Native), nameof(handle));
    public bool IsValid(ArcHandle handle) => NativeMethods.WorldIsValid(Handle, handle.Native) != 0;
    public Shape GetShape(ArcHandle handle)
    {
        NativeMethods.Check(NativeMethods.WorldGetShape(Handle, handle.Native, out NativeShape shape), nameof(handle));
        return shape.ToManagedOwned();
    }
    public bool TryGetShape(ArcHandle handle, out Shape shape)
    {
        if (!IsValid(handle)) { shape = default; return false; }
        shape = GetShape(handle); return true;
    }
    public CollisionFilter GetFilter(ArcHandle handle) { NativeMethods.Check(NativeMethods.WorldGetFilter(Handle, handle.Native, out CollisionFilter filter), nameof(handle)); return filter; }
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
    public void SetFilter(ArcHandle handle, in CollisionFilter filter) => NativeMethods.Check(NativeMethods.WorldSetFilter(Handle, handle.Native, filter), nameof(handle));
    public bool IsEnabled(ArcHandle handle) { NativeMethods.Check(NativeMethods.WorldGetEnabled(Handle, handle.Native, out int enabled), nameof(handle)); return enabled != 0; }
    public void SetEnabled(ArcHandle handle, bool enabled) => NativeMethods.Check(NativeMethods.WorldSetEnabled(Handle, handle.Native, enabled ? 1 : 0), nameof(handle));
    public void Clear() => NativeMethods.Check(NativeMethods.WorldClear(Handle));

    public void ComputePairs(List<CandidatePair> results)
    {
        ArgumentNullException.ThrowIfNull(results); results.Clear();
        NativeStatus status = NativeMethods.WorldComputePairs(Handle, null, 0, out int required);
        if (status == NativeStatus.Ok) return;
        if (status != NativeStatus.BufferTooSmall) NativeMethods.Check(status);
        if (_pairs.Length < required) _pairs = new NativePair[required];
        NativeMethods.Check(NativeMethods.WorldComputePairs(Handle, _pairs, _pairs.Length, out required));
        for (int i = 0; i < required; i++) results.Add(new CandidatePair(_pairs[i]));
    }

    public void Query(in Shape query, List<ArcHandle> results) => QueryCore(query, null, results);
    public void Query(in Shape query, in CollisionFilter filter, List<ArcHandle> results) { CollisionFilter copy = filter; QueryCore(query, &copy, results); }
    private void QueryCore(in Shape query, CollisionFilter* filter, List<ArcHandle> results)
    {
        _ = Handle;
        ArgumentNullException.ThrowIfNull(results);
        results.Clear();
        FixedValidation.Shape(query);
        NativeShape native = query.ToNative();
        NativeStatus status = NativeMethods.WorldQuery(Handle, native, filter, null, 0, out int required);
        if (status == NativeStatus.BufferTooSmall)
        {
            if (_handles.Length < required) _handles = new NativeHandle[required];
            NativeMethods.Check(NativeMethods.WorldQuery(Handle, native, filter, _handles, _handles.Length, out required));
            for (int i = 0; i < required; i++) results.Add(new ArcHandle(_handles[i]));
        }
        else NativeMethods.Check(status);
        GC.KeepAlive(query.PolygonObject);
    }

    public bool TryComputeContact(in CandidatePair pair, out ContactPair contact)
    {
        NativeStatus status = NativeMethods.WorldContactPair(Handle, pair.Native, out NativeContact native, out int colliding);
        if (status == NativeStatus.InvalidHandle) { contact = default; return false; }
        NativeMethods.Check(status); contact = colliding != 0 ? new ContactPair(native) : default; return colliding != 0;
    }
    public bool TryComputeContact(in Shape query, ArcHandle target, out Manifold manifold) => ContactShape(query, null, target, out manifold);
    public bool TryComputeContact(in Shape query, in CollisionFilter filter, ArcHandle target, out Manifold manifold) { CollisionFilter copy = filter; return ContactShape(query, &copy, target, out manifold); }
    private bool ContactShape(in Shape query, CollisionFilter* filter, ArcHandle target, out Manifold manifold)
    {
        _ = Handle;
        FixedValidation.Shape(query);
        NativeShape native = query.ToNative(); NativeStatus status = NativeMethods.WorldContactShape(Handle, native, filter, target.Native, out manifold, out int colliding);
        GC.KeepAlive(query.PolygonObject); if (status == NativeStatus.InvalidHandle) { manifold = Manifold.None; return false; }
        NativeMethods.Check(status); return colliding != 0;
    }

    public bool ShapeCast(in Shape mover, Vec2 motion, out WorldCastHit closest) => ShapeCastCore(mover, motion, null, out closest);
    public bool ShapeCast(in Shape mover, Vec2 motion, in CollisionFilter filter, out WorldCastHit closest) { CollisionFilter copy = filter; return ShapeCastCore(mover, motion, &copy, out closest); }
    private bool ShapeCastCore(in Shape mover, Vec2 motion, CollisionFilter* filter, out WorldCastHit closest)
    {
        _ = Handle;
        FixedValidation.Shape(mover);
        FixedValidation.Vec2(motion);
        NativeShape native = mover.ToNative(); NativeMethods.Check(NativeMethods.WorldShapeCast(Handle, native, motion, filter, out NativeCastHit hit, out int found));
        GC.KeepAlive(mover.PolygonObject); closest = found != 0 ? new WorldCastHit(hit) : default; return found != 0;
    }
    public void ShapeCastAll(in Shape mover, Vec2 motion, List<WorldCastHit> results) => ShapeCastAllCore(mover, motion, null, results);
    public void ShapeCastAll(in Shape mover, Vec2 motion, in CollisionFilter filter, List<WorldCastHit> results) { CollisionFilter copy = filter; ShapeCastAllCore(mover, motion, &copy, results); }
    private void ShapeCastAllCore(in Shape mover, Vec2 motion, CollisionFilter* filter, List<WorldCastHit> results)
    {
        _ = Handle;
        ArgumentNullException.ThrowIfNull(results);
        results.Clear();
        FixedValidation.Shape(mover);
        FixedValidation.Vec2(motion);
        NativeShape native = mover.ToNative();
        NativeStatus status = NativeMethods.WorldShapeCastAll(Handle, native, motion, filter, null, 0, out int required);
        if (status == NativeStatus.BufferTooSmall)
        {
            if (_hits.Length < required) _hits = new NativeCastHit[required];
            NativeMethods.Check(NativeMethods.WorldShapeCastAll(Handle, native, motion, filter, _hits, _hits.Length, out required));
            for (int i = 0; i < required; i++) results.Add(new WorldCastHit(_hits[i]));
        }
        else NativeMethods.Check(status);
        GC.KeepAlive(mover.PolygonObject);
    }
    public bool RayCast(Vec2 origin, Vec2 motion, out WorldCastHit closest) => RayCastCore(origin, motion, null, out closest);
    public bool RayCast(Vec2 origin, Vec2 motion, in CollisionFilter filter, out WorldCastHit closest) { CollisionFilter copy = filter; return RayCastCore(origin, motion, &copy, out closest); }
    private bool RayCastCore(Vec2 origin, Vec2 motion, CollisionFilter* filter, out WorldCastHit closest)
    {
        _ = Handle;
        FixedValidation.Vec2(origin);
        FixedValidation.Vec2(motion);
        NativeMethods.Check(NativeMethods.WorldRayCast(Handle, origin, motion, filter, out NativeCastHit hit, out int found));
        closest = found != 0 ? new WorldCastHit(hit) : default;
        return found != 0;
    }
    public void RayCastAll(Vec2 origin, Vec2 motion, List<WorldCastHit> results) => RayCastAllCore(origin, motion, null, results);
    public void RayCastAll(Vec2 origin, Vec2 motion, in CollisionFilter filter, List<WorldCastHit> results) { CollisionFilter copy = filter; RayCastAllCore(origin, motion, &copy, results); }
    private void RayCastAllCore(Vec2 origin, Vec2 motion, CollisionFilter* filter, List<WorldCastHit> results)
    {
        _ = Handle;
        ArgumentNullException.ThrowIfNull(results);
        results.Clear();
        FixedValidation.Vec2(origin);
        FixedValidation.Vec2(motion);
        NativeStatus status = NativeMethods.WorldRayCastAll(Handle, origin, motion, filter, null, 0, out int required);
        if (status == NativeStatus.BufferTooSmall) { if (_hits.Length < required) _hits = new NativeCastHit[required]; NativeMethods.Check(NativeMethods.WorldRayCastAll(Handle, origin, motion, filter, _hits, _hits.Length, out required)); for (int i = 0; i < required; i++) results.Add(new WorldCastHit(_hits[i])); }
        else NativeMethods.Check(status);
    }

    public void Dispose() { _handle?.Dispose(); _handle = null; }
}
