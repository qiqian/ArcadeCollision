namespace ArcCollision.Wrapper;

/// <summary>
/// Broadphase axis-aligned bounds in the 24.8 fixed-point grid, stored as int32
/// min/max. Drop-in equivalent of <c>ArcCollision.Ref.BpBounds</c>: the shape-derived
/// constructors compute their min/max in the native library so they are bit-for-bit
/// identical to the reference backend; the pure-integer operations run in managed
/// code. Blittable (four <see cref="int"/> fields, 16 bytes) matching the native
/// arc_bp_bounds for by-value P/Invoke. Derived spans widen to long.
/// </summary>
public readonly struct BpBounds
{
    public readonly int MinX, MinY, MaxX, MaxY;

    public BpBounds(long minX, long minY, long maxX, long maxY)
    {
        MinX = (int)minX;
        MinY = (int)minY;
        MaxX = (int)maxX;
        MaxY = (int)maxY;
    }

    public BpBounds(Aabb box) { this = NativeMethods.BpBoundsFromShape(new NativeShape(new Shape(box))); }
    public BpBounds(Circle circle) { this = NativeMethods.BpBoundsFromShape(new NativeShape(new Shape(circle))); }
    public BpBounds(Capsule capsule) { this = NativeMethods.BpBoundsFromShape(new NativeShape(new Shape(capsule))); }
    public BpBounds(Obb box) { this = NativeMethods.BpBoundsFromShape(new NativeShape(new Shape(box))); }
    public BpBounds(Polygon polygon) : this(polygon, Vec2.Zero, new Angle32(0)) { }

    public BpBounds(Polygon polygon, Vec2 translation, Angle32 rotation)
    {
        this = NativeMethods.BpBoundsFromShape(new NativeShape(new Shape(polygon, translation, rotation)));
    }

    public BpBounds(Shape shape) { this = NativeMethods.BpBoundsFromShape(new NativeShape(shape)); }

    public int CenterX => (int)(MinX + (((long)MaxX - MinX) >> 1));
    public int CenterY => (int)(MinY + (((long)MaxY - MinY) >> 1));
    public long Perimeter => 2 * (((long)MaxX - MinX) + ((long)MaxY - MinY));

    public bool Overlaps(in BpBounds other) =>
        MinX <= other.MaxX && other.MinX <= MaxX
        && MinY <= other.MaxY && other.MinY <= MaxY;

    public bool Contains(in BpBounds other) =>
        MinX <= other.MinX && MinY <= other.MinY
        && MaxX >= other.MaxX && MaxY >= other.MaxY;

    public BpBounds Expanded(long margin) =>
        new(MinX - margin, MinY - margin, MaxX + margin, MaxY + margin);

    public BpBounds Translated(long x, long y) =>
        new(MinX + x, MinY + y, MaxX + x, MaxY + y);

    public static BpBounds Union(in BpBounds a, in BpBounds b) => new(
        Math.Min(a.MinX, b.MinX), Math.Min(a.MinY, b.MinY),
        Math.Max(a.MaxX, b.MaxX), Math.Max(a.MaxY, b.MaxY));

    public Aabb ToAabb() => Aabb.FromMinMax(
        new Vec2(MinX / 256f, MinY / 256f),
        new Vec2(MaxX / 256f, MaxY / 256f));
}

/// <summary>
/// Incremental broadphase over the native library: a drop-in equivalent of
/// <c>ArcCollision.Ref.DynamicAabbTree</c> backed by the C API. Proxy indices returned
/// by <see cref="CreateProxy"/> stay valid until <see cref="DestroyProxy"/> /
/// <see cref="Clear"/> / <see cref="Dispose"/>.
/// </summary>
public sealed unsafe class DynamicAabbTree : IDisposable
{
    private readonly NativeDynamicTreeHandle _handle;
    private NativeBuffer<int>? _queryBuffer;
    private NativeBuffer<NativeIntPair>? _pairBuffer;

    public DynamicAabbTree()
    {
        if (NativeMethods.GetAbiVersion() != 7)
            throw new InvalidOperationException("ArcCollision native ABI version mismatch.");
        _handle = NativeMethods.DynamicTreeCreate();
        if (_handle.IsInvalid)
            NativeMethods.ThrowLastOperationError("Native dynamic tree creation failed.");
    }

    private NativeDynamicTreeHandle Handle =>
        _handle is { IsClosed: false } handle ? handle : throw new ObjectDisposedException(nameof(DynamicAabbTree));

    public int Count => NativeMethods.DynamicTreeCount(Handle);

    public void EnsureCapacity(int proxyCapacity) =>
        NativeMethods.Check(NativeMethods.DynamicTreeEnsureCapacity(Handle, proxyCapacity), nameof(proxyCapacity));

    public void Clear() => NativeMethods.Check(NativeMethods.DynamicTreeClear(Handle));

    public int CreateProxy(int id, in BpBounds fatBounds)
    {
        NativeMethods.Check(NativeMethods.DynamicTreeCreateProxy(Handle, id, fatBounds, out int proxy));
        return proxy;
    }

    public bool MoveProxy(int proxy, in BpBounds bounds, in BpBounds fatBounds)
    {
        NativeMethods.Check(NativeMethods.DynamicTreeMoveProxy(Handle, proxy, bounds, fatBounds, out int moved));
        return moved != 0;
    }

    public void DestroyProxy(int proxy) =>
        NativeMethods.Check(NativeMethods.DynamicTreeDestroyProxy(Handle, proxy));

    public void Query(in BpBounds bounds, List<int> results)
    {
        ArgumentNullException.ThrowIfNull(results);
        NativeStatus status = NativeMethods.DynamicTreeQuery(Handle, bounds, null, 0, out int required);
        if (status == NativeStatus.BufferTooSmall)
        {
            NativeBuffer<int> storage = _queryBuffer ??= new();
            int* buffer = storage.EnsureCapacity(required);
            NativeMethods.Check(NativeMethods.DynamicTreeQuery(
                Handle, bounds, buffer, storage.Capacity, out required));
            for (int i = 0; i < required; i++) results.Add(buffer[i]);
            GC.KeepAlive(storage);
        }
        else NativeMethods.Check(status);
    }

    public void ComputeSelfPairs(List<(int A, int B)> results)
    {
        ArgumentNullException.ThrowIfNull(results);
        NativeStatus status = NativeMethods.DynamicTreeComputeSelfPairs(Handle, null, 0, out int required);
        if (status == NativeStatus.BufferTooSmall)
        {
            NativeBuffer<NativeIntPair> storage = _pairBuffer ??= new();
            NativeIntPair* buffer = storage.EnsureCapacity(required);
            NativeMethods.Check(NativeMethods.DynamicTreeComputeSelfPairs(
                Handle, buffer, storage.Capacity, out required));
            for (int i = 0; i < required; i++)
                results.Add((buffer[i].A, buffer[i].B));
            GC.KeepAlive(storage);
        }
        else NativeMethods.Check(status);
    }

    public void Dispose()
    {
        _handle.Dispose();
        _queryBuffer?.Dispose();
        _pairBuffer?.Dispose();
    }
}

/// <summary>
/// Static broadphase over the native library: a drop-in equivalent of
/// <c>ArcCollision.Ref.StaticBvh</c> backed by the C API. Each <see cref="Build"/>
/// fully replaces the leaf set.
/// </summary>
public sealed unsafe class StaticBvh : IDisposable
{
    private readonly NativeStaticBvhHandle _handle;
    private NativeBuffer<int>? _idsBuffer;
    private NativeBuffer<BpBounds>? _boundsBuffer;
    private NativeBuffer<int>? _queryBuffer;

    public StaticBvh()
    {
        if (NativeMethods.GetAbiVersion() != 7)
            throw new InvalidOperationException("ArcCollision native ABI version mismatch.");
        _handle = NativeMethods.StaticBvhCreate();
        if (_handle.IsInvalid)
            NativeMethods.ThrowLastOperationError("Native static BVH creation failed.");
    }

    private NativeStaticBvhHandle Handle =>
        _handle is { IsClosed: false } handle ? handle : throw new ObjectDisposedException(nameof(StaticBvh));

    public void EnsureCapacity(int leafCapacity) =>
        NativeMethods.Check(NativeMethods.StaticBvhEnsureCapacity(Handle, leafCapacity), nameof(leafCapacity));

    public void Clear() => NativeMethods.Check(NativeMethods.StaticBvhClear(Handle));

    public void Build(Dictionary<int, BpBounds> source)
    {
        ArgumentNullException.ThrowIfNull(source);
        int count = source.Count;
        if (count == 0)
        {
            NativeMethods.Check(NativeMethods.StaticBvhBuild(Handle, null, null, 0));
            return;
        }

        NativeBuffer<int> idsStorage = _idsBuffer ??= new();
        NativeBuffer<BpBounds> boundsStorage = _boundsBuffer ??= new();
        int* ids = idsStorage.EnsureCapacity(count);
        BpBounds* bounds = boundsStorage.EnsureCapacity(count);
        int i = 0;
        foreach (KeyValuePair<int, BpBounds> item in source)
        {
            ids[i] = item.Key;
            bounds[i] = item.Value;
            i++;
        }
        NativeMethods.Check(NativeMethods.StaticBvhBuild(Handle, ids, bounds, count));
        GC.KeepAlive(idsStorage);
        GC.KeepAlive(boundsStorage);
    }

    public void Query(in BpBounds bounds, List<int> results)
    {
        ArgumentNullException.ThrowIfNull(results);
        NativeStatus status = NativeMethods.StaticBvhQuery(Handle, bounds, null, 0, out int required);
        if (status == NativeStatus.BufferTooSmall)
        {
            NativeBuffer<int> storage = _queryBuffer ??= new();
            int* buffer = storage.EnsureCapacity(required);
            NativeMethods.Check(NativeMethods.StaticBvhQuery(
                Handle, bounds, buffer, storage.Capacity, out required));
            for (int i = 0; i < required; i++) results.Add(buffer[i]);
            GC.KeepAlive(storage);
        }
        else NativeMethods.Check(status);
    }

    public void Dispose()
    {
        _handle.Dispose();
        _idsBuffer?.Dispose();
        _boundsBuffer?.Dispose();
        _queryBuffer?.Dispose();
    }
}
