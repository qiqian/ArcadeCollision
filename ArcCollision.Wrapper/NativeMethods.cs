using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;

namespace ArcCollision.Wrapper;

internal enum NativeStatus
{
    Ok, InvalidArgument, OutOfRange, InvalidHandle, BufferTooSmall, WorldLimit, InternalError
}

// Mirrors the native arc_shape tagged union (pack 4, 24 bytes) byte-for-byte:
// kind@0, then a union@4 where the primitive geometries and the polygon transform
// (rotation@4, translation@8, pointer@16) share storage -- only the member
// matching Kind is live. Offsets are locked by static_asserts in the native
// arccollision_api.cpp; keep the two in lockstep.
[StructLayout(LayoutKind.Explicit, Pack = 4, Size = 24)]
internal struct NativeShape
{
    [FieldOffset(0)] public int Kind;
    [FieldOffset(4)] public Circle Circle;
    [FieldOffset(4)] public Aabb Aabb;
    [FieldOffset(4)] public Capsule Capsule;
    [FieldOffset(4)] public Obb Obb;
    [FieldOffset(4)] public uint PolygonRotation;
    [FieldOffset(8)] public Vec2 PolygonTranslation;
    [FieldOffset(16)] public IntPtr Polygon;

    public NativeShape(in Shape shape)
    {
        this = default;
        Kind = (int)shape.Kind;
        if (shape.TryGetCircle(out Circle circle)) Circle = circle;
        else if (shape.TryGetAabb(out Aabb aabb)) Aabb = aabb;
        else if (shape.TryGetCapsule(out Capsule capsule)) Capsule = capsule;
        else if (shape.TryGetObb(out Obb obb)) Obb = obb;
        else if (shape.TryGetPolygon(out Polygon? polygon, out Vec2 translation, out Angle32 rotation))
        {
            Polygon = polygon!.Handle.DangerousGetHandle();
            PolygonTranslation = translation;
            PolygonRotation = rotation.Raw;
        }
    }

    public Shape ToManagedOwned()
    {
        return (ShapeKind)Kind switch
        {
            ShapeKind.Circle => new Shape(Circle),
            ShapeKind.Aabb => new Shape(Aabb),
            ShapeKind.Capsule => new Shape(Capsule),
            ShapeKind.Obb => new Shape(Obb),
            ShapeKind.Polygon => new Shape(
                new Polygon(Polygon), PolygonTranslation, new Angle32(PolygonRotation)),
            _ => throw new InvalidOperationException("Native shape kind is invalid."),
        };
    }
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct NativeHandle
{
    public readonly uint PackedIndex, PackedEntityId;
    public NativeHandle(uint index, uint entity) { PackedIndex = index; PackedEntityId = entity; }
}
[StructLayout(LayoutKind.Sequential)] internal readonly struct NativePair
{
    public readonly NativeHandle A, B;
    public NativePair(NativeHandle a, NativeHandle b) { A = a; B = b; }
}
[StructLayout(LayoutKind.Sequential)] internal readonly struct NativeContact { public readonly NativeHandle A, B; public readonly Manifold Manifold; }
[StructLayout(LayoutKind.Sequential)] internal readonly struct NativeCastHit { public readonly NativeHandle Handle; public readonly SweepHit Hit; }
[StructLayout(LayoutKind.Sequential)] internal readonly struct NativeOptions { public readonly float FatMargin; public readonly int ColliderCapacity, PairCapacity; public NativeOptions(in ArcWorldOptions o) { FatMargin = o.FatMargin; ColliderCapacity = o.InitialColliderCapacity; PairCapacity = o.InitialPairCapacity; } }

internal sealed class NativeWorldHandle : SafeHandleZeroOrMinusOneIsInvalid
{
    public NativeWorldHandle() : base(true) { }
    protected override bool ReleaseHandle() { NativeMethods.WorldDestroy(handle); return true; }
}

internal sealed class NativeDynamicTreeHandle : SafeHandleZeroOrMinusOneIsInvalid
{
    public NativeDynamicTreeHandle() : base(true) { }
    protected override bool ReleaseHandle() { NativeMethods.DynamicTreeDestroy(handle); return true; }
}

internal sealed class NativeStaticBvhHandle : SafeHandleZeroOrMinusOneIsInvalid
{
    public NativeStaticBvhHandle() : base(true) { }
    protected override bool ReleaseHandle() { NativeMethods.StaticBvhDestroy(handle); return true; }
}

[StructLayout(LayoutKind.Sequential)]
internal struct NativeIntPair
{
    public int A;
    public int B;
}

internal static unsafe class NativeMethods
{
    private const string Library = "arccollision";

    static NativeMethods()
    {
        NativeLibrary.SetDllImportResolver(typeof(NativeMethods).Assembly, ResolveLibrary);
    }

    private static IntPtr ResolveLibrary(string name, Assembly assembly, DllImportSearchPath? path)
    {
        if (name != Library) return IntPtr.Zero;
        if (OperatingSystem.IsIOS()) return NativeLibrary.GetMainProgramHandle();

        string? explicitPath = Environment.GetEnvironmentVariable("ARCCOLLISION_NATIVE_PATH");
        if (!string.IsNullOrWhiteSpace(explicitPath) && NativeLibrary.TryLoad(explicitPath, out IntPtr handle))
            return handle;

        string basePath = AppContext.BaseDirectory;
        string fileName = OperatingSystem.IsWindows()
            ? "arccollision.dll"
            : OperatingSystem.IsMacOS()
                ? "libarccollision.dylib"
                : "libarccollision.so";

        foreach (string candidate in NativeCandidates(basePath, fileName))
            if (NativeLibrary.TryLoad(candidate, out handle)) return handle;

        // Let the runtime apply platform-specific probing (including Android
        // packaged native libraries) when no explicit or local asset matched.
        if (NativeLibrary.TryLoad(fileName, out handle)) return handle;
        return IntPtr.Zero;
    }

    private static IEnumerable<string> NativeCandidates(string basePath, string fileName)
    {
        yield return Path.Combine(basePath, fileName);

        string architecture = RuntimeInformation.ProcessArchitecture switch
        {
            Architecture.X86 => "x86",
            Architecture.X64 => "x64",
            Architecture.Arm => "arm",
            Architecture.Arm64 => "arm64",
            _ => RuntimeInformation.ProcessArchitecture.ToString().ToLowerInvariant(),
        };

        string[] runtimeIdentifiers = OperatingSystem.IsWindows()
            ? [$"win-{architecture}", $"win10-{architecture}"]
            : OperatingSystem.IsMacOS()
                ? [$"osx-{architecture}"]
                : OperatingSystem.IsAndroid()
                    ? [$"android-{architecture}"]
                    : [$"linux-{architecture}", $"linux-musl-{architecture}"];

        foreach (string runtimeIdentifier in runtimeIdentifiers)
            yield return Path.Combine(
                basePath, "runtimes", runtimeIdentifier, "native", fileName);
    }

    internal static void Check(NativeStatus status, string? parameter = null)
    {
        if (status == NativeStatus.Ok) return;
        string message = Marshal.PtrToStringUTF8(GetLastError()) ?? "Native ArcCollision call failed.";
        throw status switch
        {
            NativeStatus.InvalidArgument => new ArgumentException(message, parameter),
            NativeStatus.OutOfRange => new ArgumentOutOfRangeException(parameter, message),
            NativeStatus.InvalidHandle => new ArgumentException(message.Length == 0 ? "Handle is stale or belongs to another world." : message, parameter ?? "handle"),
            NativeStatus.WorldLimit => new InvalidOperationException(message),
            _ => new InvalidOperationException(message),
        };
    }

    internal static void ThrowLastError(string fallback)
    {
        string message = Marshal.PtrToStringUTF8(GetLastError()) ?? fallback;
        throw new ArgumentException(message.Length == 0 ? fallback : message);
    }

    internal static void ThrowLastOperationError(string fallback)
    {
        string message = Marshal.PtrToStringUTF8(GetLastError()) ?? fallback;
        throw new InvalidOperationException(message.Length == 0 ? fallback : message);
    }

    [DllImport(Library, EntryPoint="arc_get_abi_version", CallingConvention=CallingConvention.Cdecl)] internal static extern uint GetAbiVersion();
    [DllImport(Library, EntryPoint="arc_get_last_error", CallingConvention=CallingConvention.Cdecl)] internal static extern IntPtr GetLastError();
    [DllImport(Library, EntryPoint="arc_polygon_create", CallingConvention=CallingConvention.Cdecl)] internal static extern IntPtr PolygonCreate([In] Vec2[] vertices, int count);
    [DllImport(Library, EntryPoint="arc_polygon_release", CallingConvention=CallingConvention.Cdecl)] internal static extern void PolygonRelease(IntPtr polygon);
    [DllImport(Library, EntryPoint="arc_polygon_get_count", CallingConvention=CallingConvention.Cdecl)] internal static extern int PolygonGetCount(IntPtr polygon);
    [DllImport(Library, EntryPoint="arc_polygon_get_vertices", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus PolygonGetVertices(IntPtr polygon, [Out] Vec2[] output, int capacity, out int required);
    [DllImport(Library, EntryPoint="arc_polygon_get_bounds", CallingConvention=CallingConvention.Cdecl)] internal static extern Aabb PolygonGetBounds(IntPtr polygon);
    [DllImport(Library, EntryPoint="arc_polygon_moved", CallingConvention=CallingConvention.Cdecl)] internal static extern IntPtr PolygonMoved(IntPtr polygon, Vec2 delta);
    [DllImport(Library, EntryPoint="arc_shape_get_bounds", CallingConvention=CallingConvention.Cdecl)] internal static extern Aabb ShapeBounds(in NativeShape shape);

    [DllImport(Library, EntryPoint="arc_closest_point_on_segment", CallingConvention=CallingConvention.Cdecl)] internal static extern Vec2 ClosestPointSegment(Vec2 p, Vec2 a, Vec2 b, out float t);
    [DllImport(Library, EntryPoint="arc_closest_point_on_aabb", CallingConvention=CallingConvention.Cdecl)] internal static extern Vec2 ClosestPointAabb(Vec2 p, Aabb box);
    [DllImport(Library, EntryPoint="arc_closest_points_segment_segment", CallingConvention=CallingConvention.Cdecl)] internal static extern float ClosestSegments(Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2, out Vec2 c1, out Vec2 c2);
    [DllImport(Library, EntryPoint="arc_point_in_circle", CallingConvention=CallingConvention.Cdecl)] internal static extern int PointCircle(Vec2 p, Circle c);
    [DllImport(Library, EntryPoint="arc_point_in_aabb", CallingConvention=CallingConvention.Cdecl)] internal static extern int PointAabb(Vec2 p, Aabb b);
    [DllImport(Library, EntryPoint="arc_point_in_capsule", CallingConvention=CallingConvention.Cdecl)] internal static extern int PointCapsule(Vec2 p, Capsule c);
    [DllImport(Library, EntryPoint="arc_circle_vs_circle", CallingConvention=CallingConvention.Cdecl)] internal static extern Manifold CircleCircle(Circle a, Circle b);
    [DllImport(Library, EntryPoint="arc_aabb_vs_aabb", CallingConvention=CallingConvention.Cdecl)] internal static extern Manifold AabbAabb(Aabb a, Aabb b);
    [DllImport(Library, EntryPoint="arc_circle_vs_aabb", CallingConvention=CallingConvention.Cdecl)] internal static extern Manifold CircleAabb(Circle c, Aabb b);
    [DllImport(Library, EntryPoint="arc_circle_vs_capsule", CallingConvention=CallingConvention.Cdecl)] internal static extern Manifold CircleCapsule(Circle c, Capsule b);
    [DllImport(Library, EntryPoint="arc_capsule_vs_capsule", CallingConvention=CallingConvention.Cdecl)] internal static extern Manifold CapsuleCapsule(Capsule a, Capsule b);
    [DllImport(Library, EntryPoint="arc_capsule_vs_aabb", CallingConvention=CallingConvention.Cdecl)] internal static extern Manifold CapsuleAabb(Capsule c, Aabb b);
    [DllImport(Library, EntryPoint="arc_shape_vs_shape", CallingConvention=CallingConvention.Cdecl)] internal static extern Manifold ShapeShape(in NativeShape a, in NativeShape b);
    [DllImport(Library, EntryPoint="arc_shapes_overlap", CallingConvention=CallingConvention.Cdecl)] internal static extern int ShapesOverlap(in NativeShape a, in NativeShape b);
    [DllImport(Library, EntryPoint="arc_ray_vs_circle", CallingConvention=CallingConvention.Cdecl)] internal static extern SweepHit RayCircle(Vec2 o, Vec2 m, Circle c);
    [DllImport(Library, EntryPoint="arc_ray_vs_aabb", CallingConvention=CallingConvention.Cdecl)] internal static extern SweepHit RayAabb(Vec2 o, Vec2 m, Aabb b);
    [DllImport(Library, EntryPoint="arc_moving_circle_vs_circle", CallingConvention=CallingConvention.Cdecl)] internal static extern SweepHit MovingCircleCircle(Circle a, Vec2 m, Circle b);
    [DllImport(Library, EntryPoint="arc_moving_circle_vs_aabb", CallingConvention=CallingConvention.Cdecl)] internal static extern SweepHit MovingCircleAabb(Circle a, Vec2 m, Aabb b);
    [DllImport(Library, EntryPoint="arc_moving_circle_vs_capsule", CallingConvention=CallingConvention.Cdecl)] internal static extern SweepHit MovingCircleCapsule(Circle a, Vec2 m, Capsule b);
    [DllImport(Library, EntryPoint="arc_moving_circle_vs_obb", CallingConvention=CallingConvention.Cdecl)] internal static extern SweepHit MovingCircleObb(Circle a, Vec2 m, Obb b);
    [DllImport(Library, EntryPoint="arc_moving_aabb_vs_aabb", CallingConvention=CallingConvention.Cdecl)] internal static extern SweepHit MovingAabbAabb(Aabb a, Vec2 m, Aabb b);
    [DllImport(Library, EntryPoint="arc_moving_shape_vs_shape", CallingConvention=CallingConvention.Cdecl)] internal static extern SweepHit MovingShapeShape(in NativeShape a, Vec2 m, in NativeShape b);
    [DllImport(Library, EntryPoint="arc_get_sweep_algorithm", CallingConvention=CallingConvention.Cdecl)] internal static extern int GetSweepAlgorithm(in NativeShape a, in NativeShape b);

    [DllImport(Library, EntryPoint="arc_world_create", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeWorldHandle WorldCreate(in NativeOptions options);
    [DllImport(Library, EntryPoint="arc_world_destroy", CallingConvention=CallingConvention.Cdecl)] internal static extern void WorldDestroy(IntPtr world);
    [DllImport(Library, EntryPoint="arc_world_clear", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldClear(NativeWorldHandle world);
    [DllImport(Library, EntryPoint="arc_world_ensure_capacity", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldEnsureCapacity(NativeWorldHandle world, int collider, int pairs);
    [DllImport(Library, EntryPoint="arc_world_build_static", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldBuildStatic(NativeWorldHandle world);
    [DllImport(Library, EntryPoint="arc_world_get_count", CallingConvention=CallingConvention.Cdecl)] internal static extern int WorldCount(NativeWorldHandle world);
    [DllImport(Library, EntryPoint="arc_world_get_enabled_count", CallingConvention=CallingConvention.Cdecl)] internal static extern int WorldEnabledCount(NativeWorldHandle world);
    [DllImport(Library, EntryPoint="arc_world_get_dynamic_count", CallingConvention=CallingConvention.Cdecl)] internal static extern int WorldDynamicCount(NativeWorldHandle world);
    [DllImport(Library, EntryPoint="arc_world_get_static_count", CallingConvention=CallingConvention.Cdecl)] internal static extern int WorldStaticCount(NativeWorldHandle world);
    [DllImport(Library, EntryPoint="arc_world_get_fat_margin", CallingConvention=CallingConvention.Cdecl)] internal static extern float WorldFatMargin(NativeWorldHandle world);
    [DllImport(Library, EntryPoint="arc_world_add", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldAdd(NativeWorldHandle world, int entity, in NativeShape shape, CollisionFilter filter, int isStatic, int enabled, out NativeHandle handle);
    [DllImport(Library, EntryPoint="arc_world_update", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldUpdate(NativeWorldHandle world, NativeHandle handle, in NativeShape shape);
    [DllImport(Library, EntryPoint="arc_world_remove", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldRemove(NativeWorldHandle world, NativeHandle handle);
    [DllImport(Library, EntryPoint="arc_world_is_valid", CallingConvention=CallingConvention.Cdecl)] internal static extern int WorldIsValid(NativeWorldHandle world, NativeHandle handle);
    [DllImport(Library, EntryPoint="arc_world_get_shape", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldGetShape(NativeWorldHandle world, NativeHandle handle, out NativeShape shape);
    [DllImport(Library, EntryPoint="arc_world_get_filter", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldGetFilter(NativeWorldHandle world, NativeHandle handle, out CollisionFilter filter);
    [DllImport(Library, EntryPoint="arc_world_set_filter", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldSetFilter(NativeWorldHandle world, NativeHandle handle, CollisionFilter filter);
    [DllImport(Library, EntryPoint="arc_world_get_enabled", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldGetEnabled(NativeWorldHandle world, NativeHandle handle, out int enabled);
    [DllImport(Library, EntryPoint="arc_world_set_enabled", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldSetEnabled(NativeWorldHandle world, NativeHandle handle, int enabled);
    [DllImport(Library, EntryPoint="arc_world_shift_origin", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldShiftOrigin(NativeWorldHandle world, Vec2 delta);
    [DllImport(Library, EntryPoint="arc_world_compute_pairs", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldComputePairs(NativeWorldHandle world, [Out] NativePair[]? output, int capacity, out int required);
    [DllImport(Library, EntryPoint="arc_world_query", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldQuery(NativeWorldHandle world, in NativeShape query, CollisionFilter* filter, [Out] NativeHandle[]? output, int capacity, out int required);
    [DllImport(Library, EntryPoint="arc_world_try_contact_pair", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldContactPair(NativeWorldHandle world, NativePair pair, out NativeContact contact, out int colliding);
    [DllImport(Library, EntryPoint="arc_world_try_contact_shape", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldContactShape(NativeWorldHandle world, in NativeShape query, CollisionFilter* filter, NativeHandle target, out Manifold manifold, out int colliding);
    [DllImport(Library, EntryPoint="arc_world_shape_cast", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldShapeCast(NativeWorldHandle world, in NativeShape mover, Vec2 motion, CollisionFilter* filter, out NativeCastHit hit, out int found);
    [DllImport(Library, EntryPoint="arc_world_shape_cast_all", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldShapeCastAll(NativeWorldHandle world, in NativeShape mover, Vec2 motion, CollisionFilter* filter, [Out] NativeCastHit[]? output, int capacity, out int required);
    [DllImport(Library, EntryPoint="arc_world_ray_cast", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldRayCast(NativeWorldHandle world, Vec2 origin, Vec2 motion, CollisionFilter* filter, out NativeCastHit hit, out int found);
    [DllImport(Library, EntryPoint="arc_world_ray_cast_all", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus WorldRayCastAll(NativeWorldHandle world, Vec2 origin, Vec2 motion, CollisionFilter* filter, [Out] NativeCastHit[]? output, int capacity, out int required);

    [DllImport(Library, EntryPoint="arc_bp_bounds_from_shape", CallingConvention=CallingConvention.Cdecl)] internal static extern BpBounds BpBoundsFromShape(in NativeShape shape);

    [DllImport(Library, EntryPoint="arc_dynamic_tree_create", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeDynamicTreeHandle DynamicTreeCreate();
    [DllImport(Library, EntryPoint="arc_dynamic_tree_destroy", CallingConvention=CallingConvention.Cdecl)] internal static extern void DynamicTreeDestroy(IntPtr tree);
    [DllImport(Library, EntryPoint="arc_dynamic_tree_clear", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus DynamicTreeClear(NativeDynamicTreeHandle tree);
    [DllImport(Library, EntryPoint="arc_dynamic_tree_ensure_capacity", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus DynamicTreeEnsureCapacity(NativeDynamicTreeHandle tree, int proxyCapacity);
    [DllImport(Library, EntryPoint="arc_dynamic_tree_get_count", CallingConvention=CallingConvention.Cdecl)] internal static extern int DynamicTreeCount(NativeDynamicTreeHandle tree);
    [DllImport(Library, EntryPoint="arc_dynamic_tree_create_proxy", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus DynamicTreeCreateProxy(NativeDynamicTreeHandle tree, int id, BpBounds fatBounds, out int proxy);
    [DllImport(Library, EntryPoint="arc_dynamic_tree_move_proxy", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus DynamicTreeMoveProxy(NativeDynamicTreeHandle tree, int proxy, BpBounds bounds, BpBounds fatBounds, out int moved);
    [DllImport(Library, EntryPoint="arc_dynamic_tree_destroy_proxy", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus DynamicTreeDestroyProxy(NativeDynamicTreeHandle tree, int proxy);
    [DllImport(Library, EntryPoint="arc_dynamic_tree_query", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus DynamicTreeQuery(NativeDynamicTreeHandle tree, BpBounds bounds, [Out] int[]? output, int capacity, out int required);
    [DllImport(Library, EntryPoint="arc_dynamic_tree_compute_self_pairs", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus DynamicTreeComputeSelfPairs(NativeDynamicTreeHandle tree, [Out] NativeIntPair[]? output, int capacity, out int required);

    [DllImport(Library, EntryPoint="arc_static_bvh_create", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStaticBvhHandle StaticBvhCreate();
    [DllImport(Library, EntryPoint="arc_static_bvh_destroy", CallingConvention=CallingConvention.Cdecl)] internal static extern void StaticBvhDestroy(IntPtr bvh);
    [DllImport(Library, EntryPoint="arc_static_bvh_clear", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus StaticBvhClear(NativeStaticBvhHandle bvh);
    [DllImport(Library, EntryPoint="arc_static_bvh_ensure_capacity", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus StaticBvhEnsureCapacity(NativeStaticBvhHandle bvh, int leafCapacity);
    [DllImport(Library, EntryPoint="arc_static_bvh_build", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus StaticBvhBuild(NativeStaticBvhHandle bvh, [In] int[]? ids, [In] BpBounds[]? bounds, int count);
    [DllImport(Library, EntryPoint="arc_static_bvh_query", CallingConvention=CallingConvention.Cdecl)] internal static extern NativeStatus StaticBvhQuery(NativeStaticBvhHandle bvh, BpBounds bounds, [Out] int[]? output, int capacity, out int required);
}
