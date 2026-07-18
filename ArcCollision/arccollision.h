#ifndef ARCCOLLISION_H
#define ARCCOLLISION_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32) || defined(__CYGWIN__)
#  define ARC_CALL __cdecl
#  if defined(ARC_BUILD_SHARED)
#    define ARC_API __declspec(dllexport)
#  elif defined(ARC_USE_SHARED)
#    define ARC_API __declspec(dllimport)
#  else
#    define ARC_API
#  endif
#else
#  define ARC_CALL
#  if (defined(ARC_BUILD_SHARED) || defined(ARC_BUILD_STATIC)) \
      && (defined(__GNUC__) || defined(__clang__))
#    if defined(__APPLE__) && defined(ARC_BUILD_STATIC)
#      define ARC_API __attribute__((visibility("default"), used))
#    else
#      define ARC_API __attribute__((visibility("default")))
#    endif
#  else
#    define ARC_API
#  endif
#endif

#define ARC_EXPORT ARC_API

#ifdef __cplusplus
extern "C" {
#endif

#define ARC_ABI_VERSION 4u   /* Collider updates take a rigid transform, not a shape. */
#define ARC_MAX_WORLD_COUNT 15
#define ARC_MAX_COLLIDER_COUNT 1048576
#define ARC_MAX_ENTITY_ID 268435455
#define ARC_COLLISION_GRID_SIZE (1.0f / 256.0f)
/* Max |world coordinate| and |extent| accepted at the boundary. Beyond the
   fixed-point range, this bound also keeps every broadphase AABB (position +/-
   extent, scaled by 256) inside the int32 arc_bp_bounds storage. Raising it past
   ~4M would risk broadphase overflow, not just fixed-point range. */
#define ARC_MAX_COORDINATE 1953125.0f
#define ARC_CATEGORY_DEFAULT 1u
#define ARC_CATEGORY_ALL UINT32_MAX

typedef int32_t arc_bool;
/* Fixed-width status type: keep function return ABI stable even when a caller
   compiles C/C++ with options such as -fshort-enums. */
typedef int32_t arc_status;
enum {
    ARC_STATUS_OK = 0,
    ARC_STATUS_INVALID_ARGUMENT = 1,
    ARC_STATUS_OUT_OF_RANGE = 2,
    ARC_STATUS_INVALID_HANDLE = 3,
    ARC_STATUS_BUFFER_TOO_SMALL = 4,
    ARC_STATUS_WORLD_LIMIT = 5,
    ARC_STATUS_INTERNAL_ERROR = 6
};

typedef struct arc_vec2 { float x, y; } arc_vec2;
typedef struct arc_circle { arc_vec2 center; float radius; } arc_circle;
typedef struct arc_aabb { arc_vec2 center, half_extents; } arc_aabb;
typedef struct arc_capsule { arc_vec2 a, b; float radius; } arc_capsule;
typedef struct arc_obb { arc_vec2 center, half_extents; uint32_t angle; } arc_obb;
typedef struct arc_manifold {
    arc_bool colliding; arc_vec2 normal; float depth; arc_vec2 contact;
} arc_manifold;
typedef struct arc_sweep_hit {
    arc_bool hit; float time; arc_vec2 normal; arc_vec2 point;
} arc_sweep_hit;

typedef enum arc_shape_kind {
    ARC_SHAPE_CIRCLE = 0, ARC_SHAPE_AABB = 1, ARC_SHAPE_CAPSULE = 2,
    ARC_SHAPE_OBB = 3, ARC_SHAPE_POLYGON = 4
} arc_shape_kind;
typedef enum arc_sweep_algorithm {
    ARC_SWEEP_ANALYTIC_CIRCLE = 0, ARC_SWEEP_ROUNDED_AABB = 1,
    ARC_SWEEP_ROUNDED_SEGMENT = 2, ARC_SWEEP_LOCAL_SPACE_ROUNDED_AABB = 3,
    ARC_SWEEP_AABB = 4, ARC_SWEEP_CONTINUOUS_SAT = 5,
    ARC_SWEEP_FEATURE_CAST = 6
} arc_sweep_algorithm;

typedef struct arc_polygon arc_polygon;
typedef struct arc_world arc_world;
typedef struct arc_dynamic_tree arc_dynamic_tree;
typedef struct arc_static_bvh arc_static_bvh;

/* Broadphase axis-aligned bounds in the 24.8 fixed-point grid (integer min/max).
   Stored as int32 (16 bytes) to keep tree nodes cache-dense; this holds because
   the coordinate limit keeps every bound within int32 (see ARC_MAX_COORDINATE).
   Layout matches the internal arc::Bounds so it is bit-copyable. */
typedef struct arc_bp_bounds { int32_t min_x, min_y, max_x, max_y; } arc_bp_bounds;
typedef struct arc_int_pair { int32_t a, b; } arc_int_pair;

/* Only the member selected by kind is read. Polygon pointers are borrowed for
   function arguments. arc_world_get_shape returns a retained polygon pointer.
   The primitive geometries and the polygon transform share storage (a tagged
   union), shrinking the struct from ~96 bytes to 24. Packed to 4 so the layout
   is deterministic (no 8-byte pointer padding) and easy to mirror exactly in the
   managed wrapper; the exact offsets are locked by static_asserts in
   arccollision_api.cpp. The nameless union/struct is a universal compiler
   extension; the pragmas below just silence the pedantic note for it. */
#if defined(_MSC_VER)
#  pragma warning(push)
#  pragma warning(disable: 4201)
#elif defined(__clang__)
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wgnu-anonymous-struct"
#  pragma clang diagnostic ignored "-Wnested-anon-types"
#elif defined(__GNUC__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wpedantic"
#endif
#pragma pack(push, 4)
typedef struct arc_shape {
    int32_t kind;              /* @0 */
    union {                    /* @4 */
        arc_circle circle;
        arc_aabb aabb;
        arc_capsule capsule;
        arc_obb obb;
        /* Polygon geometry pointer plus its transform. All three are live at
           once for a polygon, so they are grouped (not overlapped); the group
           shares the union with the primitives, which polygons don't use. */
        struct {
            uint32_t polygon_rotation;     /* @4  */
            arc_vec2 polygon_translation;  /* @8 */
            arc_polygon* polygon;          /* @16 */
        };
    };
} arc_shape;                    /* sizeof == 24 */
#pragma pack(pop)
#if defined(_MSC_VER)
#  pragma warning(pop)
#elif defined(__clang__)
#  pragma clang diagnostic pop
#elif defined(__GNUC__)
#  pragma GCC diagnostic pop
#endif
typedef struct arc_collision_filter { uint32_t categories, collides_with; } arc_collision_filter;
typedef struct arc_handle { uint32_t packed_index, packed_entity_id; } arc_handle;
typedef struct arc_candidate_pair { arc_handle a, b; } arc_candidate_pair;
typedef struct arc_contact_pair { arc_handle a, b; arc_manifold manifold; } arc_contact_pair;
typedef struct arc_world_cast_hit { arc_handle handle; arc_sweep_hit hit; } arc_world_cast_hit;
typedef struct arc_world_options {
    float fat_margin; int32_t initial_collider_capacity, initial_pair_capacity;
} arc_world_options;

/* Placement of a collider's immutable base shape: world position of the shape's
   local origin, a rotation applied to its authored orientation (Angle32, like
   arc_obb.angle), and a uniform scale. At the world boundary position is
   quantized to Q24.8 and scale to Q16.16; composition and materialization are
   integer-only. Identity is
   {position=0, rotation=0, scale=1} and reproduces the authored pose. Circles and
   axis-aligned boxes ignore rotation; OBB/capsule/polygon respond to it. */
typedef struct arc_transform { arc_vec2 position; uint32_t rotation; float scale; } arc_transform;

/* ABI and diagnostics. arc_get_last_error returns thread-local UTF-8 text that
   remains valid until the next ArcCollision call on the same thread. */
ARC_API uint32_t ARC_CALL arc_get_abi_version(void);
ARC_API const char* ARC_CALL arc_get_last_error(void);

/* Immutable polygon geometry. Creation returns one owned reference. Shapes
   borrow polygon pointers for the duration of a call; worlds retain them. */
ARC_API arc_polygon* ARC_CALL arc_polygon_create(const arc_vec2* vertices, int32_t count);
ARC_API void ARC_CALL arc_polygon_retain(arc_polygon* polygon);
ARC_API void ARC_CALL arc_polygon_release(arc_polygon* polygon);
ARC_API int32_t ARC_CALL arc_polygon_get_count(const arc_polygon* polygon);
ARC_API arc_status ARC_CALL arc_polygon_get_vertices(const arc_polygon* polygon, arc_vec2* output, int32_t capacity, int32_t* required);
ARC_API arc_aabb ARC_CALL arc_polygon_get_bounds(const arc_polygon* polygon);
ARC_API arc_polygon* ARC_CALL arc_polygon_moved(const arc_polygon* polygon, arc_vec2 delta);

/* Distance and discrete collision. Public floats are quantized to the 24.8
   grid at entry. Manifold normals point from the first shape to the second. */
ARC_API arc_vec2 ARC_CALL arc_closest_point_on_segment(arc_vec2 p, arc_vec2 a, arc_vec2 b, float* out_t);
ARC_API arc_vec2 ARC_CALL arc_closest_point_on_aabb(arc_vec2 p, arc_aabb box);
ARC_API float ARC_CALL arc_closest_points_segment_segment(arc_vec2 p1, arc_vec2 q1, arc_vec2 p2, arc_vec2 q2, arc_vec2* out_c1, arc_vec2* out_c2);
ARC_API arc_bool ARC_CALL arc_point_in_circle(arc_vec2 p, arc_circle circle);
ARC_API arc_bool ARC_CALL arc_point_in_aabb(arc_vec2 p, arc_aabb box);
ARC_API arc_bool ARC_CALL arc_point_in_capsule(arc_vec2 p, arc_capsule capsule);
ARC_API arc_manifold ARC_CALL arc_circle_vs_circle(arc_circle a, arc_circle b);
ARC_API arc_manifold ARC_CALL arc_aabb_vs_aabb(arc_aabb a, arc_aabb b);
ARC_API arc_manifold ARC_CALL arc_circle_vs_aabb(arc_circle circle, arc_aabb box);
ARC_API arc_manifold ARC_CALL arc_circle_vs_capsule(arc_circle circle, arc_capsule capsule);
ARC_API arc_manifold ARC_CALL arc_capsule_vs_capsule(arc_capsule a, arc_capsule b);
ARC_API arc_manifold ARC_CALL arc_capsule_vs_aabb(arc_capsule capsule, arc_aabb box);
ARC_API arc_manifold ARC_CALL arc_shape_vs_shape(const arc_shape* a, const arc_shape* b);
ARC_API arc_bool ARC_CALL arc_shapes_overlap(const arc_shape* a, const arc_shape* b);
ARC_API arc_aabb ARC_CALL arc_shape_get_bounds(const arc_shape* shape);

/* Continuous collision. motion is the full translation and hit.time is the
   first impact fraction in [0,1]. Shape orientation remains fixed. */
ARC_API arc_sweep_hit ARC_CALL arc_ray_vs_circle(arc_vec2 origin, arc_vec2 motion, arc_circle circle);
ARC_API arc_sweep_hit ARC_CALL arc_ray_vs_aabb(arc_vec2 origin, arc_vec2 motion, arc_aabb box);
ARC_API arc_sweep_hit ARC_CALL arc_moving_circle_vs_circle(arc_circle mover, arc_vec2 motion, arc_circle target);
ARC_API arc_sweep_hit ARC_CALL arc_moving_circle_vs_aabb(arc_circle mover, arc_vec2 motion, arc_aabb target);
ARC_API arc_sweep_hit ARC_CALL arc_moving_circle_vs_capsule(arc_circle mover, arc_vec2 motion, arc_capsule target);
ARC_API arc_sweep_hit ARC_CALL arc_moving_circle_vs_obb(arc_circle mover, arc_vec2 motion, arc_obb target);
ARC_API arc_sweep_hit ARC_CALL arc_moving_aabb_vs_aabb(arc_aabb mover, arc_vec2 motion, arc_aabb target);
ARC_API arc_sweep_hit ARC_CALL arc_moving_shape_vs_shape(const arc_shape* mover, arc_vec2 motion, const arc_shape* target);
ARC_API int32_t ARC_CALL arc_get_sweep_algorithm(const arc_shape* mover, const arc_shape* target);

/* Collision world. Handles become stale after remove/clear/destroy. Static
   additions may be batched and finalized with arc_world_build_static.

   Coordinate limit: every collider's position and extent must stay within
   +/-ARC_MAX_COORDINATE (~1.95M world units). Besides the fixed-point range, the
   broadphase now stores AABBs as int32 (24.8), so a position plus its extent must
   remain inside int32; the +/-ARC_MAX_COORDINATE bound guarantees this. Inputs
   outside the range are rejected at the boundary.

   The compute-pairs, query and cast-all APIs return borrowed read-only views of
   result buffers owned by the world. Consume the returned data immediately; it
   must not be modified or freed and becomes invalid on the next call using the
   same world (or when that world is cleared/destroyed). Empty results return
   data=NULL and count=0. World access is not synchronized; callers must prevent
   concurrent operations on the same world. */
ARC_API arc_world* ARC_CALL arc_world_create(const arc_world_options* options);
ARC_API void ARC_CALL arc_world_destroy(arc_world* world);
ARC_API arc_status ARC_CALL arc_world_clear(arc_world* world);
ARC_API arc_status ARC_CALL arc_world_ensure_capacity(arc_world* world, int32_t collider_capacity, int32_t pair_capacity);
ARC_API arc_status ARC_CALL arc_world_build_static(arc_world* world);
ARC_API int32_t ARC_CALL arc_world_get_count(const arc_world* world);
ARC_API int32_t ARC_CALL arc_world_get_enabled_count(const arc_world* world);
ARC_API int32_t ARC_CALL arc_world_get_dynamic_count(const arc_world* world);
ARC_API int32_t ARC_CALL arc_world_get_static_count(const arc_world* world);
ARC_API float ARC_CALL arc_world_get_fat_margin(const arc_world* world);
ARC_API arc_status ARC_CALL arc_world_add(arc_world* world, int32_t entity_id, const arc_shape* shape, arc_collision_filter filter, arc_bool is_static, arc_bool enabled, arc_handle* out_handle);
/* Re-place a collider's immutable base shape. update_transform sets the absolute
   transform; update_transform_delta composes onto the current one (position
   added, rotation composed, scale multiplied), so an identity delta is a no-op
   and a pure position delta moves the collider keeping its orientation. */
ARC_API arc_status ARC_CALL arc_world_update_transform(arc_world* world, arc_handle handle, const arc_transform* transform);
ARC_API arc_status ARC_CALL arc_world_update_transform_delta(arc_world* world, arc_handle handle, const arc_transform* delta);
ARC_API arc_status ARC_CALL arc_world_remove(arc_world* world, arc_handle handle);
ARC_API arc_bool ARC_CALL arc_world_is_valid(const arc_world* world, arc_handle handle);
ARC_API arc_status ARC_CALL arc_world_get_shape(const arc_world* world, arc_handle handle, arc_shape* out_shape);
ARC_API arc_status ARC_CALL arc_world_get_filter(const arc_world* world, arc_handle handle, arc_collision_filter* out_filter);
ARC_API arc_status ARC_CALL arc_world_set_filter(arc_world* world, arc_handle handle, arc_collision_filter filter);
ARC_API arc_status ARC_CALL arc_world_get_enabled(const arc_world* world, arc_handle handle, arc_bool* out_enabled);
ARC_API arc_status ARC_CALL arc_world_set_enabled(arc_world* world, arc_handle handle, arc_bool enabled);
ARC_API arc_status ARC_CALL arc_world_shift_origin(arc_world* world, arc_vec2 origin_delta);
ARC_API arc_status ARC_CALL arc_world_compute_pairs(arc_world* world, const arc_candidate_pair** out_data, int32_t* out_count);
ARC_API arc_status ARC_CALL arc_world_query(arc_world* world, const arc_shape* query, const arc_collision_filter* filter_or_null, const arc_handle** out_data, int32_t* out_count);
/* Batched box query: queries[0..query_count) are resolved together through a
   4-wide SIMD packet broadphase descent. out_handles receives all results
   concatenated (borrowed, like arc_world_query); out_counts receives query_count
   per-query counts (query k's handles are the out_counts[k] entries following the
   sum of the earlier counts); out_total is the total handle count. query_count==0
   yields out_handles=NULL, out_counts=NULL, out_total=0. */
ARC_API arc_status ARC_CALL arc_world_query_batch(arc_world* world, const arc_shape* queries, int32_t query_count, const arc_collision_filter* filter_or_null, const arc_handle** out_handles, const int32_t** out_counts, int32_t* out_total);
ARC_API arc_status ARC_CALL arc_world_try_contact_pair(arc_world* world, arc_candidate_pair pair, arc_contact_pair* out_contact, arc_bool* out_colliding);
ARC_API arc_status ARC_CALL arc_world_try_contact_shape(arc_world* world, const arc_shape* query, const arc_collision_filter* filter_or_null, arc_handle target, arc_manifold* out_manifold, arc_bool* out_colliding);
ARC_API arc_status ARC_CALL arc_world_shape_cast(arc_world* world, const arc_shape* mover, arc_vec2 motion, const arc_collision_filter* filter_or_null, arc_world_cast_hit* out_hit, arc_bool* out_found);
ARC_API arc_status ARC_CALL arc_world_shape_cast_all(arc_world* world, const arc_shape* mover, arc_vec2 motion, const arc_collision_filter* filter_or_null, const arc_world_cast_hit** out_data, int32_t* out_count);
ARC_API arc_status ARC_CALL arc_world_ray_cast(arc_world* world, arc_vec2 origin, arc_vec2 motion, const arc_collision_filter* filter_or_null, arc_world_cast_hit* out_hit, arc_bool* out_found);
ARC_API arc_status ARC_CALL arc_world_ray_cast_all(arc_world* world, arc_vec2 origin, arc_vec2 motion, const arc_collision_filter* filter_or_null, const arc_world_cast_hit** out_data, int32_t* out_count);

/* Standalone broadphase structures. These expose the same incremental dynamic
   tree and one-shot static BVH the world uses internally, for callers that want
   direct broadphase control. Proxy indices from create_proxy stay valid until
   destroy_proxy/clear/destroy. Query uses the two-call pattern (output=NULL,
   capacity=0 to obtain required, then a caller-owned buffer). */
ARC_API arc_bp_bounds ARC_CALL arc_bp_bounds_from_shape(const arc_shape* shape);

ARC_API arc_dynamic_tree* ARC_CALL arc_dynamic_tree_create(void);
ARC_API void ARC_CALL arc_dynamic_tree_destroy(arc_dynamic_tree* tree);
ARC_API arc_status ARC_CALL arc_dynamic_tree_clear(arc_dynamic_tree* tree);
ARC_API arc_status ARC_CALL arc_dynamic_tree_ensure_capacity(arc_dynamic_tree* tree, int32_t proxy_capacity);
ARC_API int32_t ARC_CALL arc_dynamic_tree_get_count(const arc_dynamic_tree* tree);
ARC_API arc_status ARC_CALL arc_dynamic_tree_create_proxy(arc_dynamic_tree* tree, int32_t id, arc_bp_bounds fat_bounds, int32_t* out_proxy);
ARC_API arc_status ARC_CALL arc_dynamic_tree_move_proxy(arc_dynamic_tree* tree, int32_t proxy, arc_bp_bounds bounds, arc_bp_bounds fat_bounds, arc_bool* out_moved);
ARC_API arc_status ARC_CALL arc_dynamic_tree_destroy_proxy(arc_dynamic_tree* tree, int32_t proxy);
ARC_API arc_status ARC_CALL arc_dynamic_tree_query(const arc_dynamic_tree* tree, arc_bp_bounds bounds, int32_t* output, int32_t capacity, int32_t* required);
ARC_API arc_status ARC_CALL arc_dynamic_tree_compute_self_pairs(const arc_dynamic_tree* tree, arc_int_pair* output, int32_t capacity, int32_t* required);

ARC_API arc_static_bvh* ARC_CALL arc_static_bvh_create(void);
ARC_API void ARC_CALL arc_static_bvh_destroy(arc_static_bvh* bvh);
ARC_API arc_status ARC_CALL arc_static_bvh_clear(arc_static_bvh* bvh);
ARC_API arc_status ARC_CALL arc_static_bvh_ensure_capacity(arc_static_bvh* bvh, int32_t leaf_capacity);
ARC_API arc_status ARC_CALL arc_static_bvh_build(arc_static_bvh* bvh, const int32_t* ids, const arc_bp_bounds* bounds, int32_t count);
ARC_API arc_status ARC_CALL arc_static_bvh_query(arc_static_bvh* bvh, arc_bp_bounds bounds, int32_t* output, int32_t capacity, int32_t* required);

#ifdef __cplusplus
}
#endif
#endif
