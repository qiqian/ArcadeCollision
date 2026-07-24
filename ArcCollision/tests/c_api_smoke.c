#include "arccollision.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned add_hash(unsigned hash, unsigned value)
{
    return (hash ^ value) * 16777619u;
}

static unsigned float_bits(float value)
{
    unsigned bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static int locked_hash_smoke(void)
{
    arc_shape shapes[6] = {{0}};
    int entities[6] = {10, 20, 30, 40, 50, 60};
    arc_world* world = arc_world_create(&(arc_world_options){1, 16, 16});
    const arc_handle* query = 0;
    const arc_candidate_pair* pair_view = 0;
    arc_candidate_pair pairs[15];
    int query_count = 0, pair_count = 0, i;
    unsigned hash = 2166136261u;
    arc_sweep_hit sweep;
    if (!world) return 31;

    shapes[0].kind = ARC_SHAPE_CIRCLE;
    shapes[0].circle = (arc_circle){{-2, 0}, 1.25f};
    shapes[1].kind = ARC_SHAPE_AABB;
    shapes[1].aabb = (arc_aabb){{-0.5f, 0}, {1, 1}};
    shapes[2].kind = ARC_SHAPE_CAPSULE;
    shapes[2].capsule = (arc_capsule){{1, -1}, {1, 1}, 0.75f};
    shapes[3].kind = ARC_SHAPE_OBB;
    shapes[3].obb = (arc_obb){{2.25f, 0}, {1, 0.5f}, 170891319u};
    shapes[4].kind = ARC_SHAPE_CIRCLE;
    shapes[4].circle = (arc_circle){{10, 10}, 0.5f};
    shapes[5].kind = ARC_SHAPE_AABB;
    shapes[5].aabb = (arc_aabb){{0, 3}, {4, 0.25f}};

    for (i = 0; i < 6; ++i) {
        arc_handle handle;
        if (arc_world_add(world, entities[i], &shapes[i],
                (arc_collision_filter){ARC_CATEGORY_DEFAULT, ARC_CATEGORY_ALL},
                i == 3 || i == 5, 1, &handle) != ARC_STATUS_OK)
            return 32;
    }
    if (arc_world_build_static(world) != ARC_STATUS_OK) return 33;
    {
        arc_shape all = {0};
        all.kind = ARC_SHAPE_AABB;
        all.aabb = (arc_aabb){{0, 0}, {20, 20}};
        if (arc_world_query(world, &all, 0, &query, &query_count)
            != ARC_STATUS_OK) return 34;
    }
    for (i = 0; i < query_count; ++i) {
        hash = add_hash(hash, query[i].packed_entity_id & ARC_MAX_ENTITY_ID);
    }
    if (arc_world_compute_pairs(world, &pair_view, &pair_count)
        != ARC_STATUS_OK) return 35;
    memcpy(pairs, pair_view, (size_t)pair_count * sizeof(*pairs));
    for (i = 0; i < pair_count; ++i) {
        arc_contact_pair contact;
        arc_bool colliding;
        hash = add_hash(hash, pairs[i].a.packed_entity_id & ARC_MAX_ENTITY_ID);
        hash = add_hash(hash, pairs[i].b.packed_entity_id & ARC_MAX_ENTITY_ID);
        if (arc_world_try_contact_pair(
                world, pairs[i], ARC_MANIFOLD_ALL,
                &contact, &colliding) != ARC_STATUS_OK)
            return 36;
        if (!colliding) continue;
        hash = add_hash(hash, float_bits(contact.manifold.depth));
        hash = add_hash(hash, float_bits(contact.manifold.normal.x));
        hash = add_hash(hash, float_bits(contact.manifold.normal.y));
    }
    sweep = arc_moving_circle_vs_aabb(
        (arc_circle){{-5, 0}, 0.5f}, (arc_vec2){10, 0},
        (arc_aabb){{0, 0}, {1, 1}});
    hash = add_hash(hash, float_bits(sweep.time));
    hash = add_hash(hash, float_bits(sweep.normal.x));
    hash = add_hash(hash, float_bits(sweep.normal.y));
    arc_world_destroy(world);
    if (hash != 2644972881u) {
        fprintf(stderr, "locked hash: expected 2644972881, got %u\n", hash);
        return 37;
    }
    return 0;
}

static int broadphase_stress(void)
{
    enum { count = 128 };
    arc_world_options options = {2.0f, count, count * 4};
    arc_world* world = arc_world_create(&options);
    arc_handle handles[count];
    arc_circle circles[count];
    int active[count];
    int enabled[count];
    int is_static[count];
    int i, j;
    int expected_pairs = 0;
    int required = 0;
    const arc_candidate_pair* pairs = 0;
    const arc_handle* query_results = 0;
    arc_shape query = {0};
    if (!world) return 20;

    for (i = 0; i < count; ++i) {
        arc_shape shape = {0};
        circles[i].center.x = (float)(i % 16) * 2.0f;
        circles[i].center.y = (float)(i / 16) * 2.0f;
        circles[i].radius = 1.25f;
        shape.kind = ARC_SHAPE_CIRCLE;
        shape.circle = circles[i];
        active[i] = enabled[i] = 1;
        is_static[i] = i % 5 == 0;
        if (arc_world_add(world, i, &shape,
                (arc_collision_filter){ARC_CATEGORY_DEFAULT, ARC_CATEGORY_ALL},
                is_static[i], 1, &handles[i]) != ARC_STATUS_OK) {
            arc_world_destroy(world);
            return 21;
        }
    }

    for (i = 0; i < count; ++i) {
        if (!is_static[i] && i % 7 == 0) {
            arc_transform transform;
            circles[i].center.x += i % 14 == 0 ? 4.5f : 0.75f;
            transform.position = circles[i].center;
            transform.rotation = 0u;
            transform.scale = 1.0f;
            if (arc_world_update_transform(world, handles[i], &transform) != ARC_STATUS_OK)
                return 22;
        }
        if (is_static[i] && i % 17 == 0) {
            arc_transform transform;
            circles[i].center.y += 1.0f;
            transform.position = circles[i].center;
            transform.rotation = 0u;
            transform.scale = 1.0f;
            if (arc_world_update_transform(world, handles[i], &transform) != ARC_STATUS_OK)
                return 22;
        }
        if (i % 11 == 0) {
            enabled[i] = 0;
            if (arc_world_set_enabled(world, handles[i], 0) != ARC_STATUS_OK)
                return 23;
        }
        if (i % 13 == 0) {
            active[i] = enabled[i] = 0;
            if (arc_world_remove(world, handles[i]) != ARC_STATUS_OK)
                return 24;
        }
    }
    if (arc_world_build_static(world) != ARC_STATUS_OK) return 25;

    for (i = 0; i < count; ++i) {
        if (!active[i] || !enabled[i]) continue;
        for (j = i + 1; j < count; ++j) {
            if (!active[j] || !enabled[j]
                || (is_static[i] && is_static[j])) continue;
            if (fabsf(circles[i].center.x - circles[j].center.x) <= 2.5f
                && fabsf(circles[i].center.y - circles[j].center.y) <= 2.5f)
                ++expected_pairs;
        }
    }
    if (arc_world_compute_pairs(world, &pairs, &required) != ARC_STATUS_OK
        || required != expected_pairs)
        return 26;
    for (i = 1; i < required; ++i) {
        unsigned previous_a = pairs[i - 1].a.packed_entity_id & ARC_MAX_ENTITY_ID;
        unsigned current_a = pairs[i].a.packed_entity_id & ARC_MAX_ENTITY_ID;
        unsigned previous_b = pairs[i - 1].b.packed_entity_id & ARC_MAX_ENTITY_ID;
        unsigned current_b = pairs[i].b.packed_entity_id & ARC_MAX_ENTITY_ID;
        if (previous_a > current_a
            || (previous_a == current_a && previous_b > current_b))
            return 29;
    }

    query.kind = ARC_SHAPE_AABB;
    query.aabb.center = (arc_vec2){8, 6};
    query.aabb.half_extents = (arc_vec2){4, 3};
    expected_pairs = 0;
    for (i = 0; i < count; ++i) {
        if (active[i] && enabled[i]
            && fabsf(circles[i].center.x - 8.0f) <= 5.25f
            && fabsf(circles[i].center.y - 6.0f) <= 4.25f)
            ++expected_pairs;
    }
    if (arc_world_query(world, &query, 0, &query_results, &required)
            != ARC_STATUS_OK
        || required != expected_pairs)
        return 30;

    /* arc_world_query_any must agree with arc_world_query about emptiness. The
       managed wrappers always pass a filter, so the NULL-filter path is only
       reachable from C -- cover it here so it cannot rot untested. */
    {
        arc_bool any = 0;
        arc_shape empty_query = {0};
        empty_query.kind = ARC_SHAPE_AABB;
        empty_query.aabb.center = (arc_vec2){90000, 90000};
        empty_query.aabb.half_extents = (arc_vec2){1, 1};
        if (arc_world_query_any(world, &query, 0, &any) != ARC_STATUS_OK
            || (any != 0) != (expected_pairs > 0))
            return 31;
        if (arc_world_query_any(world, &empty_query, 0, &any) != ARC_STATUS_OK
            || any != 0)
            return 32;
    }
    arc_world_destroy(world);
    return 0;
}

/* Locks the generic sweep result for two rotated concave polygons.  Each shape
   decomposes into several indexed triangle proxies, so this catches changes to
   proxy preparation as well as the order of the fixed-point transform. */
static int sweep_proxy_preparation_regression(void)
{
    static const arc_vec2 mover_vertices[] = {
        {-1.5f, -1.5f}, {1.5f, -1.5f}, {1.5f, -0.5f},
        {-0.5f, -0.5f}, {-0.5f, 1.5f}, {-1.5f, 1.5f}
    };
    static const arc_vec2 target_vertices[] = {
        {-1.75f, -1.25f}, {1.75f, -1.25f}, {1.75f, 0.25f},
        {0.25f, 0.25f}, {0.25f, 1.25f}, {-1.75f, 1.25f}
    };
    arc_polygon* mover_polygon = arc_polygon_create(
        mover_vertices, (int32_t)(sizeof(mover_vertices) / sizeof(mover_vertices[0])));
    arc_polygon* target_polygon = arc_polygon_create(
        target_vertices, (int32_t)(sizeof(target_vertices) / sizeof(target_vertices[0])));
    arc_shape mover = {0}, target = {0};
    arc_sweep_hit hit;
    unsigned hash = 2166136261u;
    if (!mover_polygon || !target_polygon) {
        arc_polygon_release(mover_polygon);
        arc_polygon_release(target_polygon);
        return 38;
    }

    mover.kind = ARC_SHAPE_POLYGON;
    mover.polygon_rotation = 0x10000000u;
    mover.polygon_translation = (arc_vec2){-6.0f, 0.25f};
    mover.polygon = mover_polygon;
    target.kind = ARC_SHAPE_POLYGON;
    target.polygon_rotation = 0xe0000000u;
    target.polygon_translation = (arc_vec2){2.5f, -0.5f};
    target.polygon = target_polygon;
    hit = arc_moving_shape_vs_shape(&mover, (arc_vec2){11.0f, 0.75f}, &target);

    hash = add_hash(hash, (unsigned)hit.hit);
    hash = add_hash(hash, float_bits(hit.time));
    hash = add_hash(hash, float_bits(hit.normal.x));
    hash = add_hash(hash, float_bits(hit.normal.y));
    hash = add_hash(hash, float_bits(hit.point.x));
    hash = add_hash(hash, float_bits(hit.point.y));
    arc_polygon_release(mover_polygon);
    arc_polygon_release(target_polygon);
    if (!hit.hit) return 39;
    if (hash != 303570820u) {
        fprintf(stderr,
            "sweep proxy hash: expected 303570820, got %u\n", hash);
        return 40;
    }
    return 0;
}

/* Same-rotation/scale UpdateTransform calls use the translation-only path.  The
   capsule portion catches endpoint drift from translating already-rounded public
   floats at a magnitude where one Q24.8 unit is below the float ULP.  The polygon
   portion locks the more important allocation property: scaled immutable geometry
   is retained and reused instead of being rebuilt for a pure translation. */
static int same_pose_translation_regression(void)
{
    const arc_collision_filter filter = {
        ARC_CATEGORY_DEFAULT, ARC_CATEGORY_ALL
    };
    const arc_world_options options = {1.0f, 8, 8};
    arc_world* fast = arc_world_create(&options);
    arc_world* forced = arc_world_create(&options);
    arc_polygon* source_polygon = 0;
    arc_shape fast_result = {0}, forced_result = {0};
    arc_shape polygon_before = {0}, polygon_after = {0};
    arc_handle fast_handle, forced_handle, polygon_handle;
    int have_polygon_before = 0, have_polygon_after = 0;
    int result = 0;

    if (!fast || !forced) {
        result = 41;
        goto cleanup;
    }

    {
        arc_shape capsule = {0};
        arc_transform final_pose = {
            {65536.0078125f, 0.0f}, 0u, 1.0f
        };
        arc_transform detour_pose = final_pose;
        capsule.kind = ARC_SHAPE_CAPSULE;
        /* Midpoint = one fixed unit; cached local offsets are exactly -1/+1. */
        capsule.capsule = (arc_capsule){
            {0.0f, 0.0f}, {0.0078125f, 0.0f}, 0.00390625f
        };
        if (arc_world_add(fast, 1, &capsule, filter, 0, 1, &fast_handle)
                != ARC_STATUS_OK
            || arc_world_add(forced, 1, &capsule, filter, 0, 1, &forced_handle)
                != ARC_STATUS_OK) {
            result = 42;
            goto cleanup;
        }
        if (arc_world_update_transform(fast, fast_handle, &final_pose)
                != ARC_STATUS_OK) {
            result = 43;
            goto cleanup;
        }

        /* The final call is deliberately forced through full materialization,
           providing an independent integer-path oracle for the fast result. */
        detour_pose.rotation = 1u;
        if (arc_world_update_transform(forced, forced_handle, &detour_pose)
                != ARC_STATUS_OK
            || arc_world_update_transform(forced, forced_handle, &final_pose)
                != ARC_STATUS_OK
            || arc_world_get_shape(fast, fast_handle, &fast_result)
                != ARC_STATUS_OK
            || arc_world_get_shape(forced, forced_handle, &forced_result)
                != ARC_STATUS_OK) {
            result = 44;
            goto cleanup;
        }
        if (fast_result.kind != ARC_SHAPE_CAPSULE
            || forced_result.kind != ARC_SHAPE_CAPSULE
            || float_bits(fast_result.capsule.a.x)
                != float_bits(forced_result.capsule.a.x)
            || float_bits(fast_result.capsule.a.y)
                != float_bits(forced_result.capsule.a.y)
            || float_bits(fast_result.capsule.b.x)
                != float_bits(forced_result.capsule.b.x)
            || float_bits(fast_result.capsule.b.y)
                != float_bits(forced_result.capsule.b.y)
            || float_bits(fast_result.capsule.radius)
                != float_bits(forced_result.capsule.radius)
            || float_bits(fast_result.capsule.a.x) != float_bits(65536.0f)
            || float_bits(fast_result.capsule.b.x) != float_bits(65536.015625f)) {
            result = 45;
            goto cleanup;
        }
    }

    if (arc_world_clear(fast) != ARC_STATUS_OK) {
        result = 46;
        goto cleanup;
    }
    {
        static const arc_vec2 vertices[] = {
            {-2.0f, -1.0f}, {2.0f, -1.0f}, {0.5f, 2.0f}
        };
        arc_shape polygon = {0};
        arc_transform first_pose = {
            {10.0f, -3.0f}, 0x12345678u, 1.5f
        };
        arc_transform translated_pose = first_pose;
        source_polygon = arc_polygon_create(vertices, 3);
        if (!source_polygon) {
            result = 47;
            goto cleanup;
        }
        polygon.kind = ARC_SHAPE_POLYGON;
        polygon.polygon = source_polygon;
        polygon.polygon_rotation = 0x01020304u;
        polygon.polygon_translation = (arc_vec2){0.0f, 0.0f};
        if (arc_world_add(fast, 2, &polygon, filter, 1, 1, &polygon_handle)
                != ARC_STATUS_OK
            || arc_world_update_transform(fast, polygon_handle, &first_pose)
                != ARC_STATUS_OK
            || arc_world_get_shape(fast, polygon_handle, &polygon_before)
                != ARC_STATUS_OK) {
            result = 48;
            goto cleanup;
        }
        have_polygon_before = 1;
        translated_pose.position = (arc_vec2){25.0f, 7.0f};
        if (arc_world_update_transform(fast, polygon_handle, &translated_pose)
                != ARC_STATUS_OK
            || arc_world_get_shape(fast, polygon_handle, &polygon_after)
                != ARC_STATUS_OK) {
            result = 49;
            goto cleanup;
        }
        have_polygon_after = 1;
        if (polygon_before.polygon == source_polygon
            || polygon_after.polygon != polygon_before.polygon
            || polygon_after.polygon_rotation != polygon_before.polygon_rotation
            || float_bits(polygon_after.polygon_translation.x)
                != float_bits(25.0f)
            || float_bits(polygon_after.polygon_translation.y)
                != float_bits(7.0f)) {
            result = 50;
            goto cleanup;
        }
    }

cleanup:
    if (have_polygon_after) arc_polygon_release(polygon_after.polygon);
    if (have_polygon_before) arc_polygon_release(polygon_before.polygon);
    arc_polygon_release(source_polygon);
    arc_world_destroy(forced);
    arc_world_destroy(fast);
    return result;
}

int arc_run_c_api_smoke(void)
{
    if (arc_get_abi_version() != ARC_ABI_VERSION) return 1;
    if (sizeof(arc_status) != sizeof(int32_t)) return 13;
    if (sizeof(arc_manifold_fields) != sizeof(uint8_t)) return 14;

    arc_circle a = {{0.0f, 0.0f}, 1.0f};
    arc_circle b = {{1.5f, 0.0f}, 1.0f};
    arc_manifold manifold = arc_circle_vs_circle(a, b);
    if (!manifold.colliding || manifold.depth != 0.5f) return 2;

    {
        arc_shape first = {0}, second = {0};
        arc_manifold normal_depth, overlap_only;
        first.kind = ARC_SHAPE_CIRCLE;
        first.circle = a;
        second.kind = ARC_SHAPE_CIRCLE;
        second.circle = b;
        normal_depth = arc_shape_vs_shape(
            &first, &second, ARC_MANIFOLD_NORMAL_DEPTH);
        overlap_only = arc_shape_vs_shape(
            &first, &second, ARC_MANIFOLD_NONE);
        if (!normal_depth.colliding || normal_depth.depth != manifold.depth
            || normal_depth.normal.x != manifold.normal.x
            || normal_depth.normal.y != manifold.normal.y
            || normal_depth.contact.x != 0 || normal_depth.contact.y != 0)
            return 15;
        if (!overlap_only.colliding || overlap_only.depth != 0
            || overlap_only.normal.x != 0 || overlap_only.normal.y != 0
            || overlap_only.contact.x != 0 || overlap_only.contact.y != 0)
            return 16;
    }

    {
        arc_vec2 c1, c2;
        float distance_squared = arc_closest_points_segment_segment(
            (arc_vec2){0, 0}, (arc_vec2){2, 0},
            (arc_vec2){0, 3}, (arc_vec2){2, 3}, &c1, &c2);
        if (distance_squared != 9.0f || c1.y != 0.0f || c2.y != 3.0f)
            return 3;
        distance_squared = arc_closest_points_segment_segment(
            (arc_vec2){-1, -1}, (arc_vec2){-1, 1},
            (arc_vec2){2, 0}, (arc_vec2){1, 0}, &c1, &c2);
        if (distance_squared != 4.0f || c1.x != -1.0f || c1.y != 0.0f
            || c2.x != 1.0f || c2.y != 0.0f)
            return 11;
    }

    {
        arc_sweep_hit hit = arc_moving_circle_vs_circle(
            (arc_circle){{0, 0}, 1}, (arc_vec2){10, 0},
            (arc_circle){{5, 0}, 1});
        if (!hit.hit || fabsf(hit.time - 0.3f) > ARC_COLLISION_GRID_SIZE)
            return 4;
    }

    {
        arc_shape capsule = {0};
        arc_aabb bounds;
        capsule.kind = ARC_SHAPE_CAPSULE;
        capsule.capsule = (arc_capsule){{-0.0f, -1}, {0.0f, 1}, -0.0f};
        bounds = arc_shape_get_bounds(&capsule);
        if (float_bits(bounds.half_extents.x) != float_bits(0.0f))
            return 12;
    }

    arc_world_options options = {16.0f, 16, 16};
    arc_world* world = arc_world_create(&options);
    if (!world) return 5;
    arc_shape shape = {0};
    shape.kind = ARC_SHAPE_CIRCLE;
    shape.circle = a;
    arc_handle handle;
    arc_collision_filter filter = {ARC_CATEGORY_DEFAULT, ARC_CATEGORY_ALL};
    arc_status status = arc_world_add(world, 7, &shape, filter, 0, 1, &handle);
    if (status != ARC_STATUS_OK || !arc_world_is_valid(world, handle))
        return 6;

    shape.circle.center.x = 1.5f;
    {
        arc_handle second;
        int required = 0;
        const arc_candidate_pair* pairs = 0;
        status = arc_world_add(world, 8, &shape, filter, 1, 1, &second);
        if (status != ARC_STATUS_OK) return 7;
        status = arc_world_compute_pairs(world, &pairs, &required);
        if (status != ARC_STATUS_OK || required != 1) return 8;
        {
            arc_candidate_pair pair = pairs[0];
            if (pair.a.packed_entity_id % (1u << 28) != 7
                || pair.b.packed_entity_id % (1u << 28) != 8)
                return 9;
        }
        if (arc_world_remove(world, second) != ARC_STATUS_OK
            || arc_world_is_valid(world, second))
            return 10;
    }

    arc_world_destroy(world);
    {
        int result = broadphase_stress();
        if (result != 0) return result;
        result = locked_hash_smoke();
        if (result != 0) return result;
        result = sweep_proxy_preparation_regression();
        if (result != 0) return result;
        return same_pose_translation_regression();
    }
}
