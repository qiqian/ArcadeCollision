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
    arc_handle query[6];
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
        if (arc_world_query(world, &all, 0, query, 6, &query_count)
            != ARC_STATUS_OK) return 34;
    }
    for (i = 0; i < query_count; ++i) {
        hash = add_hash(hash, query[i].packed_entity_id & 0x0fffffffu);
    }
    if (arc_world_compute_pairs(world, pairs, 15, &pair_count)
        != ARC_STATUS_OK) return 35;
    for (i = 0; i < pair_count; ++i) {
        arc_contact_pair contact;
        arc_bool colliding;
        hash = add_hash(hash, pairs[i].a.packed_entity_id & 0x0fffffffu);
        hash = add_hash(hash, pairs[i].b.packed_entity_id & 0x0fffffffu);
        if (arc_world_try_contact_pair(
                world, pairs[i], &contact, &colliding) != ARC_STATUS_OK)
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
    arc_candidate_pair* pairs;
    arc_handle query_results[count];
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
            arc_shape shape = {0};
            circles[i].center.x += i % 14 == 0 ? 4.5f : 0.75f;
            shape.kind = ARC_SHAPE_CIRCLE;
            shape.circle = circles[i];
            if (arc_world_update(world, handles[i], &shape) != ARC_STATUS_OK)
                return 22;
        }
        if (is_static[i] && i % 17 == 0) {
            arc_shape shape = {0};
            circles[i].center.y += 1.0f;
            shape.kind = ARC_SHAPE_CIRCLE;
            shape.circle = circles[i];
            if (arc_world_update(world, handles[i], &shape) != ARC_STATUS_OK)
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
    if (arc_world_compute_pairs(world, 0, 0, &required)
            != (expected_pairs ? ARC_STATUS_BUFFER_TOO_SMALL : ARC_STATUS_OK)
        || required != expected_pairs)
        return 26;
    pairs = (arc_candidate_pair*)malloc(
        (size_t)(required ? required : 1) * sizeof(*pairs));
    if (!pairs) return 27;
    if (arc_world_compute_pairs(world, pairs, required, &required)
        != ARC_STATUS_OK) return 28;
    for (i = 1; i < required; ++i) {
        unsigned previous_a = pairs[i - 1].a.packed_entity_id & 0x0fffffffu;
        unsigned current_a = pairs[i].a.packed_entity_id & 0x0fffffffu;
        unsigned previous_b = pairs[i - 1].b.packed_entity_id & 0x0fffffffu;
        unsigned current_b = pairs[i].b.packed_entity_id & 0x0fffffffu;
        if (previous_a > current_a
            || (previous_a == current_a && previous_b > current_b))
            return 29;
    }
    free(pairs);

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
    if (arc_world_query(world, &query, 0, query_results, count, &required)
            != ARC_STATUS_OK
        || required != expected_pairs)
        return 30;
    arc_world_destroy(world);
    return 0;
}

int main(void)
{
    if (arc_get_abi_version() != ARC_ABI_VERSION) return 1;
    if (sizeof(arc_status) != sizeof(int32_t)) return 13;

    arc_circle a = {{0.0f, 0.0f}, 1.0f};
    arc_circle b = {{1.5f, 0.0f}, 1.0f};
    arc_manifold manifold = arc_circle_vs_circle(a, b);
    if (!manifold.colliding || manifold.depth != 0.5f) return 2;

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
        status = arc_world_add(world, 8, &shape, filter, 1, 1, &second);
        if (status != ARC_STATUS_OK) return 7;
        status = arc_world_compute_pairs(world, 0, 0, &required);
        if (status != ARC_STATUS_BUFFER_TOO_SMALL || required != 1) return 8;
        {
            arc_candidate_pair pair;
            status = arc_world_compute_pairs(world, &pair, 1, &required);
            if (status != ARC_STATUS_OK || required != 1
                || pair.a.packed_entity_id % (1u << 28) != 7
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
        return result != 0 ? result : locked_hash_smoke();
    }
}
