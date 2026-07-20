// Regression locks for narrowphase cache optimizations. These cases deliberately
// exercise unrotated indexed triangle proxies and box contacts; cached geometry
// must preserve every public manifold bit.

#include "arccollision.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

uint32_t add_hash(uint32_t hash, uint32_t value) {
    return (hash ^ value) * 16777619u;
}

uint32_t float_bits(float value) {
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

uint32_t add_manifold(uint32_t hash, const arc_manifold& manifold) {
    hash = add_hash(hash, static_cast<uint32_t>(manifold.colliding));
    hash = add_hash(hash, float_bits(manifold.normal.x));
    hash = add_hash(hash, float_bits(manifold.normal.y));
    hash = add_hash(hash, float_bits(manifold.depth));
    hash = add_hash(hash, float_bits(manifold.contact.x));
    return add_hash(hash, float_bits(manifold.contact.y));
}

} // namespace

extern "C" int arc_run_collision_cache_tests(void) {
    constexpr arc_vec2 first_vertices[] = {
        {-2.0f, -2.0f}, {2.0f, -2.0f}, {2.0f, -1.0f},
        {-1.0f, -1.0f}, {-1.0f, 2.0f}, {-2.0f, 2.0f},
    };
    constexpr arc_vec2 second_vertices[] = {
        {-1.5f, -1.5f}, {1.5f, -1.5f}, {1.5f, 1.5f},
        {0.5f, 1.5f}, {0.5f, -0.5f}, {-1.5f, -0.5f},
    };
    arc_polygon* first_polygon = arc_polygon_create(
        first_vertices, static_cast<int32_t>(
            sizeof(first_vertices) / sizeof(first_vertices[0])));
    arc_polygon* second_polygon = arc_polygon_create(
        second_vertices, static_cast<int32_t>(
            sizeof(second_vertices) / sizeof(second_vertices[0])));
    if (!first_polygon || !second_polygon) {
        arc_polygon_release(first_polygon);
        arc_polygon_release(second_polygon);
        return 1;
    }

    arc_shape first{};
    first.kind = ARC_SHAPE_POLYGON;
    first.polygon_translation = {0.25f, -0.125f};
    first.polygon = first_polygon;
    arc_shape second{};
    second.kind = ARC_SHAPE_POLYGON;
    second.polygon_translation = {0.875f, 0.625f};
    second.polygon = second_polygon;

    uint32_t hash = 2166136261u;
    hash = add_hash(hash, static_cast<uint32_t>(arc_shapes_overlap(&first, &second)));
    hash = add_manifold(
        hash, arc_shape_vs_shape(&first, &second, ARC_MANIFOLD_ALL));

    arc_shape aabb{};
    aabb.kind = ARC_SHAPE_AABB;
    aabb.aabb = {{-0.25f, 0.125f}, {1.75f, 0.875f}};
    arc_shape obb{};
    obb.kind = ARC_SHAPE_OBB;
    obb.obb = {{0.875f, 0.375f}, {1.375f, 0.625f}, 0x18000000u};
    arc_shape second_obb{};
    second_obb.kind = ARC_SHAPE_OBB;
    second_obb.obb = {{-0.625f, -0.25f}, {1.125f, 0.75f}, 0xe8000000u};
    arc_shape capsule{};
    capsule.kind = ARC_SHAPE_CAPSULE;
    capsule.capsule = {{-1.25f, -0.75f}, {1.5f, 0.625f}, 0.5f};

    hash = add_manifold(hash, arc_shape_vs_shape(&aabb, &obb, ARC_MANIFOLD_ALL));
    hash = add_manifold(hash, arc_shape_vs_shape(&obb, &aabb, ARC_MANIFOLD_ALL));
    hash = add_manifold(
        hash, arc_shape_vs_shape(&obb, &second_obb, ARC_MANIFOLD_ALL));
    hash = add_manifold(
        hash, arc_shape_vs_shape(&capsule, &obb, ARC_MANIFOLD_ALL));

    arc_polygon_release(first_polygon);
    arc_polygon_release(second_polygon);
    constexpr uint32_t expected_hash = 1692970966u;
    if (hash != expected_hash) {
        std::fprintf(stderr, "collision cache hash: got %u\n", hash);
        return 2;
    }
    return 0;
}
