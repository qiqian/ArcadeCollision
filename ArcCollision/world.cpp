// The collision world and the bulk of the C ABI. A world owns a slot array of
// colliders (each a shape + filter + broadphase proxy, addressed by a generational
// arc_handle), the hybrid broadphase, and reusable scratch buffers. Handles carry a
// world id so a stale handle from another world is rejected. This file also hosts
// the standalone broadphase C API (arc_dynamic_tree / arc_static_bvh) at the end.
// World bulk operations publish borrowed views of world-owned result vectors;
// standalone broadphase arrays retain the two-call output/capacity protocol.
#include "broadphase.h"

#include <cstring>
#include <limits>
#include <mutex>
#include <new>

namespace {

constexpr uint32_t IndexMask = (uint32_t{1} << 20) - 1;
constexpr uint32_t GenerationMax = (uint32_t{1} << 12) - 1;
constexpr uint32_t EntityMask = (uint32_t{1} << 28) - 1;

uint32_t handle_index(arc_handle handle) {
    return handle.packed_index & IndexMask;
}
uint32_t handle_generation(arc_handle handle) {
    return handle.packed_index >> 20;
}
uint32_t handle_world(arc_handle handle) {
    return handle.packed_entity_id >> 28;
}
int32_t handle_entity(arc_handle handle) {
    return static_cast<int32_t>(handle.packed_entity_id & EntityMask);
}

bool handle_less(arc_handle a, arc_handle b) {
    if (handle_entity(a) != handle_entity(b))
        return handle_entity(a) < handle_entity(b);
    if (handle_index(a) != handle_index(b))
        return handle_index(a) < handle_index(b);
    return handle_generation(a) < handle_generation(b);
}

struct StoredShape {
    arc_shape value{};
    StoredShape() = default;
    explicit StoredShape(const arc_shape& shape) : value(shape) {
        arc::retain_shape(value);
    }
    StoredShape(const StoredShape& other) : value(other.value) {
        arc::retain_shape(value);
    }
    StoredShape(StoredShape&& other) noexcept : value(other.value) {
        other.value = {};
    }
    StoredShape& operator=(const StoredShape& other) {
        if (this != &other) {
            arc::retain_shape(other.value);
            arc::release_shape(value);
            value = other.value;
        }
        return *this;
    }
    StoredShape& operator=(StoredShape&& other) noexcept {
        if (this != &other) {
            arc::release_shape(value);
            value = other.value;
            other.value = {};
        }
        return *this;
    }
    ~StoredShape() { arc::release_shape(value); }
};

struct FixedTransform {
    arc::Vec position;
    uint32_t rotation = 0;
    int64_t scale16 = arc::TOne;
};

// Immutable local collider geometry as a tagged union. Primitive payloads are
// fixed-point only; polygon geometry is shared and reference-counted. Capsule is
// the largest member (40 bytes), so LocalShape stays 48 bytes on supported
// 64-bit targets instead of reserving storage for every shape representation.
struct LocalShape {
    struct CircleData { int64_t radius = 0; };
    struct BoxData { arc::Vec half; };
    struct ObbData { arc::Vec half; uint32_t base_angle = 0; };
    struct CapsuleData { arc::Vec a; arc::Vec b; int64_t radius = 0; };
    struct PolygonData {
        arc_polygon* geometry = nullptr;
        uint32_t base_angle = 0;
    };

    union Payload {
        CircleData circle;
        BoxData aabb;
        ObbData obb;
        CapsuleData capsule;
        PolygonData polygon;

        Payload() : circle{} {}
    } payload;

    int32_t kind = ARC_SHAPE_CIRCLE;

    LocalShape() = default;

    LocalShape(const LocalShape& other)
        : payload(other.payload), kind(other.kind) {
        retain_polygon();
    }

    LocalShape(LocalShape&& other) noexcept
        : payload(other.payload), kind(other.kind) {
        other.kind = ARC_SHAPE_CIRCLE;
        other.payload.circle = {};
    }

    LocalShape& operator=(const LocalShape& other) {
        if (this != &other) {
            if (other.kind == ARC_SHAPE_POLYGON && other.payload.polygon.geometry)
                arc_polygon_retain(other.payload.polygon.geometry);
            release_polygon();
            payload = other.payload;
            kind = other.kind;
        }
        return *this;
    }

    LocalShape& operator=(LocalShape&& other) noexcept {
        if (this != &other) {
            release_polygon();
            payload = other.payload;
            kind = other.kind;
            other.kind = ARC_SHAPE_CIRCLE;
            other.payload.circle = {};
        }
        return *this;
    }

    ~LocalShape() { release_polygon(); }

    void set_polygon(arc_polygon* geometry, uint32_t base_angle) {
        // Retain first so replacing a polygon with the same geometry is safe.
        if (geometry) arc_polygon_retain(geometry);
        release_polygon();
        kind = ARC_SHAPE_POLYGON;
        payload.polygon = {geometry, base_angle};
    }

private:
    void retain_polygon() const {
        if (kind == ARC_SHAPE_POLYGON && payload.polygon.geometry)
            arc_polygon_retain(payload.polygon.geometry);
    }

    void release_polygon() const {
        if (kind == ARC_SHAPE_POLYGON && payload.polygon.geometry)
            arc_polygon_release(payload.polygon.geometry);
    }
};

static_assert(sizeof(LocalShape) == 48,
              "LocalShape union layout must remain cache-dense");

struct Slot {
    StoredShape shape;       // materialized world shape (for narrowphase)
    LocalShape local;        // immutable fixed local form (precomputed on add)
    FixedTransform transform; // current integer placement of `local`
    arc::Bounds bounds;
    arc_collision_filter filter{};
    int32_t entity_id = 0;
    int tree_proxy = -1;
    int next_free = -1;
    uint16_t generation = 0;
    // These flags are always accessed independently of their physical layout,
    // so compiler-specific bit-field packing does not affect world behaviour.
    bool active : 1;
    bool enabled : 1;
    bool is_static : 1;

    // Bit-field default member initializers require C++20. Keep the library at
    // C++17 and initialize them explicitly instead.
    Slot() : active(false), enabled(false), is_static(false) {}
};

std::mutex world_id_mutex;
bool world_ids[ARC_MAX_WORLD_COUNT + 1]{};
std::vector<uint16_t> generation_tables[ARC_MAX_WORLD_COUNT + 1];

} // namespace

struct arc_world {
    explicit arc_world(uint32_t world_id, const arc_world_options& options)
        : id(world_id), broadphase(options.fat_margin) {
        slots.reserve(static_cast<size_t>(std::max(16, options.initial_collider_capacity)));
        candidates.reserve(static_cast<size_t>(std::max(16, options.initial_collider_capacity)));
        pairs.reserve(static_cast<size_t>(options.initial_pair_capacity));
        pair_values.reserve(static_cast<size_t>(options.initial_pair_capacity));
        query_values.reserve(static_cast<size_t>(
            std::max(16, options.initial_collider_capacity)));
        cast_values.reserve(static_cast<size_t>(
            std::max(16, options.initial_collider_capacity)));
        broadphase.ensure_capacity(std::max(16, options.initial_collider_capacity));
    }

    uint32_t id;
    arc::SpatialHash broadphase;
    std::vector<Slot> slots;
    std::vector<int> candidates;
    std::vector<std::pair<int, int>> pairs;
    std::vector<arc_candidate_pair> pair_values;
    std::vector<arc_handle> query_values;
    std::vector<arc_world_cast_hit> cast_values;
    // Scratch for arc_world_query_batch. Sparse batches write directly to
    // query_values in input order. Dense batches are Morton-sorted so each packet
    // of four is spatially coherent; those results land in morton_handles and are
    // scattered back to query_values. Counts/slice-starts use the original order.
    std::vector<int32_t> query_batch_counts;   // per input query (published)
    std::vector<arc::Bounds> query_batch_bounds;
    std::vector<uint64_t> query_morton;        // per input query
    std::vector<int> query_order;              // input indices, Morton-sorted
    std::vector<int32_t> query_slice_start;    // per input query: start in morton_handles
    std::vector<arc_handle> morton_handles;    // results in Morton (packet) order
    std::vector<int> packet_candidates[4];
    std::vector<arc::PacketFrame> packet_stack;
    int free_list = -1;
    int active_count = 0;
    int enabled_count = 0;
    int dynamic_count = 0;
};

namespace {

bool valid(const arc_world* world, arc_handle handle) {
    if (!world || handle_world(handle) != world->id) return false;
    const uint32_t index = handle_index(handle);
    return index < world->slots.size()
        && world->slots[index].active
        && world->slots[index].generation == handle_generation(handle);
}

Slot* get_slot(arc_world* world, arc_handle handle) {
    return valid(world, handle)
        ? &world->slots[static_cast<size_t>(handle_index(handle))] : nullptr;
}
const Slot* get_slot(const arc_world* world, arc_handle handle) {
    return valid(world, handle)
        ? &world->slots[static_cast<size_t>(handle_index(handle))] : nullptr;
}

arc_handle make_handle(const arc_world* world, size_t index) {
    const Slot& slot = world->slots[index];
    return {
        (static_cast<uint32_t>(slot.generation) << 20)
            | static_cast<uint32_t>(index),
        (world->id << 28) | static_cast<uint32_t>(slot.entity_id),
    };
}

arc_candidate_pair make_pair(const arc_world* world, int a, int b) {
    arc_handle first = make_handle(world, static_cast<size_t>(a));
    arc_handle second = make_handle(world, static_cast<size_t>(b));
    if (handle_less(second, first)) std::swap(first, second);
    return {first, second};
}

bool pair_less(const arc_candidate_pair& a, const arc_candidate_pair& b) {
    return handle_less(a.a, b.a)
        || (!handle_less(b.a, a.a) && handle_less(a.b, b.b));
}

bool cast_less(const arc_world_cast_hit& a, const arc_world_cast_hit& b) {
    return a.hit.time < b.hit.time
        || (a.hit.time == b.hit.time && handle_less(a.handle, b.handle));
}

bool query_slot(
    const Slot& slot, const arc_collision_filter* filter) {
    return slot.active && slot.enabled
        && (!filter || arc::filter_allows(*filter, slot.filter));
}

// Spread the low 32 bits of v into even bit positions of a 64-bit word.
uint64_t spread_bits(uint32_t v) {
    uint64_t x = v;
    x = (x | (x << 16)) & 0x0000FFFF0000FFFFULL;
    x = (x | (x << 8))  & 0x00FF00FF00FF00FFULL;
    x = (x | (x << 4))  & 0x0F0F0F0F0F0F0F0FULL;
    x = (x | (x << 2))  & 0x3333333333333333ULL;
    x = (x | (x << 1))  & 0x5555555555555555ULL;
    return x;
}

// 64-bit Morton code of a bounds centre: spatially near centres get near codes,
// so sorting queries by it makes each group of four coherent. The signed 24.8
// centre is biased to an order-preserving unsigned value before interleaving.
uint64_t morton_code(const arc::Bounds& bounds) {
    const uint32_t x = static_cast<uint32_t>(bounds.center_x()) ^ 0x80000000u;
    const uint32_t y = static_cast<uint32_t>(bounds.center_y()) ^ 0x80000000u;
    return spread_bits(x) | (spread_bits(y) << 1);
}

arc::Bounds swept_bounds(const arc_shape& mover, arc_vec2 motion) {
    const arc::Bounds start = arc::shape_bounds(mover);
    const arc::Bounds end = start.translated(
        arc::from_float(motion.x), arc::from_float(motion.y));
    return arc::Bounds::unite(start, end);
}

// --- Integer transform placement (mirrors ArcCollision.Ref/Transform.cs) ------
// Public transform floats are quantized once: position -> Q24.8, scale -> Q16.16.
// Every composition, scale and rotation below is integer-only. Angle32 rotation
// uses the same Q1.30 CORDIC axes as collision/SAT.

int64_t float_bits_to_fixed(float value, int fixed_shift, int64_t max_raw) {
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    const bool negative = (bits & 0x80000000u) != 0;
    const int exponent_bits = static_cast<int>((bits >> 23) & 0xffu);
    const uint32_t fraction_bits = bits & 0x007fffffu;
    if (exponent_bits == 0xff)
        throw std::out_of_range("Transform value must be finite.");
    if (exponent_bits == 0 && fraction_bits == 0) return 0;

    uint64_t significand;
    int exponent;
    if (exponent_bits == 0) {
        significand = fraction_bits;
        exponent = -149 + fixed_shift;
    } else {
        significand = 0x00800000u | fraction_bits;
        exponent = exponent_bits - 127 - 23 + fixed_shift;
    }

    uint64_t rounded;
    if (exponent >= 0) {
        if (exponent >= 63
            || significand > (static_cast<uint64_t>(max_raw) >> exponent))
            throw std::out_of_range("Fixed-point transform value is too large.");
        rounded = significand << exponent;
    } else {
        const int right = -exponent;
        if (right >= 64) {
            rounded = 0;
        } else {
            rounded = significand >> right;
            const uint64_t mask = (uint64_t{1} << right) - 1;
            const uint64_t remainder = significand & mask;
            const uint64_t half = uint64_t{1} << (right - 1);
            if (remainder > half || (remainder == half && (rounded & 1) != 0))
                ++rounded;
        }
        if (rounded > static_cast<uint64_t>(max_raw))
            throw std::out_of_range("Fixed-point transform value is too large.");
    }
    const int64_t result = static_cast<int64_t>(rounded);
    return negative ? -result : result;
}

FixedTransform fixed_transform_from_public(const arc_transform& value) {
    if (!arc::valid_vec(value.position) || value.scale < 0.0f) {
        throw std::out_of_range("Invalid transform.");
    }
    return {
        {float_bits_to_fixed(value.position.x, arc::FxShift,
                             std::numeric_limits<int64_t>::max()),
         float_bits_to_fixed(value.position.y, arc::FxShift,
                             std::numeric_limits<int64_t>::max())},
        value.rotation,
        float_bits_to_fixed(value.scale, arc::TShift,
            std::numeric_limits<int64_t>::max() / arc::TOne),
    };
}

int64_t mul_scale(int64_t value, int64_t scale16) {
    if (scale16 < 0) throw std::out_of_range("Scale must be non-negative.");
    // Translation-only updates dominate typical game worlds. Preserve the
    // already-quantized local value instead of entering the wide multiply path.
    if (scale16 == arc::TOne) return value;
    const uint64_t magnitude = arc::magnitude(value);
    const uint64_t whole = static_cast<uint64_t>(scale16) >> arc::TShift;
    const uint64_t fraction = static_cast<uint64_t>(scale16) & (arc::TOne - 1);
    if (whole != 0
        && magnitude > static_cast<uint64_t>(INT64_MAX) / whole)
        throw std::out_of_range("Scaled value is too large.");
    uint64_t result = magnitude * whole;
    const uint64_t fractional =
        (magnitude * fraction + (arc::TOne >> 1)) >> arc::TShift;
    if (result > static_cast<uint64_t>(INT64_MAX) - fractional)
        throw std::out_of_range("Scaled value is too large.");
    result += fractional;
    const int64_t signed_result = static_cast<int64_t>(result);
    return value < 0 ? -signed_result : signed_result;
}

int64_t mul_scales(int64_t a, int64_t b) {
    if (a < 0 || b < 0) throw std::out_of_range("Scale must be non-negative.");
    // In particular, make a translation/rotation-only delta free of scale math.
    if (b == arc::TOne) return a;
    if (a == arc::TOne) return b;
    const uint64_t whole = static_cast<uint64_t>(b) >> arc::TShift;
    const uint64_t fraction = static_cast<uint64_t>(b) & (arc::TOne - 1);
    if (whole != 0 && static_cast<uint64_t>(a) > UINT64_C(0x7fffffffffffffff) / whole)
        throw std::out_of_range("Composed scale is too large.");
    const uint64_t result = static_cast<uint64_t>(a) * whole;
    const uint64_t fractional =
        (static_cast<uint64_t>(a) * fraction + (arc::TOne >> 1)) >> arc::TShift;
    const uint64_t limit = static_cast<uint64_t>(INT64_MAX / arc::TOne);
    if (result > limit || fractional > limit - result)
        throw std::out_of_range("Composed scale is too large.");
    return static_cast<int64_t>(result + fractional);
}

arc::Vec scale_vec(const arc::Vec& value, int64_t scale16) {
    // Check once for the whole vector so the identity path performs no
    // per-component multiply, rounding, or overflow bookkeeping.
    if (scale16 == arc::TOne) return value;
    return {mul_scale(value.x, scale16), mul_scale(value.y, scale16)};
}

LocalShape to_local(const arc_shape& shape, FixedTransform& initial) {
    LocalShape local;
    switch (shape.kind) {
    case ARC_SHAPE_CIRCLE: {
        local.kind = ARC_SHAPE_CIRCLE;
        initial = {arc::Vec::from(shape.circle.center), 0u, arc::TOne};
        local.payload.circle = {arc::from_float(shape.circle.radius)};
        break;
    }
    case ARC_SHAPE_AABB: {
        local.kind = ARC_SHAPE_AABB;
        initial = {arc::Vec::from(shape.aabb.center), 0u, arc::TOne};
        local.payload.aabb = {arc::Vec::from(shape.aabb.half_extents)};
        break;
    }
    case ARC_SHAPE_OBB: {
        local.kind = ARC_SHAPE_OBB;
        initial = {arc::Vec::from(shape.obb.center), 0u, arc::TOne};
        local.payload.obb = {
            arc::Vec::from(shape.obb.half_extents), shape.obb.angle};
        break;
    }
    case ARC_SHAPE_CAPSULE: {
        local.kind = ARC_SHAPE_CAPSULE;
        const arc::Vec a = arc::Vec::from(shape.capsule.a);
        const arc::Vec b = arc::Vec::from(shape.capsule.b);
        const arc::Vec mid{(a.x + b.x) / 2, (a.y + b.y) / 2};
        initial = {mid, 0u, arc::TOne};
        local.payload.capsule = {
            a - mid, b - mid, arc::from_float(shape.capsule.radius)};
        break;
    }
    case ARC_SHAPE_POLYGON: {
        initial = {arc::Vec::from(shape.polygon_translation), 0u, arc::TOne};
        local.set_polygon(shape.polygon, shape.polygon_rotation);
        break;
    }
    default:
        break;
    }
    return local;
}

arc::Bounds fixed_polygon_bounds(
    const arc_polygon& polygon, const arc::Vec& translation, uint32_t rotation) {
    if (rotation == 0) return polygon.bounds.translated(translation.x, translation.y);
    const arc::Axis axis_x = arc::Axis::from_angle(rotation);
    const arc::Axis axis_y = axis_x.perpendicular();
    auto transform = [&](const arc::Vec& vertex) {
        return translation + axis_x.scale(vertex.x) + axis_y.scale(vertex.y);
    };
    arc::Vec first = transform(polygon.fixed_vertices[0]);
    arc::Bounds bounds{static_cast<int32_t>(first.x), static_cast<int32_t>(first.y),
                       static_cast<int32_t>(first.x), static_cast<int32_t>(first.y)};
    for (size_t i = 1; i < polygon.fixed_vertices.size(); ++i) {
        const arc::Vec vertex = transform(polygon.fixed_vertices[i]);
        bounds.min_x = std::min(bounds.min_x, static_cast<int32_t>(vertex.x));
        bounds.min_y = std::min(bounds.min_y, static_cast<int32_t>(vertex.y));
        bounds.max_x = std::max(bounds.max_x, static_cast<int32_t>(vertex.x));
        bounds.max_y = std::max(bounds.max_y, static_cast<int32_t>(vertex.y));
    }
    return bounds;
}

struct MaterializedShape {
    arc_shape shape{};
    arc::Bounds bounds{};
    arc_polygon* owned_polygon = nullptr;
};

MaterializedShape materialize(const LocalShape& local, const FixedTransform& t) {
    MaterializedShape result;
    result.shape.kind = local.kind;
    const arc_vec2 position = t.position.to_public();
    switch (local.kind) {
    case ARC_SHAPE_CIRCLE: {
        const int64_t radius = mul_scale(local.payload.circle.radius, t.scale16);
        result.shape.circle = {position, arc::to_float(radius)};
        result.bounds = {static_cast<int32_t>(t.position.x - radius),
                         static_cast<int32_t>(t.position.y - radius),
                         static_cast<int32_t>(t.position.x + radius),
                         static_cast<int32_t>(t.position.y + radius)};
        break;
    }
    case ARC_SHAPE_AABB: {
        const arc::Vec half = scale_vec(local.payload.aabb.half, t.scale16);
        result.shape.aabb = {position, half.to_public()};
        result.bounds = {static_cast<int32_t>(t.position.x - half.x),
                         static_cast<int32_t>(t.position.y - half.y),
                         static_cast<int32_t>(t.position.x + half.x),
                         static_cast<int32_t>(t.position.y + half.y)};
        break;
    }
    case ARC_SHAPE_OBB: {
        const arc::Vec half = scale_vec(local.payload.obb.half, t.scale16);
        const uint32_t angle = local.payload.obb.base_angle + t.rotation;
        result.shape.obb = {position, half.to_public(), angle};
        if (angle == 0) {
            // The final angle matters here: an identity transform must not erase
            // an OBB's authored base angle.
            result.bounds = {static_cast<int32_t>(t.position.x - half.x),
                             static_cast<int32_t>(t.position.y - half.y),
                             static_cast<int32_t>(t.position.x + half.x),
                             static_cast<int32_t>(t.position.y + half.y)};
            break;
        }
        const arc::Axis axis_x = arc::Axis::from_angle(angle);
        const arc::Axis axis_y = axis_x.perpendicular();
        const int64_t extent_x = arc::ceil_axis_positive(
            arc::magnitude(axis_x.x) * half.x
            + arc::magnitude(axis_y.x) * half.y);
        const int64_t extent_y = arc::ceil_axis_positive(
            arc::magnitude(axis_x.y) * half.x
            + arc::magnitude(axis_y.y) * half.y);
        result.bounds = {static_cast<int32_t>(t.position.x - extent_x),
                         static_cast<int32_t>(t.position.y - extent_y),
                         static_cast<int32_t>(t.position.x + extent_x),
                         static_cast<int32_t>(t.position.y + extent_y)};
        break;
    }
    case ARC_SHAPE_CAPSULE: {
        arc::Vec local_a = local.payload.capsule.a;
        arc::Vec local_b = local.payload.capsule.b;
        int64_t radius = local.payload.capsule.radius;
        if (t.scale16 != arc::TOne) {
            // Test identity once for all three capsule dimensions.
            local_a = scale_vec(local_a, t.scale16);
            local_b = scale_vec(local_b, t.scale16);
            radius = mul_scale(radius, t.scale16);
        }
        if (t.rotation != 0) {
            // A capsule has two local endpoints. Build the integer rotation
            // basis once and reuse it for both instead of running CORDIC twice.
            const arc::Axis axis_x = arc::Axis::from_angle(t.rotation);
            const arc::Axis axis_y = axis_x.perpendicular();
            local_a = axis_x.scale(local_a.x) + axis_y.scale(local_a.y);
            local_b = axis_x.scale(local_b.x) + axis_y.scale(local_b.y);
        }
        const arc::Vec a = t.position + local_a;
        const arc::Vec b = t.position + local_b;
        result.shape.capsule = {a.to_public(), b.to_public(), arc::to_float(radius)};
        result.bounds = {
            static_cast<int32_t>(std::min(a.x, b.x) - radius),
            static_cast<int32_t>(std::min(a.y, b.y) - radius),
            static_cast<int32_t>(std::max(a.x, b.x) + radius),
            static_cast<int32_t>(std::max(a.y, b.y) + radius)};
        break;
    }
    case ARC_SHAPE_POLYGON: {
        result.shape.polygon_rotation =
            local.payload.polygon.base_angle + t.rotation;
        result.shape.polygon_translation = position;
        arc_polygon* polygon = local.payload.polygon.geometry;
        if (t.scale16 != arc::TOne) {
            std::vector<arc::Vec> vertices(polygon->fixed_vertices.size());
            for (size_t i = 0; i < vertices.size(); ++i)
                vertices[i] = scale_vec(polygon->fixed_vertices[i], t.scale16);
            result.owned_polygon = arc::polygon_from_fixed(vertices);
            polygon = result.owned_polygon;
        }
        result.shape.polygon = polygon;
        if (polygon)
            result.bounds = fixed_polygon_bounds(
                *polygon, t.position, result.shape.polygon_rotation);
        break;
    }
    default:
        break;
    }
    return result;
}

FixedTransform compose_transform(
    const FixedTransform& current, const FixedTransform& delta) {
    return {current.position + delta.position,
            current.rotation + delta.rotation,
            mul_scales(current.scale16, delta.scale16)};
}

void append_query(
    arc_world* world, const arc::Bounds& bounds,
    const arc_collision_filter* filter) {
    for (int index : world->candidates) {
        const Slot& slot = world->slots[static_cast<size_t>(index)];
        if (query_slot(slot, filter) && slot.bounds.overlaps(bounds))
            world->query_values.push_back(
                make_handle(world, static_cast<size_t>(index)));
    }
}

// Resolve one prevalidated query into the shared input-order result buffer.
// Keeping this loop native amortizes the C ABI transition without paying the
// Morton sort and packet bookkeeping that dominate genuinely sparse batches.
int32_t append_scalar_batch_query(
    arc_world* world, const arc::Bounds& bounds,
    const arc_collision_filter* filter) {
    const size_t start = world->query_values.size();
    world->candidates.clear();
    world->broadphase.query_dynamic(bounds, world->candidates);
    append_query(world, bounds, filter);
    world->candidates.clear();
    world->broadphase.query_static(bounds, world->candidates);
    append_query(world, bounds, filter);
    std::sort(
        world->query_values.begin() + static_cast<ptrdiff_t>(start),
        world->query_values.end(), handle_less);
    return static_cast<int32_t>(world->query_values.size() - start);
}

int sampled_query_matches(
    arc_world* world, const arc::Bounds& bounds,
    const arc_collision_filter* filter) {
    int matches = 0;
    world->candidates.clear();
    world->broadphase.query_dynamic(bounds, world->candidates);
    world->broadphase.query_static(bounds, world->candidates);
    for (int index : world->candidates) {
        const Slot& slot = world->slots[static_cast<size_t>(index)];
        if (query_slot(slot, filter) && slot.bounds.overlaps(bounds))
            ++matches;
    }
    return matches;
}

// Morton sorting and four-lane traversal pay off when queries have enough local
// tree work for spatially adjacent packets to share. Probe a fixed, evenly spaced
// sample first: small batches stay scalar, and dense batches retain the
// Morton-sorted SIMD path. Light batches only use input-order packets when their
// groups of four are spatially coherent: scattered lanes share almost no nodes,
// so the 4-wide union traversal costs measurably more than four scalar descents.
// This changes only traversal order; published per-query handles remain identical.
enum class QueryBatchPath {
    Scalar,
    DirectPacket,
    MortonPacket,
};

// True when one packet's four queries would descend a mostly shared subtree:
// their united box is within a few member sizes, so per-node SIMD tests replace
// up to four scalar tests instead of adding union-traversal work. Scattered
// groups unite to world-scale boxes (hundreds of member sizes), so the
// threshold only has to separate "bundled" from "unrelated".
bool coherent_packet_group(const arc::Bounds* group) {
    arc::Bounds united = group[0];
    int64_t largest = group[0].perimeter();
    for (int lane = 1; lane < 4; ++lane) {
        united = arc::Bounds::unite(united, group[lane]);
        largest = std::max(largest, group[lane].perimeter());
    }
    return united.perimeter() <= 4 * largest;
}

QueryBatchPath select_query_batch_path(
    arc_world* world, const std::vector<arc::Bounds>& bounds,
    const arc_collision_filter* filter) {
    constexpr size_t min_packet_queries = 64;
    constexpr size_t probe_count = 8;
    constexpr int min_average_matches = 4;
    if (bounds.size() < min_packet_queries) return QueryBatchPath::Scalar;

    const size_t samples = std::min(probe_count, bounds.size());
    int matches = 0;
    for (size_t sample = 0; sample < samples; ++sample) {
        const size_t index = sample * bounds.size() / samples;
        matches += sampled_query_matches(world, bounds[index], filter);
    }
    if (matches >= static_cast<int>(samples) * min_average_matches)
        return QueryBatchPath::MortonPacket;

    const size_t groups = bounds.size() / 4;
    size_t coherent = 0;
    for (size_t sample = 0; sample < samples; ++sample)
        if (coherent_packet_group(&bounds[(sample * groups / samples) * 4]))
            ++coherent;
    return coherent * 2 >= samples
        ? QueryBatchPath::DirectPacket
        : QueryBatchPath::Scalar;
}

void append_casts(
    arc_world* world, const arc_shape& mover, arc_vec2 motion,
    const arc_collision_filter* filter) {
    for (int index : world->candidates) {
        Slot& slot = world->slots[static_cast<size_t>(index)];
        if (!query_slot(slot, filter)) continue;
        const arc_sweep_hit hit =
            arc::sweep_shapes(mover, arc::Vec::from(motion), slot.shape.value)
                .to_public(motion);
        if (hit.hit)
            world->cast_values.push_back({
                make_handle(world, static_cast<size_t>(index)), hit});
    }
}

void find_closest_cast(
    arc_world* world, const arc_shape& mover, arc_vec2 motion,
    const arc_collision_filter* filter,
    bool& found, arc_world_cast_hit& closest) {
    for (int index : world->candidates) {
        Slot& slot = world->slots[static_cast<size_t>(index)];
        if (!query_slot(slot, filter)) continue;
        const arc_sweep_hit hit =
            arc::sweep_shapes(mover, arc::Vec::from(motion), slot.shape.value)
                .to_public(motion);
        if (!hit.hit) continue;
        const arc_world_cast_hit candidate{
            make_handle(world, static_cast<size_t>(index)), hit};
        if (!found || cast_less(candidate, closest)) {
            closest = candidate;
            found = true;
        }
    }
}

arc_status ensure_world(const arc_world* world) {
    if (world) return ARC_STATUS_OK;
    arc::set_error("World is null.");
    return ARC_STATUS_INVALID_ARGUMENT;
}

// Publish a read-only view of a result vector owned by the world. The pointer is
// never transferred to the caller and is valid only for the lifetime documented
// by the public World bulk-result API contract.
template<class T>
bool initialize_world_results(const T** output, int32_t* count) {
    if (output) *output = nullptr;
    if (count) *count = 0;
    if (!output || !count) {
        arc::set_error("Result data and count outputs are required.");
        return false;
    }
    return true;
}

template<class T>
arc_status publish_world_results(
    const std::vector<T>& values, const T** output, int32_t* count) {
    if (values.size() > static_cast<size_t>(std::numeric_limits<int32_t>::max())) {
        arc::set_error("Result count exceeds the C ABI limit.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
    *output = values.empty() ? nullptr : values.data();
    *count = static_cast<int32_t>(values.size());
    return ARC_STATUS_OK;
}

} // namespace

extern "C" {

arc_world* ARC_CALL arc_world_create(const arc_world_options* supplied) {
    const arc_world_options options = supplied
        ? *supplied : arc_world_options{16.0f, 16, 16};
    if (!arc::valid_scalar(options.fat_margin) || options.fat_margin < 0
        || options.initial_collider_capacity < 0
        || options.initial_collider_capacity > ARC_MAX_COLLIDER_COUNT
        || options.initial_pair_capacity < 0) {
        arc::set_error("Invalid world options.");
        return nullptr;
    }
    uint32_t id = 0;
    {
        std::lock_guard<std::mutex> lock(world_id_mutex);
        for (uint32_t candidate = 1; candidate <= ARC_MAX_WORLD_COUNT; ++candidate) {
            if (!world_ids[candidate]) {
                world_ids[candidate] = true;
                id = candidate;
                break;
            }
        }
    }
    if (id == 0) {
        arc::set_error("At most 15 ArcWorld instances may be alive at once.");
        return nullptr;
    }
    try {
        return new arc_world(id, options);
    } catch (const std::exception& exception) {
        std::lock_guard<std::mutex> lock(world_id_mutex);
        world_ids[id] = false;
        arc::set_error(exception.what());
        return nullptr;
    } catch (...) {
        std::lock_guard<std::mutex> lock(world_id_mutex);
        world_ids[id] = false;
        arc::set_error("World allocation failed.");
        return nullptr;
    }
}

void ARC_CALL arc_world_destroy(arc_world* world) {
    if (!world) return;
    const uint32_t id = world->id;
    delete world;
    std::lock_guard<std::mutex> lock(world_id_mutex);
    world_ids[id] = false;
}

arc_status ARC_CALL arc_world_clear(arc_world* world) {
    if (!world) return ensure_world(world);
    world->broadphase.clear();
    world->candidates.clear();
    world->pairs.clear();
    world->pair_values.clear();
    world->query_values.clear();
    world->cast_values.clear();
    world->active_count = world->enabled_count = world->dynamic_count = 0;
    world->free_list = world->slots.empty() ? -1 : 0;
    for (size_t i = 0; i < world->slots.size(); ++i) {
        Slot& slot = world->slots[i];
        slot.shape = StoredShape{};
        slot.local = LocalShape{};
        slot.transform = {};
        slot.bounds = {};
        slot.filter = {};
        slot.entity_id = 0;
        slot.tree_proxy = -1;
        slot.active = slot.enabled = slot.is_static = false;
        slot.generation = 0;
        slot.next_free = i + 1 < world->slots.size()
            ? static_cast<int>(i + 1) : -1;
    }
    return ARC_STATUS_OK;
}

arc_status ARC_CALL arc_world_ensure_capacity(
    arc_world* world, int32_t collider_capacity, int32_t pair_capacity) {
    if (!world) return ensure_world(world);
    if (collider_capacity < 0 || collider_capacity > ARC_MAX_COLLIDER_COUNT
        || pair_capacity < 0) {
        arc::set_error("Capacity is outside the supported range.");
        return ARC_STATUS_OUT_OF_RANGE;
    }
    try {
        world->slots.reserve(static_cast<size_t>(collider_capacity));
        world->candidates.reserve(static_cast<size_t>(collider_capacity));
        world->pairs.reserve(static_cast<size_t>(pair_capacity));
        world->pair_values.reserve(static_cast<size_t>(pair_capacity));
        world->query_values.reserve(static_cast<size_t>(collider_capacity));
        world->cast_values.reserve(static_cast<size_t>(collider_capacity));
        world->broadphase.ensure_capacity(collider_capacity);
        return ARC_STATUS_OK;
    } catch (...) {
        arc::set_error("Capacity allocation failed.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
}

arc_status ARC_CALL arc_world_build_static(arc_world* world) {
    if (!world) return ensure_world(world);
    try {
        world->broadphase.build_static();
        return ARC_STATUS_OK;
    } catch (...) {
        arc::set_error("Static BVH build failed.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
}

int32_t ARC_CALL arc_world_get_count(const arc_world* world) {
    return world ? world->active_count : 0;
}
int32_t ARC_CALL arc_world_get_enabled_count(const arc_world* world) {
    return world ? world->enabled_count : 0;
}
int32_t ARC_CALL arc_world_get_dynamic_count(const arc_world* world) {
    return world ? world->dynamic_count : 0;
}
int32_t ARC_CALL arc_world_get_static_count(const arc_world* world) {
    return world ? world->active_count - world->dynamic_count : 0;
}
float ARC_CALL arc_world_get_fat_margin(const arc_world* world) {
    return world ? world->broadphase.fat_margin() : 0;
}

arc_status ARC_CALL arc_world_add(
    arc_world* world, int32_t entity_id, const arc_shape* shape,
    arc_collision_filter filter, arc_bool is_static, arc_bool enabled,
    arc_handle* out_handle) {
    if (!world || !shape || !out_handle) {
        arc::set_error("World, shape, and output handle are required.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    if (!arc::validate_shape(*shape)) {
        arc::set_error("Invalid shape.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    if (entity_id < 0 || entity_id > ARC_MAX_ENTITY_ID) {
        arc::set_error("Entity id is outside the supported range.");
        return ARC_STATUS_OUT_OF_RANGE;
    }
    try {
        StoredShape stored(*shape);
        const arc::Bounds bounds = arc::shape_bounds(*shape);
        size_t index;
        if (world->free_list != -1) {
            index = static_cast<size_t>(world->free_list);
            world->free_list = world->slots[index].next_free;
        } else {
            if (world->slots.size() >= ARC_MAX_COLLIDER_COUNT)
                return ARC_STATUS_OUT_OF_RANGE;
            index = world->slots.size();
            world->slots.emplace_back();
        }
        auto& table = generation_tables[world->id];
        if (table.size() <= index) table.resize(index + 1);
        uint16_t generation = static_cast<uint16_t>(table[index] + 1);
        if (generation == 0 || generation > GenerationMax) generation = 1;
        table[index] = generation;

        Slot& slot = world->slots[index];
        FixedTransform initial;
        slot.local = to_local(*shape, initial);
        slot.transform = initial;
        slot.shape = std::move(stored);
        slot.bounds = bounds;
        slot.filter = filter;
        slot.entity_id = entity_id;
        slot.tree_proxy = -1;
        slot.next_free = -1;
        slot.generation = generation;
        slot.active = true;
        slot.enabled = enabled != 0;
        slot.is_static = is_static != 0;
        // A static collider joins the BVH as soon as it is active, even when it
        // starts disabled: enable/disable must not touch the one-shot tree, so
        // membership tracks `active` and `enabled` is applied at query time.
        if (slot.is_static)
            world->broadphase.add_or_update_static(static_cast<int>(index), bounds);
        else if (slot.enabled)
            slot.tree_proxy =
                world->broadphase.add_dynamic(static_cast<int>(index), bounds);
        ++world->active_count;
        if (slot.enabled) ++world->enabled_count;
        if (!slot.is_static) ++world->dynamic_count;
        *out_handle = make_handle(world, index);
        return ARC_STATUS_OK;
    } catch (const std::out_of_range& exception) {
        arc::set_error(exception.what());
        return ARC_STATUS_OUT_OF_RANGE;
    } catch (const std::exception& exception) {
        arc::set_error(exception.what());
        return ARC_STATUS_INTERNAL_ERROR;
    } catch (...) {
        arc::set_error("Add failed.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
}

// Re-place a collider's immutable base shape at `transform`: materialize the world
// shape from the local form, recompute bounds, and move its broadphase proxy.
arc_status apply_transform(arc_world* world, arc_handle handle,
    Slot* slot, const FixedTransform& transform) {
    try {
        MaterializedShape world_shape = materialize(slot->local, transform);
        if (world_shape.shape.kind == ARC_SHAPE_POLYGON
            && !world_shape.shape.polygon) {
            arc::set_error("Transform produces an invalid polygon.");
            return ARC_STATUS_INVALID_ARGUMENT;
        }
        StoredShape stored(world_shape.shape);
        // StoredShape retained the generated geometry; drop materialize's
        // creation reference before any later operation can throw.
        if (world_shape.owned_polygon)
            arc_polygon_release(world_shape.owned_polygon);
        slot->shape = std::move(stored);
        slot->transform = transform;
        slot->bounds = world_shape.bounds;
        // A disabled static is still in the BVH, so its leaf bounds must follow
        // the move; a disabled dynamic owns no proxy and is re-added on enable.
        if (slot->is_static)
            world->broadphase.add_or_update_static(
                static_cast<int>(handle_index(handle)), world_shape.bounds);
        else if (slot->enabled)
            world->broadphase.update_dynamic(slot->tree_proxy, world_shape.bounds);
        return ARC_STATUS_OK;
    } catch (const std::out_of_range& exception) {
        arc::set_error(exception.what());
        return ARC_STATUS_OUT_OF_RANGE;
    } catch (...) {
        arc::set_error("Update failed.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
}

arc_status ARC_CALL arc_world_update_transform(
    arc_world* world, arc_handle handle, const arc_transform* transform) {
    Slot* slot = get_slot(world, handle);
    if (!slot) {
        arc::set_error("Handle is stale or belongs to another world.");
        return ARC_STATUS_INVALID_HANDLE;
    }
    if (!transform || !arc::valid_vec(transform->position)
        || !(transform->scale >= 0.0f)) {
        arc::set_error("Invalid transform.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    try {
        return apply_transform(
            world, handle, slot, fixed_transform_from_public(*transform));
    } catch (const std::out_of_range& exception) {
        arc::set_error(exception.what());
        return ARC_STATUS_OUT_OF_RANGE;
    }
}

arc_status ARC_CALL arc_world_update_transform_delta(
    arc_world* world, arc_handle handle, const arc_transform* delta) {
    Slot* slot = get_slot(world, handle);
    if (!slot) {
        arc::set_error("Handle is stale or belongs to another world.");
        return ARC_STATUS_INVALID_HANDLE;
    }
    if (!delta || !arc::valid_vec(delta->position)
        || !(delta->scale >= 0.0f)) {
        arc::set_error("Invalid transform delta.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    try {
        const FixedTransform fixed_delta = fixed_transform_from_public(*delta);
        return apply_transform(world, handle, slot,
            compose_transform(slot->transform, fixed_delta));
    } catch (const std::out_of_range& exception) {
        arc::set_error(exception.what());
        return ARC_STATUS_OUT_OF_RANGE;
    }
}

arc_status ARC_CALL arc_world_remove(arc_world* world, arc_handle handle) {
    Slot* slot = get_slot(world, handle);
    if (!slot) {
        arc::set_error("Handle is stale or belongs to another world.");
        return ARC_STATUS_INVALID_HANDLE;
    }
    const int index = static_cast<int>(handle_index(handle));
    // Statics are in the BVH regardless of `enabled`, so removal is
    // unconditional: leaving a leaf behind would alias the recycled slot index.
    if (slot->is_static)
        world->broadphase.remove_static(index);
    else if (slot->enabled)
        world->broadphase.remove_dynamic(slot->tree_proxy);
    if (slot->enabled) --world->enabled_count;
    if (!slot->is_static) --world->dynamic_count;
    slot->shape = StoredShape{};
    slot->local = LocalShape{};
    slot->transform = {};
    slot->bounds = {};
    slot->filter = {};
    slot->entity_id = 0;
    slot->tree_proxy = -1;
    slot->active = slot->enabled = slot->is_static = false;
    slot->generation = 0;
    slot->next_free = world->free_list;
    world->free_list = index;
    --world->active_count;
    return ARC_STATUS_OK;
}

arc_bool ARC_CALL arc_world_is_valid(
    const arc_world* world, arc_handle handle) {
    return valid(world, handle);
}

arc_status ARC_CALL arc_world_get_shape(
    const arc_world* world, arc_handle handle, arc_shape* output) {
    const Slot* slot = get_slot(world, handle);
    if (!slot) return ARC_STATUS_INVALID_HANDLE;
    if (!output) return ARC_STATUS_INVALID_ARGUMENT;
    *output = slot->shape.value;
    arc::retain_shape(*output);
    return ARC_STATUS_OK;
}

arc_status ARC_CALL arc_world_get_filter(
    const arc_world* world, arc_handle handle, arc_collision_filter* output) {
    const Slot* slot = get_slot(world, handle);
    if (!slot) return ARC_STATUS_INVALID_HANDLE;
    if (!output) return ARC_STATUS_INVALID_ARGUMENT;
    *output = slot->filter;
    return ARC_STATUS_OK;
}

arc_status ARC_CALL arc_world_set_filter(
    arc_world* world, arc_handle handle, arc_collision_filter filter) {
    Slot* slot = get_slot(world, handle);
    if (!slot) return ARC_STATUS_INVALID_HANDLE;
    slot->filter = filter;
    return ARC_STATUS_OK;
}

arc_status ARC_CALL arc_world_get_enabled(
    const arc_world* world, arc_handle handle, arc_bool* output) {
    const Slot* slot = get_slot(world, handle);
    if (!slot) return ARC_STATUS_INVALID_HANDLE;
    if (!output) return ARC_STATUS_INVALID_ARGUMENT;
    *output = slot->enabled;
    return ARC_STATUS_OK;
}

arc_status ARC_CALL arc_world_set_enabled(
    arc_world* world, arc_handle handle, arc_bool enabled) {
    Slot* slot = get_slot(world, handle);
    if (!slot) return ARC_STATUS_INVALID_HANDLE;
    const bool value = enabled != 0;
    if (slot->enabled == value) return ARC_STATUS_OK;
    const int index = static_cast<int>(handle_index(handle));
    // Toggling a static deliberately leaves its leaf in the BVH. Mutating the
    // static tree marks it dirty and the next query rebuilds every leaf (SAH is
    // one-shot), so a per-frame toggle would cost a full rebuild per frame.
    // Disabled statics are instead filtered after the broadphase, exactly like
    // the `active` check: query_slot and compute_pairs both re-test slot.enabled,
    // so a disabled leaf can never reach a result. The cost is a little dead
    // weight in the tree until the collider is actually removed.
    // Dynamic colliders keep the tighter invariant (tree == enabled dynamics):
    // their tree is incremental, so add/remove is O(log n) rather than a rebuild.
    if (!slot->is_static) {
        if (value)
            slot->tree_proxy = world->broadphase.add_dynamic(index, slot->bounds);
        else {
            world->broadphase.remove_dynamic(slot->tree_proxy);
            slot->tree_proxy = -1;
        }
    }
    slot->enabled = value;
    world->enabled_count += value ? 1 : -1;
    return ARC_STATUS_OK;
}

arc_status ARC_CALL arc_world_shift_origin(
    arc_world* world, arc_vec2 origin_delta) {
    if (!world || !arc::valid_vec(origin_delta)) {
        arc::set_error("Invalid world or origin delta.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    try {
        (void)arc::Vec::from(origin_delta);
        const arc::Vec fixed_delta{
            -float_bits_to_fixed(origin_delta.x, arc::FxShift,
                                 std::numeric_limits<int64_t>::max()),
            -float_bits_to_fixed(origin_delta.y, arc::FxShift,
                                 std::numeric_limits<int64_t>::max())};
        struct ShiftedSlot {
            StoredShape shape;
            arc::Bounds bounds;
            FixedTransform transform;
        };
        std::vector<ShiftedSlot> moved;
        moved.reserve(static_cast<size_t>(world->active_count));
        for (Slot& slot : world->slots) {
            if (!slot.active) continue;
            FixedTransform shifted = slot.transform;
            shifted.position += fixed_delta;
            MaterializedShape materialized = materialize(slot.local, shifted);
            if (materialized.shape.kind == ARC_SHAPE_POLYGON
                && !materialized.shape.polygon) {
                arc::set_error("Origin shift produces an invalid polygon.");
                return ARC_STATUS_INVALID_ARGUMENT;
            }
            StoredShape stored(materialized.shape);
            if (materialized.owned_polygon)
                arc_polygon_release(materialized.owned_polygon);
            moved.push_back({std::move(stored), materialized.bounds, shifted});
        }
        world->broadphase.clear();
        size_t moved_index = 0;
        for (size_t index = 0; index < world->slots.size(); ++index) {
            Slot& slot = world->slots[index];
            if (!slot.active) continue;
            slot.shape = std::move(moved[moved_index].shape);
            slot.transform = moved[moved_index].transform;
            slot.bounds = moved[moved_index].bounds;
            slot.tree_proxy = -1;
            ++moved_index;
            // Disabled statics are BVH members too, so they must be re-seeded
            // here; disabled dynamics own no proxy and keep tree_proxy == -1.
            if (slot.is_static)
                world->broadphase.add_or_update_static(
                    static_cast<int>(index), slot.bounds);
            else if (slot.enabled)
                slot.tree_proxy = world->broadphase.add_dynamic(
                    static_cast<int>(index), slot.bounds);
        }
        world->broadphase.build_static();
        return ARC_STATUS_OK;
    } catch (const std::out_of_range& exception) {
        arc::set_error(exception.what());
        return ARC_STATUS_OUT_OF_RANGE;
    } catch (...) {
        arc::set_error("Origin shift failed.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
}

arc_status ARC_CALL arc_world_compute_pairs(
    arc_world* world, const arc_candidate_pair** output, int32_t* count) {
    if (!initialize_world_results(output, count))
        return ARC_STATUS_INVALID_ARGUMENT;
    if (!world) return ensure_world(world);
    try {
        world->broadphase.compute_pairs(world->pairs);
        world->pair_values.clear();
        for (const auto& pair : world->pairs) {
            const Slot& a = world->slots[static_cast<size_t>(pair.first)];
            const Slot& b = world->slots[static_cast<size_t>(pair.second)];
            if (a.active && b.active && a.enabled && b.enabled
                && arc::filter_allows(a.filter, b.filter)
                && a.bounds.overlaps(b.bounds))
                world->pair_values.push_back(
                    make_pair(world, pair.first, pair.second));
        }
        std::sort(world->pair_values.begin(), world->pair_values.end(), pair_less);
        return publish_world_results(world->pair_values, output, count);
    } catch (...) {
        arc::set_error("Pair computation failed.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
}

arc_status ARC_CALL arc_world_query(
    arc_world* world, const arc_shape* query,
    const arc_collision_filter* filter,
    const arc_handle** output, int32_t* count) {
    if (!initialize_world_results(output, count))
        return ARC_STATUS_INVALID_ARGUMENT;
    if (!world || !query || !arc::validate_shape(*query)) {
        arc::set_error("Invalid world or query shape.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    try {
        const arc::Bounds bounds = arc::shape_bounds(*query);
        world->query_values.clear();
        world->candidates.clear();
        world->broadphase.query_dynamic(bounds, world->candidates);
        append_query(world, bounds, filter);
        world->candidates.clear();
        world->broadphase.query_static(bounds, world->candidates);
        append_query(world, bounds, filter);
        std::sort(world->query_values.begin(), world->query_values.end(), handle_less);
        return publish_world_results(world->query_values, output, count);
    } catch (...) {
        arc::set_error("Query failed.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
}

// Adaptive batch query. Small and scattered-light batches use a scalar native
// loop; bundled-light batches use input-order packets without sorting/scatter.
// Both avoid Morton setup while crossing the C ABI only once. Dense queries use
// Morton-coherent SIMD.
// Both paths filter and sort each query exactly like arc_world_query. Results are
// concatenated into one world-owned handle buffer; out_counts[k] is the number of
// handles belonging to query k (its slice starts after the previous queries'
// counts). Both views follow the borrowed-result contract of arc_world_query.
arc_status ARC_CALL arc_world_query_batch(
    arc_world* world, const arc_shape* queries, int32_t query_count,
    const arc_collision_filter* filter,
    const arc_handle** out_handles, const int32_t** out_counts,
    int32_t* out_total) {
    if (out_handles) *out_handles = nullptr;
    if (out_counts) *out_counts = nullptr;
    if (out_total) *out_total = 0;
    if (!world || !out_handles || !out_counts || !out_total
        || query_count < 0 || (query_count > 0 && !queries)) {
        arc::set_error("Invalid world or batch query arguments.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    try {
        world->query_values.clear();
        world->query_batch_counts.clear();
        world->morton_handles.clear();
        if (query_count == 0) return ARC_STATUS_OK;

        const size_t count = static_cast<size_t>(query_count);
        world->query_batch_bounds.clear();
        world->query_batch_bounds.reserve(count);
        for (int32_t i = 0; i < query_count; ++i) {
            if (!arc::validate_shape(queries[i])) {
                arc::set_error("Invalid batch query shape.");
                return ARC_STATUS_INVALID_ARGUMENT;
            }
            const arc::Bounds bounds = arc::shape_bounds(queries[i]);
            world->query_batch_bounds.push_back(bounds);
        }
        world->query_batch_counts.assign(count, 0);

        const QueryBatchPath path = select_query_batch_path(
            world, world->query_batch_bounds, filter);
        if (path == QueryBatchPath::Scalar) {
            for (size_t i = 0; i < count; ++i) {
                world->query_batch_counts[i] = append_scalar_batch_query(
                    world, world->query_batch_bounds[i], filter);
            }
            if (world->query_values.size()
                    > static_cast<size_t>(std::numeric_limits<int32_t>::max())) {
                arc::set_error("Batch result count exceeds the C ABI limit.");
                return ARC_STATUS_INTERNAL_ERROR;
            }
            *out_handles = world->query_values.empty()
                ? nullptr : world->query_values.data();
            *out_counts = world->query_batch_counts.data();
            *out_total = static_cast<int32_t>(world->query_values.size());
            return ARC_STATUS_OK;
        }

        world->query_order.resize(count);
        for (int32_t i = 0; i < query_count; ++i)
            world->query_order[static_cast<size_t>(i)] = i;
        if (path == QueryBatchPath::MortonPacket) {
            world->query_morton.resize(count);
            for (size_t i = 0; i < count; ++i)
                world->query_morton[i] = morton_code(world->query_batch_bounds[i]);
            // Dense queries are sorted so each four-lane packet is spatially
            // coherent. Sparse packets retain input order and avoid this O(N log N)
            // setup entirely.
            const std::vector<uint64_t>& morton = world->query_morton;
            std::sort(world->query_order.begin(), world->query_order.end(),
                [&morton](int a, int b) {
                    return morton[static_cast<size_t>(a)]
                        < morton[static_cast<size_t>(b)];
                });
        }
        world->query_slice_start.assign(count, 0);

        // Padding for the final partial group of four: a box that overlaps nothing
        // (colliders live within +/-ARC_MAX_COORDINATE, far from the int32 rails).
        const arc::Bounds empty_box{
            std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::max(),
            std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min()};
        std::vector<int>* lane_results[4] = {
            &world->packet_candidates[0], &world->packet_candidates[1],
            &world->packet_candidates[2], &world->packet_candidates[3]};
        for (int32_t base = 0; base < query_count; base += 4) {
            const int lanes =
                static_cast<int>(std::min<int32_t>(4, query_count - base));
            arc::Bounds group[4];
            for (int lane = 0; lane < 4; ++lane) {
                group[lane] = lane < lanes
                    ? world->query_batch_bounds[
                        static_cast<size_t>(world->query_order[static_cast<size_t>(base + lane)])]
                    : empty_box;
                world->packet_candidates[lane].clear();
            }
            // Dynamic then static into the same per-lane lists (a collider lives in
            // exactly one tree, so no lane can see a duplicate id).
            world->broadphase.query_dynamic_packet(group, lane_results, world->packet_stack);
            world->broadphase.query_static_packet(group, lane_results, world->packet_stack);
            for (int lane = 0; lane < lanes; ++lane) {
                const int query_index =
                    world->query_order[static_cast<size_t>(base + lane)];
                const arc::Bounds& bounds = group[lane];
                const size_t start = world->morton_handles.size();
                for (int index : world->packet_candidates[lane]) {
                    const Slot& slot = world->slots[static_cast<size_t>(index)];
                    if (query_slot(slot, filter) && slot.bounds.overlaps(bounds))
                        world->morton_handles.push_back(
                            make_handle(world, static_cast<size_t>(index)));
                }
                std::sort(
                    world->morton_handles.begin() + static_cast<ptrdiff_t>(start),
                    world->morton_handles.end(), handle_less);
                world->query_slice_start[static_cast<size_t>(query_index)] =
                    static_cast<int32_t>(start);
                world->query_batch_counts[static_cast<size_t>(query_index)] =
                    static_cast<int32_t>(world->morton_handles.size() - start);
            }
        }
        if (world->morton_handles.size()
                > static_cast<size_t>(std::numeric_limits<int32_t>::max())) {
            arc::set_error("Batch result count exceeds the C ABI limit.");
            return ARC_STATUS_INTERNAL_ERROR;
        }
        if (path == QueryBatchPath::DirectPacket) {
            // Identity query_order means packet results are already concatenated
            // in public input order; swap the world-owned buffers instead of copying.
            world->query_values.swap(world->morton_handles);
        } else {
            // Scatter Morton-order results back into input order: query k's handles
            // are contiguous starting at its running input-order offset.
            world->query_values.resize(world->morton_handles.size());
            size_t offset = 0;
            for (size_t q = 0; q < count; ++q) {
                const size_t start =
                    static_cast<size_t>(world->query_slice_start[q]);
                const size_t length =
                    static_cast<size_t>(world->query_batch_counts[q]);
                std::copy(world->morton_handles.begin() + static_cast<ptrdiff_t>(start),
                    world->morton_handles.begin() + static_cast<ptrdiff_t>(start + length),
                    world->query_values.begin() + static_cast<ptrdiff_t>(offset));
                offset += length;
            }
        }
        *out_handles =
            world->query_values.empty() ? nullptr : world->query_values.data();
        *out_counts = world->query_batch_counts.data();
        *out_total = static_cast<int32_t>(world->query_values.size());
        return ARC_STATUS_OK;
    } catch (...) {
        arc::set_error("Batch query failed.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
}

arc_status ARC_CALL arc_world_try_contact_pair(
    arc_world* world, arc_candidate_pair pair, arc_manifold_fields fields,
    arc_contact_pair* output, arc_bool* colliding) {
    if (!world || !output || !colliding) {
        arc::set_error("World and contact outputs are required.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    if (fields > ARC_MANIFOLD_ALL) {
        arc::set_error("Manifold fields are outside the supported range.");
        return ARC_STATUS_OUT_OF_RANGE;
    }
    *output = {};
    *colliding = 0;
    Slot* a = get_slot(world, pair.a);
    Slot* b = get_slot(world, pair.b);
    if (!a || !b) return ARC_STATUS_INVALID_HANDLE;
    if (!a->enabled || !b->enabled
        || !arc::filter_allows(a->filter, b->filter))
        return ARC_STATUS_OK;
    arc_manifold manifold{};
    if (fields == ARC_MANIFOLD_NONE) {
        manifold.colliding = arc::overlap_shapes(
            a->shape.value, b->shape.value) ? 1 : 0;
    } else {
        manifold = arc::collide_shapes(
            a->shape.value, b->shape.value,
            fields == ARC_MANIFOLD_ALL).to_public();
    }
    if (manifold.colliding) {
        *colliding = 1;
        *output = {pair.a, pair.b, manifold};
    }
    return ARC_STATUS_OK;
}

arc_status ARC_CALL arc_world_try_contact_shape(
    arc_world* world, const arc_shape* query,
    const arc_collision_filter* filter, arc_handle target,
    arc_manifold_fields fields, arc_manifold* output, arc_bool* colliding) {
    if (!world || !query || !output || !colliding
        || !arc::validate_shape(*query)) {
        arc::set_error("World, a valid query shape, and contact outputs are required.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    if (fields > ARC_MANIFOLD_ALL) {
        arc::set_error("Manifold fields are outside the supported range.");
        return ARC_STATUS_OUT_OF_RANGE;
    }
    *output = {};
    *colliding = 0;
    Slot* slot = get_slot(world, target);
    if (!slot) return ARC_STATUS_INVALID_HANDLE;
    if (!slot->enabled
        || (filter && !arc::filter_allows(*filter, slot->filter)))
        return ARC_STATUS_OK;
    if (fields == ARC_MANIFOLD_NONE) {
        output->colliding = arc::overlap_shapes(
            *query, slot->shape.value) ? 1 : 0;
    } else {
        *output = arc::collide_shapes(
            *query, slot->shape.value,
            fields == ARC_MANIFOLD_ALL).to_public();
    }
    *colliding = output->colliding;
    return ARC_STATUS_OK;
}

arc_status ARC_CALL arc_world_shape_cast_all(
    arc_world* world, const arc_shape* mover, arc_vec2 motion,
    const arc_collision_filter* filter,
    const arc_world_cast_hit** output, int32_t* count) {
    if (!initialize_world_results(output, count))
        return ARC_STATUS_INVALID_ARGUMENT;
    if (!world || !mover || !arc::validate_shape(*mover)
        || !arc::valid_vec(motion)) {
        arc::set_error("Invalid world, mover, or motion.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    try {
        const arc::Bounds bounds = swept_bounds(*mover, motion);
        world->cast_values.clear();
        world->candidates.clear();
        world->broadphase.query_dynamic(bounds, world->candidates);
        append_casts(world, *mover, motion, filter);
        world->candidates.clear();
        world->broadphase.query_static(bounds, world->candidates);
        append_casts(world, *mover, motion, filter);
        std::sort(world->cast_values.begin(), world->cast_values.end(), cast_less);
        return publish_world_results(world->cast_values, output, count);
    } catch (...) {
        arc::set_error("Shape cast failed.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
}

arc_status ARC_CALL arc_world_shape_cast(
    arc_world* world, const arc_shape* mover, arc_vec2 motion,
    const arc_collision_filter* filter,
    arc_world_cast_hit* output, arc_bool* found) {
    if (!world || !mover || !output || !found
        || !arc::validate_shape(*mover) || !arc::valid_vec(motion)) {
        arc::set_error("Invalid world, mover, motion, or output.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    try {
        const arc::Bounds bounds = swept_bounds(*mover, motion);
        bool has_hit = false;
        arc_world_cast_hit closest{};

        world->candidates.clear();
        world->broadphase.query_dynamic(bounds, world->candidates);
        find_closest_cast(
            world, *mover, motion, filter, has_hit, closest);
        world->candidates.clear();
        world->broadphase.query_static(bounds, world->candidates);
        find_closest_cast(
            world, *mover, motion, filter, has_hit, closest);

        *found = has_hit ? 1 : 0;
        if (has_hit) *output = closest;
        return ARC_STATUS_OK;
    } catch (...) {
        arc::set_error("Shape cast failed.");
        *found = 0;
        return ARC_STATUS_INTERNAL_ERROR;
    }
}

arc_status ARC_CALL arc_world_ray_cast_all(
    arc_world* world, arc_vec2 origin, arc_vec2 motion,
    const arc_collision_filter* filter,
    const arc_world_cast_hit** output, int32_t* count) {
    arc_shape point{};
    point.kind = ARC_SHAPE_CIRCLE;
    point.circle = {origin, 0};
    return arc_world_shape_cast_all(
        world, &point, motion, filter, output, count);
}

arc_status ARC_CALL arc_world_ray_cast(
    arc_world* world, arc_vec2 origin, arc_vec2 motion,
    const arc_collision_filter* filter,
    arc_world_cast_hit* output, arc_bool* found) {
    arc_shape point{};
    point.kind = ARC_SHAPE_CIRCLE;
    point.circle = {origin, 0};
    return arc_world_shape_cast(
        world, &point, motion, filter, output, found);
}

/* ---------------------------------------------------------------------------
   Standalone broadphase structures (dynamic tree + static BVH). Thin C wrappers
   over arc::DynamicAabbTree / arc::StaticBvh, mirroring the managed public API.
   Each wrapper carries a scratch vector so query has no per-call allocation. */

struct arc_dynamic_tree {
    arc::DynamicAabbTree impl;
    std::vector<int> scratch;
    std::vector<std::pair<int, int>> pair_scratch;
};
struct arc_static_bvh { arc::StaticBvh impl; std::vector<int> scratch; };

static arc::Bounds arc_to_bounds(arc_bp_bounds b) {
    return arc::Bounds{b.min_x, b.min_y, b.max_x, b.max_y};
}

arc_bp_bounds ARC_CALL arc_bp_bounds_from_shape(const arc_shape* shape) {
    if (!shape || !arc::validate_shape(*shape)) {
        arc::set_error("Invalid shape.");
        return arc_bp_bounds{0, 0, 0, 0};
    }
    const arc::Bounds b = arc::shape_bounds(*shape);
    return arc_bp_bounds{b.min_x, b.min_y, b.max_x, b.max_y};
}

arc_dynamic_tree* ARC_CALL arc_dynamic_tree_create(void) {
    try { return new arc_dynamic_tree(); }
    catch (...) { arc::set_error("Dynamic tree allocation failed."); return nullptr; }
}

void ARC_CALL arc_dynamic_tree_destroy(arc_dynamic_tree* tree) { delete tree; }

arc_status ARC_CALL arc_dynamic_tree_clear(arc_dynamic_tree* tree) {
    if (!tree) { arc::set_error("Tree is null."); return ARC_STATUS_INVALID_ARGUMENT; }
    tree->impl.clear();
    return ARC_STATUS_OK;
}

arc_status ARC_CALL arc_dynamic_tree_ensure_capacity(arc_dynamic_tree* tree, int32_t proxy_capacity) {
    if (!tree || proxy_capacity < 0) {
        arc::set_error("Tree is null or capacity is negative.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    try { tree->impl.ensure_capacity(proxy_capacity); return ARC_STATUS_OK; }
    catch (...) { arc::set_error("Capacity allocation failed."); return ARC_STATUS_INTERNAL_ERROR; }
}

int32_t ARC_CALL arc_dynamic_tree_get_count(const arc_dynamic_tree* tree) {
    return tree ? tree->impl.count() : 0;
}

arc_status ARC_CALL arc_dynamic_tree_create_proxy(
    arc_dynamic_tree* tree, int32_t id, arc_bp_bounds fat_bounds, int32_t* out_proxy) {
    if (!tree || !out_proxy) {
        arc::set_error("Tree and output proxy are required.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    try { *out_proxy = tree->impl.create_proxy(id, arc_to_bounds(fat_bounds)); return ARC_STATUS_OK; }
    catch (...) { arc::set_error("Create proxy failed."); return ARC_STATUS_INTERNAL_ERROR; }
}

arc_status ARC_CALL arc_dynamic_tree_move_proxy(
    arc_dynamic_tree* tree, int32_t proxy, arc_bp_bounds bounds,
    arc_bp_bounds fat_bounds, arc_bool* out_moved) {
    if (!tree) { arc::set_error("Tree is null."); return ARC_STATUS_INVALID_ARGUMENT; }
    try {
        bool moved = tree->impl.move_proxy(
            proxy, arc_to_bounds(bounds), arc_to_bounds(fat_bounds));
        if (out_moved) *out_moved = moved ? 1 : 0;
        return ARC_STATUS_OK;
    } catch (...) { arc::set_error("Move proxy failed."); return ARC_STATUS_INTERNAL_ERROR; }
}

arc_status ARC_CALL arc_dynamic_tree_destroy_proxy(arc_dynamic_tree* tree, int32_t proxy) {
    if (!tree) { arc::set_error("Tree is null."); return ARC_STATUS_INVALID_ARGUMENT; }
    try { tree->impl.destroy_proxy(proxy); return ARC_STATUS_OK; }
    catch (...) { arc::set_error("Destroy proxy failed."); return ARC_STATUS_INTERNAL_ERROR; }
}

arc_status ARC_CALL arc_dynamic_tree_query(
    const arc_dynamic_tree* tree, arc_bp_bounds bounds,
    int32_t* output, int32_t capacity, int32_t* required) {
    if (!tree) { arc::set_error("Tree is null."); return ARC_STATUS_INVALID_ARGUMENT; }
    try {
        arc_dynamic_tree* mutable_tree = const_cast<arc_dynamic_tree*>(tree);
        mutable_tree->scratch.clear();
        tree->impl.query(arc_to_bounds(bounds), mutable_tree->scratch);
        return arc::copy_results(mutable_tree->scratch, output, capacity, required);
    } catch (...) { arc::set_error("Query failed."); return ARC_STATUS_INTERNAL_ERROR; }
}

arc_status ARC_CALL arc_dynamic_tree_compute_self_pairs(
    const arc_dynamic_tree* tree, arc_int_pair* output, int32_t capacity, int32_t* required) {
    if (!tree) { arc::set_error("Tree is null."); return ARC_STATUS_INVALID_ARGUMENT; }
    if (!required || capacity < 0) {
        arc::set_error("Invalid output buffer.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    try {
        arc_dynamic_tree* mutable_tree = const_cast<arc_dynamic_tree*>(tree);
        mutable_tree->pair_scratch.clear();
        tree->impl.compute_self_pairs(mutable_tree->pair_scratch);
        if (mutable_tree->pair_scratch.size()
            > static_cast<size_t>(std::numeric_limits<int32_t>::max())) {
            arc::set_error("Result count exceeds the C ABI limit.");
            return ARC_STATUS_INTERNAL_ERROR;
        }
        *required = static_cast<int32_t>(mutable_tree->pair_scratch.size());
        if (capacity < *required || (!output && *required != 0))
            return ARC_STATUS_BUFFER_TOO_SMALL;
        for (int32_t i = 0; i < *required; ++i) {
            output[i].a = mutable_tree->pair_scratch[static_cast<size_t>(i)].first;
            output[i].b = mutable_tree->pair_scratch[static_cast<size_t>(i)].second;
        }
        return ARC_STATUS_OK;
    } catch (...) { arc::set_error("Compute self pairs failed."); return ARC_STATUS_INTERNAL_ERROR; }
}

arc_static_bvh* ARC_CALL arc_static_bvh_create(void) {
    try { return new arc_static_bvh(); }
    catch (...) { arc::set_error("Static BVH allocation failed."); return nullptr; }
}

void ARC_CALL arc_static_bvh_destroy(arc_static_bvh* bvh) { delete bvh; }

arc_status ARC_CALL arc_static_bvh_clear(arc_static_bvh* bvh) {
    if (!bvh) { arc::set_error("BVH is null."); return ARC_STATUS_INVALID_ARGUMENT; }
    bvh->impl.clear();
    return ARC_STATUS_OK;
}

arc_status ARC_CALL arc_static_bvh_ensure_capacity(arc_static_bvh* bvh, int32_t leaf_capacity) {
    if (!bvh || leaf_capacity < 0) {
        arc::set_error("BVH is null or capacity is negative.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    try { bvh->impl.ensure_capacity(leaf_capacity); return ARC_STATUS_OK; }
    catch (...) { arc::set_error("Capacity allocation failed."); return ARC_STATUS_INTERNAL_ERROR; }
}

arc_status ARC_CALL arc_static_bvh_build(
    arc_static_bvh* bvh, const int32_t* ids, const arc_bp_bounds* bounds, int32_t count) {
    if (!bvh || count < 0 || (count > 0 && (!ids || !bounds))) {
        arc::set_error("BVH is null or build arrays are invalid.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    try {
        // Full replace to mirror the managed Build(dictionary): clear resets the
        // source set, then re-add every leaf and rebuild in one shot.
        bvh->impl.clear();
        for (int32_t i = 0; i < count; ++i)
            bvh->impl.add_or_update(ids[i], arc_to_bounds(bounds[i]));
        bvh->impl.build();
        return ARC_STATUS_OK;
    } catch (...) { arc::set_error("Static BVH build failed."); return ARC_STATUS_INTERNAL_ERROR; }
}

arc_status ARC_CALL arc_static_bvh_query(
    arc_static_bvh* bvh, arc_bp_bounds bounds,
    int32_t* output, int32_t capacity, int32_t* required) {
    if (!bvh) { arc::set_error("BVH is null."); return ARC_STATUS_INVALID_ARGUMENT; }
    try {
        bvh->scratch.clear();
        bvh->impl.query(arc_to_bounds(bounds), bvh->scratch);
        return arc::copy_results(bvh->scratch, output, capacity, required);
    } catch (...) { arc::set_error("Query failed."); return ARC_STATUS_INTERNAL_ERROR; }
}

} // extern "C"
