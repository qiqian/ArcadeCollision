// The collision world and the bulk of the C ABI. A world owns a slot array of
// colliders (each a shape + filter + broadphase proxy, addressed by a generational
// arc_handle), the hybrid broadphase, and reusable scratch buffers. Handles carry a
// world id so a stale handle from another world is rejected. This file also hosts
// the standalone broadphase C API (arc_dynamic_tree / arc_static_bvh) at the end.
// Array-returning entry points use the two-call output=null/capacity=0 protocol.
#include "broadphase.h"

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

struct Slot {
    StoredShape shape;
    arc::Bounds bounds;
    arc_collision_filter filter{};
    int32_t entity_id = 0;
    int tree_proxy = -1;
    int next_free = -1;
    uint16_t generation = 0;
    bool active = false;
    bool enabled = false;
    bool is_static = false;
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
        cast_values.reserve(static_cast<size_t>(options.initial_pair_capacity));
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

arc::Bounds swept_bounds(const arc_shape& mover, arc_vec2 motion) {
    const arc::Bounds start = arc::shape_bounds(mover);
    const arc::Bounds end = start.translated(
        arc::from_float(motion.x), arc::from_float(motion.y));
    return arc::Bounds::unite(start, end);
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

void append_casts(
    arc_world* world, const arc_shape& mover, arc_vec2 motion,
    const arc_collision_filter* filter) {
    for (int index : world->candidates) {
        Slot& slot = world->slots[static_cast<size_t>(index)];
        if (!query_slot(slot, filter)) continue;
        const arc_sweep_hit hit =
            arc::sweep_shapes(mover, arc::Vec::from(motion), slot.shape.value)
                .to_public();
        if (hit.hit)
            world->cast_values.push_back({
                make_handle(world, static_cast<size_t>(index)), hit});
    }
}

arc_status ensure_world(const arc_world* world) {
    if (world) return ARC_STATUS_OK;
    arc::set_error("World is null.");
    return ARC_STATUS_INVALID_ARGUMENT;
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
        world->cast_values.reserve(static_cast<size_t>(pair_capacity));
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
        if (slot.enabled && slot.is_static)
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

arc_status ARC_CALL arc_world_update(
    arc_world* world, arc_handle handle, const arc_shape* shape) {
    Slot* slot = get_slot(world, handle);
    if (!slot) {
        arc::set_error("Handle is stale or belongs to another world.");
        return ARC_STATUS_INVALID_HANDLE;
    }
    if (!shape || !arc::validate_shape(*shape)) {
        arc::set_error("Invalid shape.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    try {
        StoredShape stored(*shape);
        const arc::Bounds bounds = arc::shape_bounds(*shape);
        slot->shape = std::move(stored);
        slot->bounds = bounds;
        if (slot->enabled && slot->is_static)
            world->broadphase.add_or_update_static(
                static_cast<int>(handle_index(handle)), bounds);
        else if (slot->enabled)
            world->broadphase.update_dynamic(slot->tree_proxy, bounds);
        return ARC_STATUS_OK;
    } catch (const std::out_of_range& exception) {
        arc::set_error(exception.what());
        return ARC_STATUS_OUT_OF_RANGE;
    } catch (...) {
        arc::set_error("Update failed.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
}

arc_status ARC_CALL arc_world_remove(arc_world* world, arc_handle handle) {
    Slot* slot = get_slot(world, handle);
    if (!slot) {
        arc::set_error("Handle is stale or belongs to another world.");
        return ARC_STATUS_INVALID_HANDLE;
    }
    const int index = static_cast<int>(handle_index(handle));
    if (slot->enabled && slot->is_static)
        world->broadphase.remove_static(index);
    else if (slot->enabled)
        world->broadphase.remove_dynamic(slot->tree_proxy);
    if (slot->enabled) --world->enabled_count;
    if (!slot->is_static) --world->dynamic_count;
    slot->shape = StoredShape{};
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
    if (value && slot->is_static)
        world->broadphase.add_or_update_static(index, slot->bounds);
    else if (value)
        slot->tree_proxy = world->broadphase.add_dynamic(index, slot->bounds);
    else if (slot->is_static)
        world->broadphase.remove_static(index);
    else {
        world->broadphase.remove_dynamic(slot->tree_proxy);
        slot->tree_proxy = -1;
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
        const arc_vec2 delta{-origin_delta.x, -origin_delta.y};
        std::vector<std::pair<arc_shape, arc::Bounds>> moved;
        moved.reserve(static_cast<size_t>(world->active_count));
        for (Slot& slot : world->slots) {
            if (!slot.active) continue;
            const arc_shape shape = arc::moved_shape(slot.shape.value, delta);
            moved.emplace_back(shape, arc::shape_bounds(shape));
        }
        world->broadphase.clear();
        size_t moved_index = 0;
        for (size_t index = 0; index < world->slots.size(); ++index) {
            Slot& slot = world->slots[index];
            if (!slot.active) continue;
            slot.shape.value = moved[moved_index].first;
            slot.bounds = moved[moved_index].second;
            slot.tree_proxy = -1;
            ++moved_index;
            if (!slot.enabled) continue;
            if (slot.is_static)
                world->broadphase.add_or_update_static(
                    static_cast<int>(index), slot.bounds);
            else
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
    arc_world* world, arc_candidate_pair* output,
    int32_t capacity, int32_t* required) {
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
        return arc::copy_results(
            world->pair_values, output, capacity, required);
    } catch (...) {
        arc::set_error("Pair computation failed.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
}

arc_status ARC_CALL arc_world_query(
    arc_world* world, const arc_shape* query,
    const arc_collision_filter* filter, arc_handle* output,
    int32_t capacity, int32_t* required) {
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
        return arc::copy_results(
            world->query_values, output, capacity, required);
    } catch (...) {
        arc::set_error("Query failed.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
}

arc_status ARC_CALL arc_world_try_contact_pair(
    arc_world* world, arc_candidate_pair pair,
    arc_contact_pair* output, arc_bool* colliding) {
    if (!world || !output || !colliding) return ARC_STATUS_INVALID_ARGUMENT;
    *colliding = 0;
    Slot* a = get_slot(world, pair.a);
    Slot* b = get_slot(world, pair.b);
    if (!a || !b) return ARC_STATUS_INVALID_HANDLE;
    if (!a->enabled || !b->enabled
        || !arc::filter_allows(a->filter, b->filter))
        return ARC_STATUS_OK;
    const arc_manifold manifold =
        arc::collide_shapes(a->shape.value, b->shape.value).to_public();
    if (manifold.colliding) {
        *colliding = 1;
        *output = {pair.a, pair.b, manifold};
    }
    return ARC_STATUS_OK;
}

arc_status ARC_CALL arc_world_try_contact_shape(
    arc_world* world, const arc_shape* query,
    const arc_collision_filter* filter, arc_handle target,
    arc_manifold* output, arc_bool* colliding) {
    if (!world || !query || !output || !colliding
        || !arc::validate_shape(*query))
        return ARC_STATUS_INVALID_ARGUMENT;
    *colliding = 0;
    Slot* slot = get_slot(world, target);
    if (!slot) return ARC_STATUS_INVALID_HANDLE;
    if (!slot->enabled
        || (filter && !arc::filter_allows(*filter, slot->filter)))
        return ARC_STATUS_OK;
    *output = arc::collide_shapes(*query, slot->shape.value).to_public();
    *colliding = output->colliding;
    return ARC_STATUS_OK;
}

arc_status ARC_CALL arc_world_shape_cast_all(
    arc_world* world, const arc_shape* mover, arc_vec2 motion,
    const arc_collision_filter* filter, arc_world_cast_hit* output,
    int32_t capacity, int32_t* required) {
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
        return arc::copy_results(
            world->cast_values, output, capacity, required);
    } catch (...) {
        arc::set_error("Shape cast failed.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
}

arc_status ARC_CALL arc_world_shape_cast(
    arc_world* world, const arc_shape* mover, arc_vec2 motion,
    const arc_collision_filter* filter,
    arc_world_cast_hit* output, arc_bool* found) {
    if (!world || !mover || !output || !found)
        return ARC_STATUS_INVALID_ARGUMENT;
    int32_t required = 0;
    arc_status status = arc_world_shape_cast_all(
        world, mover, motion, filter, nullptr, 0, &required);
    if (required == 0) {
        *found = 0;
        return status == ARC_STATUS_BUFFER_TOO_SMALL ? ARC_STATUS_OK : status;
    }
    if (status != ARC_STATUS_BUFFER_TOO_SMALL) return status;
    std::vector<arc_world_cast_hit> values(static_cast<size_t>(required));
    status = arc_world_shape_cast_all(
        world, mover, motion, filter, values.data(), required, &required);
    if (status == ARC_STATUS_OK) {
        *output = values[0];
        *found = 1;
    }
    return status;
}

arc_status ARC_CALL arc_world_ray_cast_all(
    arc_world* world, arc_vec2 origin, arc_vec2 motion,
    const arc_collision_filter* filter, arc_world_cast_hit* output,
    int32_t capacity, int32_t* required) {
    arc_shape point{};
    point.kind = ARC_SHAPE_CIRCLE;
    point.circle = {origin, 0};
    return arc_world_shape_cast_all(
        world, &point, motion, filter, output, capacity, required);
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
