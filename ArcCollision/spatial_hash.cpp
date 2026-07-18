// Hybrid broadphase driver: keeps a dynamic tree (movers) and a static BVH, and
// merges their candidate pairs. compute_pairs = dynamic self-pairs + a dual
// descent between the dynamic tree and the static BVH; static-static pairs are
// never reported (static geometry does not collide with itself). Mirrors the
// managed SpatialHash.cs.
#include "broadphase.h"

#include <stdexcept>

namespace arc {

SpatialHash::SpatialHash(float fat_margin)
    : fat_margin_(from_float(fat_margin)) {
    if (fat_margin_ < 0)
        throw std::out_of_range("Fat margin must be non-negative.");
}

void SpatialHash::ensure_capacity(int capacity) {
    if (capacity < 0) throw std::out_of_range("Negative broadphase capacity.");
    dynamic_.ensure_capacity(capacity);
    static_.ensure_capacity(capacity);
}

int SpatialHash::add_dynamic(int id, const Bounds& bounds) {
    return dynamic_.create_proxy(id, bounds.expanded(fat_margin_));
}

void SpatialHash::update_dynamic(int proxy, const Bounds& bounds) {
    dynamic_.move_proxy(proxy, bounds, bounds.expanded(fat_margin_));
}

void SpatialHash::remove_dynamic(int proxy) { dynamic_.destroy_proxy(proxy); }
void SpatialHash::add_or_update_static(int id, const Bounds& bounds) {
    static_.add_or_update(id, bounds);
}
void SpatialHash::remove_static(int id) { static_.remove(id); }
void SpatialHash::query_dynamic(
    const Bounds& bounds, std::vector<int>& results) const {
    dynamic_.query(bounds, results);
}
void SpatialHash::query_static(
    const Bounds& bounds, std::vector<int>& results) {
    static_.query(bounds, results);
}

void SpatialHash::query_dynamic_packet(
    const Bounds queries[4], std::vector<int>* results[4],
    std::vector<PacketFrame>& stack) const {
    dynamic_.query_packet(queries, results, stack);
}

void SpatialHash::query_static_packet(
    const Bounds queries[4], std::vector<int>* results[4],
    std::vector<PacketFrame>& stack) {
    static_.query_packet(queries, results, stack);
}

void SpatialHash::compute_pairs(std::vector<std::pair<int, int>>& results) {
    results.clear();
    dynamic_.compute_self_pairs(results);
    static_.build();
    const int dynamic_root = dynamic_.root_index();
    const int static_root = static_.root_index();
    if (dynamic_root == -1 || static_root == -1) return;
    int count = 0;
    push_pair(count, dynamic_root, static_root);
    while (count != 0) {
        --count;
        const int dynamic_node =
            pair_stack_dynamic_[static_cast<size_t>(count)];
        const int static_node =
            pair_stack_static_[static_cast<size_t>(count)];
        const Bounds& dynamic_bounds = dynamic_.bounds_at(dynamic_node);
        const Bounds& static_bounds = static_.bounds_at(static_node);
        if (!dynamic_bounds.overlaps(static_bounds)) continue;
        const bool dynamic_leaf = dynamic_.is_leaf(dynamic_node);
        const bool static_leaf = static_.is_leaf(static_node);
        if (dynamic_leaf && static_leaf) {
            results.emplace_back(
                dynamic_.id_at(dynamic_node), static_.id_at(static_node));
            continue;
        }
        if (static_leaf
            || (!dynamic_leaf
                && dynamic_bounds.perimeter() >= static_bounds.perimeter())) {
            push_pair(count, dynamic_.child1_at(dynamic_node), static_node);
            push_pair(count, dynamic_.child2_at(dynamic_node), static_node);
        } else {
            push_pair(count, dynamic_node, static_.child1_at(static_node));
            push_pair(count, dynamic_node, static_.child2_at(static_node));
        }
    }
}

void SpatialHash::build_static() { static_.build(); }

void SpatialHash::clear() {
    dynamic_.clear();
    static_.clear();
}

void SpatialHash::push_pair(
    int& count, int dynamic_node, int static_node) {
    if (count == static_cast<int>(pair_stack_dynamic_.size())) {
        pair_stack_dynamic_.resize(pair_stack_dynamic_.size() * 2);
        pair_stack_static_.resize(pair_stack_static_.size() * 2);
    }
    pair_stack_dynamic_[static_cast<size_t>(count)] = dynamic_node;
    pair_stack_static_[static_cast<size_t>(count)] = static_node;
    ++count;
}

} // namespace arc
