// One-shot BVH for static colliders. build() recurses over the leaf set, at each
// node binning centroids along the wider axis and picking the split that minimizes
// a surface-area heuristic (perimeter * count on each side), falling back to a
// median split when binning can't separate them. Mirrors managed StaticBvh.cs.
#include "broadphase.h"

#include <array>
#include <stdexcept>

namespace arc {

void StaticBvh::add_or_update(int id, const Bounds& bounds) {
    for (auto& item : source_) {
        if (item.id == id) {
            item.bounds = bounds;
            dirty_ = true;
            return;
        }
    }
    source_.push_back({id, bounds});
    dirty_ = true;
}

void StaticBvh::remove(int id) {
    const auto found = std::find_if(source_.begin(), source_.end(),
        [id](const Leaf& item) { return item.id == id; });
    if (found != source_.end()) {
        source_.erase(found);
        dirty_ = true;
    }
}

void StaticBvh::ensure_capacity(int leaf_capacity) {
    if (leaf_capacity < 0) throw std::out_of_range("Negative BVH capacity.");
    source_.reserve(static_cast<size_t>(leaf_capacity));
    const int required = leaf_capacity == 0 ? 0 : leaf_capacity * 2 - 1;
    if (nodes_.size() < static_cast<size_t>(required))
        nodes_.resize(static_cast<size_t>(required));
}

void StaticBvh::clear() {
    root_ = -1;
    node_count_ = 0;
    source_.clear();
    dirty_ = false;
}

void StaticBvh::build() {
    if (!dirty_) return;
    const int count = static_cast<int>(source_.size());
    if (count == 0) {
        clear();
        dirty_ = false;
        return;
    }
    ensure_capacity(count);
    // Sort by id so the tree is deterministic regardless of add order; the
    // recursion then partitions source_ in place (order between builds is
    // irrelevant, since every build re-sorts from scratch).
    std::sort(source_.begin(), source_.end(),
        [](const Leaf& a, const Leaf& b) { return a.id < b.id; });
    const int required = count * 2 - 1;
    if (nodes_.size() < static_cast<size_t>(required))
        nodes_.resize(static_cast<size_t>(required));
    node_count_ = 0;
    root_ = build_range(0, count);
    dirty_ = false;
}

void StaticBvh::query(
    const Bounds& bounds, std::vector<int>& results) {
    build();
    if (root_ == -1) return;
    int count = 0;
    push_query(count, root_);
    while (count != 0) {
        const Node& node =
            nodes_[static_cast<size_t>(query_stack_[static_cast<size_t>(--count)])];
        if (!node.bounds.overlaps(bounds)) continue;
        if (node.child1 == -1) {
            results.push_back(node.id);
        } else {
            push_query(count, node.child1);
            push_query(count, node.child2);
        }
    }
}

// Existence-only box query; see DynamicAabbTree::query_any. Builds the tree
// first, exactly like the scalar query, then stops at the first accepted leaf.
bool StaticBvh::query_any(
    const Bounds& bounds, ProxyPredicate accept, void* context) {
    build();
    if (root_ == -1) return false;
    int count = 0;
    push_query(count, root_);
    while (count != 0) {
        const Node& node =
            nodes_[static_cast<size_t>(query_stack_[static_cast<size_t>(--count)])];
        if (!node.bounds.overlaps(bounds)) continue;
        if (node.child1 == -1) {
            if (accept(context, node.id)) return true;
        } else {
            push_query(count, node.child1);
            push_query(count, node.child2);
        }
    }
    return false;
}

// Packet (4-wide) box query; see DynamicAabbTree::query_packet for the descent.
// Builds the tree first (like the scalar query), then shares one traversal across
// four queries with a single SIMD overlap per node and a per-branch live mask.
void StaticBvh::query_packet(
    const Bounds queries[4], std::vector<int>* results[4],
    std::vector<PacketFrame>& stack) {
    build();
    if (root_ == -1) return;
    const simd128::Box4 q0 = simd128::load_box(&queries[0].min_x);
    const simd128::Box4 q1 = simd128::load_box(&queries[1].min_x);
    const simd128::Box4 q2 = simd128::load_box(&queries[2].min_x);
    const simd128::Box4 q3 = simd128::load_box(&queries[3].min_x);
    stack.clear();
    stack.push_back({root_, 0xF});
    while (!stack.empty()) {
        const PacketFrame frame = stack.back();
        stack.pop_back();
        const Node& node = nodes_[static_cast<size_t>(frame.node)];
        const int mask = simd128::overlap_mask_broadcast(
            simd128::load_box(&node.bounds.min_x), q0, q1, q2, q3) & frame.mask;
        if (mask == 0) continue;
        if (node.child1 == -1) {
            for (int lane = 0; lane < 4; ++lane)
                if (mask & (1 << lane)) results[lane]->push_back(node.id);
        } else {
            stack.push_back({node.child1, mask});
            stack.push_back({node.child2, mask});
        }
    }
}

// Recursively build the subtree over leaves [start, start+count). Compute the
// node bounds and centroid spread, pick a split (SAH-binned), partition in place,
// and recurse. A degenerate split (all on one side) falls back to a median sort so
// the recursion always makes progress.
int StaticBvh::build_range(int start, int count) {
    const int node_index = node_count_++;
    if (count == 1) {
        const Leaf leaf = source_[static_cast<size_t>(start)];
        nodes_[static_cast<size_t>(node_index)] = {
            leaf.bounds, -1, -1, leaf.id};
        return node_index;
    }
    Bounds bounds = source_[static_cast<size_t>(start)].bounds;
    int64_t min_x = bounds.center_x(), max_x = min_x;
    int64_t min_y = bounds.center_y(), max_y = min_y;
    for (int i = start + 1; i < start + count; ++i) {
        const Bounds& leaf = source_[static_cast<size_t>(i)].bounds;
        bounds = Bounds::unite(bounds, leaf);
        min_x = std::min(min_x, static_cast<int64_t>(leaf.center_x()));
        max_x = std::max(max_x, static_cast<int64_t>(leaf.center_x()));
        min_y = std::min(min_y, static_cast<int64_t>(leaf.center_y()));
        max_y = std::max(max_y, static_cast<int64_t>(leaf.center_y()));
    }
    int axis, split;
    find_split(start, count, min_x, max_x, min_y, max_y, axis, split);
    int middle = partition(start, count, axis, split,
        axis == 0 ? min_x : min_y, axis == 0 ? max_x : max_y);
    if (middle == start || middle == start + count) {
        std::sort(source_.begin() + start, source_.begin() + start + count,
            [axis](const Leaf& a, const Leaf& b) { return leaf_less(a, b, axis); });
        middle = start + count / 2;
    }
    const int child1 = build_range(start, middle - start);
    const int child2 = build_range(middle, start + count - middle);
    nodes_[static_cast<size_t>(node_index)] = {
        bounds, child1, child2, -1};
    return node_index;
}

// Surface-area-heuristic split search: bin the centroids into BinCount buckets on
// each axis, sweep prefix/suffix bounds+counts, and choose the (axis, boundary)
// minimizing left.perimeter*left.count + right.perimeter*right.count.
void StaticBvh::find_split(
    int start, int count, int64_t min_x, int64_t max_x,
    int64_t min_y, int64_t max_y, int& best_axis, int& best_split) {
    int64_t best_cost = std::numeric_limits<int64_t>::max();
    best_axis = max_x - min_x >= max_y - min_y ? 0 : 1;
    best_split = BinCount / 2 - 1;
    std::array<Bin, BinCount> bins{};
    std::array<Bounds, BinCount - 1> left_bounds{};
    std::array<Bounds, BinCount - 1> right_bounds{};
    std::array<int, BinCount - 1> left_counts{};
    std::array<int, BinCount - 1> right_counts{};
    for (int axis = 0; axis < 2; ++axis) {
        const int64_t min = axis == 0 ? min_x : min_y;
        const int64_t max = axis == 0 ? max_x : max_y;
        if (min == max) continue;
        bins = {};
        for (int i = start; i < start + count; ++i) {
            const Bounds& bounds = source_[static_cast<size_t>(i)].bounds;
            const int64_t center = axis == 0 ? bounds.center_x() : bounds.center_y();
            bins[static_cast<size_t>(to_bin(center, min, max))].add(bounds);
        }
        Bounds accumulated{};
        int accumulated_count = 0;
        bool has_bounds = false;
        for (int i = 0; i < BinCount - 1; ++i) {
            const Bin& bin = bins[static_cast<size_t>(i)];
            if (bin.has_bounds) {
                accumulated = has_bounds
                    ? Bounds::unite(accumulated, bin.bounds) : bin.bounds;
                has_bounds = true;
            }
            accumulated_count += bin.count;
            left_bounds[static_cast<size_t>(i)] = accumulated;
            left_counts[static_cast<size_t>(i)] = accumulated_count;
        }
        accumulated = {};
        accumulated_count = 0;
        has_bounds = false;
        for (int i = BinCount - 1; i > 0; --i) {
            const Bin& bin = bins[static_cast<size_t>(i)];
            if (bin.has_bounds) {
                accumulated = has_bounds
                    ? Bounds::unite(accumulated, bin.bounds) : bin.bounds;
                has_bounds = true;
            }
            accumulated_count += bin.count;
            right_bounds[static_cast<size_t>(i - 1)] = accumulated;
            right_counts[static_cast<size_t>(i - 1)] = accumulated_count;
        }
        for (int split = 0; split < BinCount - 1; ++split) {
            const int left_count = left_counts[static_cast<size_t>(split)];
            const int right_count = right_counts[static_cast<size_t>(split)];
            if (left_count == 0 || right_count == 0) continue;
            const int64_t cost =
                left_bounds[static_cast<size_t>(split)].perimeter() * left_count
                + right_bounds[static_cast<size_t>(split)].perimeter() * right_count;
            if (cost < best_cost) {
                best_cost = cost;
                best_axis = axis;
                best_split = split;
            }
        }
    }
}

int StaticBvh::partition(
    int start, int count, int axis, int split, int64_t min, int64_t max) {
    int left = start;
    int right = start + count - 1;
    while (left <= right) {
        const Bounds& bounds = source_[static_cast<size_t>(left)].bounds;
        const int64_t center = axis == 0 ? bounds.center_x() : bounds.center_y();
        if (to_bin(center, min, max) <= split) {
            ++left;
        } else {
            std::swap(source_[static_cast<size_t>(left)],
                      source_[static_cast<size_t>(right)]);
            --right;
        }
    }
    return left;
}

int StaticBvh::to_bin(int64_t center, int64_t min, int64_t max) {
    const int64_t range = max - min;
    if (range <= 0) return 0;
    const int bin = static_cast<int>(((center - min) * BinCount) / (range + 1));
    return std::min(bin, BinCount - 1);
}

bool StaticBvh::leaf_less(const Leaf& a, const Leaf& b, int axis) {
    auto compare = [](int64_t x, int64_t y) { return x < y ? -1 : x > y ? 1 : 0; };
    int result = axis == 0
        ? compare(a.bounds.center_x(), b.bounds.center_x())
        : compare(a.bounds.center_y(), b.bounds.center_y());
    if (result != 0) return result < 0;
    result = axis == 0
        ? compare(a.bounds.center_y(), b.bounds.center_y())
        : compare(a.bounds.center_x(), b.bounds.center_x());
    if (result != 0) return result < 0;
    result = compare(a.bounds.min_x, b.bounds.min_x);
    if (result != 0) return result < 0;
    result = compare(a.bounds.min_y, b.bounds.min_y);
    if (result != 0) return result < 0;
    result = compare(a.bounds.max_x, b.bounds.max_x);
    if (result != 0) return result < 0;
    result = compare(a.bounds.max_y, b.bounds.max_y);
    return result != 0 ? result < 0 : a.id < b.id;
}

void StaticBvh::push_query(int& count, int value) const {
    if (count == static_cast<int>(query_stack_.size()))
        query_stack_.resize(query_stack_.size() * 2);
    query_stack_[static_cast<size_t>(count++)] = value;
}

} // namespace arc
