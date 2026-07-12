#include "broadphase.h"

#include <array>
#include <stdexcept>

namespace arc {

void StaticBvh::add_or_update(int id, const Bounds& bounds) {
    for (auto& item : source_) {
        if (item.first == id) {
            item.second = bounds;
            dirty_ = true;
            return;
        }
    }
    source_.emplace_back(id, bounds);
    dirty_ = true;
}

void StaticBvh::remove(int id) {
    const auto found = std::find_if(source_.begin(), source_.end(),
        [id](const auto& item) { return item.first == id; });
    if (found != source_.end()) {
        source_.erase(found);
        dirty_ = true;
    }
}

void StaticBvh::ensure_capacity(int leaf_capacity) {
    if (leaf_capacity < 0) throw std::out_of_range("Negative BVH capacity.");
    source_.reserve(static_cast<size_t>(leaf_capacity));
    if (leaves_.size() < static_cast<size_t>(leaf_capacity))
        leaves_.resize(static_cast<size_t>(leaf_capacity));
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
    std::sort(source_.begin(), source_.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });
    for (int i = 0; i < count; ++i)
        leaves_[static_cast<size_t>(i)] = {
            source_[static_cast<size_t>(i)].first,
            source_[static_cast<size_t>(i)].second};
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

int StaticBvh::build_range(int start, int count) {
    const int node_index = node_count_++;
    if (count == 1) {
        const Leaf leaf = leaves_[static_cast<size_t>(start)];
        nodes_[static_cast<size_t>(node_index)] = {
            leaf.bounds, -1, -1, leaf.id};
        return node_index;
    }
    Bounds bounds = leaves_[static_cast<size_t>(start)].bounds;
    int64_t min_x = bounds.center_x(), max_x = min_x;
    int64_t min_y = bounds.center_y(), max_y = min_y;
    for (int i = start + 1; i < start + count; ++i) {
        const Bounds& leaf = leaves_[static_cast<size_t>(i)].bounds;
        bounds = Bounds::unite(bounds, leaf);
        min_x = std::min(min_x, leaf.center_x());
        max_x = std::max(max_x, leaf.center_x());
        min_y = std::min(min_y, leaf.center_y());
        max_y = std::max(max_y, leaf.center_y());
    }
    int axis, split;
    find_split(start, count, min_x, max_x, min_y, max_y, axis, split);
    int middle = partition(start, count, axis, split,
        axis == 0 ? min_x : min_y, axis == 0 ? max_x : max_y);
    if (middle == start || middle == start + count) {
        std::sort(leaves_.begin() + start, leaves_.begin() + start + count,
            [axis](const Leaf& a, const Leaf& b) { return leaf_less(a, b, axis); });
        middle = start + count / 2;
    }
    const int child1 = build_range(start, middle - start);
    const int child2 = build_range(middle, start + count - middle);
    nodes_[static_cast<size_t>(node_index)] = {
        bounds, child1, child2, -1};
    return node_index;
}

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
            const Bounds& bounds = leaves_[static_cast<size_t>(i)].bounds;
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
        const Bounds& bounds = leaves_[static_cast<size_t>(left)].bounds;
        const int64_t center = axis == 0 ? bounds.center_x() : bounds.center_y();
        if (to_bin(center, min, max) <= split) {
            ++left;
        } else {
            std::swap(leaves_[static_cast<size_t>(left)],
                      leaves_[static_cast<size_t>(right)]);
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
