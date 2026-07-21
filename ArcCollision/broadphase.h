#ifndef ARCCOLLISION_BROADPHASE_H
#define ARCCOLLISION_BROADPHASE_H

#include "internal.h"

namespace arc {

// One entry of a packet (4-wide) broadphase descent: a node index plus the 4-bit
// mask of which of the four simultaneous queries are still live down this branch.
// The caller owns the descent stack so packet queries hold no shared state and
// can run concurrently against a read-only tree.
struct PacketFrame { int node; int mask; };

// Per-leaf acceptance test for the early-out queries. Returning true stops the
// descent immediately, so an "is anything here?" test does not pay for the full
// candidate set. A plain function pointer keeps the traversal in the .cpp
// (no template in this header) and lets the world supply its slot/filter check
// without the broadphase knowing about slots.
using ProxyPredicate = bool (*)(void* context, int id);

// Incremental broadphase: a self-balancing tree of fat AABB proxies. Supports
// add/move/remove in ~log n and box queries / self-pair enumeration. Nodes live
// in a pooled array with a free list; leaves store the caller's id. Mirrors the
// managed DynamicAabbTree and is exposed through the arc_dynamic_tree C API.
class DynamicAabbTree {
public:
    DynamicAabbTree();
    int create_proxy(int id, const Bounds& bounds);
    bool move_proxy(int proxy, const Bounds& bounds, const Bounds& fat_bounds);
    void destroy_proxy(int proxy);
    void query(const Bounds& bounds, std::vector<int>& results) const;
    // Stops at the first leaf `accept` returns true for; see ProxyPredicate.
    bool query_any(const Bounds& bounds, ProxyPredicate accept, void* context) const;
    void query_packet(const Bounds queries[4], std::vector<int>* results[4],
        std::vector<PacketFrame>& stack) const;
    void compute_self_pairs(std::vector<std::pair<int, int>>& results) const;
    void ensure_capacity(int capacity);
    void clear();
    int count() const { return leaf_count_; }
    int root_index() const { return root_; }
    bool is_leaf(int index) const { return nodes_[static_cast<size_t>(index)].child1 == -1; }
    const Bounds& bounds_at(int index) const { return nodes_[static_cast<size_t>(index)].bounds; }
    int child1_at(int index) const { return nodes_[static_cast<size_t>(index)].child1; }
    int child2_at(int index) const { return nodes_[static_cast<size_t>(index)].child2; }
    int id_at(int index) const { return nodes_[static_cast<size_t>(index)].id; }

private:
    struct Node {
        Bounds bounds;
        int parent = -1;
        int child1 = -1;
        int child2 = -1;
        int height = -1;
        int id = -1;
        bool is_leaf() const { return child1 == -1; }
    };
    std::vector<Node> nodes_;
    mutable std::vector<int> query_stack_;
    mutable std::vector<int> pair_stack_a_;
    mutable std::vector<int> pair_stack_b_;
    int root_ = -1;
    int free_list_ = -1;
    int node_count_ = 0;
    int leaf_count_ = 0;

    void initialize_free_list(int start);
    void grow_nodes();
    int allocate_node();
    void free_node(int index);
    void insert_leaf(int leaf);
    void remove_leaf(int leaf);
    void fix_upward(int index);
    int balance(int index);
    void replace_parent(int old_child, int new_child);
    void refit(int index);
    int64_t descend_cost(int child, const Bounds& leaf, int64_t inheritance) const;
    void push_query(int& count, int value) const;
    void push_pair(int& count, int a, int b) const;
};

// Static broadphase: a SAH-binned AABB tree built in one shot from a set of
// leaves (dirty until build()). Cheaper to query and more compact than the
// dynamic tree, for colliders that don't move. Exposed via arc_static_bvh.
class StaticBvh {
public:
    void add_or_update(int id, const Bounds& bounds);
    void remove(int id);
    void build();
    void query(const Bounds& bounds, std::vector<int>& results);
    // Stops at the first leaf `accept` returns true for; see ProxyPredicate.
    bool query_any(const Bounds& bounds, ProxyPredicate accept, void* context);
    void query_packet(const Bounds queries[4], std::vector<int>* results[4],
        std::vector<PacketFrame>& stack);
    void ensure_capacity(int capacity);
    void clear();
    int root_index() const { return root_; }
    bool is_leaf(int index) const { return nodes_[static_cast<size_t>(index)].child1 == -1; }
    const Bounds& bounds_at(int index) const { return nodes_[static_cast<size_t>(index)].bounds; }
    int child1_at(int index) const { return nodes_[static_cast<size_t>(index)].child1; }
    int child2_at(int index) const { return nodes_[static_cast<size_t>(index)].child2; }
    int id_at(int index) const { return nodes_[static_cast<size_t>(index)].id; }

private:
    static constexpr int BinCount = 12;
    struct Leaf { int id = -1; Bounds bounds; };
    struct Node { Bounds bounds; int child1 = -1; int child2 = -1; int id = -1; };
    struct Bin {
        Bounds bounds;
        int count = 0;
        bool has_bounds = false;
        void add(const Bounds& value) {
            bounds = has_bounds ? Bounds::unite(bounds, value) : value;
            has_bounds = true;
            ++count;
        }
    };
    // The input set doubles as the build working array: build() sorts it by id
    // (deterministic) then partitions it in place, so no separate leaf copy.
    std::vector<Leaf> source_;
    std::vector<Node> nodes_;
    mutable std::vector<int> query_stack_{64};
    int node_count_ = 0;
    int root_ = -1;
    bool dirty_ = false;

    int build_range(int start, int count);
    void find_split(int start, int count,
        int64_t min_x, int64_t max_x, int64_t min_y, int64_t max_y,
        int& best_axis, int& best_split);
    int partition(int start, int count, int axis, int split, int64_t min, int64_t max);
    static int to_bin(int64_t center, int64_t min, int64_t max);
    static bool leaf_less(const Leaf& a, const Leaf& b, int axis);
    void push_query(int& count, int value) const;
};

// The world's hybrid broadphase: moving colliders live in the dynamic tree,
// static colliders in the static BVH. compute_pairs reports dynamic-dynamic and
// dynamic-static candidate pairs (never static-static). The name is historical;
// it is a tree pair, not a hash grid.
class SpatialHash {
public:
    explicit SpatialHash(float fat_margin);
    float fat_margin() const { return to_float(fat_margin_); }
    void ensure_capacity(int capacity);
    int add_dynamic(int id, const Bounds& bounds);
    void update_dynamic(int proxy, const Bounds& bounds);
    void remove_dynamic(int proxy);
    void add_or_update_static(int id, const Bounds& bounds);
    void remove_static(int id);
    void query_dynamic(const Bounds& bounds, std::vector<int>& results) const;
    void query_static(const Bounds& bounds, std::vector<int>& results);
    // Early-out existence tests; stop at the first accepted leaf.
    bool query_dynamic_any(const Bounds& bounds, ProxyPredicate accept, void* context) const;
    bool query_static_any(const Bounds& bounds, ProxyPredicate accept, void* context);
    void query_dynamic_packet(const Bounds queries[4], std::vector<int>* results[4],
        std::vector<PacketFrame>& stack) const;
    void query_static_packet(const Bounds queries[4], std::vector<int>* results[4],
        std::vector<PacketFrame>& stack);
    void compute_pairs(std::vector<std::pair<int, int>>& results);
    void build_static();
    void clear();

private:
    int64_t fat_margin_;
    DynamicAabbTree dynamic_;
    StaticBvh static_;
    std::vector<int> pair_stack_dynamic_{64};
    std::vector<int> pair_stack_static_{64};

    void push_pair(int& count, int dynamic_node, int static_node);
};

} // namespace arc

#endif
