// Incremental AABB tree (Box2D-style). Leaves carry a fat AABB so small moves
// don't re-insert; insertion descends by a surface-area cost heuristic and the
// tree is kept balanced by single rotations on the way up. Mirrors the managed
// DynamicAabbTree.cs.
#include "broadphase.h"

#include <stdexcept>

namespace arc {

DynamicAabbTree::DynamicAabbTree()
    : nodes_(16), query_stack_(64), pair_stack_a_(64), pair_stack_b_(64) {
    initialize_free_list(0);
}

void DynamicAabbTree::ensure_capacity(int proxy_capacity) {
    if (proxy_capacity < 0) throw std::out_of_range("Negative proxy capacity.");
    const int required = proxy_capacity == 0 ? 0 : proxy_capacity * 2 - 1;
    while (static_cast<int>(nodes_.size()) < required) grow_nodes();
}

void DynamicAabbTree::clear() {
    root_ = -1;
    node_count_ = 0;
    leaf_count_ = 0;
    initialize_free_list(0);
}

int DynamicAabbTree::create_proxy(int id, const Bounds& bounds) {
    const int leaf = allocate_node();
    Node& node = nodes_[static_cast<size_t>(leaf)];
    node.bounds = bounds;
    node.id = id;
    node.height = 0;
    node.child1 = -1;
    node.child2 = -1;
    insert_leaf(leaf);
    ++leaf_count_;
    return leaf;
}

// Re-fit a moved proxy. If the tight bounds still sit inside the stored fat
// bounds, nothing changed structurally -- skip the reinsert (the whole point of
// the fat margin). Otherwise pull the leaf and reinsert with fresh fat bounds.
bool DynamicAabbTree::move_proxy(
    int proxy, const Bounds& bounds, const Bounds& fat_bounds) {
    if (nodes_[static_cast<size_t>(proxy)].bounds.contains(bounds)) return false;
    remove_leaf(proxy);
    nodes_[static_cast<size_t>(proxy)].bounds = fat_bounds;
    insert_leaf(proxy);
    return true;
}

void DynamicAabbTree::destroy_proxy(int proxy) {
    remove_leaf(proxy);
    free_node(proxy);
    --leaf_count_;
}

void DynamicAabbTree::query(
    const Bounds& bounds, std::vector<int>& results) const {
    if (root_ == -1) return;
    int count = 0;
    push_query(count, root_);
    while (count != 0) {
        const int index = query_stack_[static_cast<size_t>(--count)];
        const Node& node = nodes_[static_cast<size_t>(index)];
        if (!node.bounds.overlaps(bounds)) continue;
        if (node.is_leaf()) {
            results.push_back(node.id);
        } else {
            push_query(count, node.child1);
            push_query(count, node.child2);
        }
    }
}

// Existence-only box query: the same descent as query(), but it hands each
// overlapping leaf to `accept` and returns as soon as one is taken. Callers that
// only need "is anything here?" therefore skip both the candidate vector and the
// rest of the traversal. Result order is irrelevant to a boolean, so this needs
// no sort and stays deterministic across backends regardless of descent order.
bool DynamicAabbTree::query_any(
    const Bounds& bounds, ProxyPredicate accept, void* context) const {
    if (root_ == -1) return false;
    int count = 0;
    push_query(count, root_);
    while (count != 0) {
        const int index = query_stack_[static_cast<size_t>(--count)];
        const Node& node = nodes_[static_cast<size_t>(index)];
        if (!node.bounds.overlaps(bounds)) continue;
        if (node.is_leaf()) {
            if (accept(context, node.id)) return true;
        } else {
            push_query(count, node.child1);
            push_query(count, node.child2);
        }
    }
    return false;
}

// Packet (4-wide) box query: descends the tree once for four queries at a time,
// testing each node against all four with a single SIMD overlap. A per-branch
// mask drops queries that diverge, so scattered queries prune quickly while
// coherent ones share the traversal. results[i] receives the ids overlapping
// queries[i]. The caller owns `stack`, so this holds no shared state and is
// reentrant against the read-only tree.
void DynamicAabbTree::query_packet(
    const Bounds queries[4], std::vector<int>* results[4],
    std::vector<PacketFrame>& stack) const {
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
        if (node.is_leaf()) {
            for (int lane = 0; lane < 4; ++lane)
                if (mask & (1 << lane)) results[lane]->push_back(node.id);
        } else {
            stack.push_back({node.child1, mask});
            stack.push_back({node.child2, mask});
        }
    }
}

// Enumerate every pair of leaves whose fat bounds overlap, each ordered small-id
// first. Iterative dual-node descent over the tree: (node,node) self-pairs expand
// into their child cross-products, and disjoint node pairs are pruned wholesale.
void DynamicAabbTree::compute_self_pairs(
    std::vector<std::pair<int, int>>& results) const {
    if (root_ == -1 || leaf_count_ < 2) return;
    int count = 0;
    push_pair(count, root_, root_);
    while (count != 0) {
        --count;
        const int a = pair_stack_a_[static_cast<size_t>(count)];
        const int b = pair_stack_b_[static_cast<size_t>(count)];
        const Node& node_a = nodes_[static_cast<size_t>(a)];
        const Node& node_b = nodes_[static_cast<size_t>(b)];
        if (a == b) {
            if (node_a.is_leaf()) continue;
            push_pair(count, node_a.child1, node_a.child1);
            push_pair(count, node_a.child1, node_a.child2);
            push_pair(count, node_a.child2, node_a.child2);
            continue;
        }
        if (!node_a.bounds.overlaps(node_b.bounds)) continue;
        if (node_a.is_leaf() && node_b.is_leaf()) {
            results.emplace_back(
                std::min(node_a.id, node_b.id), std::max(node_a.id, node_b.id));
            continue;
        }
        if (node_b.is_leaf()
            || (!node_a.is_leaf()
                && node_a.bounds.perimeter() >= node_b.bounds.perimeter())) {
            push_pair(count, node_a.child1, b);
            push_pair(count, node_a.child2, b);
        } else {
            push_pair(count, a, node_b.child1);
            push_pair(count, a, node_b.child2);
        }
    }
}

// Descend from the root choosing the child whose subtree grows least (a
// surface-area/perimeter cost), stopping where making the leaf a direct sibling
// is cheaper than descending further; then splice in a new internal parent and
// refit/balance up to the root.
void DynamicAabbTree::insert_leaf(int leaf) {
    if (root_ == -1) {
        root_ = leaf;
        nodes_[static_cast<size_t>(leaf)].parent = -1;
        return;
    }
    const Bounds leaf_bounds = nodes_[static_cast<size_t>(leaf)].bounds;
    int sibling = root_;
    while (!nodes_[static_cast<size_t>(sibling)].is_leaf()) {
        const Node& sibling_node = nodes_[static_cast<size_t>(sibling)];
        const int child1 = sibling_node.child1;
        const int child2 = sibling_node.child2;
        const int64_t area = sibling_node.bounds.perimeter();
        const Bounds combined = Bounds::unite(sibling_node.bounds, leaf_bounds);
        const int64_t combined_area = combined.perimeter();
        const int64_t direct_cost = 2 * combined_area;
        const int64_t inheritance = 2 * (combined_area - area);
        const int64_t cost1 = descend_cost(child1, leaf_bounds, inheritance);
        const int64_t cost2 = descend_cost(child2, leaf_bounds, inheritance);
        if (direct_cost < cost1 && direct_cost < cost2) break;
        sibling = cost1 < cost2 ? child1 : child2;
    }
    const int old_parent = nodes_[static_cast<size_t>(sibling)].parent;
    const int new_parent = allocate_node();
    Node& parent = nodes_[static_cast<size_t>(new_parent)];
    parent.parent = old_parent;
    parent.bounds = Bounds::unite(leaf_bounds, nodes_[static_cast<size_t>(sibling)].bounds);
    parent.height = nodes_[static_cast<size_t>(sibling)].height + 1;
    parent.child1 = sibling;
    parent.child2 = leaf;
    parent.id = -1;
    nodes_[static_cast<size_t>(sibling)].parent = new_parent;
    nodes_[static_cast<size_t>(leaf)].parent = new_parent;
    if (old_parent == -1) {
        root_ = new_parent;
    } else if (nodes_[static_cast<size_t>(old_parent)].child1 == sibling) {
        nodes_[static_cast<size_t>(old_parent)].child1 = new_parent;
    } else {
        nodes_[static_cast<size_t>(old_parent)].child2 = new_parent;
    }
    fix_upward(new_parent);
}

int64_t DynamicAabbTree::descend_cost(
    int child, const Bounds& leaf, int64_t inheritance) const {
    const Node& node = nodes_[static_cast<size_t>(child)];
    const Bounds combined = Bounds::unite(node.bounds, leaf);
    return node.is_leaf()
        ? combined.perimeter() + inheritance
        : combined.perimeter() - node.bounds.perimeter() + inheritance;
}

void DynamicAabbTree::remove_leaf(int leaf) {
    if (leaf == root_) {
        root_ = -1;
        return;
    }
    const int parent = nodes_[static_cast<size_t>(leaf)].parent;
    const int grand_parent = nodes_[static_cast<size_t>(parent)].parent;
    const int sibling = nodes_[static_cast<size_t>(parent)].child1 == leaf
        ? nodes_[static_cast<size_t>(parent)].child2
        : nodes_[static_cast<size_t>(parent)].child1;
    if (grand_parent == -1) {
        root_ = sibling;
        nodes_[static_cast<size_t>(sibling)].parent = -1;
    } else {
        Node& grand = nodes_[static_cast<size_t>(grand_parent)];
        if (grand.child1 == parent) grand.child1 = sibling;
        else grand.child2 = sibling;
        nodes_[static_cast<size_t>(sibling)].parent = grand_parent;
        fix_upward(grand_parent);
    }
    free_node(parent);
    nodes_[static_cast<size_t>(leaf)].parent = -1;
}

void DynamicAabbTree::fix_upward(int index) {
    while (index != -1) {
        index = balance(index);
        Node& node = nodes_[static_cast<size_t>(index)];
        const Node& child1 = nodes_[static_cast<size_t>(node.child1)];
        const Node& child2 = nodes_[static_cast<size_t>(node.child2)];
        node.height = 1 + std::max(child1.height, child2.height);
        node.bounds = Bounds::unite(child1.bounds, child2.bounds);
        index = node.parent;
    }
}

// One rotation step toward balance: if a's children differ in height by >1, pivot
// the taller grandchild subtree up. Returns the new subtree root. Called for every
// node on the refit path so the tree stays within height ~log n.
int DynamicAabbTree::balance(int a) {
    Node& node_a = nodes_[static_cast<size_t>(a)];
    if (node_a.is_leaf() || node_a.height < 2) return a;
    const int b = node_a.child1;
    const int c = node_a.child2;
    const int balance_value =
        nodes_[static_cast<size_t>(c)].height - nodes_[static_cast<size_t>(b)].height;
    if (balance_value > 1) {
        const int f = nodes_[static_cast<size_t>(c)].child1;
        const int g = nodes_[static_cast<size_t>(c)].child2;
        nodes_[static_cast<size_t>(c)].child1 = a;
        replace_parent(a, c);
        node_a.parent = c;
        if (nodes_[static_cast<size_t>(f)].height > nodes_[static_cast<size_t>(g)].height) {
            nodes_[static_cast<size_t>(c)].child2 = f;
            node_a.child2 = g;
            nodes_[static_cast<size_t>(g)].parent = a;
            nodes_[static_cast<size_t>(f)].parent = c;
        } else {
            nodes_[static_cast<size_t>(c)].child2 = g;
            node_a.child2 = f;
            nodes_[static_cast<size_t>(f)].parent = a;
            nodes_[static_cast<size_t>(g)].parent = c;
        }
        refit(a);
        refit(c);
        return c;
    }
    if (balance_value < -1) {
        const int d = nodes_[static_cast<size_t>(b)].child1;
        const int e = nodes_[static_cast<size_t>(b)].child2;
        nodes_[static_cast<size_t>(b)].child1 = a;
        replace_parent(a, b);
        node_a.parent = b;
        if (nodes_[static_cast<size_t>(d)].height > nodes_[static_cast<size_t>(e)].height) {
            nodes_[static_cast<size_t>(b)].child2 = d;
            node_a.child1 = e;
            nodes_[static_cast<size_t>(e)].parent = a;
            nodes_[static_cast<size_t>(d)].parent = b;
        } else {
            nodes_[static_cast<size_t>(b)].child2 = e;
            node_a.child1 = d;
            nodes_[static_cast<size_t>(d)].parent = a;
            nodes_[static_cast<size_t>(e)].parent = b;
        }
        refit(a);
        refit(b);
        return b;
    }
    return a;
}

void DynamicAabbTree::replace_parent(int old_child, int new_child) {
    const int parent = nodes_[static_cast<size_t>(old_child)].parent;
    nodes_[static_cast<size_t>(new_child)].parent = parent;
    if (parent == -1) root_ = new_child;
    else if (nodes_[static_cast<size_t>(parent)].child1 == old_child)
        nodes_[static_cast<size_t>(parent)].child1 = new_child;
    else nodes_[static_cast<size_t>(parent)].child2 = new_child;
}

void DynamicAabbTree::refit(int index) {
    Node& node = nodes_[static_cast<size_t>(index)];
    const Node& child1 = nodes_[static_cast<size_t>(node.child1)];
    const Node& child2 = nodes_[static_cast<size_t>(node.child2)];
    node.bounds = Bounds::unite(child1.bounds, child2.bounds);
    node.height = 1 + std::max(child1.height, child2.height);
}

int DynamicAabbTree::allocate_node() {
    if (free_list_ == -1) grow_nodes();
    const int index = free_list_;
    Node& node = nodes_[static_cast<size_t>(index)];
    free_list_ = node.parent;
    node.parent = -1;
    node.child1 = -1;
    node.child2 = -1;
    node.height = 0;
    ++node_count_;
    return index;
}

void DynamicAabbTree::free_node(int index) {
    Node& node = nodes_[static_cast<size_t>(index)];
    node.parent = free_list_;
    node.height = -1;
    node.id = -1;
    free_list_ = index;
    --node_count_;
}

void DynamicAabbTree::grow_nodes() {
    const int old_length = static_cast<int>(nodes_.size());
    const int old_free = free_list_;
    nodes_.resize(static_cast<size_t>(old_length * 2));
    for (int i = old_length; i < static_cast<int>(nodes_.size()) - 1; ++i) {
        nodes_[static_cast<size_t>(i)].parent = i + 1;
        nodes_[static_cast<size_t>(i)].height = -1;
    }
    nodes_.back().parent = old_free;
    nodes_.back().height = -1;
    free_list_ = old_length;
}

void DynamicAabbTree::initialize_free_list(int start) {
    for (int i = start; i < static_cast<int>(nodes_.size()) - 1; ++i) {
        nodes_[static_cast<size_t>(i)].parent = i + 1;
        nodes_[static_cast<size_t>(i)].height = -1;
    }
    nodes_.back().parent = -1;
    nodes_.back().height = -1;
    free_list_ = start;
}

void DynamicAabbTree::push_query(int& count, int value) const {
    if (count == static_cast<int>(query_stack_.size()))
        query_stack_.resize(query_stack_.size() * 2);
    query_stack_[static_cast<size_t>(count++)] = value;
}

void DynamicAabbTree::push_pair(int& count, int a, int b) const {
    if (count == static_cast<int>(pair_stack_a_.size())) {
        pair_stack_a_.resize(pair_stack_a_.size() * 2);
        pair_stack_b_.resize(pair_stack_b_.size() * 2);
    }
    pair_stack_a_[static_cast<size_t>(count)] = a;
    pair_stack_b_[static_cast<size_t>(count)] = b;
    ++count;
}

} // namespace arc
