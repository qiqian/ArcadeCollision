#include "internal.h"

#include <cmath>
#include <stdexcept>

namespace arc {
namespace {

Axis canonical_axis(Axis axis) {
    return axis.x < 0 || (axis.x == 0 && axis.y < 0) ? -axis : axis;
}

bool canonical_before(Axis candidate, Axis current) {
    candidate = canonical_axis(candidate);
    current = canonical_axis(current);
    return candidate.x < current.x
        || (candidate.x == current.x && candidate.y < current.y);
}

Axis orient_axis(
    Axis axis, int64_t toward_positive, int64_t toward_negative, Vec center_delta) {
    if (toward_positive < toward_negative) return axis;
    if (toward_negative < toward_positive) return -axis;
    return axis.dot(center_delta) < 0 ? -axis : axis;
}

void project(const Proxy& proxy, Axis axis, int64_t& min, int64_t& max) {
    min = max = axis.dot(proxy.vertices[0]);
    for (size_t i = 1; i < proxy.vertices.size(); ++i) {
        const int64_t value = axis.dot(proxy.vertices[i]);
        min = std::min(min, value);
        max = std::max(max, value);
    }
    const int64_t radius = proxy.radius * AxisOne;
    min -= radius;
    max += radius;
}

Vec support(const Proxy& proxy, Axis direction) {
    Vec best = proxy.vertices[0];
    int64_t best_projection = direction.dot(best);
    for (size_t i = 1; i < proxy.vertices.size(); ++i) {
        const int64_t projection = direction.dot(proxy.vertices[i]);
        if (projection > best_projection) {
            best = proxy.vertices[i];
            best_projection = projection;
        }
    }
    return best + direction.scale(proxy.radius);
}

Vec clamp_contact(Vec contact, const FxAabb& a, const FxAabb& b) {
    const Vec amin = a.min(), amax = a.max();
    const Vec bmin = b.min(), bmax = b.max();
    const int64_t min_x = std::max(amin.x, bmin.x);
    const int64_t min_y = std::max(amin.y, bmin.y);
    const int64_t max_x = std::min(amax.x, bmax.x);
    const int64_t max_y = std::min(amax.y, bmax.y);
    if (min_x > max_x || min_y > max_y) return contact;
    return {std::clamp(contact.x, min_x, max_x),
            std::clamp(contact.y, min_y, max_y)};
}

struct SatState {
    int64_t depth = std::numeric_limits<int64_t>::max();
    int64_t overlap = std::numeric_limits<int64_t>::max();
    Axis axis;
    bool has_axis = false;
};

bool test_axis(Vec raw, const Proxy& a, const Proxy& b, SatState& state) {
    const Axis axis = Axis::from_vector(raw, {});
    if (axis.is_zero()) return true;
    int64_t min_a, max_a, min_b, max_b;
    project(a, axis, min_a, max_a);
    project(b, axis, min_b, max_b);
    const int64_t toward_positive = max_a - min_b;
    const int64_t toward_negative = max_b - min_a;
    if (toward_positive < 0 || toward_negative < 0) return false;
    const int64_t overlap = std::min(toward_positive, toward_negative);
    const Axis oriented = orient_axis(
        axis, toward_positive, toward_negative, b.center - a.center);
    if (!state.has_axis || overlap < state.overlap
        || (overlap == state.overlap && canonical_before(oriented, state.axis))) {
        state.has_axis = true;
        state.overlap = overlap;
        state.depth = round_div(overlap, AxisOne);
        state.axis = oriented;
    }
    return true;
}

bool test_edge_axes(
    const Proxy& source, const Proxy& a, const Proxy& b, SatState& state) {
    for (int edge = 0; edge < source.edge_count(); ++edge) {
        const auto points = source.edge(edge);
        const Vec delta = points.second - points.first;
        if (!test_axis({-delta.y, delta.x}, a, b, state)) return false;
    }
    return true;
}

bool test_vertex_edge_axes(
    const Proxy& vertices, const Proxy& edges,
    const Proxy& a, const Proxy& b, SatState& state) {
    for (Vec point : vertices.vertices) {
        for (int edge = 0; edge < edges.edge_count(); ++edge) {
            const auto segment = edges.edge(edge);
            const Vec closest = closest_segment(
                point, segment.first, segment.second);
            if (!test_axis(closest - point, a, b, state)) return false;
        }
    }
    return true;
}

FxManifold circle_circle(FxCircle a, FxCircle b) {
    const Vec delta = b.center - a.center;
    const int64_t radius = a.radius + b.radius;
    const int64_t distance_sq = delta.length_sq();
    if (distance_sq > radius * radius) return {};
    const int64_t distance = sqrt_i64(distance_sq);
    const Axis normal = distance > 0
        ? Axis::from_vector(delta, Axis::unit_x()) : Axis::unit_x();
    const int64_t depth = radius - distance;
    return {true, normal, depth,
            a.center + normal.scale(a.radius - depth / 2)};
}

FxManifold aabb_aabb(FxAabb a, FxAabb b) {
    const Vec delta = b.center - a.center;
    const int64_t overlap_x = a.half.x + b.half.x - std::abs(delta.x);
    if (overlap_x < 0) return {};
    const int64_t overlap_y = a.half.y + b.half.y - std::abs(delta.y);
    if (overlap_y < 0) return {};
    if (overlap_x < overlap_y) {
        const int64_t sign = delta.x < 0 ? -1 : 1;
        const Axis normal = sign < 0 ? -Axis::unit_x() : Axis::unit_x();
        const Vec contact = clamp_contact(
            {a.center.x + sign * a.half.x, b.center.y}, a, b);
        return {true, normal, overlap_x, contact};
    }
    const int64_t sign = delta.y < 0 ? -1 : 1;
    const Axis normal = sign < 0 ? -Axis::unit_y() : Axis::unit_y();
    const Vec contact = clamp_contact(
        {b.center.x, a.center.y + sign * a.half.y}, a, b);
    return {true, normal, overlap_y, contact};
}

FxManifold circle_aabb(FxCircle circle, FxAabb box) {
    const Vec min = box.min();
    const Vec max = box.max();
    const Vec closest{std::clamp(circle.center.x, min.x, max.x),
                      std::clamp(circle.center.y, min.y, max.y)};
    const Vec delta = closest - circle.center;
    const int64_t distance_sq = delta.length_sq();
    if (distance_sq > circle.radius * circle.radius) return {};
    if (distance_sq > 0) {
        const int64_t distance = sqrt_i64(distance_sq);
        return {true, Axis::from_vector(delta, Axis::unit_x()),
                circle.radius - distance, closest};
    }
    const Vec local = circle.center - box.center;
    const int64_t overlap_x = box.half.x - std::abs(local.x);
    const int64_t overlap_y = box.half.y - std::abs(local.y);
    if (overlap_x < overlap_y) {
        const int64_t out = local.x < 0 ? -1 : 1;
        const Axis normal = out < 0 ? Axis::unit_x() : -Axis::unit_x();
        return {true, normal, overlap_x + circle.radius,
                {box.center.x + out * box.half.x, circle.center.y}};
    }
    const int64_t out = local.y < 0 ? -1 : 1;
    const Axis normal = out < 0 ? Axis::unit_y() : -Axis::unit_y();
    return {true, normal, overlap_y + circle.radius,
            {circle.center.x, box.center.y + out * box.half.y}};
}

struct BoxProxy {
    Vec center;
    Axis axis_x;
    Axis axis_y;
    int64_t half_x;
    int64_t half_y;
};

BoxProxy make_box(arc_aabb box) {
    return {Vec::from(box.center), Axis::unit_x(), Axis::unit_y(),
            std::abs(from_float(box.half_extents.x)),
            std::abs(from_float(box.half_extents.y))};
}

BoxProxy make_box(arc_obb box) {
    const Axis axis_x = Axis::from_angle(box.angle);
    return {Vec::from(box.center), axis_x, axis_x.perpendicular(),
            std::abs(from_float(box.half_extents.x)),
            std::abs(from_float(box.half_extents.y))};
}

void project_box(
    const BoxProxy& box, Axis axis, int64_t& min, int64_t& max) {
    const int64_t center = axis.dot(box.center);
    const int64_t radius =
        std::abs(box.axis_x.dot(axis)) * box.half_x
        + std::abs(box.axis_y.dot(axis)) * box.half_y;
    min = center - radius;
    max = center + radius;
}

Vec box_support(const BoxProxy& box, Axis direction) {
    Vec result = box.center;
    result += box.axis_x.scale(
        box.axis_x.dot(direction) >= 0 ? box.half_x : -box.half_x);
    result += box.axis_y.scale(
        box.axis_y.dot(direction) >= 0 ? box.half_y : -box.half_y);
    return result;
}

FxAabb box_bounds(const BoxProxy& box) {
    const Vec x = box.axis_x.scale(box.half_x);
    const Vec y = box.axis_y.scale(box.half_y);
    return {box.center,
            {std::abs(x.x) + std::abs(y.x),
             std::abs(x.y) + std::abs(y.y)}};
}

bool test_box_axis(
    Axis test, const BoxProxy& a, const BoxProxy& b,
    int64_t& best_overlap, int64_t& best_depth, Axis& best_axis) {
    int64_t min_a, max_a, min_b, max_b;
    project_box(a, test, min_a, max_a);
    project_box(b, test, min_b, max_b);
    const int64_t toward_positive = max_a - min_b;
    const int64_t toward_negative = max_b - min_a;
    const int64_t overlap = std::min(toward_positive, toward_negative);
    if (overlap < 0) return false;
    const Axis oriented = orient_axis(
        test, toward_positive, toward_negative, b.center - a.center);
    if (overlap < best_overlap
        || (overlap == best_overlap && canonical_before(oriented, best_axis))) {
        best_overlap = overlap;
        best_depth = round_div(overlap, AxisOne);
        best_axis = oriented;
    }
    return true;
}

FxManifold box_box(const BoxProxy& a, const BoxProxy& b) {
    int64_t overlap = std::numeric_limits<int64_t>::max();
    int64_t depth = std::numeric_limits<int64_t>::max();
    Axis axis = Axis::unit_x();
    if (!test_box_axis(a.axis_x, a, b, overlap, depth, axis)
        || !test_box_axis(a.axis_y, a, b, overlap, depth, axis)
        || !test_box_axis(b.axis_x, a, b, overlap, depth, axis)
        || !test_box_axis(b.axis_y, a, b, overlap, depth, axis))
        return {};
    return {true, axis, depth,
            clamp_contact(midpoint(
                box_support(a, axis), box_support(b, -axis)),
                box_bounds(a), box_bounds(b))};
}

FxManifold circle_obb(arc_circle circle, arc_obb box) {
    const BoxProxy target = make_box(box);
    const FxCircle source = fixed_circle(circle);
    const Vec delta = source.center - target.center;
    const FxCircle local{{
        round_div(target.axis_x.dot(delta), AxisOne),
        round_div(target.axis_y.dot(delta), AxisOne)}, std::abs(source.radius)};
    const FxManifold result = circle_aabb(
        local, {{0, 0}, {target.half_x, target.half_y}});
    if (!result.colliding) return {};
    const Axis normal =
        Axis::transform(target.axis_x, target.axis_y, result.normal);
    const Vec contact = target.center
        + target.axis_x.scale(result.contact.x)
        + target.axis_y.scale(result.contact.y);
    return {true, normal, result.depth, contact};
}

void project_capsule(
    Vec a, Vec b, int64_t radius, Axis axis, int64_t& min, int64_t& max) {
    const int64_t first = axis.dot(a);
    const int64_t second = axis.dot(b);
    const int64_t radius_projection = radius * AxisOne;
    min = std::min(first, second) - radius_projection;
    max = std::max(first, second) + radius_projection;
}

Vec capsule_support(Vec a, Vec b, int64_t radius, Axis direction) {
    const Vec endpoint = direction.dot(a) >= direction.dot(b) ? a : b;
    return endpoint + direction.scale(radius);
}

Vec box_vertex(const BoxProxy& box, int index) {
    const int64_t x = index == 1 || index == 2 ? box.half_x : -box.half_x;
    const int64_t y = index >= 2 ? box.half_y : -box.half_y;
    return box.center + box.axis_x.scale(x) + box.axis_y.scale(y);
}

FxAabb segment_bounds(Vec a, Vec b, int64_t radius) {
    const int64_t min_x = std::min(a.x, b.x) - radius;
    const int64_t min_y = std::min(a.y, b.y) - radius;
    const int64_t max_x = std::max(a.x, b.x) + radius;
    const int64_t max_y = std::max(a.y, b.y) + radius;
    return {{(min_x + max_x) / 2, (min_y + max_y) / 2},
            {(max_x - min_x) / 2, (max_y - min_y) / 2}};
}

bool test_capsule_box_axis(
    Axis test, Vec a, Vec b, int64_t radius, const BoxProxy& box,
    int64_t& best_overlap, int64_t& best_depth, Axis& best_axis) {
    int64_t min_a, max_a, min_b, max_b;
    project_capsule(a, b, radius, test, min_a, max_a);
    project_box(box, test, min_b, max_b);
    const int64_t toward_positive = max_a - min_b;
    const int64_t toward_negative = max_b - min_a;
    const int64_t overlap = std::min(toward_positive, toward_negative);
    if (overlap < 0) return false;
    const Axis oriented = orient_axis(
        test, toward_positive, toward_negative, box.center - midpoint(a, b));
    if (overlap < best_overlap
        || (overlap == best_overlap && canonical_before(oriented, best_axis))) {
        best_overlap = overlap;
        best_depth = round_div(overlap, AxisOne);
        best_axis = oriented;
    }
    return true;
}

FxManifold capsule_box(arc_capsule capsule, const BoxProxy& box) {
    const Vec a = Vec::from(capsule.a);
    const Vec b = Vec::from(capsule.b);
    const int64_t radius = std::abs(from_float(capsule.radius));
    int64_t overlap = std::numeric_limits<int64_t>::max();
    int64_t depth = std::numeric_limits<int64_t>::max();
    Axis axis = Axis::unit_x();
    if (!test_capsule_box_axis(
            box.axis_x, a, b, radius, box, overlap, depth, axis)
        || !test_capsule_box_axis(
            box.axis_y, a, b, radius, box, overlap, depth, axis))
        return {};
    const Vec spine = b - a;
    if (spine.length_sq() != 0
        && !test_capsule_box_axis(
            Axis::from_vector({-spine.y, spine.x}, Axis::unit_y()),
            a, b, radius, box, overlap, depth, axis))
        return {};
    for (int corner = 0; corner < 4; ++corner) {
        const Vec vertex = box_vertex(box, corner);
        const Vec closest = closest_segment(vertex, a, b);
        const Vec raw_axis = closest - vertex;
        if (raw_axis.length_sq() != 0
            && !test_capsule_box_axis(
                Axis::from_vector(raw_axis, Axis::unit_x()),
                a, b, radius, box, overlap, depth, axis))
            return {};
    }
    return {true, axis, depth,
            clamp_contact(midpoint(
                capsule_support(a, b, radius, axis),
                box_support(box, -axis)),
                segment_bounds(a, b, radius), box_bounds(box))};
}

FxManifold circle_capsule(arc_circle circle, arc_capsule capsule) {
    const FxCircle fixed = fixed_circle(circle);
    const Vec a = Vec::from(capsule.a);
    const Vec b = Vec::from(capsule.b);
    const Vec closest = closest_segment(fixed.center, a, b);
    const int64_t capsule_radius = std::abs(from_float(capsule.radius));
    const Vec delta = closest - fixed.center;
    if (delta.length_sq() != 0 || (a.x == b.x && a.y == b.y))
        return circle_circle(fixed, {closest, capsule_radius});
    const int64_t depth = std::abs(fixed.radius) + capsule_radius;
    const Vec spine = b - a;
    const Axis normal = Axis::from_vector(
        {-spine.y, spine.x}, Axis::unit_x());
    return {true, normal, depth,
            fixed.center + normal.scale(std::abs(fixed.radius) - depth / 2)};
}

FxManifold reverse(FxManifold value) {
    if (value.colliding) {
        const uint8_t old_mask = value.negative_zero_mask;
        value.normal = -value.normal;
        value.negative_zero_mask = 0;
        if (value.normal.x == 0 && (old_mask & 1) == 0)
            value.negative_zero_mask |= 1;
        if (value.normal.y == 0 && (old_mask & 2) == 0)
            value.negative_zero_mask |= 2;
    }
    return value;
}

FxManifold capsule_capsule(arc_capsule first, arc_capsule second) {
    const Vec a0 = Vec::from(first.a), a1 = Vec::from(first.b);
    const Vec b0 = Vec::from(second.a), b1 = Vec::from(second.b);
    const int64_t radius_a = std::abs(from_float(first.radius));
    const int64_t radius_b = std::abs(from_float(second.radius));
    Proxy difference;
    difference.vertices = {a0 - b0, a1 - b0, a1 - b1, a0 - b1};
    difference.radius = radius_a + radius_b;
    for (Vec value : difference.vertices) difference.center += value;
    difference.center.x /= 4; difference.center.y /= 4;
    Proxy origin;
    origin.vertices = {{0, 0}};
    const FxManifold configuration = collide_proxy(difference, origin);
    if (!configuration.colliding) return {};
    const Axis normal = configuration.normal;
    auto capsule_support = [](Vec a, Vec b, int64_t radius, Axis direction) {
        const Vec endpoint = direction.dot(a) >= direction.dot(b) ? a : b;
        return endpoint + direction.scale(radius);
    };
    auto segment_bounds = [](Vec a, Vec b, int64_t radius) {
        const int64_t min_x = std::min(a.x, b.x) - radius;
        const int64_t min_y = std::min(a.y, b.y) - radius;
        const int64_t max_x = std::max(a.x, b.x) + radius;
        const int64_t max_y = std::max(a.y, b.y) + radius;
        return FxAabb{{(min_x + max_x) / 2, (min_y + max_y) / 2},
                      {(max_x - min_x) / 2, (max_y - min_y) / 2}};
    };
    const Vec contact = clamp_contact(
        midpoint(capsule_support(a0, a1, radius_a, normal),
                 capsule_support(b0, b1, radius_b, -normal)),
        segment_bounds(a0, a1, radius_a), segment_bounds(b0, b1, radius_b));
    return {true, normal, configuration.depth, contact};
}

bool is_better_piece(FxManifold candidate, FxManifold best) {
    if (!best.colliding || candidate.depth > best.depth) return true;
    if (candidate.depth < best.depth) return false;
    return canonical_before(candidate.normal, best.normal);
}

FxManifold deepest_piece(
    const arc_shape& a, const arc_shape& b, int pieces_a, int pieces_b, Vec offset) {
    FxManifold best;
    for (int i = 0; i < pieces_a; ++i) {
        const Proxy proxy_a = make_proxy(a, i).translated(offset);
        for (int j = 0; j < pieces_b; ++j) {
            const FxManifold candidate = collide_proxy(proxy_a, make_proxy(b, j));
            if (candidate.colliding && is_better_piece(candidate, best))
                best = candidate;
        }
    }
    return best;
}

Vec guaranteed_separation(const Bounds& a, const Bounds& b) {
    const int64_t left = b.min_x - a.max_x - 2;
    const int64_t right = b.max_x - a.min_x + 2;
    const int64_t down = b.min_y - a.max_y - 2;
    const int64_t up = b.max_y - a.min_y + 2;
    Vec best{left, 0};
    uint64_t best_magnitude = magnitude(left);
    if (magnitude(right) < best_magnitude) {
        best = {right, 0};
        best_magnitude = magnitude(right);
    }
    if (magnitude(down) < best_magnitude) {
        best = {0, down};
        best_magnitude = magnitude(down);
    }
    if (magnitude(up) < best_magnitude) best = {0, up};
    return best;
}

FxManifold concave_collision(
    const arc_shape& a, const arc_shape& b, int pieces_a, int pieces_b) {
    Vec offset;
    const FxManifold first = deepest_piece(a, b, pieces_a, pieces_b, offset);
    if (!first.colliding) return {};
    const int limit = std::min(64, 8 + 2 * (pieces_a + pieces_b));
    for (int iteration = 0; iteration < limit; ++iteration) {
        const FxManifold candidate =
            deepest_piece(a, b, pieces_a, pieces_b, offset);
        if (!candidate.colliding) {
            const int64_t depth = offset.length() + 2;
            return {true, Axis::from_vector(-offset, first.normal),
                    depth, first.contact};
        }
        offset -= candidate.normal.scale(candidate.depth + 2);
    }
    const Vec separation = guaranteed_separation(shape_bounds(a), shape_bounds(b));
    return {true, Axis::from_vector(-separation, first.normal),
            separation.length(), first.contact};
}

} // namespace

FxManifold collide_proxy(const Proxy& a, const Proxy& b) {
    if (a.vertices.empty() || b.vertices.empty()) return {};
    SatState state;
    if (!test_edge_axes(a, a, b, state)
        || !test_edge_axes(b, a, b, state))
        return {};
    if (a.radius != 0 || b.radius != 0) {
        if (!test_vertex_edge_axes(a, b, a, b, state)
            || !test_vertex_edge_axes(b, a, a, b, state))
            return {};
    }
    if (!state.has_axis) {
        Vec fallback = b.center - a.center;
        if (fallback.length_sq() == 0) fallback = {FxOne, 0};
        if (!test_axis(fallback, a, b, state)) return {};
    }
    const Vec contact = clamp_contact(
        midpoint(support(a, state.axis), support(b, -state.axis)),
        proxy_bounds(a), proxy_bounds(b));
    return {true, state.axis, state.depth, contact};
}

FxManifold collide_shapes(const arc_shape& a, const arc_shape& b) {
    if (a.kind == ARC_SHAPE_CIRCLE && b.kind == ARC_SHAPE_CIRCLE)
        return circle_circle(fixed_circle(a.circle), fixed_circle(b.circle));
    if (a.kind == ARC_SHAPE_AABB && b.kind == ARC_SHAPE_AABB)
        return aabb_aabb(fixed_aabb(a.aabb), fixed_aabb(b.aabb));
    if (a.kind == ARC_SHAPE_CIRCLE && b.kind == ARC_SHAPE_AABB)
        return circle_aabb(fixed_circle(a.circle), fixed_aabb(b.aabb));
    if (a.kind == ARC_SHAPE_AABB && b.kind == ARC_SHAPE_CIRCLE)
        return reverse(circle_aabb(fixed_circle(b.circle), fixed_aabb(a.aabb)));
    if (a.kind == ARC_SHAPE_CIRCLE && b.kind == ARC_SHAPE_CAPSULE)
        return circle_capsule(a.circle, b.capsule);
    if (a.kind == ARC_SHAPE_CAPSULE && b.kind == ARC_SHAPE_CIRCLE)
        return reverse(circle_capsule(b.circle, a.capsule));
    if (a.kind == ARC_SHAPE_CAPSULE && b.kind == ARC_SHAPE_CAPSULE)
        return capsule_capsule(a.capsule, b.capsule);
    if (a.kind == ARC_SHAPE_CIRCLE && b.kind == ARC_SHAPE_OBB)
        return circle_obb(a.circle, b.obb);
    if (a.kind == ARC_SHAPE_OBB && b.kind == ARC_SHAPE_CIRCLE)
        return reverse(circle_obb(b.circle, a.obb));
    if (a.kind == ARC_SHAPE_AABB && b.kind == ARC_SHAPE_OBB)
        return box_box(make_box(a.aabb), make_box(b.obb));
    if (a.kind == ARC_SHAPE_OBB && b.kind == ARC_SHAPE_AABB)
        return box_box(make_box(a.obb), make_box(b.aabb));
    if (a.kind == ARC_SHAPE_OBB && b.kind == ARC_SHAPE_OBB)
        return box_box(make_box(a.obb), make_box(b.obb));
    if (a.kind == ARC_SHAPE_CAPSULE && b.kind == ARC_SHAPE_AABB)
        return capsule_box(a.capsule, make_box(b.aabb));
    if (a.kind == ARC_SHAPE_AABB && b.kind == ARC_SHAPE_CAPSULE)
        return reverse(capsule_box(b.capsule, make_box(a.aabb)));
    if (a.kind == ARC_SHAPE_CAPSULE && b.kind == ARC_SHAPE_OBB)
        return capsule_box(a.capsule, make_box(b.obb));
    if (a.kind == ARC_SHAPE_OBB && b.kind == ARC_SHAPE_CAPSULE)
        return reverse(capsule_box(b.capsule, make_box(a.obb)));

    const int pieces_a = piece_count(a);
    const int pieces_b = piece_count(b);
    if (pieces_a > 1 || pieces_b > 1)
        return concave_collision(a, b, pieces_a, pieces_b);
    FxManifold best;
    for (int i = 0; i < pieces_a; ++i) {
        const Proxy proxy_a = make_proxy(a, i);
        for (int j = 0; j < pieces_b; ++j) {
            const FxManifold candidate = collide_proxy(proxy_a, make_proxy(b, j));
            if (candidate.colliding
                && (!best.colliding || candidate.depth > best.depth))
                best = candidate;
        }
    }
    return best;
}

bool overlap_shapes(const arc_shape& a, const arc_shape& b) {
    return collide_shapes(a, b).colliding;
}

} // namespace arc

namespace {

arc_shape shape_circle(arc_circle value) {
    arc_shape shape{};
    shape.kind = ARC_SHAPE_CIRCLE;
    shape.circle = value;
    return shape;
}
arc_shape shape_aabb(arc_aabb value) {
    arc_shape shape{};
    shape.kind = ARC_SHAPE_AABB;
    shape.aabb = value;
    return shape;
}
arc_shape shape_capsule(arc_capsule value) {
    arc_shape shape{};
    shape.kind = ARC_SHAPE_CAPSULE;
    shape.capsule = value;
    return shape;
}

arc_manifold collide_checked(const arc_shape* a, const arc_shape* b) {
    if (!a || !b || !arc::validate_shape(*a) || !arc::validate_shape(*b)) {
        arc::set_error("Invalid shape.");
        return {};
    }
    try {
        return arc::collide_shapes(*a, *b).to_public();
    } catch (const std::exception& exception) {
        arc::set_error(exception.what());
        return {};
    } catch (...) {
        arc::set_error("Collision failed.");
        return {};
    }
}

} // namespace

extern "C" {

arc_bool ARC_CALL arc_point_in_circle(arc_vec2 point, arc_circle circle) {
    try {
        const arc::Vec p = arc::Vec::from(point);
        const arc::FxCircle c = arc::fixed_circle(circle);
        return p.dist_sq(c.center) <= c.radius * c.radius;
    } catch (...) { return 0; }
}

arc_bool ARC_CALL arc_point_in_aabb(arc_vec2 point, arc_aabb box) {
    try {
        const arc::Vec p = arc::Vec::from(point);
        const arc::FxAabb b = arc::fixed_aabb(box);
        return std::abs(p.x - b.center.x) <= b.half.x
            && std::abs(p.y - b.center.y) <= b.half.y;
    } catch (...) { return 0; }
}

arc_bool ARC_CALL arc_point_in_capsule(arc_vec2 point, arc_capsule capsule) {
    try {
        const arc::Vec p = arc::Vec::from(point);
        const arc::Vec closest = arc::closest_segment(
            p, arc::Vec::from(capsule.a), arc::Vec::from(capsule.b));
        const int64_t radius = arc::from_float(capsule.radius);
        return p.dist_sq(closest) <= radius * radius;
    } catch (...) { return 0; }
}

arc_manifold ARC_CALL arc_shape_vs_shape(const arc_shape* a, const arc_shape* b) {
    return collide_checked(a, b);
}

arc_bool ARC_CALL arc_shapes_overlap(const arc_shape* a, const arc_shape* b) {
    if (!a || !b || !arc::validate_shape(*a) || !arc::validate_shape(*b)) {
        arc::set_error("Invalid shape.");
        return 0;
    }
    try { return arc::overlap_shapes(*a, *b); }
    catch (...) { arc::set_error("Overlap test failed."); return 0; }
}

arc_aabb ARC_CALL arc_shape_get_bounds(const arc_shape* shape) {
    if (!shape || !arc::validate_shape(*shape)) {
        arc::set_error("Invalid shape.");
        return {};
    }
    try {
        switch (shape->kind) {
        case ARC_SHAPE_CIRCLE:
            return {shape->circle.center,
                {shape->circle.radius, shape->circle.radius}};
        case ARC_SHAPE_AABB:
            return shape->aabb;
        case ARC_SHAPE_CAPSULE: {
            const float min_x = std::min(shape->capsule.a.x, shape->capsule.b.x)
                - shape->capsule.radius;
            const float min_y = std::min(shape->capsule.a.y, shape->capsule.b.y)
                - shape->capsule.radius;
            const float max_x = std::max(shape->capsule.a.x, shape->capsule.b.x)
                + shape->capsule.radius;
            const float max_y = std::max(shape->capsule.a.y, shape->capsule.b.y)
                + shape->capsule.radius;
            return {{(min_x + max_x) * 0.5f, (min_y + max_y) * 0.5f},
                    {(max_x - min_x) * 0.5f, (max_y - min_y) * 0.5f}};
        }
        case ARC_SHAPE_OBB: {
            const arc::Axis x = arc::Axis::from_angle(shape->obb.angle);
            const arc::Axis y = x.perpendicular();
            const int64_t half_x =
                std::abs(arc::from_float(shape->obb.half_extents.x));
            const int64_t half_y =
                std::abs(arc::from_float(shape->obb.half_extents.y));
            return {shape->obb.center, {
                arc::to_float(arc::ceil_div_positive(
                    std::abs(x.x) * half_x + std::abs(y.x) * half_y,
                    arc::AxisOne)),
                arc::to_float(arc::ceil_div_positive(
                    std::abs(x.y) * half_x + std::abs(y.y) * half_y,
                    arc::AxisOne))}};
        }
        case ARC_SHAPE_POLYGON:
            if (shape->polygon_rotation == 0) {
                arc_aabb bounds = shape->polygon->public_bounds;
                bounds.center.x += shape->polygon_translation.x;
                bounds.center.y += shape->polygon_translation.y;
                return bounds;
            }
            return arc::shape_bounds(*shape).to_public();
        default:
            return {};
        }
    }
    catch (...) { arc::set_error("Bounds calculation failed."); return {}; }
}

arc_manifold ARC_CALL arc_circle_vs_circle(arc_circle a, arc_circle b) {
    const arc_shape first = shape_circle(a), second = shape_circle(b);
    return collide_checked(&first, &second);
}
arc_manifold ARC_CALL arc_aabb_vs_aabb(arc_aabb a, arc_aabb b) {
    const arc_shape first = shape_aabb(a), second = shape_aabb(b);
    return collide_checked(&first, &second);
}
arc_manifold ARC_CALL arc_circle_vs_aabb(arc_circle a, arc_aabb b) {
    const arc_shape first = shape_circle(a), second = shape_aabb(b);
    return collide_checked(&first, &second);
}
arc_manifold ARC_CALL arc_circle_vs_capsule(arc_circle a, arc_capsule b) {
    const arc_shape first = shape_circle(a), second = shape_capsule(b);
    return collide_checked(&first, &second);
}
arc_manifold ARC_CALL arc_capsule_vs_capsule(arc_capsule a, arc_capsule b) {
    const arc_shape first = shape_capsule(a), second = shape_capsule(b);
    return collide_checked(&first, &second);
}
arc_manifold ARC_CALL arc_capsule_vs_aabb(arc_capsule a, arc_aabb b) {
    const arc_shape first = shape_capsule(a), second = shape_aabb(b);
    return collide_checked(&first, &second);
}

} // extern "C"
