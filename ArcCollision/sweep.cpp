// Continuous (swept) collision: earliest time-of-impact of a moving shape against
// a static target, to stop fast movers tunnelling through thin geometry. Rays use
// closed forms (slab method for boxes, a quadratic for circles); general shapes
// use a conservative-advancement SAT. Times are 16.16 fractions of the motion.
// Near tangency the TOI is ill-conditioned, so accuracy is judged by clearance at
// the reported time rather than the time itself. Mirrors ArcCollision.Ref/Sweep.
#include "internal.h"

#include <cmath>

namespace arc {
namespace {

// Keep the earliest-hit candidate.
void add_earlier(FxSweep candidate, FxSweep& best) {
    if (candidate.hit && (!best.hit || candidate.time < best.time))
        best = candidate;
}

// One axis of the slab method: intersect the motion with the [slabMin, slabMax]
// span on this axis, tightening the running [tMin, tMax] entry/exit interval and
// tracking which face produced the latest entry (the hit normal).
bool slab(
    int64_t origin, int64_t direction, int64_t min, int64_t max,
    Axis min_normal, Axis max_normal,
    int64_t& enter, int64_t& exit, Axis& normal) {
    if (direction == 0) return origin >= min && origin <= max;
    int64_t first = ratio_t(min - origin, direction);
    int64_t second = ratio_t(max - origin, direction);
    Axis first_normal = min_normal;
    if (first > second) {
        std::swap(first, second);
        first_normal = max_normal;
    }
    if (first > enter) {
        enter = first;
        normal = first_normal;
    }
    if (second < exit) exit = second;
    return enter <= exit;
}

FxSweep moving_circle_aabb(FxCircle mover, Vec motion, FxAabb box) {
    const int64_t radius = mover.radius;
    const FxAabb expanded{
        box.center, {box.half.x + radius, box.half.y + radius}};
    FxSweep best;
    const FxSweep face = ray_aabb(mover.center, motion, expanded);
    if (face.hit) {
        const Vec at = mover.center + motion.times_t(face.time);
        const Vec min = box.min(), max = box.max();
        const bool corner_zone =
            (at.x < min.x || at.x > max.x) && (at.y < min.y || at.y > max.y);
        if (!corner_zone) best = face;
    }
    const Vec min = box.min(), max = box.max();
    const std::array<Vec, 4> corners{{
        min, {max.x, min.y}, max, {min.x, max.y},
    }};
    for (Vec corner : corners)
        add_earlier(ray_circle(mover.center, motion, {corner, radius}), best);
    if (!best.hit) return {};
    if (best.time == 0) {
        const FxManifold initial = collide_circle_aabb(mover, box);
        if (initial.colliding)
            return {true, 0, -initial.normal, initial.contact};
    }
    const Vec at = mover.center + motion.times_t(best.time);
    return {true, best.time, best.normal, at - best.normal.scale(radius)};
}

FxSweep moving_circle_circle(FxCircle mover, Vec motion, FxCircle target) {
    const FxSweep hit = ray_circle(
        mover.center, motion, {target.center, target.radius + mover.radius});
    if (!hit.hit) return {};
    if (hit.time == 0) {
        const FxManifold initial = collide_circle_circle(mover, target);
        if (initial.colliding)
            return {true, 0, -initial.normal, initial.contact};
    }
    const Vec center = mover.center + motion.times_t(hit.time);
    return {true, hit.time, hit.normal,
            center - hit.normal.scale(mover.radius)};
}

FxSweep moving_circle_capsule(
    FxCircle mover, Vec motion, arc_capsule target) {
    const int64_t radius =
        std::abs(mover.radius) + std::abs(from_float(target.radius));
    const FxSweep hit = ray_capsule(
        mover.center, motion, Vec::from(target.a), Vec::from(target.b), radius);
    if (!hit.hit) return {};
    return {true, hit.time, hit.normal,
            hit.point - hit.normal.scale(std::abs(mover.radius))};
}

FxSweep moving_circle_obb(FxCircle mover, Vec motion, arc_obb target) {
    const Axis axis_x = Axis::from_angle(target.angle);
    const Axis axis_y = axis_x.perpendicular();
    const Vec center = Vec::from(target.center);
    const Vec relative = mover.center - center;
    const FxCircle local_circle{{
        round_div(axis_x.dot(relative), AxisOne),
        round_div(axis_y.dot(relative), AxisOne)}, std::abs(mover.radius)};
    const Vec local_motion{
        round_div(axis_x.dot(motion), AxisOne),
        round_div(axis_y.dot(motion), AxisOne)};
    const FxAabb local_box{{0, 0}, {
        std::abs(from_float(target.half_extents.x)),
        std::abs(from_float(target.half_extents.y))}};
    const FxSweep local = moving_circle_aabb(local_circle, local_motion, local_box);
    if (!local.hit) return {};
    return {true, local.time,
            Axis::transform(axis_x, axis_y, local.normal),
            center + axis_x.scale(local.point.x) + axis_y.scale(local.point.y)};
}

FxSweep moving_aabb_aabb(FxAabb mover, Vec motion, FxAabb target) {
    const FxAabb expanded{target.center,
        {target.half.x + mover.half.x, target.half.y + mover.half.y}};
    const FxSweep hit = ray_aabb(mover.center, motion, expanded);
    if (!hit.hit) return {};
    if (hit.time == 0) {
        const FxManifold initial = collide_aabb_aabb(mover, target);
        if (initial.colliding)
            return {true, 0, -initial.normal, initial.contact};
    }
    const Vec center = mover.center + motion.times_t(hit.time);
    const int64_t extent = round_div(
        std::abs(hit.normal.x) * mover.half.x
        + std::abs(hit.normal.y) * mover.half.y, AxisOne);
    return {true, hit.time, hit.normal, center - hit.normal.scale(extent)};
}

bool is_polygonal(int kind) {
    return kind == ARC_SHAPE_AABB || kind == ARC_SHAPE_OBB
        || kind == ARC_SHAPE_POLYGON;
}

void project_sweep(const Proxy& proxy, Axis axis, int64_t& min, int64_t& max) {
    min = max = axis.dot(proxy.vertices[0]);
    for (size_t i = 1; i < proxy.vertices.size(); ++i) {
        const int64_t projection = axis.dot(proxy.vertices[i]);
        min = std::min(min, projection);
        max = std::max(max, projection);
    }
}

Vec sweep_support(const Proxy& proxy, Axis direction) {
    Vec best = proxy.vertices[0];
    int64_t projection = direction.dot(best);
    for (size_t i = 1; i < proxy.vertices.size(); ++i) {
        const int64_t candidate = direction.dot(proxy.vertices[i]);
        if (candidate > projection) {
            projection = candidate;
            best = proxy.vertices[i];
        }
    }
    return best;
}

bool sweep_axis(
    Axis axis, const Proxy& mover, Vec motion, const Proxy& target,
    int64_t& enter, int64_t& exit, Axis& normal) {
    int64_t min_a, max_a, min_b, max_b;
    project_sweep(mover, axis, min_a, max_a);
    project_sweep(target, axis, min_b, max_b);
    const int64_t velocity = axis.dot(motion);
    if (velocity == 0) return max_a >= min_b && max_b >= min_a;
    int64_t axis_enter = ratio_t(min_b - max_a, velocity);
    int64_t axis_exit = ratio_t(max_b - min_a, velocity);
    if (axis_enter > axis_exit) std::swap(axis_enter, axis_exit);
    if (axis_enter > enter) {
        enter = axis_enter;
        normal = velocity > 0 ? -axis : axis;
    }
    exit = std::min(exit, axis_exit);
    return enter <= exit;
}

bool sweep_axes(
    const Proxy& source, const Proxy& mover, Vec motion, const Proxy& target,
    int64_t& enter, int64_t& exit, Axis& normal) {
    for (int i = 0; i < source.edge_count(); ++i) {
        const auto edge = source.edge(i);
        const Vec delta = edge.second - edge.first;
        const Axis axis = Axis::from_vector(
            {-delta.y, delta.x}, Axis::unit_x());
        if (!sweep_axis(axis, mover, motion, target, enter, exit, normal))
            return false;
    }
    return true;
}

// Swept SAT: for each separating axis, turn the projected gap and the motion's
// projection into an entry/exit time interval; intersect all of them. The largest
// entry time (if it precedes the smallest exit) is the TOI, and the axis that set
// it is the contact normal. Handles any convex mover vs convex target.
FxSweep swept_sat(const Proxy& mover, Vec motion, const Proxy& target) {
    int64_t enter = 0;
    int64_t exit = TOne;
    Axis normal;
    if (!sweep_axes(mover, mover, motion, target, enter, exit, normal)
        || !sweep_axes(target, mover, motion, target, enter, exit, normal)
        || enter < 0 || enter > TOne)
        return {};
    if (normal.is_zero())
        normal = Axis::from_vector(-motion, Axis::unit_x());
    const Vec point_a = sweep_support(mover, -normal) + motion.times_t(enter);
    const Vec point_b = sweep_support(target, normal);
    return {true, enter, normal, midpoint(point_a, point_b)};
}

// Ray vs a radius-r disc (a capsule endpoint / rounded corner). Degenerate hit at
// the exact centre falls back to the reversed motion direction for the normal.
FxSweep rounded_point(Vec origin, Vec motion, Vec center, int64_t radius) {
    FxSweep hit = ray_circle(origin, motion, {center, radius});
    if (!hit.hit || (hit.point - center).length_sq() != 0) return hit;
    hit.normal = Axis::from_vector(-motion, Axis::unit_x());
    return hit;
}

void add_mover_vertex(FxSweep hit, int64_t radius, FxSweep& best) {
    if (!hit.hit || (best.hit && hit.time >= best.time)) return;
    hit.point -= hit.normal.scale(radius);
    best = hit;
}

void sweep_convex(
    const Proxy& mover, Vec motion, const Proxy& target, FxSweep& best) {
    if (mover.radius == 0 && target.radius == 0
        && mover.vertices.size() >= 3 && target.vertices.size() >= 3) {
        add_earlier(swept_sat(mover, motion, target), best);
        return;
    }
    const int64_t radius = mover.radius + target.radius;
    if (mover.vertices.size() == 1 && target.vertices.size() == 1) {
        add_mover_vertex(rounded_point(
            mover.vertices[0], motion, target.vertices[0], radius),
            mover.radius, best);
        return;
    }
    for (Vec vertex : mover.vertices) {
        for (int edge_index = 0; edge_index < target.edge_count(); ++edge_index) {
            const auto edge = target.edge(edge_index);
            add_mover_vertex(ray_capsule(
                vertex, motion, edge.first, edge.second, radius),
                mover.radius, best);
        }
    }
    const Vec reverse_motion = -motion;
    for (Vec vertex : target.vertices) {
        for (int edge_index = 0; edge_index < mover.edge_count(); ++edge_index) {
            const auto edge = mover.edge(edge_index);
            const FxSweep hit = ray_capsule(
                vertex, reverse_motion, edge.first, edge.second, radius);
            if (!hit.hit || (best.hit && hit.time >= best.time)) continue;
            const Axis normal = -hit.normal;
            best = {true, hit.time, normal,
                    vertex + normal.scale(target.radius)};
        }
    }
}

FxSweep reverse_relative(FxSweep hit, Vec original_motion) {
    if (!hit.hit) return hit;
    hit.normal = -hit.normal;
    hit.point += original_motion.times_t(hit.time);
    return hit;
}

// Fast paths: shape pairs with an exact swept closed form (anything with a moving
// circle, or aabb/aabb). Circle-second cases are solved in the target's frame with
// negated motion, then mapped back. Returns false to defer to the SAT sweep.
bool fast_sweep(
    const arc_shape& mover, Vec motion, const arc_shape& target, FxSweep& hit) {
    if (mover.kind == ARC_SHAPE_CIRCLE && target.kind == ARC_SHAPE_CIRCLE) {
        hit = moving_circle_circle(
            fixed_circle(mover.circle), motion, fixed_circle(target.circle));
        return true;
    }
    if (mover.kind == ARC_SHAPE_CIRCLE && target.kind == ARC_SHAPE_AABB) {
        hit = moving_circle_aabb(
            fixed_circle(mover.circle), motion, fixed_aabb(target.aabb));
        return true;
    }
    if (mover.kind == ARC_SHAPE_CIRCLE && target.kind == ARC_SHAPE_CAPSULE) {
        hit = moving_circle_capsule(fixed_circle(mover.circle), motion, target.capsule);
        return true;
    }
    if (mover.kind == ARC_SHAPE_CIRCLE && target.kind == ARC_SHAPE_OBB) {
        hit = moving_circle_obb(fixed_circle(mover.circle), motion, target.obb);
        return true;
    }
    if (mover.kind == ARC_SHAPE_AABB && target.kind == ARC_SHAPE_AABB) {
        hit = moving_aabb_aabb(
            fixed_aabb(mover.aabb), motion, fixed_aabb(target.aabb));
        return true;
    }
    if (mover.kind == ARC_SHAPE_AABB && target.kind == ARC_SHAPE_CIRCLE) {
        hit = reverse_relative(moving_circle_aabb(
            fixed_circle(target.circle), -motion, fixed_aabb(mover.aabb)), motion);
        return true;
    }
    if (mover.kind == ARC_SHAPE_CAPSULE && target.kind == ARC_SHAPE_CIRCLE) {
        hit = reverse_relative(moving_circle_capsule(
            fixed_circle(target.circle), -motion, mover.capsule), motion);
        return true;
    }
    if (mover.kind == ARC_SHAPE_OBB && target.kind == ARC_SHAPE_CIRCLE) {
        hit = reverse_relative(moving_circle_obb(
            fixed_circle(target.circle), -motion, mover.obb), motion);
        return true;
    }
    return false;
}

} // namespace

// Ray vs circle: smallest t in [0,1] solving |origin + t*motion - center|^2 = r^2.
// The quadratic's coefficients are pre-scaled (product_shift) so the discriminant
// stays in int64; an origin already inside reports t=0.
FxSweep ray_circle(Vec origin, Vec motion, FxCircle circle) {
    const Vec relative = origin - circle.center;
    const int64_t a = motion.length_sq();
    if (a == 0) {
        return relative.length_sq() <= circle.radius * circle.radius
            ? FxSweep{true, 0,
                Axis::from_vector(relative, Axis::unit_x()), origin}
            : FxSweep{};
    }
    const int64_t b = relative.dot(motion);
    const int64_t c =
        relative.length_sq() - circle.radius * circle.radius;
    if (c <= 0) {
        return {true, 0, Axis::from_vector(relative,
            Axis::from_vector(-motion, Axis::unit_x())), origin};
    }
    const int shift = product_shift(a, b, c);
    const int64_t scaled_a = scale_product_operand(a, shift);
    const int64_t scaled_b = scale_product_operand(b, shift);
    const int64_t scaled_c = scale_product_operand(c, shift);
    if (scaled_a == 0) return {};
    const int64_t discriminant =
        scaled_b * scaled_b - scaled_a * scaled_c;
    if (discriminant < 0) return {};
    const int64_t time = ratio_t(
        -scaled_b - sqrt_i64(discriminant), scaled_a);
    if (time < 0 || time > TOne) return {};
    const Vec point = origin + motion.times_t(time);
    return {true, time,
            Axis::from_vector(point - circle.center, Axis::unit_x()), point};
}

FxSweep ray_aabb(Vec origin, Vec motion, FxAabb box) {
    const Vec min = box.min(), max = box.max();
    int64_t enter = 0, exit = TOne;
    Axis normal;
    if (!slab(origin.x, motion.x, min.x, max.x,
              -Axis::unit_x(), Axis::unit_x(), enter, exit, normal)
        || !slab(origin.y, motion.y, min.y, max.y,
              -Axis::unit_y(), Axis::unit_y(), enter, exit, normal))
        return {};
    if (normal.is_zero()) {
        int64_t best = origin.x - min.x;
        normal = -Axis::unit_x();
        int64_t distance = max.x - origin.x;
        if (distance < best) { best = distance; normal = Axis::unit_x(); }
        distance = origin.y - min.y;
        if (distance < best) { best = distance; normal = -Axis::unit_y(); }
        distance = max.y - origin.y;
        if (distance < best) normal = Axis::unit_y();
    }
    return {true, enter, normal, origin + motion.times_t(enter)};
}

FxSweep ray_capsule(
    Vec origin, Vec motion, Vec a, Vec b, int64_t radius) {
    const Vec segment = b - a;
    if (segment.length_sq() == 0)
        return ray_circle(origin, motion, {a, radius});
    FxSweep best;
    add_earlier(rounded_point(origin, motion, a, radius), best);
    add_earlier(rounded_point(origin, motion, b, radius), best);
    const Axis normal_axis = Axis::from_vector(
        {-segment.y, segment.x}, Axis::unit_y());
    const int64_t position =
        round_div(normal_axis.dot(origin - a), AxisOne);
    const int64_t velocity =
        round_div(normal_axis.dot(motion), AxisOne);
    int64_t enter;
    int64_t exit;
    if (velocity == 0) {
        if (std::abs(position) > radius) return best;
        enter = 0;
        exit = TOne;
    } else {
        const int64_t first = ratio_t(-radius - position, velocity);
        const int64_t second = ratio_t(radius - position, velocity);
        enter = std::max<int64_t>(0, std::min(first, second));
        exit = std::min<int64_t>(TOne, std::max(first, second));
        if (enter > exit) return best;
    }
    const Vec point = origin + motion.times_t(enter);
    const int64_t projection = (point - a).dot(segment);
    if (projection >= 0 && projection <= segment.length_sq()) {
        const Vec closest = closest_segment(point, a, b);
        const Axis normal = Axis::from_vector(
            point - closest, Axis::from_vector(-motion, normal_axis));
        add_earlier({true, enter, normal, point}, best);
    }
    return best;
}

// Top-level swept dispatch: try the exact fast paths first; if the shapes already
// overlap at t=0 report an immediate hit; otherwise sweep every convex-piece pair
// (concave shapes decompose) and keep the earliest impact.
FxSweep sweep_shapes(
    const arc_shape& mover, Vec motion, const arc_shape& target) {
    FxSweep fast;
    if (fast_sweep(mover, motion, target, fast)) return fast;
    const FxManifold initial = collide_shapes(mover, target);
    if (initial.colliding)
        return {true, 0, -initial.normal, initial.contact};
    FxSweep best;
    for (int i = 0; i < piece_count(mover); ++i)
        for (int j = 0; j < piece_count(target); ++j)
            sweep_convex(make_proxy(mover, i), motion, make_proxy(target, j), best);
    return best;
}

} // namespace arc

namespace {

arc_shape circle_shape(arc_circle value) {
    arc_shape shape{};
    shape.kind = ARC_SHAPE_CIRCLE;
    shape.circle = value;
    return shape;
}
arc_shape aabb_shape(arc_aabb value) {
    arc_shape shape{};
    shape.kind = ARC_SHAPE_AABB;
    shape.aabb = value;
    return shape;
}
arc_shape capsule_shape(arc_capsule value) {
    arc_shape shape{};
    shape.kind = ARC_SHAPE_CAPSULE;
    shape.capsule = value;
    return shape;
}
arc_shape obb_shape(arc_obb value) {
    arc_shape shape{};
    shape.kind = ARC_SHAPE_OBB;
    shape.obb = value;
    return shape;
}

arc_sweep_hit sweep_checked(
    const arc_shape* mover, arc_vec2 motion, const arc_shape* target) {
    if (!mover || !target || !arc::validate_shape(*mover)
        || !arc::validate_shape(*target) || !arc::valid_vec(motion)) {
        arc::set_error("Invalid shape or motion.");
        return {0, 1.0f, {0, 0}, {0, 0}};
    }
    try {
        return arc::sweep_shapes(*mover, arc::Vec::from(motion), *target).to_public();
    } catch (const std::exception& exception) {
        arc::set_error(exception.what());
        return {0, 1.0f, {0, 0}, {0, 0}};
    } catch (...) {
        arc::set_error("Sweep failed.");
        return {0, 1.0f, {0, 0}, {0, 0}};
    }
}

} // namespace

extern "C" {

arc_sweep_hit ARC_CALL arc_ray_vs_circle(
    arc_vec2 origin, arc_vec2 motion, arc_circle circle) {
    try {
        return arc::ray_circle(
            arc::Vec::from(origin), arc::Vec::from(motion),
            arc::fixed_circle(circle)).to_public();
    } catch (...) {
        arc::set_error("Invalid ray or circle.");
        return {0, 1.0f, {0, 0}, {0, 0}};
    }
}

arc_sweep_hit ARC_CALL arc_ray_vs_aabb(
    arc_vec2 origin, arc_vec2 motion, arc_aabb box) {
    try {
        return arc::ray_aabb(
            arc::Vec::from(origin), arc::Vec::from(motion),
            arc::fixed_aabb(box)).to_public();
    } catch (...) {
        arc::set_error("Invalid ray or box.");
        return {0, 1.0f, {0, 0}, {0, 0}};
    }
}

arc_sweep_hit ARC_CALL arc_moving_shape_vs_shape(
    const arc_shape* mover, arc_vec2 motion, const arc_shape* target) {
    return sweep_checked(mover, motion, target);
}

arc_sweep_hit ARC_CALL arc_moving_circle_vs_circle(
    arc_circle mover, arc_vec2 motion, arc_circle target) {
    const arc_shape a = circle_shape(mover), b = circle_shape(target);
    return sweep_checked(&a, motion, &b);
}
arc_sweep_hit ARC_CALL arc_moving_circle_vs_aabb(
    arc_circle mover, arc_vec2 motion, arc_aabb target) {
    const arc_shape a = circle_shape(mover), b = aabb_shape(target);
    return sweep_checked(&a, motion, &b);
}
arc_sweep_hit ARC_CALL arc_moving_circle_vs_capsule(
    arc_circle mover, arc_vec2 motion, arc_capsule target) {
    const arc_shape a = circle_shape(mover), b = capsule_shape(target);
    return sweep_checked(&a, motion, &b);
}
arc_sweep_hit ARC_CALL arc_moving_circle_vs_obb(
    arc_circle mover, arc_vec2 motion, arc_obb target) {
    const arc_shape a = circle_shape(mover), b = obb_shape(target);
    return sweep_checked(&a, motion, &b);
}
arc_sweep_hit ARC_CALL arc_moving_aabb_vs_aabb(
    arc_aabb mover, arc_vec2 motion, arc_aabb target) {
    const arc_shape a = aabb_shape(mover), b = aabb_shape(target);
    return sweep_checked(&a, motion, &b);
}

int32_t ARC_CALL arc_get_sweep_algorithm(
    const arc_shape* mover, const arc_shape* target) {
    if (!mover || !target) return ARC_SWEEP_FEATURE_CAST;
    if (mover->kind == ARC_SHAPE_CIRCLE && target->kind == ARC_SHAPE_CIRCLE)
        return ARC_SWEEP_ANALYTIC_CIRCLE;
    if ((mover->kind == ARC_SHAPE_CIRCLE && target->kind == ARC_SHAPE_AABB)
        || (mover->kind == ARC_SHAPE_AABB && target->kind == ARC_SHAPE_CIRCLE))
        return ARC_SWEEP_ROUNDED_AABB;
    if ((mover->kind == ARC_SHAPE_CIRCLE && target->kind == ARC_SHAPE_CAPSULE)
        || (mover->kind == ARC_SHAPE_CAPSULE && target->kind == ARC_SHAPE_CIRCLE))
        return ARC_SWEEP_ROUNDED_SEGMENT;
    if ((mover->kind == ARC_SHAPE_CIRCLE && target->kind == ARC_SHAPE_OBB)
        || (mover->kind == ARC_SHAPE_OBB && target->kind == ARC_SHAPE_CIRCLE))
        return ARC_SWEEP_LOCAL_SPACE_ROUNDED_AABB;
    if (mover->kind == ARC_SHAPE_AABB && target->kind == ARC_SHAPE_AABB)
        return ARC_SWEEP_AABB;
    return arc::is_polygonal(mover->kind) && arc::is_polygonal(target->kind)
        ? ARC_SWEEP_CONTINUOUS_SAT : ARC_SWEEP_FEATURE_CAST;
}

} // extern "C"
