// Distance primitives: closest point on a segment and closest points between two
// segments. These underlie capsule collisions and are exposed through the public
// arc_closest_* C API. All integer; the segment-segment solve scales its operands
// down (product_shift) so the 2x2 system stays inside int64 without 128-bit math.
#include "internal.h"

#include <cmath>

namespace arc {

// Closest point on segment [a,b] to `point`. out_time receives the 16.16
// parameter along the segment (0 at a, TOne at b), clamped to the endpoints.
Vec closest_segment(Vec point, Vec a, Vec b, int64_t* out_time) {
    const Vec ab = b - a;
    const int64_t length_sq = ab.length_sq();
    if (length_sq == 0) {
        if (out_time) *out_time = 0;
        return a;
    }
    const int64_t time = clamped_param((point - a).dot(ab), length_sq);
    if (out_time) *out_time = time;
    return a + ab.times_t(time);
}

// Closest points c1 on [p1,q1] and c2 on [p2,q2], returning the squared distance
// between them (scale 2^16). Follows the standard Ericson clamped-parameter
// solution: solve for s on segment 1, project onto segment 2, re-clamp. Degenerate
// (zero-length) segments fall back to the point/segment cases.
int64_t closest_segments(
    Vec p1, Vec q1, Vec p2, Vec q2, Vec& c1, Vec& c2) {
    const Vec d1 = q1 - p1;
    const Vec d2 = q2 - p2;
    const Vec r = p1 - p2;
    const int64_t a = d1.length_sq();      // |d1|^2
    const int64_t e = d2.length_sq();      // |d2|^2
    const int64_t f = d2.dot(r);
    int64_t s = 0;
    int64_t t = 0;
    if (a == 0 && e == 0) {
        s = t = 0;                         // both segments are points
    } else if (a == 0) {
        t = clamped_param(f, e);           // segment 1 is a point
    } else {
        const int64_t c = d1.dot(r);
        if (e == 0) {
            s = clamped_param(-c, a);      // segment 2 is a point
        } else {
            const int64_t b = d1.dot(d2);
            // Pre-scale the 2x2 coefficients so a*e - b*b fits in int64.
            const int shift = product_shift(a, b, c, e, f);
            const int64_t scaled_a = scale_product_operand(a, shift);
            const int64_t scaled_b = scale_product_operand(b, shift);
            const int64_t scaled_c = scale_product_operand(c, shift);
            const int64_t scaled_e = scale_product_operand(e, shift);
            const int64_t scaled_f = scale_product_operand(f, shift);
            const int64_t denominator =
                scaled_a * scaled_e - scaled_b * scaled_b;
            if (denominator > 0) {
                const int64_t numerator =
                    scaled_b * scaled_f - scaled_c * scaled_e;
                s = clamped_param(numerator, denominator);
            }
            const int64_t projected = mul_t(b, s) + f;
            if (projected < 0) {
                t = 0;
                s = clamped_param(-c, a);
            } else if (projected > e) {
                t = TOne;
                s = clamped_param(b - c, a);
            } else {
                t = clamped_param(projected, e);
            }
        }
    }
    c1 = p1 + d1.times_t(s);
    c2 = p2 + d2.times_t(t);
    return c1.dist_sq(c2);
}

} // namespace arc

extern "C" {

arc_vec2 ARC_CALL arc_closest_point_on_segment(
    arc_vec2 p, arc_vec2 a, arc_vec2 b, float* out_t) {
    try {
        int64_t time = 0;
        const arc::Vec result = arc::closest_segment(
            arc::Vec::from(p), arc::Vec::from(a), arc::Vec::from(b), &time);
        if (out_t) *out_t = arc::to_t(time);
        return result.to_public();
    } catch (const std::exception& exception) {
        arc::set_error(exception.what());
        if (out_t) *out_t = 0;
        return {};
    }
}

arc_vec2 ARC_CALL arc_closest_point_on_aabb(arc_vec2 p, arc_aabb box) {
    try {
        const arc::Vec point = arc::Vec::from(p);
        const arc::FxAabb fixed = arc::fixed_aabb(box);
        const arc::Vec min = fixed.min();
        const arc::Vec max = fixed.max();
        return arc::Vec{
            std::clamp(point.x, min.x, max.x),
            std::clamp(point.y, min.y, max.y)}.to_public();
    } catch (const std::exception& exception) {
        arc::set_error(exception.what());
        return {};
    }
}

float ARC_CALL arc_closest_points_segment_segment(
    arc_vec2 p1, arc_vec2 q1, arc_vec2 p2, arc_vec2 q2,
    arc_vec2* out_c1, arc_vec2* out_c2) {
    try {
        arc::Vec c1;
        arc::Vec c2;
        const int64_t result = arc::closest_segments(
            arc::Vec::from(p1), arc::Vec::from(q1),
            arc::Vec::from(p2), arc::Vec::from(q2), c1, c2);
        if (out_c1) *out_c1 = c1.to_public();
        if (out_c2) *out_c2 = c2.to_public();
        return arc::to_sq(result);
    } catch (const std::exception& exception) {
        arc::set_error(exception.what());
        if (out_c1) *out_c1 = {};
        if (out_c2) *out_c2 = {};
        return 0;
    }
}

} // extern "C"
