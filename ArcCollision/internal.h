#ifndef ARCCOLLISION_INTERNAL_H
#define ARCCOLLISION_INTERNAL_H

#include "arccollision.h"
#include "simd128.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

// Internal implementation of the ArcCollision native library. This mirrors the
// C# reference (ArcCollision.Ref) operation-for-operation so the two backends
// agree; see that project for the long-form rationale behind each algorithm.
//
// Fixed-point conventions (all math is integer, no floats past the boundary):
//   * position / size / radius : 24.8   (world unit * 2^8  = FxOne),   int64
//   * time parameters          : 16.16  (t in [0,1] * 2^16 = TOne),    int64
//   * unit axes / normals      : Q1.30  (direction  * 2^30 = AxisOne), int64
//   * squared lengths          : scale 2^16 (a 24.8 value squared)
// Floats are quantized to 24.8 on the way in (from_float) and converted back on
// the way out (to_float / to_t). Keeping directions at Q1.30 rather than 24.8 is
// what keeps SAT depth/normal error at the position-grid level instead of the
// ~0.4%-of-extent a coarse 24.8 axis would cause.
namespace arc {

constexpr int FxShift = 8;
constexpr int64_t FxOne = int64_t{1} << FxShift;    // 256: one world unit
constexpr int TShift = 16;
constexpr int64_t TOne = int64_t{1} << TShift;      // 65536: parameter 1.0
constexpr int AxisShift = 30;
constexpr int64_t AxisOne = int64_t{1} << AxisShift; // unit direction magnitude

// Hot fixed-denominator conversions between Q1.30 projections and Q24.8
// distances. Keeping these inline lets x64/ARM64 emit add+shift directly at SAT
// call sites instead of paying for a generic integer division and function call.
inline int64_t round_axis(int64_t value) {
    constexpr int64_t half = AxisOne >> 1;
    return value >= 0
        ? (value + half) >> AxisShift
        : -((-value + half) >> AxisShift);
}

inline int64_t ceil_axis_positive(int64_t value) {
    return value == 0 ? 0 : 1 + ((value - 1) >> AxisShift);
}

// Per-thread last-error string, surfaced through arc_get_last_error.
extern thread_local std::string error_text;
void set_error(const char* text);

// Boundary validation + float<->fixed conversion (see fixed.cpp).
bool valid_scalar(float value);           // finite and within the safe range
bool valid_vec(const arc_vec2& value);
int64_t from_float(float value);          // world float -> 24.8, round-half-even
float to_float(int64_t value);            // 24.8 -> world float
float to_t(int64_t value);                // 16.16 -> [0,1] float
float to_sq(int64_t value);               // squared (2^16) -> float
uint64_t magnitude(int64_t value);        // |value| as unsigned, overflow-safe

// Integer division helpers with explicit rounding modes.
int64_t round_div(int64_t numerator, int64_t denominator);        // round-half, symmetric
int64_t floor_div(int64_t numerator, int64_t denominator);        // toward -inf
int64_t ceil_div_positive(int64_t numerator, int64_t denominator); // toward +inf, positives
int64_t mul_t(int64_t value, int64_t time);   // apply a 16.16 parameter: (value*t)>>16
int64_t clamped_param(int64_t numerator, int64_t denominator);    // num/den clamped to [0,1] 16.16
int64_t ratio_t(int64_t numerator, int64_t denominator);          // num/den as 16.16

// Adaptive scaling for degree-four discriminants: product_shift picks a right
// shift so the squared products stay inside int64, and scale_product_operand
// applies it. Lets sweeps solve quadratics without 128-bit arithmetic.
int product_shift(int64_t a, int64_t b, int64_t c);
int product_shift(int64_t a, int64_t b, int64_t c, int64_t d, int64_t e);
int64_t scale_product_operand(int64_t value, int shift);
int64_t sqrt_i64(int64_t value);          // exact floor integer square root (integer Newton)

// A 2D point/vector in 24.8 fixed point. dot()/length_sq() return values at the
// squared 2^16 scale; length() takes the integer sqrt back to 24.8.
struct Vec {
    int64_t x = 0;
    int64_t y = 0;

    Vec() = default;
    Vec(int64_t px, int64_t py) : x(px), y(py) {}
    static Vec from(const arc_vec2& value) { return {from_float(value.x), from_float(value.y)}; }
    arc_vec2 to_public() const { return {to_float(x), to_float(y)}; }
    Vec operator+(const Vec& other) const { return {x + other.x, y + other.y}; }
    Vec operator-(const Vec& other) const { return {x - other.x, y - other.y}; }
    Vec operator-() const { return {-x, -y}; }
    Vec& operator+=(const Vec& other) { x += other.x; y += other.y; return *this; }
    Vec& operator-=(const Vec& other) { x -= other.x; y -= other.y; return *this; }
    int64_t dot(const Vec& other) const { return x * other.x + y * other.y; }
    int64_t length_sq() const { return x * x + y * y; }
    int64_t dist_sq(const Vec& other) const { return (*this - other).length_sq(); }
    int64_t length() const { return sqrt_i64(length_sq()); }
    Vec times_t(int64_t time) const { return {mul_t(x, time), mul_t(y, time)}; }
};

// A unit direction in Q1.30 (components ~= dir * 2^30). Separate from Vec so the
// higher-precision axis math (SAT normals, contact frames) never gets truncated
// to the coarse 24.8 grid. dot(Vec) mixes a Q1.30 axis with a 24.8 position and
// yields a scale-2^38 projection; scale() multiplies a distance by the axis.
struct Axis {
    int64_t x = 0;
    int64_t y = 0;

    Axis() = default;
    Axis(int64_t px, int64_t py) : x(px), y(py) {}
    static Axis unit_x() { return {AxisOne, 0}; }
    static Axis unit_y() { return {0, AxisOne}; }
    static Axis from_vector(const Vec& value, const Axis& fallback);
    static Axis from_components(int64_t x, int64_t y, const Axis& fallback);
    static Axis from_angle(uint32_t angle);
    static Axis transform(const Axis& basis_x, const Axis& basis_y, const Axis& local);
    bool is_zero() const { return x == 0 && y == 0; }
    Axis perpendicular() const { return {-y, x}; }
    Axis operator-() const { return {-x, -y}; }
    int64_t dot(const Vec& position) const { return position.x * x + position.y * y; }
    int64_t dot(const Axis& other) const;
    Vec scale(int64_t distance) const;
    arc_vec2 to_public() const {
        return {x / static_cast<float>(AxisOne), y / static_cast<float>(AxisOne)};
    }
};

struct FxCircle {
    Vec center;
    int64_t radius = 0;
};

// Axis-aligned box stored as centre + half-extents (24.8).
struct FxAabb {
    Vec center;
    Vec half;
    Vec min() const { return center - half; }
    Vec max() const { return center + half; }
};

// Broadphase bounds as int32 fixed-point (24.8) min/max. int32 halves the node
// footprint vs int64, so more nodes fit per cache line -- the real broadphase win,
// since traversal is memory-bound. This requires |position| + |extent| to stay in
// int32 (< 2^31 fixed ~= 8.39M world units), which the +/-ARC_MAX_COORDINATE input
// limit guarantees with margin. Only min/max are stored as int32; derived spans
// (max - min, up to ~2^31) widen to int64 for perimeter/center/SAH so they never
// overflow. overlaps/contains stay scalar on purpose: they short-circuit (most
// broadphase pairs are disjoint), which beats a branchless SIMD reduction here.
// Mirrors C# BpBounds.
struct Bounds {
    int32_t min_x = 0;
    int32_t min_y = 0;
    int32_t max_x = 0;
    int32_t max_y = 0;

    int32_t center_x() const {
        return static_cast<int32_t>(min_x + ((static_cast<int64_t>(max_x) - min_x) >> 1));
    }
    int32_t center_y() const {
        return static_cast<int32_t>(min_y + ((static_cast<int64_t>(max_y) - min_y) >> 1));
    }
    int64_t perimeter() const {
        return 2 * ((static_cast<int64_t>(max_x) - min_x)
                  + (static_cast<int64_t>(max_y) - min_y));
    }
    bool overlaps(const Bounds& other) const {
        return min_x <= other.max_x && other.min_x <= max_x
            && min_y <= other.max_y && other.min_y <= max_y;
    }
    bool contains(const Bounds& other) const {
        return min_x <= other.min_x && min_y <= other.min_y
            && max_x >= other.max_x && max_y >= other.max_y;
    }
    // Margin/offset are 24.8 fixed (int64) that stay within int32 for in-range
    // colliders; narrow the result. Callers pass int64 without casting.
    Bounds expanded(int64_t margin) const {
        return {static_cast<int32_t>(min_x - margin), static_cast<int32_t>(min_y - margin),
                static_cast<int32_t>(max_x + margin), static_cast<int32_t>(max_y + margin)};
    }
    Bounds translated(int64_t x, int64_t y) const {
        return {static_cast<int32_t>(min_x + x), static_cast<int32_t>(min_y + y),
                static_cast<int32_t>(max_x + x), static_cast<int32_t>(max_y + y)};
    }
    static Bounds unite(const Bounds& a, const Bounds& b) {
        return {std::min(a.min_x, b.min_x), std::min(a.min_y, b.min_y),
                std::max(a.max_x, b.max_x), std::max(a.max_y, b.max_y)};
    }
    FxAabb to_fx_aabb() const {
        const int64_t half_w = (static_cast<int64_t>(max_x) - min_x) / 2;
        const int64_t half_h = (static_cast<int64_t>(max_y) - min_y) / 2;
        return {{min_x + half_w, min_y + half_h}, {half_w, half_h}};
    }
    arc_aabb to_public() const;
};

// SIMD loads rely only on the private integer layouts, never on public C ABI
// structs. Fail at compile time if a compiler ever inserts unexpected padding.
static_assert(sizeof(Vec) == sizeof(int64_t) * 2, "Vec must be two packed int64 lanes");
static_assert(sizeof(Bounds) == sizeof(int32_t) * 4, "Bounds must be four packed int32 lanes");

// Result of a discrete overlap test. normal points from the first shape toward
// the second; depth is the penetration along it (0 for an exact touch). The
// negative_zero_mask preserves signed-zero components so the public float
// manifold round-trips bit-exactly under mirroring.
struct FxManifold {
    bool colliding = false;
    Axis normal;
    int64_t depth = 0;
    Vec contact;
    uint8_t negative_zero_mask = 0;
    arc_manifold to_public() const;
};

// Result of a swept test. time is the 16.16 fraction of the motion at first
// contact; a default (hit=false, time=TOne) is a miss.
struct FxSweep {
    bool hit = false;
    int64_t time = TOne;
    Axis normal;
    Vec point;
    uint8_t negative_zero_mask = 0;
    // Reverse-relative fast paths map the fixed contact back to the mover's
    // frame at the public float boundary, matching Vec2 multiplication/addition
    // order in the managed implementation without affecting collision math.
    bool translate_public_point = false;

    // Converts the internal deterministic sweep result to the float-based C API
    // representation. Time, normal and contact point remain integers throughout
    // the collision calculation and are converted only at this public boundary.
    //
    // The motion overload is required by reverse-relative fast paths (AABB,
    // capsule or OBB moving against a circle). Their contact point is initially
    // calculated in the circle's relative frame, then mapped back with the
    // caller's original float motion so the operation order and result bits match
    // ArcCollision.Ref. Direct ray/primitive sweeps use the parameterless form.
    arc_sweep_hit to_public() const;
    arc_sweep_hit to_public(const arc_vec2& public_motion) const;
    static FxSweep miss() { return {}; }
};

// Allocation-free convex proxy matching the managed ConvexProxy/SweepProxy.
// Primitive hulls use four inline vertices. Polygon proxies borrow immutable
// polygon storage and retain only an optional index window plus transform data.
// The owning arc_shape/polygon is kept alive for every proxy call site.
struct Proxy {
    std::array<Vec, 4> inline_vertices{};
    const Vec* borrowed_vertices = nullptr;
    const int* borrowed_indices = nullptr;
    int borrowed_index_offset = 0;
    int count = 0;
    Vec offset;
    Axis axis_x;
    Axis axis_y;
    bool transformed = false;
    int64_t radius = 0;
    Vec center;

    Vec vertex(int index) const {
        Vec value;
        if (borrowed_vertices) {
            const int source_index = borrowed_indices
                ? borrowed_indices[borrowed_index_offset + index] : index;
            value = borrowed_vertices[source_index];
            if (transformed)
                value = axis_x.scale(value.x) + axis_y.scale(value.y);
        } else {
            value = inline_vertices[static_cast<size_t>(index)];
        }
        return value + offset;
    }
    int edge_count() const {
        return count <= 1 ? 0 : count == 2 ? 1 : count;
    }
    std::pair<Vec, Vec> edge(int index) const {
        return {vertex(index), vertex(count == 2 ? 1 : (index + 1) % count)};
    }
    Proxy translated(const Vec& offset) const;
};

// Distance / geometry primitives (distance.cpp).
Vec closest_segment(const Vec& point, const Vec& a, const Vec& b, int64_t* out_time = nullptr);
int64_t closest_segments(const Vec& p1, const Vec& q1, const Vec& p2, const Vec& q2, Vec& c1, Vec& c2);
Vec midpoint(const Vec& a, const Vec& b);

// Shape handling (shapes.cpp): bounds, validation, translation, polygon
// refcounting, and reduction to SAT proxies (a concave polygon has >1 piece).
FxAabb proxy_bounds(const Proxy& proxy);
Bounds shape_bounds(const arc_shape& shape);
bool validate_shape(const arc_shape& shape);
arc_shape moved_shape(arc_shape shape, const arc_vec2& delta);
void retain_shape(const arc_shape& shape);
void release_shape(const arc_shape& shape);
arc_polygon* polygon_from_fixed(const std::vector<Vec>& vertices);
int piece_count(const arc_shape& shape);
Proxy make_proxy(const arc_shape& shape, int piece = 0);

// Discrete + swept narrowphase (collide.cpp / sweep.cpp).
FxManifold collide_proxy(const Proxy& a, const Proxy& b);
FxManifold collide_shapes(const arc_shape& a, const arc_shape& b);
// Fixed-point primitive collisions (no float round-trip) for the sweep t=0 paths.
FxManifold collide_circle_circle(const FxCircle& a, const FxCircle& b);
FxManifold collide_circle_aabb(const FxCircle& circle, const FxAabb& box);
FxManifold collide_aabb_aabb(const FxAabb& a, const FxAabb& b);
bool overlap_shapes(const arc_shape& a, const arc_shape& b);
FxSweep ray_circle(const Vec& origin, const Vec& motion, const FxCircle& circle);
FxSweep ray_aabb(const Vec& origin, const Vec& motion, const FxAabb& box);
FxSweep ray_capsule(const Vec& origin, const Vec& motion, const Vec& a, const Vec& b, int64_t radius);
FxSweep sweep_shapes(const arc_shape& mover, const Vec& motion, const arc_shape& target);

inline FxCircle fixed_circle(const arc_circle& value) {
    return {Vec::from(value.center), from_float(value.radius)};
}
inline FxAabb fixed_aabb(const arc_aabb& value) {
    return {Vec::from(value.center), Vec::from(value.half_extents)};
}
// Two colliders interact only when each filter accepts the other's category.
inline bool filter_allows(const arc_collision_filter& a, const arc_collision_filter& b) {
    return (a.categories & b.collides_with) != 0
        && (b.categories & a.collides_with) != 0;
}

// Shared two-call array-return helper for the C API: pass output=null,capacity=0
// to learn *required, then call again with a buffer. An empty result is OK, not
// BUFFER_TOO_SMALL, so callers can treat that as a plain success.
template<class T>
arc_status copy_results(const std::vector<T>& values, T* output,
                        int32_t capacity, int32_t* required) {
    if (!required || capacity < 0) {
        set_error("Invalid output buffer.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    if (values.size() > static_cast<size_t>(std::numeric_limits<int32_t>::max())) {
        set_error("Result count exceeds the C ABI limit.");
        return ARC_STATUS_INTERNAL_ERROR;
    }
    *required = static_cast<int32_t>(values.size());
    if (capacity < *required || (!output && *required != 0))
        return ARC_STATUS_BUFFER_TOO_SMALL;
    std::copy(values.begin(), values.end(), output);
    return ARC_STATUS_OK;
}

} // namespace arc

// Reference-counted polygon shared across arc_shape handles. Caches both the
// public float vertices and their 24.8 fixed form, a fan/ear triangulation
// (concave polygons collide per-triangle), fixed centroid, and precomputed
// bounds. Created by arc_polygon_create and released via retain/release.
struct arc_polygon {
    std::atomic<uint32_t> refs{1};
    std::vector<arc_vec2> vertices;
    std::vector<arc::Vec> fixed_vertices;
    std::vector<int> triangles;
    arc::Vec centroid;
    arc::Bounds bounds;
    arc_aabb public_bounds{};
    bool convex = false;
};

#endif
