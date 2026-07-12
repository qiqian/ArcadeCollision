#ifndef ARCCOLLISION_INTERNAL_H
#define ARCCOLLISION_INTERNAL_H

#include "arccollision.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace arc {

constexpr int FxShift = 8;
constexpr int64_t FxOne = int64_t{1} << FxShift;
constexpr int TShift = 16;
constexpr int64_t TOne = int64_t{1} << TShift;
constexpr int AxisShift = 30;
constexpr int64_t AxisOne = int64_t{1} << AxisShift;

extern thread_local std::string error_text;
void set_error(const char* text);
bool valid_scalar(float value);
bool valid_vec(arc_vec2 value);
int64_t from_float(float value);
float to_float(int64_t value);
float to_t(int64_t value);
float to_sq(int64_t value);
uint64_t magnitude(int64_t value);
int64_t round_div(int64_t numerator, int64_t denominator);
int64_t floor_div(int64_t numerator, int64_t denominator);
int64_t ceil_div_positive(int64_t numerator, int64_t denominator);
int64_t mul_t(int64_t value, int64_t time);
int64_t clamped_param(int64_t numerator, int64_t denominator);
int64_t ratio_t(int64_t numerator, int64_t denominator);
int product_shift(int64_t a, int64_t b, int64_t c);
int product_shift(int64_t a, int64_t b, int64_t c, int64_t d, int64_t e);
int64_t scale_product_operand(int64_t value, int shift);
int64_t sqrt_i64(int64_t value);

struct Vec {
    int64_t x = 0;
    int64_t y = 0;

    Vec() = default;
    Vec(int64_t px, int64_t py) : x(px), y(py) {}
    static Vec from(arc_vec2 value) { return {from_float(value.x), from_float(value.y)}; }
    arc_vec2 to_public() const { return {to_float(x), to_float(y)}; }
    Vec operator+(Vec other) const { return {x + other.x, y + other.y}; }
    Vec operator-(Vec other) const { return {x - other.x, y - other.y}; }
    Vec operator-() const { return {-x, -y}; }
    Vec& operator+=(Vec other) { x += other.x; y += other.y; return *this; }
    Vec& operator-=(Vec other) { x -= other.x; y -= other.y; return *this; }
    int64_t dot(Vec other) const { return x * other.x + y * other.y; }
    int64_t length_sq() const { return x * x + y * y; }
    int64_t dist_sq(Vec other) const { return (*this - other).length_sq(); }
    int64_t length() const { return sqrt_i64(length_sq()); }
    Vec times_t(int64_t time) const { return {mul_t(x, time), mul_t(y, time)}; }
};

struct Axis {
    int64_t x = 0;
    int64_t y = 0;

    Axis() = default;
    Axis(int64_t px, int64_t py) : x(px), y(py) {}
    static Axis unit_x() { return {AxisOne, 0}; }
    static Axis unit_y() { return {0, AxisOne}; }
    static Axis from_vector(Vec value, Axis fallback);
    static Axis from_components(int64_t x, int64_t y, Axis fallback);
    static Axis from_angle(uint32_t angle);
    static Axis transform(Axis basis_x, Axis basis_y, Axis local);
    bool is_zero() const { return x == 0 && y == 0; }
    Axis perpendicular() const { return {-y, x}; }
    Axis operator-() const { return {-x, -y}; }
    int64_t dot(Vec position) const { return position.x * x + position.y * y; }
    int64_t dot(Axis other) const;
    Vec scale(int64_t distance) const;
    arc_vec2 to_public() const {
        return {x / static_cast<float>(AxisOne), y / static_cast<float>(AxisOne)};
    }
};

struct FxCircle {
    Vec center;
    int64_t radius = 0;
};

struct FxAabb {
    Vec center;
    Vec half;
    Vec min() const { return center - half; }
    Vec max() const { return center + half; }
};

struct Bounds {
    int64_t min_x = 0;
    int64_t min_y = 0;
    int64_t max_x = 0;
    int64_t max_y = 0;

    int64_t center_x() const { return min_x + ((max_x - min_x) >> 1); }
    int64_t center_y() const { return min_y + ((max_y - min_y) >> 1); }
    int64_t perimeter() const { return 2 * ((max_x - min_x) + (max_y - min_y)); }
    bool overlaps(const Bounds& other) const {
        return min_x <= other.max_x && other.min_x <= max_x
            && min_y <= other.max_y && other.min_y <= max_y;
    }
    bool contains(const Bounds& other) const {
        return min_x <= other.min_x && min_y <= other.min_y
            && max_x >= other.max_x && max_y >= other.max_y;
    }
    Bounds expanded(int64_t margin) const {
        return {min_x - margin, min_y - margin, max_x + margin, max_y + margin};
    }
    Bounds translated(int64_t x, int64_t y) const {
        return {min_x + x, min_y + y, max_x + x, max_y + y};
    }
    static Bounds unite(const Bounds& a, const Bounds& b) {
        return {std::min(a.min_x, b.min_x), std::min(a.min_y, b.min_y),
                std::max(a.max_x, b.max_x), std::max(a.max_y, b.max_y)};
    }
    FxAabb to_fx_aabb() const {
        return {{min_x + ((max_x - min_x) / 2), min_y + ((max_y - min_y) / 2)},
                {(max_x - min_x) / 2, (max_y - min_y) / 2}};
    }
    arc_aabb to_public() const;
};

struct FxManifold {
    bool colliding = false;
    Axis normal;
    int64_t depth = 0;
    Vec contact;
    uint8_t negative_zero_mask = 0;
    arc_manifold to_public() const;
};

struct FxSweep {
    bool hit = false;
    int64_t time = TOne;
    Axis normal;
    Vec point;
    arc_sweep_hit to_public() const;
    static FxSweep miss() { return {}; }
};

struct Proxy {
    std::vector<Vec> vertices;
    int64_t radius = 0;
    Vec center;

    int edge_count() const {
        return vertices.size() <= 1 ? 0 : vertices.size() == 2 ? 1
            : static_cast<int>(vertices.size());
    }
    std::pair<Vec, Vec> edge(int index) const {
        return {vertices[static_cast<size_t>(index)],
                vertices[vertices.size() == 2 ? 1
                    : (static_cast<size_t>(index) + 1) % vertices.size()]};
    }
    Proxy translated(Vec offset) const;
};

Vec closest_segment(Vec point, Vec a, Vec b, int64_t* out_time = nullptr);
int64_t closest_segments(Vec p1, Vec q1, Vec p2, Vec q2, Vec& c1, Vec& c2);
Vec midpoint(Vec a, Vec b);
FxAabb proxy_bounds(const Proxy& proxy);
Bounds shape_bounds(const arc_shape& shape);
bool validate_shape(const arc_shape& shape);
arc_shape moved_shape(arc_shape shape, arc_vec2 delta);
void retain_shape(const arc_shape& shape);
void release_shape(const arc_shape& shape);
int piece_count(const arc_shape& shape);
Proxy make_proxy(const arc_shape& shape, int piece = 0);
FxManifold collide_proxy(const Proxy& a, const Proxy& b);
FxManifold collide_shapes(const arc_shape& a, const arc_shape& b);
bool overlap_shapes(const arc_shape& a, const arc_shape& b);
FxSweep ray_circle(Vec origin, Vec motion, FxCircle circle);
FxSweep ray_aabb(Vec origin, Vec motion, FxAabb box);
FxSweep ray_capsule(Vec origin, Vec motion, Vec a, Vec b, int64_t radius);
FxSweep sweep_shapes(const arc_shape& mover, Vec motion, const arc_shape& target);

inline FxCircle fixed_circle(arc_circle value) {
    return {Vec::from(value.center), from_float(value.radius)};
}
inline FxAabb fixed_aabb(arc_aabb value) {
    return {Vec::from(value.center), Vec::from(value.half_extents)};
}
inline bool filter_allows(arc_collision_filter a, arc_collision_filter b) {
    return (a.categories & b.collides_with) != 0
        && (b.categories & a.collides_with) != 0;
}

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

struct arc_polygon {
    std::atomic<uint32_t> refs{1};
    std::vector<arc_vec2> vertices;
    std::vector<arc::Vec> fixed_vertices;
    std::vector<int> triangles;
    arc::Bounds bounds;
    arc_aabb public_bounds{};
    bool convex = false;
};

#endif
