// Shape handling: polygon construction (validation, winding, ear-clip
// triangulation, convexity test), reduction of any shape to SAT proxies (a
// concave polygon yields one proxy per triangle), bounds computation, and the
// polygon retain/release refcounting behind the C API. Mirrors the managed shape
// types and Polygon.cs.
#include "internal.h"

#include <cmath>
#include <new>
#include <stdexcept>

namespace arc {
namespace {

// Signed area x2 of triangle (a,b,c): >0 CCW, <0 CW, 0 collinear.
int64_t cross(Vec a, Vec b, Vec c) {
    const Vec ab = b - a;
    const Vec ac = c - a;
    return ab.x * ac.y - ab.y * ac.x;
}

bool on_segment(Vec a, Vec b, Vec point) {
    return point.x >= std::min(a.x, b.x) && point.x <= std::max(a.x, b.x)
        && point.y >= std::min(a.y, b.y) && point.y <= std::max(a.y, b.y);
}

bool segments_intersect(Vec a, Vec b, Vec c, Vec d) {
    const int64_t ab_c = cross(a, b, c);
    const int64_t ab_d = cross(a, b, d);
    const int64_t cd_a = cross(c, d, a);
    const int64_t cd_b = cross(c, d, b);
    if (ab_c == 0 && on_segment(a, b, c)) return true;
    if (ab_d == 0 && on_segment(a, b, d)) return true;
    if (cd_a == 0 && on_segment(c, d, a)) return true;
    if (cd_b == 0 && on_segment(c, d, b)) return true;
    return (ab_c < 0) != (ab_d < 0) && (cd_a < 0) != (cd_b < 0);
}

bool point_in_triangle(Vec point, Vec a, Vec b, Vec c, int winding) {
    const int64_t ab = cross(a, b, point);
    const int64_t bc = cross(b, c, point);
    const int64_t ca = cross(c, a, point);
    return winding > 0
        ? ab >= 0 && bc >= 0 && ca >= 0
        : ab <= 0 && bc <= 0 && ca <= 0;
}

// Validate and precompute a polygon: quantize vertices to 24.8, normalize winding,
// reject self-intersecting/degenerate input, ear-clip into a triangle fan, flag
// convexity (convex polygons skip per-triangle collision), and cache bounds.
bool build_polygon(arc_polygon& polygon, const arc_vec2* vertices, int32_t count) {
    if (!vertices || count < 3) {
        set_error("A polygon requires at least three vertices.");
        return false;
    }
    polygon.vertices.assign(vertices, vertices + count);
    float min_x = vertices[0].x, min_y = vertices[0].y;
    float max_x = vertices[0].x, max_y = vertices[0].y;
    Vec centroid_sum;
    polygon.fixed_vertices.reserve(static_cast<size_t>(count));
    for (int32_t i = 0; i < count; ++i) {
        if (!valid_vec(vertices[i])) {
            set_error("Polygon vertex is outside the fixed-point range.");
            return false;
        }
        min_x = std::min(min_x, vertices[i].x);
        min_y = std::min(min_y, vertices[i].y);
        max_x = std::max(max_x, vertices[i].x);
        max_y = std::max(max_y, vertices[i].y);
        const Vec fixed_vertex = Vec::from(vertices[i]);
        polygon.fixed_vertices.push_back(fixed_vertex);
        centroid_sum += fixed_vertex;
    }
    polygon.centroid = {
        centroid_sum.x / count,
        centroid_sum.y / count,
    };
    polygon.public_bounds = {
        {(min_x + max_x) * 0.5f, (min_y + max_y) * 0.5f},
        {(max_x - min_x) * 0.5f, (max_y - min_y) * 0.5f},
    };
    for (int32_t i = 0; i < count; ++i) {
        const Vec a = polygon.fixed_vertices[static_cast<size_t>(i)];
        const Vec b = polygon.fixed_vertices[static_cast<size_t>((i + 1) % count)];
        if (a.x == b.x && a.y == b.y) {
            set_error("Polygon has a zero-length edge after fixed-point quantization.");
            return false;
        }
        const Vec c = polygon.fixed_vertices[static_cast<size_t>((i + 2) % count)];
        if (cross(a, b, c) == 0 && (b - a).dot(c - b) < 0) {
            set_error("Polygon has overlapping adjacent edges.");
            return false;
        }
        for (int32_t j = i + 1; j < count; ++j) {
            const bool adjacent = j == i + 1 || (i == 0 && j == count - 1);
            if (adjacent) continue;
            const Vec c0 = polygon.fixed_vertices[static_cast<size_t>(j)];
            const Vec c1 = polygon.fixed_vertices[static_cast<size_t>((j + 1) % count)];
            if (segments_intersect(a, b, c0, c1)) {
                set_error("Polygon must be simple and non-self-intersecting.");
                return false;
            }
        }
    }

    const Vec first = polygon.fixed_vertices[0];
    polygon.bounds = {first.x, first.y, first.x, first.y};
    int turn_sign = 0;
    int64_t signed_area = 0;
    polygon.convex = true;
    for (int32_t i = 0; i < count; ++i) {
        const Vec a = polygon.fixed_vertices[static_cast<size_t>(i)];
        const Vec b = polygon.fixed_vertices[static_cast<size_t>((i + 1) % count)];
        const Vec c = polygon.fixed_vertices[static_cast<size_t>((i + 2) % count)];
        polygon.bounds.min_x = std::min(polygon.bounds.min_x, a.x);
        polygon.bounds.min_y = std::min(polygon.bounds.min_y, a.y);
        polygon.bounds.max_x = std::max(polygon.bounds.max_x, a.x);
        polygon.bounds.max_y = std::max(polygon.bounds.max_y, a.y);
        signed_area += a.x * b.y - a.y * b.x;
        const int64_t turn = cross(a, b, c);
        if (turn == 0) continue;
        const int current = turn > 0 ? 1 : -1;
        if (turn_sign != 0 && current != turn_sign) polygon.convex = false;
        turn_sign = current;
    }
    if (turn_sign == 0) {
        set_error("Polygon vertices must enclose a non-zero area.");
        return false;
    }
    if (polygon.convex) return true;

    const int winding = signed_area >= 0 ? 1 : -1;
    std::vector<int> remaining(static_cast<size_t>(count));
    for (int32_t i = 0; i < count; ++i)
        remaining[static_cast<size_t>(i)] = i;
    polygon.triangles.reserve(static_cast<size_t>(count - 2) * 3);
    while (remaining.size() > 3) {
        bool clipped = false;
        for (size_t i = 0; i < remaining.size(); ++i) {
            const int previous = remaining[(i + remaining.size() - 1) % remaining.size()];
            const int current = remaining[i];
            const int next = remaining[(i + 1) % remaining.size()];
            const int64_t corner = cross(
                polygon.fixed_vertices[static_cast<size_t>(previous)],
                polygon.fixed_vertices[static_cast<size_t>(current)],
                polygon.fixed_vertices[static_cast<size_t>(next)]);
            if ((winding > 0 && corner <= 0) || (winding < 0 && corner >= 0))
                continue;
            bool contains = false;
            for (int candidate : remaining) {
                if (candidate == previous || candidate == current || candidate == next)
                    continue;
                if (point_in_triangle(
                        polygon.fixed_vertices[static_cast<size_t>(candidate)],
                        polygon.fixed_vertices[static_cast<size_t>(previous)],
                        polygon.fixed_vertices[static_cast<size_t>(current)],
                        polygon.fixed_vertices[static_cast<size_t>(next)], winding)) {
                    contains = true;
                    break;
                }
            }
            if (contains) continue;
            polygon.triangles.push_back(previous);
            polygon.triangles.push_back(current);
            polygon.triangles.push_back(next);
            remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            break;
        }
        if (!clipped) {
            set_error("Polygon must be simple and non-self-intersecting.");
            return false;
        }
    }
    polygon.triangles.insert(
        polygon.triangles.end(), remaining.begin(), remaining.end());
    return true;
}

Vec transform_vertex(Vec vertex, Vec translation, Axis axis_x, Axis axis_y) {
    return translation + axis_x.scale(vertex.x) + axis_y.scale(vertex.y);
}

void finish_proxy(Proxy& proxy) {
    if (proxy.count == 0) return;
    int64_t x = 0;
    int64_t y = 0;
    for (int i = 0; i < proxy.count; ++i) {
        const Vec vertex = proxy.vertex(i);
        x += vertex.x;
        y += vertex.y;
    }
    proxy.center = {x / proxy.count, y / proxy.count};
}

} // namespace

bool validate_shape(const arc_shape& shape) {
    switch (shape.kind) {
    case ARC_SHAPE_CIRCLE:
        return valid_vec(shape.circle.center) && valid_scalar(shape.circle.radius)
            && shape.circle.radius >= 0;
    case ARC_SHAPE_AABB:
        return valid_vec(shape.aabb.center) && valid_vec(shape.aabb.half_extents)
            && shape.aabb.half_extents.x >= 0 && shape.aabb.half_extents.y >= 0;
    case ARC_SHAPE_CAPSULE:
        return valid_vec(shape.capsule.a) && valid_vec(shape.capsule.b)
            && valid_scalar(shape.capsule.radius) && shape.capsule.radius >= 0;
    case ARC_SHAPE_OBB:
        return valid_vec(shape.obb.center) && valid_vec(shape.obb.half_extents)
            && shape.obb.half_extents.x >= 0 && shape.obb.half_extents.y >= 0;
    case ARC_SHAPE_POLYGON:
        return shape.polygon != nullptr && valid_vec(shape.polygon_translation);
    default:
        return false;
    }
}

// Number of convex SAT pieces: 1 for everything except a concave polygon, which
// is its triangle count. collide/sweep iterate over each piece.
int piece_count(const arc_shape& shape) {
    return shape.kind == ARC_SHAPE_POLYGON && shape.polygon
        && !shape.polygon->convex
        ? static_cast<int>(shape.polygon->triangles.size() / 3) : 1;
}

// Reduce a shape (or one triangle of a concave polygon) to a SAT proxy: hull
// vertices plus a rounding radius (circle = centre + r, capsule = 2 points + r).
Proxy make_proxy(const arc_shape& shape, int piece) {
    Proxy proxy;
    bool center_is_set = false;
    switch (shape.kind) {
    case ARC_SHAPE_CIRCLE:
        proxy.count = 1;
        proxy.inline_vertices[0] = Vec::from(shape.circle.center);
        proxy.center = proxy.inline_vertices[0];
        center_is_set = true;
        proxy.radius = std::abs(from_float(shape.circle.radius));
        break;
    case ARC_SHAPE_CAPSULE:
        proxy.count = 2;
        proxy.inline_vertices[0] = Vec::from(shape.capsule.a);
        proxy.inline_vertices[1] = Vec::from(shape.capsule.b);
        proxy.center = midpoint(proxy.inline_vertices[0], proxy.inline_vertices[1]);
        center_is_set = true;
        proxy.radius = std::abs(from_float(shape.capsule.radius));
        break;
    case ARC_SHAPE_AABB: {
        const Vec center = Vec::from(shape.aabb.center);
        const Vec half{std::abs(from_float(shape.aabb.half_extents.x)),
                       std::abs(from_float(shape.aabb.half_extents.y))};
        const Vec min = center - half;
        const Vec max = center + half;
        proxy.count = 4;
        proxy.inline_vertices = {min, Vec{max.x, min.y}, max, Vec{min.x, max.y}};
        break;
    }
    case ARC_SHAPE_OBB: {
        const Vec center = Vec::from(shape.obb.center);
        const Axis axis_x = Axis::from_angle(shape.obb.angle);
        const Axis axis_y = axis_x.perpendicular();
        const Vec x = axis_x.scale(std::abs(from_float(shape.obb.half_extents.x)));
        const Vec y = axis_y.scale(std::abs(from_float(shape.obb.half_extents.y)));
        proxy.count = 4;
        proxy.inline_vertices = {center - x - y, center + x - y,
                                 center + x + y, center - x + y};
        break;
    }
    case ARC_SHAPE_POLYGON: {
        if (!shape.polygon) break;
        const arc_polygon& polygon = *shape.polygon;
        const Vec translation = Vec::from(shape.polygon_translation);
        const Axis axis_x = Axis::from_angle(shape.polygon_rotation);
        const Axis axis_y = axis_x.perpendicular();
        Vec local_center;
        if (polygon.convex) {
            proxy.borrowed_vertices = polygon.fixed_vertices.data();
            proxy.count = static_cast<int>(polygon.fixed_vertices.size());
            local_center = polygon.centroid;
        } else {
            const size_t offset = static_cast<size_t>(piece) * 3;
            proxy.borrowed_vertices = polygon.fixed_vertices.data();
            proxy.borrowed_indices = polygon.triangles.data();
            proxy.borrowed_index_offset = static_cast<int>(offset);
            proxy.count = 3;
            for (size_t i = 0; i < 3; ++i) {
                const Vec value = polygon.fixed_vertices[
                    static_cast<size_t>(polygon.triangles[offset + i])];
                local_center += value;
            }
            local_center.x /= 3;
            local_center.y /= 3;
        }
        proxy.offset = translation;
        proxy.transformed = shape.polygon_rotation != 0;
        proxy.axis_x = axis_x;
        proxy.axis_y = axis_y;
        proxy.center = shape.polygon_rotation == 0
            ? local_center + translation
            : transform_vertex(local_center, translation, axis_x, axis_y);
        center_is_set = true;
        break;
    }
    default:
        break;
    }
    if (!center_is_set) finish_proxy(proxy);
    return proxy;
}

FxAabb proxy_bounds(const Proxy& proxy) {
    Vec first = proxy.vertex(0);
    int64_t min_x = first.x, min_y = first.y, max_x = first.x, max_y = first.y;
    for (int i = 1; i < proxy.count; ++i) {
        const Vec value = proxy.vertex(i);
        min_x = std::min(min_x, value.x);
        min_y = std::min(min_y, value.y);
        max_x = std::max(max_x, value.x);
        max_y = std::max(max_y, value.y);
    }
    min_x -= proxy.radius; min_y -= proxy.radius;
    max_x += proxy.radius; max_y += proxy.radius;
    return {{(min_x + max_x) / 2, (min_y + max_y) / 2},
            {(max_x - min_x) / 2, (max_y - min_y) / 2}};
}

// Broadphase bounds for a shape. The OBB/rotated-polygon cases derive the extent
// from the Q1.30 axes (ceil-divided so the box never under-covers), which is why
// the same integer axis must be used everywhere -- a coarser axis would produce a
// slightly different fat AABB and desync the broadphase from the reference.
Bounds shape_bounds(const arc_shape& shape) {
    switch (shape.kind) {
    case ARC_SHAPE_CIRCLE: {
        const Vec center = Vec::from(shape.circle.center);
        const int64_t radius = std::abs(from_float(shape.circle.radius));
        return {center.x - radius, center.y - radius,
                center.x + radius, center.y + radius};
    }
    case ARC_SHAPE_AABB: {
        const Vec center = Vec::from(shape.aabb.center);
        const Vec half{std::abs(from_float(shape.aabb.half_extents.x)),
                       std::abs(from_float(shape.aabb.half_extents.y))};
        return {center.x - half.x, center.y - half.y,
                center.x + half.x, center.y + half.y};
    }
    case ARC_SHAPE_CAPSULE: {
        const Vec a = Vec::from(shape.capsule.a);
        const Vec b = Vec::from(shape.capsule.b);
        const int64_t radius = std::abs(from_float(shape.capsule.radius));
        return {std::min(a.x, b.x) - radius, std::min(a.y, b.y) - radius,
                std::max(a.x, b.x) + radius, std::max(a.y, b.y) + radius};
    }
    case ARC_SHAPE_OBB: {
        const Vec center = Vec::from(shape.obb.center);
        const int64_t half_x = std::abs(from_float(shape.obb.half_extents.x));
        const int64_t half_y = std::abs(from_float(shape.obb.half_extents.y));
        const Axis axis_x = Axis::from_angle(shape.obb.angle);
        const Axis axis_y = axis_x.perpendicular();
        const int64_t extent_x = ceil_div_positive(
            std::abs(axis_x.x) * half_x + std::abs(axis_y.x) * half_y, AxisOne);
        const int64_t extent_y = ceil_div_positive(
            std::abs(axis_x.y) * half_x + std::abs(axis_y.y) * half_y, AxisOne);
        return {center.x - extent_x, center.y - extent_y,
                center.x + extent_x, center.y + extent_y};
    }
    case ARC_SHAPE_POLYGON: {
        const arc_polygon& polygon = *shape.polygon;
        const Vec translation = Vec::from(shape.polygon_translation);
        if (shape.polygon_rotation == 0)
            return polygon.bounds.translated(translation.x, translation.y);
        const Axis axis_x = Axis::from_angle(shape.polygon_rotation);
        const Axis axis_y = axis_x.perpendicular();
        Vec first = transform_vertex(
            polygon.fixed_vertices[0], translation, axis_x, axis_y);
        Bounds result{first.x, first.y, first.x, first.y};
        for (size_t i = 1; i < polygon.fixed_vertices.size(); ++i) {
            const Vec vertex = transform_vertex(
                polygon.fixed_vertices[i], translation, axis_x, axis_y);
            result.min_x = std::min(result.min_x, vertex.x);
            result.min_y = std::min(result.min_y, vertex.y);
            result.max_x = std::max(result.max_x, vertex.x);
            result.max_y = std::max(result.max_y, vertex.y);
        }
        return result;
    }
    default:
        throw std::out_of_range("Invalid shape kind.");
    }
}

arc_shape moved_shape(arc_shape shape, const arc_vec2& motion) {
    switch (shape.kind) {
    case ARC_SHAPE_CIRCLE:
        shape.circle.center.x += motion.x; shape.circle.center.y += motion.y; break;
    case ARC_SHAPE_AABB:
        shape.aabb.center.x += motion.x; shape.aabb.center.y += motion.y; break;
    case ARC_SHAPE_CAPSULE:
        shape.capsule.a.x += motion.x; shape.capsule.a.y += motion.y;
        shape.capsule.b.x += motion.x; shape.capsule.b.y += motion.y; break;
    case ARC_SHAPE_OBB:
        shape.obb.center.x += motion.x; shape.obb.center.y += motion.y; break;
    case ARC_SHAPE_POLYGON:
        shape.polygon_translation.x += motion.x;
        shape.polygon_translation.y += motion.y;
        break;
    default:
        break;
    }
    return shape;
}

// Polygon shapes hold a shared refcounted arc_polygon; retain/release keep it
// alive while any shape (or the world) references it. Primitives own no resources.
void retain_shape(const arc_shape& shape) {
    if (shape.kind == ARC_SHAPE_POLYGON && shape.polygon)
        shape.polygon->refs.fetch_add(1, std::memory_order_relaxed);
}

void release_shape(const arc_shape& shape) {
    if (shape.kind == ARC_SHAPE_POLYGON && shape.polygon
        && shape.polygon->refs.fetch_sub(1, std::memory_order_acq_rel) == 1)
        delete shape.polygon;
}

} // namespace arc

extern "C" {

arc_polygon* ARC_CALL arc_polygon_create(const arc_vec2* vertices, int32_t count) {
    try {
        auto* polygon = new arc_polygon;
        if (!arc::build_polygon(*polygon, vertices, count)) {
            delete polygon;
            return nullptr;
        }
        return polygon;
    } catch (const std::exception& exception) {
        arc::set_error(exception.what());
        return nullptr;
    } catch (...) {
        arc::set_error("Polygon allocation failed.");
        return nullptr;
    }
}

void ARC_CALL arc_polygon_retain(arc_polygon* polygon) {
    if (polygon) polygon->refs.fetch_add(1, std::memory_order_relaxed);
}

void ARC_CALL arc_polygon_release(arc_polygon* polygon) {
    if (polygon
        && polygon->refs.fetch_sub(1, std::memory_order_acq_rel) == 1)
        delete polygon;
}

int32_t ARC_CALL arc_polygon_get_count(const arc_polygon* polygon) {
    return polygon ? static_cast<int32_t>(polygon->vertices.size()) : 0;
}

arc_status ARC_CALL arc_polygon_get_vertices(
    const arc_polygon* polygon, arc_vec2* output, int32_t capacity, int32_t* required) {
    if (!polygon) {
        arc::set_error("Polygon is null.");
        return ARC_STATUS_INVALID_ARGUMENT;
    }
    return arc::copy_results(polygon->vertices, output, capacity, required);
}

arc_aabb ARC_CALL arc_polygon_get_bounds(const arc_polygon* polygon) {
    return polygon ? polygon->public_bounds : arc_aabb{};
}

arc_polygon* ARC_CALL arc_polygon_moved(const arc_polygon* polygon, arc_vec2 delta) {
    if (!polygon || !arc::valid_vec(delta)) {
        arc::set_error("Invalid polygon or translation.");
        return nullptr;
    }
    try {
        const arc::Vec fixed_delta = arc::Vec::from(delta);
        auto* moved = new arc_polygon;
        moved->vertices.resize(polygon->vertices.size());
        moved->fixed_vertices.resize(polygon->fixed_vertices.size());
        for (size_t i = 0; i < moved->vertices.size(); ++i) {
            moved->vertices[i] = {
                polygon->vertices[i].x + delta.x,
                polygon->vertices[i].y + delta.y,
            };
            moved->fixed_vertices[i] = polygon->fixed_vertices[i] + fixed_delta;
        }
        moved->triangles = polygon->triangles;
        moved->convex = polygon->convex;
        moved->centroid = polygon->centroid + fixed_delta;
        moved->bounds = polygon->bounds.translated(fixed_delta.x, fixed_delta.y);
        moved->public_bounds = {
            {polygon->public_bounds.center.x + delta.x,
             polygon->public_bounds.center.y + delta.y},
            polygon->public_bounds.half_extents,
        };
        return moved;
    } catch (const std::exception& exception) {
        arc::set_error(exception.what());
        return nullptr;
    } catch (...) {
        arc::set_error("Polygon move failed.");
        return nullptr;
    }
}

} // extern "C"
