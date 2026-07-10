/*
    arccollision.h - single-header 2D collision library for arcade games
    ====================================================================

    A performance-oriented C99 port of the ArcCollision.Ref C# library. The two
    implementations share identical semantics so the reference project can be
    used to validate this one.

    USAGE
    -----
    This is an stb-style single-header library. In *exactly one* .c/.cpp file:

        #define ARCCOLLISION_IMPLEMENTATION
        #include "arccollision.h"

    Everywhere else just #include "arccollision.h".

    All vector math helpers are `static inline` and always available. The larger
    collision routines are declared as ARC_API and defined only in the
    implementation translation unit.

    CONVENTIONS
    -----------
    * AABBs are stored as center + half-extents.
    * Discrete tests return an arc_manifold whose `normal` points from the first
      shape towards the second; move A by -normal*depth (or B by +normal*depth)
      to separate them.
    * Swept tests return an arc_sweep_hit; `time` is the fraction (0..1) of the
      supplied motion at first contact.

    LICENSE: see LICENSE file at the repository root.
*/

#ifndef ARCCOLLISION_H
#define ARCCOLLISION_H

#include <math.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ARC_API
#define ARC_API extern
#endif

#ifndef ARC_INLINE
#define ARC_INLINE static inline
#endif

#define ARC_EPS 1e-6f

/* ------------------------------------------------------------------ types */

typedef struct { float x, y; } arc_vec2;

typedef struct { arc_vec2 center; float radius; } arc_circle;

/* center + half extents */
typedef struct { arc_vec2 center; arc_vec2 half; } arc_aabb;

/* capsule = points within `radius` of segment a..b */
typedef struct { arc_vec2 a, b; float radius; } arc_capsule;

typedef struct {
    int      colliding;   /* boolean */
    arc_vec2 normal;      /* from A towards B, unit length when colliding */
    float    depth;       /* penetration distance */
    arc_vec2 contact;     /* approximate world contact point */
} arc_manifold;

typedef struct {
    int      hit;         /* boolean */
    float    time;        /* fraction of motion (0..1) at first contact */
    arc_vec2 normal;      /* surface normal at contact */
    arc_vec2 point;       /* world contact point */
} arc_sweep_hit;

/* ------------------------------------------------------------- vec2 math */

ARC_INLINE arc_vec2 arc_v2(float x, float y)          { arc_vec2 v; v.x = x; v.y = y; return v; }
ARC_INLINE arc_vec2 arc_v2_add(arc_vec2 a, arc_vec2 b){ return arc_v2(a.x + b.x, a.y + b.y); }
ARC_INLINE arc_vec2 arc_v2_sub(arc_vec2 a, arc_vec2 b){ return arc_v2(a.x - b.x, a.y - b.y); }
ARC_INLINE arc_vec2 arc_v2_neg(arc_vec2 a)            { return arc_v2(-a.x, -a.y); }
ARC_INLINE arc_vec2 arc_v2_scale(arc_vec2 a, float s) { return arc_v2(a.x * s, a.y * s); }
ARC_INLINE float    arc_v2_dot(arc_vec2 a, arc_vec2 b){ return a.x * b.x + a.y * b.y; }
ARC_INLINE float    arc_v2_cross(arc_vec2 a, arc_vec2 b){ return a.x * b.y - a.y * b.x; }
ARC_INLINE float    arc_v2_len_sq(arc_vec2 a)         { return a.x * a.x + a.y * a.y; }
ARC_INLINE float    arc_v2_len(arc_vec2 a)            { return sqrtf(a.x * a.x + a.y * a.y); }
ARC_INLINE arc_vec2 arc_v2_perp(arc_vec2 a)           { return arc_v2(-a.y, a.x); }

ARC_INLINE float arc_v2_dist_sq(arc_vec2 a, arc_vec2 b) {
    float dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
}
ARC_INLINE float arc_v2_dist(arc_vec2 a, arc_vec2 b) { return sqrtf(arc_v2_dist_sq(a, b)); }

ARC_INLINE arc_vec2 arc_v2_lerp(arc_vec2 a, arc_vec2 b, float t) {
    return arc_v2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
}

ARC_INLINE arc_vec2 arc_v2_norm(arc_vec2 a, arc_vec2 fallback) {
    float ls = a.x * a.x + a.y * a.y;
    if (ls < 1e-12f) return fallback;
    float inv = 1.0f / sqrtf(ls);
    return arc_v2(a.x * inv, a.y * inv);
}

ARC_INLINE arc_vec2 arc_v2_min(arc_vec2 a, arc_vec2 b) {
    return arc_v2(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y);
}
ARC_INLINE arc_vec2 arc_v2_max(arc_vec2 a, arc_vec2 b) {
    return arc_v2(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y);
}

ARC_INLINE float arc_clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* --------------------------------------------------------- shape helpers */

ARC_INLINE arc_aabb arc_aabb_from_min_max(arc_vec2 mn, arc_vec2 mx) {
    arc_aabb b;
    b.center = arc_v2_scale(arc_v2_add(mn, mx), 0.5f);
    b.half   = arc_v2_scale(arc_v2_sub(mx, mn), 0.5f);
    return b;
}
ARC_INLINE arc_vec2 arc_aabb_min(arc_aabb b) { return arc_v2_sub(b.center, b.half); }
ARC_INLINE arc_vec2 arc_aabb_max(arc_aabb b) { return arc_v2_add(b.center, b.half); }
ARC_INLINE arc_aabb arc_aabb_expand(arc_aabb b, float amt) {
    arc_aabb r; r.center = b.center; r.half = arc_v2(b.half.x + amt, b.half.y + amt); return r;
}
ARC_INLINE int arc_aabb_overlaps(arc_aabb a, arc_aabb b) {
    return fabsf(a.center.x - b.center.x) <= (a.half.x + b.half.x)
        && fabsf(a.center.y - b.center.y) <= (a.half.y + b.half.y);
}
ARC_INLINE arc_aabb arc_aabb_union(arc_aabb a, arc_aabb b) {
    return arc_aabb_from_min_max(arc_v2_min(arc_aabb_min(a), arc_aabb_min(b)),
                                 arc_v2_max(arc_aabb_max(a), arc_aabb_max(b)));
}
ARC_INLINE arc_aabb arc_circle_bounds(arc_circle c) {
    arc_aabb b; b.center = c.center; b.half = arc_v2(c.radius, c.radius); return b;
}

/* --------------------------------------------------------- API: distance */

ARC_API arc_vec2 arc_closest_on_segment(arc_vec2 p, arc_vec2 a, arc_vec2 b, float *out_t);
ARC_API arc_vec2 arc_closest_on_aabb(arc_vec2 p, arc_aabb box);
ARC_API float    arc_closest_seg_seg(arc_vec2 p1, arc_vec2 q1, arc_vec2 p2, arc_vec2 q2,
                                     arc_vec2 *c1, arc_vec2 *c2);

/* -------------------------------------------------------- API: point in */

ARC_API int arc_point_in_circle(arc_vec2 p, arc_circle c);
ARC_API int arc_point_in_aabb(arc_vec2 p, arc_aabb box);
ARC_API int arc_point_in_capsule(arc_vec2 p, arc_capsule cap);

/* ---------------------------------------------------- API: narrowphase */

ARC_API arc_manifold arc_circle_vs_circle(arc_circle a, arc_circle b);
ARC_API arc_manifold arc_aabb_vs_aabb(arc_aabb a, arc_aabb b);
ARC_API arc_manifold arc_circle_vs_aabb(arc_circle c, arc_aabb box);
ARC_API arc_manifold arc_circle_vs_capsule(arc_circle c, arc_capsule cap);
ARC_API arc_manifold arc_capsule_vs_capsule(arc_capsule a, arc_capsule b);
ARC_API arc_manifold arc_capsule_vs_aabb(arc_capsule cap, arc_aabb box);

/* ---------------------------------------------------------- API: swept */

ARC_API arc_sweep_hit arc_ray_vs_circle(arc_vec2 origin, arc_vec2 d, arc_circle circle);
ARC_API arc_sweep_hit arc_ray_vs_aabb(arc_vec2 origin, arc_vec2 d, arc_aabb box);
ARC_API arc_sweep_hit arc_moving_circle_vs_circle(arc_circle mover, arc_vec2 motion, arc_circle target);
ARC_API arc_sweep_hit arc_moving_circle_vs_aabb(arc_circle mover, arc_vec2 motion, arc_aabb box);

/* ------------------------------------------------- API: grid broadphase */
/*
    A uniform-grid broadphase. Backing storage is caller-owned; no allocation
    happens inside the library unless you compile the helper constructor with
    ARCCOLLISION_USE_STDLIB (on by default). Cells store entity indices.
*/
#ifndef ARCCOLLISION_NO_BROADPHASE

typedef struct {
    int   id;
    arc_aabb bounds;
} arc_grid_entry;

typedef struct {
    float          cell_size;
    float          inv_cell_size;
    int            cols, rows;
    int            origin_x, origin_y; /* cell coord of index (0,0) */
    int           *cell_start;         /* cols*rows+1 prefix offsets */
    int           *cell_items;         /* entry indices bucketed per cell */
    arc_grid_entry*entries;
    int            entry_count;
    int            entry_cap;
    int            item_cap;
} arc_grid;

/* Initialise a grid covering `world` with the given cell size, using caller
   supplied scratch buffers. Returns 1 on success. */
ARC_API int  arc_grid_init(arc_grid *g, arc_aabb world, float cell_size,
                           arc_grid_entry *entry_buf, int entry_cap,
                           int *cell_start_buf, int *cell_items_buf, int item_cap);
ARC_API void arc_grid_clear(arc_grid *g);
ARC_API int  arc_grid_insert(arc_grid *g, int id, arc_aabb bounds); /* 1 on success */
ARC_API void arc_grid_build(arc_grid *g); /* call once after inserts, before queries */
/* Query entries overlapping `region`; writes up to `cap` ids, returns count. */
ARC_API int  arc_grid_query(const arc_grid *g, arc_aabb region, int *out_ids, int cap);

#endif /* ARCCOLLISION_NO_BROADPHASE */

/* ==================================================================== */
/*                          IMPLEMENTATION                              */
/* ==================================================================== */
#ifdef ARCCOLLISION_IMPLEMENTATION

static const arc_manifold ARC_MANIFOLD_NONE = { 0, {0,0}, 0.0f, {0,0} };
static const arc_sweep_hit ARC_SWEEP_MISS    = { 0, 1.0f, {0,0}, {0,0} };

arc_vec2 arc_closest_on_segment(arc_vec2 p, arc_vec2 a, arc_vec2 b, float *out_t) {
    arc_vec2 ab = arc_v2_sub(b, a);
    float len_sq = arc_v2_len_sq(ab);
    float t;
    if (len_sq < 1e-12f) { if (out_t) *out_t = 0.0f; return a; }
    t = arc_v2_dot(arc_v2_sub(p, a), ab) / len_sq;
    t = arc_clampf(t, 0.0f, 1.0f);
    if (out_t) *out_t = t;
    return arc_v2_add(a, arc_v2_scale(ab, t));
}

arc_vec2 arc_closest_on_aabb(arc_vec2 p, arc_aabb box) {
    arc_vec2 mn = arc_aabb_min(box), mx = arc_aabb_max(box);
    return arc_v2(arc_clampf(p.x, mn.x, mx.x), arc_clampf(p.y, mn.y, mx.y));
}

float arc_closest_seg_seg(arc_vec2 p1, arc_vec2 q1, arc_vec2 p2, arc_vec2 q2,
                          arc_vec2 *c1, arc_vec2 *c2) {
    arc_vec2 d1 = arc_v2_sub(q1, p1);
    arc_vec2 d2 = arc_v2_sub(q2, p2);
    arc_vec2 r  = arc_v2_sub(p1, p2);
    float a = arc_v2_len_sq(d1);
    float e = arc_v2_len_sq(d2);
    float f = arc_v2_dot(d2, r);
    float s, t;
    const float eps = 1e-12f;

    if (a <= eps && e <= eps) {
        s = t = 0.0f;
    } else if (a <= eps) {
        s = 0.0f;
        t = arc_clampf(f / e, 0.0f, 1.0f);
    } else {
        float c = arc_v2_dot(d1, r);
        if (e <= eps) {
            t = 0.0f;
            s = arc_clampf(-c / a, 0.0f, 1.0f);
        } else {
            float b = arc_v2_dot(d1, d2);
            float denom = a * e - b * b;
            s = denom > eps ? arc_clampf((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
            t = (b * s + f) / e;
            if (t < 0.0f) { t = 0.0f; s = arc_clampf(-c / a, 0.0f, 1.0f); }
            else if (t > 1.0f) { t = 1.0f; s = arc_clampf((b - c) / a, 0.0f, 1.0f); }
        }
    }
    {
        arc_vec2 r1 = arc_v2_add(p1, arc_v2_scale(d1, s));
        arc_vec2 r2 = arc_v2_add(p2, arc_v2_scale(d2, t));
        if (c1) *c1 = r1;
        if (c2) *c2 = r2;
        return arc_v2_dist_sq(r1, r2);
    }
}

int arc_point_in_circle(arc_vec2 p, arc_circle c) {
    return arc_v2_dist_sq(p, c.center) <= c.radius * c.radius;
}
int arc_point_in_aabb(arc_vec2 p, arc_aabb box) {
    arc_vec2 d = arc_v2_sub(p, box.center);
    return fabsf(d.x) <= box.half.x && fabsf(d.y) <= box.half.y;
}
int arc_point_in_capsule(arc_vec2 p, arc_capsule cap) {
    arc_vec2 cp = arc_closest_on_segment(p, cap.a, cap.b, NULL);
    return arc_v2_dist_sq(p, cp) <= cap.radius * cap.radius;
}

arc_manifold arc_circle_vs_circle(arc_circle a, arc_circle b) {
    arc_vec2 delta = arc_v2_sub(b.center, a.center);
    float r = a.radius + b.radius;
    float dist_sq = arc_v2_len_sq(delta);
    if (dist_sq > r * r) return ARC_MANIFOLD_NONE;
    {
        float dist = sqrtf(dist_sq);
        arc_vec2 normal = dist > 1e-6f ? arc_v2_scale(delta, 1.0f / dist) : arc_v2(1.0f, 0.0f);
        float depth = r - dist;
        arc_manifold m;
        m.colliding = 1;
        m.normal = normal;
        m.depth = depth;
        m.contact = arc_v2_add(a.center, arc_v2_scale(normal, a.radius - depth * 0.5f));
        return m;
    }
}

arc_manifold arc_aabb_vs_aabb(arc_aabb a, arc_aabb b) {
    arc_vec2 delta = arc_v2_sub(b.center, a.center);
    float overlap_x = (a.half.x + b.half.x) - fabsf(delta.x);
    float overlap_y;
    if (overlap_x <= 0.0f) return ARC_MANIFOLD_NONE;
    overlap_y = (a.half.y + b.half.y) - fabsf(delta.y);
    if (overlap_y <= 0.0f) return ARC_MANIFOLD_NONE;

    {
        arc_manifold m; m.colliding = 1;
        if (overlap_x < overlap_y) {
            float sign = delta.x < 0.0f ? -1.0f : 1.0f;
            m.normal = arc_v2(sign, 0.0f);
            m.depth = overlap_x;
            m.contact = arc_v2(a.center.x + sign * a.half.x, b.center.y);
        } else {
            float sign = delta.y < 0.0f ? -1.0f : 1.0f;
            m.normal = arc_v2(0.0f, sign);
            m.depth = overlap_y;
            m.contact = arc_v2(b.center.x, a.center.y + sign * a.half.y);
        }
        return m;
    }
}

arc_manifold arc_circle_vs_aabb(arc_circle c, arc_aabb box) {
    arc_vec2 closest = arc_closest_on_aabb(c.center, box);
    arc_vec2 delta = arc_v2_sub(closest, c.center);
    float dist_sq = arc_v2_len_sq(delta);
    if (dist_sq > c.radius * c.radius) return ARC_MANIFOLD_NONE;

    if (dist_sq > 1e-12f) {
        float dist = sqrtf(dist_sq);
        arc_manifold m; m.colliding = 1;
        m.normal = arc_v2_scale(delta, 1.0f / dist);
        m.depth = c.radius - dist;
        m.contact = closest;
        return m;
    } else {
        /* center inside the box: eject along nearest face. The normal stays
           A->B (towards the box centre) so it matches the outside branch. */
        arc_vec2 d = arc_v2_sub(c.center, box.center);
        float overlap_x = box.half.x - fabsf(d.x);
        float overlap_y = box.half.y - fabsf(d.y);
        arc_manifold m; m.colliding = 1;
        if (overlap_x < overlap_y) {
            float out_sign = d.x < 0.0f ? -1.0f : 1.0f;
            m.normal = arc_v2(-out_sign, 0.0f);
            m.depth = overlap_x + c.radius;
            m.contact = arc_v2(box.center.x + out_sign * box.half.x, c.center.y);
        } else {
            float out_sign = d.y < 0.0f ? -1.0f : 1.0f;
            m.normal = arc_v2(0.0f, -out_sign);
            m.depth = overlap_y + c.radius;
            m.contact = arc_v2(c.center.x, box.center.y + out_sign * box.half.y);
        }
        return m;
    }
}

arc_manifold arc_circle_vs_capsule(arc_circle c, arc_capsule cap) {
    arc_vec2 closest = arc_closest_on_segment(c.center, cap.a, cap.b, NULL);
    arc_circle spine; spine.center = closest; spine.radius = cap.radius;
    return arc_circle_vs_circle(c, spine);
}

arc_manifold arc_capsule_vs_capsule(arc_capsule a, arc_capsule b) {
    arc_vec2 c1, c2;
    arc_circle ca, cb;
    arc_closest_seg_seg(a.a, a.b, b.a, b.b, &c1, &c2);
    ca.center = c1; ca.radius = a.radius;
    cb.center = c2; cb.radius = b.radius;
    return arc_circle_vs_circle(ca, cb);
}

arc_manifold arc_capsule_vs_aabb(arc_capsule cap, arc_aabb box) {
    arc_vec2 sp = arc_closest_on_segment(box.center, cap.a, cap.b, NULL);
    arc_circle c; c.center = sp; c.radius = cap.radius;
    return arc_circle_vs_aabb(c, box);
}

arc_sweep_hit arc_ray_vs_circle(arc_vec2 origin, arc_vec2 d, arc_circle circle) {
    arc_vec2 m = arc_v2_sub(origin, circle.center);
    float a = arc_v2_len_sq(d);
    if (a < 1e-12f) {
        if (arc_v2_len_sq(m) <= circle.radius * circle.radius) {
            arc_sweep_hit h; h.hit = 1; h.time = 0.0f;
            h.normal = arc_v2_norm(m, arc_v2(1.0f, 0.0f));
            h.point = origin;
            return h;
        }
        return ARC_SWEEP_MISS;
    }
    {
        float b = arc_v2_dot(m, d);
        float c = arc_v2_len_sq(m) - circle.radius * circle.radius;
        if (c <= 0.0f) {
            arc_sweep_hit h; h.hit = 1; h.time = 0.0f;
            h.normal = arc_v2_norm(m, arc_v2_neg(arc_v2_norm(d, arc_v2(1.0f, 0.0f))));
            h.point = origin;
            return h;
        }
        {
            float disc = b * b - a * c;
            float t;
            arc_sweep_hit h;
            if (disc < 0.0f) return ARC_SWEEP_MISS;
            t = (-b - sqrtf(disc)) / a;
            if (t < 0.0f || t > 1.0f) return ARC_SWEEP_MISS;
            h.hit = 1; h.time = t;
            h.point = arc_v2_add(origin, arc_v2_scale(d, t));
            h.normal = arc_v2_norm(arc_v2_sub(h.point, circle.center), arc_v2(1.0f, 0.0f));
            return h;
        }
    }
}

static int arc__slab(float origin, float dir, float smin, float smax,
                     arc_vec2 nmin, arc_vec2 nmax,
                     float *tmin, float *tmax, arc_vec2 *normal) {
    const float eps = 1e-9f;
    if (fabsf(dir) < eps) {
        return origin >= smin && origin <= smax;
    } else {
        float inv = 1.0f / dir;
        float t1 = (smin - origin) * inv;
        float t2 = (smax - origin) * inv;
        arc_vec2 n1 = nmin, n2 = nmax;
        if (t1 > t2) { float tt = t1; t1 = t2; t2 = tt; { arc_vec2 nn = n1; n1 = n2; n2 = nn; } }
        if (t1 > *tmin) { *tmin = t1; *normal = n1; }
        if (t2 < *tmax) { *tmax = t2; }
        return *tmin <= *tmax;
    }
}

arc_sweep_hit arc_ray_vs_aabb(arc_vec2 origin, arc_vec2 d, arc_aabb box) {
    arc_vec2 mn = arc_aabb_min(box), mx = arc_aabb_max(box);
    float tmin = 0.0f, tmax = 1.0f;
    arc_vec2 normal = arc_v2(0.0f, 0.0f);
    if (!arc__slab(origin.x, d.x, mn.x, mx.x, arc_v2(-1,0), arc_v2(1,0), &tmin, &tmax, &normal))
        return ARC_SWEEP_MISS;
    if (!arc__slab(origin.y, d.y, mn.y, mx.y, arc_v2(0,-1), arc_v2(0,1), &tmin, &tmax, &normal))
        return ARC_SWEEP_MISS;
    {
        arc_sweep_hit h; h.hit = 1; h.time = tmin; h.normal = normal;
        h.point = arc_v2_add(origin, arc_v2_scale(d, tmin));
        return h;
    }
}

arc_sweep_hit arc_moving_circle_vs_circle(arc_circle mover, arc_vec2 motion, arc_circle target) {
    arc_circle expanded; arc_sweep_hit hit;
    expanded.center = target.center; expanded.radius = target.radius + mover.radius;
    hit = arc_ray_vs_circle(mover.center, motion, expanded);
    if (!hit.hit) return ARC_SWEEP_MISS;
    {
        arc_vec2 mover_at = arc_v2_add(mover.center, arc_v2_scale(motion, hit.time));
        hit.point = arc_v2_sub(mover_at, arc_v2_scale(hit.normal, mover.radius));
        return hit;
    }
}

arc_sweep_hit arc_moving_circle_vs_aabb(arc_circle mover, arc_vec2 motion, arc_aabb box) {
    float r = mover.radius;
    arc_aabb expanded = arc_aabb_expand(box, r);
    arc_sweep_hit best = ARC_SWEEP_MISS;
    arc_sweep_hit face = arc_ray_vs_aabb(mover.center, motion, expanded);
    arc_vec2 mn = arc_aabb_min(box), mx = arc_aabb_max(box);
    int i;
    arc_vec2 corners[4];

    if (face.hit) {
        arc_vec2 at = arc_v2_add(mover.center, arc_v2_scale(motion, face.time));
        int corner_zone = (at.x < mn.x || at.x > mx.x) && (at.y < mn.y || at.y > mx.y);
        if (!corner_zone) best = face;
    }

    corners[0] = mn;
    corners[1] = arc_v2(mx.x, mn.y);
    corners[2] = mx;
    corners[3] = arc_v2(mn.x, mx.y);
    for (i = 0; i < 4; ++i) {
        arc_circle cc; arc_sweep_hit h;
        cc.center = corners[i]; cc.radius = r;
        h = arc_ray_vs_circle(mover.center, motion, cc);
        if (h.hit && (!best.hit || h.time < best.time)) best = h;
    }
    return best;
}

/* ------------------------------------------------------------ broadphase */
#ifndef ARCCOLLISION_NO_BROADPHASE

ARC_INLINE int arc__cell_index(const arc_grid *g, int cx, int cy) {
    int lx = cx - g->origin_x;
    int ly = cy - g->origin_y;
    if (lx < 0) lx = 0; else if (lx >= g->cols) lx = g->cols - 1;
    if (ly < 0) ly = 0; else if (ly >= g->rows) ly = g->rows - 1;
    return ly * g->cols + lx;
}

int arc_grid_init(arc_grid *g, arc_aabb world, float cell_size,
                  arc_grid_entry *entry_buf, int entry_cap,
                  int *cell_start_buf, int *cell_items_buf, int item_cap) {
    arc_vec2 mn, mx;
    if (!g || cell_size <= 0.0f) return 0;
    mn = arc_aabb_min(world);
    mx = arc_aabb_max(world);
    g->cell_size = cell_size;
    g->inv_cell_size = 1.0f / cell_size;
    g->origin_x = (int)floorf(mn.x * g->inv_cell_size);
    g->origin_y = (int)floorf(mn.y * g->inv_cell_size);
    g->cols = (int)floorf(mx.x * g->inv_cell_size) - g->origin_x + 1;
    g->rows = (int)floorf(mx.y * g->inv_cell_size) - g->origin_y + 1;
    if (g->cols < 1) g->cols = 1;
    if (g->rows < 1) g->rows = 1;
    g->entries = entry_buf;
    g->entry_cap = entry_cap;
    g->entry_count = 0;
    g->cell_start = cell_start_buf;   /* needs cols*rows+1 ints */
    g->cell_items = cell_items_buf;   /* needs item_cap ints */
    g->item_cap = item_cap;
    return 1;
}

void arc_grid_clear(arc_grid *g) { if (g) g->entry_count = 0; }

int arc_grid_insert(arc_grid *g, int id, arc_aabb bounds) {
    if (!g || g->entry_count >= g->entry_cap) return 0;
    g->entries[g->entry_count].id = id;
    g->entries[g->entry_count].bounds = bounds;
    g->entry_count++;
    return 1;
}

void arc_grid_build(arc_grid *g) {
    /* Counting-sort the entry/cell references into cell_items. An entry that
       straddles several cells is referenced from each. */
    int cells, i, cx, cy;
    if (!g) return;
    cells = g->cols * g->rows;
    for (i = 0; i <= cells; ++i) g->cell_start[i] = 0;

    /* 1. count references, storing counts shifted by one slot */
    for (i = 0; i < g->entry_count; ++i) {
        arc_vec2 mn = arc_aabb_min(g->entries[i].bounds);
        arc_vec2 mx = arc_aabb_max(g->entries[i].bounds);
        int x0 = (int)floorf(mn.x * g->inv_cell_size);
        int y0 = (int)floorf(mn.y * g->inv_cell_size);
        int x1 = (int)floorf(mx.x * g->inv_cell_size);
        int y1 = (int)floorf(mx.y * g->inv_cell_size);
        for (cy = y0; cy <= y1; ++cy)
            for (cx = x0; cx <= x1; ++cx)
                g->cell_start[arc__cell_index(g, cx, cy) + 1]++;
    }

    /* 2. prefix sum -> cell_start[i] is the write offset for cell i */
    for (i = 0; i < cells; ++i) g->cell_start[i + 1] += g->cell_start[i];

    /* 3. scatter; advancing cell_start[ci] leaves it at cell (ci+1)'s start */
    for (i = 0; i < g->entry_count; ++i) {
        arc_vec2 mn = arc_aabb_min(g->entries[i].bounds);
        arc_vec2 mx = arc_aabb_max(g->entries[i].bounds);
        int x0 = (int)floorf(mn.x * g->inv_cell_size);
        int y0 = (int)floorf(mn.y * g->inv_cell_size);
        int x1 = (int)floorf(mx.x * g->inv_cell_size);
        int y1 = (int)floorf(mx.y * g->inv_cell_size);
        for (cy = y0; cy <= y1; ++cy) {
            for (cx = x0; cx <= x1; ++cx) {
                int ci = arc__cell_index(g, cx, cy);
                int slot = g->cell_start[ci];
                if (slot < g->item_cap) g->cell_items[slot] = i;
                g->cell_start[ci]++; /* advance even on overflow to keep offsets valid */
            }
        }
    }

    /* 4. restore prefix offsets: after step 3 cell_start[i] == old start[i+1] */
    for (i = cells; i > 0; --i) g->cell_start[i] = g->cell_start[i - 1];
    g->cell_start[0] = 0;
}

int arc_grid_query(const arc_grid *g, arc_aabb region, int *out_ids, int cap) {
    int count = 0, cx, cy;
    arc_vec2 mn, mx;
    int x0, y0, x1, y1;
    if (!g) return 0;
    mn = arc_aabb_min(region); mx = arc_aabb_max(region);
    x0 = (int)floorf(mn.x * g->inv_cell_size);
    y0 = (int)floorf(mn.y * g->inv_cell_size);
    x1 = (int)floorf(mx.x * g->inv_cell_size);
    y1 = (int)floorf(mx.y * g->inv_cell_size);
    for (cy = y0; cy <= y1; ++cy) {
        for (cx = x0; cx <= x1; ++cx) {
            int ci = arc__cell_index(g, cx, cy);
            int s = g->cell_start[ci];
            int e = g->cell_start[ci + 1];
            int k;
            for (k = s; k < e; ++k) {
                int ei = g->cell_items[k];
                arc_grid_entry *entry = &g->entries[ei];
                if (arc_aabb_overlaps(entry->bounds, region)) {
                    int dup = 0, q;
                    for (q = 0; q < count; ++q) if (out_ids[q] == entry->id) { dup = 1; break; }
                    if (!dup) {
                        if (count >= cap) return count;
                        out_ids[count++] = entry->id;
                    }
                }
            }
        }
    }
    return count;
}

#endif /* ARCCOLLISION_NO_BROADPHASE */

#endif /* ARCCOLLISION_IMPLEMENTATION */

#ifdef __cplusplus
}
#endif

#endif /* ARCCOLLISION_H */
