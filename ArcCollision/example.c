/*
    example.c - smoke test + micro-benchmark for arccollision.h

    Build (any C99 compiler):
        cl /O2 example.c            (MSVC)
        gcc -O2 -o example example.c -lm
        clang -O2 -o example example.c -lm
*/

#define ARCCOLLISION_IMPLEMENTATION
#include "arccollision.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static int g_failures = 0;

static void check(const char *name, int cond) {
    printf("  [%s] %s\n", cond ? "PASS" : "FAIL", name);
    if (!cond) g_failures++;
}

static int nearly(float a, float b) {
    float d = a - b;
    if (d < 0) d = -d;
    return d <= 1e-3f;
}

static void test_narrowphase(void) {
    arc_circle a, b;
    arc_aabb box;
    arc_manifold m;

    printf("Narrowphase\n");

    a.center = arc_v2(0, 0); a.radius = 1.0f;
    b.center = arc_v2(1.5f, 0); b.radius = 1.0f;
    m = arc_circle_vs_circle(a, b);
    check("circle/circle overlap detected", m.colliding);
    check("circle/circle depth ~0.5", nearly(m.depth, 0.5f));
    check("circle/circle normal +x", nearly(m.normal.x, 1.0f) && nearly(m.normal.y, 0.0f));

    b.center = arc_v2(3.0f, 0);
    m = arc_circle_vs_circle(a, b);
    check("circle/circle separated", !m.colliding);

    box.center = arc_v2(0, 0); box.half = arc_v2(1, 1);
    a.center = arc_v2(1.8f, 0); a.radius = 1.0f;
    m = arc_circle_vs_aabb(a, box);
    check("circle/aabb face overlap", m.colliding);
    /* normal points from circle A towards box B => -x */
    check("circle/aabb normal is A->B (-x)", nearly(m.normal.x, -1.0f));

    /* center-inside case: normal is A->B (-x), ejection pushes circle out +x */
    a.center = arc_v2(0.4f, 0); a.radius = 0.5f;
    m = arc_circle_vs_aabb(a, box);
    check("circle/aabb inside normal A->B (-x)", m.colliding && nearly(m.normal.x, -1.0f));

    {
        arc_aabb b1, b2;
        b1.center = arc_v2(0, 0);   b1.half = arc_v2(1, 1);
        b2.center = arc_v2(1.5f, 0);b2.half = arc_v2(1, 1);
        m = arc_aabb_vs_aabb(b1, b2);
        check("aabb/aabb overlap on x", m.colliding && nearly(m.depth, 0.5f) && nearly(m.normal.x, 1.0f));
    }
}

static void test_swept(void) {
    arc_circle mover, target;
    arc_aabb box;
    arc_sweep_hit h;

    printf("Swept\n");

    mover.center = arc_v2(-5, 0); mover.radius = 0.5f;
    target.center = arc_v2(0, 0);  target.radius = 1.0f;
    h = arc_moving_circle_vs_circle(mover, arc_v2(10, 0), target);
    check("moving circle hits circle", h.hit);
    /* first contact when centers are 1.5 apart: mover at x=-1.5 => t=3.5/10 */
    check("moving circle time ~0.35", nearly(h.time, 0.35f));

    box.center = arc_v2(0, 0); box.half = arc_v2(1, 1);
    mover.center = arc_v2(-5, 0); mover.radius = 0.5f;
    h = arc_moving_circle_vs_aabb(mover, arc_v2(10, 0), box);
    check("moving circle hits aabb", h.hit);
    /* contact when mover center reaches x = -1.5 => t = 3.5/10 */
    check("moving circle vs aabb time ~0.35", nearly(h.time, 0.35f));
    check("moving circle vs aabb normal -x", nearly(h.normal.x, -1.0f));

    /* fast mover that would tunnel a discrete test still connects */
    mover.center = arc_v2(-50, 0); mover.radius = 0.2f;
    h = arc_moving_circle_vs_aabb(mover, arc_v2(100, 0), box);
    check("no tunnelling for fast mover", h.hit);
}

#define N 2000
static void bench_broadphase(void) {
    static arc_circle circles[N];
    static arc_grid_entry entries[N];
    static int cell_start[64 * 64 + 1];
    static int cell_items[N * 4];
    static int query_out[256];
    arc_grid grid;
    arc_aabb world;
    int i, pair_checks = 0, hits = 0;
    clock_t t0, t1;

    printf("Broadphase (%d entities)\n", N);

    srand(1234);
    for (i = 0; i < N; ++i) {
        circles[i].center = arc_v2((float)(rand() % 1000), (float)(rand() % 1000));
        circles[i].radius = 4.0f + (float)(rand() % 6);
    }

    world = arc_aabb_from_min_max(arc_v2(0, 0), arc_v2(1000, 1000));
    arc_grid_init(&grid, world, 32.0f, entries, N,
                  cell_start, cell_items, N * 4);

    t0 = clock();
    arc_grid_clear(&grid);
    for (i = 0; i < N; ++i)
        arc_grid_insert(&grid, i, arc_circle_bounds(circles[i]));
    arc_grid_build(&grid);

    for (i = 0; i < N; ++i) {
        arc_aabb region = arc_circle_bounds(circles[i]);
        int n = arc_grid_query(&grid, region, query_out, 256);
        int k;
        for (k = 0; k < n; ++k) {
            int j = query_out[k];
            if (j <= i) continue;
            pair_checks++;
            if (arc_circle_vs_circle(circles[i], circles[j]).colliding) hits++;
        }
    }
    t1 = clock();

    printf("  narrowphase pairs tested: %d, overlaps: %d\n", pair_checks, hits);
    printf("  elapsed: %.2f ms\n", 1000.0 * (double)(t1 - t0) / CLOCKS_PER_SEC);
    check("broadphase produced candidate pairs", pair_checks > 0);
}

int main(void) {
    printf("arccollision.h smoke test\n=========================\n");
    test_narrowphase();
    test_swept();
    bench_broadphase();
    printf("\n%s\n", g_failures == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
    return g_failures == 0 ? 0 : 1;
}
