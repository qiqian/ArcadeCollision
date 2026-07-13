#include "internal.h"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace {

using arc::Bounds;

bool equal(const Bounds& a, const Bounds& b) {
    return a.min_x == b.min_x && a.min_y == b.min_y
        && a.max_x == b.max_x && a.max_y == b.max_y;
}

bool scalar_overlaps(const Bounds& a, const Bounds& b) {
    return a.min_x <= b.max_x && b.min_x <= a.max_x
        && a.min_y <= b.max_y && b.min_y <= a.max_y;
}

bool scalar_contains(const Bounds& a, const Bounds& b) {
    return a.min_x <= b.min_x && a.min_y <= b.min_y
        && a.max_x >= b.max_x && a.max_y >= b.max_y;
}

Bounds scalar_unite(const Bounds& a, const Bounds& b) {
    return {std::min(a.min_x, b.min_x), std::min(a.min_y, b.min_y),
            std::max(a.max_x, b.max_x), std::max(a.max_y, b.max_y)};
}

uint64_t next_random(uint64_t& state) {
    state ^= state >> 12;
    state ^= state << 25;
    state ^= state >> 27;
    return state * UINT64_C(2685821657736338717);
}

int check_pair(const Bounds& a, const Bounds& b) {
    if (a.overlaps(b) != scalar_overlaps(a, b)) return 1;
    if (a.contains(b) != scalar_contains(a, b)) return 2;
    if (!equal(Bounds::unite(a, b), scalar_unite(a, b))) return 3;
    return 0;
}

} // namespace

int main() {
    const int64_t low = std::numeric_limits<int64_t>::min();
    const int64_t high = std::numeric_limits<int64_t>::max();
    const Bounds edge_cases[] = {
        {0, 0, 0, 0},
        {-1, -1, 1, 1},
        {low, low, low + 1, low + 1},
        {high - 1, high - 1, high, high},
        {-500000000, -500000000, 500000000, 500000000},
        {-10, 20, -5, 30},
    };
    for (const Bounds& a : edge_cases)
        for (const Bounds& b : edge_cases)
            if (const int result = check_pair(a, b)) return result;

    uint64_t state = UINT64_C(0x4d595df4d0f33173);
    for (int i = 0; i < 100000; ++i) {
        int64_t ax = static_cast<int64_t>(next_random(state) % 1000000001) - 500000000;
        int64_t ay = static_cast<int64_t>(next_random(state) % 1000000001) - 500000000;
        int64_t bx = static_cast<int64_t>(next_random(state) % 1000000001) - 500000000;
        int64_t by = static_cast<int64_t>(next_random(state) % 1000000001) - 500000000;
        const int64_t aw = static_cast<int64_t>(next_random(state) % 1000000);
        const int64_t ah = static_cast<int64_t>(next_random(state) % 1000000);
        const int64_t bw = static_cast<int64_t>(next_random(state) % 1000000);
        const int64_t bh = static_cast<int64_t>(next_random(state) % 1000000);
        const Bounds a{ax, ay, ax + aw, ay + ah};
        const Bounds b{bx, by, bx + bw, by + bh};
        if (const int result = check_pair(a, b)) return result + 10;

        const int64_t margin = static_cast<int64_t>(next_random(state) % 10000);
        const Bounds expanded{a.min_x - margin, a.min_y - margin,
                              a.max_x + margin, a.max_y + margin};
        if (!equal(a.expanded(margin), expanded)) return 20;

        const int64_t dx = static_cast<int64_t>(next_random(state) % 20001) - 10000;
        const int64_t dy = static_cast<int64_t>(next_random(state) % 20001) - 10000;
        const Bounds translated{a.min_x + dx, a.min_y + dy,
                                a.max_x + dx, a.max_y + dy};
        if (!equal(a.translated(dx, dy), translated)) return 21;
    }
    return 0;
}
