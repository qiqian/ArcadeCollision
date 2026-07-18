#include "internal.h"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace {

using arc::Bounds;
using arc::Axis;

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

uint64_t scalar_magnitude(int64_t value) {
    return value < 0
        ? static_cast<uint64_t>(-(value + 1)) + 1
        : static_cast<uint64_t>(value);
}

int scalar_bit_width(uint64_t value) {
    int bits = 0;
    while (value != 0) {
        ++bits;
        value >>= 1;
    }
    return bits;
}

int64_t scalar_sqrt(int64_t value) {
    if (value <= 0) return 0;
    uint64_t x = static_cast<uint64_t>(value);
    uint64_t result = 0;
    uint64_t bit = uint64_t{1} << 62;
    while (bit > x) bit >>= 2;
    while (bit != 0) {
        if (x >= result + bit) {
            x -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return static_cast<int64_t>(result);
}

int64_t scalar_fraction_bits(
    uint64_t remainder, uint64_t divisor, int bit_count) {
    int64_t result = 0;
    for (int i = 0; i < bit_count; ++i) {
        result <<= 1;
        if (remainder >= divisor - remainder) {
            remainder -= divisor - remainder;
            result |= 1;
        } else {
            remainder += remainder;
        }
    }
    return result;
}

int64_t scalar_ratio_q30(int64_t numerator, int64_t denominator) {
    const bool negative = numerator < 0;
    const uint64_t remainder = scalar_magnitude(numerator);
    const uint64_t divisor = static_cast<uint64_t>(denominator);
    const int64_t result = remainder >= divisor
        ? arc::AxisOne
        : scalar_fraction_bits(remainder, divisor, arc::AxisShift);
    return negative ? -result : result;
}

Axis scalar_axis(int64_t x, int64_t y, const Axis& fallback) {
    const int64_t length_sq = x * x + y * y;
    if (length_sq == 0) return fallback;
    int extra_shift = 60;
    while (extra_shift > 0
           && (length_sq >> (62 - extra_shift)) != 0)
        extra_shift -= 2;
    const int64_t high_length = scalar_sqrt(length_sq << extra_shift);
    if (high_length == 0) return fallback;
    const int64_t scale = int64_t{1} << (extra_shift >> 1);
    return {scalar_ratio_q30(x * scale, high_length),
            scalar_ratio_q30(y * scale, high_length)};
}

bool equal(const Axis& a, const Axis& b) {
    return a.x == b.x && a.y == b.y;
}

int check_fixed_core() {
    const int64_t sqrt_cases[] = {
        0, 1, 2, 3, 4, 15, 16, 17,
        INT64_C(0x3fffffff), INT64_C(0x40000000),
        INT64_C(0x3fffffffffffffff),
        std::numeric_limits<int64_t>::max(),
    };
    for (int64_t value : sqrt_cases)
        if (arc::sqrt_i64(value) != scalar_sqrt(value)) return 30;

    // Newton starts from a power-of-two upper bound. Values immediately around
    // perfect squares pin both its convergence condition and floor semantics,
    // including the largest root whose square fits in a signed int64.
    const int64_t sqrt_roots[] = {
        0, 1, 2, 3, 15, 16, 17, 255, 256, 257,
        32767, 65535, INT64_C(1048576), INT64_C(16777216),
        INT64_C(3037000499),
    };
    for (int64_t root : sqrt_roots) {
        const int64_t square = root * root;
        const int64_t first = square == 0 ? 0 : square - 1;
        const int64_t last = square == std::numeric_limits<int64_t>::max()
            ? square : square + 1;
        for (int64_t value = first;; ++value) {
            if (arc::sqrt_i64(value) != scalar_sqrt(value)) return 38;
            if (value == last) break;
        }
    }

    const int64_t division_cases[] = {
        -INT64_C(4000000000000000000), -arc::AxisOne - 1,
        -arc::AxisOne, -1, 0, 1, arc::AxisOne - 1, arc::AxisOne,
        INT64_C(4000000000000000000),
    };
    for (int64_t value : division_cases) {
        const int64_t half = arc::AxisOne >> 1;
        const int64_t rounded = value >= 0
            ? (value + half) / arc::AxisOne
            : -((-value + half) / arc::AxisOne);
        if (arc::round_div(value, arc::AxisOne) != rounded) return 31;
        if (value >= 0) {
            const int64_t ceiling = value == 0
                ? 0 : 1 + (value - 1) / arc::AxisOne;
            if (arc::ceil_div_positive(value, arc::AxisOne) != ceiling) return 32;
        }
    }

    const int64_t axis_cases[][2] = {
        {0, 0}, {1, 0}, {0, 1}, {-1, 1}, {1, -1},
        {255, 256}, {1000000, -999999},
        {500000000, 0}, {-500000000, 500000000},
    };
    const Axis fallback{123, -456};
    for (const auto& value : axis_cases) {
        if (!equal(Axis::from_components(value[0], value[1], fallback),
                   scalar_axis(value[0], value[1], fallback)))
            return 33;
    }

    uint64_t state = UINT64_C(0x243f6a8885a308d3);
    for (int i = 0; i < 100000; ++i) {
        uint64_t divisor = next_random(state)
            & static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
        if (divisor == 0) divisor = 1;
        const uint64_t rx = next_random(state) % divisor;
        const uint64_t ry = next_random(state) % divisor;
        const auto fractions = arc::simd128::fraction_bits_2(
            arc::simd128::set(static_cast<int64_t>(rx), static_cast<int64_t>(ry)),
            static_cast<int64_t>(divisor), arc::AxisShift);
        alignas(16) int64_t actual[2];
        arc::simd128::store(actual, fractions);
        if (actual[0] != scalar_fraction_bits(rx, divisor, arc::AxisShift)
            || actual[1] != scalar_fraction_bits(ry, divisor, arc::AxisShift))
            return 34;

        const int64_t x = static_cast<int64_t>(
            next_random(state) % UINT64_C(1000000001)) - 500000000;
        const int64_t y = static_cast<int64_t>(
            next_random(state) % UINT64_C(1000000001)) - 500000000;
        if (!equal(Axis::from_components(x, y, fallback),
                   scalar_axis(x, y, fallback)))
            return 35;

        const uint64_t magnitude = next_random(state)
            & static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
        const int expected_shift = std::max(0, scalar_bit_width(magnitude) - 30);
        if (arc::product_shift(static_cast<int64_t>(magnitude), 0, 0)
            != expected_shift)
            return 36;
        if (arc::sqrt_i64(static_cast<int64_t>(magnitude))
            != scalar_sqrt(static_cast<int64_t>(magnitude)))
            return 37;
    }
    return 0;
}

int check_pair(const Bounds& a, const Bounds& b) {
    if (a.overlaps(b) != scalar_overlaps(a, b)) return 1;
    if (a.contains(b) != scalar_contains(a, b)) return 2;
    if (!equal(Bounds::unite(a, b), scalar_unite(a, b))) return 3;
    return 0;
}

} // namespace

extern "C" int arc_run_simd_integer_tests() {
    const int32_t low = std::numeric_limits<int32_t>::min();
    const int32_t high = std::numeric_limits<int32_t>::max();
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
        int32_t ax = static_cast<int32_t>(next_random(state) % 1000000001) - 500000000;
        int32_t ay = static_cast<int32_t>(next_random(state) % 1000000001) - 500000000;
        int32_t bx = static_cast<int32_t>(next_random(state) % 1000000001) - 500000000;
        int32_t by = static_cast<int32_t>(next_random(state) % 1000000001) - 500000000;
        const int32_t aw = static_cast<int32_t>(next_random(state) % 1000000);
        const int32_t ah = static_cast<int32_t>(next_random(state) % 1000000);
        const int32_t bw = static_cast<int32_t>(next_random(state) % 1000000);
        const int32_t bh = static_cast<int32_t>(next_random(state) % 1000000);
        const Bounds a{ax, ay, ax + aw, ay + ah};
        const Bounds b{bx, by, bx + bw, by + bh};
        if (const int result = check_pair(a, b)) return result + 10;

        const int32_t margin = static_cast<int32_t>(next_random(state) % 10000);
        const Bounds expanded{a.min_x - margin, a.min_y - margin,
                              a.max_x + margin, a.max_y + margin};
        if (!equal(a.expanded(margin), expanded)) return 20;

        const int32_t dx = static_cast<int32_t>(next_random(state) % 20001) - 10000;
        const int32_t dy = static_cast<int32_t>(next_random(state) % 20001) - 10000;
        const Bounds translated{a.min_x + dx, a.min_y + dy,
                                a.max_x + dx, a.max_y + dy};
        if (!equal(a.translated(dx, dy), translated)) return 21;
    }
    if (const int result = check_fixed_core()) return result;
    return 0;
}
