#include "internal.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

namespace {

using arc::Bounds;
using arc::Axis;

#if defined(_MSC_VER)
#define ARC_NOINLINE __declspec(noinline)
#else
#define ARC_NOINLINE __attribute__((noinline))
#endif

struct Scene {
    std::vector<Bounds> first;
    std::vector<Bounds> second;
    std::vector<uint32_t> order;
};

struct FixedScene {
    std::vector<int64_t> x;
    std::vector<int64_t> y;
    std::vector<int64_t> values;
};

volatile uint64_t benchmark_sink = 0;

uint64_t next_random(uint64_t& state) {
    state ^= state >> 12;
    state ^= state << 25;
    state ^= state >> 27;
    return state * UINT64_C(2685821657736338717);
}

Scene make_scene(size_t count, bool dense) {
    Scene scene;
    scene.first.reserve(count);
    scene.second.reserve(count);
    scene.order.resize(count);
    uint64_t state = dense
        ? UINT64_C(0x6a09e667f3bcc909) : UINT64_C(0xbb67ae8584caa73b);
    for (size_t i = 0; i < count; ++i) {
        const int64_t x = static_cast<int64_t>(next_random(state) % 1000000000);
        const int64_t y = static_cast<int64_t>(next_random(state) % 1000000000);
        const int64_t width = dense ? 100000 : 64;
        const int64_t offset_x = dense
            ? static_cast<int64_t>(next_random(state) % 150001) - 75000
            : static_cast<int64_t>(next_random(state) % 1000000) + 1000;
        const int64_t offset_y = dense
            ? static_cast<int64_t>(next_random(state) % 150001) - 75000
            : static_cast<int64_t>(next_random(state) % 1000000) + 1000;
        // Bounds is int32 now; the generated values fit int32, so narrow explicitly.
        scene.first.push_back({static_cast<int32_t>(x), static_cast<int32_t>(y),
                               static_cast<int32_t>(x + width), static_cast<int32_t>(y + width)});
        scene.second.push_back({
            static_cast<int32_t>(x + offset_x), static_cast<int32_t>(y + offset_y),
            static_cast<int32_t>(x + offset_x + width), static_cast<int32_t>(y + offset_y + width)});
        scene.order[i] = static_cast<uint32_t>(i);
    }
    for (size_t i = count; i > 1; --i) {
        const size_t other = static_cast<size_t>(next_random(state) % i);
        std::swap(scene.order[i - 1], scene.order[other]);
    }
    return scene;
}

FixedScene make_fixed_scene(size_t count) {
    FixedScene scene;
    scene.x.reserve(count);
    scene.y.reserve(count);
    scene.values.reserve(count);
    uint64_t state = UINT64_C(0x3c6ef372fe94f82b);
    for (size_t i = 0; i < count; ++i) {
        scene.x.push_back(static_cast<int64_t>(
            next_random(state) % UINT64_C(1000000001)) - 500000000);
        scene.y.push_back(static_cast<int64_t>(
            next_random(state) % UINT64_C(1000000001)) - 500000000);
        // Typical squared collision distances are well below the int64 ceiling;
        // retain 48 bits so sqrt's former start-bit scan is represented while
        // still covering values far larger than ordinary game geometry.
        scene.values.push_back(static_cast<int64_t>(
            next_random(state) & UINT64_C(0x0000ffffffffffff)));
    }
    return scene;
}

uint64_t scalar_magnitude(int64_t value) {
    return value < 0
        ? static_cast<uint64_t>(-(value + 1)) + 1
        : static_cast<uint64_t>(value);
}

ARC_NOINLINE int scalar_product_shift(int64_t value) {
    uint64_t magnitude = scalar_magnitude(value);
    int bits = 0;
    while (magnitude != 0) {
        ++bits;
        magnitude >>= 1;
    }
    return std::max(0, bits - 30);
}

ARC_NOINLINE int64_t scalar_sqrt(int64_t value) {
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

int64_t scalar_ratio_q30(int64_t numerator, int64_t denominator) {
    const bool negative = numerator < 0;
    uint64_t remainder = scalar_magnitude(numerator);
    const uint64_t divisor = static_cast<uint64_t>(denominator);
    if (remainder >= divisor)
        return negative ? -arc::AxisOne : arc::AxisOne;
    int64_t result = 0;
    for (int i = 0; i < arc::AxisShift; ++i) {
        result <<= 1;
        if (remainder >= divisor - remainder) {
            remainder -= divisor - remainder;
            result |= 1;
        } else {
            remainder += remainder;
        }
    }
    return negative ? -result : result;
}

ARC_NOINLINE Axis scalar_axis(int64_t x, int64_t y) {
    const int64_t length_sq = x * x + y * y;
    if (length_sq == 0) return {};
    int extra_shift = 60;
    while (extra_shift > 0
           && (length_sq >> (62 - extra_shift)) != 0)
        extra_shift -= 2;
    const int64_t high_length = scalar_sqrt(length_sq << extra_shift);
    const int64_t scale = int64_t{1} << (extra_shift >> 1);
    return {scalar_ratio_q30(x * scale, high_length),
            scalar_ratio_q30(y * scale, high_length)};
}

ARC_NOINLINE int64_t scalar_round_div(int64_t numerator, int64_t denominator) {
    const int64_t half = denominator >> 1;
    return numerator >= 0
        ? (numerator + half) / denominator
        : -((-numerator + half) / denominator);
}

ARC_NOINLINE uint64_t scalar_axis_pass(const FixedScene& scene, int repeats) {
    uint64_t result = 0;
    for (int repeat = 0; repeat < repeats; ++repeat)
        for (size_t i = 0; i < scene.x.size(); ++i) {
            const Axis axis = scalar_axis(scene.x[i], scene.y[i]);
            result += static_cast<uint64_t>(axis.x ^ axis.y);
        }
    return result;
}

ARC_NOINLINE uint64_t simd_axis_pass(const FixedScene& scene, int repeats) {
    uint64_t result = 0;
    for (int repeat = 0; repeat < repeats; ++repeat)
        for (size_t i = 0; i < scene.x.size(); ++i) {
            const Axis axis = Axis::from_components(scene.x[i], scene.y[i], {});
            result += static_cast<uint64_t>(axis.x ^ axis.y);
        }
    return result;
}

ARC_NOINLINE uint64_t scalar_shift_pass(const FixedScene& scene, int repeats) {
    uint64_t result = 0;
    for (int repeat = 0; repeat < repeats; ++repeat)
        for (int64_t value : scene.values)
            result += static_cast<uint64_t>(scalar_product_shift(value));
    return result;
}

ARC_NOINLINE uint64_t optimized_shift_pass(const FixedScene& scene, int repeats) {
    uint64_t result = 0;
    for (int repeat = 0; repeat < repeats; ++repeat)
        for (int64_t value : scene.values)
            result += static_cast<uint64_t>(arc::product_shift(value, 0, 0));
    return result;
}

ARC_NOINLINE uint64_t scalar_div_pass(const FixedScene& scene, int repeats) {
    uint64_t result = 0;
    for (int repeat = 0; repeat < repeats; ++repeat)
        for (int64_t value : scene.values)
            result += static_cast<uint64_t>(
                scalar_round_div(value, arc::AxisOne));
    return result;
}

ARC_NOINLINE uint64_t optimized_div_pass(const FixedScene& scene, int repeats) {
    uint64_t result = 0;
    for (int repeat = 0; repeat < repeats; ++repeat)
        for (int64_t value : scene.values)
            result += static_cast<uint64_t>(arc::round_axis(value));
    return result;
}

ARC_NOINLINE uint64_t scalar_overlap_pass(const Scene& scene, int repeats) {
    uint64_t result = 0;
    for (int repeat = 0; repeat < repeats; ++repeat) {
        for (uint32_t index : scene.order) {
            const Bounds& a = scene.first[index];
            const Bounds& b = scene.second[index];
            result += a.min_x <= b.max_x && b.min_x <= a.max_x
                && a.min_y <= b.max_y && b.min_y <= a.max_y;
        }
    }
    return result;
}

ARC_NOINLINE uint64_t simd_overlap_pass(const Scene& scene, int repeats) {
    uint64_t result = 0;
    for (int repeat = 0; repeat < repeats; ++repeat)
        for (uint32_t index : scene.order)
            result += scene.first[index].overlaps(scene.second[index]);
    return result;
}

// Gather four (randomly-indexed) box pairs and test them with one 4-wide SIMD
// overlap (SoA transpose + one separating-axis test), counting the mask bits.
ARC_NOINLINE uint64_t gather_overlap_pass(const Scene& scene, int repeats) {
    uint64_t result = 0;
    const size_t n = scene.order.size();
    for (int repeat = 0; repeat < repeats; ++repeat) {
        size_t i = 0;
        for (; i + 4 <= n; i += 4) {
            const uint32_t i0 = scene.order[i], i1 = scene.order[i + 1];
            const uint32_t i2 = scene.order[i + 2], i3 = scene.order[i + 3];
            const int mask = arc::simd128::overlap_mask4(
                arc::simd128::load_box(&scene.first[i0].min_x),
                arc::simd128::load_box(&scene.first[i1].min_x),
                arc::simd128::load_box(&scene.first[i2].min_x),
                arc::simd128::load_box(&scene.first[i3].min_x),
                arc::simd128::load_box(&scene.second[i0].min_x),
                arc::simd128::load_box(&scene.second[i1].min_x),
                arc::simd128::load_box(&scene.second[i2].min_x),
                arc::simd128::load_box(&scene.second[i3].min_x));
            result += (mask & 1) + ((mask >> 1) & 1)
                    + ((mask >> 2) & 1) + ((mask >> 3) & 1);
        }
        for (; i < n; ++i)
            result += scene.first[scene.order[i]].overlaps(scene.second[scene.order[i]]);
    }
    return result;
}

ARC_NOINLINE uint64_t scalar_unite_pass(const Scene& scene, int repeats) {
    uint64_t result = 0;
    for (int repeat = 0; repeat < repeats; ++repeat) {
        for (uint32_t index : scene.order) {
            const Bounds& a = scene.first[index];
            const Bounds& b = scene.second[index];
            const Bounds united{
                std::min(a.min_x, b.min_x), std::min(a.min_y, b.min_y),
                std::max(a.max_x, b.max_x), std::max(a.max_y, b.max_y)};
            result += static_cast<uint64_t>(united.min_x ^ united.min_y
                ^ united.max_x ^ united.max_y);
        }
    }
    return result;
}

ARC_NOINLINE uint64_t simd_unite_pass(const Scene& scene, int repeats) {
    uint64_t result = 0;
    for (int repeat = 0; repeat < repeats; ++repeat) {
        for (uint32_t index : scene.order) {
            const Bounds united = Bounds::unite(
                scene.first[index], scene.second[index]);
            result += static_cast<uint64_t>(united.min_x ^ united.min_y
                ^ united.max_x ^ united.max_y);
        }
    }
    return result;
}

template<class Function>
double best_milliseconds(Function function) {
    double best = std::numeric_limits<double>::max();
    for (int attempt = 0; attempt < 5; ++attempt) {
        const auto start = std::chrono::steady_clock::now();
        benchmark_sink ^= function();
        const auto stop = std::chrono::steady_clock::now();
        best = std::min(best,
            std::chrono::duration<double, std::milli>(stop - start).count());
    }
    return best;
}

void print_comparison(
    const char* name, double scalar_ms, double simd_ms) {
    std::cout << name << ": scalar=" << scalar_ms
              << " ms, simd=" << simd_ms
              << " ms, speedup=" << scalar_ms / simd_ms << "x\n";
}

} // namespace

int main() {
    constexpr size_t Count = 1u << 18;
    constexpr int Repeats = 16;
    const Scene sparse = make_scene(Count, false);
    const Scene dense = make_scene(Count, true);
    constexpr size_t FixedCount = 1u << 15;
    constexpr int FixedRepeats = 4;
    const FixedScene fixed = make_fixed_scene(FixedCount);

    // Warm code and data before taking the best of several runs.
    benchmark_sink ^= scalar_overlap_pass(sparse, 1);
    benchmark_sink ^= simd_overlap_pass(sparse, 1);

    // The 4-wide gather must count exactly the same overlaps as scalar.
    if (gather_overlap_pass(sparse, 1) != scalar_overlap_pass(sparse, 1)
        || gather_overlap_pass(dense, 1) != scalar_overlap_pass(dense, 1)) {
        std::cout << "gather overlap mask mismatch!\n";
        return 2;
    }

    print_comparison("overlap sparse (scalar vs gather4)",
        best_milliseconds([&] { return scalar_overlap_pass(sparse, Repeats); }),
        best_milliseconds([&] { return gather_overlap_pass(sparse, Repeats); }));
    print_comparison("overlap dense  (scalar vs gather4)",
        best_milliseconds([&] { return scalar_overlap_pass(dense, Repeats); }),
        best_milliseconds([&] { return gather_overlap_pass(dense, Repeats); }));
    print_comparison("bounds unite   (scalar vs simd)",
        best_milliseconds([&] { return scalar_unite_pass(dense, Repeats); }),
        best_milliseconds([&] { return simd_unite_pass(dense, Repeats); }));

    if (scalar_axis_pass(fixed, 1) != simd_axis_pass(fixed, 1)
        || scalar_shift_pass(fixed, 1) != optimized_shift_pass(fixed, 1)
        || scalar_div_pass(fixed, 1) != optimized_div_pass(fixed, 1)) {
        std::cout << "fixed-core optimization mismatch!\n";
        return 3;
    }
    print_comparison("axis normalize (scalar vs SIMD2)",
        best_milliseconds([&] { return scalar_axis_pass(fixed, FixedRepeats); }),
        best_milliseconds([&] { return simd_axis_pass(fixed, FixedRepeats); }));
    print_comparison("product shift (scan vs BSR)",
        best_milliseconds([&] { return scalar_shift_pass(fixed, FixedRepeats); }),
        best_milliseconds([&] { return optimized_shift_pass(fixed, FixedRepeats); }));
    print_comparison("axis division (idiv vs shift)",
        best_milliseconds([&] { return scalar_div_pass(fixed, FixedRepeats); }),
        best_milliseconds([&] { return optimized_div_pass(fixed, FixedRepeats); }));
    return benchmark_sink == UINT64_C(0xffffffffffffffff) ? 1 : 0;
}
