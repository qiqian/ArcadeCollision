#include "internal.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

namespace {

using arc::Bounds;

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
    return benchmark_sink == UINT64_C(0xffffffffffffffff) ? 1 : 0;
}
