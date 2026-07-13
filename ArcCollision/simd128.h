#ifndef ARCCOLLISION_SIMD128_H
#define ARCCOLLISION_SIMD128_H

#include <cstdint>

// Private, integer-only 128-bit SIMD primitives. ArcCollision intentionally
// requires SSE4.2 on x86/x64 PCs and NEON on ARM64. No floating-point intrinsic
// belongs in this layer: float quantization and public conversion remain in
// fixed.cpp at the C ABI boundary.
#if defined(__aarch64__) || defined(_M_ARM64)
#if defined(_MSC_VER)
#include <arm64_neon.h>
#else
#include <arm_neon.h>
#endif
#define ARC_SIMD128_NEON 1
#elif defined(__x86_64__) || defined(_M_X64) \
    || defined(__i386__) || defined(_M_IX86)
#include <nmmintrin.h>
#define ARC_SIMD128_SSE42 1
#else
#error "ArcCollision requires an SSE4.2 PC or ARM64 NEON target. ARMv7 is unsupported."
#endif

namespace arc::simd128 {

#if defined(ARC_SIMD128_SSE42)

using I64x2 = __m128i;

inline I64x2 load(const int64_t* values) {
    return _mm_loadu_si128(reinterpret_cast<const __m128i*>(values));
}

inline void store(int64_t* values, I64x2 value) {
    _mm_storeu_si128(reinterpret_cast<__m128i*>(values), value);
}

inline I64x2 set(int64_t x, int64_t y) {
    return _mm_set_epi64x(y, x);
}

inline I64x2 splat(int64_t value) {
    return _mm_set1_epi64x(value);
}

inline I64x2 add(I64x2 a, I64x2 b) { return _mm_add_epi64(a, b); }
inline I64x2 subtract(I64x2 a, I64x2 b) { return _mm_sub_epi64(a, b); }

inline I64x2 minimum(I64x2 a, I64x2 b) {
    const I64x2 a_is_greater = _mm_cmpgt_epi64(a, b);
    return _mm_blendv_epi8(a, b, a_is_greater);
}

inline I64x2 maximum(I64x2 a, I64x2 b) {
    const I64x2 a_is_greater = _mm_cmpgt_epi64(a, b);
    return _mm_blendv_epi8(b, a, a_is_greater);
}

// True when neither comparison contains a lane where left > right.
inline bool no_lane_greater(
    I64x2 first_left, I64x2 first_right,
    I64x2 second_left, I64x2 second_right) {
    const I64x2 invalid = _mm_or_si128(
        _mm_cmpgt_epi64(first_left, first_right),
        _mm_cmpgt_epi64(second_left, second_right));
    return _mm_testz_si128(invalid, invalid) != 0;
}

#elif defined(ARC_SIMD128_NEON)

using I64x2 = int64x2_t;

inline I64x2 load(const int64_t* values) { return vld1q_s64(values); }
inline void store(int64_t* values, I64x2 value) { vst1q_s64(values, value); }

inline I64x2 set(int64_t x, int64_t y) {
    I64x2 result = vdupq_n_s64(x);
    return vsetq_lane_s64(y, result, 1);
}

inline I64x2 splat(int64_t value) { return vdupq_n_s64(value); }
inline I64x2 add(I64x2 a, I64x2 b) { return vaddq_s64(a, b); }
inline I64x2 subtract(I64x2 a, I64x2 b) { return vsubq_s64(a, b); }

inline I64x2 select(uint64x2_t mask, I64x2 when_true, I64x2 when_false) {
    return vreinterpretq_s64_u64(vbslq_u64(
        mask, vreinterpretq_u64_s64(when_true),
        vreinterpretq_u64_s64(when_false)));
}

inline I64x2 minimum(I64x2 a, I64x2 b) {
    return select(vcgtq_s64(a, b), b, a);
}

inline I64x2 maximum(I64x2 a, I64x2 b) {
    return select(vcgtq_s64(a, b), a, b);
}

inline bool no_lane_greater(
    I64x2 first_left, I64x2 first_right,
    I64x2 second_left, I64x2 second_right) {
    const uint64x2_t invalid = vorrq_u64(
        vcgtq_s64(first_left, first_right),
        vcgtq_s64(second_left, second_right));
    return (vgetq_lane_u64(invalid, 0) | vgetq_lane_u64(invalid, 1)) == 0;
}

#endif

} // namespace arc::simd128

#endif
