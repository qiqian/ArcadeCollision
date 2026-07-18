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

// A four-lane int32 vector holding one AABB {min_x, min_y, max_x, max_y}.
using Box4 = __m128i;
inline Box4 load_box(const int32_t* box) {
    return _mm_loadu_si128(reinterpret_cast<const __m128i*>(box));
}

// Overlap test of four box pairs at once. Each argument is a whole AABB. Returns
// a 4-bit mask whose bit i is set iff a<i> overlaps b<i>. Transposes the AoS boxes
// to SoA lanes, then one 4-lane separating-axis test -- no per-test branch/reduce.
inline int overlap_mask4(
    Box4 a0, Box4 a1, Box4 a2, Box4 a3,
    Box4 b0, Box4 b1, Box4 b2, Box4 b3) {
    const Box4 al0 = _mm_unpacklo_epi32(a0, a1);   // a0.min a1.min
    const Box4 al1 = _mm_unpacklo_epi32(a2, a3);
    const Box4 ah0 = _mm_unpackhi_epi32(a0, a1);   // a0.max a1.max
    const Box4 ah1 = _mm_unpackhi_epi32(a2, a3);
    const Box4 a_min_x = _mm_unpacklo_epi64(al0, al1);
    const Box4 a_min_y = _mm_unpackhi_epi64(al0, al1);
    const Box4 a_max_x = _mm_unpacklo_epi64(ah0, ah1);
    const Box4 a_max_y = _mm_unpackhi_epi64(ah0, ah1);
    const Box4 bl0 = _mm_unpacklo_epi32(b0, b1);
    const Box4 bl1 = _mm_unpacklo_epi32(b2, b3);
    const Box4 bh0 = _mm_unpackhi_epi32(b0, b1);
    const Box4 bh1 = _mm_unpackhi_epi32(b2, b3);
    const Box4 b_min_x = _mm_unpacklo_epi64(bl0, bl1);
    const Box4 b_min_y = _mm_unpackhi_epi64(bl0, bl1);
    const Box4 b_max_x = _mm_unpacklo_epi64(bh0, bh1);
    const Box4 b_max_y = _mm_unpackhi_epi64(bh0, bh1);
    const Box4 separated = _mm_or_si128(
        _mm_or_si128(_mm_cmpgt_epi32(a_min_x, b_max_x),
                     _mm_cmpgt_epi32(b_min_x, a_max_x)),
        _mm_or_si128(_mm_cmpgt_epi32(a_min_y, b_max_y),
                     _mm_cmpgt_epi32(b_min_y, a_max_y)));
    return (~_mm_movemask_ps(_mm_castsi128_ps(separated))) & 0xF;
}

// Overlap of one box `a` against four query boxes at once (the packet-traversal
// primitive: `a` is a tree node, q0..q3 are four simultaneous queries). Returns a
// 4-bit mask whose bit i is set iff a overlaps q<i>. `a` broadcasts each of its
// four components across the lanes; only the query side is transposed to SoA.
inline int overlap_mask_broadcast(
    Box4 a, Box4 q0, Box4 q1, Box4 q2, Box4 q3) {
    const Box4 a_min_x = _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 0, 0, 0));
    const Box4 a_min_y = _mm_shuffle_epi32(a, _MM_SHUFFLE(1, 1, 1, 1));
    const Box4 a_max_x = _mm_shuffle_epi32(a, _MM_SHUFFLE(2, 2, 2, 2));
    const Box4 a_max_y = _mm_shuffle_epi32(a, _MM_SHUFFLE(3, 3, 3, 3));
    const Box4 ql0 = _mm_unpacklo_epi32(q0, q1);
    const Box4 ql1 = _mm_unpacklo_epi32(q2, q3);
    const Box4 qh0 = _mm_unpackhi_epi32(q0, q1);
    const Box4 qh1 = _mm_unpackhi_epi32(q2, q3);
    const Box4 q_min_x = _mm_unpacklo_epi64(ql0, ql1);
    const Box4 q_min_y = _mm_unpackhi_epi64(ql0, ql1);
    const Box4 q_max_x = _mm_unpacklo_epi64(qh0, qh1);
    const Box4 q_max_y = _mm_unpackhi_epi64(qh0, qh1);
    const Box4 separated = _mm_or_si128(
        _mm_or_si128(_mm_cmpgt_epi32(a_min_x, q_max_x),
                     _mm_cmpgt_epi32(q_min_x, a_max_x)),
        _mm_or_si128(_mm_cmpgt_epi32(a_min_y, q_max_y),
                     _mm_cmpgt_epi32(q_min_y, a_max_y)));
    return (~_mm_movemask_ps(_mm_castsi128_ps(separated))) & 0xF;
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

// A four-lane int32 vector holding one AABB {min_x, min_y, max_x, max_y}.
using Box4 = int32x4_t;
inline Box4 load_box(const int32_t* box) { return vld1q_s32(box); }

// Overlap test of four box pairs at once; see the SSE version for semantics.
inline int overlap_mask4(
    Box4 a0, Box4 a1, Box4 a2, Box4 a3,
    Box4 b0, Box4 b1, Box4 b2, Box4 b3) {
    const int32x4x2_t at0 = vtrnq_s32(a0, a1);
    const int32x4x2_t at1 = vtrnq_s32(a2, a3);
    const Box4 a_min_x = vcombine_s32(vget_low_s32(at0.val[0]), vget_low_s32(at1.val[0]));
    const Box4 a_min_y = vcombine_s32(vget_low_s32(at0.val[1]), vget_low_s32(at1.val[1]));
    const Box4 a_max_x = vcombine_s32(vget_high_s32(at0.val[0]), vget_high_s32(at1.val[0]));
    const Box4 a_max_y = vcombine_s32(vget_high_s32(at0.val[1]), vget_high_s32(at1.val[1]));
    const int32x4x2_t bt0 = vtrnq_s32(b0, b1);
    const int32x4x2_t bt1 = vtrnq_s32(b2, b3);
    const Box4 b_min_x = vcombine_s32(vget_low_s32(bt0.val[0]), vget_low_s32(bt1.val[0]));
    const Box4 b_min_y = vcombine_s32(vget_low_s32(bt0.val[1]), vget_low_s32(bt1.val[1]));
    const Box4 b_max_x = vcombine_s32(vget_high_s32(bt0.val[0]), vget_high_s32(bt1.val[0]));
    const Box4 b_max_y = vcombine_s32(vget_high_s32(bt0.val[1]), vget_high_s32(bt1.val[1]));
    const uint32x4_t separated = vorrq_u32(
        vorrq_u32(vcgtq_s32(a_min_x, b_max_x), vcgtq_s32(b_min_x, a_max_x)),
        vorrq_u32(vcgtq_s32(a_min_y, b_max_y), vcgtq_s32(b_min_y, a_max_y)));
    const uint32_t weights[4] = {1u, 2u, 4u, 8u};
    const uint32x4_t weighted = vandq_u32(vmvnq_u32(separated), vld1q_u32(weights));
    return static_cast<int>(vaddvq_u32(weighted));
}

// One box `a` against four query boxes at once; see the SSE version for semantics.
inline int overlap_mask_broadcast(
    Box4 a, Box4 q0, Box4 q1, Box4 q2, Box4 q3) {
    const Box4 a_min_x = vdupq_laneq_s32(a, 0);
    const Box4 a_min_y = vdupq_laneq_s32(a, 1);
    const Box4 a_max_x = vdupq_laneq_s32(a, 2);
    const Box4 a_max_y = vdupq_laneq_s32(a, 3);
    const int32x4x2_t qt0 = vtrnq_s32(q0, q1);
    const int32x4x2_t qt1 = vtrnq_s32(q2, q3);
    const Box4 q_min_x = vcombine_s32(vget_low_s32(qt0.val[0]), vget_low_s32(qt1.val[0]));
    const Box4 q_min_y = vcombine_s32(vget_low_s32(qt0.val[1]), vget_low_s32(qt1.val[1]));
    const Box4 q_max_x = vcombine_s32(vget_high_s32(qt0.val[0]), vget_high_s32(qt1.val[0]));
    const Box4 q_max_y = vcombine_s32(vget_high_s32(qt0.val[1]), vget_high_s32(qt1.val[1]));
    const uint32x4_t separated = vorrq_u32(
        vorrq_u32(vcgtq_s32(a_min_x, q_max_x), vcgtq_s32(q_min_x, a_max_x)),
        vorrq_u32(vcgtq_s32(a_min_y, q_max_y), vcgtq_s32(q_min_y, a_max_y)));
    const uint32_t weights[4] = {1u, 2u, 4u, 8u};
    const uint32x4_t weighted = vandq_u32(vmvnq_u32(separated), vld1q_u32(weights));
    return static_cast<int>(vaddvq_u32(weighted));
}

#endif

} // namespace arc::simd128

#endif
