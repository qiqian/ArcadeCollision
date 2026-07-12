// Fixed-point core: float<->fixed conversion, rounding-controlled integer
// division, the restoring integer square root, adaptive product scaling for
// degree-four discriminants, and the Q1.30 axis math (bit-serial division plus a
// CORDIC angle->unit-vector). Everything here is exact integer arithmetic so the
// managed reference and this native backend agree; see ArcCollision.Ref/Fixed.cs.
#include "internal.h"

#include <cmath>
#include <stdexcept>

namespace arc {
namespace {

// Arithmetic right shift with round-to-nearest, symmetric around zero.
int64_t round_shift(int64_t value, int shift) {
    const int64_t half = int64_t{1} << (shift - 1);
    return value >= 0 ? (value + half) >> shift : -((-value + half) >> shift);
}

// How far to pre-shift operands so their squared products stay within int64:
// leave ~30 significant bits, so a product of two scaled operands fits in ~60.
int product_shift_from_max(uint64_t value) {
    int bits = 0;
    while (value != 0) {
        ++bits;
        value >>= 1;
    }
    return std::max(0, bits - 30);
}

// numerator/denominator as a Q1.30 fraction (denominator > 0, |num| < den), by
// bit-serial long division: 30 iterations each emit one fractional bit. Used to
// normalise an axis component to unit length without a floating divide.
int64_t ratio_q30(int64_t numerator, int64_t denominator) {
    if (denominator <= 0)
        throw std::out_of_range("Axis denominator must be positive.");
    const bool negative = numerator < 0;
    uint64_t remainder = magnitude(numerator);
    const uint64_t divisor = static_cast<uint64_t>(denominator);
    if (remainder >= divisor)
        return negative ? -AxisOne : AxisOne;
    int64_t result = 0;
    for (int i = 0; i < AxisShift; ++i) {
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

// CORDIC rotation-mode table: arctan(2^-i) expressed as a 32-bit turn (2^32 =
// full circle). CordicGainInverse pre-divides by the CORDIC gain so the vectoring
// loop yields cosine/sine directly. Drives Axis::from_angle.
constexpr std::array<int32_t, 31> CordicAngles{{
    0x20000000, 0x12E4051E, 0x09FB385B, 0x051111D4,
    0x028B0D43, 0x0145D7E1, 0x00A2F61E, 0x00517C55,
    0x0028BE53, 0x00145F2F, 0x000A2F98, 0x000517CC,
    0x00028BE6, 0x000145F3, 0x0000A2FA, 0x0000517D,
    0x000028BE, 0x0000145F, 0x00000A30, 0x00000518,
    0x0000028C, 0x00000146, 0x000000A3, 0x00000051,
    0x00000029, 0x00000014, 0x0000000A, 0x00000005,
    0x00000003, 0x00000001, 0x00000001,
}};
constexpr int64_t CordicGainInverse = 652032874;

} // namespace

thread_local std::string error_text;

void set_error(const char* text) { error_text = text ? text : ""; }

bool valid_scalar(float value) {
    return std::isfinite(value) && value >= -ARC_MAX_COORDINATE
        && value <= ARC_MAX_COORDINATE;
}

bool valid_vec(arc_vec2 value) {
    return valid_scalar(value.x) && valid_scalar(value.y);
}

// Quantize a world-space float to 24.8 with round-half-to-even (banker's
// rounding), matching C# MathF.Round so both backends land on the same grid.
int64_t from_float(float value) {
    if (!valid_scalar(value))
        throw std::out_of_range("Fixed-point input must be finite and within range.");
    const float scaled = value * 256.0f;
    const bool negative = scaled < 0;
    const float absolute = std::fabs(scaled);
    const int64_t whole = static_cast<int64_t>(std::floor(absolute));
    const float fraction = absolute - static_cast<float>(whole);
    int64_t rounded = whole;
    if (fraction > 0.5f || (fraction == 0.5f && (whole & 1) != 0))
        ++rounded;
    return negative ? -rounded : rounded;
}

float to_float(int64_t value) { return value / static_cast<float>(FxOne); }
float to_t(int64_t value) { return value / static_cast<float>(TOne); }
float to_sq(int64_t value) {
    return value / static_cast<float>(FxOne * FxOne);
}

// |value| as unsigned, without overflowing on INT64_MIN.
uint64_t magnitude(int64_t value) {
    return value < 0
        ? static_cast<uint64_t>(-(value + 1)) + 1
        : static_cast<uint64_t>(value);
}

int64_t round_div(int64_t numerator, int64_t denominator) {
    if (denominator == 0)
        throw std::domain_error("Division by zero.");
    if (denominator < 0) {
        numerator = -numerator;
        denominator = -denominator;
    }
    const int64_t half = denominator >> 1;
    return numerator >= 0
        ? (numerator + half) / denominator
        : -((-numerator + half) / denominator);
}

int64_t floor_div(int64_t numerator, int64_t denominator) {
    if (denominator == 0)
        throw std::domain_error("Division by zero.");
    int64_t quotient = numerator / denominator;
    if ((numerator < 0) != (denominator < 0)
        && numerator % denominator != 0)
        --quotient;
    return quotient;
}

int64_t ceil_div_positive(int64_t numerator, int64_t denominator) {
    if (numerator < 0 || denominator <= 0)
        throw std::out_of_range("Ceiling division requires non-negative values.");
    return numerator == 0 ? 0 : 1 + (numerator - 1) / denominator;
}

int64_t mul_t(int64_t value, int64_t time) {
    return round_shift(value * time, TShift);
}

int64_t clamped_param(int64_t numerator, int64_t denominator) {
    if (denominator <= 0)
        throw std::out_of_range("Parameter denominator must be positive.");
    if (numerator <= 0) return 0;
    if (numerator >= denominator) return TOne;
    uint64_t remainder = static_cast<uint64_t>(numerator);
    const uint64_t divisor = static_cast<uint64_t>(denominator);
    int64_t result = 0;
    for (int i = 0; i < TShift; ++i) {
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

// numerator/denominator as a signed 16.16 value (whole part + 16 bit-serial
// fractional bits). Used for sweep times and slab entry/exit parameters.
int64_t ratio_t(int64_t numerator, int64_t denominator) {
    if (denominator == 0)
        throw std::domain_error("Division by zero.");
    const bool negative = (numerator < 0) != (denominator < 0);
    const uint64_t den = magnitude(denominator);
    const uint64_t num = magnitude(numerator);
    const uint64_t whole = num / den;
    uint64_t remainder = num % den;
    uint64_t fraction = 0;
    for (int i = 0; i < TShift; ++i) {
        fraction <<= 1;
        if (remainder >= den - remainder) {
            remainder -= den - remainder;
            fraction |= 1;
        } else {
            remainder += remainder;
        }
    }
    const uint64_t raw = whole * static_cast<uint64_t>(TOne) + fraction;
    if (raw > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
        throw std::overflow_error("Fixed-point ratio overflow.");
    const int64_t result = static_cast<int64_t>(raw);
    return negative ? -result : result;
}

int product_shift(int64_t a, int64_t b, int64_t c) {
    return product_shift_from_max(
        std::max(magnitude(a), std::max(magnitude(b), magnitude(c))));
}

int product_shift(int64_t a, int64_t b, int64_t c, int64_t d, int64_t e) {
    return product_shift_from_max(std::max(
        std::max(magnitude(a), magnitude(b)),
        std::max(magnitude(c), std::max(magnitude(d), magnitude(e)))));
}

int64_t scale_product_operand(int64_t value, int shift) {
    return shift == 0 ? value : value >> shift;
}

// Floor integer square root by the restoring (digit-by-digit) method: exact,
// branch-simple, and identical across platforms. Returns floor(sqrt(value)).
int64_t sqrt_i64(int64_t value) {
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

Axis Axis::from_vector(Vec value, Axis fallback) {
    return from_components(value.x, value.y, fallback);
}

// Normalize a raw (px,py) to a Q1.30 unit axis with adaptive precision. A short
// vector's length_sq is tiny, so a plain integer sqrt would lose most of its
// significant bits and the "unit" axis would be well off 1.0 (this is exactly the
// short-vector bug the reference fixed). Fix: left-shift length_sq as far as
// int64 allows before the sqrt, giving high_length = |v| * 2^(extra_shift/2);
// pre-scale the numerators by the matching 2^(extra_shift/2) so ratio_q30 sees
// full precision even for near-degenerate edges.
Axis Axis::from_components(int64_t px, int64_t py, Axis fallback) {
    const int64_t length_sq = px * px + py * py;
    if (length_sq == 0) return fallback;
    int extra_shift = 60;
    while (extra_shift > 0
           && (length_sq >> (62 - extra_shift)) != 0)
        extra_shift -= 2;
    const int64_t high_length = sqrt_i64(length_sq << extra_shift);
    if (high_length == 0) return fallback;
    const int component_shift = extra_shift >> 1;
    const int64_t scale = int64_t{1} << component_shift;
    return {ratio_q30(px * scale, high_length),
            ratio_q30(py * scale, high_length)};
}

// Turn a 32-bit angle (2^32 = full turn) into a Q1.30 unit axis via integer
// CORDIC. The four cardinal turns are returned exactly; otherwise the angle is
// folded into the first octant (quadrant + swap), the vectoring loop produces
// cosine/sine, and the result is rotated back into the correct quadrant. Using
// the same integer angle in both backends makes OBB axes bit-identical (and
// translation-invariant, since the axis never depends on position).
Axis Axis::from_angle(uint32_t angle) {
    if (angle == 0) return unit_x();
    if (angle == 0x40000000u) return unit_y();
    if (angle == 0x80000000u) return -unit_x();
    if (angle == 0xC0000000u) return -unit_y();
    const uint32_t quadrant = angle >> 30;
    const int64_t phase = angle & 0x3FFFFFFFu;
    const bool swap = phase > 0x20000000LL;   // fold the upper half of the quadrant
    int64_t z = swap ? 0x40000000LL - phase : phase;
    int64_t x_value = CordicGainInverse;
    int64_t y_value = 0;
    for (size_t i = 0; i < CordicAngles.size(); ++i) {
        const int64_t direction = z >= 0 ? 1 : -1;
        const int64_t next_x = x_value
            - direction * (y_value >> static_cast<int>(i));
        const int64_t next_y = y_value
            + direction * (x_value >> static_cast<int>(i));
        z -= direction * CordicAngles[i];
        x_value = next_x;
        y_value = next_y;
    }
    const int64_t cosine = swap ? y_value : x_value;
    const int64_t sine = swap ? x_value : y_value;
    switch (quadrant) {
    case 0: return from_components(cosine, sine, unit_x());
    case 1: return from_components(-sine, cosine, unit_y());
    case 2: return from_components(-cosine, -sine, -unit_x());
    default: return from_components(sine, -cosine, -unit_y());
    }
}

// Rotate a local-frame axis back into world space by the (basis_x, basis_y)
// frame, renormalizing so the result stays a Q1.30 unit axis.
Axis Axis::transform(Axis basis_x, Axis basis_y, Axis local) {
    const int64_t px = round_shift(
        basis_x.x * local.x + basis_y.x * local.y, AxisShift);
    const int64_t py = round_shift(
        basis_x.y * local.x + basis_y.y * local.y, AxisShift);
    return from_components(px, py, unit_x());
}

// Cosine of the angle between two unit axes: (Q1.30 . Q1.30) >> 30 back to Q1.30.
int64_t Axis::dot(Axis other) const {
    return round_shift(x * other.x + y * other.y, AxisShift);
}

// A 24.8 offset of `distance` along this unit axis: (axis * distance) >> 30.
Vec Axis::scale(int64_t distance) const {
    return {round_shift(x * distance, AxisShift),
            round_shift(y * distance, AxisShift)};
}

arc_aabb Bounds::to_public() const {
    // Match BpBounds.ToAabb/Aabb.FromMinMax exactly. Converting min/max first
    // preserves half-grid centres and extents when the integer span is odd.
    const float public_min_x = to_float(min_x);
    const float public_min_y = to_float(min_y);
    const float public_max_x = to_float(max_x);
    const float public_max_y = to_float(max_y);
    return {{(public_min_x + public_max_x) * 0.5f,
             (public_min_y + public_max_y) * 0.5f},
            {(public_max_x - public_min_x) * 0.5f,
             (public_max_y - public_min_y) * 0.5f}};
}

// Convert to the public float manifold, restoring signed-zero normal components
// (a mirrored contact wants -0.0, not +0.0) so mirror symmetry is bit-exact.
arc_manifold FxManifold::to_public() const {
    if (!colliding) return {};
    arc_vec2 public_normal = normal.to_public();
    if (normal.x == 0 && (negative_zero_mask & 1) != 0)
        public_normal.x = -0.0f;
    if (normal.y == 0 && (negative_zero_mask & 2) != 0)
        public_normal.y = -0.0f;
    return {1, public_normal, to_float(depth), contact.to_public()};
}

arc_sweep_hit FxSweep::to_public() const {
    if (!hit) return {0, 1.0f, {0, 0}, {0, 0}};
    arc_vec2 public_normal = normal.to_public();
    if (normal.x == 0 && (negative_zero_mask & 1) != 0)
        public_normal.x = -0.0f;
    if (normal.y == 0 && (negative_zero_mask & 2) != 0)
        public_normal.y = -0.0f;
    return {1, to_t(time), public_normal, point.to_public()};
}

Vec midpoint(Vec a, Vec b) {
    return {a.x + ((b.x - a.x) >> 1), a.y + ((b.y - a.y) >> 1)};
}

Proxy Proxy::translated(Vec delta) const {
    Proxy result = *this;
    result.offset += delta;
    result.center += delta;
    return result;
}

} // namespace arc
