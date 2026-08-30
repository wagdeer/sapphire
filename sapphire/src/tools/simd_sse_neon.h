#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

#if defined(__aarch64__) || defined(_M_ARM64)
#include <arm_neon.h>
#define SIMD_NEON 1
#elif defined(__SSE2__) || defined(_M_X64)
#include <emmintrin.h>
#include <xmmintrin.h>
#define SIMD_SSE2 1
#else
#error "FBoW requires AArch64 NEON or x86 SSE2"
#endif

namespace simd {

inline constexpr std::size_t alignment = 16U;

class alignas(alignment) Mask32x4 {
public:
#if defined(SIMD_NEON)
    using native_type = uint32x4_t;
#else
    using native_type = __m128;
#endif

    explicit Mask32x4(const native_type value) : value_(value) {}

    bool lane(const std::size_t index) const {
#if defined(SIMD_NEON)
        alignas(alignment) std::uint32_t values[4];
        vst1q_u32(values, value_);
        return values[index] != 0U;
#else
        return (_mm_movemask_ps(value_) & (1 << index)) != 0;
#endif
    }

    bool all() const {
#if defined(SIMD_NEON)
        return vminvq_u32(value_) == std::numeric_limits<std::uint32_t>::max();
#else
        return _mm_movemask_ps(value_) == 0xF;
#endif
    }

    bool any() const {
#if defined(SIMD_NEON)
        return vmaxvq_u32(value_) != 0U;
#else
        return _mm_movemask_ps(value_) != 0;
#endif
    }

    native_type native() const {
        return value_;
    }

private:
    native_type value_;
};

class alignas(alignment) Mask64x2 {
public:
#if defined(SIMD_NEON)
    using native_type = uint64x2_t;
#else
    using native_type = __m128d;
#endif

    explicit Mask64x2(const native_type value) : value_(value) {}

    bool lane(const std::size_t index) const {
#if defined(SIMD_NEON)
        return (index == 0U ? vgetq_lane_u64(value_, 0) : vgetq_lane_u64(value_, 1)) != 0U;
#else
        return (_mm_movemask_pd(value_) & (1 << index)) != 0;
#endif
    }

    bool all() const {
        return lane(0U) && lane(1U);
    }

    bool any() const {
        return lane(0U) || lane(1U);
    }

    native_type native() const {
        return value_;
    }

private:
    native_type value_;
};

inline Mask32x4 operator&(const Mask32x4 lhs, const Mask32x4 rhs) {
#if defined(SIMD_NEON)
    return Mask32x4(vandq_u32(lhs.native(), rhs.native()));
#else
    return Mask32x4(_mm_and_ps(lhs.native(), rhs.native()));
#endif
}

inline Mask32x4 operator|(const Mask32x4 lhs, const Mask32x4 rhs) {
#if defined(SIMD_NEON)
    return Mask32x4(vorrq_u32(lhs.native(), rhs.native()));
#else
    return Mask32x4(_mm_or_ps(lhs.native(), rhs.native()));
#endif
}

inline Mask64x2 operator&(const Mask64x2 lhs, const Mask64x2 rhs) {
#if defined(SIMD_NEON)
    return Mask64x2(vandq_u64(lhs.native(), rhs.native()));
#else
    return Mask64x2(_mm_and_pd(lhs.native(), rhs.native()));
#endif
}

inline Mask64x2 operator|(const Mask64x2 lhs, const Mask64x2 rhs) {
#if defined(SIMD_NEON)
    return Mask64x2(vorrq_u64(lhs.native(), rhs.native()));
#else
    return Mask64x2(_mm_or_pd(lhs.native(), rhs.native()));
#endif
}

class alignas(alignment) Int4 {
public:
#if defined(SIMD_NEON)
    using native_type = int32x4_t;
#else
    using native_type = __m128i;
#endif

    Int4() : Int4(0) {}

    Int4(const std::int32_t value) {
#if defined(SIMD_NEON)
        value_ = vdupq_n_s32(value);
#else
        value_ = _mm_set1_epi32(value);
#endif
    }

    explicit Int4(const native_type value) : value_(value) {}

    void store_aligned(std::int32_t* values) const {
#if defined(SIMD_NEON)
        vst1q_s32(values, value_);
#else
        _mm_store_si128(reinterpret_cast<__m128i*>(values), value_);
#endif
    }

    std::int32_t lane(const std::size_t index) const {
#if defined(SIMD_NEON)
        switch (index) {
            case 0U: return vgetq_lane_s32(value_, 0);
            case 1U: return vgetq_lane_s32(value_, 1);
            case 2U: return vgetq_lane_s32(value_, 2);
            default: return vgetq_lane_s32(value_, 3);
        }
#else
        alignas(alignment) std::int32_t values[4];
        _mm_store_si128(reinterpret_cast<__m128i*>(values), value_);
        return values[index];
#endif
    }

    native_type native() const {
        return value_;
    }

private:
    native_type value_;
};

class alignas(alignment) Float4 {
public:
#if defined(SIMD_NEON)
    using native_type = float32x4_t;
#else
    using native_type = __m128;
#endif

    Float4() : Float4(0.0F) {}

    Float4(const float value) {
#if defined(SIMD_NEON)
        value_ = vdupq_n_f32(value);
#else
        value_ = _mm_set1_ps(value);
#endif
    }

    Float4(const float x0, const float x1, const float x2, const float x3) {
#if defined(SIMD_NEON)
        const float values[4] = {x0, x1, x2, x3};
        value_ = vld1q_f32(values);
#else
        value_ = _mm_set_ps(x3, x2, x1, x0);
#endif
    }

    explicit Float4(const native_type value) : value_(value) {}

    static Float4 load(const float* values) {
#if defined(SIMD_NEON)
        return Float4(vld1q_f32(values));
#else
        return Float4(_mm_loadu_ps(values));
#endif
    }

    static Float4 load_aligned(const float* values) {
#if defined(SIMD_NEON)
        return Float4(vld1q_f32(values));
#else
        return Float4(_mm_load_ps(values));
#endif
    }

    void store(float* values) const {
#if defined(SIMD_NEON)
        vst1q_f32(values, value_);
#else
        _mm_storeu_ps(values, value_);
#endif
    }

    void store_aligned(float* values) const {
#if defined(SIMD_NEON)
        vst1q_f32(values, value_);
#else
        _mm_store_ps(values, value_);
#endif
    }

    float lane(const std::size_t index) const {
#if defined(SIMD_NEON)
        switch (index) {
            case 0U: return vgetq_lane_f32(value_, 0);
            case 1U: return vgetq_lane_f32(value_, 1);
            case 2U: return vgetq_lane_f32(value_, 2);
            default: return vgetq_lane_f32(value_, 3);
        }
#else
        switch (index) {
            case 0U: return _mm_cvtss_f32(value_);
            case 1U: return _mm_cvtss_f32(_mm_shuffle_ps(value_, value_, 0x55));
            case 2U: return _mm_cvtss_f32(_mm_shuffle_ps(value_, value_, 0xAA));
            default: return _mm_cvtss_f32(_mm_shuffle_ps(value_, value_, 0xFF));
        }
#endif
    }

    void set_lane(const std::size_t index, const float value) {
        alignas(alignment) float values[4];
        store_aligned(values);
        values[index] = value;
        *this = load_aligned(values);
    }

    float sum() const {
#if defined(SIMD_NEON)
        return vaddvq_f32(value_);
#else
        const __m128 pair_sum = _mm_add_ps(value_, _mm_movehl_ps(value_, value_));
        const __m128 swapped = _mm_shuffle_ps(pair_sum, pair_sum, 0x1);
        return _mm_cvtss_f32(_mm_add_ss(pair_sum, swapped));
#endif
    }

    native_type native() const {
        return value_;
    }

    friend Float4 operator+(const Float4 lhs, const Float4 rhs) {
#if defined(SIMD_NEON)
        return Float4(vaddq_f32(lhs.value_, rhs.value_));
#else
        return Float4(_mm_add_ps(lhs.value_, rhs.value_));
#endif
    }

    friend Float4 operator-(const Float4 lhs, const Float4 rhs) {
#if defined(SIMD_NEON)
        return Float4(vsubq_f32(lhs.value_, rhs.value_));
#else
        return Float4(_mm_sub_ps(lhs.value_, rhs.value_));
#endif
    }

    friend Float4 operator*(const Float4 lhs, const Float4 rhs) {
#if defined(SIMD_NEON)
        return Float4(vmulq_f32(lhs.value_, rhs.value_));
#else
        return Float4(_mm_mul_ps(lhs.value_, rhs.value_));
#endif
    }

    friend Float4 operator/(const Float4 lhs, const Float4 rhs) {
#if defined(SIMD_NEON)
        return Float4(vdivq_f32(lhs.value_, rhs.value_));
#else
        return Float4(_mm_div_ps(lhs.value_, rhs.value_));
#endif
    }

    friend Float4 operator-(const Float4 value) {
        return Float4(0.0F) - value;
    }

    Float4& operator+=(const Float4 rhs) {
        *this = *this + rhs;
        return *this;
    }

    Float4& operator-=(const Float4 rhs) {
        *this = *this - rhs;
        return *this;
    }

    Float4& operator*=(const Float4 rhs) {
        *this = *this * rhs;
        return *this;
    }

    static Float4 sqrt(const Float4 value) {
#if defined(SIMD_NEON)
        return Float4(vsqrtq_f32(value.value_));
#else
        return Float4(_mm_sqrt_ps(value.value_));
#endif
    }

    static Float4 reciprocal(const Float4 value) {
        return Float4(1.0F) / value;
    }

    static Float4 square(const Float4 value) {
        return value * value;
    }

    static Float4 multiply_add(const Float4 lhs, const Float4 rhs, const Float4 addend) {
#if defined(SIMD_NEON)
        return Float4(vfmaq_f32(addend.value_, lhs.value_, rhs.value_));
#else
        return Float4(_mm_add_ps(_mm_mul_ps(lhs.value_, rhs.value_), addend.value_));
#endif
    }

    static Float4 abs(const Float4 value) {
#if defined(SIMD_NEON)
        return Float4(vabsq_f32(value.value_));
#else
        return Float4(_mm_andnot_ps(_mm_set1_ps(-0.0F), value.value_));
#endif
    }

    static Float4 min(const Float4 lhs, const Float4 rhs) {
#if defined(SIMD_NEON)
        return Float4(vminq_f32(lhs.value_, rhs.value_));
#else
        return Float4(_mm_min_ps(lhs.value_, rhs.value_));
#endif
    }

    static Float4 max(const Float4 lhs, const Float4 rhs) {
#if defined(SIMD_NEON)
        return Float4(vmaxq_f32(lhs.value_, rhs.value_));
#else
        return Float4(_mm_max_ps(lhs.value_, rhs.value_));
#endif
    }

private:
    native_type value_;
};

inline Float4 pairwise_sum(const Float4 lhs, const Float4 rhs) {
#if defined(SIMD_NEON)
    return Float4(vpaddq_f32(lhs.native(), rhs.native()));
#else
    const __m128 lhs_pairs = _mm_add_ps(lhs.native(), _mm_shuffle_ps(lhs.native(), lhs.native(), _MM_SHUFFLE(2, 3, 0, 1)));
    const __m128 rhs_pairs = _mm_add_ps(rhs.native(), _mm_shuffle_ps(rhs.native(), rhs.native(), _MM_SHUFFLE(2, 3, 0, 1)));
    return Float4(_mm_shuffle_ps(lhs_pairs, rhs_pairs, _MM_SHUFFLE(2, 0, 2, 0)));
#endif
}

inline Int4 round_to_int(const Float4 value) {
#if defined(SIMD_NEON)
    return Int4(vcvtnq_s32_f32(value.native()));
#else
    return Int4(_mm_cvtps_epi32(value.native()));
#endif
}

class alignas(alignment) Double2 {
public:
#if defined(SIMD_NEON)
    using native_type = float64x2_t;
#else
    using native_type = __m128d;
#endif

    Double2() : Double2(0.0) {}

    Double2(const double value) {
#if defined(SIMD_NEON)
        value_ = vdupq_n_f64(value);
#else
        value_ = _mm_set1_pd(value);
#endif
    }

    Double2(const double first, const double second) {
#if defined(SIMD_NEON)
        const double values[2] = {first, second};
        value_ = vld1q_f64(values);
#else
        value_ = _mm_set_pd(second, first);
#endif
    }

    explicit Double2(const native_type value) : value_(value) {}

    static Double2 load(const double* values) {
#if defined(SIMD_NEON)
        return Double2(vld1q_f64(values));
#else
        return Double2(_mm_loadu_pd(values));
#endif
    }

    static Double2 load_aligned(const double* values) {
#if defined(SIMD_NEON)
        return Double2(vld1q_f64(values));
#else
        return Double2(_mm_load_pd(values));
#endif
    }

    void store(double* values) const {
#if defined(SIMD_NEON)
        vst1q_f64(values, value_);
#else
        _mm_storeu_pd(values, value_);
#endif
    }

    void store_aligned(double* values) const {
#if defined(SIMD_NEON)
        vst1q_f64(values, value_);
#else
        _mm_store_pd(values, value_);
#endif
    }

    double lane(const std::size_t index) const {
#if defined(SIMD_NEON)
        return index == 0U ? vgetq_lane_f64(value_, 0) : vgetq_lane_f64(value_, 1);
#else
        return index == 0U ? _mm_cvtsd_f64(value_) : _mm_cvtsd_f64(_mm_unpackhi_pd(value_, value_));
#endif
    }

    void set_lane(const std::size_t index, const double value) {
#if defined(SIMD_NEON)
        value_ = index == 0U ? vsetq_lane_f64(value, value_, 0) : vsetq_lane_f64(value, value_, 1);
#else
        value_ = index == 0U ? _mm_move_sd(value_, _mm_set_sd(value)) : _mm_unpacklo_pd(value_, _mm_set_sd(value));
#endif
    }

    double sum() const {
#if defined(SIMD_NEON)
        return vaddvq_f64(value_);
#else
        const __m128d high = _mm_unpackhi_pd(value_, value_);
        return _mm_cvtsd_f64(_mm_add_sd(value_, high));
#endif
    }

    native_type native() const {
        return value_;
    }

    friend Double2 operator+(const Double2 lhs, const Double2 rhs) {
#if defined(SIMD_NEON)
        return Double2(vaddq_f64(lhs.value_, rhs.value_));
#else
        return Double2(_mm_add_pd(lhs.value_, rhs.value_));
#endif
    }

    friend Double2 operator-(const Double2 lhs, const Double2 rhs) {
#if defined(SIMD_NEON)
        return Double2(vsubq_f64(lhs.value_, rhs.value_));
#else
        return Double2(_mm_sub_pd(lhs.value_, rhs.value_));
#endif
    }

    friend Double2 operator*(const Double2 lhs, const Double2 rhs) {
#if defined(SIMD_NEON)
        return Double2(vmulq_f64(lhs.value_, rhs.value_));
#else
        return Double2(_mm_mul_pd(lhs.value_, rhs.value_));
#endif
    }

    friend Double2 operator/(const Double2 lhs, const Double2 rhs) {
#if defined(SIMD_NEON)
        return Double2(vdivq_f64(lhs.value_, rhs.value_));
#else
        return Double2(_mm_div_pd(lhs.value_, rhs.value_));
#endif
    }

    friend Double2 operator-(const Double2 value) {
        return Double2(0.0) - value;
    }

    Double2& operator+=(const Double2 rhs) {
        *this = *this + rhs;
        return *this;
    }

    Double2& operator-=(const Double2 rhs) {
        *this = *this - rhs;
        return *this;
    }

    Double2& operator*=(const Double2 rhs) {
        *this = *this * rhs;
        return *this;
    }

    static Double2 sqrt(const Double2 value) {
#if defined(SIMD_NEON)
        return Double2(vsqrtq_f64(value.value_));
#else
        return Double2(_mm_sqrt_pd(value.value_));
#endif
    }

    static Double2 reciprocal(const Double2 value) {
        return Double2(1.0) / value;
    }

    static Double2 square(const Double2 value) {
        return value * value;
    }

    static Double2 multiply_add(const Double2 lhs, const Double2 rhs, const Double2 addend) {
#if defined(SIMD_NEON)
        return Double2(vfmaq_f64(addend.value_, lhs.value_, rhs.value_));
#else
        return Double2(_mm_add_pd(_mm_mul_pd(lhs.value_, rhs.value_), addend.value_));
#endif
    }

    static Double2 abs(const Double2 value) {
#if defined(SIMD_NEON)
        return Double2(vabsq_f64(value.value_));
#else
        return Double2(_mm_andnot_pd(_mm_set1_pd(-0.0), value.value_));
#endif
    }

    static Double2 min(const Double2 lhs, const Double2 rhs) {
#if defined(SIMD_NEON)
        return Double2(vminq_f64(lhs.value_, rhs.value_));
#else
        return Double2(_mm_min_pd(lhs.value_, rhs.value_));
#endif
    }

    static Double2 max(const Double2 lhs, const Double2 rhs) {
#if defined(SIMD_NEON)
        return Double2(vmaxq_f64(lhs.value_, rhs.value_));
#else
        return Double2(_mm_max_pd(lhs.value_, rhs.value_));
#endif
    }

private:
    native_type value_;
};

inline Double2 widen_low(const Float4 value) {
#if defined(SIMD_NEON)
    return Double2(vcvt_f64_f32(vget_low_f32(value.native())));
#else
    return Double2(_mm_cvtps_pd(value.native()));
#endif
}

inline Double2 load_widened(const float* values) {
#if defined(SIMD_NEON)
    return Double2(vcvt_f64_f32(vld1_f32(values)));
#else
    const __m128 packed =
        _mm_castsi128_ps(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(values)));
    return Double2(_mm_cvtps_pd(packed));
#endif
}

inline Double2 widen_high(const Float4 value) {
#if defined(SIMD_NEON)
    return Double2(vcvt_f64_f32(vget_high_f32(value.native())));
#else
    return Double2(_mm_cvtps_pd(_mm_movehl_ps(value.native(), value.native())));
#endif
}

inline Float4 narrow(const Double2 low, const Double2 high) {
#if defined(SIMD_NEON)
    return Float4(vcombine_f32(vcvt_f32_f64(low.native()), vcvt_f32_f64(high.native())));
#else
    return Float4(_mm_movelh_ps(_mm_cvtpd_ps(low.native()), _mm_cvtpd_ps(high.native())));
#endif
}

inline Mask32x4 operator>(const Float4 lhs, const Float4 rhs) {
#if defined(SIMD_NEON)
    return Mask32x4(vcgtq_f32(lhs.native(), rhs.native()));
#else
    return Mask32x4(_mm_cmpgt_ps(lhs.native(), rhs.native()));
#endif
}

inline Mask32x4 operator>=(const Float4 lhs, const Float4 rhs) {
#if defined(SIMD_NEON)
    return Mask32x4(vcgeq_f32(lhs.native(), rhs.native()));
#else
    return Mask32x4(_mm_cmpge_ps(lhs.native(), rhs.native()));
#endif
}

inline Mask32x4 operator<(const Float4 lhs, const Float4 rhs) {
    return rhs > lhs;
}

inline Mask32x4 operator<=(const Float4 lhs, const Float4 rhs) {
    return rhs >= lhs;
}

inline Mask32x4 operator==(const Float4 lhs, const Float4 rhs) {
#if defined(SIMD_NEON)
    return Mask32x4(vceqq_f32(lhs.native(), rhs.native()));
#else
    return Mask32x4(_mm_cmpeq_ps(lhs.native(), rhs.native()));
#endif
}

inline Mask64x2 operator>(const Double2 lhs, const Double2 rhs) {
#if defined(SIMD_NEON)
    return Mask64x2(vcgtq_f64(lhs.native(), rhs.native()));
#else
    return Mask64x2(_mm_cmpgt_pd(lhs.native(), rhs.native()));
#endif
}

inline Mask64x2 operator>=(const Double2 lhs, const Double2 rhs) {
#if defined(SIMD_NEON)
    return Mask64x2(vcgeq_f64(lhs.native(), rhs.native()));
#else
    return Mask64x2(_mm_cmpge_pd(lhs.native(), rhs.native()));
#endif
}

inline Mask64x2 operator<(const Double2 lhs, const Double2 rhs) {
    return rhs > lhs;
}

inline Mask64x2 operator<=(const Double2 lhs, const Double2 rhs) {
    return rhs >= lhs;
}

inline Mask64x2 operator==(const Double2 lhs, const Double2 rhs) {
#if defined(SIMD_NEON)
    return Mask64x2(vceqq_f64(lhs.native(), rhs.native()));
#else
    return Mask64x2(_mm_cmpeq_pd(lhs.native(), rhs.native()));
#endif
}

inline Float4 select(const Mask32x4 mask, const Float4 on_true, const Float4 on_false) {
#if defined(SIMD_NEON)
    return Float4(vbslq_f32(mask.native(), on_true.native(), on_false.native()));
#else
    return Float4(_mm_or_ps(_mm_and_ps(mask.native(), on_true.native()), _mm_andnot_ps(mask.native(), on_false.native())));
#endif
}

inline Double2 select(const Mask64x2 mask, const Double2 on_true, const Double2 on_false) {
#if defined(SIMD_NEON)
    return Double2(vbslq_f64(mask.native(), on_true.native(), on_false.native()));
#else
    return Double2(_mm_or_pd(_mm_and_pd(mask.native(), on_true.native()), _mm_andnot_pd(mask.native(), on_false.native())));
#endif
}

inline Mask32x4 is_finite(const Float4 value) {
    return (value == value) & (Float4::abs(value) <= Float4(std::numeric_limits<float>::max()));
}

inline Mask64x2 is_finite(const Double2 value) {
    return (value == value) & (Double2::abs(value) <= Double2(std::numeric_limits<double>::max()));
}

static_assert(sizeof(Float4) == 16U);
static_assert(sizeof(Double2) == 16U);
static_assert(sizeof(Int4) == 16U);
static_assert(alignof(Float4) == alignment);
static_assert(alignof(Double2) == alignment);
static_assert(alignof(Int4) == alignment);

}  // namespace simd

