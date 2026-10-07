#pragma once

#include <array>
#include <cassert>
#include <cstddef>
#include <limits>
#include <new>
#include <type_traits>
#include <utility>
#include <vector>

#include "tools/simd_sse_neon.h"

namespace simd {

template <typename T>
inline constexpr std::size_t packet_width = std::is_same_v<T, float> ? 4U : 2U;

template <typename T>
constexpr std::size_t floor_packet(const std::size_t size) {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>);
  return size - size % packet_width<T>;
}

template <typename T>
constexpr std::size_t ceil_packet(const std::size_t size) {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>);
  return (size + packet_width<T> - 1U) / packet_width<T> * packet_width<T>;
}

inline void widen_batch(const float* input, double* output, const std::size_t count) noexcept {
  std::size_t index = 0U;
  const std::size_t packed_size = floor_packet<float>(count);
  for (; index < packed_size; index += packet_width<float>) {
    const Float4 packed = Float4::load_aligned(input + index);
    widen_low(packed).store_aligned(output + index);
    widen_high(packed).store_aligned(output + index + packet_width<double>);
  }
  for (; index < count; ++index) {
    output[index] = static_cast<double>(input[index]);
  }
}

inline void narrow_batch(const double* input, float* output, const std::size_t count) noexcept {
  std::size_t index = 0U;
  const std::size_t packed_size = floor_packet<float>(count);
  for (; index < packed_size; index += packet_width<float>) {
    narrow(Double2::load_aligned(input + index), Double2::load_aligned(input + index + packet_width<double>)).store_aligned(output + index);
  }
  for (; index < count; ++index) {
    output[index] = static_cast<float>(input[index]);
  }
}

template <typename T>
class AlignedAllocator {
 public:
  using value_type = T;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;

  AlignedAllocator() noexcept = default;

  template <typename U>
  AlignedAllocator(const AlignedAllocator<U>&) noexcept {}

  [[nodiscard]] T* allocate(const std::size_t count) {
    if (count > std::numeric_limits<std::size_t>::max() / sizeof(T)) {
      throw std::bad_array_new_length();
    }
    return static_cast<T*>(::operator new(count * sizeof(T), std::align_val_t(alignment)));
  }

  void deallocate(T* memory, std::size_t) noexcept { ::operator delete(memory, std::align_val_t(alignment)); }

  template <typename U>
  bool operator==(const AlignedAllocator<U>&) const noexcept {
    return true;
  }

  template <typename U>
  bool operator!=(const AlignedAllocator<U>&) const noexcept {
    return false;
  }
};

template <typename T>
using aligned_vector = std::vector<T, AlignedAllocator<T>>;

template <typename T>
class SoA2 {
 public:
  void clear() {
    x.clear();
    y.clear();
  }

  void reserve(const std::size_t size) {
    x.reserve(size);
    y.reserve(size);
  }

  void resize(const std::size_t size) {
    x.resize(size);
    y.resize(size);
  }

  void push_back(const T x_value, const T y_value) {
    x.push_back(x_value);
    y.push_back(y_value);
  }

  std::size_t size() const {
    assert(x.size() == y.size());
    return x.size();
  }

  bool empty() const { return size() == 0U; }

  void append(const SoA2<T>& values) {
    x.insert(x.end(), values.x.begin(), values.x.end());
    y.insert(y.end(), values.y.begin(), values.y.end());
  }

  aligned_vector<T> x;
  aligned_vector<T> y;
};

template <typename T>
class SoA3 {
 public:
  void clear() {
    x.clear();
    y.clear();
    z.clear();
  }

  void reserve(const std::size_t size) {
    x.reserve(size);
    y.reserve(size);
    z.reserve(size);
  }

  void resize(const std::size_t size) {
    x.resize(size);
    y.resize(size);
    z.resize(size);
  }

  void push_back(const T x_value, const T y_value, const T z_value) {
    x.push_back(x_value);
    y.push_back(y_value);
    z.push_back(z_value);
  }

  std::size_t size() const {
    assert(x.size() == y.size() && x.size() == z.size());
    return x.size();
  }

  bool empty() const { return size() == 0U; }

  void set(const std::size_t index, const T x_value, const T y_value, const T z_value) {
    x[index] = x_value;
    y[index] = y_value;
    z[index] = z_value;
  }

  void append(const SoA3<T>& values) {
    x.insert(x.end(), values.x.begin(), values.x.end());
    y.insert(y.end(), values.y.begin(), values.y.end());
    z.insert(z.end(), values.z.begin(), values.z.end());
  }

  aligned_vector<T> x;
  aligned_vector<T> y;
  aligned_vector<T> z;
};

inline void set(const std::size_t size, const float value, float* output) {
  const std::size_t packed_size = floor_packet<float>(size);
  const Float4 packet(value);
  std::size_t index = 0U;
  for (; index < packed_size; index += packet_width<float>) {
    packet.store(output + index);
  }
  for (; index < size; ++index) {
    output[index] = value;
  }
}

inline void set(const std::size_t size, const double value, double* output) {
  const std::size_t packed_size = floor_packet<double>(size);
  const Double2 packet(value);
  std::size_t index = 0U;
  for (; index < packed_size; index += packet_width<double>) {
    packet.store(output + index);
  }
  for (; index < size; ++index) {
    output[index] = value;
  }
}

inline void multiply_add(const std::size_t size, const float scale, const float* input, float* output) {
  const std::size_t packed_size = floor_packet<float>(size);
  const Float4 scale_packet(scale);
  std::size_t index = 0U;
  for (; index < packed_size; index += packet_width<float>) {
    Float4::multiply_add(Float4::load(input + index), scale_packet, Float4::load(output + index)).store(output + index);
  }
  for (; index < size; ++index) {
    output[index] += scale * input[index];
  }
}

inline void multiply_add(const std::size_t size, const double scale, const double* input, double* output) {
  const std::size_t packed_size = floor_packet<double>(size);
  const Double2 scale_packet(scale);
  std::size_t index = 0U;
  for (; index < packed_size; index += packet_width<double>) {
    Double2::multiply_add(Double2::load(input + index), scale_packet, Double2::load(output + index)).store(output + index);
  }
  for (; index < size; ++index) {
    output[index] += scale * input[index];
  }
}

inline float dot(const std::size_t size, const float* lhs, const float* rhs) {
  const std::size_t packed_size = floor_packet<float>(size);
  Float4 sum(0.0F);
  std::size_t index = 0U;
  for (; index < packed_size; index += packet_width<float>) {
    sum += Float4::load(lhs + index) * Float4::load(rhs + index);
  }
  float result = sum.sum();
  for (; index < size; ++index) {
    result += lhs[index] * rhs[index];
  }
  return result;
}

inline double dot(const std::size_t size, const double* lhs, const double* rhs) {
  const std::size_t packed_size = floor_packet<double>(size);
  Double2 sum(0.0);
  std::size_t index = 0U;
  for (; index < packed_size; index += packet_width<double>) {
    sum += Double2::load(lhs + index) * Double2::load(rhs + index);
  }
  double result = sum.sum();
  for (; index < size; ++index) {
    result += lhs[index] * rhs[index];
  }
  return result;
}

inline void normalize(const Float4 x, const Float4 y, const Float4 z, Float4& normalized_x, Float4& normalized_y, Float4& normalized_z) {
  const Float4 inverse_norm = Float4(1.0F) / Float4::sqrt(x * x + y * y + z * z);
  normalized_x = x * inverse_norm;
  normalized_y = y * inverse_norm;
  normalized_z = z * inverse_norm;
}

inline void normalize(const Double2 x, const Double2 y, const Double2 z, Double2& normalized_x, Double2& normalized_y, Double2& normalized_z) {
  const Double2 inverse_norm = Double2(1.0) / Double2::sqrt(x * x + y * y + z * z);
  normalized_x = x * inverse_norm;
  normalized_y = y * inverse_norm;
  normalized_z = z * inverse_norm;
}

inline void transform(const std::array<float, 9>& rotation, const std::array<float, 3>& translation, const Float4 x, const Float4 y, const Float4 z,
                      Float4& transformed_x, Float4& transformed_y, Float4& transformed_z) {
  transformed_x = Float4::multiply_add(Float4(rotation[0]), x,
                                       Float4::multiply_add(Float4(rotation[1]), y, Float4::multiply_add(Float4(rotation[2]), z, Float4(translation[0]))));
  transformed_y = Float4::multiply_add(Float4(rotation[3]), x,
                                       Float4::multiply_add(Float4(rotation[4]), y, Float4::multiply_add(Float4(rotation[5]), z, Float4(translation[1]))));
  transformed_z = Float4::multiply_add(Float4(rotation[6]), x,
                                       Float4::multiply_add(Float4(rotation[7]), y, Float4::multiply_add(Float4(rotation[8]), z, Float4(translation[2]))));
}

inline void transform(const std::array<double, 9>& rotation, const std::array<double, 3>& translation, const Double2 x, const Double2 y, const Double2 z,
                      Double2& transformed_x, Double2& transformed_y, Double2& transformed_z) {
  transformed_x = Double2::multiply_add(
      Double2(rotation[0]), x, Double2::multiply_add(Double2(rotation[1]), y, Double2::multiply_add(Double2(rotation[2]), z, Double2(translation[0]))));
  transformed_y = Double2::multiply_add(
      Double2(rotation[3]), x, Double2::multiply_add(Double2(rotation[4]), y, Double2::multiply_add(Double2(rotation[5]), z, Double2(translation[1]))));
  transformed_z = Double2::multiply_add(
      Double2(rotation[6]), x, Double2::multiply_add(Double2(rotation[7]), y, Double2::multiply_add(Double2(rotation[8]), z, Double2(translation[2]))));
}

}  // namespace simd
