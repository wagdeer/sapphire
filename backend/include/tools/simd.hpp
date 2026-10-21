#pragma once

#include <array>
#include <cassert>
#include <cstddef>
#include <limits>
#include <new>
#include <type_traits>
#include <utility>
#include <vector>

#include "backend/tools/simd_sse_neon.hpp"

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

template <class ISA_TYPE = Native>
class Kernel {
  static_assert(isa_available<ISA_TYPE>, "SIMD ISA is unavailable on this target");
  template <class T>
  struct PacketType {
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>, "SIMD packets require float or double");
    using type = std::conditional_t<std::is_same_v<ISA_TYPE, Scalar>, ScalarPacket<T>, std::conditional_t<std::is_same_v<T, float>, Float8, Double2>>;
  };

 public:
  using isa_type = ISA_TYPE;
  template <class T>
  using Packet = typename PacketType<T>::type;
  template <class T>
  static constexpr std::size_t width = Packet<T>::width;

  template <class T>
  void set(std::size_t size, T value, T* output) const {
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>);
    const Packet<T> v(value);
    const auto packed = size - size % width<T>;
    std::size_t i = 0;
    for (; i < packed; i += width<T>) v.store(output + i);
    for (; i < size; ++i) output[i] = value;
  }
  template <class T>
  void multiply_add(std::size_t size, T scale, const T* input, T* output) const {
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>);
    const Packet<T> v(scale);
    const auto packed = size - size % width<T>;
    std::size_t i = 0;
    for (; i < packed; i += width<T>) Packet<T>::multiply_add(Packet<T>::load(input + i), v, Packet<T>::load(output + i)).store(output + i);
    for (; i < size; ++i) output[i] += scale * input[i];
  }
  template <class T>
  T dot(std::size_t size, const T* lhs, const T* rhs) const {
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>);
    Packet<T> total(T(0));
    const auto packed = size - size % width<T>;
    std::size_t i = 0;
    for (; i < packed; i += width<T>) total += Packet<T>::load(lhs + i) * Packet<T>::load(rhs + i);
    T result = total.sum();
    for (; i < size; ++i) result += lhs[i] * rhs[i];
    return result;
  }
  Hamming256<ISA_TYPE> hamming256(const std::uint8_t* query) const { return Hamming256<ISA_TYPE>(query); }
};

// Preserve existing callers while routing bulk arithmetic through one ISA-selected kernel.
inline void set(std::size_t n, float value, float* out) { Kernel<>().set(n, value, out); }
inline void set(std::size_t n, double value, double* out) { Kernel<>().set(n, value, out); }
inline void multiply_add(std::size_t n, float scale, const float* in, float* out) { Kernel<>().multiply_add(n, scale, in, out); }
inline void multiply_add(std::size_t n, double scale, const double* in, double* out) { Kernel<>().multiply_add(n, scale, in, out); }
inline float dot(std::size_t n, const float* lhs, const float* rhs) { return Kernel<>().dot(n, lhs, rhs); }
inline double dot(std::size_t n, const double* lhs, const double* rhs) { return Kernel<>().dot(n, lhs, rhs); }

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
