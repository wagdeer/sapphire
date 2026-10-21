#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>
#include <stdexcept>

#include "distance.hpp"

namespace sapphire::backend::feature {
template <class Format>
class landmark_descriptors;
template <class Format, class Range, class Projector>
class descriptor_range;
// An appearance value owns its metric and model domain. Binary access fails
// closed for learned descriptors; no reinterpretation as ORB is permitted.
// Copies share immutable learned storage. Binary points retain inline bytes.
class LocalDescriptor {
 public:
  using Binary = descriptor_storage<Binary_U8>;
  using Learned = descriptor_storage<Float_F32>;
  LocalDescriptor() = default;
  LocalDescriptor(const Binary& bytes) : binary_(bytes) {}
  LocalDescriptor& operator=(const Binary& bytes) {
    binary_ = bytes;
    floating_.reset();
    model_ = 0;
    dimensions_ = 32;
    return *this;
  }
  static LocalDescriptor from_row(const cv::Mat& matrix, int row, uint64_t model) {
    if (row < 0 || row >= matrix.rows) throw std::invalid_argument("Descriptor row out of bounds");
    LocalDescriptor value;
    if (!model) {
      if (matrix.type() != CV_8UC1 || matrix.cols != 32) throw std::invalid_argument("Invalid ORB descriptor row");
      std::copy_n(matrix.ptr<uint8_t>(row), 32, value.binary_.begin());
    } else {
      if (matrix.type() != CV_32FC1 || (matrix.cols != 64 && matrix.cols != 256)) throw std::invalid_argument("Invalid learned descriptor row");
      value = matrix.cols == 64 ? from_learned_values<64>(matrix.ptr<float>(row), model) : from_learned_values<256>(matrix.ptr<float>(row), model);
    }
    return value;
  }
  template <size_t N>
  static LocalDescriptor from_learned(const std::array<float, N>& data, uint64_t model) {
    return from_learned_values<N>(data.data(), model);
  }
  uint64_t model_id() const { return model_; }
  DistanceMetric metric() const { return floating_ ? DistanceMetric::l2 : DistanceMetric::hamming; }
  int dimensions() const { return dimensions_; }
  struct FloatView {
    const float* values;
    const float* data() const { return values; }
    float operator[](size_t i) const { return values[i]; }
  };
  FloatView floating() const {
    if (!floating_) throw std::logic_error("Binary descriptor has no learned values");
    return {floating_.get()};
  }
  const Binary& binary() const {
    require_binary();
    return binary_;
  }
  const uint8_t* data() const {
    require_binary();
    return binary_.data();
  }
  uint8_t* data() {
    require_binary();
    return binary_.data();
  }
  auto begin() {
    require_binary();
    return binary_.begin();
  }
  auto end() {
    require_binary();
    return binary_.end();
  }
  auto begin() const {
    require_binary();
    return binary_.begin();
  }
  auto end() const {
    require_binary();
    return binary_.end();
  }
  size_t size() const {
    require_binary();
    return binary_.size();
  }
  uint8_t& operator[](size_t row) {
    require_binary();
    return binary_.at(row);
  }
  const uint8_t& operator[](size_t row) const {
    require_binary();
    return binary_.at(row);
  }
  void fill(uint8_t byte) {
    require_binary();
    binary_.fill(byte);
  }
  size_t dynamic_bytes() const { return floating_ ? dimensions_ * sizeof(float) + 2 * sizeof(void*) : 0; }
  bool operator==(const LocalDescriptor& other) const { return model_ == other.model_ && dimensions_ == other.dimensions_ && (floating_ ? std::equal(floating_.get(), floating_.get() + dimensions_, other.floating_.get()) : binary_ == other.binary_); }
  bool operator!=(const LocalDescriptor& other) const { return !(*this == other); }
  float distance(const LocalDescriptor& other) const {
    if (model_ != other.model_ || dimensions_ != other.dimensions_) throw std::invalid_argument("Descriptor model mismatch");
    if (floating_) return std::sqrt(cv::hal::normL2Sqr_(floating_.get(), other.floating_.get(), dimensions_));
    return float(descriptor_distance<Binary_U8>(row_unchecked<Binary_U8>()).distance(other.row_unchecked<Binary_U8>()));
  }
  void copy_to(cv::Mat& matrix, int row) const {
    if (row < 0 || row >= matrix.rows || matrix.type() != (model_ ? CV_32FC1 : CV_8UC1) || matrix.cols != dimensions_)
      throw std::invalid_argument("Descriptor destination layout mismatch");
    if (floating_)
      std::copy_n(floating_.get(), dimensions_, matrix.ptr<float>(row));
    else
      std::copy(binary_.begin(), binary_.end(), matrix.ptr<uint8_t>(row));
  }

 private:
  template <size_t N>
  static LocalDescriptor from_learned_values(const float* data, uint64_t model) {
    static_assert(N == 64 || N == 256);
    if (!model) throw std::invalid_argument("Learned descriptor requires model identity");
    // Validate and copy directly into the final allocation, without a 1 KiB
    // stack array followed by a second copy into shared storage.
    auto storage = std::make_shared<std::array<float, N>>();
    double norm = 0;
    for (size_t i = 0; i < storage->size(); ++i) {
      const float v = data[i];
      if (!std::isfinite(v)) throw std::invalid_argument("Nonfinite learned descriptor");
      norm += double(v) * v;
      (*storage)[i] = v;
    }
    if (std::abs(std::sqrt(norm) - 1) > 1e-3) throw std::invalid_argument("Learned descriptor must be normalized");
    LocalDescriptor value;
    value.model_ = model;
    value.dimensions_ = N;
    value.floating_ = std::shared_ptr<const float>(storage, storage->data());
    return value;
  }
  template <class Format>
  friend class landmark_descriptors;
  template <class Format, class Range, class Projector>
  friend class descriptor_range;
  template <class Format>
  descriptor_row<Format> row_unchecked() const noexcept {
    static_assert(std::is_same_v<Format, Binary_U8> || std::is_same_v<typename descriptor_traits<Format>::scalar, float>, "Map precision requires an explicit storage schema");
    if constexpr (std::is_same_v<Format, Binary_U8>)
      return descriptor_row<Format>(binary_.data());
    else
      return descriptor_row<Format>(floating_.get());
  }
  void require_binary() const {
    if (floating_) throw std::logic_error("Learned descriptor cannot enter binary operation");
  }
  Binary binary_{};
  std::shared_ptr<const float> floating_;
  uint16_t dimensions_ = 32;
  uint64_t model_ = 0;
};
}  // namespace sapphire::backend::feature
