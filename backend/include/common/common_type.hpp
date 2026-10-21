#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <opencv2/core/cvdef.h>
#include <string>
#include <type_traits>

namespace sapphire::backend {

using ParametersMap = std::map<std::string, std::string, std::less<>>;

template <int R, int C, typename Scalar, int Options = Eigen::Matrix<Scalar, R, C>::Options>
using MatRC_t = Eigen::Matrix<Scalar, R, C, Options>;

template <int R, typename Scalar>
using VecR_t = Eigen::Matrix<Scalar, R, 1>;

template <typename Scalar, int Options = Eigen::Matrix<Scalar, 2, 2>::Options>
using Mat22_t = MatRC_t<2, 2, Scalar, Options>;

template <typename Scalar, int Options = Eigen::Matrix<Scalar, 3, 3>::Options>
using Mat33_t = MatRC_t<3, 3, Scalar, Options>;

template <typename Scalar, int Options = Eigen::Matrix<Scalar, 4, 4>::Options>
using Mat44_t = MatRC_t<4, 4, Scalar, Options>;

template <typename Scalar, int Options = Eigen::Matrix<Scalar, 6, 6>::Options>
using Mat66_t = MatRC_t<6, 6, Scalar, Options>;

template <typename Scalar, int Options = Eigen::Matrix<Scalar, 3, 4>::Options>
using Mat34_t = MatRC_t<3, 4, Scalar, Options>;

template <typename Scalar, int Options = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Options>
using MatX_t = MatRC_t<Eigen::Dynamic, Eigen::Dynamic, Scalar, Options>;

template <typename Scalar>
using Mat34RowMajor_t = Mat34_t<Scalar, Eigen::RowMajor>;

template <typename Scalar>
using Vec2_t = VecR_t<2, Scalar>;

template <typename Scalar>
using Vec3_t = VecR_t<3, Scalar>;

template <typename Scalar>
using Vec4_t = VecR_t<4, Scalar>;

template <typename Scalar>
using Vec5_t = VecR_t<5, Scalar>;

template <typename Scalar>
using Vec6_t = VecR_t<6, Scalar>;

template <typename Scalar>
using Vec7_t = VecR_t<7, Scalar>;

template <typename Scalar>
using Vec9_t = VecR_t<9, Scalar>;

template <typename Scalar>
using VecX_t = VecR_t<Eigen::Dynamic, Scalar>;

template <typename Scalar>
using Quat_t = Eigen::Quaternion<Scalar>;

// Representation and metric are independent of how features were extracted.
// Matching gates and archive versions belong to their respective consumers.
enum class DistanceMetric { hamming, l2 };

struct Binary_U8 {};
struct Float_F16 {};
struct Float_F32 {};

template <class Format>
struct descriptor_traits;

template <>
struct descriptor_traits<Binary_U8> {
  using scalar = uint8_t;
  static constexpr int dimensions = 32, cv_type = CV_8UC1;
  static constexpr DistanceMetric metric = DistanceMetric::hamming;
};

template <>
struct descriptor_traits<Float_F16> {
  using scalar = cv::float16_t;
  static constexpr int dimensions = 256, cv_type = CV_16FC1;
  static constexpr DistanceMetric metric = DistanceMetric::l2;
};

template <>
struct descriptor_traits<Float_F32> {
  using scalar = float;
  static constexpr int dimensions = 256, cv_type = CV_32FC1;
  static constexpr DistanceMetric metric = DistanceMetric::l2;
};

// Dimension is part of the compiled format; the original public tags keep
// their 256-dimensional defaults for existing models and archives.
template <class Precision, int Dimensions>
struct descriptor_format {};
template <class Precision, int Dimensions>
struct descriptor_traits<descriptor_format<Precision, Dimensions>> : descriptor_traits<Precision> {
  static_assert((std::is_same_v<Precision, Float_F16> || std::is_same_v<Precision, Float_F32>) && (Dimensions == 64 || Dimensions == 256));
  static constexpr int dimensions = Dimensions;
};
using Float64_F32 = descriptor_format<Float_F32, 64>;
using Float64_F16 = descriptor_format<Float_F16, 64>;
template <class Format>
using descriptor_storage = std::array<typename descriptor_traits<Format>::scalar, descriptor_traits<Format>::dimensions>;

}  // namespace sapphire::backend
