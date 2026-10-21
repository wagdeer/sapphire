#pragma once

#include <backend/common/common_type.hpp>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>

namespace sapphire::backend {

template <typename T>
class StateGroup {
  static_assert(std::is_floating_point_v<T>, "StateGroup scalar must be floating point");

 public:
  using Scalar = T;
  using Coefficients = std::array<T, 12>;
  using Vector3 = VecR_t<3, T>;
  using Tangent6 = VecR_t<6, T>;
  using Matrix3 = MatRC_t<3, 3, T>;
  using Matrix4 = MatRC_t<4, 4, T>;
  using Matrix6 = MatRC_t<6, 6, T>;
  using Matrix34 = MatRC_t<3, 4, T, Eigen::RowMajor>;

  static constexpr std::size_t kCoefficientCount = 12;

  constexpr StateGroup() noexcept = default;

  explicit constexpr StateGroup(const Coefficients& coefficients) noexcept : coefficients_(coefficients) {}

  StateGroup(T r11, T r12, T r13, T tx, T r21, T r22, T r23, T ty, T r31, T r32, T r33, T tz) noexcept
      : coefficients_{r11, r12, r13, tx, r21, r22, r23, ty, r31, r32, r33, tz} {}

  explicit StateGroup(const Matrix34& matrix) noexcept { Eigen::Map<Matrix34>(coefficients_.data()) = matrix; }

  explicit StateGroup(const Matrix4& matrix) noexcept {
    const Matrix34 compact = matrix.template block<3, 4>(0, 0);
    Eigen::Map<Matrix34>(coefficients_.data()) = compact;
  }

  template <typename U>
  explicit StateGroup(const StateGroup<U>& other) noexcept {
    for (std::size_t index = 0; index < kCoefficientCount; ++index) {
      coefficients_[index] = static_cast<T>(other.coefficients()[index]);
    }
  }

  [[nodiscard]] static StateGroup identity() noexcept {
    return StateGroup(T(1), T(0), T(0), T(0), T(0), T(1), T(0), T(0), T(0), T(0), T(1), T(0));
  }

  [[nodiscard]] static StateGroup fromRotationTranslation(const Matrix3& rotation, const Vector3& translation) noexcept {
    Matrix34 matrix;
    matrix.template block<3, 3>(0, 0) = rotation;
    matrix.col(3) = translation;
    return StateGroup(matrix);
  }

  [[nodiscard]] static StateGroup fromTranslation(const Vector3& translation) noexcept {
    return fromRotationTranslation(Matrix3::Identity(), translation);
  }

  [[nodiscard]] const Coefficients& coefficients() const noexcept { return coefficients_; }

  [[nodiscard]] T x() const noexcept { return coefficients_[3]; }
  [[nodiscard]] T y() const noexcept { return coefficients_[7]; }
  [[nodiscard]] T z() const noexcept { return coefficients_[11]; }

  [[nodiscard]] T rotation(std::size_t row, std::size_t column) const noexcept { return coefficients_[row * 4 + column]; }

  [[nodiscard]] bool isNull() const noexcept {
    bool allZero = true;
    for (const T coefficient : coefficients_) {
      if (!std::isfinite(coefficient)) {
        return true;
      }
      allZero = allZero && coefficient == T(0);
    }
    return allZero;
  }

  [[nodiscard]] bool isIdentity() const noexcept { return coefficients_ == identity().coefficients_; }

  [[nodiscard]] bool isValid(T tolerance = defaultTolerance()) const noexcept {
    if (isNull()) {
      return false;
    }
    const Matrix3 rotation = rotationMatrix();
    return rotation.allFinite() && translationVector().allFinite() &&
           (rotation.transpose() * rotation - Matrix3::Identity()).cwiseAbs().maxCoeff() <= tolerance &&
           std::abs(rotation.determinant() - T(1)) <= tolerance;
  }

  [[nodiscard]] Matrix34 matrix34() const noexcept { return Eigen::Map<const Matrix34>(coefficients_.data()); }

  [[nodiscard]] Matrix4 matrix() const noexcept {
    Matrix4 matrix = Matrix4::Identity();
    matrix.template block<3, 4>(0, 0) = matrix34();
    return matrix;
  }

  [[nodiscard]] Matrix3 rotationMatrix() const noexcept { return matrix34().template block<3, 3>(0, 0); }

  [[nodiscard]] Vector3 translationVector() const noexcept {
    return Vector3(coefficients_[3], coefficients_[7], coefficients_[11]);
  }

  [[nodiscard]] StateGroup withTranslation(const Vector3& translation) const noexcept {
    if (isNull()) {
      return {};
    }
    return fromRotationTranslation(rotationMatrix(), translation);
  }

  [[nodiscard]] StateGroup scaledTranslation(T scale) const noexcept {
    return withTranslation(scale * translationVector());
  }

  [[nodiscard]] StateGroup normalized() const noexcept {
    if (isNull()) {
      return {};
    }
    const Eigen::JacobiSVD<Matrix3> svd(rotationMatrix(), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix3 diagonal = Matrix3::Identity();
    diagonal(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
    return fromRotationTranslation(svd.matrixU() * diagonal * svd.matrixV().transpose(), translationVector());
  }

  [[nodiscard]] StateGroup compose(const StateGroup& other) const noexcept {
    if (isNull() || other.isNull()) {
      return {};
    }
    const Matrix3 rotation = rotationMatrix();
    return fromRotationTranslation(rotation * other.rotationMatrix(), rotation * other.translationVector() + translationVector());
  }

  [[nodiscard]] StateGroup inverse() const noexcept {
    if (isNull()) {
      return {};
    }
    const Matrix3 inverseRotation = rotationMatrix().transpose();
    return fromRotationTranslation(inverseRotation, -(inverseRotation * translationVector()));
  }

  [[nodiscard]] StateGroup between(const StateGroup& other) const noexcept { return inverse().compose(other); }

  [[nodiscard]] StateGroup conjugatedBy(const StateGroup& frame) const noexcept {
    return frame.compose(*this).compose(frame.inverse());
  }

  [[nodiscard]] StateGroup rotationPart() const noexcept {
    return isNull() ? StateGroup() : fromRotationTranslation(rotationMatrix(), Vector3::Zero());
  }

  [[nodiscard]] StateGroup translationPart() const noexcept {
    return isNull() ? StateGroup() : fromTranslation(translationVector());
  }

  [[nodiscard]] StateGroup projectPlanar(bool keepZ = false) const noexcept {
    if (isNull()) {
      return {};
    }
    const T heading = std::atan2(rotation(1, 0), rotation(0, 0));
    const T cosine = std::cos(heading);
    const T sine = std::sin(heading);
    const Matrix3 planarRotation = (Matrix3() << cosine, -sine, T(0), sine, cosine, T(0), T(0), T(0), T(1)).finished();
    Vector3 translation = translationVector();
    if (!keepZ) {
      translation.z() = T(0);
    }
    return fromRotationTranslation(planarRotation, translation);
  }

  [[nodiscard]] Vector3 transformPoint(const Vector3& point) const noexcept {
    if (isNull()) {
      return Vector3::Constant(std::numeric_limits<T>::quiet_NaN());
    }
    return rotationMatrix() * point + translationVector();
  }

  void transformPoint(T px, T py, T pz, T& outputX, T& outputY, T& outputZ) const noexcept {
    if (isNull()) {
      outputX = outputY = outputZ = std::numeric_limits<T>::quiet_NaN();
      return;
    }
    outputX = rotation(0, 0) * px + rotation(0, 1) * py + rotation(0, 2) * pz + x();
    outputY = rotation(1, 0) * px + rotation(1, 1) * py + rotation(1, 2) * pz + y();
    outputZ = rotation(2, 0) * px + rotation(2, 1) * py + rotation(2, 2) * pz + z();
  }

  [[nodiscard]] Vector3 rotateVector(const Vector3& vector) const noexcept {
    if (isNull()) {
      return Vector3::Constant(std::numeric_limits<T>::quiet_NaN());
    }
    return rotationMatrix() * vector;
  }

  void transformPointsInPlace(T* xValues, T* yValues, T* zValues, std::size_t count) const noexcept {
    for (std::size_t index = 0; index < count; ++index) {
      transformPoint(xValues[index], yValues[index], zValues[index], xValues[index], yValues[index], zValues[index]);
    }
  }

  void rotateVectorsInPlace(T* xValues, T* yValues, T* zValues, std::size_t count) const noexcept {
    if (isNull()) {
      const T invalid = std::numeric_limits<T>::quiet_NaN();
      std::fill_n(xValues, count, invalid);
      std::fill_n(yValues, count, invalid);
      std::fill_n(zValues, count, invalid);
      return;
    }
    for (std::size_t index = 0; index < count; ++index) {
      const T x = xValues[index];
      const T y = yValues[index];
      const T z = zValues[index];
      xValues[index] = rotation(0, 0) * x + rotation(0, 1) * y + rotation(0, 2) * z;
      yValues[index] = rotation(1, 0) * x + rotation(1, 1) * y + rotation(1, 2) * z;
      zValues[index] = rotation(2, 0) * x + rotation(2, 1) * y + rotation(2, 2) * z;
    }
  }

  [[nodiscard]] T translationNorm() const noexcept {
    return isNull() ? std::numeric_limits<T>::quiet_NaN() : translationVector().norm();
  }
  [[nodiscard]] T translationSquaredNorm() const noexcept {
    return isNull() ? std::numeric_limits<T>::quiet_NaN() : translationVector().squaredNorm();
  }
  [[nodiscard]] T translationDistance(const StateGroup& other) const noexcept {
    return isNull() || other.isNull() ? std::numeric_limits<T>::quiet_NaN() : (translationVector() - other.translationVector()).norm();
  }
  [[nodiscard]] T translationSquaredDistance(const StateGroup& other) const noexcept {
    return isNull() || other.isNull() ? std::numeric_limits<T>::quiet_NaN() : (translationVector() - other.translationVector()).squaredNorm();
  }

  [[nodiscard]] T rotationDistance(const StateGroup& other) const noexcept {
    if (isNull() || other.isNull()) {
      return std::numeric_limits<T>::quiet_NaN();
    }
    return rotationLog(rotationMatrix().transpose() * other.rotationMatrix()).norm();
  }

  [[nodiscard]] StateGroup interpolate(T alpha, const StateGroup& other) const noexcept {
    if (isNull() || other.isNull()) {
      return {};
    }
    Eigen::Quaternion<T> first(rotationMatrix());
    Eigen::Quaternion<T> second(other.rotationMatrix());
    first.normalize();
    second.normalize();
    const Matrix3 rotation = first.slerp(alpha, second).normalized().toRotationMatrix();
    return fromRotationTranslation(rotation, translationVector() + alpha * (other.translationVector() - translationVector()));
  }

  [[nodiscard]] static Matrix3 wedge(const Vector3& vector) noexcept {
    Matrix3 matrix;
    matrix << T(0), -vector.z(), vector.y(), vector.z(), T(0), -vector.x(), -vector.y(), vector.x(), T(0);
    return matrix;
  }

  [[nodiscard]] static Vector3 vee(const Matrix3& matrix) noexcept {
    return Vector3(matrix(2, 1), matrix(0, 2), matrix(1, 0));
  }

  [[nodiscard]] static Matrix4 wedge(const Tangent6& tangent) noexcept {
    Matrix4 matrix = Matrix4::Zero();
    const Vector3 rotation = tangent.template tail<3>();
    matrix.template block<3, 3>(0, 0) = wedge(rotation);
    matrix.template block<3, 1>(0, 3) = tangent.template head<3>();
    return matrix;
  }

  [[nodiscard]] static Tangent6 vee(const Matrix4& matrix) noexcept {
    Tangent6 tangent;
    tangent.template head<3>() = matrix.template block<3, 1>(0, 3);
    const Matrix3 rotationAlgebra = matrix.template block<3, 3>(0, 0);
    tangent.template tail<3>() = vee(rotationAlgebra);
    return tangent;
  }

  [[nodiscard]] static Matrix3 leftJacobian(const Vector3& rotationVector) noexcept {
    const T squaredAngle = rotationVector.squaredNorm();
    const Matrix3 skew = wedge(rotationVector);
    T firstCoefficient;
    T secondCoefficient;
    if (squaredAngle < smallAngleSquared()) {
      firstCoefficient = T(0.5) - squaredAngle / T(24) + squaredAngle * squaredAngle / T(720);
      secondCoefficient = T(1.0 / 6.0) - squaredAngle / T(120) + squaredAngle * squaredAngle / T(5040);
    } else {
      const T angle = std::sqrt(squaredAngle);
      firstCoefficient = (T(1) - std::cos(angle)) / squaredAngle;
      secondCoefficient = (angle - std::sin(angle)) / (squaredAngle * angle);
    }
    return Matrix3::Identity() + firstCoefficient * skew + secondCoefficient * skew * skew;
  }

  [[nodiscard]] static Matrix3 invLeftJacobian(const Vector3& rotationVector) noexcept {
    const T squaredAngle = rotationVector.squaredNorm();
    const Matrix3 skew = wedge(rotationVector);
    T coefficient;
    if (squaredAngle < smallAngleSquared()) {
      coefficient = T(1.0 / 12.0) + squaredAngle / T(720) + squaredAngle * squaredAngle / T(30240);
    } else {
      const T angle = std::sqrt(squaredAngle);
      coefficient = (T(1) - T(0.5) * angle / std::tan(T(0.5) * angle)) / squaredAngle;
    }
    return Matrix3::Identity() - T(0.5) * skew + coefficient * skew * skew;
  }

  [[nodiscard]] static Matrix3 rightJacobian(const Vector3& rotationVector) noexcept {
    const Vector3 negative = -rotationVector;
    return leftJacobian(negative);
  }

  [[nodiscard]] static Matrix3 invRightJacobian(const Vector3& rotationVector) noexcept {
    const Vector3 negative = -rotationVector;
    return invLeftJacobian(negative);
  }

  [[nodiscard]] static Matrix6 leftJacobian(const Tangent6& tangent) noexcept {
    const Vector3 translation = tangent.template head<3>();
    const Vector3 rotation = tangent.template tail<3>();
    if (rotation.squaredNorm() < smallAngleSquared()) {
      const Matrix6 algebraAdjoint = adjointAlgebra(tangent);
      const Matrix6 squared = algebraAdjoint * algebraAdjoint;
      return Matrix6::Identity() + T(0.5) * algebraAdjoint + T(1.0 / 6.0) * squared +
             T(1.0 / 24.0) * squared * algebraAdjoint + T(1.0 / 120.0) * squared * squared;
    }
    const Matrix3 rotationJacobian = leftJacobian(rotation);
    const Matrix3 coupling = se3LeftJacobianCoupling(rotation, translation);
    Matrix6 jacobian = Matrix6::Zero();
    jacobian.template block<3, 3>(0, 0) = rotationJacobian;
    jacobian.template block<3, 3>(0, 3) = coupling;
    jacobian.template block<3, 3>(3, 3) = rotationJacobian;
    return jacobian;
  }

  [[nodiscard]] static Matrix6 invLeftJacobian(const Tangent6& tangent) noexcept {
    const Vector3 rotation = tangent.template tail<3>();
    if (rotation.squaredNorm() < smallAngleSquared()) {
      const Matrix6 algebraAdjoint = adjointAlgebra(tangent);
      const Matrix6 squared = algebraAdjoint * algebraAdjoint;
      const Matrix6 fourth = squared * squared;
      return Matrix6::Identity() - T(0.5) * algebraAdjoint + T(1.0 / 12.0) * squared - T(1.0 / 720.0) * fourth +
             T(1.0 / 30240.0) * fourth * squared;
    }
    const Vector3 translation = tangent.template head<3>();
    const Matrix3 inverseRotationJacobian = invLeftJacobian(rotation);
    const Matrix3 coupling = se3LeftJacobianCoupling(rotation, translation);
    Matrix6 inverse = Matrix6::Zero();
    inverse.template block<3, 3>(0, 0) = inverseRotationJacobian;
    inverse.template block<3, 3>(0, 3) = -inverseRotationJacobian * coupling * inverseRotationJacobian;
    inverse.template block<3, 3>(3, 3) = inverseRotationJacobian;
    return inverse;
  }

  [[nodiscard]] static Matrix6 rightJacobian(const Tangent6& tangent) noexcept {
    const Tangent6 negative = -tangent;
    return leftJacobian(negative);
  }

  [[nodiscard]] static Matrix6 invRightJacobian(const Tangent6& tangent) noexcept {
    const Tangent6 negative = -tangent;
    return invLeftJacobian(negative);
  }

  [[nodiscard]] static StateGroup rotationExp(const Vector3& rotationVector) noexcept {
    return fromRotationTranslation(rotationExpMatrix(rotationVector), Vector3::Zero());
  }

  [[nodiscard]] Vector3 rotationLog() const noexcept {
    if (isNull()) {
      return Vector3::Constant(std::numeric_limits<T>::quiet_NaN());
    }
    return rotationLog(rotationMatrix());
  }

  [[nodiscard]] static StateGroup exp(const Tangent6& tangent) noexcept {
    const Vector3 rotationVector = tangent.template tail<3>();
    return fromRotationTranslation(rotationExpMatrix(rotationVector), leftJacobian(rotationVector) * tangent.template head<3>());
  }

  [[nodiscard]] Tangent6 log() const noexcept {
    if (isNull()) {
      return Tangent6::Constant(std::numeric_limits<T>::quiet_NaN());
    }
    Tangent6 tangent;
    tangent.template tail<3>() = rotationLog();
    const Vector3 rotationVector = tangent.template tail<3>();
    tangent.template head<3>() = invLeftJacobian(rotationVector) * translationVector();
    return tangent;
  }

  [[nodiscard]] Matrix6 adjoint() const noexcept {
    if (isNull()) {
      return Matrix6::Constant(std::numeric_limits<T>::quiet_NaN());
    }
    const Matrix3 rotation = rotationMatrix();
    Matrix6 result = Matrix6::Zero();
    result.template block<3, 3>(0, 0) = rotation;
    result.template block<3, 3>(0, 3) = wedge(translationVector()) * rotation;
    result.template block<3, 3>(3, 3) = rotation;
    return result;
  }

  [[nodiscard]] std::string debugString() const {
    if (isNull()) {
      return "[null]";
    }
    std::ostringstream stream;
    stream << matrix34();
    return stream.str();
  }

  [[nodiscard]] bool operator==(const StateGroup& other) const noexcept { return coefficients_ == other.coefficients_; }
  [[nodiscard]] bool operator!=(const StateGroup& other) const noexcept { return !(*this == other); }

 private:
  [[nodiscard]] static constexpr T defaultTolerance() noexcept {
    return std::is_same_v<T, float> ? T(1e-4) : T(1e-10);
  }

  [[nodiscard]] static constexpr T smallAngleSquared() noexcept {
    const T threshold = std::is_same_v<T, float> ? T(1e-2) : T(1e-4);
    return threshold * threshold;
  }

  [[nodiscard]] static Matrix3 rotationExpMatrix(const Vector3& rotationVector) noexcept {
    const T squaredAngle = rotationVector.squaredNorm();
    const Matrix3 skew = wedge(rotationVector);
    T sineCoefficient;
    T cosineCoefficient;
    if (squaredAngle < smallAngleSquared()) {
      sineCoefficient = T(1) - squaredAngle / T(6) + squaredAngle * squaredAngle / T(120);
      cosineCoefficient = T(0.5) - squaredAngle / T(24) + squaredAngle * squaredAngle / T(720);
    } else {
      const T angle = std::sqrt(squaredAngle);
      sineCoefficient = std::sin(angle) / angle;
      cosineCoefficient = (T(1) - std::cos(angle)) / squaredAngle;
    }
    return Matrix3::Identity() + sineCoefficient * skew + cosineCoefficient * skew * skew;
  }

  [[nodiscard]] static Vector3 rotationLog(const Matrix3& rotation) noexcept {
    Eigen::Quaternion<T> quaternion(rotation);
    quaternion.normalize();
    if (quaternion.w() < T(0)) {
      quaternion.coeffs() *= T(-1);
    }
    const T vectorNorm = quaternion.vec().norm();
    if (vectorNorm < std::sqrt(smallAngleSquared())) {
      return (T(2) / quaternion.w()) * quaternion.vec();
    }
    return (T(2) * std::atan2(vectorNorm, quaternion.w()) / vectorNorm) * quaternion.vec();
  }

  [[nodiscard]] static Matrix6 adjointAlgebra(const Tangent6& tangent) noexcept {
    Matrix6 result = Matrix6::Zero();
    const Vector3 rotation = tangent.template tail<3>();
    const Vector3 translation = tangent.template head<3>();
    const Matrix3 rotationSkew = wedge(rotation);
    result.template block<3, 3>(0, 0) = rotationSkew;
    result.template block<3, 3>(0, 3) = wedge(translation);
    result.template block<3, 3>(3, 3) = rotationSkew;
    return result;
  }

  [[nodiscard]] static Matrix3 se3LeftJacobianCoupling(const Vector3& rotation, const Vector3& translation) noexcept {
    const Matrix3 rotationSkew = wedge(rotation);
    const Matrix3 translationSkew = wedge(translation);
    const T angle = rotation.norm();
    const T angle2 = angle * angle;
    const T angle3 = angle2 * angle;
    const T angle4 = angle3 * angle;
    const T angle5 = angle4 * angle;
    const T coefficient1 = (angle - std::sin(angle)) / angle3;
    const T coefficient2 = (T(0.5) * angle2 + std::cos(angle) - T(1)) / angle4;
    const T coefficient3 = (angle * (T(1) + T(0.5) * std::cos(angle)) - T(1.5) * std::sin(angle)) / angle5;
    const Matrix3 first = rotationSkew * translationSkew + translationSkew * rotationSkew + rotationSkew * translationSkew * rotationSkew;
    const Matrix3 second =
        rotationSkew * rotationSkew * translationSkew + translationSkew * rotationSkew * rotationSkew -
        T(3) * rotationSkew * translationSkew * rotationSkew;
    const Matrix3 third =
        rotationSkew * translationSkew * rotationSkew * rotationSkew + rotationSkew * rotationSkew * translationSkew * rotationSkew;
    return T(0.5) * translationSkew + coefficient1 * first + coefficient2 * second + coefficient3 * third;
  }

  alignas(16) Coefficients coefficients_{};
};

template <typename T>
std::ostream& operator<<(std::ostream& stream, const StateGroup<T>& state) {
  return stream << state.debugString();
}

using StateGroupf = StateGroup<float>;
using StateGroupd = StateGroup<double>;

static_assert(sizeof(StateGroupf) == 12 * sizeof(float), "StateGroupf storage must remain 12 floats");
static_assert(sizeof(StateGroupd) == 12 * sizeof(double), "StateGroupd storage must remain 12 doubles");

}  // namespace sapphire::backend
