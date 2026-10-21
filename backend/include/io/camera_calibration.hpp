#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <backend/io/sensor_config.hpp>
#include <backend/system/action/sensor_runtime.hpp>
#include <memory>
#include <backend/system/action/preprocess.hpp>
#include <backend/sensor/camera/perspective.hpp>
#include <filesystem>
#include <fstream>
#include <limits>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <vector>

namespace sapphire::backend::io {
namespace detail {

inline YAML::Node resolveYAMLNode(const YAML::Node& node, const std::string& key, const std::string& context) {
  if (!node || !node.IsMap()) {
    throw std::runtime_error(context + " must be a map");
  }
  const YAML::Node value = node[key];
  if (!value) {
    throw std::runtime_error("Missing '" + key + "' in " + context);
  }
  return value;
}

inline YAML::Node resolveYAMLTopic(const YAML::Node& root, const std::string& topic, const std::string& path) {
  const YAML::Node topics = resolveYAMLNode(root, "topics", path);
  const YAML::Node payload = topics[topic];
  if (!payload || !payload.IsMap()) {
    throw std::runtime_error("Missing map topic '" + topic + "' in " + path);
  }
  return payload;
}

inline void resolveNumericValues(const YAML::Node& node, const std::string& context, std::vector<double>& values) {
  if (!node) {
    throw std::runtime_error("Empty numeric sequence at " + context);
  }
  if (node.IsScalar()) {
    values.push_back(node.as<double>());
    return;
  }
  if (!node.IsSequence()) {
    throw std::runtime_error(context + " must be a numeric scalar or sequence");
  }
  for (const auto& entry : node) {
    resolveNumericValues(entry, context, values);
  }
}

inline std::vector<double> getNumericValues(const YAML::Node& node, const std::string& context) {
  std::vector<double> values;
  resolveNumericValues(node, context, values);
  for (double value : values) {
    if (!std::isfinite(value)) {
      throw std::runtime_error(context + " contains a non-finite value");
    }
  }
  return values;
}

inline Eigen::Matrix3d resolveEigenMat3d(const YAML::Node& node, const std::string& context) {
  const std::vector<double> values = getNumericValues(node, context);
  if (values.size() != 9) {
    throw std::runtime_error(context + " must contain 9 numeric values");
  }
  Eigen::Matrix3d matrix;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      matrix(row, col) = values[static_cast<std::size_t>(row * 3 + col)];
    }
  }
  return matrix;
}

inline Eigen::Vector3d resolveEigenVec3d(const YAML::Node& node, const std::string& context) {
  const std::vector<double> values = getNumericValues(node, context);
  if (values.size() != 3) {
    throw std::runtime_error(context + " must contain 3 numeric values");
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

inline void checkOrthoOrient(const Eigen::Matrix3d& rotation, const std::string& context) {
  if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-5 || std::abs(rotation.determinant() - 1.0) > 1e-5) {
    throw std::runtime_error(context + " is not a valid orthogonal orientation matrix");
  }
}

// T_A_B maps points from B into A. Calibration R/T follow vslam's
// convention: p_camera = R * p_reference + T (translations in metres).
inline Eigen::Isometry3d readRigidTransform(const YAML::Node& payload, const std::string& rotation_key,
                                          const std::string& translation_key, const std::string& context) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = resolveEigenMat3d(resolveYAMLNode(payload, rotation_key, context), context + "." + rotation_key);
  transform.translation() = resolveEigenVec3d(resolveYAMLNode(payload, translation_key, context), context + "." + translation_key);
  checkOrthoOrient(transform.linear(), context + "." + rotation_key);
  return transform;
}

inline Eigen::Isometry3d baseFromRawLeft(const YAML::Node& extrinsics, const std::string& stereo_key,
                                       const std::string& path) {
  const auto topics = resolveYAMLNode(extrinsics, "topics", path);
  // Both keys explicitly identify the raw stereo-left anchor. Rectified topic
  // aliases are deliberately excluded: interpreting them as raw would rotate twice.
  const std::string raw_key = "/camera_left_eye/image_raw";
  auto read = [&](const std::string& key) {
    return readRigidTransform(resolveYAMLTopic(extrinsics, key, path), "camera_to_base_rotation",
                              "camera_to_base_translation", path + ":" + key);
  };
  if (topics[stereo_key]) {
    const auto result = read(stereo_key);
    if (stereo_key != raw_key && topics[raw_key] && !result.matrix().isApprox(read(raw_key).matrix(), 1e-6))
      throw std::runtime_error(path + ": conflicting raw stereo-left anchors: " + stereo_key + " and " + raw_key);
    return result;
  }
  if (topics[raw_key]) return read(raw_key);
  throw std::runtime_error(path + ": missing raw stereo-left to base extrinsic under " + stereo_key + " or " + raw_key);
}

inline Eigen::Isometry3d baseFromRawSide(const YAML::Node& calibration, const Eigen::Isometry3d& base_from_left,
                                       const std::string& stereo_key, const std::string& side_key, const std::string& path) {
  const auto stereo = resolveYAMLTopic(calibration, stereo_key, path);
  const auto side = resolveYAMLTopic(calibration, side_key, path);
  if (resolveYAMLNode(stereo, "camera_model", stereo_key).as<std::string>() != "stereo_kb4" ||
      resolveYAMLNode(side, "camera_model", side_key).as<std::string>() != "kb4")
    throw std::runtime_error(path + ": expected stereo_kb4 anchor and raw kb4 side camera");
  const auto left_from_reference = readRigidTransform(stereo, "left_R", "left_T", path + ":" + stereo_key);
  const auto side_from_reference = readRigidTransform(side, "R", "T", path + ":" + side_key);
  return base_from_left * left_from_reference * side_from_reference.inverse();
}

inline Eigen::Isometry3d baseFromRawRight(const YAML::Node& stereo, const Eigen::Isometry3d& base_from_left,
                                        const std::string& context) {
  const auto right_from_left = readRigidTransform(stereo, "R_right_left", "T_right_left", context);
  return base_from_left * right_from_left.inverse();
}

inline Eigen::Isometry3d baseFromRectified(const Eigen::Isometry3d& base_from_raw, const Eigen::Matrix3d& rectify_rotation,
                                         const std::string& context) {
  checkOrthoOrient(rectify_rotation, context);
  Eigen::Isometry3d raw_from_rectified = Eigen::Isometry3d::Identity();
  raw_from_rectified.linear() = rectify_rotation.transpose();
  return base_from_raw * raw_from_rectified;
}

inline void readCalibration(const std::string& intrin_path, const std::string& extin_path, const std::string& topic_key,
                            sapphire::backend::camera::Perspective& perspective, const std::string& raw_key = {}) {
  if (intrin_path.empty() || extin_path.empty()) {
    throw std::runtime_error("intrinsic_path and extrinsic_path are required for resolving calibration yaml file.");
  }
  std::ifstream intrin_stream(intrin_path);
  if (!intrin_stream.good()) {
    throw std::runtime_error("Intrinsic file does not exist or is not readable: " + intrin_path);
  }
  std::ifstream extin_stream(extin_path);
  if (!extin_stream.good()) {
    throw std::runtime_error("Extrinsic file does not exist or is not readable: " + extin_path);
  }

  const YAML::Node intrin_payload = resolveYAMLTopic(YAML::Load(intrin_stream), topic_key, intrin_path);
  const auto base_from_left = baseFromRawLeft(YAML::Load(extin_stream), raw_key.empty() ? topic_key : raw_key, extin_path);
  const std::string intrin_context = intrin_path + ":" + topic_key;

  const Eigen::Matrix3d intrinsics = resolveEigenMat3d(resolveYAMLNode(intrin_payload, "rectify_params", intrin_context), intrin_context + ".rectify_params");
  const Eigen::Matrix3d rectifyR1 = resolveEigenMat3d(resolveYAMLNode(intrin_payload, "rectify_R1", intrin_context), intrin_context + ".rectify_R1");
  const std::vector<double> imageSize =
      getNumericValues(resolveYAMLNode(intrin_payload, "rectify_image_size", intrin_context), intrin_context + ".rectify_image_size");

  if (imageSize.size() != 2 || imageSize[0] <= 0.0 || imageSize[1] <= 0.0 || imageSize[0] > static_cast<double>(std::numeric_limits<int>::max()) ||
      imageSize[1] > static_cast<double>(std::numeric_limits<int>::max())) {
    throw std::runtime_error(intrin_context + ".rectify_image_size must contain two valid positive image dimensions");
  }
  if (intrinsics(0, 0) <= 0.0 || intrinsics(1, 1) <= 0.0) {
    throw std::runtime_error(intrin_context + ".rectify_params has invalid focal lengths");
  }
  const auto base_from_rectified = baseFromRectified(base_from_left, rectifyR1, intrin_context + ".rectify_R1");
  const sapphire::backend::StateGroupf cameraToBase = sapphire::backend::StateGroupf::fromRotationTranslation(
      base_from_rectified.linear().cast<float>(), base_from_rectified.translation().cast<float>());

  const double baseline = intrin_payload["baseline"] ? intrin_payload["baseline"].as<double>() : 0;
  if (!std::isfinite(baseline) || baseline < 0) throw std::runtime_error(intrin_context + ".baseline must be finite and nonnegative");
  perspective = sapphire::backend::camera::Perspective(intrinsics(0, 0), intrinsics(1, 1), intrinsics(0, 2), intrinsics(1, 2), cameraToBase,
                                              cv::Size(static_cast<int>(std::llround(imageSize[0])), static_cast<int>(std::llround(imageSize[1]))),
                                              intrinsics(0, 0) * baseline);
}

}  // namespace detail
inline camera::Perspective loadCalibration(const std::filesystem::path& directory, const std::string& key) {
  camera::Perspective camera;
  detail::readCalibration((directory / "calibration_results.yaml").string(), (directory / "left_to_baselink_extrinsics.yaml").string(), key,
                          camera);
  return camera;
}
inline camera::Perspective loadCalibration(const SensorConfig& input) {
  camera::Perspective camera;
  detail::readCalibration((input.directory / "calibration_results.yaml").string(),
                          (input.directory / "left_to_baselink_extrinsics.yaml").string(),
                          input.rectificationKey(), camera, input.calibration_key);
  return camera;
}
// Capture/binning scales the calibrated coordinate axes around their origin,
// matching the sensor profile and frontend. This is not a cv::resize half-pixel
// transform, and does not alter the independently calibrated rectified canvas.
inline cv::Size rawImageSize(const SensorConfig& input, int64_t id) {
  const auto it = input.cameras.find(id);
  if (it == input.cameras.end()) return {};
  return {it->second.raw_image_size[0], it->second.raw_image_size[1]};
}
inline Eigen::Matrix3d rawIntrinsics(const SensorConfig& input, int64_t id, Eigen::Matrix3d K) {
  const auto it = input.cameras.find(id);
  if (it == input.cameras.end() ||
      (it->second.calibration_image_size == std::array<int,2>{} && it->second.raw_image_size == std::array<int,2>{})) {
    const double scale = input.device_input ? input.device_calibration_scale : input.ros_calibration_scale;
    if (!std::isfinite(scale) || scale <= 0.0)
      throw std::invalid_argument("Raw camera calibration scale must be finite and positive");
    K.row(0) *= scale;
    K.row(1) *= scale;
    return K;
  }
  const auto a = it->second.calibration_image_size, b = it->second.raw_image_size;
  // Explicit per-camera geometry is authoritative; never scale K twice.
  if (a[0] <= 0 || a[1] <= 0 || b[0] <= 0 || b[1] <= 0 || int64_t(a[0])*b[1] != int64_t(a[1])*b[0])
    throw std::invalid_argument("Invalid raw camera capture geometry");
  K.row(0) *= double(b[0])/a[0];
  K.row(1) *= double(b[1])/a[1];
  return K;
}
// Raw left optical frame: do not apply rectify_R1 or use rectified intrinsics.
// Input dimensions come from the actual unscaled capture.
inline camera::Perspective loadRawLeftCalibration(const SensorConfig& input, cv::Mat& distortion) {
  using namespace detail;
  const auto ip = (input.directory / "calibration_results.yaml").string();
  const auto ep = (input.directory / "left_to_baselink_extrinsics.yaml").string();
  const auto i = resolveYAMLTopic(YAML::LoadFile(ip), input.calibration_key, ip);
  const auto base_from_left = baseFromRawLeft(YAML::LoadFile(ep), input.calibration_key, ep);
  const auto& key = input.calibration_key;
  const auto model = resolveYAMLNode(i,"camera_model",key).as<std::string>();
  if (model != "stereo_kb4" && model != "kb4")
    throw std::invalid_argument(key + ": raw left camera must be kb4 or stereo_kb4");
  const char* k = model == "stereo_kb4" ? "left_K" : "K";
  const char* d = model == "stereo_kb4" ? "left_D" : "D";
  const auto K = rawIntrinsics(input, input.stereo_cameras[0], resolveEigenMat3d(resolveYAMLNode(i,k,key),key+"."+k));
  const auto D = getNumericValues(resolveYAMLNode(i,d,key),key+"."+d);
  if (K(0,0)<=0 || K(1,1)<=0 || D.size()!=4) throw std::invalid_argument(key+": invalid raw left calibration");
  distortion=cv::Mat(D).clone();
  return camera::Perspective(K(0,0),K(1,1),K(0,2),K(1,2),
                            StateGroupf::fromRotationTranslation(base_from_left.linear().cast<float>(),base_from_left.translation().cast<float>()),
                            rawImageSize(input, input.stereo_cameras[0]));
}
// Register active physical cameras once. Raw models and the rectified processing
// profile remain distinct, even when they belong to the same physical sensor.
inline std::shared_ptr<SensorRuntime> loadSensorRuntime(const SensorConfig& input, const camera::Perspective& rectified,
    std::vector<SensorRuntime::Id> active, bool register_stereo,
    std::vector<SensorRuntime::Channel> channels, const ParametersMap& parameters) {
  using namespace detail;
  if (register_stereo) { active.push_back(input.stereo_cameras[0]); active.push_back(input.stereo_cameras[1]); }
  std::sort(active.begin(), active.end());
  active.erase(std::unique(active.begin(), active.end()), active.end());
  const auto ip = (input.directory / "calibration_results.yaml").string();
  const auto ep = (input.directory / "left_to_baselink_extrinsics.yaml").string();
  const auto intrinsics = YAML::LoadFile(ip);
  const auto base_from_left = baseFromRawLeft(YAML::LoadFile(ep), input.calibration_key, ep);
  const auto anchor = resolveYAMLTopic(intrinsics, input.calibration_key, ip);
  std::vector<SensorRuntime::Camera> cameras;
  for (const auto id : active) {
    const auto binding = input.cameras.find(id);
    if (binding == input.cameras.end() && !input.cameras.empty()) throw std::invalid_argument("Camera has no calibration binding: " + std::to_string(id));
    const auto key = binding == input.cameras.end() ? input.calibration_key : binding->second.calibration_key;
    const auto intrinsic = resolveYAMLTopic(intrinsics, key, ip);
    const bool left = id == input.stereo_cameras[0], right = id == input.stereo_cameras[1];
    const auto type = resolveYAMLNode(intrinsic, "camera_model", key).as<std::string>();
    if (type != "kb4" && type != "stereo_kb4") throw std::invalid_argument(key + ": unsupported raw camera model");
    const bool stereo = type == "stereo_kb4";
    if (stereo && !left && !right) throw std::invalid_argument(key + ": stereo calibration requires a pair member");
    Eigen::Isometry3d transform = base_from_left;
    if (right) {
      if (anchor["R_right_left"] || anchor["T_right_left"]) transform = baseFromRawRight(anchor, base_from_left, input.calibration_key);
      else {
        const auto target = resolveYAMLTopic(intrinsics, input.rectificationKey(), ip);
        const auto R2 = resolveEigenMat3d(resolveYAMLNode(target, "rectify_R2", input.rectificationKey()), "rectify_R2");
        checkOrthoOrient(R2, "rectify_R2");
        Eigen::Isometry3d base_rect = Eigen::Isometry3d::Identity();
        base_rect.linear() = rectified.cameraToBase().rotationMatrix().cast<double>();
        base_rect.translation() = rectified.cameraToBase().translationVector().cast<double>();
        base_rect.translation() += base_rect.linear() * Eigen::Vector3d(rectified.focalXBaseline() / rectified.fx(), 0, 0);
        Eigen::Isometry3d rect_from_raw = Eigen::Isometry3d::Identity();
        rect_from_raw.linear() = R2;
        transform = base_rect * rect_from_raw;
      }
    } else if (!left) transform = baseFromRawSide(intrinsics, base_from_left, input.calibration_key, key, ip);
    const std::string k = stereo ? (left ? "left_K" : "right_K") : "K";
    const std::string d = stereo ? (left ? "left_D" : "right_D") : "D";
    const auto K = rawIntrinsics(input, id, resolveEigenMat3d(resolveYAMLNode(intrinsic, k, key), key + "." + k));
    const auto D = getNumericValues(resolveYAMLNode(intrinsic, d, key), key + "." + d);
    if (K(0, 0) <= 0 || K(1, 1) <= 0 || D.size() != 4) throw std::invalid_argument(key + ": invalid raw camera calibration");
    cameras.push_back({id, camera::Perspective(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
        StateGroupf::fromRotationTranslation(transform.linear().cast<float>(), transform.translation().cast<float>()), rawImageSize(input, id)),
        std::array<double, 4>{D[0], D[1], D[2], D[3]}});
  }
  // sensors.toml currently selects one pair; its stable configured pair ID is 0.
  std::vector<SensorRuntime::Pair> pairs;
  if (register_stereo) pairs.push_back({0, input.stereo_cameras[0], input.stereo_cameras[1], rectified});
  return std::make_shared<SensorRuntime>(std::move(cameras), std::move(pairs), std::move(channels), parameters);
}

inline StereoRectifier loadStereoRectifier(const SensorConfig& input) {
  using namespace sapphire::backend::io::detail;
  const auto path = (input.directory / "calibration_results.yaml").string();
  const auto calibration = YAML::LoadFile(path);
  const auto rectify_key = input.rectificationKey();
  const auto target = resolveYAMLTopic(calibration, rectify_key, path);
  const auto context = path + ":" + rectify_key;
  const auto raw_left = resolveYAMLTopic(calibration, input.calibration_key, path);
  Eigen::Matrix3d rotations[2];
  for (int i = 0; i < 2; ++i) {
    const char* key = i ? "rectify_R2" : "rectify_R1";
    rotations[i] = resolveEigenMat3d(resolveYAMLNode(target, key, context), context + "." + key);
    checkOrthoOrient(rotations[i], context + "." + key);
  }
  const auto K = resolveEigenMat3d(resolveYAMLNode(target, "rectify_params", context), context + ".rectify_params");
  const auto rightK = resolveEigenMat3d(resolveYAMLNode(target, "rectify_params_right", context), context + ".rectify_params_right");
  // The disparity-based stereo camera uses one rectified intrinsic matrix.
  // Reject a different right projection instead of silently replacing it.
  if (!K.isApprox(rightK, 1e-9))
    throw std::runtime_error(context + ".rectify_params_right must match rectify_params for horizontal stereo");
  const auto size = resolveYAMLNode(target, "rectify_image_size", context).as<std::vector<int>>();
  if (size.size() != 2 || size[0] <= 0 || size[1] <= 0 || K(0, 0) <= 0 || K(1, 1) <= 0)
    throw std::runtime_error(context + ": invalid rectification target");
  const cv::Size output_size(size[0], size[1]);
  const double baseline = resolveYAMLNode(target, "baseline", context).as<double>();
  if (!std::isfinite(baseline) || baseline <= 0) throw std::runtime_error(context + ".baseline must be finite and positive");
  // A supplied relative pose must agree with the raw->rectified rotations and
  // the positive horizontal baseline. This catches reversed R/T conventions.
  if (raw_left["R_right_left"] || raw_left["T_right_left"]) {
    const auto right_from_left = readRigidTransform(raw_left, "R_right_left", "T_right_left", path + ":" + input.calibration_key);
    if ((rotations[1] * right_from_left.linear() * rotations[0].transpose() - Eigen::Matrix3d::Identity()).norm() > 1e-5 ||
        (rotations[1] * right_from_left.translation() - Eigen::Vector3d(-baseline, 0, 0)).norm() > 1e-5)
      throw std::runtime_error(context + ": R_right_left/T_right_left must map raw left to raw right and agree with rectification/baseline");
  }
  StereoRectifier result;
  for (int i = 0; i < 2; ++i) {
    const auto entry = input.cameras.find(input.stereo_cameras[i]);
    const auto raw_key = entry == input.cameras.end() ? input.calibration_key : entry->second.calibration_key;
    const auto raw = resolveYAMLTopic(calibration, raw_key, path);
    const auto raw_context = path + ":" + raw_key;
    const auto model = resolveYAMLNode(raw, "camera_model", raw_context).as<std::string>();
    if (model != "kb4" && model != "stereo_kb4") throw std::runtime_error(raw_context + ".camera_model must be kb4 or stereo_kb4");
    const char* intrinsic_key = model == "kb4" ? "K" : i ? "right_K" : "left_K";
    const char* distortion_key = model == "kb4" ? "D" : i ? "right_D" : "left_D";
    const auto rawK = rawIntrinsics(input, input.stereo_cameras[i], resolveEigenMat3d(resolveYAMLNode(raw, intrinsic_key, raw_context), raw_context + "." + intrinsic_key));
    if (rawK(0, 0) <= 0 || rawK(1, 1) <= 0) throw std::runtime_error(context + "." + intrinsic_key + " has invalid focal lengths");
    const auto D = getNumericValues(resolveYAMLNode(raw, distortion_key, raw_context), raw_context + "." + distortion_key);
    if (D.size() != 4) throw std::runtime_error(context + "." + distortion_key + " requires four distortion coefficients");
    const sapphire::backend::camera::KB4 camera_model(rawK(0, 0), rawK(1, 1), rawK(0, 2), rawK(1, 2), {D[0], D[1], D[2], D[3]});
    result.configure(i != 0, camera_model, rotations[i], K, output_size, rawImageSize(input, input.stereo_cameras[i]));
  }
  return result;
}
}  // namespace sapphire::backend::io
