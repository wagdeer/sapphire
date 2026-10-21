#include <backend/common/common_type.hpp>
#include <backend/params/feature.hpp>
#include <backend/params/preprocess.hpp>
#include <backend/system/action/sensor_runtime.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sapphire::backend {
namespace {
ParametersMap resource_parameters(const ParametersMap& input) {
  auto result = FeatureParams{}.withOverrides(input).parameters();
  const auto preprocess = PreprocessParams{}.withOverrides(input);
  result.emplace("preprocess.feature_image_decimation", std::to_string(preprocess.feature_image_decimation));
  result.emplace("preprocess.stereo_image_width", std::to_string(preprocess.stereo_image_width));
  result.emplace("preprocess.stereo_image_height", std::to_string(preprocess.stereo_image_height));
  return result;
}
}
SensorRuntime::SensorRuntime(std::vector<Camera> cameras, std::vector<Pair> pairs, std::vector<Channel> channels, const ParametersMap& parameters)
    : cameras_(std::move(cameras)), pairs_(std::move(pairs)), channels_(std::move(channels)), parameters_(resource_parameters(parameters)) {
  for (size_t i = 0; i < cameras_.size(); ++i) {
    const auto& camera = cameras_[i];
    if (camera.id < 0 || !camera.model.isValidForProjection() || !camera.model.cameraToBase().isValid() || !camera_slots_.emplace(camera.id, i).second)
      throw std::invalid_argument("Invalid or duplicate camera registration");
    if (camera.kb4)
      for (double d : *camera.kb4)
        if (!std::isfinite(d)) throw std::invalid_argument("Invalid camera distortion");
  }
  for (size_t i = 0; i < pairs_.size(); ++i) {
    const auto& pair = pairs_[i];
    if (pair.id < 0 || pair.left == pair.right || !pair_slots_.emplace(pair.id, i).second || !camera_slots_.count(pair.left) || !camera_slots_.count(pair.right) ||
        !pair.rectified_left.isValidForReprojection() || !pair.rectified_left.cameraToBase().isValid() || !(pair.rectified_left.focalXBaseline() > 0) ||
        !std::isfinite(pair.rectified_left.focalXBaseline()))
      throw std::invalid_argument("Invalid stereo pair registration");
    relative_.push_back(camera(camera_slot(pair.right)).model.cameraToBase().inverse().compose(camera(camera_slot(pair.left)).model.cameraToBase()));
  }
  const auto preprocess = PreprocessParams{}.withOverrides(parameters);
  const auto decimation = preprocess.feature_image_decimation;
  const auto features = FeatureParams{}.withOverrides(parameters);
  for (size_t i = 0; i < channels_.size(); ++i) {
    const auto& channel = channels_[i];
    if (!channel_slots_.emplace(channel.camera, i).second) throw std::invalid_argument("Duplicate visual processing channel");
    const auto& registered = camera(camera_slot(channel.camera));
    auto model = registered.model;
    if (channel.pair) {
      const auto& pair = this->pair(pair_slot(*channel.pair));
      if (pair.left != channel.camera) throw std::invalid_argument("Stereo channel must identify its left camera");
      model = pair.rectified_left;
    } else if (registered.kb4) throw std::invalid_argument("Raw KB4 features require a rectified processing profile");
    if (model.imageSize().width <= 0 || model.imageSize().height <= 0) throw std::invalid_argument("Processing channel requires calibrated image dimensions");
    models_.push_back(model);
    auto setup = features;
    setup.max_features = channel.feature_budget;
    auto extractor = std::make_unique<FastOrbStereo>(setup);
    const auto size = channel.pair && preprocess.stereo_image_width ? cv::Size(preprocess.stereo_image_width, preprocess.stereo_image_height) : model.scaled(1.0 / decimation).imageSize();
    if (channel.feature_budget >= 0) extractor->prepare(size, channel.pair.has_value());
    extractors_.push_back(std::move(extractor));
  }
}
void SensorRuntime::validate_parameters(const ParametersMap& parameters) const {
  if (resource_parameters(parameters) != parameters_) throw std::invalid_argument("Sensor runtime configuration changed; prepare a new runtime between sessions");
}
void SensorRuntime::validate_parameters(const FeatureParams& feature, unsigned int decimation, cv::Size stereo_size) const {
  auto parameters = feature.parameters();
  parameters.emplace("preprocess.feature_image_decimation", std::to_string(decimation));
  parameters.emplace("preprocess.stereo_image_width", std::to_string(stereo_size.width));
  parameters.emplace("preprocess.stereo_image_height", std::to_string(stereo_size.height));
  if (parameters != parameters_) throw std::invalid_argument("Sensor runtime configuration changed; prepare a new runtime between sessions");
}
}  // namespace sapphire::backend
