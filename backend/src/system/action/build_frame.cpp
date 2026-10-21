#include <algorithm>
#include <backend/common/common_type.hpp>
#include <backend/registration/descriptor/xfeat.hpp>
#include <backend/system/action/build_frame.hpp>
#include <backend/system/action/preprocess.hpp>
#include <backend/system/action/sensor_runtime.hpp>
#include <backend/params/feature.hpp>
#include <backend/params/preprocess.hpp>
#include <backend/sensor/camera/utils.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/timer.hpp>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace sapphire::backend {
namespace {
camera::Perspective stereoCanvas(const camera::Perspective& source, cv::Size target) {
  const double sx = double(target.width) / source.imageSize().width;
  const double sy = double(target.height) / source.imageSize().height;
  return {source.fx() * sx,      source.fy() * sy, (source.cx() + .5) * sx - .5, (source.cy() + .5) * sy - .5,
          source.cameraToBase(), target,           source.focalXBaseline() * sx};
}
}  // namespace
FrameBuilder::FrameBuilder(const ParametersMap& parameters, std::shared_ptr<SensorRuntime> runtime)
    : FrameBuilder(PreprocessParams{}.withOverrides(parameters), FeatureParams{}.withOverrides(parameters), std::move(runtime)) {}

FrameBuilder::FrameBuilder(const PreprocessParams& preprocess, const FeatureParams& feature, std::shared_ptr<SensorRuntime> runtime,
                           std::shared_ptr<feature::RknnModels> models)
    : feature_enabled_(feature.enabled),
      runtime_(std::move(runtime)),
      extractor_(feature),
      pre_decimation_(preprocess.feature_image_decimation),
      post_decimation_(preprocess.storage_image_decimation),
      stereo_size_(preprocess.stereo_image_width, preprocess.stereo_image_height) {
  feature.validate(preprocess);
  if (feature.uses_xfeat() && !models) models = std::make_shared<feature::RknnModels>(feature);
  if (models) models->validate(feature);
  if (feature.uses_xfeat()) {
    xfeat_ = models->xfeat();
    global_encoder_ = models->global();
  }
  if (runtime_) runtime_->validate_parameters(feature, pre_decimation_, stereo_size_);
}

FastOrbStereo& FrameBuilder::channel(size_t index, const SensorData& input, unsigned int decimation, bool resized_stereo) {
  if (runtime_) {
    if (input.visual_camera_ids.size() != input.perspectives().size())
      throw std::invalid_argument("Registered visual input requires camera IDs in layout order");
    const size_t slot = runtime_->channel_slot(input.visual_camera_ids.at(index));
    const auto expected = resized_stereo ? stereoCanvas(runtime_->model(slot), stereo_size_) : runtime_->model(slot).scaled(1.0 / decimation);
    const auto& actual = input.perspectives().at(index);
    if (actual.imageSize() != expected.imageSize() || actual.fx() != expected.fx() || actual.fy() != expected.fy() || actual.cx() != expected.cx() ||
        actual.cy() != expected.cy() || actual.focalXBaseline() != expected.focalXBaseline() ||
        actual.cameraToBase().coefficients() != expected.cameraToBase().coefficients())
      throw std::invalid_argument("Input camera does not match registered processing profile");
    if (!input.stereo_right().empty() && !runtime_->stereo(slot)) throw std::invalid_argument("Right image supplied to a monocular channel");
    return runtime_->extractor(slot);
  }
  if (input.perspectives().size() == 1) return extractor_;
  if (camera_extractors_.size() != input.perspectives().size()) {
    std::vector<std::unique_ptr<FastOrbStereo>> channels;
    const int maximum = extractor_.getMaxFeatures();
    auto setup = extractor_.configuration();
    setup.max_features = maximum > 0 ? (maximum + int(input.perspectives().size()) - 1) / int(input.perspectives().size()) : maximum;
    for (size_t i = 0; i < input.perspectives().size(); ++i) channels.push_back(std::make_unique<FastOrbStereo>(setup));
    camera_extractors_ = std::move(channels);
  }
  return *camera_extractors_.at(index);
}

FrameBuilder::Result FrameBuilder::build(const SensorData& input) {
  Result result;
  if (!feature_enabled_) {
    result.image = input.imageRaw();
    result.depth = input.depthRaw();
    result.cameras = input.perspectives();
    return result;
  }
  const SensorData& data = input;
  const bool isIntermediateNode = input.id() < 0;
  if (runtime_ && !input.imageRaw().empty()) {
    for (size_t i = 0; i < input.perspectives().size(); ++i) {
      channel(i, input, 1);
      if (runtime_->stereo(runtime_->channel_slot(input.visual_camera_ids[i])) && input.stereo_right().empty())
        throw std::invalid_argument("Registered stereo channel requires a right image");
      if (std::find(input.visual_camera_ids.begin(), input.visual_camera_ids.begin() + i, input.visual_camera_ids[i]) !=
          input.visual_camera_ids.begin() + i)
        throw std::invalid_argument("Duplicate camera in visual input layout");
    }
  }
  camera::validateImageMask(input.feature_mask(), input.imageRaw().size());
  camera::validateImageMask(input.stereo_mask(), input.stereo_right().size());
  auto& keypoints = result.features.keypoints;
  auto& keypoints3D = result.features.points;
  auto& descriptors = result.features.descriptors;
  const auto prepareDescriptors = [&](FastOrbStereo& extractor, const cv::Mat& image, std::vector<cv::KeyPoint>& keys,
                                      const cv::Mat& mask = cv::Mat()) {
    // Epipolar stereo reads image patches; XFeatDescriptor supplies the stored descriptors
    // below. Keep ORB's keypoint filtering/orientations, but not its unused bytes.
    if (xfeat_) {
      extractor.prepareKeypoints(image, keys, mask);
      return cv::Mat();
    }
    return extractor.generateDescriptors(image, keys, mask);
  };
  keypoints = input.visualFrame().keypoints();
  keypoints3D = input.visualFrame().keypoints3D();
  descriptors = input.visualFrame().descriptors();
  EXT_ASSERT(keypoints3D.empty() || keypoints3D.size() == keypoints.size());
  EXT_ASSERT(descriptors.empty() || descriptors.rows == static_cast<int>(keypoints.size()));
  tools::Timer timer;
  float t;
  unsigned int preDecimation = 1;
  SensorData decimatedData;
  bool resizedStereo = false;
  if (keypoints.empty() && extractor_.getMaxFeatures() >= 0 && !data.imageRaw().empty() && !isIntermediateNode) {
    if (stereo_size_.width && !data.stereo_right().empty() && data.imageRaw().size() != stereo_size_) {
      if (data.perspectives().size() != 1 || data.stereo_right().size() != data.imageRaw().size())
        throw std::invalid_argument("Fixed stereo canvas requires a single calibrated, same-size left/right pair");
      tools::Timer resizeTimer;
      cv::Mat image, right, depth;
      cv::resize(data.imageRaw(), image, stereo_size_, 0, 0, cv::INTER_AREA);
      cv::resize(data.stereo_right(), right, stereo_size_, 0, 0, cv::INTER_AREA);
      if (!data.depthRaw().empty()) cv::resize(data.depthRaw(), depth, stereo_size_, 0, 0, cv::INTER_NEAREST_EXACT);
      decimatedData = data;
      decimatedData.setImageData(image, depth, stereoCanvas(data.perspectives().front(), stereo_size_), true);
      decimatedData.set_stereo_right(right);
      decimatedData.set_feature_masks(camera::resizeImageMask(data.feature_mask(), stereo_size_),
                                      camera::resizeImageMask(data.stereo_mask(), stereo_size_));
      resizedStereo = true;
      result.timing.post_decimation_ms = resizeTimer.elapsed_seconds() * 1000;
    } else if (pre_decimation_ > 1) {
      decimatedData = data;
      preDecimation = pre_decimation_;
      auto cameras = decimatedData.perspectives();
      cv::Mat image = decimatedData.imageRaw(), depth = decimatedData.depthRaw();
      decimateImages(image, depth, cameras, pre_decimation_);
      decimatedData.setImageData(image, depth, std::move(cameras));
      // Resize each camera independently so masks never bleed across a mosaic seam.
      std::vector<cv::Mat> masks;
      const auto originalRois = camera::cameraImageRois(data.perspectives());
      const auto reducedRois = camera::cameraImageRois(decimatedData.perspectives());
      if (!data.feature_mask().empty())
        for (size_t i = 0; i < originalRois.size(); ++i)
          masks.push_back(camera::resizeImageMask(data.feature_mask()(originalRois[i]), reducedRois[i].size()));
      decimatedData.set_feature_masks(camera::makeCameraMosaic(masks), camera::resizeImageMask(data.stereo_mask(), image.size()));
    }

    EXT_LOG_DEBUG("Extract features");
    const SensorData& extractionData = decimatedData.imageRaw().empty() ? data : decimatedData;
    const cv::Mat& imageMono = extractionData.imageRaw();

    const std::vector<cv::Rect> cameraRois = camera::cameraImageRois(extractionData.perspectives());
    std::vector<cv::Mat> cameraDescriptors;
    cameraDescriptors.reserve(cameraRois.size());
    double keypointsSeconds = 0, descriptorsSeconds = 0;
    tools::Timer featureTimer;
    for (size_t cameraIndex = 0; cameraIndex < cameraRois.size(); ++cameraIndex) {
      const auto& cameraRoi = cameraRois[cameraIndex];
      auto& cameraExtractor = channel(cameraIndex, extractionData, preDecimation, resizedStereo);
      const cv::Mat cameraImage = imageMono(cameraRoi);
      featureTimer.lap_seconds();
      const auto mask = extractionData.feature_mask().empty() ? cv::Mat() : extractionData.feature_mask()(cameraRoi);
      std::vector<cv::KeyPoint> cameraKeypoints = cameraExtractor.generateKeypoints(cameraImage, mask);
      keypointsSeconds += featureTimer.lap_seconds();
      cv::Mat cameraDescriptor = prepareDescriptors(cameraExtractor, cameraImage, cameraKeypoints, mask);
      descriptorsSeconds += featureTimer.lap_seconds();
      for (cv::KeyPoint& keypoint : cameraKeypoints) {
        keypoint.pt.x += cameraRoi.x;
        keypoint.pt.y += cameraRoi.y;
      }
      if (cameraRois.size() == 1)
        keypoints = std::move(cameraKeypoints);
      else
        keypoints.insert(keypoints.end(), cameraKeypoints.begin(), cameraKeypoints.end());
      if (!cameraDescriptor.empty()) {
        cameraDescriptors.push_back(cameraDescriptor);
      }
    }
    if (cameraDescriptors.size() == 1) {
      descriptors = std::move(cameraDescriptors.front());
    } else if (!cameraDescriptors.empty()) {
      cv::vconcat(cameraDescriptors, descriptors);
    }

    timer.lap_seconds();
    t = keypointsSeconds;
    result.timing.keypoints_ms = t * 1000.0f;
    EXT_LOG_DEBUG("time keypoints ({:d}) = {:.6f}s", (int)keypoints.size(), t);

    t = descriptorsSeconds;
    result.timing.descriptors_ms = t * 1000.0f;
    EXT_LOG_DEBUG("time descriptors ({:d}) = {:.6f}s", descriptors.rows, t);

  } else if (!keypoints.empty() && descriptors.empty() && !data.imageRaw().empty() && !isIntermediateNode) {
    descriptors = prepareDescriptors(data.perspectives().size() == 1 ? channel(0, data, 1) : extractor_, data.imageRaw(), keypoints);
    EXT_ASSERT(descriptors.empty() || descriptors.rows == static_cast<int>(keypoints.size()));
  } else if (keypoints.empty() && data.imageRaw().empty()) {
    EXT_LOG_DEBUG("Empty image, cannot extract features...");
  } else if (keypoints.empty() && extractor_.getMaxFeatures() < 0) {
    EXT_LOG_DEBUG("extractor_.getMaxFeatures()({:d}<0) so don't extract any features...", extractor_.getMaxFeatures());
  } else if (keypoints.empty()) {
    EXT_LOG_DEBUG("Intermediate node detected, don't extract features!");
  }

  const SensorData& featureData = decimatedData.imageRaw().empty() ? data : decimatedData;
  if (keypoints3D.empty() && !keypoints.empty()) {
    if (!data.stereo_right().empty()) {
      if (featureData.perspectives().size() != 1) throw std::invalid_argument("Sparse stereo requires one rectified left camera");
      cv::Mat right = resizedStereo       ? featureData.stereo_right()
                      : preDecimation > 1 ? camera::decimateImage(data.stereo_right(), preDecimation)
                                          : data.stereo_right();
      tools::Timer stereo_timer;
      keypoints3D = channel(0, featureData, preDecimation, resizedStereo)
                        .stereo_points(featureData.imageRaw(), right, keypoints, featureData.perspectives().front(), featureData.stereo_mask(),
                                       featureData.feature_mask());
      const auto valid = std::count_if(keypoints3D.begin(), keypoints3D.end(), [](const auto& p) { return std::isfinite(p.z); });
      EXT_LOG_DEBUG("[stereo] Valid depth={}/{} elapsed_ms={:.1f}", valid, keypoints3D.size(), stereo_timer.elapsed_seconds() * 1000);
    } else {
      keypoints3D.assign(keypoints.size(), cv::Point3f(NAN, NAN, NAN));
    }
  }

  result.right = resizedStereo ? decimatedData.stereo_right() : data.stereo_right();
  result.left_mask = resizedStereo ? decimatedData.feature_mask() : data.feature_mask();
  result.right_mask = resizedStereo ? decimatedData.stereo_mask() : data.stereo_mask();
  auto& image = result.image;
  image = resizedStereo ? decimatedData.imageRaw() : data.imageRaw();
  auto& depthImage = result.depth;
  depthImage = resizedStereo ? decimatedData.depthRaw() : data.depthRaw();

  auto& perspectives = result.cameras;
  perspectives = resizedStereo ? decimatedData.perspectives() : data.perspectives();

  // apply decimation?
  if (post_decimation_ > 1 && !isIntermediateNode) {
    if (post_decimation_ == preDecimation && decimatedData.isValid()) {
      image = decimatedData.imageRaw();
      depthImage = decimatedData.depthRaw();
      perspectives = decimatedData.perspectives();
    } else {
      decimateImages(image, depthImage, perspectives, post_decimation_);
    }

    if (!image.empty() && (depthImage.cols > image.cols || depthImage.rows > image.rows)) {
      EXT_LOG_WARN("Depth image is bigger than RGB image after post decimation, {}={:d} is too high! RGB={:d}x{:d}, depth={:d}x{:d}",
                   "preprocess.storage_image_decimation", post_decimation_, image.cols, image.rows, depthImage.cols, depthImage.rows);
    }

    t = timer.lap_seconds();
    result.timing.post_decimation_ms = t * 1000.0f;
    EXT_LOG_DEBUG("time post-decimation = {:.6f}s", t);
  }

  if (preDecimation != post_decimation_ && !isIntermediateNode) {
    const float scale = float(preDecimation) / post_decimation_;
    for (auto& keypoint : keypoints) {
      keypoint.pt *= scale;
      keypoint.size *= scale;
    }
  }
  if (xfeat_ && !isIntermediateNode && !result.image.empty()) {
    if (result.cameras.size() != 1) throw std::invalid_argument("XFeat requires one rectified left camera");
    tools::Timer descriptor_timer;
    result.features.descriptors = xfeat_->describe(result.image, result.features.keypoints);
    result.features.descriptor_model_id = xfeat_->model_id();
    result.timing.descriptors_ms += descriptor_timer.elapsed_seconds() * 1000;
  }
  return result;
}
}  // namespace sapphire::backend
