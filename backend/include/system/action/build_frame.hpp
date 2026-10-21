#pragma once
#include <backend/common/common_type.hpp>
#include <backend/common/sensor_data.hpp>
#include <backend/common/visual_frame.hpp>
#include <backend/registration/fast_orb_stereo.hpp>
#include <backend/registration/utils/rknn_models.hpp>
#include <backend/registration/descriptor/siglip2.hpp>
#include <backend/params/feature.hpp>
#include <backend/params/preprocess.hpp>
#include <fstream>

namespace sapphire::backend {
class SensorRuntime;
// One mapping-thread producer. Owns extraction/detection workspaces, never maps,
// retrieval postings or persistence. The caller decides whether a frame enters a map.
class FrameBuilder final {
 public:
  struct Result {
    // Pixels and cameras use the final output layout. Feature row order and
    // extraction octave are preserved; keypoint coordinates use that same layout.
    cv::Mat image, depth;
    // Temporary shared views, released after admission; right image is not archived.
    cv::Mat right, left_mask, right_mask;
    std::vector<camera::Perspective> cameras;
    VisualFeatures features;
    struct Timing {
      double keypoints_ms = 0, descriptors_ms = 0, post_decimation_ms = 0;
    } timing;
  };
  explicit FrameBuilder(const ParametersMap& parameters = {}, std::shared_ptr<SensorRuntime> runtime = {});
  explicit FrameBuilder(const PreprocessParams& preprocess, const FeatureParams& feature, std::shared_ptr<SensorRuntime> runtime = {},
                        std::shared_ptr<feature::RknnModels> models = {});
  // Input buffers are read-only. Output images/descriptors may share their
  // storage with the input; indexed immutable publication remains the caller's job.
  // Intermediate observations retain supplied features but skip extraction.
  Result build(const SensorData& input);
  const FastOrbStereo& extractor() const { return extractor_; }
  int max_features() const { return extractor_.getMaxFeatures(); }
  float pixel_scale() const { return float(pre_decimation_) / post_decimation_; }
  // Called only after admission, not during the redundancy check.
  std::vector<GlobalDescriptor> global_descriptors(const cv::Mat& gray) {
    return global_encoder_ ? std::vector<GlobalDescriptor>{global_encoder_->encode(gray)} : std::vector<GlobalDescriptor>{};
  }

 private:
  bool feature_enabled_ = true;
  FastOrbStereo& channel(size_t index, const SensorData& input, unsigned int decimation, bool resized_stereo = false);
  std::shared_ptr<SensorRuntime> runtime_;
  std::vector<std::unique_ptr<FastOrbStereo>> camera_extractors_;
  FastOrbStereo extractor_;
  std::shared_ptr<feature::Siglip2Descriptor> global_encoder_;
  std::shared_ptr<feature::XFeatDescriptor> xfeat_;
  unsigned int pre_decimation_ = sapphire::backend::PreprocessParams{}.feature_image_decimation;
  unsigned int post_decimation_ = sapphire::backend::PreprocessParams{}.storage_image_decimation;
  cv::Size stereo_size_;
};
}  // namespace sapphire::backend
