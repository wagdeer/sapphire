#pragma once
#include <backend/common/common_type.hpp>
#include <backend/sensor/camera/kb4.hpp>

#include "backend/common/sensor_data.hpp"

namespace sapphire::backend {
// Reusable rectification maps. Calibration parsing and ROS message lifetime stay
// at the input boundary; every accepted image uses the same core pixel kernel.
class StereoRectifier final {
 public:
  void configure(bool right, const camera::KB4&, const Mat33_t<double>& rotation, const Mat33_t<double>& intrinsics, cv::Size output_size,
                 cv::Size expected_input_size = {});
  // Called serially by the input dispatcher; the first image fixes each camera's
  // input dimensions and sampling offsets until the next configure().
  cv::Mat rectify(const cv::Mat& mono, bool right);
  void setRawMask(bool right, const cv::Mat& mask);
  const cv::Mat& featureMask(bool right) const { return rectified_mask_[right ? 1 : 0]; }

 private:
  cv::Size input_size_[2], output_size_[2], expected_input_size_[2];
  cv::Mat raw_mask_[2], rectified_mask_[2];
  cv::Mat map1_[2], map2_[2], offsets_[2];
  bool inside_[2]{};
};
class LocalGridMaker;
struct LocalGridTiming;
class Statistics;
// Classify one observation for navigation. Missing/disabled depth contributes no
// free space or footprint evidence. The returned immutable grid is shared onward.
std::shared_ptr<const LocalGrid> preprocess_depth(const SensorData&, const StateGroupf&, const LocalGridMaker&, bool enabled,
                                                  LocalGridTiming* timing = nullptr);
// Legacy node input uses the same navigation algorithm with the final image
// layout. Already classified input is shared without a second depth pass.
bool prepare_navimap(SensorData& observation, const SensorData& input, const StateGroupf& pose, const LocalGridMaker&, bool enabled,
                     Statistics* = nullptr);
// Live input boundary: normalize float meters to uint16 millimeters once.
// Existing millimeter buffers remain shared. Archive decoding is a separate database operation.
SensorData prepareSensorData(const cv::Mat& image, const cv::Mat& depth, const camera::Perspective& camera, int id = 0, double stamp = 0.0,
                             const cv::Mat& userData = {});
SensorData prepareSensorData(const cv::Mat& image, const cv::Mat& depth, const std::vector<camera::Perspective>& cameras, int id = 0,
                             double stamp = 0.0, const cv::Mat& userData = {});

// Decimation changes pixel dimensions and intrinsics together; extrinsics stay unchanged.
// Input pixel storage is never modified through a shared cv::Mat view.
void decimateImages(cv::Mat& image, cv::Mat& depth, std::vector<camera::Perspective>& cameras, int factor);
}  // namespace sapphire::backend
