#pragma once

#include <cstddef>
#include <memory>
#include <map>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <utility>

#include "backend/sensor/camera/perspective.hpp"

namespace sapphire::backend {

// Measurements from one image layout. Depth and descriptors may be absent.
// Shared as const; algorithms transfer owned arrays when publishing a frame.
struct VisualFeatures {
  std::vector<cv::KeyPoint> keypoints;
  std::vector<cv::Point3f> points;
  cv::Mat descriptors;
  // Zero is ORB/Hamming. Nonzero fingerprints a learned model + sampler domain.
  uint64_t descriptor_model_id = 0;
  size_t memory_size() const {
    return sizeof(VisualFeatures) + keypoints.capacity() * sizeof(cv::KeyPoint) +
        points.capacity() * sizeof(cv::Point3f) + descriptors.total() * descriptors.elemSize();
  }
};

// One frame/camera's pixel measurement of a landmark. BA uses only position and
// camera identity; descriptors and detector metadata stay in the feature block.
struct FrameObservation {
  cv::Point2f pixel;
  int camera_index = 0;
};

// Calibrated solver input: pixel coordinates within one camera image and
// isotropic variance in pixels squared. Octave/layout conversion happens once
// at the feature boundary; no detector metadata is needed by geometry solvers.
struct PixelObservation {
  cv::Point2f pixel;
  double variance = 1;
  int camera_index = 0;
};

using FeatureObservations = std::map<int, std::map<int, FrameObservation>>;  // feature ID -> frame ID -> measurement
using RejectedObservations = std::map<int, std::set<int>>;                   // feature ID -> rejected frame IDs

struct GlobalDescriptor {
  GlobalDescriptor() = default;
  GlobalDescriptor(int type, cv::Mat data, cv::Mat info = {}) : type_(type), info_(std::move(info)), data_(std::move(data)) {}

  int type() const noexcept { return type_; }
  const cv::Mat& info() const noexcept { return info_; }
  const cv::Mat& data() const noexcept { return data_; }

 private:
  int type_ = -1;
  cv::Mat info_;
  cv::Mat data_;
};

class KeypointGrid2D;

class VisualFrame {
 public:
  VisualFrame();

  void setImageBoundary(const cv::Size& size);
  const cv::Size& imageBoundary() const { return _imageBoundary; }

  void setPerspective(const camera::Perspective& model);
  void setPerspectives(const std::vector<camera::Perspective>& models);
  void setPerspectives(std::vector<camera::Perspective>&& models);
  const std::vector<camera::Perspective>& perspectives() const {
    static const std::vector<camera::Perspective> empty;
    return perspectives_ ? *perspectives_ : empty;
  }

  void setFeatures(const std::vector<cv::KeyPoint>& keypoints, const std::vector<cv::Point3f>& keypoints3D, const cv::Mat& descriptors);
  // Transfer owned arrays; the caller must retain no writable aliases.
  void set_features_owned(std::vector<cv::KeyPoint>&& keypoints, std::vector<cv::Point3f>&& points, cv::Mat&& descriptors);
  const std::vector<cv::KeyPoint>& keypoints() const { return features().keypoints; }
  const std::vector<cv::Point3f>& keypoints3D() const { return features().points; }
  const cv::Mat& descriptors() const { return features().descriptors; }

  const std::shared_ptr<const KeypointGrid2D>& keypointGrid() const { if (!_keypointGrid) updateKeypointGrid(); return _keypointGrid; }
  std::vector<std::size_t> keypointIndicesInRadius(const cv::Point2f& point, float radius) const;

  // Unsorted spatial candidates, with a hard bound on examined keypoints. False
  // means incomplete; output is cleared so callers cannot use a truncated search.
  bool collect_keypoint_indices(const cv::Point2f& point, float radius, std::size_t max_visits,
                               std::vector<std::size_t>& output, std::size_t& visits) const;

  int isPointVisibleFromCameras(const cv::Point3f& point) const;
  unsigned long getMemoryUsed() const;

  // Share an immutable measurement allocation, independently of how the caller
  // indexes or archives it. Replacing measurements invalidates the spatial grid.
  const std::shared_ptr<const VisualFeatures>& shared_features() const { return features_; }
  void share_features(std::shared_ptr<const VisualFeatures> features);

 private:
  void updateKeypointGrid() const;

  cv::Size _imageBoundary;
  std::shared_ptr<const std::vector<camera::Perspective>> perspectives_;
  const VisualFeatures& features() const {
    static const VisualFeatures empty;
    return features_ ? *features_ : empty;
  }
  std::shared_ptr<const VisualFeatures> features_;
  mutable std::shared_ptr<const KeypointGrid2D> _keypointGrid;
};

}  // namespace sapphire::backend
