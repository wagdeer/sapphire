#include "backend/common/visual_frame.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>

#include "backend/tools/logging.hpp"

namespace sapphire::backend {

struct KeypointGridBounds {
  float left = 0.0f;
  float top = 0.0f;
  float right = 0.0f;
  float bottom = 0.0f;
};

namespace {

bool sameFloat(float left, float right) { return left == right || (std::isnan(left) && std::isnan(right)); }

bool sameBounds(const KeypointGridBounds& left, const KeypointGridBounds& right) {
  return sameFloat(left.left, right.left) && sameFloat(left.top, right.top) && sameFloat(left.right, right.right) && sameFloat(left.bottom, right.bottom);
}

KeypointGridBounds keypointGridBounds(const cv::Size& imageBoundary, const std::vector<camera::Perspective>& models,
                                      const std::vector<cv::KeyPoint>& keypoints) {
  KeypointGridBounds bounds;
  if (imageBoundary.width > 0 && imageBoundary.height > 0) {
    bounds.right = static_cast<float>(imageBoundary.width);
    bounds.bottom = static_cast<float>(imageBoundary.height);
  }

  bool validCameraBoundary = !models.empty();
  int cameraWidth = 0;
  int cameraHeight = 0;
  for (const camera::Perspective& model : models) {
    const cv::Size size = model.imageSize();
    if (size.width <= 0 || size.height <= 0) {
      validCameraBoundary = false;
      break;
    }
    cameraWidth += size.width;
    cameraHeight = std::max(cameraHeight, size.height);
  }
  if (validCameraBoundary) {
    bounds.right = std::max(bounds.right, static_cast<float>(cameraWidth));
    bounds.bottom = std::max(bounds.bottom, static_cast<float>(cameraHeight));
  }

  for (const cv::KeyPoint& keypoint : keypoints) {
    if (std::isfinite(keypoint.pt.x) && std::isfinite(keypoint.pt.y)) {
      bounds.left = std::min(bounds.left, keypoint.pt.x);
      bounds.top = std::min(bounds.top, keypoint.pt.y);
      bounds.right = std::max(bounds.right, keypoint.pt.x);
      bounds.bottom = std::max(bounds.bottom, keypoint.pt.y);
    }
  }
  return bounds;
}

}  // namespace

class KeypointGrid2D {
 public:
  KeypointGrid2D(const std::vector<cv::KeyPoint>& keypoints, const KeypointGridBounds& bounds) : bounds_(bounds) {
    points_.reserve(keypoints.size());
    for (std::size_t i = 0; i < keypoints.size(); ++i) {
      points_.push_back(keypoints[i].pt);
      if (std::isfinite(keypoints[i].pt.x) && std::isfinite(keypoints[i].pt.y)) {
        const int x = cellCoordinate(keypoints[i].pt.x);
        const int y = cellCoordinate(keypoints[i].pt.y);
        cells_[cellKey(x, y)].push_back(i);
        minCellX_ = std::min(minCellX_, x);
        minCellY_ = std::min(minCellY_, y);
        maxCellX_ = std::max(maxCellX_, x);
        maxCellY_ = std::max(maxCellY_, y);
      }
    }
  }

  bool matches(const std::vector<cv::KeyPoint>& keypoints, const KeypointGridBounds& bounds) const {
    if (!sameBounds(bounds_, bounds) || points_.size() != keypoints.size()) {
      return false;
    }
    for (std::size_t i = 0; i < points_.size(); ++i) {
      if (!sameFloat(points_[i].x, keypoints[i].pt.x) || !sameFloat(points_[i].y, keypoints[i].pt.y)) {
        return false;
      }
    }
    return true;
  }

  std::vector<std::size_t> radiusSearch(const cv::Point2f& point, float radius) const {
    std::vector<std::size_t> result;
    if (cells_.empty() || radius < 0.0f || !std::isfinite(radius) || !std::isfinite(point.x) || !std::isfinite(point.y)) {
      return result;
    }

    const int firstX = std::max(minCellX_, cellCoordinate(point.x - radius));
    const int lastX = std::min(maxCellX_, cellCoordinate(point.x + radius));
    const int firstY = std::max(minCellY_, cellCoordinate(point.y - radius));
    const int lastY = std::min(maxCellY_, cellCoordinate(point.y + radius));
    const double radiusSquared = static_cast<double>(radius) * radius;
    std::vector<std::pair<double, std::size_t>> sorted;
    for (std::int64_t y = firstY; y <= lastY; ++y) {
      for (std::int64_t x = firstX; x <= lastX; ++x) {
        const auto cell = cells_.find(cellKey(static_cast<int>(x), static_cast<int>(y)));
        if (cell == cells_.end()) {
          continue;
        }
        for (std::size_t index : cell->second) {
          const double dx = static_cast<double>(points_[index].x) - point.x;
          const double dy = static_cast<double>(points_[index].y) - point.y;
          const double distanceSquared = dx * dx + dy * dy;
          if (distanceSquared <= radiusSquared) {
            sorted.emplace_back(distanceSquared, index);
          }
        }
      }
    }
    std::sort(sorted.begin(), sorted.end(),
              [](const auto& left, const auto& right) { return left.first < right.first || (left.first == right.first && left.second < right.second); });
    result.reserve(sorted.size());
    for (const auto& item : sorted) {
      result.push_back(item.second);
    }
    return result;
  }

  bool collect_indices(const cv::Point2f& point, float radius, std::size_t max_visits,
                       std::vector<std::size_t>& output, std::size_t& visits) const {
    output.clear();
    visits = 0;
    if (cells_.empty() || radius < 0 || !std::isfinite(radius) || !std::isfinite(point.x) || !std::isfinite(point.y))
      return true;
    const int first_x = std::max(minCellX_, cellCoordinate(point.x - radius));
    const int last_x = std::min(maxCellX_, cellCoordinate(point.x + radius));
    const int first_y = std::max(minCellY_, cellCoordinate(point.y - radius));
    const int last_y = std::min(maxCellY_, cellCoordinate(point.y + radius));
    const double radius_squared = double(radius) * radius;
    for (std::int64_t y = first_y; y <= last_y; ++y) {
      for (std::int64_t x = first_x; x <= last_x; ++x) {
        const auto cell = cells_.find(cellKey(static_cast<int>(x), static_cast<int>(y)));
        if (cell == cells_.end()) continue;
        for (const auto index : cell->second) {
          if (visits == max_visits) {
            output.clear();
            return false;
          }
          ++visits;
          const double dx = double(points_[index].x) - point.x;
          const double dy = double(points_[index].y) - point.y;
          if (dx * dx + dy * dy <= radius_squared) output.push_back(index);
        }
      }
    }
    return true;
  }

  unsigned long getMemoryUsed() const {
    unsigned long memory = sizeof(KeypointGrid2D) + points_.capacity() * sizeof(cv::Point2f) + cells_.bucket_count() * sizeof(void*);
    for (const auto& cell : cells_) {
      memory += sizeof(decltype(cells_)::value_type) + cell.second.capacity() * sizeof(std::size_t);
    }
    return memory;
  }

 private:
  static constexpr float kCellSize = 32.0f;

  static int cellCoordinate(float coordinate) {
    const double cell = std::floor(static_cast<double>(coordinate) / kCellSize);
    if (cell <= std::numeric_limits<int>::min()) {
      return std::numeric_limits<int>::min();
    }
    if (cell >= std::numeric_limits<int>::max()) {
      return std::numeric_limits<int>::max();
    }
    return static_cast<int>(cell);
  }

  static std::uint64_t cellKey(int x, int y) { return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) | static_cast<std::uint32_t>(y); }

  KeypointGridBounds bounds_;
  std::vector<cv::Point2f> points_;
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> cells_;
  int minCellX_ = std::numeric_limits<int>::max();
  int minCellY_ = std::numeric_limits<int>::max();
  int maxCellX_ = std::numeric_limits<int>::min();
  int maxCellY_ = std::numeric_limits<int>::min();
};

VisualFrame::VisualFrame() = default;

void VisualFrame::setImageBoundary(const cv::Size& size) {
  EXT_ASSERT_MSG(size.width >= 0 && size.height >= 0, "Visual frame image boundary cannot be negative.");
  if (_imageBoundary != size) {
    _imageBoundary = size;
    if (_keypointGrid) updateKeypointGrid();
  }
}

void VisualFrame::setPerspective(const camera::Perspective& model) {
  perspectives_ = std::make_shared<const std::vector<camera::Perspective>>(1, model);
  if (_keypointGrid) updateKeypointGrid();
}

void VisualFrame::setPerspectives(const std::vector<camera::Perspective>& models) {
  perspectives_ = models.empty() ? nullptr : std::make_shared<const std::vector<camera::Perspective>>(models);
  if (_keypointGrid) updateKeypointGrid();
}

void VisualFrame::setPerspectives(std::vector<camera::Perspective>&& models) {
  perspectives_ = models.empty() ? nullptr : std::make_shared<const std::vector<camera::Perspective>>(std::move(models));
  if (_keypointGrid) updateKeypointGrid();
}

void VisualFrame::setFeatures(const std::vector<cv::KeyPoint>& keypoints, const std::vector<cv::Point3f>& keypoints3D, const cv::Mat& descriptors) {
  set_features_owned(std::vector<cv::KeyPoint>(keypoints), std::vector<cv::Point3f>(keypoints3D), descriptors.clone());
}

void VisualFrame::set_features_owned(std::vector<cv::KeyPoint>&& keypoints, std::vector<cv::Point3f>&& keypoints3D, cv::Mat&& descriptors) {
  EXT_ASSERT_MSG(keypoints3D.empty() || keypoints.size() == keypoints3D.size(), "{}",
                 fmt::format("keypoints={} keypoints3D={}", static_cast<int>(keypoints.size()), static_cast<int>(keypoints3D.size())));
  EXT_ASSERT_MSG(descriptors.empty() || static_cast<int>(keypoints.size()) == descriptors.rows, "{}",
                 fmt::format("keypoints={} descriptors={}", static_cast<int>(keypoints.size()), descriptors.rows));
  auto features = std::make_shared<VisualFeatures>();
  features->keypoints = std::move(keypoints);
  features->points = std::move(keypoints3D);
  features->descriptors = std::move(descriptors);
  features_ = std::move(features);
  if (_keypointGrid) updateKeypointGrid();
}

std::vector<std::size_t> VisualFrame::keypointIndicesInRadius(const cv::Point2f& point, float radius) const {
  if (!_keypointGrid) updateKeypointGrid();
  return _keypointGrid ? _keypointGrid->radiusSearch(point, radius) : std::vector<std::size_t>();
}

bool VisualFrame::collect_keypoint_indices(const cv::Point2f& point, float radius, std::size_t max_visits,
                                           std::vector<std::size_t>& output, std::size_t& visits) const {
  if (!_keypointGrid) updateKeypointGrid();
  return _keypointGrid->collect_indices(point, radius, max_visits, output, visits);
}

void VisualFrame::share_features(std::shared_ptr<const VisualFeatures> features) {
  features_ = std::move(features);
  _keypointGrid.reset();
}

void VisualFrame::updateKeypointGrid() const {
  const KeypointGridBounds bounds = keypointGridBounds(_imageBoundary, perspectives(), keypoints());
  if (!_keypointGrid || !_keypointGrid->matches(keypoints(), bounds)) {
    _keypointGrid = std::make_shared<const KeypointGrid2D>(keypoints(), bounds);
  }
}

unsigned long VisualFrame::getMemoryUsed() const {
  return sizeof(VisualFrame) + (features_ ? features_->memory_size() : 0) +
      perspectives().capacity() * sizeof(camera::Perspective) + (_keypointGrid ? _keypointGrid->getMemoryUsed() : 0);
}

int VisualFrame::isPointVisibleFromCameras(const cv::Point3f& pt) const {
  if (!perspectives().empty()) {
    for (unsigned int i = 0; i < perspectives().size(); ++i) {
      if (perspectives()[i].isValidForProjection() && !perspectives()[i].cameraToBase().isNull()) {
        const StateGroupf cameraFromBase = perspectives()[i].cameraToBase().inverse();
        const StateGroupf::Vector3 transformed = cameraFromBase.transformPoint(StateGroupf::Vector3(pt.x, pt.y, pt.z));
        const cv::Point3f ptInCameraFrame(transformed.x(), transformed.y(), transformed.z());
        const std::optional<cv::Point2d> pixel = perspectives()[i].project(ptInCameraFrame);
        if (pixel && perspectives()[i].isInImage(pixel->x, pixel->y)) {
          return i;
        }
      }
    }
  } else {
    EXT_LOG_ERROR("no valid camera model!");
  }
  return -1;
}

}  // namespace sapphire::backend
