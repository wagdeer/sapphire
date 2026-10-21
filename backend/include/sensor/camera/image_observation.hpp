#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <opencv2/core/mat.hpp>
#include <utility>
#include <vector>

namespace sapphire {

struct ImageProjection {
  int width = 0, height = 0;
  enum class Model : std::uint8_t { Pinhole = 0, Kb4 = 1, Ucm = 2 };
  std::array<double, 4> intrinsics{}; // fx, fy, cx, cy
  Model model = Model::Pinhole;
  // Pinhole: empty or k1,k2,p1,p2[,k3]; KB4: k1..k4; UCM: alpha.
  std::vector<double> distortion;
};

// Appearance observation only. No LiDAR-clock pose, tracks or derived features.
class VisualFrame final {
 public:
  explicit VisualFrame(double timestamp, std::size_t camera_id = 0) : timestamp_(timestamp), camera_id_(camera_id) {}

  VisualFrame(const VisualFrame &) = delete;
  VisualFrame &operator=(const VisualFrame &) = delete;
  VisualFrame(VisualFrame &&) noexcept = default;
  VisualFrame &operator=(VisualFrame &&) noexcept = default;

  std::size_t camera_id() const noexcept { return camera_id_; }
  // Camera-local time; archive-loaded images use -1 because it is not persisted.
  double timestamp() const noexcept { return timestamp_; }
  // Internal owned payload. PNG validation belongs to the storage reader.
  void set_image_png(std::vector<std::uint8_t> bytes) { image_png_ = std::move(bytes); }
  const std::vector<std::uint8_t> &image_png() const noexcept { return image_png_; }
  bool has_image() const noexcept { return !image_png_.empty(); }
  static constexpr std::size_t max_image_bytes = 2 * 1024 * 1024;
  static constexpr std::size_t max_image_pixels = 1024 * 1024;
  const std::optional<ImageProjection> &projection() const noexcept { return projection_; }
  void set_projection(ImageProjection projection) { projection_ = std::move(projection); }

 private:
  double timestamp_;
  std::size_t camera_id_ = 0;
  std::vector<std::uint8_t> image_png_;
  std::optional<ImageProjection> projection_;
};

}  // namespace sapphire
