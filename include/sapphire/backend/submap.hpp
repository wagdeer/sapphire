#pragma once

#include <memory>
#include <sapphire/backend/pose_graph.hpp>
#include <utility>
#include <vector>

namespace sapphire {

class SubmapArxiv;
class LocalPlaneExchange;

// Owned and called exclusively by the pose graph worker.
class Submap {
 public:
  Submap(SubmapArxiv& arxiv, const Config::MethodB& config);
  ~Submap();
  void build(InertialFrame frame);
  // Return true when enough keyframes and the next anchor are available.
  bool update(InertialFrame frame);
  // With a lookahead frame, keep that frame for the next window.
  // Without lookahead, seal the remaining frames at shutdown.
  void seal();
  // Reuse the tail as the next anchor.
  void reset();
  // Release only the active window. Sealed blocks remain available.
  void clear();
  PoseGraphStats stats() const;
  PoseGraphSnapshot snapshot() const;
  std::vector<std::pair<double, Isometry3d>> optimizedFrames() const;
  PointCloudPtr globalMap(double leaf) const;
  LocalPlaneExchange& localPlanes();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sapphire
