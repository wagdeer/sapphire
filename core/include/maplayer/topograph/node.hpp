#pragma once
#include <common/se3_state.hpp>
#include <maplayer/topograph/link.hpp>
#include <list>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include "common/common_type.hpp"

namespace sapphire {

// Graph node with immutable node-local geometry and an optimizable graph pose.
// Shared cloud snapshots are immutable. New nodes are detached until
// Memory admits them; persisted nodes carry a database ID and map identity.
class Node {
 public:
  Node();
  Node(GaussianCloudPtr cloud, double stamp, const SE3f& odom_pose);

 private:
  GaussianCloudPtr lidar_cloud_;

 public:
  Node(int id, int mapId = -1, int weight = 0, double stamp = 0.0, const std::string& label = std::string(), const SE3f& pose = SE3f());

  bool has_lidar() const { return lidar_cloud_ && !lidar_cloud_->empty(); }
  const GaussianCloudPtr& lidar_cloud() const { return lidar_cloud_; }
  void set_lidar_cloud(GaussianCloudPtr cloud) {
    if (cloud) validateGaussianCloud(*cloud);
    _geometryModified = _modified = true;
    lidar_cloud_ = std::move(cloud);
  }
  bool isBadNode() const;

  int id() const { return _id; }

  // In-process writer fence. Never part of the submap or its archive.
  uint64_t write_epoch = 0;

  int mapId() const { return _mapId; }

  void setWeight(int weight) {
    _modified = _modified || _weight != weight;
    _weight = weight;
  }

  int getWeight() const { return _weight; }

  void setLabel(const std::string& label) {
    _modified = _modified || _label.compare(label) != 0;
    _label = label;
  }

  const std::string& getLabel() const { return _label; }

  double getStamp() const { return _stamp; }

  void addLinks(const std::list<Link>& links);

  void addLinks(const std::map<int, Link>& links);

  void addLink(const Link& link);

  bool hasLink(int idTo, std::optional<Link::Type> type = std::nullopt) const;

  void changeLinkIds(int idFrom, int idTo);

  void removeLinks(bool keepSelfReferringLinks = false);

  void removeLink(int idTo);

  void setSaved(bool saved) { _saved = saved; }

  void setModified(bool modified) {
    _modified = modified;
    _linksModified = modified;
    if (!modified) _geometryModified = false;
  }

  const std::multimap<int, Link>& getLinks() const { return _links; }

  bool isSaved() const { return _saved; }

  bool isModified() const { return _modified || _linksModified; }

  bool isGeometryModified() const { return _geometryModified; }
  bool isLinksModified() const { return _linksModified; }

  void setPose(const SE3f& pose) { _pose = pose; }

  void setVelocity(float vx, float vy, float vz, float vroll, float vpitch, float vyaw) {
    _velocity = std::vector<float>(6, 0);
    _velocity[0] = vx;
    _velocity[1] = vy;
    _velocity[2] = vz;
    _velocity[3] = vroll;
    _velocity[4] = vpitch;
    _velocity[5] = vyaw;
  }

  const SE3f& getPose() const { return _pose; }

  const std::vector<float>& getVelocity() const { return _velocity; }

  unsigned long getMemoryUsed() const;

 private:
  friend class Memory;
  void assignIdentity(int id, int map_id);

  int _id;                          ///< Unique node ID (0 if invalid)
  int _mapId;                       ///< Map ID this node belongs to (-1 if no map)
  double _stamp;                    ///< Timestamp in seconds
  std::multimap<int, Link> _links;  ///< Links to other nodes (target ID -> Link, allows multiple links per target)
  int _weight;                      ///< Weight/importance of this node
  std::string _label;               ///< Optional label/name for this node
  bool _geometryModified = false;
  bool _saved;                      ///< Flag indicating if node is saved to database
  bool _modified;                   ///< Flag indicating if node data has been modified
  bool _linksModified;              ///< Flag indicating if links have been modified (optimization for database updates)

  SE3f _pose;             ///< Original T_odom_base anchor; optimized map poses belong to Topograph.
  std::vector<float> _velocity;  ///< 6DOF velocity [vx, vy, vz, vroll, vpitch, vyaw]
};

}  // namespace sapphire
