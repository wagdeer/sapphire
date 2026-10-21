#pragma once
#include <backend/common/state_group.hpp>
#include <backend/common/visual_frame.hpp>
#include <backend/maplayer/topograph/link.hpp>
#include <backend/maplayer/featmap.hpp>
#include <backend/maplayer/featmap/landmark_map.hpp>
#include <list>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <optional>
#include <set>
#include <vector>

#include "backend/sensor/lidar/geometry.hpp"

namespace sapphire::backend {
struct LocalMap;

// Persistent graph node containing one or more fused observations.
// Graph pose/links place its feature and optional navigation layers in the world;
// geometry and observation history remain in the node-local frame.
// Copies share layer snapshots until a private draft changes their content.
class Node {
 public:
  // One representative descriptor per independent natural landmark.
  // Capture measurements and the local BA budget are separate from this bank.
  static constexpr uint32_t max_natural_points = 512;

  Node();

 private:
  GaussianCloudPtr lidar_cloud_;

 public:
  Node(int id, int mapId = -1, int weight = 0, double stamp = 0.0, const std::string& label = std::string(), const StateGroupf& pose = StateGroupf(),
       VisualFrame frame = {});

  float compareTo(const Node& node) const;

  bool has_lidar() const { return lidar_cloud_ && !lidar_cloud_->empty(); }
  const GaussianCloudPtr& lidar_cloud() const { return lidar_cloud_; }
  void set_lidar_cloud(GaussianCloudPtr cloud) {
    if (cloud) lidar::validate(*cloud);
    lidar_cloud_ = std::move(cloud);
  }
  bool isBadNode() const;
  bool has_natural_features() const;

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
    if (!modified) {
      feature_modified_ = false;
    }
    _modified = modified;
    _linksModified = modified;
  }

  const std::multimap<int, Link>& getLinks() const { return _links; }

  bool isSaved() const { return _saved; }

  bool isModified() const { return _modified || _linksModified; }

  bool isLinksModified() const { return _linksModified; }

  void removeAllWords();

  void setWords(const std::multimap<int, int>& words, const std::vector<cv::KeyPoint>& keypoints, const std::vector<cv::Point3f>& words3,
                const cv::Mat& descriptors);

  void set_loop_features(std::shared_ptr<const bow::FeatureBlock> features);
  std::shared_ptr<const bow::FeatureBlock> loop_features() const { return feature_map_.loop_features(); }

  const featmap::FeatureMap& featmap() const { return feature_map_; }
  const std::shared_ptr<const LocalMap>& navimap() const { return navimap_map_; }
  void set_navimap(std::shared_ptr<const LocalMap> map);

  bool features_erased() const { return features_erased_; }
  bool feature_modified() const { return feature_modified_; }
  void mark_features_saved() { feature_modified_ = false; }
  bool has_map_features() const { return mapped_features_; }
  uint64_t feature_token() const { return feature_token_; }
  const std::shared_ptr<const featmap::LandmarkMap>& map_features() const { return feature_map_.landmarks(); }
  void set_map_features(std::shared_ptr<const featmap::LandmarkMap> features);
  void cache_map_features(std::shared_ptr<const featmap::LandmarkMap> features);
  void clear_feature_cache() { feature_map_.clear(); }
  // Point geometry and observation history share the same submap lifetime.
  // Mutation is performed on a draft; copied owners retain their old observations.
  bool has_observation_history() const { return feature_map_.has_history(); }
  const featmap::NodeObservations& observation_history() const { return feature_map_.history(); }
  void reject_observations(const std::vector<std::pair<int, uint64_t>>& rejected);
  void set_feature_content(std::shared_ptr<const featmap::LandmarkMap> features, featmap::NodeObservations history);
  void replace_landmark(uint64_t removed, uint64_t retained);
  void retire_unsupported(const std::set<uint64_t>& previously_observed);
  void fuse_landmarks(const VisualFrame& query, const std::shared_ptr<const bow::FeatureBlock>& query_features, const StateGroupf& query_from_map,
                      const std::vector<featmap::Match>& seeds);
  void maintain_landmarks(const VisualFrame&, const std::shared_ptr<const bow::FeatureBlock>&, int capture, const StateGroupf& query_from_map,
                          const std::vector<double>& pixel_variances, const std::map<uint64_t, uint32_t>& verified_rows = {});
  void prune_views(float pyramid_scale = 1.2f, const std::set<int>& protected_observers = {});
  void update_descriptors(const std::map<int, StateGroupf>& boundaries);
  void set_feature_summary(std::vector<bow::WordCount> words, uint64_t hash, uint32_t point_count);
  uint32_t retrieval_mass() const { return retrieval_mass_; }

  bool isEnabled() const { return _enabled; }

  void setEnabled(bool enabled) { _enabled = enabled; }

  const std::multimap<int, int>& getWords() const;
  const std::vector<bow::WordCount>& retrieval_words() const { return retrieval_words_; }
  uint32_t feature_count() const { return feature_count_; }
  uint64_t vocabulary_hash() const { return vocabulary_hash_; }
  void set_raw_summary(uint32_t count);
  void set_retrieval_words(std::vector<bow::WordCount> words, uint64_t vocabulary_hash);

  const std::vector<cv::KeyPoint>& getWordsKpts() const { return feature_map_.reference_frame().keypoints(); }

  int getInvalidWordsCount() const { return _invalidWordsCount; }

  const cv::Mat& getWordsDescriptors() const { return feature_map_.reference_frame().descriptors(); }

  void setPose(const StateGroupf& pose) { _pose = pose; }

  void setVelocity(float vx, float vy, float vz, float vroll, float vpitch, float vyaw) {
    _velocity = std::vector<float>(6, 0);
    _velocity[0] = vx;
    _velocity[1] = vy;
    _velocity[2] = vz;
    _velocity[3] = vroll;
    _velocity[4] = vpitch;
    _velocity[5] = vyaw;
  }

  const std::vector<cv::Point3f>& getWords3() const { return feature_map_.reference_frame().keypoints3D(); }

  const VisualFrame& visualFrame() const { return feature_map_.reference_frame(); }
  void set_reference_frame(VisualFrame frame);
  void set_calibration(std::vector<camera::Perspective> cameras) {
    auto frame = visualFrame();
    frame.setPerspectives(std::move(cameras));
    feature_map_.set_reference_frame(std::move(frame));
  }
  void clear_reference_features() { feature_map_.clear_reference_features(); }
  void set_global_descriptors(std::vector<GlobalDescriptor> descriptors) { feature_map_.set_global_descriptors(std::move(descriptors)); }

  const StateGroupf& getPose() const { return _pose; }

  const std::vector<float>& getVelocity() const { return _velocity; }

  unsigned long getMemoryUsed() const;

 private:
  int _id;                          ///< Unique node ID (0 if invalid)
  int _mapId;                       ///< Map ID this node belongs to (-1 if no map)
  double _stamp;                    ///< Timestamp in seconds
  std::multimap<int, Link> _links;  ///< Links to other nodes (target ID -> Link, allows multiple links per target)
  int _weight;                      ///< Weight/importance of this node
  std::string _label;               ///< Optional label/name for this node
  bool _saved;                      ///< Flag indicating if node is saved to database
  bool _modified;                   ///< Flag indicating if node data has been modified
  bool _linksModified;              ///< Flag indicating if links have been modified (optimization for database updates)

  bool features_erased_ = false;
  bool mapped_features_ = false;
  uint64_t feature_token_ = 0;  // Process-local incarnation, never serialized.
  bool feature_modified_ = false;
  featmap::FeatureMap feature_map_;
  std::shared_ptr<const LocalMap> navimap_map_;
  void sync_feature_summary(featmap::FeatureMap content);
  std::vector<bow::WordCount> retrieval_words_;
  uint32_t feature_count_ = 0;
  uint32_t retrieval_mass_ = 0;
  uint64_t vocabulary_hash_ = 0;
  mutable std::multimap<int, int> _words;  ///< Visual words: word ID -> feature index (multimap allows duplicates)
  bool _enabled;                           ///< Flag indicating if node is enabled for place recognition
  int _invalidWordsCount;                  ///< Count of invalid words (ID <= 0)

  StateGroupf _pose;             ///< Current pose in map coordinate frame
  std::vector<float> _velocity;  ///< 6DOF velocity [vx, vy, vz, vroll, vpitch, vyaw]
};

}  // namespace sapphire::backend
