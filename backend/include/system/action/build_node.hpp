#pragma once
#include <optional>
#include <backend/common/common_type.hpp>
#include "backend/maplayer/featmap/refinement.hpp"

#include <backend/system/action/build_frame.hpp>
#include <backend/maplayer/node.hpp>
#include <backend/maplayer/featmap/merge.hpp>
#include <backend/maplayer/featmap/region_merge.hpp>
#include <backend/system/action/visual_registration.hpp>

namespace sapphire::backend {
class Memory;
class ArchiveWriter;
class LocalGridMaker;
class Statistics;
class VisualRegistration;
class Relocalizer;
class LocalMapCollection;
class FbowIndex;

struct NodeBuildOptions {
  int max_features = 0;
  bool raw_features = false;
  bool keep_intermediate = false;
  bool use_odometry_gravity = false;
};

// Populate a new private draft whose identity, pose and calibration are supplied
// by the owner. Consumes feature arrays, retains no input packet, and preserves
// any optional navigation layer already attached to the draft. Vocabulary access
// is read-only: residency, index admission and persistence belong to the caller.
void build_node(Node& draft, FrameBuilder::Result& frame, const SensorData& input,
                  const NodeBuildOptions& options, const FbowIndex* vocabulary,
                  const std::vector<double>& pixel_variances, double* quantization_ms = nullptr);

int rehearse_node(Memory&, const Node*, const VisualRegistration&, Statistics* = nullptr);

// Algorithm facts are returned directly. Statistics is an optional diagnostic sink.
struct AdmissionResult {
  bool accepted = false;
  int rehearsed_id = 0;
  std::map<int, int> reduced_ids;
};
AdmissionResult admit_frame(Memory&, FrameBuilder&, const VisualRegistration&, ArchiveWriter&, const LocalGridMaker&,
                            const SensorData&, const StateGroupf&, const Mat66_t<double>&,
                            const std::vector<float>& = {}, Statistics* = nullptr,
                            FrameBuilder::Result* prepared = nullptr, bool allow_rehearsal = true);

// Owner-thread business operations over resident/cold immutable evidence.
struct NodeMergeCandidate {
  bool refresh = false;
  StateGroupf relative;
  std::optional<Mat66_t<double>> covariance;  // BA estimate at relative; empty when no replacement measurement is available.
  std::shared_ptr<const Node> replacement, primary_replacement;
  int region = 0;
  cv::Mat expected_region, expected_target, expected_source;
  bool redundant_submap = false;
  bool shared_geometry_redundant = false;
  bool association_complete = false;
  featmap::NodeObservations bundle;
  size_t bundle_views = 0;
  double bundle_ms = 0;
  size_t bundle_measurements = 0, bundle_survivors = 0;
  uint64_t bundle_identity_digest = 1469598103934665603ULL;

  int retained = 0, removed = 0;
  std::shared_ptr<const featmap::LandmarkMap> target;
  VisualFrame query;
  std::shared_ptr<const bow::FeatureBlock> query_features;
  std::vector<featmap::Match> inliers;
  std::vector<uint32_t> rejected_rows;
  featmap::GeometryOptions geometry;
  featmap::AppearanceOptions appearance;
  sapphire::backend::featmap::NodeBundleOptions bundle_options;
};

// Prepare optional content fusion for a verified loop. No publication or persistence.
// No candidate means this measurement can constrain the graph without contracting content.
std::optional<NodeMergeCandidate> prepare_node_fusion(Memory&, const Relocalizer&, const Node& query, int retained,
                                                     const Registration&, const std::map<int, StateGroupf>& poses,
                                                     int previous_capture, const LocalMapCollection* navimap);

NodeMergeCandidate prepare_node_merge(const Memory&, const Relocalizer&, int retained, int removed, const Registration&);
NodeMergeCandidate prepare_node_refresh(const Memory&, const Relocalizer&, int retained, int query, const Registration&, const std::map<int, StateGroupf>& poses);
void prepare_node_previous(Memory&, const Relocalizer&, NodeMergeCandidate&, int previous_keyframe, const std::map<int, StateGroupf>& poses);
NodeMergeCandidate prepare_node_regions(Memory&, const Relocalizer&, int retained, int removed, const StateGroupf&,
                                                     const std::map<uint64_t, uint64_t>& correspondences);
featmap::MergeDraft prepare_node_content(const NodeMergeCandidate&, const StateGroupf&);

}  // namespace sapphire::backend
