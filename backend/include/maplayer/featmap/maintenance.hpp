#pragma once
#include <backend/maplayer/featmap/observations.hpp>

namespace sapphire::backend::featmap {
struct Match;
std::shared_ptr<const LandmarkMap> representative_descriptors(const LandmarkMap&, const NodeObservations&, const std::map<int, StateGroupf>&);
// Apply solved geometry and select representative appearances in one private
// point buffer; publish one immutable feature/index snapshot after both steps.
// Unsolved geometry follows its deterministic original observer reference; the
// updated history includes actual outlier removals and new query associations.
std::shared_ptr<const LandmarkMap> refined_node_features(const LandmarkMap&, const NodeObservations&, const NodeObservations& updated,
                                                          const std::map<uint64_t, cv::Point3f>& solved, const std::map<uint64_t, Mat33_t<double>>& covariances = {});
// VSLAM-style node admission: mature shared points already have three
// equal/better-scale source-submap observers. New coverage is checked by caller.
bool redundant_node_observations(const LandmarkMap&, const NodeObservations&, const bow::FeatureBlock&, const std::vector<Match>&,
                                   const std::vector<double>& variances, float pyramid_scale);

}  // namespace sapphire::backend::featmap
