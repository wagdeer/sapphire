/**
 * Derived from G3Reg, Zhijian QIAO / HKUST Aerial Robotics.
 * Sapphire retains the serial FPFH/PAGOR/GNC route.
 */
#include "reglib.h"

#include <stdexcept>

#include "geo_verify.h"
#include "registration.h"
#include "fpfh_utils.h"
#include "tic_toc.h"
namespace g3reg {
FRGresult GlobalRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
                             const Config& config) {
  config.validate();
  if (!source || !target) throw std::invalid_argument("Null G3Reg point cloud");
  for (const auto* cloud : {source.get(), target.get()})
    for (const auto& point : *cloud)
      if (!point.getVector3fMap().allFinite()) throw std::invalid_argument("Non-finite G3Reg point");
  FRGresult result;
  if (source->size() < 3 || target->size() < 3) return result;
  robot_utils::TicToc timer, features;
  std::vector<clique_solver::GraphVertex::Ptr> src_nodes, tgt_nodes;
  const auto associations = fpfh::matching(source, target, src_nodes, tgt_nodes, config);
  double feature_time = features.toc();
  result = pagor::PyramidRegistrationSolver(config).solve(src_nodes, tgt_nodes, associations);
  result.feature_time = feature_time;
  if (!result.candidates.empty()) {
    robot_utils::TicToc verify;
    result.tf = GeometryVerify(source, target, result.candidates, config);
    result.valid = result.valid && result.tf.allFinite();
    result.verify_time = verify.toc();
  }
  result.total_time = timer.toc();
  return result;
}
}  // namespace g3reg
