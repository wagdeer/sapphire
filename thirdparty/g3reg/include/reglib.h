/**
 * Derived from G3Reg, Zhijian QIAO / HKUST Aerial Robotics.
 * Sapphire retains the serial FPFH/PAGOR/GNC route.
 */
#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "config.h"
namespace g3reg {
struct FRGresult {
  // T_target_source; no final loop-acceptance guarantee is implied by valid.
  Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
  std::vector<Eigen::Matrix4d> candidates;
  double feature_time = 0, graph_time = 0, clique_time = 0, tf_solver_time = 0, verify_time = 0, total_time = 0;
  bool valid = false;
};
FRGresult GlobalRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
                             const Config& options = Config{});
}  // namespace g3reg
