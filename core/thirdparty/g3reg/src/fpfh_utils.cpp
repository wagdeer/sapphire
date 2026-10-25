/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
//
#include "fpfh_utils.h"

#include <pcl/common/transforms.h>

#include "fpfh_matcher.h"
#include "downsample.h"

using namespace g3reg;

namespace fpfh {

void Match(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud, std::vector<std::pair<int, int>> &corres,
           const Config &config) {
  // KITTI parameters for FPFH while voxel downsampling resolution is 0.3
  double normal_radius = config.normal_radius;
  double fpfh_radius = config.fpfh_radius;

  // Compute FPFH
  teaser::FPFHEstimation fpfh;
  teaser::FPFHCloudPtr obj_descriptors = fpfh.computeFPFHFeatures(src_cloud, normal_radius, fpfh_radius);
  teaser::FPFHCloudPtr scene_descriptors = fpfh.computeFPFHFeatures(tgt_cloud, normal_radius, fpfh_radius);
  teaser::Matcher matcher;
  corres = matcher.calculateCorrespondences(src_cloud, tgt_cloud, *obj_descriptors, *scene_descriptors, true, true, false, 0.95);
}

clique_solver::Association matching(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
                                    std::vector<clique_solver::GraphVertex::Ptr> &src_nodes, std::vector<clique_solver::GraphVertex::Ptr> &tgt_nodes,
                                    const Config &config) {
  double tSrc, tTgt;
  pcl::PointCloud<pcl::PointXYZ> srcGround;
  pcl::PointCloud<pcl::PointXYZ> tgtGround;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptrSrcNonground(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTgtNonground(new pcl::PointCloud<pcl::PointXYZ>);
  travel::estimateGround(*(src_cloud), srcGround, *ptrSrcNonground, tSrc, config);
  travel::estimateGround(*(tgt_cloud), tgtGround, *ptrTgtNonground, tTgt, config);

  // voxelize
  const double voxel_size = config.ds_resolution;
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_ds(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_ds(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::voxelize<pcl::PointXYZ>(ptrSrcNonground, src_ds, voxel_size);
  pcl::voxelize<pcl::PointXYZ>(ptrTgtNonground, tgt_ds, voxel_size);

  if (src_ds->size() < 3 || tgt_ds->size() < 3) return clique_solver::Association(0, 2);
  std::vector<std::pair<int, int>> corres;
  Match(src_ds, tgt_ds, corres, config);
  clique_solver::Association assoc = clique_solver::Association::Zero(corres.size(), 2);
  for (int i = 0; i < corres.size(); i++) {
    assoc(i, 0) = corres[i].first;
    assoc(i, 1) = corres[i].second;
  }

  // construct the matching node pairs
  src_nodes.clear();
  tgt_nodes.clear();
  src_nodes.reserve(src_ds->size());
  tgt_nodes.reserve(tgt_ds->size());

  for (int i = 0; i < src_ds->size(); i++) {
    const Eigen::Vector3d &center = src_ds->at(i).getVector3fMap().cast<double>();
    src_nodes.push_back(clique_solver::create_vertex(center, config.vertex_info));
  }

  for (int i = 0; i < tgt_ds->size(); i++) {
    const Eigen::Vector3d &center = tgt_ds->at(i).getVector3fMap().cast<double>();
    tgt_nodes.push_back(clique_solver::create_vertex(center, config.vertex_info));
  }
  return assoc;
}
}  // namespace fpfh
