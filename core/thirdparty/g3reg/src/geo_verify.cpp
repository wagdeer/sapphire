/**
 * Derived from G3Reg, Zhijian QIAO / HKUST Aerial Robotics.
 * Sapphire retains the serial FPFH/PAGOR/GNC route.
 */
#include "geo_verify.h"

#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

#include "downsample.h"
Eigen::Matrix4d GeometryVerify(typename pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, typename pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud,
                               const std::vector<Eigen::Matrix4d>& candidates, const g3reg::Config& config) {
  if (candidates.size() == 1) {
    return candidates[0];
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
  double voxel_size = config.plane_resolution;
  pcl::voxelize<pcl::PointXYZ>(src_cloud, src_cloud_down, voxel_size);
  pcl::voxelize<pcl::PointXYZ>(tgt_cloud, tgt_cloud_down, voxel_size);

  // compute the chamfer distance
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(tgt_cloud_down);

  Eigen::Matrix4d best_pose = Eigen::Matrix4d::Identity();
  float threshold = voxel_size * 2;
  double best_score = INFINITY;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_src(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto tf : candidates) {
    double total_residual = 0.0, dist;
    pcl::transformPointCloud(*src_cloud_down, *transformed_src, tf);
    for (int i = 0; i < transformed_src->size(); ++i) {
      std::vector<int> pointIdxNKNSearch;
      std::vector<float> pointNKNSquaredDistance;
      tree->nearestKSearch(transformed_src->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
      total_residual +=
          threshold * threshold * (1 - std::exp(-pointNKNSquaredDistance[0] * pointNKNSquaredDistance[0] / (2 * threshold * threshold)));
    }
    total_residual = transformed_src->size() > 0 ? total_residual / transformed_src->size() : INFINITY;
    if (total_residual < best_score) {
      best_score = total_residual;
      best_pose = tf;
    }
  }
  return best_pose;
}