/**
 * Derived from G3Reg, Zhijian QIAO / HKUST Aerial Robotics.
 * Sapphire retains the serial FPFH/PAGOR/GNC route.
 */
#pragma once
#include <pcl/filters/voxel_grid.h>

#include "tgs.hpp"
#include "config.h"
namespace pcl {
template <typename T>
void voxelize(const typename pcl::PointCloud<T>::Ptr& source, const typename pcl::PointCloud<T>::Ptr& target, double resolution) {
  pcl::VoxelGrid<T> filter;
  filter.setInputCloud(source);
  filter.setLeafSize(resolution, resolution, resolution);
  filter.filter(*target);
}
}  // namespace pcl
namespace travel {
template <typename T>
void estimateGround(pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& ground, pcl::PointCloud<T>& nonground, double& time,
                    const g3reg::Config& options) {
  TravelGroundSeg<T> segmenter;
  segmenter.setParams(options.max_range, options.min_range);
  segmenter.estimateGround(cloud, ground, nonground, time);
}
}  // namespace travel
