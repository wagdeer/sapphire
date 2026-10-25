/**
 * Derived from G3Reg, Zhijian QIAO / HKUST Aerial Robotics.
 * Sapphire retains the serial FPFH/PAGOR/GNC route.
 */
#pragma once
#include "reglib.h"
Eigen::Matrix4d GeometryVerify(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                               const std::vector<Eigen::Matrix4d>& candidates, const g3reg::Config& config);
