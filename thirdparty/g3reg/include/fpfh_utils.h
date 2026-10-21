/**
 * Derived from G3Reg, Zhijian QIAO / HKUST Aerial Robotics.
 * Sapphire retains the serial FPFH/PAGOR/GNC route.
 */
#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "graph.h"
#include "config.h"
namespace fpfh {
clique_solver::Association matching(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                                    std::vector<clique_solver::GraphVertex::Ptr>& source_nodes,
                                    std::vector<clique_solver::GraphVertex::Ptr>& target_nodes, const g3reg::Config& config);
}
