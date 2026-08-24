#pragma once

#include "parameters.h"
#include "ros2_utils.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <spdlog/spdlog.h>

using namespace std;

enum LID_TYPE{LIVOX, ROBOSENSE};

namespace livox_pcl {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      double timestamp;
      std::uint8_t line;
      std::uint8_t tag;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace livox_pcl
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_pcl::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (std::uint8_t, line, line)
    (std::uint8_t, tag, tag)
)

namespace rslidar_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      std::uint16_t ring;
      double timestamp;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

class LidarProcessor {
public:
  int lidar_type = LIVOX;
  int point_filter_num = 3;
  double blind = 0.01;

  LidarProcessor() = default;
  LidarProcessor(const int lidar_type, const int point_filter_num, const double blind_lowerbound)
    : lidar_type(lidar_type), point_filter_num(point_filter_num), blind(blind_lowerbound) {}

  void configure(const SensorParameters &parameters) {
    lidar_type = parameters.lidar_type;
    point_filter_num = parameters.point_filter_num;
    blind = parameters.blind_squared;
  }

  // 处理点云，返回第一个点的时间戳
  double process(const PointCloud2ConstPtr &msg, std::vector<LidarPoint> &pl_full) {
    double t0 = stamp_to_sec(msg->header.stamp);
    switch(lidar_type) {
      case LIVOX:
        livox_handler(msg, pl_full);
        break;

      case ROBOSENSE:
        t0 = robosense_handler(msg, pl_full);
        break;

      default:
        spdlog::error("Unsupported lidar type: {}", lidar_type);
        exit(0);
    }
    return t0;
  }

  // livox 的第一个点的时间戳是雷达内部的时钟源
  void livox_handler(const PointCloud2ConstPtr &msg, std::vector<LidarPoint> &pl_full) {
    pcl::PointCloud<livox_pcl::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    if(plsize == 0) return;
    const double first_time = pl_orig.points.front().timestamp;
    for(int i=0; i<plsize; i++) {
      livox_pcl::Point &iter = pl_orig.points[i];
      LidarPoint ap;
      ap.x = iter.x;
      ap.y = iter.y;
      ap.z = iter.z;
      ap.intensity = iter.intensity;
      ap.time_offset = static_cast<float>((iter.timestamp - first_time) * 1e-9);   // livox timestamp is in nanoseconds.
      if(i % point_filter_num == 0) {
        if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z > blind) {
          pl_full.push_back(ap);
        }
      }
    }
  }

  // robosense 的第一个点的时间戳跟imu来自同一个时钟源，所以直接返回第一个点的时间戳
  double robosense_handler(const PointCloud2ConstPtr &msg, std::vector<LidarPoint> &pl_full) {
    pcl::PointCloud<rslidar_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    double t0 = pl_orig[0].timestamp;
    for(int i=0; i<plsize; i++) {
      LidarPoint ap;
      ap.x = pl_orig.points[i].x;
      ap.y = pl_orig.points[i].y;
      ap.z = pl_orig.points[i].z;
      ap.intensity = pl_orig.points[i].intensity;
      ap.time_offset = static_cast<float>(pl_orig[i].timestamp - t0);
      if(i % point_filter_num == 0) {
        if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z > blind) {
          pl_full.push_back(ap);
        }
      }
    }
    return t0;
  }
};