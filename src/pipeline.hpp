#pragma once

#include "common.hpp"
#include "imu_estimator.hpp"
#include "voxel_map.hpp"
#include "eskf.hpp"
#include "lidar_preprocess.hpp"
#include "lio_frame.hpp"
#include "parameters.h"
#include "ros2_utils.hpp"

#include <atomic>
#include <cmath>
#include <malloc.h>
#include <mutex>
#include <optional>
#include <spdlog/spdlog.h>
#include <utility>

#include <Eigen/Eigenvalues>

using namespace std;

class Synchronizer
{
private:
  mutex mutex_;
  deque<ImuMeas> imu_buf_;
  deque<std::shared_ptr<std::vector<LidarPoint>>> pcl_buf_;
  deque<double> pcl_time_buf_;
  MeasGroup pending_;

public:
  void push_imu(ImuMeas &&imu)
  {
    lock_guard<mutex> lock(mutex_);
    imu_buf_.push_back(std::move(imu));
  }

  void push_lidar(double timestamp, std::shared_ptr<std::vector<LidarPoint>> cloud)
  {
    lock_guard<mutex> lock(mutex_);
    pcl_time_buf_.push_back(timestamp);
    pcl_buf_.push_back(std::move(cloud));
  }

  bool sync_packages(MeasGroup &measures)
  {
    if(!pending_.lidar_cloud)
    {
      {
        lock_guard<mutex> lock(mutex_);
        if(pcl_buf_.empty()) return false;
        pending_.lidar_cloud = std::move(pcl_buf_.front());
        pending_.lidar_begin_time = pcl_time_buf_.front();
        pcl_buf_.pop_front();
        pcl_time_buf_.pop_front();
      }
      pending_.lidar_end_time = pending_.lidar_begin_time + pending_.lidar_cloud->back().time_offset;
    }

    {
      lock_guard<mutex> lock(mutex_);
      if(imu_buf_.empty() || imu_buf_.back().timestamp <= pending_.lidar_end_time)
        return false;

      while(!imu_buf_.empty())
      {
        if(imu_buf_.front().timestamp > pending_.lidar_end_time) break;
        pending_.imu_buf.push_back(std::move(imu_buf_.front()));
        imu_buf_.pop_front();
      }
    }

    measures = std::move(pending_);
    pending_.clear();
    return measures.imu_buf.size() > 4;
  }
};

LidarProcessor lidarproc_;
Synchronizer synchronizer;

void imu_handler(const ImuConstPtr msg_in)
{
  ImuMeas msg;
  msg.timestamp = stamp_to_sec(msg_in->header.stamp);
  msg.gyro << msg_in->angular_velocity.x, msg_in->angular_velocity.y, msg_in->angular_velocity.z;
  msg.accel << msg_in->linear_acceleration.x, msg_in->linear_acceleration.y, msg_in->linear_acceleration.z;

  static bool first_message = true;
  if(first_message)
  {
    first_message = false;
    spdlog::info("Time0: {}", msg.timestamp);
  }

  synchronizer.push_imu(std::move(msg));
}

void pcl_handler(const PointCloud2ConstPtr msg)
{
  auto pl_ptr = make_shared<std::vector<LidarPoint>>();
  const double t0 = lidarproc_.process(msg, *pl_ptr);

  if(pl_ptr->empty())
  {
    LidarPoint point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    point.intensity = 0;
    point.time_offset = 0.0f;
    pl_ptr->push_back(point);
    point.time_offset = 0.09f;
    pl_ptr->push_back(point);
  }

  sort(pl_ptr->begin(), pl_ptr->end(), [](const LidarPoint &x, const LidarPoint &y)
  {
    return x.time_offset < y.time_offset;
  });
  while(pl_ptr->back().time_offset > 0.11f)
  {
    pl_ptr->pop_back();
  }
  synchronizer.push_lidar(t0, std::move(pl_ptr));
}

