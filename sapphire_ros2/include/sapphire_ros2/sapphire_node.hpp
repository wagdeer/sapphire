#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <thread>

#include "lidar_preprocess.hpp"
#include "pipeline.hpp"

namespace sapphire_ros {

class SapphireNode : public rclcpp::Node {
 public:
  SapphireNode();
  ~SapphireNode() override;

 private:
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &message);
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message);
  void finish_callback();
  void enqueue_output(std::function<void()> task);
  void output_loop();
  void stop_output();

  void publish_odom(const StateGroup &state);
  void publish_scan(std::shared_ptr<const sapphire::vvec<double, 3>> points);
  void publish_trajectory(std::shared_ptr<const std::vector<TrajectoryPoint>> trajectory);
  void publish_local_map(std::shared_ptr<const sapphire::vvec<double, 3>> points);
  void publish_map_odom(const Eigen::Isometry3d &transform);
  void publish_map_pose(const Eigen::Isometry3d &transform, double timestamp);
  void publish_navigation_grid(std::shared_ptr<const NavigationGrid> grid);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cmap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;

  rclcpp::TimerBase::SharedPtr finish_timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::unique_ptr<LidarProcessor> lidar_processor_;
  std::unique_ptr<sapphire::SlamPipeline> pipeline_;

  std::mutex output_mutex_;
  std::condition_variable output_cv_;
  std::deque<std::function<void()>> output_queue_;
  std::thread output_thread_;
  bool output_stopping_ = false;

  size_t last_map_revision_ = 0;

  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_link";

  std::atomic_bool shutdown_started_{false};
};

}  // namespace sapphire_ros
