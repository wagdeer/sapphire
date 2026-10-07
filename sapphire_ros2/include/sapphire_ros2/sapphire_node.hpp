#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <condition_variable>
#include <array>
#include <optional>
#include <functional>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <thread>

#include "img_preprocess.hpp"
#include "lidar_preprocess.hpp"
#include "lidar_sync.hpp"
#include "pipeline.hpp"
#include "backend/grid/rolling_grid.hpp"
#include "sapphire_ros2/msg/revisioned_map_correction.hpp"
#include "sapphire_ros2/msg/revisioned_map_pose.hpp"
#include "sapphire_ros2/msg/revisioned_navigation_grid.hpp"
#include "sapphire_ros2/msg/map_publication_status.hpp"

namespace sapphire_ros {

class SapphireNode : public rclcpp::Node {
 public:
  explicit SapphireNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void request_stop() noexcept { shutdown_started_.store(true); }
  bool stop_requested() const noexcept { return shutdown_started_.load(); }
  bool finish_checked();
  void retry_publication();
  const sapphire_ros2::msg::MapPublicationStatus &final_report() const {
    if (!finished_) throw std::logic_error("Checked finish has not completed");
    return final_status_;
  }
  ~SapphireNode() override;

 private:
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &message);
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message);
  void rear_lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message);
  void drain_lidar();
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &message);
  void compressed_image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &message);
  void finish_callback();
  friend struct SapphireNodeTestAccess;
  void enqueue_output(std::size_t slot, std::function<void()> task, std::size_t payload_bytes = 0);
  void observe_optional_copy(std::size_t conversion_bytes);
  void reset_output(bool begin);
  void attempt_map(bool final);
  void publish_status();
  sapphire_ros2::msg::MapPublicationStatus make_status();
  void publish_correction(const std::string &uuid, std::uint64_t revision, std::uint64_t generation,
                          std::uint64_t source, double timestamp, const Eigen::Isometry3d &correction,
                          const Eigen::Isometry3d &pose);
  void output_loop();
  void stop_output();

  void publish_odom(const sapphire::StateGroup &state);
  void publish_scan(std::shared_ptr<const sapphire::vvec<double, 3>> points);
  void publish_local_grid(const sapphire::vvec<double, 3>& points, const Eigen::Vector3d& origin, double stamp, std::uint64_t generation);
  void publish_trajectory(std::shared_ptr<const std::vector<sapphire::TrajectoryPoint>> trajectory);
  void publish_local_map(std::shared_ptr<const sapphire::vvec<double, 3>> points);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cmap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_grid_pub_;
  std::unique_ptr<sapphire::RollingGrid> local_grid_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rear_lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_, right_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr right_compressed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;

  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
  rclcpp::CallbackGroup::SharedPtr image_callback_group_;

  rclcpp::TimerBase::SharedPtr finish_timer_;
  rclcpp::TimerBase::SharedPtr lidar_timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  ImageProcessor image_processor_;
  std::unique_ptr<LidarProcessor> lidar_processor_;
  std::unique_ptr<LidarSync> lidar_sync_;
  size_t last_lidar_drop_count_ = 0;
  std::chrono::steady_clock::time_point last_lidar_report_{};
  std::unique_ptr<sapphire::SlamPipeline> pipeline_;

  std::mutex output_mutex_;
  std::condition_variable output_cv_;
  std::array<std::size_t, 4> output_slot_bytes_{};
  std::size_t active_optional_bytes_ = 0, optional_copy_high_water_ = 0;
  std::array<std::function<void()>, 4> output_slots_; // odom, scan, full trajectory, incremental local map
  std::atomic_bool map_dirty_{true}, retry_requested_{false};
  bool output_active_ = false, output_resetting_ = false, final_requested_ = false;
  bool mapping_ok_ = true, finished_ = false, finish_success_ = false;
  bool navigation_enabled_ = false, close_attempted_ = false, close_success_ = false;
  std::exception_ptr mapping_error_, close_error_;
  sapphire_ros2::msg::MapPublicationStatus final_status_;
  struct Group {
    std::optional<std::uint64_t> published, target;
    unsigned online_attempts = 0;
    std::chrono::steady_clock::time_point attempted{};
    std::exception_ptr first_error;
    std::string error;
  } correction_group_, navigation_group_;
  std::string map_uuid_, incarnation_;
  std::uint64_t generation_ = 0;
  std::optional<std::uint64_t> final_revision_, final_source_;
  std::chrono::steady_clock::duration output_period_ = std::chrono::seconds(1);
  std::chrono::steady_clock::time_point next_grid_{};
  std::atomic_uint64_t map_coalesced_{0}, ready_ns_{0};
  double ready_selection_ms_ = 0, selection_success_ms_ = 0, ready_success_ms_ = 0;
  std::size_t success_samples_ = 0, payload_copy_high_water_ = 0;
  double conversion_ms_ = 0, checked_publish_ms_ = 0;
  std::uint64_t coalesced_ = 0, optional_dropped_ = 0, incremental_dropped_ = 0, retries_ = 0;
  std::size_t pending_high_water_ = 0;
  double capture_ms_ = 0, grid_ms_ = 0, publication_ms_ = 0;
  rclcpp::Publisher<sapphire_ros2::msg::RevisionedMapCorrection>::SharedPtr correction_pub_;
  rclcpp::Publisher<sapphire_ros2::msg::RevisionedMapPose>::SharedPtr revisioned_pose_pub_;
  rclcpp::Publisher<sapphire_ros2::msg::RevisionedNavigationGrid>::SharedPtr revisioned_grid_pub_;
  rclcpp::Publisher<sapphire_ros2::msg::MapPublicationStatus>::SharedPtr status_pub_;
  std::thread output_thread_;
  bool output_stopping_ = false;


  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_link";

  std::atomic_bool shutdown_started_{false};
};

}  // namespace sapphire_ros
