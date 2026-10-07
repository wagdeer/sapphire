#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <optional>
#include <sapphire_ros2/sapphire_node.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <random>
#include <sstream>
#include <iomanip>
#include <rcl/publisher.h>
#include "sapphire_ros2/checked_publish.hpp"
#include <rclcpp/serialization.hpp>

namespace sapphire_ros {
namespace {

template <typename T>
T parameter(rclcpp::Node &node, const std::string &name, const T &default_value) {
  return node.declare_parameter<T>(name, default_value);
}

builtin_interfaces::msg::Time stamp_from_seconds(double seconds) {
  return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)), RCL_ROS_TIME);
}

geometry_msgs::msg::TransformStamped make_transform(const builtin_interfaces::msg::Time &stamp, const char *parent, const char *child,
                                                    const Eigen::Isometry3d &transform) {
  geometry_msgs::msg::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = parent;
  message.child_frame_id = child;
  message.transform.translation.x = transform.translation().x();
  message.transform.translation.y = transform.translation().y();
  message.transform.translation.z = transform.translation().z();
  const Eigen::Quaterniond q(transform.rotation());
  message.transform.rotation.x = q.x();
  message.transform.rotation.y = q.y();
  message.transform.rotation.z = q.z();
  message.transform.rotation.w = q.w();
  return message;
}

sensor_msgs::msg::PointCloud2 point_cloud_message(const sapphire::vvec<double, 3> &points, const builtin_interfaces::msg::Time &stamp,
                                                  const char *frame, std::size_t *conversion_bytes = nullptr) {
  pcl::PointCloud<pcl::PointXYZINormal> cloud;
  cloud.reserve(points.size());
  for (const Eigen::Vector3d &point : points) {
    pcl::PointXYZINormal output;
    output.x = point.x();
    output.y = point.y();
    output.z = point.z();
    output.intensity = 0.0F;
    cloud.push_back(output);
  }
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.stamp = stamp;
  message.header.frame_id = frame;
  if (conversion_bytes) *conversion_bytes = cloud.points.capacity() * sizeof(pcl::PointXYZINormal) + message.data.capacity();
  return message;
}

}  // namespace

SapphireNode::SapphireNode(const rclcpp::NodeOptions &options) : Node("cmn_sapphire", options) {
  const bool finish = parameter(*this, "finish", false);
  if (finish) {
    RCLCPP_WARN(get_logger(), "finish is already true at startup");
  }

  const std::string package_share = ament_index_cpp::get_package_share_directory("sapphire_ros2");
  rcl_interfaces::msg::ParameterDescriptor mode_descriptor;
  mode_descriptor.read_only = true;
  mode_descriptor.description = "LiDAR input mode: single or dual";
  const std::string lidar_mode = declare_parameter<std::string>("lidar_mode", "single", mode_descriptor);
  if (lidar_mode != "single" && lidar_mode != "dual") {
    throw std::invalid_argument("lidar_mode must be single or dual");
  }
  const bool dual = lidar_mode == "dual";
  rcl_interfaces::msg::ParameterDescriptor obs_descriptor;
  obs_descriptor.read_only = true;
  obs_descriptor.description = "Observation mode: lio; livo is reserved for visual-inertial fusion";
  const std::string obs_mode = declare_parameter<std::string>("obs_mode", "lio", obs_descriptor);
  if (obs_mode != "lio" && obs_mode != "livo") {
    throw std::invalid_argument("obs_mode must be lio or livo");
  }
  if (obs_mode == "livo") {
    throw std::invalid_argument("obs_mode=livo is reserved: visual observations are not implemented yet; use lio");
  }
  const std::string algorithm_config = parameter(*this, "algorithm_config", package_share +
      (dual ? "/config/dual.toml" : "/config/mid360.toml"));
  const std::string lidar_topic = parameter(*this, "topics.lidar", std::string("/front_lidar"));
  const std::string rear_lidar_topic = parameter(*this, "topics.rear_lidar", std::string("/rear_lidar"));
  const std::string imu_topic = parameter(*this, "topics.imu", std::string("/front_lidar/imu"));
  const std::string image_topic = parameter(*this, "topics.image", std::string("/camera/image"));
  const bool compressed_image = parameter(*this, "image.compressed", false);
  const std::string scan_topic = parameter(*this, "topics.scan", std::string("/map_scan"));
  const std::string local_map_topic = parameter(*this, "topics.local_map", std::string("/map_cmap"));
  const std::string trajectory_topic = parameter(*this, "topics.trajectory", std::string("/map_path"));
  const std::string odom_topic = parameter(*this, "topics.odom", std::string("/odom"));
  const std::string map_pose_topic = parameter(*this, "topics.map_pose", std::string("/map_odom"));
  const std::string navigation_topic = parameter(*this, "topics.navigation", std::string("/map"));
  map_frame_ = parameter(*this, "frames.map", map_frame_);
  odom_frame_ = parameter(*this, "frames.odom", odom_frame_);
  base_frame_ = parameter(*this, "frames.base", base_frame_);
  const int imu_qos_depth = parameter(*this, "qos.imu_depth", 1000);
  const int lidar_qos_depth = parameter(*this, "qos.lidar_depth", 5);
  const int image_qos_depth = parameter(*this, "qos.image_depth", 2);
  if (imu_qos_depth <= 0 || lidar_qos_depth <= 0 || image_qos_depth <= 0) {
    throw std::invalid_argument("QoS depths must be positive");
  }

  sapphire::SapphireParameters parameters = sapphire::load_parameters(algorithm_config);
  const bool visual_enabled=parameters.pose_graph.visual.enabled;
  const bool stereo_enabled=visual_enabled && parameters.pose_graph.visual.mode=="stereo";
  const auto right_topic=parameter(*this,"topics.image_right",std::string("/camera/right/image"));
  rcl_interfaces::msg::ParameterDescriptor type_descriptor;
  type_descriptor.read_only = true;
  type_descriptor.description = "Point cloud format: livox, airy or hesai; defaults to sensor.lidar_type in the algorithm config";
  const std::string lidar_type = declare_parameter<std::string>("lidar_type", parameters.sensor.lidar_type == 0 ? "livox" : (parameters.sensor.lidar_type == 1 ? "airy" : "hesai"), type_descriptor);
  if (lidar_type != "livox" && lidar_type != "airy" && lidar_type != "hesai") {
    throw std::invalid_argument("lidar_type must be livox, airy or hesai");
  }
  if (dual && lidar_type != "airy") {
    throw std::invalid_argument("dual currently supports lidar_type=airy; other dual formats are not implemented");
  }
  parameters.sensor.lidar_type = lidar_type == "livox" ? 0 : (lidar_type == "airy" ? 1 : 2);
  if (dual) {
    if (get_node_topics_interface()->resolve_topic_name(lidar_topic) == get_node_topics_interface()->resolve_topic_name(rear_lidar_topic)) {
      throw std::invalid_argument("dual requires two distinct LiDAR topics");
    }
    LidarSyncParameters lidar_parameters;
    const auto translation = parameter(*this, "lidar.extrinsic_tran", std::vector<double>{});
    const auto rotation = parameter(*this, "lidar.extrinsic_rota", std::vector<double>{});
    if (translation.size() != 3 || rotation.size() != 9) {
      throw std::invalid_argument("Set lidar.extrinsic_tran (3) and extrinsic_rota (9 row-major values) from calibration");
    }
    lidar_parameters.extrinsic.translation() = Eigen::Map<const Eigen::Vector3d>(translation.data());
    lidar_parameters.extrinsic.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotation.data());
    lidar_parameters.front_time_offset = parameter(*this, "lidar.front_time_offset", 0.0);
    lidar_parameters.rear_time_offset = parameter(*this, "lidar.rear_time_offset", 0.0);
    lidar_parameters.max_time_gap = parameter(*this, "lidar.max_time_gap", 0.02);
    lidar_parameters.max_wait = parameter(*this, "lidar.max_wait", 0.3);
    rcl_interfaces::msg::ParameterDescriptor rate_descriptor;
    rate_descriptor.read_only = true;
    rate_descriptor.description = "Startup fusion rate in Hz, 0 for front-frame windows; does not change LiDAR hardware sampling";
    lidar_parameters.lidar_merge_hz = declare_parameter<double>("lidar.lidar_merge_hz", 10.0, rate_descriptor);
    lidar_parameters.imu_rate_hz = parameter(*this, "lidar.imu_rate_hz", 200.0);
    const int min_points = parameter(*this, "lidar.min_points", 1);
    if (min_points < 1) throw std::invalid_argument("lidar.min_points must be positive");
    lidar_parameters.min_points = static_cast<size_t>(min_points);
    const int queue_size = parameter(*this, "lidar.queue_size", 8);
    if (queue_size < 2) throw std::invalid_argument("lidar.queue_size must be >= 2");
    lidar_parameters.queue_size = static_cast<size_t>(queue_size);
    lidar_parameters.max_scan_duration = parameters.sensor.max_scan_duration;
    lidar_parameters.point_filter_num = parameters.sensor.point_filter_num;
    lidar_parameters.blind_squared = parameters.sensor.blind_squared;
    lidar_sync_ = std::make_unique<LidarSync>(lidar_parameters);
    RCLCPP_INFO(get_logger(), "dual fusion=%.3f Hz (0=front-frame), expected IMU=%.1f Hz; hardware scan rate unchanged",
                lidar_parameters.lidar_merge_hz, lidar_parameters.imu_rate_hz);
  } else {
    lidar_processor_ = std::make_unique<LidarProcessor>(parameters.sensor);
  }

  scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(scan_topic, rclcpp::QoS(10));
  cmap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(local_map_topic, rclcpp::QoS(2));
  path_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(trajectory_topic, rclcpp::QoS(2));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(100));
  map_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(map_pose_topic, rclcpp::QoS(10));
  if (parameters.navi_map.enabled)
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(navigation_topic, rclcpp::QoS(1).transient_local());
  if (parameter(*this, "local_grid.enabled", true)) {
    sapphire::RollingGrid::Parameters local;
    local.extent = parameter(*this, "local_grid.extent", local.extent);
    local.resolution = parameter(*this, "local_grid.resolution", local.resolution);
    local.max_age = parameter(*this, "local_grid.max_age", local.max_age);
    local.sensor_height = parameter(*this, "local_grid.sensor_height", local.sensor_height);
    local.obstacle_min_height = parameter(*this, "local_grid.obstacle_min_height", local.obstacle_min_height);
    local.obstacle_max_height = parameter(*this, "local_grid.obstacle_max_height", local.obstacle_max_height);
    local.raytrace = !dual; // Merged scans do not retain a per-point ray origin.
    local_grid_ = std::make_unique<sapphire::RollingGrid>(local);
    local_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        parameter(*this, "topics.local_grid", std::string("/sapphire/local_grid")), rclcpp::QoS(1));
    RCLCPP_INFO(get_logger(), "Local rolling grid: %.1fm, %.2fm cells, %.1fs expiry, ray clearing=%s",
                local.extent, local.resolution, local.max_age, local.raytrace ? "enabled" : "disabled (merged ray origins unavailable)");
  }
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  for (const auto *frame : {&map_frame_, &odom_frame_, &base_frame_})
    if (frame->empty() || frame->size() > 255) throw std::length_error("B3 frame identifier exceeds envelope");
  static const std::string process_incarnation = [] {
    std::random_device random; std::ostringstream text;
    for (int i = 0; i < 16; ++i) {
      if (i == 4 || i == 6 || i == 8 || i == 10) text << '-';
      unsigned byte = random() & 255U;
      if (i == 6) byte = (byte & 15U) | 64U;
      if (i == 8) byte = (byte & 63U) | 128U;
      text << std::hex << std::setw(2) << std::setfill('0') << byte;
    }
    return text.str();
  }();
  incarnation_ = process_incarnation;
  navigation_enabled_ = parameters.navi_map.enabled;
  output_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(parameters.pose_graph.update_period_sec));
  correction_pub_ = create_publisher<sapphire_ros2::msg::RevisionedMapCorrection>(
      parameter(*this, "revisioned_correction_topic", std::string("/sapphire/map_correction")), rclcpp::QoS(1));
  revisioned_pose_pub_ = create_publisher<sapphire_ros2::msg::RevisionedMapPose>(map_pose_topic + "/revisioned", rclcpp::QoS(1));
  if (navigation_enabled_)
    revisioned_grid_pub_ = create_publisher<sapphire_ros2::msg::RevisionedNavigationGrid>(navigation_topic + "/revisioned", rclcpp::QoS(1).transient_local());
  status_pub_ = create_publisher<sapphire_ros2::msg::MapPublicationStatus>("/sapphire/publication_status", rclcpp::QoS(1).transient_local());
  sapphire::OutputSink output;
  output.ready_changed = [this] {
    ready_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
    if (map_dirty_.exchange(true)) ++map_coalesced_;
    output_cv_.notify_one();
  };
  output.owner_reset = [this](bool begin) { reset_output(begin); };
  output.optional_drop = [this](bool incremental) {
    std::lock_guard<std::mutex> lock(output_mutex_); ++optional_dropped_; if (incremental) ++incremental_dropped_;
  };
  output.odom_state = [this](const sapphire::StateGroup &state) { enqueue_output(0, [this, state] { publish_odom(state); }); };
  output.local_scan = [this](std::shared_ptr<const sapphire::vvec<double, 3>> points,
                             const Eigen::Vector3d& origin, double stamp, std::uint64_t generation) {
    const auto bytes = points ? points->capacity() * sizeof(Eigen::Vector3d) : 0;
    enqueue_output(1, [this, points = std::move(points), origin, stamp, generation] {
      publish_scan(points);
      if (points && local_grid_) publish_local_grid(*points, origin, stamp, generation);
    }, bytes);
  };
  output.trajectory = [this](std::shared_ptr<const std::vector<sapphire::TrajectoryPoint>> trajectory) {
    const auto bytes = trajectory ? trajectory->capacity() * sizeof(sapphire::TrajectoryPoint) : 0;
    enqueue_output(2, [this, trajectory = std::move(trajectory)] { publish_trajectory(trajectory); }, bytes);
  };
  output.local_map = [this](std::shared_ptr<const sapphire::vvec<double, 3>> points) {
    const auto bytes = points ? points->capacity() * sizeof(Eigen::Vector3d) : 0;
    enqueue_output(3, [this, points = std::move(points)] { publish_local_map(points); }, bytes);
  };
  pipeline_ = std::make_unique<sapphire::SlamPipeline>(parameters, std::move(output));
  output_thread_ = std::thread(&SapphireNode::output_loop, this);

  imu_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  lidar_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  image_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions imu_options;
  imu_options.callback_group = imu_callback_group_;
  rclcpp::SubscriptionOptions lidar_options;
  lidar_options.callback_group = lidar_callback_group_;
  rclcpp::SubscriptionOptions image_options;
  image_options.callback_group = image_callback_group_;
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(imu_qos_depth)),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr message) { imu_callback(message); }, imu_options);
  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(lidar_qos_depth)),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { lidar_callback(message); }, lidar_options);
  if (dual) {
    rear_lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        rear_lidar_topic, rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(lidar_qos_depth)),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { rear_lidar_callback(message); }, lidar_options);
    lidar_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { drain_lidar(); }, lidar_callback_group_);
  }
  if (visual_enabled && compressed_image) {
    compressed_image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        image_topic, rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(image_qos_depth)),
        [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr message) { compressed_image_callback(message); }, image_options);
  } else if (visual_enabled) {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(image_qos_depth)),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr message) { image_callback(message); }, image_options);
  }
  if(stereo_enabled && compressed_image) {
    right_compressed_sub_=create_subscription<sensor_msgs::msg::CompressedImage>(right_topic,rclcpp::SensorDataQoS().keep_last(image_qos_depth),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg){if(shutdown_started_.load()) return; auto image=image_processor_.process(msg);if(image){image->camera_id=1;pipeline_->push_image(std::move(*image));}},image_options);
  } else if(stereo_enabled) {
    right_image_sub_=create_subscription<sensor_msgs::msg::Image>(right_topic,rclcpp::SensorDataQoS().keep_last(image_qos_depth),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg){if(shutdown_started_.load()) return; auto image=image_processor_.process(msg);if(image){image->camera_id=1;pipeline_->push_image(std::move(*image));}},image_options);
  }
  finish_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { finish_callback(); });

  RCLCPP_INFO(get_logger(), "lidar_mode=%s, lidar_type=%s, obs_mode=%s, Sapphire core config: %s",
              lidar_mode.c_str(), lidar_type.c_str(), obs_mode.c_str(), algorithm_config.c_str());
}

SapphireNode::~SapphireNode() {
  finish_timer_.reset();
  lidar_timer_.reset();
  compressed_image_sub_.reset();
  image_sub_.reset();
  right_image_sub_.reset();
  right_compressed_sub_.reset();
  lidar_sub_.reset();
  rear_lidar_sub_.reset();
  imu_sub_.reset();
  try { finish_checked(); } catch (...) { stop_output(); }
  pipeline_.reset();
}

void SapphireNode::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &message) {
  if (shutdown_started_.load()) return;
  sapphire::ImuMeas imu;
  imu.timestamp = static_cast<double>(message->header.stamp.sec) + static_cast<double>(message->header.stamp.nanosec) * 1e-9;
  imu.gyro << message->angular_velocity.x, message->angular_velocity.y, message->angular_velocity.z;
  imu.accel << message->linear_acceleration.x, message->linear_acceleration.y, message->linear_acceleration.z;
  if (!pipeline_->push_imu(std::move(imu))) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected invalid or out-of-order IMU sample");
  }
}

void SapphireNode::lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message) {
  if (shutdown_started_.load()) return;
  if (lidar_sync_) {
    lidar_sync_->push(false, message);
    drain_lidar();
    return;
  }
  double timestamp = 0.0;
  std::optional<double> end_timestamp;
  std::vector<sapphire::LidarPoint> points;
  if (!lidar_processor_->process(message, timestamp, points, end_timestamp)) {
    pipeline_->notifyInputLoss();
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported or malformed PointCloud2 input");
    return;
  }
  if (!pipeline_->push_lidar(timestamp, std::move(points), end_timestamp)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected invalid or out-of-order LiDAR scan");
  }
}

void SapphireNode::rear_lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message) {
  if (shutdown_started_.load()) return;
  lidar_sync_->push(true, message);
  drain_lidar();
}

void SapphireNode::drain_lidar() {
  if (shutdown_started_.load()) return;
  while (auto scan = lidar_sync_->pop()) {
    if (lidar_sync_->statistics().dropped() != last_lidar_drop_count_) pipeline_->notifyInputLoss();
    if (!pipeline_->push_lidar(scan->begin, std::move(scan->points), scan->end)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected fused LiDAR scan");
    }
  }
  const auto &s = lidar_sync_->statistics();
  if (s.dropped() != last_lidar_drop_count_) pipeline_->notifyInputLoss();
  const auto now = std::chrono::steady_clock::now();
  if (s.dropped() != last_lidar_drop_count_ && now - last_lidar_report_ >= std::chrono::seconds(5)) {
    last_lidar_drop_count_ = s.dropped();
    last_lidar_report_ = now;
    RCLCPP_WARN(get_logger(),
        "dual totals: emitted=%zu invalid=%zu out_of_order=%zu overflow=%zu timeout=%zu coverage=%zu empty=%zu",
        s.emitted, s.invalid, s.out_of_order, s.overflow, s.timeout, s.coverage, s.empty);
  }
}

void SapphireNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &message) {
  if (shutdown_started_.load()) return;
  std::optional<sapphire::ImageMeas> image = image_processor_.process(message);
  if (!image) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported or malformed raw image input");
    return;
  }
  if (!pipeline_->push_image(std::move(*image))) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected grayscale image");
  }
}

void SapphireNode::compressed_image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &message) {
  if (shutdown_started_.load()) return;
  std::optional<sapphire::ImageMeas> image = image_processor_.process(message);
  if (!image) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Unable to decode compressed image input");
    return;
  }
  if (!pipeline_->push_image(std::move(*image))) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected grayscale image");
  }
}

void SapphireNode::finish_callback() {
  bool finish = false;
  get_parameter("finish", finish);
  finish = finish || (pipeline_ && pipeline_->failed());
  if (!finish || shutdown_started_.exchange(true)) {
    return;
  }
  RCLCPP_INFO(get_logger(), "finish requested; waiting for active callbacks before flushing");
}

void SapphireNode::enqueue_output(std::size_t slot, std::function<void()> task, std::size_t payload_bytes) {
  std::lock_guard<std::mutex> lock(output_mutex_);
  if (output_stopping_ || final_requested_ || output_resetting_) {
    ++optional_dropped_; if (slot == 3) ++incremental_dropped_; return;
  }
  if (output_slots_[slot]) { ++coalesced_; ++optional_dropped_; if (slot == 3) ++incremental_dropped_; }
  output_slots_[slot] = std::move(task); output_slot_bytes_[slot] = payload_bytes;
  std::size_t retained = active_optional_bytes_; for (auto bytes : output_slot_bytes_) retained += bytes;
  optional_copy_high_water_ = std::max(optional_copy_high_water_, retained);
  std::size_t count = 0; for (const auto &value : output_slots_) count += bool(value);
  pending_high_water_ = std::max(pending_high_water_, count);
  output_cv_.notify_one();
}

void SapphireNode::reset_output(bool begin) {
  std::unique_lock<std::mutex> lock(output_mutex_);
  if (begin) {
    output_resetting_ = true;
    output_cv_.notify_all();
    for (std::size_t i = 0; i < output_slots_.size(); ++i) if (output_slots_[i]) {
      ++optional_dropped_; if (i == 3) ++incremental_dropped_; output_slots_[i] = {}; output_slot_bytes_[i] = 0;
    }
    output_cv_.wait(lock, [this] { return !output_active_; });
    if (local_grid_) local_grid_->reset();
    correction_group_ = {}; navigation_group_ = {}; map_uuid_.clear(); generation_ = 0;
    retry_requested_.store(false); ready_ns_.store(0); next_grid_ = {}; map_dirty_.store(false);
  } else {
    output_resetting_ = false;
    // Failed replacement terminates this executor: even late optional producer
    // callbacks must not restart old/new-domain output after acknowledgment.
    if (pipeline_ && pipeline_->failed()) { output_stopping_ = true; map_dirty_.store(false); }
    else map_dirty_.store(true);
    output_cv_.notify_all();
  }
}

void SapphireNode::retry_publication() {
  std::lock_guard<std::mutex> lock(output_mutex_);
  retry_requested_.store(true);
  map_dirty_.store(true); output_cv_.notify_one();
}

void SapphireNode::output_loop() {
  auto next_poll = std::chrono::steady_clock::now();
  for (;;) {
    std::function<void()> optional;
    std::size_t slot = 0;
    bool map = false, final = false, mapping_ok = true;
    {
      std::unique_lock<std::mutex> lock(output_mutex_);
      output_cv_.wait_until(lock, next_poll, [this] {
        return output_stopping_ || (!output_resetting_ && (final_requested_ || map_dirty_.load() ||
          std::any_of(output_slots_.begin(), output_slots_.end(), [](const auto &v) { return bool(v); })));
      });
      if (output_stopping_) break;
      if (output_resetting_) { next_poll = std::chrono::steady_clock::now() + output_period_; continue; }
      final = final_requested_; mapping_ok = mapping_ok_;
      map = final || map_dirty_.exchange(false) || std::chrono::steady_clock::now() >= next_poll;
      if (map) next_poll = std::chrono::steady_clock::now() + output_period_;
      else for (; slot < output_slots_.size(); ++slot) if (output_slots_[slot]) { optional = std::move(output_slots_[slot]); output_slots_[slot] = {}; active_optional_bytes_ = output_slot_bytes_[slot]; output_slot_bytes_[slot] = 0; break; }
      output_active_ = true;
    }
    try {
      if (map) { if (mapping_ok) attempt_map(final); if (!final) publish_status(); }
      else if (optional) optional();
    } catch (...) {
      if (optional) { std::lock_guard<std::mutex> lock(output_mutex_); ++optional_dropped_; if (slot == 3) ++incremental_dropped_; }
      else { if (!correction_group_.first_error) correction_group_.first_error = std::current_exception(); }
    }
    {
      std::lock_guard<std::mutex> lock(output_mutex_); output_active_ = false; active_optional_bytes_ = 0;
      if (final) { output_stopping_ = true; final_requested_ = false; map_dirty_.store(false); }
    }
    output_cv_.notify_all();
    if (final) break;
  }
}

void SapphireNode::stop_output() {
  { std::lock_guard<std::mutex> lock(output_mutex_); output_stopping_ = true; }
  output_cv_.notify_all();
  if (output_thread_.joinable()) output_thread_.join();
}

bool SapphireNode::finish_checked() {
  if (finished_) return finish_success_;
  request_stop();
  auto start = std::chrono::steady_clock::now();
  std::optional<std::uint64_t> target_revision, target_source;
  std::exception_ptr mapping_error;
  auto producers_joined = start;
  try {
    if (pipeline_) {
      pipeline_->shutdown(); producers_joined = std::chrono::steady_clock::now();
      auto progress = pipeline_->drain(); target_source = progress.last_completed;
      if (progress.completed_revision) target_revision = progress.ready_revision;
    }
  } catch (...) { mapping_error = std::current_exception(); }
  const auto drained = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    mapping_error_ = mapping_error; final_revision_ = target_revision; final_source_ = target_source;
    mapping_ok_ = !mapping_error_;
    for (std::size_t i = 0; i < output_slots_.size(); ++i) if (output_slots_[i]) {
      ++optional_dropped_; if (i == 3) ++incremental_dropped_; output_slots_[i] = {}; output_slot_bytes_[i] = 0;
    }
    final_requested_ = true;
  }
  output_cv_.notify_all();
  if (output_thread_.joinable()) output_thread_.join();
  const auto published = std::chrono::steady_clock::now();
  close_attempted_ = true;
  try { if (pipeline_) pipeline_->close(); close_success_ = true; } catch (...) { close_error_ = std::current_exception(); }
  try { final_status_ = make_status(); } catch (...) {}
  finished_ = true;
  finish_success_ = !mapping_error_ && close_success_ && get_node_base_interface()->get_context()->is_valid() && (!final_revision_ ||
      (correction_group_.published == final_revision_ && (!navigation_enabled_ || navigation_group_.published == final_revision_)));
  const auto ms = [](auto duration) { return std::chrono::duration<double, std::milli>(duration).count(); };
  RCLCPP_INFO(get_logger(), "B3 checked finish success=%d F=%lu Pc=%lu Pg=%lu frontend_join_ms=%.3f backend_drain_ms=%.3f final_output_ms=%.3f close_ms=%.3f total_ms=%.3f pending_high_water=%zu coalesced=%lu optional_drops=%lu increment_drops=%lu retries=%lu capture_ms=%.3f grid_ms=%.3f publication_ms=%.3f",
      finish_success_, final_revision_.value_or(0), correction_group_.published.value_or(0), navigation_group_.published.value_or(0),
      ms(producers_joined-start), ms(drained-producers_joined), ms(published-drained), ms(std::chrono::steady_clock::now()-published), ms(std::chrono::steady_clock::now()-start),
      pending_high_water_, coalesced_, optional_dropped_, incremental_dropped_, retries_, capture_ms_, grid_ms_, publication_ms_);
  RCLCPP_INFO(get_logger(), "B3 output timing samples=%zu ready_selection_ms_sum=%.3f selection_success_ms_sum=%.3f ready_success_ms_sum=%.3f conversion_serialization_ms=%.3f checked_publish_ms=%.3f payload_copy_high_water=%zu optional_copy_high_water=%zu required_coalesced=%lu",
      success_samples_, ready_selection_ms_, selection_success_ms_, ready_success_ms_, conversion_ms_, checked_publish_ms_, payload_copy_high_water_, optional_copy_high_water_, map_coalesced_.load());
  return finish_success_;
}

void SapphireNode::publish_odom(const sapphire::StateGroup &state) {
  const auto stamp = stamp_from_seconds(state.t);
  const Eigen::Quaterniond q(state.R);
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = state.p.x();
  odom.pose.pose.position.y = state.p.y();
  odom.pose.pose.position.z = state.p.z();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  const Eigen::Vector3d velocity_body = state.R.transpose() * state.v;
  odom.twist.twist.linear.x = velocity_body.x();
  odom.twist.twist.linear.y = velocity_body.y();
  odom.twist.twist.linear.z = velocity_body.z();
  odom_pub_->publish(odom);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = state.R;
  transform.translation() = state.p;
  tf_broadcaster_->sendTransform(make_transform(stamp, odom_frame_.c_str(), base_frame_.c_str(), transform));
}

void SapphireNode::observe_optional_copy(std::size_t conversion_bytes) {
  std::lock_guard<std::mutex> lock(output_mutex_);
  std::size_t bytes = active_optional_bytes_ + conversion_bytes;
  for (auto retained : output_slot_bytes_) bytes += retained;
  optional_copy_high_water_ = std::max(optional_copy_high_water_, bytes);
}

void SapphireNode::publish_scan(std::shared_ptr<const sapphire::vvec<double, 3>> points) {
  if (!points) {
    return;
  }
  std::size_t copies = 0; auto message = point_cloud_message(*points, now(), odom_frame_.c_str(), &copies);
  observe_optional_copy(copies); scan_pub_->publish(message);
}

void SapphireNode::publish_local_grid(const sapphire::vvec<double, 3>& points, const Eigen::Vector3d& origin,
                                     double stamp, std::uint64_t generation) {
  if (!local_grid_->update(points, origin, stamp, generation)) return;
  nav_msgs::msg::OccupancyGrid message;
  message.header.frame_id = odom_frame_;
  message.header.stamp = rclcpp::Time(static_cast<std::int64_t>(stamp * 1e9));
  message.info.map_load_time = message.header.stamp;
  message.info.resolution = local_grid_->resolution();
  message.info.width = message.info.height = local_grid_->width();
  message.info.origin.position.x = local_grid_->originX();
  message.info.origin.position.y = local_grid_->originY();
  message.info.origin.orientation.w = 1;
  local_grid_->raster(message.data);
  observe_optional_copy(message.data.capacity());
  local_grid_pub_->publish(message);
}

void SapphireNode::publish_trajectory(std::shared_ptr<const std::vector<sapphire::TrajectoryPoint>> trajectory) {
  if (!trajectory) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZINormal> cloud;
  cloud.reserve(trajectory->size());
  for (const sapphire::TrajectoryPoint &point : *trajectory) {
    pcl::PointXYZINormal output;
    output.x = point.x;
    output.y = point.y;
    output.z = point.z;
    output.intensity = point.session;
    output.curvature = point.distance;
    cloud.push_back(output);
  }
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.stamp = now();
  message.header.frame_id = odom_frame_;
  observe_optional_copy(cloud.points.capacity() * sizeof(pcl::PointXYZINormal) + message.data.capacity());
  path_pub_->publish(message);
}

void SapphireNode::publish_local_map(std::shared_ptr<const sapphire::vvec<double, 3>> points) {
  if (!points) {
    return;
  }
  std::size_t copies = 0; auto message = point_cloud_message(*points, now(), odom_frame_.c_str(), &copies);
  observe_optional_copy(copies); cmap_pub_->publish(message);
}

}  // namespace sapphire_ros

namespace sapphire_ros {
namespace {
std::string output_error() noexcept {
  try { throw; }
  catch (const std::exception &e) { try { return std::string(e.what()).substr(0, 1024); } catch (...) {} }
  catch (...) {}
  return "output failure";
}
}

void SapphireNode::publish_correction(const std::string &uuid, std::uint64_t revision, std::uint64_t generation,
                                    std::uint64_t source, double timestamp, const Eigen::Isometry3d &correction,
                                    const Eigen::Isometry3d &pose) {
  if (uuid.size() > 36 || incarnation_.size() > 36) throw std::length_error("Invalid B3 identity size");
  auto context = get_node_base_interface()->get_context();
  sapphire_ros2::msg::RevisionedMapCorrection c;
  c.map_uuid = uuid; c.publisher_incarnation = incarnation_; c.generation = generation;
  c.source_graph_revision = revision; c.source_sequence = source;
  c.correction = make_transform(now(), map_frame_.c_str(), odom_frame_.c_str(), correction);
  sapphire_ros2::msg::RevisionedMapPose p;
  p.map_uuid = uuid; p.publisher_incarnation = incarnation_; p.generation = generation;
  p.source_graph_revision = revision; p.source_sequence = source;
  p.pose.header.stamp = stamp_from_seconds(timestamp); p.pose.header.frame_id = map_frame_;
  p.pose.child_frame_id = base_frame_;
  p.pose.pose.pose.position.x = pose.translation().x(); p.pose.pose.pose.position.y = pose.translation().y(); p.pose.pose.pose.position.z = pose.translation().z();
  Eigen::Quaterniond q(pose.rotation());
  p.pose.pose.pose.orientation.x = q.x(); p.pose.pose.pose.orientation.y = q.y(); p.pose.pose.pose.orientation.z = q.z(); p.pose.pose.pose.orientation.w = q.w();
  checked_publish<sapphire_ros2::msg::RevisionedMapCorrection>(correction_pub_, context, c, 65536, &conversion_ms_, &checked_publish_ms_);
  if (!context->is_valid()) throw std::runtime_error("Context lost before TF");
  tf_broadcaster_->sendTransform(c.correction);
  if (!context->is_valid()) throw std::runtime_error("Context lost during TF");
  checked_publish<sapphire_ros2::msg::RevisionedMapPose>(revisioned_pose_pub_, context, p, 65536, &conversion_ms_, &checked_publish_ms_);
  checked_publish<nav_msgs::msg::Odometry>(map_odom_pub_, context, p.pose, 65536, &conversion_ms_, &checked_publish_ms_);
}

void SapphireNode::attempt_map(bool final) {
  if (!pipeline_ || pipeline_->failed()) return;
  if (!get_node_base_interface()->get_context()->is_valid()) {
    if (!correction_group_.first_error) correction_group_.first_error = std::make_exception_ptr(std::runtime_error("ROS context unavailable"));
    correction_group_.error = "ROS context unavailable";
    if (navigation_enabled_) {
      if (!navigation_group_.first_error) navigation_group_.first_error = correction_group_.first_error;
      navigation_group_.error = "ROS context unavailable";
    }
    return; // Context loss never schedules a transport retry.
  }
  const auto selected_at = std::chrono::steady_clock::now();
  const auto ready_at = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ready_ns_.load()));
  auto progress = pipeline_->continuationProgress();
  if (progress.first_failure || !progress.completed_revision) return;
  const auto may_attempt = [&](const Group &group) {
    if (group.published && *group.published >= progress.ready_revision) return false;
    if (final || retry_requested_.load() || group.target != progress.ready_revision) return true;
    return group.online_attempts < 2 && (!group.online_attempts ||
        std::chrono::steady_clock::now() - group.attempted >= output_period_);
  };
  if (!may_attempt(correction_group_) && (!navigation_enabled_ || !may_attempt(navigation_group_))) return;
  std::string uuid;
  std::uint64_t revision = 0, generation = 0, source = 0;
  double timestamp = 0;
  Eigen::Isometry3d correction, pose;
  std::shared_ptr<const sapphire::NavigationGrid> grid;
  const auto ms = [](auto start) { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-start).count(); };
  auto capture = [&](bool navigation) {
    auto start = std::chrono::steady_clock::now();
    auto result = pipeline_->captureReady(navigation, !final, uuid, revision, generation, source, timestamp, correction, pose, grid);
    if (navigation) grid_ms_ += ms(start); else capture_ms_ += ms(start);
    return result;
  };
  try { if (!capture(false)) return; }
  catch (...) {
    const auto cause = std::current_exception(); const auto detail = output_error();
    const bool correction_due = may_attempt(correction_group_);
    const bool navigation_due = navigation_enabled_ && may_attempt(navigation_group_);
    const bool explicit_request = retry_requested_.exchange(false);
    const auto failed_capture = [&](Group &group) {
      if (group.target != progress.ready_revision) { group.target = progress.ready_revision; group.online_attempts = 0; }
      if (!final && !explicit_request) ++group.online_attempts;
      group.attempted = std::chrono::steady_clock::now();
      if (!group.first_error) group.first_error = cause; group.error = detail;
    };
    if (correction_due) failed_capture(correction_group_);
    if (navigation_due) failed_capture(navigation_group_);
    return;
  }
  if (final && (!final_revision_ || revision != *final_revision_ || source != final_source_))
    throw std::logic_error("Final capture disagrees with frozen drain target");
  if (uuid != map_uuid_ || generation != generation_) {
    if (!map_uuid_.empty()) { correction_group_ = {}; navigation_group_ = {}; }
    map_uuid_ = uuid; generation_ = generation;
  }
  const bool explicit_request = retry_requested_.exchange(false);
  bool correction_explicit = explicit_request, navigation_explicit = explicit_request;
  const auto eligible = [&](Group &group) {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if (group.published && *group.published >= revision) return false;
    if (group.target != revision) { group.target = revision; group.online_attempts = 0; group.error.clear(); }
    bool &request = &group == &correction_group_ ? correction_explicit : navigation_explicit;
    bool explicit_retry = request; request = false;
    if (final) { ++retries_; return true; }
    if (explicit_retry) { ++retries_; return true; }
    if (group.online_attempts >= 2) return false;
    if (group.online_attempts && std::chrono::steady_clock::now() - group.attempted < output_period_) return false;
    if (group.online_attempts++) ++retries_;
    group.attempted = std::chrono::steady_clock::now(); return true;
  };
  const auto record_success = [&] {
    const auto success = std::chrono::steady_clock::now();
    if (ready_at.time_since_epoch().count()) {
      ready_selection_ms_ += std::chrono::duration<double,std::milli>(selected_at-ready_at).count();
      selection_success_ms_ += std::chrono::duration<double,std::milli>(success-selected_at).count();
      ready_success_ms_ += std::chrono::duration<double,std::milli>(success-ready_at).count(); ++success_samples_;
    }
  };
  const auto correction_attempt = [&] {
    if (!eligible(correction_group_)) return;
    auto start = std::chrono::steady_clock::now();
    try {
      publish_correction(uuid, revision, generation, source, timestamp, correction, pose);
      correction_group_.published = revision; correction_group_.error.clear(); record_success();
    } catch (...) {
      if (!correction_group_.first_error) correction_group_.first_error = std::current_exception();
      correction_group_.error = output_error();
    }
    publication_ms_ += ms(start);
  };
  correction_attempt();
  if (!navigation_enabled_ || (!final && !explicit_request && std::chrono::steady_clock::now() < next_grid_)) return;
  const auto prior_navigation_attempts = navigation_group_.online_attempts;
  const auto prior_navigation_target = navigation_group_.target;
  const auto prior_navigation_time = navigation_group_.attempted;
  const auto prior_grid_time = next_grid_;
  const auto prior_retries = retries_;
  if (!eligible(navigation_group_)) return;
  next_grid_ = std::chrono::steady_clock::now() + output_period_;
  auto start = std::chrono::steady_clock::now();
  try {
    // A fresh grid capture includes its correction at the SAME revision. No stale relabeling.
    const auto requested = revision;
    if (!capture(true)) {
      navigation_group_.online_attempts = prior_navigation_attempts;
      navigation_group_.target = prior_navigation_target; navigation_group_.attempted = prior_navigation_time;
      next_grid_ = prior_grid_time; retries_ = prior_retries;
      return; // Busy acquisition is not a publication attempt, including for a newer target.
    }
    if (final && revision != final_revision_) throw std::logic_error("Final grid revision mismatch");
    if (revision != requested) {
      navigation_group_.target = revision; navigation_group_.online_attempts = final ? 0 : 1;
      correction_attempt();
    }
    auto conversion_start = std::chrono::steady_clock::now();
    sapphire_ros2::msg::RevisionedNavigationGrid message;
    message.map_uuid = uuid; message.publisher_incarnation = incarnation_; message.source_graph_revision = revision;
    message.grid.header.stamp = now(); message.grid.header.frame_id = map_frame_;
    message.grid.info.resolution = grid->resolution; message.grid.info.width = grid->width; message.grid.info.height = grid->height;
    message.grid.info.origin.position.x = grid->origin_x; message.grid.info.origin.position.y = grid->origin_y;
    message.grid.info.origin.orientation.w = 1; message.grid.data = grid->data;
    conversion_ms_ += ms(conversion_start);
    payload_copy_high_water_ = std::max(payload_copy_high_water_, grid->data.capacity() + message.grid.data.capacity());
    auto context = get_node_base_interface()->get_context();
    checked_publish<sapphire_ros2::msg::RevisionedNavigationGrid>(revisioned_grid_pub_, context, message, 16 * 1024 * 1024, &conversion_ms_, &checked_publish_ms_, &payload_copy_high_water_, grid->data.capacity() + message.grid.data.capacity());
    checked_publish<nav_msgs::msg::OccupancyGrid>(map_pub_, context, message.grid, 16 * 1024 * 1024, &conversion_ms_, &checked_publish_ms_, &payload_copy_high_water_, grid->data.capacity() + message.grid.data.capacity());
    navigation_group_.published = revision; navigation_group_.error.clear(); record_success();
  } catch (...) {
    if (!navigation_group_.first_error) navigation_group_.first_error = std::current_exception();
    navigation_group_.error = output_error();
  }
  publication_ms_ += ms(start);
}

sapphire_ros2::msg::MapPublicationStatus SapphireNode::make_status() {
    sapphire_ros2::msg::MapPublicationStatus s;
    s.map_uuid = map_uuid_; s.publisher_incarnation = incarnation_; s.generation = generation_;
    s.correction_available = bool(correction_group_.published); s.correction_revision = correction_group_.published.value_or(0);
    s.navigation_available = navigation_enabled_ && bool(navigation_group_.published); s.navigation_revision = navigation_group_.published.value_or(0);
    s.correction_error = correction_group_.error; s.navigation_error = navigation_group_.error;
    { std::lock_guard<std::mutex> lock(output_mutex_);
      s.pending = map_dirty_.load() || std::any_of(output_slots_.begin(), output_slots_.end(), [](const auto &v) { return bool(v); }); s.in_flight = output_active_; s.coalesced = coalesced_ + map_coalesced_.load();
      s.optional_dropped = optional_dropped_; s.incremental_dropped = incremental_dropped_; s.retries = retries_;
    }
    const auto p = pipeline_->continuationProgress();
    if ((!s.map_uuid.empty() && s.map_uuid != p.map_uuid) ||
        (s.correction_available && p.generation && generation_ != *p.generation))
      throw std::logic_error("Output/status domain changed");
    s.map_uuid = p.map_uuid; s.generation_available = bool(p.generation);
    s.generation = p.generation.value_or(generation_);
    s.root_available = bool(p.root_revision); s.root_source = p.root_source.value_or(0); s.root_revision = p.root_revision.value_or(0);
    s.accepted_source_available = bool(p.last_accepted); s.completed_revision_available = bool(p.completed_revision);
    s.waiting = p.waiting; s.head_bytes = p.head_bytes; s.waiting_bytes = p.waiting_bytes;
    s.unaccepted_tail_dropped = pipeline_->unacceptedTailDrops();
    s.association_pending = p.association_pending; s.association_bytes = p.association_bytes;
    s.association_high_water = p.association_high_water; s.association_attempts = p.association_attempts;
    s.automatically_attached = p.automatically_attached;
    s.committed_available = bool(p.committed_revision); s.committed_revision = p.committed_revision.value_or(0);
    s.committed_outcome_known = p.committed_outcome_known; s.ready_available = p.ready_available; s.ready_revision = p.ready_revision;
    s.backend_healthy = !pipeline_->failed() && !p.first_failure; s.current = s.backend_healthy && p.completed_revision.has_value() && !close_attempted_;
    s.source_available = bool(p.last_completed); s.last_accepted = p.last_accepted.value_or(0); s.last_completed = p.last_completed.value_or(0);
    s.completed_revision = p.completed_revision.value_or(0); s.head_available = bool(p.head_sequence); s.head_sequence = p.head_sequence.value_or(0);
    s.head_state = static_cast<std::uint8_t>(p.head); s.accepted = p.accepted; s.completed = p.completed;
    s.canceled = bool(p.canceled_first); s.canceled_first = p.canceled_first.value_or(0); s.canceled_last = p.canceled_last.value_or(0);
    s.cancellation_reason = p.cancellation_reason.substr(0, 1024); s.commit_outcome_available = bool(p.last_commit_outcome);
    if (p.last_commit_outcome) s.commit_outcome = static_cast<std::uint8_t>(*p.last_commit_outcome);
    s.enabled_mask = navigation_enabled_ ? 3 : 1;
    const auto producer_error = pipeline_->producerFailure();
    std::lock_guard<std::mutex> final_fields(output_mutex_);
    s.drain_target_available = bool(final_revision_); s.drain_revision = final_revision_.value_or(0); s.drain_source = final_source_.value_or(0);
    s.close_attempted = close_attempted_; s.close_success = close_success_;
    const auto backend_error = mapping_error_ ? mapping_error_ : (p.first_failure ? p.first_failure : producer_error);
    if (backend_error) { try { std::rethrow_exception(backend_error); } catch (...) { s.backend_error = output_error(); } }
    if (close_error_) { try { std::rethrow_exception(close_error_); } catch (...) { s.close_error = output_error(); } }
    return s;
}
void SapphireNode::publish_status() {
  try {
    auto s = make_status();
    checked_publish<sapphire_ros2::msg::MapPublicationStatus>(status_pub_, get_node_base_interface()->get_context(), s);
  } catch (...) { std::lock_guard<std::mutex> lock(output_mutex_); ++optional_dropped_; }
}
}
