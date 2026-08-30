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
                                                  const char *frame) {
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
  return message;
}

}  // namespace

SapphireNode::SapphireNode() : Node("cmn_sapphire") {
  const bool finish = parameter(*this, "finish", false);
  if (finish) {
    RCLCPP_WARN(get_logger(), "finish is already true at startup");
  }

  const std::string package_share = ament_index_cpp::get_package_share_directory("sapphire_ros2");
  const std::string algorithm_config = parameter(*this, "algorithm_config", package_share + "/config/mid360.toml");
  const std::string lidar_topic = parameter(*this, "topics.lidar", std::string("/front_lidar"));
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
  lidar_processor_ = std::make_unique<LidarProcessor>(parameters.sensor);

  scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(scan_topic, rclcpp::QoS(10));
  cmap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(local_map_topic, rclcpp::QoS(2));
  path_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(trajectory_topic, rclcpp::QoS(2));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(100));
  map_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(map_pose_topic, rclcpp::QoS(10));
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(navigation_topic, rclcpp::QoS(1).transient_local());
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  sapphire::OutputSink output;
  output.odom_state = [this](const sapphire::StateGroup &state) { enqueue_output([this, state] { publish_odom(state); }); };
  output.local_scan = [this](std::shared_ptr<const sapphire::vvec<double, 3>> points) {
    enqueue_output([this, points = std::move(points)]() mutable { publish_scan(std::move(points)); });
  };
  output.trajectory = [this](std::shared_ptr<const std::vector<sapphire::TrajectoryPoint>> trajectory) {
    enqueue_output([this, trajectory = std::move(trajectory)]() mutable { publish_trajectory(std::move(trajectory)); });
  };
  output.local_map = [this](std::shared_ptr<const sapphire::vvec<double, 3>> points) {
    enqueue_output([this, points = std::move(points)]() mutable { publish_local_map(std::move(points)); });
  };
  output.map_odom = [this](const Eigen::Isometry3d &transform) { enqueue_output([this, transform] { publish_map_odom(transform); }); };
  output.map_pose = [this](const Eigen::Isometry3d &transform, double timestamp) {
    enqueue_output([this, transform, timestamp] { publish_map_pose(transform, timestamp); });
  };
  output.navigation_grid = [this](std::shared_ptr<const sapphire::NavigationGrid> grid) {
    enqueue_output([this, grid = std::move(grid)]() mutable { publish_navigation_grid(std::move(grid)); });
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
  if (compressed_image) {
    compressed_image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        image_topic, rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(image_qos_depth)),
        [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr message) { compressed_image_callback(message); }, image_options);
  } else {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(image_qos_depth)),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr message) { image_callback(message); }, image_options);
  }
  finish_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { finish_callback(); });

  RCLCPP_INFO(get_logger(), "using Sapphire core config: %s", algorithm_config.c_str());
}

SapphireNode::~SapphireNode() {
  finish_timer_.reset();
  compressed_image_sub_.reset();
  image_sub_.reset();
  lidar_sub_.reset();
  imu_sub_.reset();
  if (pipeline_) {
    pipeline_->shutdown();
  }
  pipeline_.reset();
  stop_output();
}

void SapphireNode::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &message) {
  sapphire::ImuMeas imu;
  imu.timestamp = static_cast<double>(message->header.stamp.sec) + static_cast<double>(message->header.stamp.nanosec) * 1e-9;
  imu.gyro << message->angular_velocity.x, message->angular_velocity.y, message->angular_velocity.z;
  imu.accel << message->linear_acceleration.x, message->linear_acceleration.y, message->linear_acceleration.z;
  if (!pipeline_->push_imu(std::move(imu))) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected invalid or out-of-order IMU sample");
  }
}

void SapphireNode::lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message) {
  double timestamp = 0.0;
  std::vector<sapphire::LidarPoint> points;
  if (!lidar_processor_->process(message, timestamp, points)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported or malformed PointCloud2 input");
    return;
  }
  if (!pipeline_->push_lidar(timestamp, std::move(points))) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected invalid or out-of-order LiDAR scan");
  }
}

void SapphireNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &message) {
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
  if (!finish || shutdown_started_.exchange(true)) {
    return;
  }
  RCLCPP_INFO(get_logger(), "finish requested; waiting for active callbacks before flushing");
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void SapphireNode::enqueue_output(std::function<void()> task) {
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if (output_stopping_) {
      return;
    }
    output_queue_.push_back(std::move(task));
  }
  output_cv_.notify_one();
}

void SapphireNode::output_loop() {
  while (true) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(output_mutex_);
      output_cv_.wait(lock, [this] { return output_stopping_ || !output_queue_.empty(); });
      if (output_queue_.empty()) {
        if (output_stopping_) {
          return;
        }
        continue;
      }
      task = std::move(output_queue_.front());
      output_queue_.pop_front();
    }

    try {
      task();
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Output task failed: %s", error.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Output task failed with an unknown exception");
    }
  }
}

void SapphireNode::stop_output() {
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    output_stopping_ = true;
  }
  output_cv_.notify_all();
  if (output_thread_.joinable()) {
    output_thread_.join();
  }
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

void SapphireNode::publish_scan(std::shared_ptr<const sapphire::vvec<double, 3>> points) {
  if (!points) {
    return;
  }
  scan_pub_->publish(point_cloud_message(*points, now(), odom_frame_.c_str()));
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
  path_pub_->publish(message);
}

void SapphireNode::publish_local_map(std::shared_ptr<const sapphire::vvec<double, 3>> points) {
  if (!points) {
    return;
  }
  cmap_pub_->publish(point_cloud_message(*points, now(), odom_frame_.c_str()));
}

void SapphireNode::publish_map_odom(const Eigen::Isometry3d &transform) {
  tf_broadcaster_->sendTransform(make_transform(now(), map_frame_.c_str(), odom_frame_.c_str(), transform));
}

void SapphireNode::publish_map_pose(const Eigen::Isometry3d &transform, double timestamp) {
  const Eigen::Quaterniond q(transform.rotation());
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp_from_seconds(timestamp);
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = transform.translation().x();
  odom.pose.pose.position.y = transform.translation().y();
  odom.pose.pose.position.z = transform.translation().z();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  map_odom_pub_->publish(odom);
}

void SapphireNode::publish_navigation_grid(std::shared_ptr<const sapphire::NavigationGrid> grid) {
  if (!grid || grid->revision == last_map_revision_) {
    return;
  }
  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = now();
  message.header.frame_id = map_frame_;
  message.info.resolution = grid->resolution;
  message.info.width = grid->width;
  message.info.height = grid->height;
  message.info.origin.position.x = grid->origin_x;
  message.info.origin.position.y = grid->origin_y;
  message.info.origin.orientation.w = 1.0;
  message.data = grid->data;
  map_pub_->publish(message);
  last_map_revision_ = grid->revision;
}

}  // namespace sapphire_ros
