#include <set>
#include <rcl/publisher.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cstring>
#include <limits>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <optional>
#include <rclcpp/serialization.hpp>
#include <sapphire_ros2/sapphire_node.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>


namespace {
// Local adapter completion only. The caller owns context and publisher through
// the interval; no subscriber/delivery/enqueue promise is implied.
template<class Message>
void checked_publish(const typename rclcpp::Publisher<Message>::SharedPtr &publisher,
                     const rclcpp::Context::SharedPtr &context, const Message &message,
                     std::size_t cap = 65536, double *serialization_ms = nullptr,
                     double *publish_ms = nullptr, std::size_t *copy_high_water = nullptr,
                     std::size_t retained_bytes = 0) {
  if (!publisher || !context || !context->is_valid()) throw std::runtime_error("Required ROS context unavailable");
  auto start = std::chrono::steady_clock::now();
  rclcpp::Serialization<Message> serializer;
  rclcpp::SerializedMessage encoded;
  serializer.serialize_message(&message, &encoded);
  if (encoded.size() > cap) throw std::length_error("Required encoded payload exceeds B3 envelope");
  if (serialization_ms) *serialization_ms += std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  if (copy_high_water) *copy_high_water = std::max(*copy_high_water, retained_bytes + encoded.capacity());
  start = std::chrono::steady_clock::now();
  const auto result = rcl_publish(publisher->get_publisher_handle().get(), &message, nullptr);
  if (publish_ms) *publish_ms += std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  if (result != RCL_RET_OK || !context->is_valid()) {
    rcl_reset_error();
    throw std::runtime_error("Required local ROS submission failed or context lost");
  }
}
}  // namespace

namespace sapphire_ros {
namespace {
std::string output_error() noexcept;
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
  if (points.size() > std::numeric_limits<uint32_t>::max() / 16)
    throw std::length_error("Display point cloud exceeds PointCloud2 row size");
  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp = stamp;
  message.header.frame_id = frame;
  message.height = 1;
  message.width = static_cast<uint32_t>(points.size());
  message.point_step = 16;
  message.row_step = message.width * message.point_step;
  message.is_dense = true;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  message.is_bigendian = true;
#endif
  const std::array<const char *, 4> names{"x", "y", "z", "intensity"};
  for (size_t i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i]; field.offset = i * sizeof(float);
    field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
    message.fields.push_back(std::move(field));
  }
  message.data.resize(message.row_step);
  for (size_t i = 0; i < points.size(); ++i) {
    const std::array<float, 4> point{float(points[i].x()), float(points[i].y()), float(points[i].z()), 0.F};
    std::memcpy(message.data.data() + i * message.point_step, point.data(), message.point_step);
    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) message.is_dense = false;
  }
  if (conversion_bytes) *conversion_bytes = message.data.capacity();
  return message;
}

}  // namespace

SapphireNode::SapphireNode(const rclcpp::NodeOptions &options) : Node("cmn_sapphire", options) {
  const bool finish = parameter(*this, "finish", false);
  if (finish) {
    RCLCPP_WARN(get_logger(), "finish is already true at startup");
  }

  for (const auto &[name, value] : get_node_parameters_interface()->get_parameter_overrides()) {
    if (name == "lidar_mode" || name == "lidar_type" || name == "topics.lidar" || name == "topics.rear_lidar" ||
        name == "topics.imu" || name.rfind("lidar.", 0) == 0)
      throw std::invalid_argument("Move sensor ROS parameter '" + name + "' into sensors.toml");
  }
  const std::string package_share = ament_index_cpp::get_package_share_directory("sapphire_ros2");
  const std::string algorithm_config = parameter(*this, "algorithm_config", package_share + "/config/params.toml");
  const std::string sensor_config = parameter(*this, "sensor_config", package_share + "/config/sensors.toml");
  const auto sensors = sapphire::load_sensor_parameters(sensor_config);
  const bool dual = sensors.lidars.size() == 2;
  const auto &reference = sensors.lidars.front();
  const std::string &lidar_topic = reference.topic;
  const std::string rear_lidar_topic = dual ? sensors.lidars[1].topic : "";
  const std::string &imu_topic = sensors.imu_topic;
  lidar_time_offset_ = reference.time_offset;
  std::set<std::string> input_topics;
  for (const auto &topic : {imu_topic, lidar_topic, rear_lidar_topic}) {
    if (!topic.empty() && !input_topics.insert(get_node_topics_interface()->resolve_topic_name(topic)).second)
      throw std::invalid_argument("Sensor topics resolve to the same ROS topic");
  }
  const std::string scan_topic = parameter(*this, "topics.scan", std::string("/map_scan"));
  const std::string local_map_topic = parameter(*this, "topics.local_map", std::string("/map_cmap"));
  const std::string odom_topic = parameter(*this, "topics.odom", std::string("/odom"));
  const std::string map_pose_topic = parameter(*this, "topics.map_pose", std::string("/map_odom"));
  map_frame_ = parameter(*this, "frames.map", map_frame_);
  odom_frame_ = parameter(*this, "frames.odom", odom_frame_);
  base_frame_ = parameter(*this, "frames.base", base_frame_);
  const int imu_qos_depth = parameter(*this, "qos.imu_depth", 1000);
  const int lidar_qos_depth = parameter(*this, "qos.lidar_depth", 5);
  if (imu_qos_depth <= 0 || lidar_qos_depth <= 0) {
    throw std::invalid_argument("QoS depths must be positive");
  }

  auto parameters = sapphire::load_parameters(algorithm_config, reference);
  if (dual) {
    sapphire::LidarSyncParameters lidar_parameters;
    const auto transform = [](const sapphire::SensorParameters &sensor) {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
      result.translation() = Eigen::Map<const Eigen::Vector3d>(sensor.extrinsic_tran.data());
      result.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(sensor.extrinsic_rota.data());
      return result;
    };
    lidar_parameters.extrinsic = transform(reference).inverse() * transform(sensors.lidars[1]);
    lidar_parameters.front_time_offset = reference.time_offset;
    lidar_parameters.rear_time_offset = sensors.lidars[1].time_offset;
    lidar_parameters.max_time_gap = sensors.max_time_gap;
    lidar_parameters.max_wait = sensors.max_wait;
    lidar_parameters.lidar_merge_hz = sensors.lidar_merge_hz;
    lidar_parameters.imu_rate_hz = sensors.imu_rate_hz;
    lidar_parameters.min_points = static_cast<size_t>(sensors.min_points);
    lidar_parameters.queue_size = static_cast<size_t>(sensors.queue_size);
    lidar_parameters.max_scan_duration = parameters.sensor.max_scan_duration;
    lidar_parameters.point_filter_num = parameters.sensor.point_filter_num;
    lidar_parameters.blind_squared = parameters.sensor.blind_squared;
    lidar_sync_ = std::make_unique<sapphire::LidarSync>(lidar_parameters);
    RCLCPP_INFO(get_logger(), "dual fusion=%.3f Hz (0=front-frame), expected IMU=%.1f Hz; hardware scan rate unchanged",
                lidar_parameters.lidar_merge_hz, lidar_parameters.imu_rate_hz);
  } else {
    lidar_handler_ = std::make_unique<LidarHandler>(parameters.sensor);
  }

  rcl_interfaces::msg::ParameterDescriptor publication_descriptor;
  publication_descriptor.read_only = true;
  publication_descriptor.description = "Startup policy for optional visualization outputs";
  if (declare_parameter<bool>("publish.scan", true, publication_descriptor))
    scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(scan_topic, rclcpp::QoS(10));
  if (declare_parameter<bool>("publish.local_map", true, publication_descriptor))
    cmap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(local_map_topic, rclcpp::QoS(2));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(100));
  map_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(map_pose_topic, rclcpp::QoS(10));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  for (const auto *frame : {&map_frame_, &odom_frame_, &base_frame_})
    if (frame->empty() || frame->size() > 255) throw std::length_error("B3 frame identifier exceeds envelope");
  output_period_ =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(parameters.pose_graph.update_period_sec));
  sapphire::SlamPipeline::Callbacks output;
  output.ready_changed = [this] {
    ready_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
    if (map_dirty_.exchange(true)) ++map_coalesced_;
    output_cv_.notify_one();
  };
  output.owner_reset = [this](bool begin) { reset_output(begin); };
  output.recovery_changed = [this, map_base = map_frame_, odom_base = odom_frame_](const sapphire::RecoveryProgress& status) {
    if (status.phase == sapphire::TrackingPhase::Resetting) {
      lidar_reset_requested_.store(true);
      // owner_reset(true) has joined active publication and fenced new output.
      // A reset origin must never masquerade as the previous odom/map frame.
      std::lock_guard<std::mutex> lock(output_mutex_);
      map_frame_ = map_base + "_session_" + std::to_string(status.generation);
      odom_frame_ = odom_base + "_session_" + std::to_string(status.generation);
    }
    RCLCPP_INFO(get_logger(), "recovery generation=%lu phase=%d reason=%s", status.generation,
                static_cast<int>(status.phase), sapphire::recoveryReasonName(status.reason));
  };
  output.optional_drop = [this](bool incremental) {
    std::lock_guard<std::mutex> lock(output_mutex_);
    ++optional_dropped_;
    if (incremental) ++incremental_dropped_;
  };
  output.odom_state = [this](const sapphire::StateGroup &state) { enqueue_output(0, [this, state] { publish_odom(state); }); };
  if (scan_pub_) {
    output.local_scan = [this](std::shared_ptr<const sapphire::vvec<double, 3>> points) {
      const auto bytes = points ? points->capacity() * sizeof(Eigen::Vector3d) : 0;
      enqueue_output(
          1, [this, points = std::move(points)] { publish_scan(points); }, bytes);
    };
  }
  if (cmap_pub_) {
    output.local_map = [this](std::shared_ptr<const sapphire::vvec<double, 3>> points) {
      const auto bytes = points ? points->capacity() * sizeof(Eigen::Vector3d) : 0;
      enqueue_output(
          2, [this, points = std::move(points)] { publish_local_map(points); }, bytes);
    };
  }
  pipeline_ = std::make_unique<sapphire::SlamPipeline>(parameters, std::move(output));
  output_thread_ = std::thread(&SapphireNode::output_loop, this);

  imu_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  lidar_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions imu_options;
  imu_options.callback_group = imu_callback_group_;
  rclcpp::SubscriptionOptions lidar_options;
  lidar_options.callback_group = lidar_callback_group_;
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
    lidar_timer_ = create_wall_timer(
        std::chrono::milliseconds(20), [this] { drain_lidar(); }, lidar_callback_group_);
  }

  finish_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { finish_callback(); });

  RCLCPP_INFO(get_logger(), "LiDAR inputs=%zu, sensors=%s, algorithm=%s", sensors.lidars.size(), sensor_config.c_str(), algorithm_config.c_str());
}

SapphireNode::~SapphireNode() {
  finish_timer_.reset();
  lidar_timer_.reset();
  lidar_sub_.reset();
  rear_lidar_sub_.reset();
  imu_sub_.reset();
  try {
    finish_checked();
  } catch (...) {
    stop_output();
  }
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
  if (!prepare_lidar()) return;
  if (lidar_sync_) {
    lidar_sync_->push(false, decode_lidar_scan(message), message->header.frame_id);
    drain_lidar();
    return;
  }
  double timestamp = 0.0;
  std::optional<double> end_timestamp;
  std::vector<sapphire::LidarPoint> points;
  if (!lidar_handler_->process(message, timestamp, points, end_timestamp)) {
    pipeline_->rejectLidar(rclcpp::Time(message->header.stamp).seconds(), sapphire::RecoveryReason::InvalidLidar);
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported or malformed PointCloud2 input");
    return;
  }
  timestamp += lidar_time_offset_;
  if (end_timestamp) *end_timestamp += lidar_time_offset_;
  if (!pipeline_->push_lidar(timestamp, std::move(points), end_timestamp)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected invalid or out-of-order LiDAR scan");
  }
}

void SapphireNode::rear_lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message) {
  if (shutdown_started_.load()) return;
  if (!prepare_lidar()) return;
  lidar_sync_->push(true, decode_lidar_scan(message), message->header.frame_id);
  drain_lidar();
}

bool SapphireNode::prepare_lidar() {
  if (lidar_reset_requested_.exchange(false) && lidar_sync_) lidar_sync_->reset();
  const auto state = pipeline_->recoveryProgress();
  return !state.pending && state.phase != sapphire::TrackingPhase::Resetting && state.phase != sapphire::TrackingPhase::Failed;
}
void SapphireNode::drain_lidar() {
  if (shutdown_started_.load() || !prepare_lidar()) return;
  while (auto scan = lidar_sync_->pop()) {
    if (lidar_sync_->statistics().dropped() != last_lidar_drop_count_) {
      last_lidar_drop_count_ = lidar_sync_->statistics().dropped();
      pipeline_->notifyInputLoss();
      return;
    }
    if (!pipeline_->push_lidar(scan->begin, std::move(scan->points), scan->end)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected fused LiDAR scan");
    }
  }
  const auto &s = lidar_sync_->statistics();
  if (s.dropped() != last_lidar_drop_count_) {
    last_lidar_drop_count_ = s.dropped();
    pipeline_->notifyInputLoss();
  }
  const auto now = std::chrono::steady_clock::now();
  if (s.dropped() != reported_lidar_drop_count_ && now - last_lidar_report_ >= std::chrono::seconds(5)) {
    reported_lidar_drop_count_ = s.dropped();
    last_lidar_report_ = now;
    RCLCPP_WARN(get_logger(), "dual totals: emitted=%zu invalid=%zu out_of_order=%zu overflow=%zu timeout=%zu coverage=%zu empty=%zu", s.emitted,
                s.invalid, s.out_of_order, s.overflow, s.timeout, s.coverage, s.empty);
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
    ++optional_dropped_;
    if (slot == 2) ++incremental_dropped_;
    return;
  }
  if (output_slots_[slot]) {
    ++coalesced_;
    ++optional_dropped_;
    if (slot == 2) ++incremental_dropped_;
  }
  output_slots_[slot] = std::move(task);
  output_slot_bytes_[slot] = payload_bytes;
  std::size_t retained = active_optional_bytes_;
  for (auto bytes : output_slot_bytes_) retained += bytes;
  optional_copy_high_water_ = std::max(optional_copy_high_water_, retained);
  std::size_t count = 0;
  for (const auto &value : output_slots_) count += bool(value);
  pending_high_water_ = std::max(pending_high_water_, count);
  output_cv_.notify_one();
}

void SapphireNode::reset_output(bool begin) {
  std::unique_lock<std::mutex> lock(output_mutex_);
  if (begin) {
    output_resetting_ = true;
    output_cv_.notify_all();
    for (std::size_t i = 0; i < output_slots_.size(); ++i)
      if (output_slots_[i]) {
        ++optional_dropped_;
        if (i == 2) ++incremental_dropped_;
        output_slots_[i] = {};
        output_slot_bytes_[i] = 0;
      }
    output_cv_.wait(lock, [this] { return !output_active_; });
    correction_group_ = {};
    map_uuid_.clear();
    generation_ = 0;
    retry_requested_.store(false);
    ready_ns_.store(0);
    map_dirty_.store(false);
  } else {
    output_resetting_ = false;
    // Failed replacement terminates this executor: even late optional producer
    // callbacks must not restart old/new-domain output after acknowledgment.
    if (pipeline_ && pipeline_->failed()) {
      output_stopping_ = true;
      map_dirty_.store(false);
    } else
      map_dirty_.store(true);
    output_cv_.notify_all();
  }
}

void SapphireNode::retry_publication() {
  std::lock_guard<std::mutex> lock(output_mutex_);
  retry_requested_.store(true);
  map_dirty_.store(true);
  output_cv_.notify_one();
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
        return output_stopping_ ||
               (!output_resetting_ && (final_requested_ || map_dirty_.load() ||
                                       std::any_of(output_slots_.begin(), output_slots_.end(), [](const auto &v) { return bool(v); })));
      });
      if (output_stopping_) break;
      if (output_resetting_) {
        next_poll = std::chrono::steady_clock::now() + output_period_;
        continue;
      }
      final = final_requested_;
      mapping_ok = mapping_ok_;
      map = final || map_dirty_.exchange(false) || std::chrono::steady_clock::now() >= next_poll;
      if (map)
        next_poll = std::chrono::steady_clock::now() + output_period_;
      else
        for (; slot < output_slots_.size(); ++slot)
          if (output_slots_[slot]) {
            optional = std::move(output_slots_[slot]);
            output_slots_[slot] = {};
            active_optional_bytes_ = output_slot_bytes_[slot];
            output_slot_bytes_[slot] = 0;
            break;
          }
      output_active_ = true;
    }
    try {
      if (map) {
        if (mapping_ok) attempt_map(final);
      } else if (optional)
        optional();
    } catch (...) {
      if (optional) {
        std::lock_guard<std::mutex> lock(output_mutex_);
        ++optional_dropped_;
        if (slot == 2) ++incremental_dropped_;
      } else {
        if (!correction_group_.first_error) correction_group_.first_error = std::current_exception();
      }
    }
    {
      std::lock_guard<std::mutex> lock(output_mutex_);
      output_active_ = false;
      active_optional_bytes_ = 0;
      if (final) {
        output_stopping_ = true;
        final_requested_ = false;
        map_dirty_.store(false);
      }
    }
    output_cv_.notify_all();
    if (final) break;
  }
}

void SapphireNode::stop_output() {
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    output_stopping_ = true;
  }
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
      pipeline_->shutdown();
      producers_joined = std::chrono::steady_clock::now();
      auto progress = pipeline_->drain();
      target_source = progress.last_completed;
      if (progress.completed_revision) target_revision = progress.ready_revision;
    }
  } catch (...) {
    mapping_error = std::current_exception();
    RCLCPP_ERROR(get_logger(), "SLAM drain failed: %s", output_error().c_str());
  }
  const auto drained = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    mapping_error_ = mapping_error;
    final_revision_ = target_revision;
    final_source_ = target_source;
    mapping_ok_ = !mapping_error_;
    for (std::size_t i = 0; i < output_slots_.size(); ++i)
      if (output_slots_[i]) {
        ++optional_dropped_;
        if (i == 2) ++incremental_dropped_;
        output_slots_[i] = {};
        output_slot_bytes_[i] = 0;
      }
    final_requested_ = true;
  }
  output_cv_.notify_all();
  if (output_thread_.joinable()) output_thread_.join();
  const auto published = std::chrono::steady_clock::now();
  try {
    if (pipeline_) pipeline_->close();
    close_success_ = true;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Map close failed: %s", output_error().c_str());
  }
  finished_ = true;
  finish_success_ = !mapping_error_ && close_success_ && get_node_base_interface()->get_context()->is_valid() &&
                    (!final_revision_ || correction_group_.published == final_revision_);
  if (!get_node_base_interface()->get_context()->is_valid())
    RCLCPP_ERROR(get_logger(), "SLAM shutdown failed: ROS context unavailable");
  if (final_revision_ && correction_group_.published != final_revision_) {
    std::string detail = correction_group_.error;
    if (detail.empty() && correction_group_.first_error) {
      try {
        std::rethrow_exception(correction_group_.first_error);
      } catch (...) {
        detail = output_error();
      }
    }
    RCLCPP_ERROR(get_logger(), "SLAM final publication failed: target=%lu published=%lu cause=%s",
                 *final_revision_, correction_group_.published.value_or(0),
                 detail.empty() ? "Final map output unavailable" : detail.c_str());
  }
  const auto ms = [](auto duration) { return std::chrono::duration<double, std::milli>(duration).count(); };
  RCLCPP_INFO(
      get_logger(),
      "SLAM shutdown success=%d F=%lu Pc=%lu frontend_join_ms=%.3f backend_drain_ms=%.3f final_output_ms=%.3f close_ms=%.3f total_ms=%.3f "
      "pending_high_water=%zu coalesced=%lu optional_drops=%lu increment_drops=%lu retries=%lu capture_ms=%.3f publication_ms=%.3f",
      finish_success_, final_revision_.value_or(0), correction_group_.published.value_or(0), ms(producers_joined - start),
      ms(drained - producers_joined), ms(published - drained), ms(std::chrono::steady_clock::now() - published),
      ms(std::chrono::steady_clock::now() - start), pending_high_water_, coalesced_, optional_dropped_, incremental_dropped_, retries_, capture_ms_,
      publication_ms_);
  RCLCPP_INFO(
      get_logger(),
      "SLAM output timing samples=%zu ready_selection_ms_sum=%.3f selection_success_ms_sum=%.3f ready_success_ms_sum=%.3f "
      "conversion_serialization_ms=%.3f checked_publish_ms=%.3f payload_copy_high_water=%zu optional_copy_high_water=%zu required_coalesced=%lu",
      success_samples_, ready_selection_ms_, selection_success_ms_, ready_success_ms_, conversion_ms_, checked_publish_ms_, payload_copy_high_water_,
      optional_copy_high_water_, map_coalesced_.load());
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
  if (!points || !scan_pub_) {
    return;
  }
  std::size_t copies = 0;
  auto message = point_cloud_message(*points, now(), odom_frame_.c_str(), &copies);
  observe_optional_copy(copies);
  scan_pub_->publish(message);
}

void SapphireNode::publish_local_map(std::shared_ptr<const sapphire::vvec<double, 3>> points) {
  if (!points || !cmap_pub_) {
    return;
  }
  std::size_t copies = 0;
  auto message = point_cloud_message(*points, now(), odom_frame_.c_str(), &copies);
  observe_optional_copy(copies);
  cmap_pub_->publish(message);
}

namespace {
std::string output_error() noexcept {
  try {
    throw;
  } catch (const std::exception &e) {
    try {
      return std::string(e.what()).substr(0, 1024);
    } catch (...) {
    }
  } catch (...) {
  }
  return "output failure";
}
}  // namespace

void SapphireNode::publish_map(double timestamp, const Eigen::Isometry3d &correction, const Eigen::Isometry3d &pose) {
  auto context = get_node_base_interface()->get_context();
  nav_msgs::msg::Odometry message;
  message.header.stamp = stamp_from_seconds(timestamp);
  message.header.frame_id = map_frame_;
  message.child_frame_id = base_frame_;
  message.pose.pose.position.x = pose.translation().x();
  message.pose.pose.position.y = pose.translation().y();
  message.pose.pose.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.rotation());
  message.pose.pose.orientation.x = q.x();
  message.pose.pose.orientation.y = q.y();
  message.pose.pose.orientation.z = q.z();
  message.pose.pose.orientation.w = q.w();
  checked_publish<nav_msgs::msg::Odometry>(map_odom_pub_, context, message, 65536, &conversion_ms_, &checked_publish_ms_);
  if (!context->is_valid()) throw std::runtime_error("Context lost before TF");
  tf_broadcaster_->sendTransform(make_transform(now(), map_frame_.c_str(), odom_frame_.c_str(), correction));
  if (!context->is_valid()) throw std::runtime_error("Context lost during TF");
}

void SapphireNode::attempt_map(bool final) {
  if (!pipeline_ || pipeline_->failed()) return;
  if (!get_node_base_interface()->get_context()->is_valid()) {
    if (!correction_group_.first_error) correction_group_.first_error = std::make_exception_ptr(std::runtime_error("ROS context unavailable"));
    correction_group_.error = "ROS context unavailable";
    return;  // Context loss never schedules a transport retry.
  }
  const auto selected_at = std::chrono::steady_clock::now();
  const auto ready_at = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ready_ns_.load()));
  auto progress = pipeline_->continuationProgress();
  if (progress.first_failure || !progress.completed_revision) return;
  const auto may_attempt = [&](const Group &group) {
    if (group.published && *group.published >= progress.ready_revision) return false;
    if (final || retry_requested_.load() || group.target != progress.ready_revision) return true;
    return group.online_attempts < 2 && (!group.online_attempts || std::chrono::steady_clock::now() - group.attempted >= output_period_);
  };
  if (!may_attempt(correction_group_)) return;
  std::optional<sapphire::SlamPipeline::ContinuationProgress> captured;
  double timestamp = 0;
  Eigen::Isometry3d correction, pose;
  const auto ms = [](auto start) { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count(); };
  auto capture = [&]() {
    auto start = std::chrono::steady_clock::now();
    captured = pipeline_->captureReady(!final, timestamp, correction, pose);
    capture_ms_ += ms(start);
    return captured.has_value();
  };
  try {
    if (!capture()) return;
  } catch (...) {
    const auto cause = std::current_exception();
    const auto detail = output_error();
    const bool correction_due = may_attempt(correction_group_);
    const bool explicit_request = retry_requested_.exchange(false);
    const auto failed_capture = [&](Group &group) {
      if (group.target != progress.ready_revision) {
        group.target = progress.ready_revision;
        group.online_attempts = 0;
      }
      if (!final && !explicit_request) ++group.online_attempts;
      group.attempted = std::chrono::steady_clock::now();
      if (!group.first_error) group.first_error = cause;
      group.error = detail;
    };
    if (correction_due) failed_capture(correction_group_);
    return;
  }
  const auto revision = captured->ready_revision;
  const auto source = captured->last_completed;
  const auto generation = captured->generation;
  const auto &uuid = captured->map_id;
  if (final && (!final_revision_ || revision != *final_revision_ || source != final_source_))
    throw std::logic_error("Final capture disagrees with frozen drain target");
  if (uuid != map_uuid_ || generation != generation_) {
    if (!map_uuid_.empty()) {
      correction_group_ = {};
    }
    map_uuid_ = uuid;
    generation_ = generation;
  }
  const bool explicit_request = retry_requested_.exchange(false);
  bool correction_explicit = explicit_request;
  const auto eligible = [&](Group &group) {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if (group.published && *group.published >= revision) return false;
    if (group.target != revision) {
      group.target = revision;
      group.online_attempts = 0;
      group.error.clear();
    }
    bool &request = correction_explicit;
    bool explicit_retry = request;
    request = false;
    if (final) {
      ++retries_;
      return true;
    }
    if (explicit_retry) {
      ++retries_;
      return true;
    }
    if (group.online_attempts >= 2) return false;
    if (group.online_attempts && std::chrono::steady_clock::now() - group.attempted < output_period_) return false;
    if (group.online_attempts++) ++retries_;
    group.attempted = std::chrono::steady_clock::now();
    return true;
  };
  const auto record_success = [&] {
    const auto success = std::chrono::steady_clock::now();
    if (ready_at.time_since_epoch().count()) {
      ready_selection_ms_ += std::chrono::duration<double, std::milli>(selected_at - ready_at).count();
      selection_success_ms_ += std::chrono::duration<double, std::milli>(success - selected_at).count();
      ready_success_ms_ += std::chrono::duration<double, std::milli>(success - ready_at).count();
      ++success_samples_;
    }
  };
  const auto correction_attempt = [&] {
    if (!eligible(correction_group_)) return;
    auto start = std::chrono::steady_clock::now();
    try {
      publish_map(timestamp, correction, pose);
      correction_group_.published = revision;
      correction_group_.error.clear();
      record_success();
    } catch (...) {
      if (!correction_group_.first_error) correction_group_.first_error = std::current_exception();
      correction_group_.error = output_error();
    }
    publication_ms_ += ms(start);
  };
  correction_attempt();
}

}  // namespace sapphire_ros
