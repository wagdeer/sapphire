#include "pipeline/backend.hpp"
#include "sapphire_ros2/sapphire_node.hpp"
#include "maplayer/node_builder.hpp"
#include <rcl/publisher.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <unistd.h>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <rcutils/logging.h>
#include <cstdio>
#include <mutex>
namespace {
bool fail_map = false;
bool fail_all_map = false;
std::mutex log_mutex;
std::string captured_errors;
rcutils_logging_output_handler_t original_log_handler = nullptr;
void capture_log(const rcutils_log_location_t *location, int severity, const char *name,
                 rcutils_time_point_value_t timestamp, const char *format, va_list *args) {
  if (severity >= RCUTILS_LOG_SEVERITY_ERROR) {
    char message[2048];
    va_list copy;
    va_copy(copy, *args);
    std::vsnprintf(message, sizeof(message), format, copy);
    va_end(copy);
    std::lock_guard<std::mutex> lock(log_mutex);
    captured_errors += std::string(message) + '\n';
  }
  if (original_log_handler) original_log_handler(location, severity, name, timestamp, format, args);
}
std::optional<nav_msgs::msg::Odometry> map_pose;
std::optional<sensor_msgs::msg::PointCloud2> scan;
std::optional<geometry_msgs::msg::TransformStamped> map_correction;
}
extern "C" rcl_ret_t __real_rcl_publish(const rcl_publisher_t*, const void*, rmw_publisher_allocation_t*);
extern "C" rcl_ret_t __wrap_rcl_publish(const rcl_publisher_t *pub, const void *message, rmw_publisher_allocation_t *allocation) {
  const std::string topic = rcl_publisher_get_topic_name(pub);
  if (topic == "/map_odom") {
    if (fail_all_map || std::exchange(fail_map, false)) return RCL_RET_ERROR;
    map_pose = *static_cast<const nav_msgs::msg::Odometry*>(message);
  }
  if (topic == "/map_scan") scan = *static_cast<const sensor_msgs::msg::PointCloud2*>(message);
  if (topic == "/tf") {
    for (const auto &t : static_cast<const tf2_msgs::msg::TFMessage*>(message)->transforms)
      if (t.header.frame_id == "map" && t.child_frame_id == "odom") map_correction = t;
  }
  return __real_rcl_publish(pub, message, allocation);
}
// TransformBroadcaster's shared-library call is not intercepted by --wrap=rcl_publish.
// Capture the single-transform API while retaining its real vector publication.
namespace tf2_ros {
void TransformBroadcaster::sendTransform(const geometry_msgs::msg::TransformStamped &transform) {
  if (transform.header.frame_id == "map" && transform.child_frame_id == "odom") map_correction = transform;
  sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{transform});
}
}
namespace sapphire {
struct SlamPipelineTestAccess {
  static void fail(SlamPipeline &pipeline) {
    pipeline.recordFailure(std::make_exception_ptr(std::runtime_error("injected worker failure")));
  }
  static void submit(SlamPipeline &pipeline) {
    StateGroup state; state.p = {3, -2, .8};
    GaussianCloud points;
    for (int x = 0; x < 20; ++x) for (int y = 0; y < 20; ++y) {
      GaussianPoint p; p.mean = {x * .1f - 1, y * .1f - 1, .1f};
      p.covariance = Eigen::Vector3f(.001f, .02f, .03f).asDiagonal();
      p.N = 20; p.radius = .5; p.voxel_key = {x, y, 0, 0}; points.push_back(p);
    }
    NodeBuilder buffer;
    (void)buffer.push(LidarFrame(state, std::move(points), 12, 0));
    auto submap = buffer.flush(); assert(submap);
    pipeline.backend_->submitNode(std::move(*submap));
  }
};
}
namespace sapphire_ros {
struct SapphireNodeTestAccess {
  static void pause(SapphireNode &n) { n.reset_output(true); }
  static void resume(SapphireNode &n) { n.reset_output(false); }
  static void submit(SapphireNode &n) { sapphire::SlamPipelineTestAccess::submit(*n.pipeline_); }
  static void fail(SapphireNode &n) { sapphire::SlamPipelineTestAccess::fail(*n.pipeline_); }
  static void publish(SapphireNode &n) { n.attempt_map(false); }
  static void display(SapphireNode &n) {
    auto points = std::make_shared<sapphire::vvec<double, 3>>();
    points->emplace_back(1.25, -2.5, 3.75);
    points->emplace_back(std::numeric_limits<double>::quiet_NaN(), 0, 0);
    n.publish_scan(points);
  }
  static void check_dual(SapphireNode &n) {
    assert(n.lidar_sync_ && !n.lidar_handler_);
    for (bool rear : {false, true}) {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      cloud->header.frame_id = rear ? "rear" : "front";
      cloud->height = 1; cloud->width = 5; cloud->point_step = 24; cloud->row_step = 120;
      cloud->data.resize(120);
      for (const auto &[name, offset, type] : std::vector<std::tuple<std::string, int, int>>{
          {"x",0,7}, {"y",4,7}, {"z",8,7}, {"intensity",12,7}, {"timestamp",16,8}}) {
        sensor_msgs::msg::PointField field; field.name=name; field.offset=offset; field.datatype=type; field.count=1;
        cloud->fields.push_back(field);
      }
      for (int i=0;i<5;++i) {
        const std::array<float,4> point{1,0,0,42};
        const double time = 100. + i*.05 + (rear ? .002 : -.012);
        std::memcpy(cloud->data.data()+24*i, point.data(), 16);
        std::memcpy(cloud->data.data()+24*i+16, &time, 8);
      }
      assert(n.lidar_sync_->push(rear, decode_lidar_scan(cloud), cloud->header.frame_id));
    }
    const auto scan = n.lidar_sync_->pop();
    assert(scan && std::abs(scan->begin-100.)<1e-8 && std::abs(scan->end-100.1)<1e-8);
    bool front=false, rear=false;
    for (const auto &p : scan->points) {
      front |= std::abs(p.x-1)<1e-6 && std::abs(p.y)<1e-6 && std::abs(p.z)<1e-6;
      rear |= std::abs(p.x-3)<1e-6 && std::abs(p.y+4)<1e-6 && std::abs(p.z-3)<1e-6;
    }
    assert(front && rear);
  }
  static void recover(SapphireNode& n) {
    n.pipeline_->notifyInputLoss();
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    for (;;) {
      const auto state=n.pipeline_->recoveryProgress();
      if(state.generation==1 && !state.pending && state.phase==sapphire::TrackingPhase::WaitingForInput)break;
      assert(std::chrono::steady_clock::now()<deadline);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    n.reset_output(true);
    assert(n.odom_frame_=="odom_session_1" && n.map_frame_=="map_session_1");
    assert(n.prepare_lidar());
    if(n.lidar_sync_) assert(n.lidar_sync_->buffered_front()==0 && n.lidar_sync_->buffered_rear()==0);
    assert(!n.pipeline_->continuationProgress().ready_available);
    assert(!n.correction_group_.published && n.map_uuid_.empty());
  }
  static auto error(SapphireNode &n) { return n.correction_group_.error; }
  static auto published(SapphireNode &n) { return n.correction_group_.published; }
  static auto progress(SapphireNode &n) { return n.pipeline_->continuationProgress(); }
};
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const auto directory = std::filesystem::temp_directory_path() / ("sapphire-ros-check-" + std::to_string(getpid()));
  std::filesystem::create_directories(directory);
  const auto config = directory / "config.toml";
  {
    std::ofstream out(config);
    out << "[general]\nsave_path='" << directory.string() << "'\n[map]\ndatabase_path='"
        << (directory / "map.db").string() << "'\n[mapping.registration.lidar]\nbbs_enabled=false\n";
  }
  auto node = std::make_shared<sapphire_ros::SapphireNode>(
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("algorithm_config", config.string()),
        rclcpp::Parameter("sensor_config", (std::filesystem::path(__FILE__).parent_path().parent_path() / "core/config/sensors.toml").string())}));
  using Access = sapphire_ros::SapphireNodeTestAccess;
  Access::pause(*node);
  Access::display(*node);
  assert(scan && scan->width == 2 && scan->point_step == 16 && scan->row_step == 32 && !scan->is_dense);
  assert(scan->fields.size() == 4 && scan->fields[3].name == "intensity" && scan->header.frame_id == "odom");
  std::array<float, 4> decoded;
  std::memcpy(decoded.data(), scan->data.data(), 16);
  assert(decoded[0] == 1.25F && decoded[1] == -2.5F && decoded[2] == 3.75F && decoded[3] == 0);
  Access::submit(*node);
  const auto progress = Access::progress(*node);
  assert(progress.ready_available && progress.completed_revision);
  fail_map = true; Access::publish(*node);
  assert(!Access::published(*node) && !map_correction);
  node->retry_publication(); Access::publish(*node);
  std::cerr << "published=" << Access::published(*node).value_or(0) << " expected=" << progress.ready_revision << " odom=" << bool(map_pose) << " tf=" << bool(map_correction) << " error=" << Access::error(*node) << "\n";
  assert(Access::published(*node) == progress.ready_revision && map_pose && map_correction);
  assert(map_pose->header.frame_id == "map" && map_pose->child_frame_id == "base_link");
  assert(map_pose->header.stamp.sec == 12 && std::abs(map_pose->pose.pose.position.x - 3.) < 1e-5);
  Access::resume(*node);
  Access::recover(*node); Access::display(*node);
  assert(scan->header.frame_id=="odom_session_1");
  Access::submit(*node);Access::publish(*node);
  assert(map_pose->header.frame_id=="map_session_1");
  Access::resume(*node);
  assert(node->finish_checked()); node.reset();
  for (bool worker_failure : {true, false}) {
    {
      std::ofstream out(config);
      out << "[map]\ndatabase_path='"
          << (directory / (worker_failure ? "worker.db" : "publication.db")).string()
          << "'\n[mapping.registration.lidar]\nbbs_enabled=false\n";
    }
    node = std::make_shared<sapphire_ros::SapphireNode>(rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("algorithm_config", config.string()),
        rclcpp::Parameter("sensor_config", (std::filesystem::path(__FILE__).parent_path().parent_path() / "core/config/sensors.toml").string())}));
    Access::pause(*node);
    Access::submit(*node);
    if (worker_failure) {
      Access::publish(*node);
      assert(Access::published(*node));
      Access::fail(*node);
    } else {
      fail_all_map = true;
    }
    // Checked finish must report failures without another sensor callback or
    // an executor spin, and preserve failure on repeated finish calls.
    captured_errors.clear();
    original_log_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(capture_log);
    Access::resume(*node);
    assert(!node->finish_checked());
    assert(!node->finish_checked());
    rcutils_logging_set_output_handler(original_log_handler);
    assert(captured_errors.find(worker_failure ? "SLAM drain failed: injected worker failure"
                                              : "SLAM final publication failed:") != std::string::npos);
    if (!worker_failure)
      assert(captured_errors.find("Required local ROS submission failed") != std::string::npos);
    node.reset();
    fail_all_map = false;
  }
  {
    std::ofstream out(config);
    out << "[general]\nsave_path='" << directory.string() << "'\n[pose_graph]\nenabled=false\n";
    std::ofstream sensors(directory / "sensors.toml");
    sensors << "[imu]\ntopic='/imu'\n[preprocess]\nmax_scan_duration=0.25\npoint_filter_num=1\n"
            << "[lidar0]\ntype='airy'\ntopic='/front'\ntime_offset=0.012\nextrinsic_tran=[1,2,3]\nextrinsic_rota=[0,-1,0,1,0,0,0,0,1]\n"
            << "[lidar1]\ntype='robosense'\ntopic='/rear'\ntime_offset=-0.002\nextrinsic_tran=[4,5,6]\nextrinsic_rota=[1,0,0,0,1,0,0,0,1]\n";
  }
  node = std::make_shared<sapphire_ros::SapphireNode>(rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("algorithm_config", config.string()),
      rclcpp::Parameter("sensor_config", (directory / "sensors.toml").string())}));
  Access::pause(*node); Access::check_dual(*node); Access::resume(*node);
  Access::recover(*node); Access::check_dual(*node); Access::resume(*node);
  assert(node->finish_checked()); node.reset(); rclcpp::shutdown();
  std::filesystem::remove_all(directory);
  std::cout << "PASS standard Odometry/TF, retry/drain and dual sensor calibration/time offsets\n";
}
