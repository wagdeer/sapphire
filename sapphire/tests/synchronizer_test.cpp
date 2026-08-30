#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "camera.hpp"
#include "memory.hpp"
#include "parallel_executor.hpp"
#include "pipeline.hpp"

namespace {

void check(bool condition, const char *message) {
  if (condition) {
    return;
  }
  std::cerr << "check failed: " << message << '\n';
  std::exit(1);
}

sapphire::ImuMeas imu(double timestamp) {
  sapphire::ImuMeas value;
  value.timestamp = timestamp;
  value.gyro.setZero();
  value.accel = Eigen::Vector3d(0.0, 0.0, 9.81);
  return value;
}

sapphire::LidarPoint point(float time_offset) {
  sapphire::LidarPoint value;
  value.x = 1.0F;
  value.y = 2.0F;
  value.z = 3.0F;
  value.intensity = 4.0F;
  value.time_offset = time_offset;
  return value;
}

}  // namespace

int main() {
  sapphire::CameraModel camera(1280, 720, 500.0F, 500.0F, 640.0F, 360.0F, 0.6F);
  const Eigen::Vector3f camera_point(0.2F, -0.1F, 1.0F);
  const std::optional<Eigen::Vector2f> pixel = camera.project(camera_point);
  check(pixel.has_value(), "UCM projection");
  const std::optional<Eigen::Vector3f> bearing = camera.unproject(*pixel);
  check(bearing.has_value() && bearing->dot(camera_point.normalized()) > 1.0F - 1e-5F, "UCM round trip");

  sapphire::ParallelExecutor executor(4);
  std::vector<int> visits(32, 0);
  std::vector<std::thread::id> first_worker_ids(4);
  std::vector<std::thread::id> second_worker_ids(4);
  bool foreground_executed = false;
  const size_t worker_count = executor.parallel_for(
      visits.size(), 4,
      [&](size_t worker_index, size_t begin, size_t end) {
        first_worker_ids[worker_index] = std::this_thread::get_id();
        for (size_t index = begin; index < end; ++index) {
          visits[index] = static_cast<int>(worker_index + 1);
        }
      },
      [&] { foreground_executed = true; });
  check(worker_count == 4, "parallel worker count");
  check(foreground_executed, "parallel foreground task");
  for (int visit : visits) {
    check(visit > 0, "parallel range coverage");
  }
  executor.parallel_for(visits.size(), 4, [&](size_t worker_index, size_t, size_t) { second_worker_ids[worker_index] = std::this_thread::get_id(); });
  check(first_worker_ids == second_worker_ids, "parallel workers reused");

  const sapphire::SapphireParameters file_parameters = sapphire::load_parameters(std::string(SAPPHIRE_SOURCE_DIR) + "/config/mid360.toml");
  check(file_parameters.sensor.point_filter_num == 1, "TOML sensor parameters");
  check(file_parameters.pose_graph.enabled, "TOML pose graph parameters");
  check(file_parameters.pose_graph.submap_travel_distance == 15.0 && file_parameters.pose_graph.submap_max_point_range == 20.0,
        "TOML submap parameters");

  sapphire::SapphireParameters parameters;
  parameters.sensor.blind = 0.2;
  parameters.local_submap.plane_eigen_value_thre = {2.0, 4.0, 5.0, 10.0};
  sapphire::validate_parameters(parameters);
  check(std::abs(parameters.sensor.blind_squared - 0.04) < 1e-12, "blind squared");
  check(std::abs(parameters.odometry.down_size_inv - 10.0) < 1e-12, "down size inverse");
  check(std::abs(parameters.local_submap.plane_eigen_value_thre_inv[1] - 0.25) < 1e-12, "plane threshold inverse");

  sapphire::SubmapFrameBuffer submap_buffer(1.0, 15.0, 20.0);
  submap_buffer.push_visual(sapphire::VisualFrame(1.0));
  submap_buffer.push_visual(sapphire::VisualFrame(2.0));
  sapphire::StateGroup first_state;
  auto first_cloud = std::make_shared<sapphire::PointCloud>();
  first_cloud->push_back({Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Matrix3d::Zero()});
  first_cloud->push_back({Eigen::Vector3d(21.0, 0.0, 0.0), Eigen::Matrix3d::Zero()});
  const Eigen::Matrix<double, 6, 1> variance = Eigen::Matrix<double, 6, 1>::Zero();
  sapphire::MargiFrame first_frame(first_state, std::move(first_cloud), 1.0, variance, 0.0);
  check(!submap_buffer.push(first_frame), "submap remains open before target distance");

  sapphire::StateGroup second_state;
  second_state.p.x() = 15.0;
  auto second_cloud = std::make_shared<sapphire::PointCloud>();
  second_cloud->push_back({Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Matrix3d::Zero()});
  sapphire::MargiFrame second_frame(second_state, std::move(second_cloud), 2.0, variance, 15.0);
  std::optional<sapphire::SubmapFrame> completed_submap = submap_buffer.push(second_frame);
  check(completed_submap && completed_submap->frame_count() == 2 && completed_submap->lio().pcd->size() == 2,
        "submap directly fuses marginal frames and crops distant points");
  check(completed_submap->visual_frames().size() == 2, "submap retains every visual frame");
  check(completed_submap->pyramid_voxels().valid(), "submap builds pyramid voxels with its final cloud");
  check(completed_submap->bounds().minimum.isApprox(Eigen::Vector3f(1.0F, 0.0F, 0.0F)) &&
            completed_submap->bounds().maximum.isApprox(Eigen::Vector3f(16.0F, 0.0F, 0.0F)),
        "submap stores finite local point-cloud bounds");
  check(completed_submap->navigation().samples.size() == 2 && completed_submap->navigation().spline.valid() &&
            std::abs(completed_submap->navigation().samples.back().distance - 15.0F) < 1e-5F &&
            completed_submap->navigation().spline.evaluate(0.0F).isApprox(Eigen::Vector3f::Zero(), 1e-5F) &&
            completed_submap->navigation().spline.evaluate(1.0F).isApprox(Eigen::Vector3f(15.0F, 0.0F, 0.0F), 1e-5F),
        "submap retains raw trajectory and fits a local B-spline");
  const auto [closest_path_point, closest_path_parameter] =
      completed_submap->navigation().spline.closest_point(Eigen::Vector3f(7.5F, 2.0F, 0.0F));
  const sapphire::vvec<float, 3> navigation_waypoints = completed_submap->navigation().spline.sample_by_arc_length(5.0F);
  check(closest_path_point.isApprox(Eigen::Vector3f(7.5F, 0.0F, 0.0F), 1e-4F) &&
            std::abs(closest_path_parameter - 0.5F) < 1e-4F && navigation_waypoints.size() == 4,
        "navigation spline supports nearest-path and arc-length queries");

  const std::filesystem::path database_path =
      std::filesystem::temp_directory_path() /
      ("sapphire-core-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".db");
  sapphire::LioFrame lio;
  auto submap_cloud = std::make_shared<sapphire::vvec<float, 3>>();
  submap_cloud->emplace_back(1.0F, 2.0F, 3.0F);
  lio.pcd = submap_cloud;
  lio.timestamp = 1.0;
  sapphire::VisualFrame visual(1.01);
  std::vector<sapphire::VisualPoint> visual_points;
  visual_points.emplace_back(Eigen::Vector2f(12.0F, 34.0F), 2, 0.8F);
  cv::Mat local_descriptor(1, 2, CV_8UC1);
  local_descriptor.at<std::uint8_t>(0, 0) = 7;
  local_descriptor.at<std::uint8_t>(0, 1) = 9;
  visual.update_features(std::move(visual_points), std::move(local_descriptor));
  std::vector<sapphire::VisualFrame> visual_frames;
  visual_frames.emplace_back(std::move(visual));
  visual_frames.emplace_back(1.02);
  cpu::VoxelMaps voxelmaps;
  voxelmaps.set_min_res(0.5F);
  voxelmaps.set_max_level(3);
  voxelmaps.create_voxelmaps(lio.pcd->data(), lio.pcd->size());
  sapphire::NavigationPath navigation;
  navigation.samples.push_back({1.00, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F});
  navigation.samples.push_back({1.01, 1.0F, 0.2F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 1.0F});
  navigation.samples.push_back({1.02, 2.0F, 0.4F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 2.0F});
  navigation.samples.push_back({1.03, 3.0F, 0.3F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 3.0F});
  navigation.samples.push_back({1.04, 4.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 4.0F});
  navigation.spline = sapphire::fit_navigation_spline(navigation.samples);
  check(navigation.spline.valid(), "manual navigation spline fit");
  sapphire::SubmapFrame submap(0, std::move(lio), 0, 9, 10, 0.0, 15.0, voxelmaps.release_data(), std::move(navigation),
                               std::move(visual_frames));
  cv::Mat retrieval_descriptor = (cv::Mat_<float>(1, 3) << 0.1F, 0.2F, 0.3F);
  submap.attach_tag(sapphire::MetaTag(sapphire::MetaTagType::kFloat, std::move(retrieval_descriptor)));
  sapphire::LocalGrid local_grid(0.1F);
  {
    sapphire::Memory memory(database_path.string());
    memory.saveSubmap(submap, local_grid);
    check(memory.recallSpatial(0, Eigen::Isometry3f::Identity()).size() == 1, "spatial index receives saved submap bounds");
    const std::vector<sapphire::RecallMatch> initial_matches = memory.recall(0, Eigen::Isometry3f::Identity(), 1, 10.0F);
    check(initial_matches.size() == 1 && initial_matches.front().submap_id == 0, "mandatory pose index");
    Eigen::Isometry3f optimized_pose = Eigen::Isometry3f::Identity();
    optimized_pose.translation().x() = 5.0F;
    memory.saveOptimizedPose(1, optimized_pose);
    const std::vector<sapphire::SpatialMatch> optimized_spatial = memory.recallSpatial(0, optimized_pose);
    check(optimized_spatial.size() == 1 && optimized_spatial.front().map_T_submap.translation().x() == 5.0F,
          "optimized pose updates R-tree entry");
    const std::vector<sapphire::RecallMatch> optimized_matches = memory.recall(0, optimized_pose, 1, 0.1F);
    check(optimized_matches.size() == 1 && optimized_matches.front().pose.translation().x() == 5.0F, "optimized pose index");
    check(memory.loadCloud(0)->size() == 1, "cold cloud load");
    check(memory.loadPyramidVoxel(0).valid(), "cold pyramid voxel load");
    const std::vector<sapphire::VisualFrame> loaded_visuals = memory.loadVisualFrames(0);
    check(loaded_visuals.size() == 2 && loaded_visuals.front().points().size() == 1, "cold visual load");
    const sapphire::NavigationPath loaded_navigation = memory.loadNavigation(0);
    check(loaded_navigation.samples.size() == 5 && loaded_navigation.spline.valid() &&
              loaded_navigation.samples[2].timestamp == 1.02 && loaded_navigation.samples[2].distance == 2.0F &&
              loaded_navigation.samples[2].qw == 1.0F &&
              loaded_navigation.spline.evaluate(0.5F).isApprox(submap.navigation().spline.evaluate(0.5F), 1e-5F),
          "cold raw trajectory and cubic B-spline load");
  }
  {
    sapphire::Memory memory(database_path.string());
    Eigen::Isometry3f optimized_pose = Eigen::Isometry3f::Identity();
    optimized_pose.translation().x() = 5.0F;
    const std::vector<sapphire::RecallMatch> reloaded_matches = memory.recall(0, optimized_pose, 1, 0.1F);
    check(reloaded_matches.size() == 1 && reloaded_matches.front().pose.translation().x() == 5.0F, "pose index reload");
    const std::vector<sapphire::SpatialMatch> reloaded_spatial = memory.recallSpatial(0, optimized_pose);
    check(reloaded_spatial.size() == 1 && reloaded_spatial.front().map_T_submap.translation().x() == 5.0F,
          "persisted local bounds rebuild optimized R-tree entry");
  }
  std::filesystem::remove(database_path);
  std::filesystem::remove(database_path.string() + "-wal");
  std::filesystem::remove(database_path.string() + "-shm");

  sapphire::Synchronizer synchronizer;
  check(synchronizer.push_imu(imu(0.90)), "first imu accepted");
  check(!synchronizer.push_imu(imu(0.90)), "duplicate imu rejected");
  check(!synchronizer.push_imu(imu(0.80)), "reverse imu rejected");
  check(synchronizer.push_imu(imu(0.95)), "second imu accepted");
  check(synchronizer.push_imu(imu(1.00)), "third imu accepted");
  check(synchronizer.push_imu(imu(1.03)), "fourth imu accepted");
  check(synchronizer.push_imu(imu(1.06)), "fifth imu accepted");
  check(synchronizer.push_imu(imu(1.12)), "terminal imu accepted");

  std::vector<sapphire::LidarPoint> cloud{point(0.12F), point(0.08F), point(0.01F)};
  check(synchronizer.push_lidar(1.0, cloud), "lidar accepted");
  check(!synchronizer.push_lidar(1.0, cloud), "duplicate lidar rejected");
  check(!synchronizer.push_lidar(0.9, cloud), "reverse lidar rejected");

  sapphire::MeasGroup measures;
  check(synchronizer.sync_packages(measures), "package synchronized");
  check(measures.imu_buf.size() > 4, "enough imu samples");
  check(measures.lidar_cloud->size() == 2, "long point clipped");
  check(measures.lidar_cloud->front().time_offset == 0.01F, "cloud sorted by offset");

  synchronizer.stop_accepting();
  check(!synchronizer.push_imu(imu(2.0)), "imu rejected after stop");
  check(!synchronizer.push_lidar(2.0, cloud), "lidar rejected after stop");
  return 0;
}
