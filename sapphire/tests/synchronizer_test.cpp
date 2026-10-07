#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sqlite3.h>
#include <string>
#include <thread>
#include <vector>

#include "common/camera/ucm.hpp"
#include "backend/storage/memory.hpp"
#include "tools/parallel_executor.hpp"
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

sapphire::GaussianPoint gaussian(const Eigen::Vector3d &mean, int count, const sapphire::VOXEL_LOC &key,
                                 const Eigen::Matrix3d &covariance = Eigen::Matrix3d::Identity() / 9.0, bool is_plane = false) {
  sapphire::GaussianPoint value;
  value.mean = mean.cast<float>();
  value.covariance = covariance.cast<float>();
  value.voxel_key = key;
  value.N = count;
  value.is_plane = is_plane;
  value.regularize();
  return value;
}

std::vector<std::uint8_t> readGaussianPayload(const std::filesystem::path &path) {
  sqlite3 *database = nullptr;
  check(sqlite3_open_v2(path.c_str(), &database, SQLITE_OPEN_READWRITE, nullptr) == SQLITE_OK, "open Gaussian test database");
  sqlite3_stmt *statement = nullptr;
  check(sqlite3_prepare_v2(database, "SELECT payload FROM LaserRecord WHERE node_id=1;", -1, &statement, nullptr) == SQLITE_OK,
        "prepare Gaussian payload read");
  check(sqlite3_step(statement) == SQLITE_ROW, "read Gaussian payload");
  const auto *data = static_cast<const std::uint8_t *>(sqlite3_column_blob(statement, 0));
  const int bytes = sqlite3_column_bytes(statement, 0);
  std::vector<std::uint8_t> payload(data, data + bytes);
  sqlite3_finalize(statement);
  sqlite3_close_v2(database);
  return payload;
}

void writeGaussianPayload(const std::filesystem::path &path, const std::vector<std::uint8_t> &payload) {
  sqlite3 *database = nullptr;
  check(sqlite3_open_v2(path.c_str(), &database, SQLITE_OPEN_READWRITE, nullptr) == SQLITE_OK, "open Gaussian payload for mutation");
  sqlite3_stmt *statement = nullptr;
  check(sqlite3_prepare_v2(database, "UPDATE LaserRecord SET payload=? WHERE node_id=1;", -1, &statement, nullptr) == SQLITE_OK,
        "prepare Gaussian payload mutation");
  check(sqlite3_bind_blob(statement, 1, payload.data(), static_cast<int>(payload.size()), SQLITE_TRANSIENT) == SQLITE_OK,
        "bind mutated Gaussian payload");
  check(sqlite3_step(statement) == SQLITE_DONE, "write mutated Gaussian payload");
  sqlite3_finalize(statement);
  sqlite3_close_v2(database);
}

bool cloudLoadRejected(const std::filesystem::path &path) {
  try {
    sapphire::MapDatabase database(path.string(), "resume");
    static_cast<void>(database.loadCloud(1));
  } catch (const std::runtime_error &) {
    return true;
  }
  return false;
}

struct NodeStorage {
  sqlite3_int64 odom_pose_count = 0;
  int timestamp_bytes = 0;
  int pose_bytes = 0;
  Eigen::Matrix4f submap_pose = Eigen::Matrix4f::Identity();
};

NodeStorage readNodeStorage(const std::filesystem::path &path) {
  sqlite3 *database = nullptr;
  check(sqlite3_open_v2(path.c_str(), &database, SQLITE_OPEN_READONLY, nullptr) == SQLITE_OK, "open Node storage test database");
  sqlite3_stmt *statement = nullptr;
  check(sqlite3_prepare_v2(database,
                           "SELECT odom_pose_count,length(odom_timestamps),length(odom_poses),submap_pose FROM Node WHERE id=1;", -1,
                           &statement, nullptr) == SQLITE_OK,
        "prepare Node storage read");
  check(sqlite3_step(statement) == SQLITE_ROW, "read Node storage");
  NodeStorage storage;
  storage.odom_pose_count = sqlite3_column_int64(statement, 0);
  storage.timestamp_bytes = sqlite3_column_int(statement, 1);
  storage.pose_bytes = sqlite3_column_int(statement, 2);
  check(sqlite3_column_bytes(statement, 3) == static_cast<int>(16 * sizeof(float)), "read submap_pose blob length");
  std::memcpy(storage.submap_pose.data(), sqlite3_column_blob(statement, 3), 16 * sizeof(float));
  sqlite3_finalize(statement);
  sqlite3_close_v2(database);
  return storage;
}

}  // namespace

int main() {
  static_assert(sizeof(sapphire::GaussianPoint) <= 112, "GaussianPoint must retain its compact float layout");
  std::cout << "GaussianPoint sizeof=" << sizeof(sapphire::GaussianPoint) << " B, payload_record=73 B\n";
  sapphire::UcmCameraModel camera(1280, 720, 500.0F, 500.0F, 640.0F, 360.0F, 0.6F);
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
  sapphire::SapphireParameters excessive_layer_parameters = parameters;
  excessive_layer_parameters.local_submap.max_layer = 31;
  bool excessive_layer_rejected = false;
  try {
    sapphire::validate_parameters(excessive_layer_parameters);
  } catch (const std::invalid_argument &) {
    excessive_layer_rejected = true;
  }
  check(excessive_layer_rejected, "max_layer above payload limit is rejected");

  const std::vector<int> saved_window_order = sapphire::OctoTree::window_order;
  sapphire::OctoTree::window_order = {0};
  const sapphire::VOXEL_LOC marginal_key(-3, 4, 5, 2);
  sapphire::OctoTree marginal_tree(0, 1, marginal_key);
  marginal_tree.voxel_center[0] = 0.0;
  marginal_tree.voxel_center[1] = 0.0;
  marginal_tree.voxel_center[2] = 0.0;
  marginal_tree.quater_length = 1.0F;
  marginal_tree.isexist = true;
  marginal_tree.sw = new sapphire::SlideWindow(1);
  marginal_tree.pcr_fix.push(Eigen::Vector3d(1000.0, -2000.0, 3000.0));
  marginal_tree.pcr_add.push(Eigen::Vector3d(-4000.0, 5000.0, -6000.0));
  marginal_tree.sw->pcrs_local[0].push(Eigen::Vector3d(1.0, 2.0, 3.0));
  marginal_tree.sw->pcrs_local[0].push(Eigen::Vector3d(3.0, 2.0, 1.0));
  std::vector<sapphire::StateGroup> marginal_states(1);
  sapphire::LidarFactor marginal_factor(1);
  sapphire::GaussianCloud marginalized;
  marginal_tree.margi(1, 1, marginal_states, marginal_factor, marginalized);
  const Eigen::Matrix3d expected_marginal_covariance =
      (Eigen::Matrix3d() << 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0).finished();
  check(marginalized.size() == 1 && marginalized.front().N == 2 &&
            marginalized.front().mean.isApprox(Eigen::Vector3f(2.0F, 2.0F, 2.0F), 1e-5F) &&
            marginalized.front().covariance.isApprox(expected_marginal_covariance.cast<float>(), 1e-5F) &&
            marginalized.front().voxel_key == marginal_key && marginalized.front().radius == 0.0F,
        "OctoTree marginal export contains only the current unregularized slot");
  const sapphire::OctoTree *child = marginal_tree.child(5);
  check(child->voxel_key == sapphire::VOXEL_LOC(-5, 8, 11, 3), "OctoTree child key derives from parent key and octant");
  sapphire::OctoTree overflowing_tree(
      0, 1, sapphire::VOXEL_LOC(std::numeric_limits<int64_t>::max() / 2 + 1, 0, 0, 0));
  bool child_overflow_rejected = false;
  try {
    static_cast<void>(overflowing_tree.child(4));
  } catch (const std::overflow_error &) {
    child_overflow_rejected = true;
  }
  check(child_overflow_rejected, "OctoTree child coordinate overflow is rejected");
  sapphire::OctoTree::window_order = saved_window_order;

  sapphire::SubmapFrameBuffer flush_buffer(1.0, 15.0, 20.0);
  sapphire::GaussianCloud flush_cloud;
  flush_cloud.emplace_back(gaussian(Eigen::Vector3d(2.0, 0.0, 0.0), 2, sapphire::VOXEL_LOC(2, 0, 0, 0)));
  sapphire::StateGroup flush_state;
  sapphire::MargiFrame flush_frame(flush_state, std::move(flush_cloud), 0.5, Eigen::Matrix<double, 6, 1>::Zero(), 0.0);
  check(!flush_buffer.push(std::move(flush_frame)), "partial submap remains buffered before flush");
  std::optional<sapphire::SubmapFrame> flushed_submap = flush_buffer.flush();
  check(flushed_submap && flushed_submap->frame_count() == 1 && flushed_submap->lio().pcd->size() == 1 && flush_buffer.empty(),
        "flush freezes and emits a one-frame partial submap");

  sapphire::SubmapFrameBuffer visual_buffer(1.0, 1.0, 20.0);
  for (int i = 0; i < 3; ++i) {
    sapphire::GaussianCloud cloud;
    cloud.emplace_back(gaussian(Eigen::Vector3d(2.0, 0.0, 0.0), 2, sapphire::VOXEL_LOC(2, 0, 0, 0)));
    sapphire::MargiFrame frame(flush_state, std::move(cloud), 1.0 + i, Eigen::Matrix<double, 6, 1>::Zero(), 10.0 * i);
    check(!visual_buffer.push(std::move(frame), false), "visual policy retains evidence beyond distance threshold");
  }
  auto visual_cut = visual_buffer.flush();
  check(visual_cut && visual_cut->frame_count() == 3 && visual_cut->odom_poses().size() == 3 &&
        visual_cut->id() == 0 && visual_buffer.empty(), "explicit visual/resource cut preserves evidence, odometry and ID ownership");

  sapphire::SubmapFrameBuffer cleared_buffer(1.0, 15.0, 20.0);
  sapphire::GaussianCloud cleared_cloud;
  cleared_cloud.emplace_back(gaussian(Eigen::Vector3d(3.0, 0.0, 0.0), 1, sapphire::VOXEL_LOC(3, 0, 0, 0)));
  sapphire::MargiFrame cleared_frame(flush_state, std::move(cleared_cloud), 0.75, Eigen::Matrix<double, 6, 1>::Zero(), 0.0);
  check(!cleared_buffer.push(std::move(cleared_frame)), "partial submap is buffered before clear");
  cleared_buffer.clear();
  check(cleared_buffer.empty() && !cleared_buffer.flush(), "clear discards a partial submap");

  sapphire::SubmapFrameBuffer empty_boundary_buffer(1.0, 15.0, 20.0);
  sapphire::GaussianCloud first_distant_cloud;
  first_distant_cloud.emplace_back(gaussian(Eigen::Vector3d(21.0, 0.0, 0.0), 1, sapphire::VOXEL_LOC(21, 0, 0, 0)));
  sapphire::MargiFrame first_distant_frame(flush_state, std::move(first_distant_cloud), 1.0, Eigen::Matrix<double, 6, 1>::Zero(), 0.0);
  check(!empty_boundary_buffer.push(std::move(first_distant_frame)), "distant-only submap remains pending before target distance");
  sapphire::GaussianCloud second_distant_cloud;
  second_distant_cloud.emplace_back(gaussian(Eigen::Vector3d(22.0, 0.0, 0.0), 1, sapphire::VOXEL_LOC(22, 0, 0, 0)));
  sapphire::MargiFrame second_distant_frame(flush_state, std::move(second_distant_cloud), 2.0, Eigen::Matrix<double, 6, 1>::Zero(), 15.0);
  check(!empty_boundary_buffer.push(std::move(second_distant_frame)) && empty_boundary_buffer.empty() && !empty_boundary_buffer.flush(),
        "target-distance freeze drops an empty filtered submap");

  sapphire::SubmapFrameBuffer empty_flush_buffer(1.0, 15.0, 20.0);
  sapphire::GaussianCloud flush_distant_cloud;
  flush_distant_cloud.emplace_back(gaussian(Eigen::Vector3d(25.0, 0.0, 0.0), 1, sapphire::VOXEL_LOC(25, 0, 0, 0)));
  sapphire::MargiFrame flush_distant_frame(flush_state, std::move(flush_distant_cloud), 3.0, Eigen::Matrix<double, 6, 1>::Zero(), 0.0);
  check(!empty_flush_buffer.push(std::move(flush_distant_frame)), "distant-only submap remains pending before flush");
  check(!empty_flush_buffer.flush() && empty_flush_buffer.empty(), "flush drops an empty filtered submap");

  sapphire::SubmapFrameBuffer submap_buffer(1.0, 15.0, 20.0);
  submap_buffer.push_visual(sapphire::VisualFrame(1.0));
  submap_buffer.push_visual(sapphire::VisualFrame(2.0));
  sapphire::StateGroup first_state;
  sapphire::GaussianCloud first_cloud;
  first_cloud.emplace_back(gaussian(Eigen::Vector3d(1.0, 0.0, 0.0), 2, sapphire::VOXEL_LOC(1, 0, 0, 0)));
  first_cloud.emplace_back(gaussian(Eigen::Vector3d(1.0, 0.0, 0.0), 3, sapphire::VOXEL_LOC(2, 0, 0, 1)));
  first_cloud.emplace_back(gaussian(Eigen::Vector3d(21.0, 0.0, 0.0), 1, sapphire::VOXEL_LOC(21, 0, 0, 0)));
  const sapphire::GaussianPoint *first_cloud_data = first_cloud.data();
  const Eigen::Matrix<double, 6, 1> variance = Eigen::Matrix<double, 6, 1>::Zero();
  sapphire::MargiFrame first_frame(first_state, std::move(first_cloud), 1.0, variance, 0.0);
  check(first_frame.gaussians.data() == first_cloud_data, "marginal frame takes Gaussian cloud ownership");
  check(!submap_buffer.push(std::move(first_frame)), "submap remains open before target distance");

  sapphire::StateGroup second_state;
  second_state.p.x() = 15.0;
  second_state.R = Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  sapphire::GaussianCloud second_cloud;
  second_cloud.emplace_back(gaussian(Eigen::Vector3d(1.0, 0.0, 0.0), 4, sapphire::VOXEL_LOC(1, 0, 0, 0)));
  second_cloud.emplace_back(gaussian(Eigen::Vector3d(16.0, 0.0, 0.0), 4, sapphire::VOXEL_LOC(16, 0, 0, 0)));
  sapphire::MargiFrame second_frame(second_state, std::move(second_cloud), 2.0, variance, 15.0);
  std::optional<sapphire::SubmapFrame> completed_submap = submap_buffer.push(std::move(second_frame));
  check(completed_submap && completed_submap->frame_count() == 2 && completed_submap->lio().pcd->size() == 3,
        "submap directly fuses marginal frames and crops distant points");
  check(completed_submap->odom_poses().size() == 2 && completed_submap->odom_poses()[0].timestamp == 1.0 &&
            completed_submap->odom_poses()[1].timestamp == 2.0 &&
            completed_submap->odom_poses()[0].T_odom_base.matrix().isApprox(Eigen::Isometry3d::Identity().matrix()) &&
            completed_submap->odom_poses()[1].T_odom_base.linear().isApprox(second_state.R) &&
            completed_submap->odom_poses()[1].T_odom_base.translation().isApprox(second_state.p),
        "submap retains each raw marginal-frame odometry pose before navigation smoothing");
  bool has_parent_level = false;
  bool has_child_level = false;
  bool merged_parent_key = false;
  for (const sapphire::GaussianPoint &point : *completed_submap->lio().pcd) {
    has_parent_level = has_parent_level || point.voxel_key.level == 0;
    has_child_level = has_child_level || point.voxel_key.level == 1;
    if (point.voxel_key == sapphire::VOXEL_LOC(1, 0, 0, 0)) {
      merged_parent_key = point.N == 6 && point.mean.isApprox(Eigen::Vector3f(1.0F, 0.0F, 0.0F), 1e-5F);
    }
  }
  check(has_parent_level && has_child_level, "parent and child voxel levels coexist");
  check(merged_parent_key, "submap merges only the identical frozen voxel key");
  check(completed_submap->visual_frames().size() == 2, "submap retains every visual frame");
  check(completed_submap->pyramid_voxels().valid(), "submap builds pyramid voxels with its final cloud");
  check(completed_submap->bounds().minimum.isApprox(Eigen::Vector3f(0.0F, -1.0F, -1.0F), 1e-5F) &&
            completed_submap->bounds().maximum.isApprox(Eigen::Vector3f(17.0F, 1.0F, 1.0F), 1e-5F),
        "submap AABB includes Gaussian radius");

  sapphire::GaussianCloud adapter_cloud;
  adapter_cloud.emplace_back(gaussian(Eigen::Vector3d(1.2, 0.2, 0.1), 1, sapphire::VOXEL_LOC(1, 0, 0)));
  adapter_cloud.emplace_back(gaussian(Eigen::Vector3d(2.2, 0.2, 1.0), 1, sapphire::VOXEL_LOC(2, 0, 1)));
  const sapphire::AABB adapter_bounds(adapter_cloud);
  check(adapter_bounds.minimum.isApprox(Eigen::Vector3f(0.2F, -0.8F, -0.9F), 1e-5F) &&
            adapter_bounds.maximum.isApprox(Eigen::Vector3f(3.2F, 1.2F, 2.0F), 1e-5F),
        "AABB is derived directly from Gaussian means and radii");

  sapphire::NaviMapParameters local_grid_parameters;
  local_grid_parameters.resolution = 1.0;
  local_grid_parameters.min_range = 0.0;
  local_grid_parameters.usable_range = 10.0;
  local_grid_parameters.ground_margin = 0.5;
  local_grid_parameters.h_clearance = 2.0;
  sapphire::LocalGrid adapter_grid;
  sapphire::LocalGridMaker(local_grid_parameters).createLocalMap(adapter_cloud, Eigen::Isometry3f::Identity(), adapter_grid, 0.0f);
  const auto contains_grid_point = [](const sapphire::GridPoints &points, const Eigen::Vector2f &expected) {
    return std::any_of(points.begin(), points.end(),
                       [&](const Eigen::Vector2f &point) { return point.isApprox(expected, 1e-5F); });
  };
  check(adapter_grid.groundCells.size() == 1 && adapter_grid.obstacleCells.size() == 1 &&
            contains_grid_point(adapter_grid.groundCells, Eigen::Vector2f(1.2F, 0.2F)) &&
            contains_grid_point(adapter_grid.obstacleCells, Eigen::Vector2f(2.2F, 0.2F)),
        "LocalGrid consumes Gaussian means without an XYZ cloud adapter");

  sapphire::GaussianPoint merged =
      gaussian(Eigen::Vector3d::Zero(), 2, sapphire::VOXEL_LOC(5, 6, 7, 1), Eigen::Matrix3d::Identity(), true);
  const sapphire::GaussianPoint merge_source =
      gaussian(Eigen::Vector3d(3.0, 0.0, 0.0), 1, sapphire::VOXEL_LOC(5, 6, 7, 1), 4.0 * Eigen::Matrix3d::Identity());
  const sapphire::GaussianPoint added = merged + merge_source;
  merged += merge_source;
  const Eigen::Matrix3f expected_merged_covariance = Eigen::Vector3f(4.0F, 2.0F, 2.0F).asDiagonal();
  check(merged.N == 3 && !merged.is_plane && merged.mean.isApprox(Eigen::Vector3f(1.0F, 0.0F, 0.0F), 1e-5F) &&
            merged.covariance.isApprox(expected_merged_covariance, 1e-5F) && added.N == merged.N &&
            !added.is_plane && added.mean.isApprox(merged.mean, 1e-5F) && added.covariance.isApprox(merged.covariance, 1e-5F),
        "same-key Gaussian merge preserves exact moments and conservatively merges plane state");
  const Eigen::Matrix3d transform_rotation = Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d transform_translation(1000.25, -2000.5, 3.75);
  const sapphire::GaussianPoint transformed = merged.transformed(transform_rotation, transform_translation);
  check(transformed.mean.cast<double>().isApprox(transform_rotation * merged.mean.cast<double>() + transform_translation, 1e-4) &&
            transformed.covariance.cast<double>().isApprox(
                transform_rotation * merged.covariance.cast<double>() * transform_rotation.transpose(), 1e-5) &&
            transformed.is_plane == merged.is_plane,
        "Gaussian transform uses double intermediates and preserves plane state");
  sapphire::GaussianPoint count_limit =
      gaussian(Eigen::Vector3d::Zero(), std::numeric_limits<int>::max(), sapphire::VOXEL_LOC(8, 9, 10, 0));
  bool count_overflow_rejected = false;
  try {
    count_limit += gaussian(Eigen::Vector3d::Zero(), 1, sapphire::VOXEL_LOC(8, 9, 10, 0));
  } catch (const std::overflow_error &) {
    count_overflow_rejected = true;
  }
  check(count_overflow_rejected, "Gaussian merge rejects count overflow");
  sapphire::GaussianPoint negative_count = gaussian(Eigen::Vector3d::Zero(), 1, sapphire::VOXEL_LOC(8, 9, 11, 0));
  negative_count.N = -1;
  bool negative_count_rejected = false;
  try {
    negative_count += gaussian(Eigen::Vector3d::Zero(), 1, sapphire::VOXEL_LOC(8, 9, 11, 0));
  } catch (const std::invalid_argument &) {
    negative_count_rejected = true;
  }
  check(negative_count_rejected, "Gaussian merge rejects negative counts");
  sapphire::GaussianPoint nonfinite_moment = gaussian(Eigen::Vector3d::Zero(), 1, sapphire::VOXEL_LOC(8, 9, 12, 0));
  nonfinite_moment.mean.x() = std::numeric_limits<float>::infinity();
  bool nonfinite_moment_rejected = false;
  try {
    nonfinite_moment += gaussian(Eigen::Vector3d::Zero(), 1, sapphire::VOXEL_LOC(8, 9, 12, 0));
  } catch (const std::invalid_argument &) {
    nonfinite_moment_rejected = true;
  }
  check(nonfinite_moment_rejected, "Gaussian merge rejects non-finite moments");
  check(completed_submap->navigation().samples.empty() && !completed_submap->navigation().spline.valid(),
        "online submap does not duplicate odometry as navigation samples or fit a spline");
  check(completed_submap->odom_poses()[1].T_odom_base.linear().isApprox(second_state.R) &&
            completed_submap->odom_poses()[1].T_odom_base.translation().isApprox(second_state.p),
        "raw odometry remains available for future on-demand path construction");

  const std::filesystem::path database_path =
      std::filesystem::temp_directory_path() /
      ("sapphire-core-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".db");
  sapphire::LioFrame lio;
  auto submap_cloud = std::make_shared<sapphire::GaussianCloud>();
  submap_cloud->emplace_back(gaussian(Eigen::Vector3d(1.0, 2.0, 3.0), 1, sapphire::VOXEL_LOC(1, 2, 3),
                                      Eigen::Matrix3d::Identity() / 9.0, true));
  submap_cloud->emplace_back(gaussian(Eigen::Vector3d(2.0, 3.0, 3.0), 1, sapphire::VOXEL_LOC(2, 3, 3)));
  lio.pcd = submap_cloud;
  lio.timestamp = 1.0;
  lio.T_odom_base.translation().x() = 2.0;
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
  voxelmaps.create_voxelmaps(lio.pcd->size(),
                             [&lio](std::size_t index) { return (*lio.pcd)[index].mean.cast<float>(); });
  sapphire::NavigationPath navigation;
  navigation.samples.push_back({1.00, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F});
  navigation.samples.push_back({1.01, 1.0F, 0.2F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 1.0F});
  navigation.samples.push_back({1.02, 2.0F, 0.4F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 2.0F});
  navigation.samples.push_back({1.03, 3.0F, 0.3F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 3.0F});
  navigation.samples.push_back({1.04, 4.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 4.0F});
  navigation.spline = sapphire::fit_navigation_spline(navigation.samples);
  check(navigation.spline.valid(), "manual navigation spline fit");
  sapphire::OdomPoses odom_poses;
  odom_poses.push_back({1.0, lio.T_odom_base});
  Eigen::Isometry3d second_odom_pose = lio.T_odom_base;
  second_odom_pose.translation().y() = 1.5;
  odom_poses.push_back({1.5, second_odom_pose});
  sapphire::SubmapFrame submap(0, std::move(lio), 0, 1, 2, 0.0, 15.0, voxelmaps.release_data(), std::move(odom_poses),
                               std::move(navigation),
                               std::move(visual_frames));
  cv::Mat retrieval_descriptor = (cv::Mat_<float>(1, 3) << 0.1F, 0.2F, 0.3F);
  submap.attach_tag(sapphire::MetaTag(sapphire::MetaTagType::kFloat, std::move(retrieval_descriptor)));
  sapphire::LocalGrid local_grid(0.1F);
  {
    sapphire::Memory memory(database_path.string());
    memory.saveSubmap(submap, local_grid);
    const NodeStorage initial_storage = readNodeStorage(database_path);
    check(initial_storage.odom_pose_count == 2 && initial_storage.timestamp_bytes == 16 && initial_storage.pose_bytes == 112,
          "Node stores fixed-field raw odometry blobs for two frames");
    check(initial_storage.submap_pose.isApprox(submap.lio().T_odom_base.matrix().cast<float>()),
          "Node submap_pose starts at the first-frame odometry anchor");
    check(memory.recallSpatial(0, submap.lio().T_odom_base.cast<float>()).size() == 1, "spatial index receives saved submap bounds");
    const std::vector<sapphire::RecallMatch> initial_matches = memory.recall(0, Eigen::Isometry3f::Identity(), 1, 10.0F);
    check(initial_matches.size() == 1 && initial_matches.front().submap_id == 0, "mandatory pose index");
    Eigen::Isometry3f submap_pose = Eigen::Isometry3f::Identity();
    submap_pose.translation().x() = 5.0F;
    memory.saveSubmapPose(1, submap_pose);
    check(readNodeStorage(database_path).submap_pose.isApprox(submap_pose.matrix()), "saveSubmapPose updates Node submap_pose");
    const std::vector<sapphire::SpatialMatch> updated_spatial = memory.recallSpatial(0, submap_pose);
    check(updated_spatial.size() == 1 && updated_spatial.front().map_T_submap.translation().x() == 5.0F,
          "current submap pose updates R-tree entry");
    const std::vector<sapphire::RecallMatch> updated_matches = memory.recall(0, submap_pose, 1, 0.1F);
    check(updated_matches.size() == 1 && updated_matches.front().pose.translation().x() == 5.0F, "current submap pose index");
    const sapphire::GaussianCloudPtr loaded_cloud = memory.loadCloud(0);
    const sapphire::GaussianCloudPtr cached_cloud = memory.loadCloud(0);
    check(loaded_cloud.get() == cached_cloud.get(), "active Gaussian cloud load reuses cached storage");
    check(memory.loadPyramidVoxel(0).valid(), "cold pyramid voxel load");
    const std::vector<sapphire::VisualFrame> loaded_visuals = memory.loadVisualFrames(0);
    check(loaded_visuals.size() == 2 && loaded_visuals.front().points().size() == 1, "cold visual load");
    const sapphire::NavigationPath loaded_navigation = memory.loadNavigation(0);
    check(loaded_navigation.samples.size() == 5 && loaded_navigation.spline.valid() &&
              loaded_navigation.samples[2].timestamp == 1.02 && loaded_navigation.samples[2].distance == 2.0F &&
              loaded_navigation.samples[2].qw == 1.0F &&
              loaded_navigation.spline.evaluate(0.5F).isApprox(submap.navigation().spline.evaluate(0.5F), 1e-5F),
          "cold navigation samples and cubic B-spline load");
  }
  {
    sapphire::MapDatabase database(database_path.string(), "resume");
    bool found = false;
    database.visitSpatialRecords([&](std::uint64_t id, const sapphire::AABB &bounds, const Eigen::Isometry3f &pose) {
      check(id == 0 && bounds.valid() && pose.translation().x() == 5.0F, "read-only spatial/pose records preserve committed values");
      found = true;
    });
    check(found, "persisted spatial record exists without reconstructing an index");
    const sapphire::GaussianCloudPtr loaded_cloud = database.loadCloud(1);
    check(loaded_cloud->size() == submap.lio().pcd->size(), "cold Gaussian cloud load");
    for (std::size_t index = 0; index < loaded_cloud->size(); ++index) {
      const sapphire::GaussianPoint &actual = (*loaded_cloud)[index];
      const sapphire::GaussianPoint &expected = (*submap.lio().pcd)[index];
      check(actual.voxel_key == expected.voxel_key && actual.N == expected.N && actual.mean == expected.mean &&
                actual.covariance == expected.covariance && actual.radius == expected.radius && actual.is_plane == expected.is_plane,
            "Gaussian storage field round trip");
    }
  }
  const std::vector<std::uint8_t> original_payload = readGaussianPayload(database_path);
  check(original_payload.size() == 16U + submap.lio().pcd->size() * 73U,
        "Gaussian payload stores exactly 73 bytes per point");
  std::vector<std::uint8_t> corrupted_payload = original_payload;
  corrupted_payload[0] ^= 0xffU;
  writeGaussianPayload(database_path, corrupted_payload);
  check(cloudLoadRejected(database_path), "corrupted Gaussian payload magic is rejected");
  corrupted_payload = original_payload;
  corrupted_payload.pop_back();
  writeGaussianPayload(database_path, corrupted_payload);
  check(cloudLoadRejected(database_path), "truncated Gaussian payload is rejected");
  corrupted_payload = original_payload;
  corrupted_payload[16U + 72U] = 2U;
  writeGaussianPayload(database_path, corrupted_payload);
  check(cloudLoadRejected(database_path), "invalid Gaussian is_plane byte is rejected");
  writeGaussianPayload(database_path, original_payload);

  check(!std::filesystem::exists(database_path.string() + "-wal") &&
            !std::filesystem::exists(database_path.string() + "-shm"),
        "closing the map database removes SQLite WAL sidecar files");
  std::filesystem::remove(database_path);

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
