#include <sapphire/backend/pose_graph.hpp>

#include <pcl/common/transforms.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {

void expect(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void addPoint(
    sapphire::PointCloud& cloud, float x, float y, float z)
{
    sapphire::Point point{};
    point.x = x;
    point.y = y;
    point.z = z;
    point.data[3] = 1.0f;
    cloud.push_back(point);
}

sapphire::PointCloudPtr makeLocalCloud() {
    auto cloud = std::make_shared<sapphire::PointCloud>();
    for (int x = 0; x < 5; ++x) {
        for (int y = 0; y < 5; ++y) {
            addPoint(
                *cloud,
                static_cast<float>(x) * 0.2f,
                static_cast<float>(y) * 0.2f,
                static_cast<float>((x + y) % 3) * 0.1f);
        }
    }
    return cloud;
}

/// Three orthogonal planes. Spacing stays above the test voxel size so the
/// cloud still clears the 64-point GICP gate after downsampling.
sapphire::PointCloudPtr makeCornerCloud() {
    auto cloud = std::make_shared<sapphire::PointCloud>();
    for (int iy = 0; iy <= 10; ++iy) {
        for (int iz = 0; iz <= 8; ++iz) {
            addPoint(
                *cloud,
                0.0f,
                static_cast<float>(iy) * 0.2f,
                static_cast<float>(iz) * 0.2f);
        }
    }
    for (int ix = 1; ix <= 10; ++ix) {
        for (int iz = 0; iz <= 8; ++iz) {
            addPoint(
                *cloud,
                static_cast<float>(ix) * 0.2f,
                0.0f,
                static_cast<float>(iz) * 0.2f);
        }
    }
    for (int ix = 1; ix <= 10; ++ix) {
        for (int iy = 1; iy <= 10; ++iy) {
            addPoint(
                *cloud,
                static_cast<float>(ix) * 0.2f,
                static_cast<float>(iy) * 0.2f,
                0.0f);
        }
    }
    return cloud;
}

sapphire::PointCloudPtr transformCloud(
    const sapphire::PointCloudConstPtr& cloud,
    const sapphire::Isometry3d& pose)
{
    auto transformed = std::make_shared<sapphire::PointCloud>();
    pcl::transformPointCloud(*cloud, *transformed, pose.matrix());
    return transformed;
}

void testOdometryOnlyGraphPreservesFrontendFrame() {
    sapphire::Config::Pgo config;
    config.enabled = true;
    config.keyframe_distance = 0.25;
    config.update_period_sec = 0.01;
    config.loop_min_time_separation = 1000.0;

    sapphire::PoseGraphBackend backend(config);
    const auto local = makeLocalCloud();
    for (int i = 0; i < 3; ++i) {
        sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
        pose.translation().x() = static_cast<double>(i);
        backend.addFrame(
            transformCloud(local, pose),
            pose,
            static_cast<double>(i));
    }

    for (int attempt = 0;
         attempt < 100 && backend.stats().optimized_poses < 3;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const sapphire::PoseGraphStats stats = backend.stats();
    expect(stats.keyframes == 3, "all separated frames must become keyframes");
    expect(stats.optimized_poses == 3, "all keyframes must enter ISAM2");
    expect(stats.loop_edges == 0, "test trajectory must not create loops");
    expect(
        backend.T_map_odom().matrix().isApprox(
            Eigen::Matrix4d::Identity(), 1e-9),
        "odometry-only graph must preserve the frontend frame");
}

void testLoopClosureAddsConstraint() {
    sapphire::Config::Pgo config;
    config.enabled = true;
    config.keyframe_distance = 0.05;
    config.update_period_sec = 0.02;
    config.loop_min_time_separation = 3.5;
    config.loop_min_travel_distance = 1.0;
    config.loop_search_radius = 2.0;
    config.loop_search_stride = 1;
    config.target_frame_count = 0;
    config.source_voxel_size = 0.05;
    config.target_voxel_size = 0.05;

    sapphire::PoseGraphBackend backend(config);
    const auto local = makeCornerCloud();
    // The loop target is not at the origin. The final scan observes that
    // location, but its odometry estimate is 0.5 m farther along x. The
    // offset is inside the 2 m GICP correspondence gate and the 2 m search
    // radius, and after optimization it stays inside the 3-sigma residual
    // gate. Using the registration pose itself as the between measurement
    // would push the query away from the target instead of toward it.
    const double positions[] = {10.0, 12.0, 14.0, 16.0, 10.5};
    for (int i = 0; i < 5; ++i) {
        sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
        pose.translation().x() = positions[i];
        backend.addFrame(
            transformCloud(local, pose),
            pose,
            static_cast<double>(i));
    }

    for (int attempt = 0;
         attempt < 200 && backend.stats().loop_edges == 0;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const sapphire::PoseGraphStats stats = backend.stats();
    expect(stats.optimized_poses == 5, "loop test must optimize every pose");
    expect(stats.loop_candidates >= 1,
           "returning trajectory must produce a loop candidate");
    expect(stats.loop_registration_failures == 0,
           "drifted loop candidate must pass GICP registration");
    expect(stats.loop_edges >= 1, "returning trajectory must add a loop edge");
    expect(stats.last_loop_fitness >= 0.0,
           "accepted loop must report its GICP fitness");
    expect(stats.last_loop_fitness <= config.lidar_max_fitness_m2,
           "accepted loop fitness must stay inside the metric gate");
    expect(stats.correction_translation > 1e-5,
           "loop closure must produce a non-identity global correction");

    expect(backend.snapshot().optimized_poses.empty(),
           "visualization poses must not be built without a request");
    backend.requestSnapshot();
    for (int attempt = 0;
         attempt < 100 && backend.snapshot().optimized_poses.size() < 5;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const sapphire::PoseGraphSnapshot snapshot = backend.snapshot();
    expect(snapshot.optimized_poses.size() == 5,
           "requested visualization snapshot must be built asynchronously");
    expect(snapshot.loop_edges.size() == 1,
           "visualization snapshot must contain the loop endpoints");
    const sapphire::PoseGraphEdge& edge = snapshot.loop_edges.front();
    expect(edge.source == 4 && edge.target == 0,
           "stored loop edge must run from the query to the historical frame");

    const double target_x = snapshot.optimized_poses.front().translation().x();
    const double query_x = snapshot.optimized_poses.back().translation().x();
    constexpr double odom_x = 10.5;
    expect(std::abs(target_x - 10.0) < 1e-3,
           "root prior must keep the historical keyframe in place");
    expect(query_x < odom_x - 1e-4,
           "loop factor must pull the query toward the historical pose");
    expect(query_x > target_x,
           "translation floor must leave the query short of the target");

    backend.requestGlobalMap();
    for (int attempt = 0;
         attempt < 100 && !backend.latestGlobalMap();
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto map = backend.latestGlobalMap();
    expect(map && !map->empty(),
           "requested sparse global map must be built asynchronously");
}

/// A long odometry chain whose only accepted loop is back to the root.
/// Intermediate stamps sit inside the time gate, so the search cannot
/// latch onto a neighbor of the return pose. The return offset and the
/// translation floor are chosen so the optimized residual stays inside the
/// 3-sigma gate, while the loop is still stiff enough to move a keyframe
/// that is not an endpoint. Production defaults are unchanged.
void testLoopRelinearizationMovesDistantKeyframe() {
    sapphire::Config::Pgo config;
    config.enabled = true;
    config.keyframe_distance = 0.05;
    config.update_period_sec = 0.02;
    config.loop_min_time_separation = 1000.0;
    config.loop_min_travel_distance = 1.0;
    config.loop_search_radius = 2.0;
    config.loop_search_stride = 1;
    config.target_frame_count = 0;
    config.source_voxel_size = 0.05;
    config.target_voxel_size = 0.05;
    config.lidar_translation_sigma_m = 0.02;

    constexpr int kOutbound = 40;
    constexpr double kStep = 0.5;
    constexpr double kReturnX = 0.6;
    constexpr int kDistant = 8;

    sapphire::PoseGraphBackend backend(config);
    const auto local = makeCornerCloud();
    {
        sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
        backend.addFrame(transformCloud(local, pose), pose, 0.0);
    }
    // Every outbound frame shares a stamp equal to the time gate, so none
    // of them is old enough to close against the root or against each other.
    // The return is one second later and is the only query that qualifies.
    for (int i = 1; i <= kOutbound; ++i) {
        sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
        pose.translation().x() = static_cast<double>(i) * kStep;
        backend.addFrame(
            transformCloud(local, pose),
            pose,
            config.loop_min_time_separation);
    }
    {
        sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
        pose.translation().x() = kReturnX;
        backend.addFrame(
            transformCloud(local, pose),
            pose,
            config.loop_min_time_separation + 1.0);
    }

    const int pose_count = kOutbound + 2;
    for (int attempt = 0;
         attempt < 400 && backend.stats().loop_edges == 0;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    backend.requestSnapshot();
    for (int attempt = 0;
         attempt < 200
             && static_cast<int>(backend.snapshot().optimized_poses.size())
                 < pose_count;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const sapphire::PoseGraphSnapshot snapshot = backend.snapshot();
    expect(static_cast<int>(snapshot.optimized_poses.size()) == pose_count,
           "long chain must publish an optimized pose for every keyframe");
    expect(snapshot.loop_edges.size() == 1,
           "one query must add one loop edge");
    const sapphire::PoseGraphEdge& edge = snapshot.loop_edges.front();
    expect(edge.source == static_cast<size_t>(pose_count - 1)
               && edge.target == 0,
           "the only loop must join the return frame to the root");

    const double root_x = snapshot.optimized_poses.front().translation().x();
    const double distant_x =
        snapshot.optimized_poses[kDistant].translation().x();
    const double distant_odom = static_cast<double>(kDistant) * kStep;
    const double query_x = snapshot.optimized_poses.back().translation().x();
    expect(std::abs(root_x) < 1e-3,
           "root prior must keep the first keyframe in place");
    expect(query_x < kReturnX - 1e-2,
           "loop factor must pull the return keyframe toward the root");
    expect(distant_x < distant_odom - 0.02,
           "relinearization must move a keyframe that is not a loop endpoint");
}

/// Registration succeeds, but the loop disagrees with a short odometry
/// chain by many standard deviations. The trial must be discarded: no loop
/// edge, and the committed poses stay on the frontend trajectory.
/// A second return is added only after the first loop has been committed.
/// The residual count must include that existing edge, or the next good
/// loop is dropped as an incomplete constraint set.
void testSecondLoopCommitsAfterFirst() {
    sapphire::Config::Pgo config;
    config.enabled = true;
    config.keyframe_distance = 0.05;
    config.update_period_sec = 0.02;
    config.loop_min_time_separation = 3.5;
    config.loop_min_travel_distance = 1.0;
    config.loop_search_radius = 2.0;
    config.loop_search_stride = 1;
    config.target_frame_count = 0;
    config.source_voxel_size = 0.05;
    config.target_voxel_size = 0.05;

    sapphire::PoseGraphBackend backend(config);
    const auto local = makeCornerCloud();
    const double first_positions[] = {10.0, 12.0, 14.0, 16.0, 10.5};
    for (int i = 0; i < 5; ++i) {
        sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
        pose.translation().x() = first_positions[i];
        backend.addFrame(
            transformCloud(local, pose),
            pose,
            static_cast<double>(i));
    }

    for (int attempt = 0;
         attempt < 200 && backend.stats().loop_edges == 0;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    expect(backend.stats().loop_edges == 1,
           "first return must commit one loop before the next query");

    {
        sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
        pose.translation().x() = 10.4;
        backend.addFrame(transformCloud(local, pose), pose, 5.0);
    }
    for (int attempt = 0;
         attempt < 200 && backend.stats().loop_edges < 2;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const sapphire::PoseGraphStats stats = backend.stats();
    expect(stats.loop_edges == 2,
           "a later in-gate loop must commit on top of the existing edge");
    expect(stats.loop_residual_rejections == 0,
           "the second loop must not be dropped as an incomplete graph");
}

void testInconsistentLoopLeavesCommittedGraph() {
    sapphire::Config::Pgo config;
    config.enabled = true;
    config.keyframe_distance = 0.05;
    config.update_period_sec = 0.02;
    config.loop_min_time_separation = 1.5;
    config.loop_min_travel_distance = 1.0;
    config.loop_search_radius = 2.0;
    config.loop_search_stride = 1;
    config.target_frame_count = 0;
    config.source_voxel_size = 0.05;
    config.target_voxel_size = 0.05;
    config.lidar_translation_sigma_m = 0.05;

    sapphire::PoseGraphBackend backend(config);
    const auto local = makeCornerCloud();
    const double positions[] = {0.0, 3.0, 1.5};
    for (int i = 0; i < 3; ++i) {
        sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
        pose.translation().x() = positions[i];
        backend.addFrame(
            transformCloud(local, pose),
            pose,
            static_cast<double>(i));
    }

    for (int attempt = 0;
         attempt < 400
             && backend.stats().loop_residual_rejections == 0
             && backend.stats().loop_edges == 0;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const sapphire::PoseGraphStats stats = backend.stats();
    expect(stats.optimized_poses == 3, "odometry must still enter ISAM2");
    expect(stats.loop_candidates >= 1,
           "inconsistent return must produce a loop candidate");
    expect(stats.loop_registration_failures == 0,
           "GICP itself must accept the drifted corner cloud");
    expect(stats.loop_residual_rejections >= 1,
           "a loop past 3 sigma must be discarded");
    expect(stats.loop_edges == 0,
           "a rejected trial must not add a loop edge");
    expect(
        backend.T_map_odom().matrix().isApprox(
            Eigen::Matrix4d::Identity(), 1e-3),
        "rejected loop must leave the published correction unchanged");

    backend.requestSnapshot();
    for (int attempt = 0;
         attempt < 200 && backend.snapshot().optimized_poses.size() < 3;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const sapphire::PoseGraphSnapshot snapshot = backend.snapshot();
    expect(snapshot.optimized_poses.size() == 3,
           "snapshot must still contain the odometry poses");
    expect(snapshot.loop_edges.empty(),
           "snapshot must not draw a rejected loop");
    for (int i = 0; i < 3; ++i) {
        expect(
            std::abs(snapshot.optimized_poses[i].translation().x()
                     - positions[i])
                < 1e-3,
            "rejected loop must not move a committed keyframe");
    }
}

void testSparseCloudRejectsLoop() {
    sapphire::Config::Pgo config;
    config.enabled = true;
    config.keyframe_distance = 0.05;
    config.update_period_sec = 0.02;
    config.loop_min_time_separation = 3.5;
    config.loop_min_travel_distance = 1.0;
    config.loop_search_radius = 2.0;
    config.loop_search_stride = 1;
    config.target_frame_count = 0;
    config.source_voxel_size = 0.05;
    config.target_voxel_size = 0.05;

    sapphire::PoseGraphBackend backend(config);
    const auto local = makeLocalCloud();
    const double positions[] = {0.0, 2.0, 4.0, 6.0, 1.5};
    for (int i = 0; i < 5; ++i) {
        sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
        pose.translation().x() = positions[i];
        backend.addFrame(
            transformCloud(local, pose),
            pose,
            static_cast<double>(i));
    }

    for (int attempt = 0;
         attempt < 200
             && backend.stats().loop_registration_failures == 0;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const sapphire::PoseGraphStats stats = backend.stats();
    expect(stats.loop_candidates >= 1,
           "sparse cloud must still produce a loop candidate");
    expect(stats.loop_registration_failures >= 1,
           "cloud below the inlier gate must be rejected");
    expect(stats.loop_edges == 0,
           "rejected registration must not add a loop edge");
}

}  // namespace

int main() {
    testOdometryOnlyGraphPreservesFrontendFrame();
    testLoopClosureAddsConstraint();
    testLoopRelinearizationMovesDistantKeyframe();
    testSecondLoopCommitsAfterFirst();
    testInconsistentLoopLeavesCommittedGraph();
    testSparseCloudRejectsLoop();
    std::cout << "All pose graph tests passed\n";
    return EXIT_SUCCESS;
}
