#include <sapphire/backend/pose_graph.hpp>

#include <pcl/common/transforms.h>

#include <chrono>
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

sapphire::PointCloudPtr makeLocalCloud() {
    auto cloud = std::make_shared<sapphire::PointCloud>();
    for (int x = 0; x < 5; ++x) {
        for (int y = 0; y < 5; ++y) {
            sapphire::Point point{};
            point.x = static_cast<float>(x) * 0.2f;
            point.y = static_cast<float>(y) * 0.2f;
            point.z = static_cast<float>((x + y) % 3) * 0.1f;
            point.data[3] = 1.0f;
            cloud->push_back(point);
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
    config.fitness_threshold = 0.1;

    sapphire::PoseGraphBackend backend(config);
    const auto local = makeLocalCloud();
    // The final scan observes the first location, but its odometry estimate is
    // 1.5 m away. This exceeds PCL ICP's default correspondence distance and
    // guards the DLIO-compatible loop correspondence configuration.
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
         attempt < 200 && backend.stats().loop_edges == 0;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const sapphire::PoseGraphStats stats = backend.stats();
    expect(stats.optimized_poses == 5, "loop test must optimize every pose");
    expect(stats.loop_candidates >= 1,
           "returning trajectory must produce a loop candidate");
    expect(stats.loop_registration_failures == 0,
           "drifted loop candidate must pass ICP registration");
    expect(stats.loop_edges >= 1, "returning trajectory must add a loop edge");
    expect(stats.last_loop_fitness >= 0.0,
           "accepted loop must report its ICP fitness");
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
    expect(!snapshot.loop_edges.empty(),
           "visualization snapshot must contain loop endpoints");

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

}  // namespace

int main() {
    testOdometryOnlyGraphPreservesFrontendFrame();
    testLoopClosureAddsConstraint();
    std::cout << "All pose graph tests passed\n";
    return EXIT_SUCCESS;
}
