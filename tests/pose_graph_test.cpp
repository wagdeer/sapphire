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

sapphire::OdometryResult makeOdomSample(
    double x, double stamp, double velocity_x)
{
    sapphire::OdometryResult sample;
    sample.T_world_lidar = sapphire::Isometry3d::Identity();
    sample.T_world_lidar.translation().x() = x;
    sample.v_world = Eigen::Vector3d(velocity_x, 0.0, 0.0);
    sample.stamp = stamp;
    sample.converged = true;
    return sample;
}

const sapphire::MapTrajectorySample* findSample(
    const sapphire::MapTrajectory& trajectory, double stamp)
{
    for (const sapphire::MapTrajectorySample& sample : trajectory.samples) {
        if (std::abs(sample.stamp - stamp) < 1e-9) {
            return &sample;
        }
    }
    return nullptr;
}

/// History is replayed per owning keyframe. A pose between two keyframes
/// keeps the earlier keyframe's odometry shape. The chain is long enough
/// that the return keyframe moves by centimeters while the root prior does
/// not, so an old sample must not ride the latest rigid T_map_odom. The
/// return sample is the latest keyframe, so its replay matches T_map_odom.
void testHistoricalTrajectoryFollowsOwningKeyframe() {
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
    constexpr int kAnchor = 8;
    constexpr double kRootMidX = 0.2;
    constexpr double kRootMidStamp = 0.5;
    constexpr double kAnchorMidStamp = 1000.5;
    constexpr double kVelocityX = 0.8;
    const double anchor_mid_x = static_cast<double>(kAnchor) * kStep + 0.15;

    sapphire::PoseGraphBackend backend(config);
    const auto local = makeCornerCloud();
    std::vector<double> keyframe_x;
    keyframe_x.reserve(static_cast<size_t>(kOutbound) + 2);
    {
        sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
        backend.addFrame(transformCloud(local, pose), pose, 0.0);
        backend.recordOdometrySample(makeOdomSample(0.0, 0.0, kVelocityX));
        backend.recordOdometrySample(
            makeOdomSample(kRootMidX, kRootMidStamp, kVelocityX));
        keyframe_x.push_back(0.0);
    }
    for (int i = 1; i <= kOutbound; ++i) {
        sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
        pose.translation().x() = static_cast<double>(i) * kStep;
        backend.addFrame(
            transformCloud(local, pose),
            pose,
            config.loop_min_time_separation);
        keyframe_x.push_back(pose.translation().x());
        if (i == kAnchor) {
            backend.recordOdometrySample(
                makeOdomSample(anchor_mid_x, kAnchorMidStamp, kVelocityX));
        }
    }
    {
        sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
        pose.translation().x() = kReturnX;
        const double return_stamp = config.loop_min_time_separation + 1.0;
        backend.addFrame(
            transformCloud(local, pose), pose, return_stamp);
        backend.recordOdometrySample(
            makeOdomSample(kReturnX, return_stamp, kVelocityX));
        keyframe_x.push_back(kReturnX);
    }

    const int pose_count = kOutbound + 2;
    for (int attempt = 0;
         attempt < 400 && backend.stats().loop_edges == 0;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    expect(backend.stats().loop_edges >= 1,
           "trajectory replay needs a committed loop");
    expect(backend.stats().optimized_poses == static_cast<size_t>(pose_count),
           "trajectory replay needs every keyframe optimized");

    backend.requestSnapshot();
    for (int attempt = 0;
         attempt < 200
             && static_cast<int>(backend.snapshot().optimized_poses.size())
                 < pose_count;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const sapphire::PoseGraphSnapshot snapshot = backend.snapshot();
    const sapphire::MapTrajectory trajectory = backend.mapTrajectory();
    const sapphire::Isometry3d rigid = backend.T_map_odom();
    expect(snapshot.optimized_poses.size() == keyframe_x.size(),
           "snapshot must cover every keyframe used as an anchor");
    expect(trajectory.samples.size() == 4,
           "only the recorded lidar poses are replayed");
    expect(trajectory.revision == backend.stats().revision,
           "trajectory revision must match the published correction");

    const sapphire::MapTrajectorySample* root = findSample(trajectory, 0.0);
    const sapphire::MapTrajectorySample* root_mid =
        findSample(trajectory, kRootMidStamp);
    const sapphire::MapTrajectorySample* anchor_mid =
        findSample(trajectory, kAnchorMidStamp);
    const sapphire::MapTrajectorySample* query = findSample(
        trajectory, config.loop_min_time_separation + 1.0);
    expect(root && root_mid && anchor_mid && query,
           "replay must keep every recorded stamp");
    expect(root->keyframe == 0 && root_mid->keyframe == 0,
           "a scan before the next keyframe stays on the previous keyframe");
    expect(anchor_mid->keyframe == static_cast<size_t>(kAnchor),
           "an interior scan must stay on the keyframe that owned it");
    expect(query->keyframe == keyframe_x.size() - 1,
           "the return scan is owned by the latest keyframe");

    const sapphire::Isometry3d root_odom =
        makeOdomSample(0.0, 0.0, 0.0).T_world_lidar;
    const sapphire::Isometry3d root_mid_odom =
        makeOdomSample(kRootMidX, kRootMidStamp, 0.0).T_world_lidar;
    const sapphire::Isometry3d anchor_mid_odom =
        makeOdomSample(anchor_mid_x, kAnchorMidStamp, 0.0).T_world_lidar;
    const sapphire::Isometry3d query_odom =
        makeOdomSample(kReturnX, 0.0, 0.0).T_world_lidar;

    const auto expectReplay =
        [&](const sapphire::MapTrajectorySample& sample,
            const sapphire::Isometry3d& T_odom) {
            const sapphire::Isometry3d T_odom_i =
                makeOdomSample(keyframe_x[sample.keyframe], 0.0, 0.0)
                    .T_world_lidar;
            const sapphire::Isometry3d expected =
                snapshot.optimized_poses[sample.keyframe]
                * T_odom_i.inverse()
                * T_odom;
            expect(
                sample.T_map_lidar.matrix().isApprox(expected.matrix(), 1e-6),
                "map pose must be T_map_i * T_odom_i^{-1} * T_odom");
            const Eigen::Vector3d expected_velocity =
                (expected.linear() * Eigen::Vector3d(kVelocityX, 0.0, 0.0));
            expect(
                sample.v_map.isApprox(expected_velocity, 1e-6),
                "map velocity must use the same keyframe rotation");
        };
    expectReplay(*root, root_odom);
    expectReplay(*root_mid, root_mid_odom);
    expectReplay(*anchor_mid, anchor_mid_odom);
    expectReplay(*query, query_odom);

    const sapphire::Isometry3d root_relative =
        root->T_map_lidar.inverse() * root_mid->T_map_lidar;
    const sapphire::Isometry3d root_odom_relative =
        root_odom.inverse() * root_mid_odom;
    expect(
        root_relative.matrix().isApprox(root_odom_relative.matrix(), 1e-6),
        "scans on one keyframe must keep their odometry shape");

    const double replay_x = root_mid->T_map_lidar.translation().x();
    const double rigid_x = (rigid * root_mid_odom).translation().x();
    expect(
        std::abs(replay_x - rigid_x) > 0.05,
        "an old keyframe must not follow the latest rigid correction");
    expect(
        std::abs(query->T_map_lidar.translation().x()
                 - (rigid * query_odom).translation().x())
            < 1e-6,
        "the latest keyframe replay must match T_map_odom");
    expect(
        query->T_map_lidar.translation().x() < kReturnX - 0.05,
        "loop must pull the replayed return pose toward the historical frame");
    expect(
        std::abs(root->T_map_lidar.translation().x()) < 1e-3,
        "root-anchored history must stay on the prior");
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

void addKeyframe(
    sapphire::PoseGraphBackend& backend,
    const sapphire::PointCloudConstPtr& cloud,
    double x,
    double stamp)
{
    sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
    pose.translation().x() = x;
    backend.addFrame(transformCloud(cloud, pose), pose, stamp);
}

/// The nearest in-radius frame fails the point-count gate. The next frame
/// is the same corner as the query, so it is accepted, and the farther
/// frame is not added as a second edge.
void testFailedCandidateFallsThroughToNext() {
    sapphire::Config::Pgo config;
    config.enabled = true;
    config.keyframe_distance = 0.05;
    config.update_period_sec = 0.02;
    config.loop_min_time_separation = 1.0;
    config.loop_min_travel_distance = 0.5;
    config.loop_search_radius = 2.0;
    config.loop_search_stride = 1;
    config.target_frame_count = 0;
    config.source_voxel_size = 0.05;
    config.target_voxel_size = 0.05;

    sapphire::PoseGraphBackend backend(config);
    const auto corner = makeCornerCloud();
    const auto sparse = makeLocalCloud();
    addKeyframe(backend, corner, 0.0, 0.0);
    addKeyframe(backend, sparse, 0.2, 2.0);
    addKeyframe(backend, corner, 4.0, 3.0);
    addKeyframe(backend, corner, 0.35, 5.0);

    for (int attempt = 0;
         attempt < 400 && backend.stats().loop_edges == 0;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    backend.requestSnapshot();
    for (int attempt = 0;
         attempt < 200 && backend.snapshot().optimized_poses.size() < 4;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const sapphire::PoseGraphSnapshot snapshot = backend.snapshot();
    const sapphire::PoseGraphStats stats = backend.stats();
    expect(stats.loop_registration_failures >= 1,
           "the nearer sparse cloud must be rejected before the next try");
    expect(stats.loop_edges == 1,
           "a later candidate must still add one loop edge");
    expect(snapshot.loop_edges.size() == 1,
           "one query must not keep a second successful candidate");
    expect(snapshot.loop_edges.front().source == 3
               && snapshot.loop_edges.front().target == 0,
           "the accepted loop must be the first candidate that passes GICP");
}

/// The nearest frame already has the odometry edge, so it is not registered.
/// The next historical frame is the one loop.
void testLinkedCandidateIsSkipped() {
    sapphire::Config::Pgo config;
    config.enabled = true;
    config.keyframe_distance = 0.05;
    config.update_period_sec = 0.02;
    config.loop_min_time_separation = 0.5;
    config.loop_min_travel_distance = 0.3;
    config.loop_search_radius = 2.0;
    config.loop_search_stride = 1;
    config.target_frame_count = 0;
    config.source_voxel_size = 0.05;
    config.target_voxel_size = 0.05;
    config.lidar_translation_sigma_m = 0.5;

    sapphire::PoseGraphBackend backend(config);
    const auto corner = makeCornerCloud();
    // The predecessor is the nearest corner and would register, but it is
    // only 0.4 s after the target, inside the time gate, so it never becomes
    // a query. The return is later and closer to that predecessor than to
    // the target, so skipping the odometry edge is what selects the target.
    addKeyframe(backend, corner, 0.6, 0.0);
    addKeyframe(backend, corner, 5.0, 0.2);
    addKeyframe(backend, corner, 0.45, 0.4);
    addKeyframe(backend, corner, 0.0, 1.2);

    for (int attempt = 0;
         attempt < 400 && backend.stats().loop_edges == 0;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    backend.requestSnapshot();
    for (int attempt = 0;
         attempt < 200 && backend.snapshot().optimized_poses.size() < 4;
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const sapphire::PoseGraphSnapshot snapshot = backend.snapshot();
    const sapphire::PoseGraphStats stats = backend.stats();
    expect(stats.loop_registration_failures == 0,
           "the odometry neighbor must be skipped before registration");
    expect(stats.loop_edges == 1,
           "the next unlinked candidate must be accepted");
    expect(snapshot.loop_edges.size() == 1
               && snapshot.loop_edges.front().source == 3
               && snapshot.loop_edges.front().target == 0,
           "the loop must skip the nearer keyframe that already has an edge");
}

/// Six historical frames sit inside the radius. Only the five nearest are
/// registered. Those five are too sparse to pass, and the sixth is the
/// corner that would have closed the loop.
void testSearchStopsAfterFiveCandidates() {
    sapphire::Config::Pgo config;
    config.enabled = true;
    config.keyframe_distance = 0.05;
    config.update_period_sec = 0.02;
    config.loop_min_time_separation = 0.5;
    config.loop_min_travel_distance = 0.3;
    config.loop_search_radius = 2.5;
    config.loop_search_stride = 1;
    config.target_frame_count = 0;
    config.source_voxel_size = 0.05;
    config.target_voxel_size = 0.05;

    sapphire::PoseGraphBackend backend(config);
    const auto corner = makeCornerCloud();
    const auto sparse = makeLocalCloud();
    // Shared stamps keep the historical frames inside the time gate of each
    // other. Only the return is old enough to search, and its predecessor
    // sits outside the radius.
    addKeyframe(backend, corner, 1.8, 0.0);
    addKeyframe(backend, sparse, 0.1, 0.0);
    addKeyframe(backend, sparse, 0.2, 0.0);
    addKeyframe(backend, sparse, 0.3, 0.0);
    addKeyframe(backend, sparse, 0.4, 0.0);
    addKeyframe(backend, sparse, 0.5, 0.0);
    addKeyframe(backend, corner, 5.0, 0.0);
    addKeyframe(backend, corner, 0.0, 2.0);

    for (int attempt = 0;
         attempt < 400
             && (backend.stats().optimized_poses < 8
                 || backend.stats().loop_candidates < 5);
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const sapphire::PoseGraphStats stats = backend.stats();
    expect(stats.optimized_poses == 8,
           "every keyframe must enter the graph before the cap is checked");
    expect(stats.loop_candidates == 5,
           "a query must register at most five historical frames");
    expect(stats.loop_registration_failures == 5,
           "the five nearest frames are the sparse ones");
    expect(stats.loop_residual_rejections == 0,
           "the sixth frame must not be registered or committed");
    expect(stats.loop_edges == 0,
           "stopping at five must leave the farther corner untried");
}

}  // namespace

int main() {
    testOdometryOnlyGraphPreservesFrontendFrame();
    testLoopClosureAddsConstraint();
    testLoopRelinearizationMovesDistantKeyframe();
    testSecondLoopCommitsAfterFirst();
    testInconsistentLoopLeavesCommittedGraph();
    testHistoricalTrajectoryFollowsOwningKeyframe();
    testSparseCloudRejectsLoop();
    testFailedCandidateFallsThroughToNext();
    testLinkedCandidateIsSkipped();
    testSearchStopsAfterFiveCandidates();
    std::cout << "All pose graph tests passed\n";
    return EXIT_SUCCESS;
}
