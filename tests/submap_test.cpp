#include <sapphire/odometry/submap.hpp>

#include <SO3.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

[[noreturn]] void fail(const std::string& message) {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

void expect(bool condition, const std::string& message) {
    if (!condition) {
        fail(message);
    }
}

sapphire::PointCloud::Ptr makeCloud(float x) {
    auto cloud = std::make_shared<sapphire::PointCloud>();
    sapphire::Point point{};
    point.x = x;
    point.y = 0.0f;
    point.z = 0.0f;
    point.data[3] = 1.0f;
    cloud->push_back(point);
    return cloud;
}

sapphire::Isometry3d makePose(
    double x, double angle_radians = 0.0)
{
    sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
    pose.translation().x() = x;
    pose.linear() =
        lie::SO3d::exp(angle_radians * Eigen::Vector3d::UnitZ()).R();
    return pose;
}

void testKeyframeThresholds() {
    sapphire::Config config;
    sapphire::SubmapManager translation_manager(config.odometry.submap);
    expect(translation_manager.shouldAddKeyframe(makePose(0.0)),
           "the first accepted scan must become a keyframe");
    expect(translation_manager.addKeyframe(
               makePose(0.0), makeCloud(0.0f), 1.0),
           "the first keyframe must build the target");

    expect(!translation_manager.shouldAddKeyframe(makePose(2.0)),
           "the translation threshold is strict");
    expect(translation_manager.shouldAddKeyframe(makePose(2.01)),
           "crossing 2 m must add a keyframe");
    translation_manager.addKeyframe(
        makePose(2.01), makeCloud(2.01f), 2.0);
    expect(translation_manager.keyframeCount() == 2,
           "a scan crossing 2 m must be retained");

    sapphire::SubmapManager rotation_manager(config.odometry.submap);
    rotation_manager.addKeyframe(makePose(0.0), makeCloud(0.0f), 1.0);
    const double forty_five_degrees = M_PI / 4.0;
    expect(!rotation_manager.shouldAddKeyframe(
               makePose(0.0, forty_five_degrees)),
           "the rotation threshold is strict");
    expect(rotation_manager.shouldAddKeyframe(
               makePose(0.0, forty_five_degrees + 1e-3)),
           "crossing 45 degrees must add a keyframe");
    rotation_manager.addKeyframe(
        makePose(0.0, forty_five_degrees + 1e-3),
        makeCloud(0.0f),
        2.0);
    expect(rotation_manager.keyframeCount() == 2,
           "a scan crossing 45 degrees must be retained");
}

void testNearestSelectionAndStableTarget() {
    sapphire::Config config;
    config.odometry.submap.max_keyframes = 10;
    config.odometry.submap.voxel_size = 0.01;
    sapphire::SubmapManager manager(config.odometry.submap);

    for (size_t i = 0; i < 12; ++i) {
        const double x = static_cast<double>(i) * 5.0;
        manager.addKeyframe(
            makePose(x), makeCloud(static_cast<float>(x)), x);
    }

    const std::vector<size_t> expected{2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    expect(manager.activeKeyframeIndices() == expected,
           "submap must contain the ten keyframes nearest the current pose");
    expect(manager.target()->size() == 10,
           "target must merge only active keyframe clouds");
    expect(manager.storedPointCount() == 12,
           "stored point count must include all retained keyframes");

    const size_t revision = manager.targetRevision();
    expect(!manager.shouldAddKeyframe(makePose(56.0)),
           "motion below 2 m must not create a keyframe");
    expect(manager.targetRevision() == revision,
           "continuous sub-threshold motion must keep the target stable");
}

void testPrunesStaleKeyframes() {
    // max_keyframes=3 → buffer limit is 9.  With enough frames the
    // 3-frame active window slides forward and frames far behind are
    // evicted.
    sapphire::Config config;
    config.odometry.submap.max_keyframes = 3;
    config.odometry.submap.voxel_size = 0.01;
    sapphire::SubmapManager manager(config.odometry.submap);

    // Add enough keyframes to guarantee pruning fires at least twice
    // so the stored set stabilises at exactly the active-window size.
    for (size_t i = 0; i < 24; ++i) {
        const double x = static_cast<double>(i) * 5.0;
        manager.addKeyframe(
            makePose(x), makeCloud(static_cast<float>(x)), x);
    }

    const auto active = manager.activeKeyframeIndices();
    expect(active.size() == 3,
           "active window must be bounded by max_keyframes");
    expect(manager.keyframeCount() == active.size(),
           "stale keyframes must be evicted, not just hidden");
    expect(manager.storedPointCount() == manager.keyframeCount(),
           "stored point count must match retained keyframe count");
}

void testPrunePreservesActiveIndices() {
    // After pruning, active_indices_ must still point to the correct
    // keyframes inside the compacted vector.
    sapphire::Config config;
    config.odometry.submap.max_keyframes = 2;
    config.odometry.submap.voxel_size = 0.01;
    sapphire::SubmapManager manager(config.odometry.submap);

    // Add 8 keyframes.
    for (size_t i = 0; i < 8; ++i) {
        const double x = static_cast<double>(i) * 5.0;
        manager.addKeyframe(
            makePose(x), makeCloud(static_cast<float>(x)), x);
    }

    // pruning limit = 3×2 = 6, and 8 > 6 so pruning fired.
    expect(manager.activeKeyframeIndices().size() == 2,
           "2-frame window must be maintained after pruning");
    // Each active index must be a valid subscript into keyframes_.
    for (size_t idx : manager.activeKeyframeIndices()) {
        expect(idx < manager.keyframeCount(),
               "remapped index must stay in bounds");
    }
    // The target must still resolve.
    expect(manager.target() != nullptr,
           "target must be valid after pruning");
}

void testVoxelDownsampling() {
    sapphire::Config config;
    config.odometry.submap.voxel_size = 0.25;
    sapphire::SubmapManager manager(config.odometry.submap);

    auto cloud = std::make_shared<sapphire::PointCloud>();
    cloud->push_back(*makeCloud(0.01f)->begin());
    cloud->push_back(*makeCloud(0.02f)->begin());
    manager.addKeyframe(makePose(0.0), cloud, 0.0);

    expect(manager.target()->size() == 1,
           "points in one 0.25 m voxel must be downsampled");
    expect(manager.storedPointCount() == 2,
           "stored point count must report keyframe memory, not target size");
    expect(std::abs(manager.target()->front().data[3] - 1.0f) < 1e-6f,
           "downsampled points must retain homogeneous coordinate w=1");
}

}  // namespace

int main() {
    testKeyframeThresholds();
    testNearestSelectionAndStableTarget();
    testVoxelDownsampling();
    testPrunesStaleKeyframes();
    testPrunePreservesActiveIndices();
    std::cout << "All submap tests passed\n";
    return EXIT_SUCCESS;
}
