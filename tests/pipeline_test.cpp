#include <sapphire/odometry/pipeline.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

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

void expectNear(
    double actual, double expected, double tolerance,
    const std::string& message)
{
    if (std::abs(actual - expected) > tolerance) {
        fail(message);
    }
}

sapphire::Point makePoint(
    float x, float y, float z, double timestamp)
{
    sapphire::Point point{};
    point.x = x;
    point.y = y;
    point.z = z;
    point.data[3] = 1.0f;
    point.timestamp = timestamp;
    return point;
}

sapphire::PointCloud::Ptr makeStructuredScan(float x_offset = 0.0f) {
    auto scan = std::make_shared<sapphire::PointCloud>();
    for (int x = 0; x < 5; ++x) {
        for (int y = 0; y < 5; ++y) {
            for (int z = 0; z < 4; ++z) {
                const size_t index = scan->points.size();
                scan->points.push_back(makePoint(
                    2.0f + x_offset + 0.25f * static_cast<float>(x),
                    -2.0f + 0.3f * static_cast<float>(y),
                    -1.5f + 0.4f * static_cast<float>(z),
                    0.01 + 0.01 * static_cast<double>(index % 5)));
            }
        }
    }
    scan->width = scan->points.size();
    scan->height = 1;
    scan->is_dense = true;
    return scan;
}

void testImuInitializationCreatesGravityAlignedState() {
    sapphire::Config config;
    config.imu.init.min_samples = 5;
    config.imu.init.check_interval_sec = 0.05;
    config.imu.init.timeout_sec = 1.0;

    sapphire::OdometryPipeline pipeline(config);

    const Eigen::Vector3d measured_up =
        Eigen::Vector3d(0.25, -0.35, 0.9027735043).normalized();
    const Eigen::Vector3d accel =
        measured_up * config.imu.init.gravity_mag;
    const Eigen::Vector3d gyro_bias(0.001, -0.002, 0.003);

    for (int i = 0; i < 5; ++i) {
        pipeline.pushImu({
            static_cast<double>(i) * 0.01,
            accel,
            gyro_bias,
        });
    }
    expect(!pipeline.initialized(),
           "pipeline must wait for the convergence check interval");

    for (int i = 5; i <= 10 && !pipeline.initialized(); ++i) {
        pipeline.pushImu({
            static_cast<double>(i) * 0.01,
            accel,
            gyro_bias,
        });
    }

    expect(pipeline.initialized(), "stationary IMU must initialize pipeline");

    const auto& result = pipeline.imuInitResult();
    const Eigen::Vector3d aligned_up =
        result.q_gravity * measured_up;
    expect(aligned_up.isApprox(Eigen::Vector3d::UnitZ(), 1e-9),
           "initial orientation must align measured gravity with world Z");
    expect(result.gyro_bias.isApprox(gyro_bias, 1e-12),
           "stationary gyro mean must initialize gyro bias");
    expect(result.accel_bias.isZero(1e-9),
           "gravity-only acceleration must not become accelerometer bias");
}

void testFirstLidarScanInitializesDeskewedTarget() {
    sapphire::Config config;
    config.imu.init.min_samples = 5;
    config.imu.init.check_interval_sec = 0.05;
    config.imu.init.timeout_sec = 1.0;
    config.registration.gicp.min_num_points = 50;
    config.registration.gicp.k_correspondences = 10;

    sapphire::OdometryPipeline pipeline(config);
    const Eigen::Vector3d stationary_accel(
        0.0, 0.0, config.imu.init.gravity_mag);

    for (int i = 0; i <= 16; ++i) {
        pipeline.pushImu({
            static_cast<double>(i) * 0.01,
            stationary_accel,
            Eigen::Vector3d::Zero(),
        });
    }
    expect(pipeline.initialized(), "stationary IMU must initialize pipeline");

    auto scan = makeStructuredScan();

    pipeline.pushLidar(0.10, scan);

    const auto deskewed = pipeline.latestDeskewed();
    expect(deskewed != nullptr, "first valid scan must produce deskewed output");
    expect(deskewed->size() == scan->size(),
           "first deskewed scan must retain all valid points");
    for (size_t i = 0; i < scan->size(); ++i) {
        expectNear(
            deskewed->points[i].x, scan->points[i].x, 2e-5,
            "stationary deskew must preserve point position");
    }

    const auto& result = pipeline.latestResult();
    expect(result.converged, "first target initialization must be valid");
    expectNear(result.stamp, 0.10, 1e-12,
               "DLIO first scan must use the message header timestamp");
    expect(result.T_world_lidar.matrix().isApprox(
               Eigen::Matrix4d::Identity(), 1e-9),
           "stationary first-frame pose must remain identity");
    expect(pipeline.keyframeCount() == 1,
           "first valid scan must initialize one keyframe");
    const size_t initial_target_revision = pipeline.submapTargetRevision();

    // A sensor translated +0.2 m along world X observes static geometry
    // shifted -0.2 m in its local frame. GICP should recover +0.2 m.
    auto second_scan = makeStructuredScan(-0.2f);

    std::thread lidar_thread([&pipeline, &second_scan]() {
        pipeline.pushLidar(0.20, second_scan);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    for (int i = 17; i <= 26; ++i) {
        pipeline.pushImu({
            static_cast<double>(i) * 0.01,
            stationary_accel,
            Eigen::Vector3d::Zero(),
        });
    }
    lidar_thread.join();

    const auto& second_result = pipeline.latestResult();
    expect(second_result.converged,
           "second scan must be deskewed with complete IMU coverage");
    expectNear(second_result.stamp, 0.23, 1e-12,
               "deskewed scan must use its midpoint reference timestamp");
    expectNear(
        second_result.T_world_lidar.translation().x(),
        0.2,
        1e-3,
        "GICP must recover the source-to-target translation");
    expect(second_result.T_world_lidar.rotation().isApprox(
               Eigen::Matrix3d::Identity(), 1e-5),
           "pure translation must not introduce a rotation");
    expect(pipeline.keyframeCount() == 1,
           "sub-threshold accepted motion must not add a keyframe");
    expect(pipeline.submapTargetRevision() == initial_target_revision,
           "sub-threshold accepted motion must keep the target stable");

    for (int i = 27; i <= 36; ++i) {
        pipeline.pushImu({
            static_cast<double>(i) * 0.01,
            stationary_accel,
            Eigen::Vector3d::Zero(),
        });
    }
    pipeline.pushLidar(0.30, makeStructuredScan(5.0f));
    expect(!pipeline.latestResult().converged,
           "a scan outside the correspondence gate must be rejected");
    expect(pipeline.keyframeCount() == 1,
           "a rejected scan must not enter the submap");
    expect(pipeline.submapTargetRevision() == initial_target_revision,
           "a rejected scan must not rebuild the target");

    for (int i = 37; i <= 46; ++i) {
        pipeline.pushImu({
            static_cast<double>(i) * 0.01,
            stationary_accel,
            Eigen::Vector3d::Zero(),
        });
    }
    pipeline.pushLidar(0.40, makeStructuredScan(-0.2f));

    const auto& recovered_result = pipeline.latestResult();
    expect(recovered_result.converged,
           "a valid scan after rejection must still match the stable target");
    expectNear(
        recovered_result.T_world_lidar.translation().x(),
        0.2,
        1e-2,
        "recovery after rejection must preserve the accepted trajectory");
}

}  // namespace

int main() {
    testImuInitializationCreatesGravityAlignedState();
    testFirstLidarScanInitializesDeskewedTarget();
    std::cout << "All pipeline tests passed\n";
    return EXIT_SUCCESS;
}
