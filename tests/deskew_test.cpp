#include <sapphire/odometry/deskew.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

using sapphire::DeskewResult;
using sapphire::DeskewStatus;
using sapphire::ImuData;
using sapphire::ImuBuffer;
using sapphire::ImuNoiseConfig;
using sapphire::Isometry3d;
using sapphire::Point;
using sapphire::PointCloud;

constexpr double kGravity = 9.80665;
constexpr double kTolerance = 2e-5;

[[noreturn]] void fail(const std::string& message) {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

void expect(bool condition, const std::string& message) {
    if (!condition) {
        fail(message);
    }
}

void expectNear(double actual, double expected, const std::string& message,
                double tolerance = kTolerance) {
    if (std::abs(actual - expected) > tolerance) {
        fail(message + ": expected " + std::to_string(expected)
             + ", got " + std::to_string(actual));
    }
}

Point makePoint(float x, float y, float z, double timestamp) {
    Point point{};
    point.x = x;
    point.y = y;
    point.z = z;
    point.data[3] = 1.0f;
    point.intensity = 7.0f;
    point.timestamp = timestamp;
    return point;
}

PointCloud::Ptr makeCloud(std::initializer_list<Point> points) {
    auto cloud = std::make_shared<PointCloud>();
    cloud->points.assign(points);
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

ImuBuffer makeImuBuffer(
    double start,
    double end,
    double step,
    const Eigen::Vector3d& gyro = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& accel = Eigen::Vector3d(0.0, 0.0, kGravity))
{
    ImuBuffer buffer;
    const int count = static_cast<int>(std::ceil((end - start) / step));
    for (int i = 0; i <= count; ++i) {
        ImuData imu;
        imu.stamp = start + static_cast<double>(i) * step;
        imu.gyro = gyro;
        imu.accel = accel;
        buffer.push_back(imu);
    }
    return buffer;
}

DeskewResult runDeskew(
    const PointCloud::ConstPtr& cloud,
    const ImuBuffer& imu,
    double scan_stamp,
    double prev_stamp,
    const Isometry3d& T_world_imu = Isometry3d::Identity(),
    const Eigen::Vector3d& v_world = Eigen::Vector3d::Zero(),
    const Isometry3d& T_imu_lidar = Isometry3d::Identity(),
    bool time_offset = false)
{
    return sapphire::deskew(
        cloud,
        scan_stamp,
        imu,
        prev_stamp,
        T_world_imu,
        v_world,
        T_imu_lidar,
        Eigen::Vector3d(0.0, 0.0, -kGravity),
        ImuNoiseConfig{},
        time_offset);
}

void testStaticGravityCancellation() {
    auto cloud = makeCloud({
        makePoint(1.0f, 2.0f, 3.0f, 0.025),
        makePoint(-2.0f, 0.5f, 1.0f, 0.055),
        makePoint(0.0f, -1.0f, 4.0f, 0.095),
    });
    auto imu = makeImuBuffer(0.0, 0.11, 0.01);

    const DeskewResult result = runDeskew(cloud, imu, 0.0, 0.0);
    expect(result.status == DeskewStatus::Success, "static scan must succeed");
    for (size_t i = 0; i < cloud->size(); ++i) {
        expectNear(result.cloud->points[i].x, cloud->points[i].x, "static x");
        expectNear(result.cloud->points[i].y, cloud->points[i].y, "static y");
        expectNear(result.cloud->points[i].z, cloud->points[i].z, "gravity cancellation z");
    }
    expectNear(result.v_world_ref.norm(), 0.0, "static reference velocity");
    expect(result.metrics.timestamp_groups == cloud->size(),
           "deskew metrics must expose exact timestamp group count");
    expect(result.metrics.pim_copies <= result.metrics.imu_intervals,
           "partial PIM copies must be bounded by IMU intervals");
}

void testConstantVelocity() {
    auto cloud = makeCloud({
        makePoint(0.0f, 0.0f, 0.0f, 0.025),
        makePoint(0.0f, 0.0f, 0.0f, 0.055),
        makePoint(0.0f, 0.0f, 0.0f, 0.095),
    });
    auto imu = makeImuBuffer(0.0, 0.11, 0.01);

    const DeskewResult result = runDeskew(
        cloud, imu, 0.0, 0.0, Isometry3d::Identity(),
        Eigen::Vector3d(2.0, 0.0, 0.0));
    expect(result.status == DeskewStatus::Success, "constant velocity scan must succeed");
    for (size_t i = 0; i < cloud->size(); ++i) {
        expectNear(result.cloud->points[i].x,
                   2.0 * cloud->points[i].timestamp,
                   "constant velocity displacement");
        expectNear(result.cloud->points[i].z, 0.0, "constant velocity z");
    }
}

void testConstantYawRate() {
    constexpr double yaw_rate = 1.2;
    auto cloud = makeCloud({
        makePoint(1.0f, 0.0f, 0.0f, 0.025),
        makePoint(1.0f, 0.0f, 0.0f, 0.055),
        makePoint(1.0f, 0.0f, 0.0f, 0.095),
    });
    auto imu = makeImuBuffer(
        0.0, 0.11, 0.01, Eigen::Vector3d(0.0, 0.0, yaw_rate));

    const DeskewResult result = runDeskew(cloud, imu, 0.0, 0.0);
    expect(result.status == DeskewStatus::Success, "constant yaw scan must succeed");
    for (size_t i = 0; i < cloud->size(); ++i) {
        const double angle = yaw_rate * cloud->points[i].timestamp;
        expectNear(result.cloud->points[i].x, std::cos(angle), "constant yaw x");
        expectNear(result.cloud->points[i].y, std::sin(angle), "constant yaw y");
    }
}

void testRotatingLeverArmExtrinsic() {
    constexpr double yaw_rate = 1.0;
    auto cloud = makeCloud({
        makePoint(0.0f, 0.0f, 0.0f, 0.025),
        makePoint(0.0f, 0.0f, 0.0f, 0.055),
        makePoint(0.0f, 0.0f, 0.0f, 0.095),
    });
    auto imu = makeImuBuffer(
        0.0, 0.11, 0.01, Eigen::Vector3d(0.0, 0.0, yaw_rate));
    Isometry3d T_imu_lidar = Isometry3d::Identity();
    T_imu_lidar.translation() = Eigen::Vector3d(0.4, -0.2, 0.1);

    const DeskewResult result = runDeskew(
        cloud, imu, 0.0, 0.0, Isometry3d::Identity(),
        Eigen::Vector3d::Zero(), T_imu_lidar);
    expect(result.status == DeskewStatus::Success, "lever-arm scan must succeed");
    for (size_t i = 0; i < cloud->size(); ++i) {
        const double angle = yaw_rate * cloud->points[i].timestamp;
        const Eigen::Vector3d expected =
            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())
            * T_imu_lidar.translation();
        expectNear(result.cloud->points[i].x, expected.x(), "lever-arm x");
        expectNear(result.cloud->points[i].y, expected.y(), "lever-arm y");
        expectNear(result.cloud->points[i].z, expected.z(), "lever-arm z");
    }
}

void testTimeOffsetAlignment() {
    auto cloud = makeCloud({
        makePoint(0.0f, 0.0f, 0.0f, 10.00),
        makePoint(0.0f, 0.0f, 0.0f, 10.05),
    });
    auto imu = makeImuBuffer(1.0, 1.06, 0.01);

    const DeskewResult result = runDeskew(
        cloud, imu, 1.0, 1.0, Isometry3d::Identity(),
        Eigen::Vector3d(1.0, 0.0, 0.0), Isometry3d::Identity(), true);
    expect(result.status == DeskewStatus::Success, "time-offset scan must succeed");
    expectNear(result.cloud->points[0].x, 0.0, "first point aligned to scan stamp");
    expectNear(result.cloud->points[1].x, 0.05, "relative point time retained");
}

void testNoTimestampFallbackUsesWorldTransform() {
    auto cloud = makeCloud({
        makePoint(1.0f, 0.0f, 0.0f, 0.0),
        makePoint(2.0f, 0.0f, 0.0f, 0.0),
    });
    ImuBuffer imu;
    Isometry3d T_world_imu = Isometry3d::Identity();
    T_world_imu.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);
    Isometry3d T_imu_lidar = Isometry3d::Identity();
    T_imu_lidar.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);

    const DeskewResult result = runDeskew(
        cloud, imu, 0.0, 0.0, T_world_imu,
        Eigen::Vector3d::Zero(), T_imu_lidar);
    expect(result.status == DeskewStatus::NoPointTimestamps,
           "identical timestamps must report fallback");
    expectNear(result.cloud->points[0].x, 4.5, "fallback world transform");
}

void testInsufficientImuCoverage() {
    auto cloud = makeCloud({
        makePoint(0.0f, 0.0f, 0.0f, 0.02),
        makePoint(0.0f, 0.0f, 0.0f, 0.10),
    });
    auto imu = makeImuBuffer(0.0, 0.05, 0.01);

    const DeskewResult result = runDeskew(cloud, imu, 0.0, 0.0);
    expect(result.status == DeskewStatus::InsufficientImuCoverage,
           "missing scan-end IMU must fail");
}

void testDenseTimestampsReusePartialPreintegration() {
    constexpr double yaw_rate = 1.2;
    constexpr double velocity = 2.0;
    constexpr size_t point_count = 5000;
    constexpr double point_interval = 0.1 / static_cast<double>(point_count);
    auto cloud = std::make_shared<PointCloud>();
    cloud->reserve(point_count);
    for (size_t i = 0; i < point_count; ++i) {
        const double stamp =
            point_interval + static_cast<double>(i) * point_interval;
        cloud->push_back(makePoint(1.0f, 0.0f, 0.0f, stamp));
    }
    auto imu = makeImuBuffer(
        0.0, 0.11, 0.01, Eigen::Vector3d(0.0, 0.0, yaw_rate));

    const DeskewResult result = runDeskew(
        cloud, imu, 0.0, 0.0, Isometry3d::Identity(),
        Eigen::Vector3d(velocity, 0.0, 0.0));
    expect(result.status == DeskewStatus::Success,
           "dense timestamp scan must deskew successfully");
    expect(result.metrics.timestamp_groups == cloud->size(),
           "all dense timestamps must remain exact groups");
    expect(result.metrics.pim_copies <= result.metrics.imu_intervals,
           "dense scan must copy at most once per IMU interval");
    expect(result.metrics.pim_copies * 10
               < result.metrics.timestamp_groups,
           "dense scan must avoid per-timestamp PIM copies");
    expect(result.metrics.timeline_ms >= 0.0
               && result.metrics.integration_ms >= 0.0
               && result.metrics.transform_ms >= 0.0
               && result.metrics.total_ms >= 0.0,
           "deskew stage timings must be nonnegative");

    for (size_t i = 0; i < cloud->size(); ++i) {
        const double stamp = cloud->points[i].timestamp;
        const double angle = yaw_rate * stamp;
        expectNear(
            result.cloud->points[i].x,
            velocity * stamp + std::cos(angle),
            "dense timestamp translation and yaw X");
        expectNear(
            result.cloud->points[i].y,
            std::sin(angle),
            "dense timestamp yaw Y");
    }
}

}  // namespace

int main() {
    testStaticGravityCancellation();
    testConstantVelocity();
    testConstantYawRate();
    testRotatingLeverArmExtrinsic();
    testTimeOffsetAlignment();
    testNoTimestampFallbackUsesWorldTransform();
    testInsufficientImuCoverage();
    testDenseTimestampsReusePartialPreintegration();
    std::cout << "All deskew tests passed\n";
    return EXIT_SUCCESS;
}
