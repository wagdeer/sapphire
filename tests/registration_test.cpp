#include <sapphire/odometry/registration.hpp>

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

sapphire::PointCloud::Ptr makeCloud(float x_offset = 0.0f) {
    auto cloud = std::make_shared<sapphire::PointCloud>();
    for (int x = 0; x < 5; ++x) {
        for (int y = 0; y < 5; ++y) {
            for (int z = 0; z < 4; ++z) {
                sapphire::Point point{};
                point.x = x_offset + 0.25f * static_cast<float>(x);
                point.y = 0.30f * static_cast<float>(y);
                point.z = 0.40f * static_cast<float>(z);
                point.data[3] = 1.0f;
                cloud->points.push_back(point);
            }
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

sapphire::PointCloud::Ptr makeAsymmetricCloud(float x_offset = 0.0f) {
    auto cloud = std::make_shared<sapphire::PointCloud>();
    for (int x = 0; x < 10; ++x) {
        for (int y = 0; y < 10; ++y) {
            for (int z = 0; z < 2; ++z) {
                sapphire::Point point{};
                point.x = x_offset + 0.23f * static_cast<float>(x)
                    + 0.017f * static_cast<float>(y * y)
                    + 0.031f * static_cast<float>(z);
                point.y = 0.27f * static_cast<float>(y)
                    + 0.013f * static_cast<float>(x * y)
                    + 0.041f * static_cast<float>(z);
                point.z = 0.31f * static_cast<float>(z)
                    + 0.019f * static_cast<float>(x * x)
                    + 0.023f * static_cast<float>((x + 2 * y) % 5);
                point.data[3] = 1.0f;
                cloud->points.push_back(point);
            }
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

sapphire::PointCloud::Ptr transformCloud(
    const sapphire::PointCloud& cloud,
    const sapphire::Isometry3d& pose)
{
    auto out = std::make_shared<sapphire::PointCloud>();
    out->points.reserve(cloud.size());
    const sapphire::Isometry3f pose_f = pose.cast<float>();
    for (const sapphire::Point& source : cloud.points) {
        sapphire::Point point = source;
        point.getVector3fMap() = pose_f * source.getVector3fMap();
        point.data[3] = 1.0f;
        out->points.push_back(point);
    }
    out->width = out->points.size();
    out->height = 1;
    out->is_dense = true;
    return out;
}

sapphire::Isometry3d poseAt(double x, double yaw_rad) {
    sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
    pose.translation() = sapphire::Vector3d(x, 0.0, 0.0);
    pose.linear() =
        Eigen::AngleAxisd(yaw_rad, sapphire::Vector3d::UnitZ())
            .toRotationMatrix();
    return pose;
}

sapphire::RegistrationConfig makeConfig() {
    sapphire::RegistrationConfig config;
    config.k_correspondences = 10;
    config.min_num_points = 50;
    return config;
}

void testAlignWithoutTargetIsRejected() {
    sapphire::Registration registration(makeConfig());
    registration.setSource(makeCloud());
    const auto result =
        registration.align(sapphire::Isometry3d::Identity());
    expect(!result.accepted,
           "registration without target must not be accepted");
}

void testTranslationIsRecovered() {
    sapphire::Registration registration(makeConfig());
    registration.setTarget(makeCloud());
    registration.setSource(makeCloud(-0.2f));
    const auto result =
        registration.align(sapphire::Isometry3d::Identity());

    expect(result.accepted, "structured translated clouds must align");
    expect(std::abs(result.T_world_lidar.translation().x() - 0.2) < 1e-3,
           "GICP must recover source-to-target translation");
}

void testVgicpTranslationIsRecovered() {
    auto config = makeConfig();
    config.min_num_points = 30;
    sapphire::Registration registration(config, "VGICP", 0.5);
    registration.setTarget(makeAsymmetricCloud());
    registration.setSource(makeAsymmetricCloud(-0.2f));
    const auto result =
        registration.align(sapphire::Isometry3d::Identity());

    expect(result.accepted, "structured translated clouds must align with VGICP");
    expect(
        std::abs(result.T_world_lidar.translation().x() - 0.2) < 5e-3,
        "VGICP must recover source-to-target translation, actual x="
            + std::to_string(result.T_world_lidar.translation().x()));
}

void testExcessiveCorrectionIsRejected() {
    auto config = makeConfig();
    config.max_correspondence_dist = 2.0;
    config.max_correction_trans = 0.1;
    sapphire::Registration registration(config);
    registration.setTarget(makeCloud());
    registration.setSource(makeCloud(-0.2f));
    const auto result =
        registration.align(sapphire::Isometry3d::Identity());

    expect(result.converged,
           "GICP should converge before correction gate is applied");
    expect(!result.accepted,
           "correction beyond configured translation gate must be rejected");
    expect(result.T_correction.matrix().isApprox(
               Eigen::Matrix4d::Identity(), 1e-12),
           "rejected registration must expose identity correction");
}

void testSensorYawFarFromOriginIsAccepted() {
    auto config = makeConfig();
    config.min_num_points = 30;
    config.max_correspondence_dist = 1.0;
    config.max_correction_trans = 1.0;
    config.max_correction_rot_deg = 20.0;

    const sapphire::Isometry3d T_true = poseAt(100.0, 0.0);
    const double yaw = 1.0 * M_PI / 180.0;
    const sapphire::Isometry3d T_prior = poseAt(100.0, yaw);
    const auto local = makeAsymmetricCloud();

    sapphire::Registration registration(config, "VGICP", 0.5);
    registration.setTarget(transformCloud(*local, T_true));
    registration.setSource(transformCloud(*local, T_prior));
    const auto result = registration.align(T_prior);

    expect(result.converged,
           "VGICP should converge on a 1 degree sensor yaw");
    expect(result.attempted_trans_m > 1.0,
           "lever arm at 100 m must exceed 1 m, actual "
               + std::to_string(result.attempted_trans_m));
    expect(result.attempted_sensor_trans_m < 0.2,
           "sensor displacement of a pure yaw must stay under the gate, actual "
               + std::to_string(result.attempted_sensor_trans_m));
    expect(result.accepted,
           "a 1 degree yaw about the sensor at 100 m must be accepted");
    expect(
        (result.T_world_lidar.translation() - T_true.translation()).norm()
            < 0.05,
        "accepted yaw correction must recover the sensor position");
}

void testSensorTranslationFarFromOriginIsRejected() {
    auto config = makeConfig();
    config.max_correspondence_dist = 2.0;
    config.max_correction_trans = 0.1;

    const sapphire::Isometry3d T_true = poseAt(100.0, 0.0);
    sapphire::Isometry3d T_prior = poseAt(100.0, 0.0);
    T_prior.translation().x() += 0.2;
    const auto local = makeCloud();

    sapphire::Registration registration(config);
    registration.setTarget(transformCloud(*local, T_true));
    registration.setSource(transformCloud(*local, T_prior));
    const auto result = registration.align(T_prior);

    expect(result.converged,
           "GICP should converge before the sensor translation gate");
    expect(result.attempted_sensor_trans_m > 0.1,
           "sensor displacement must exceed the gate, actual "
               + std::to_string(result.attempted_sensor_trans_m));
    expect(!result.accepted,
           "a sensor translation beyond the gate far from the origin must be rejected");
    expect(result.T_correction.matrix().isApprox(
               Eigen::Matrix4d::Identity(), 1e-12),
           "rejected registration must expose identity correction");
}

}  // namespace

int main() {
    testAlignWithoutTargetIsRejected();
    testTranslationIsRecovered();
    testVgicpTranslationIsRecovered();
    testExcessiveCorrectionIsRejected();
    testSensorYawFarFromOriginIsAccepted();
    testSensorTranslationFarFromOriginIsRejected();
    std::cout << "All registration tests passed\n";
    return EXIT_SUCCESS;
}
