#include <sapphire/config.hpp>

#include <toml++/toml.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace sapphire {
namespace {

double requiredNumber(const std::optional<double>& value, const char* name) {
    if (!value.has_value()) {
        throw std::runtime_error(
            std::string("missing or non-numeric TOML field: ") + name);
    }
    return *value;
}

Eigen::Vector3d requiredVector3(const toml::array* array, const char* name) {
    if (array == nullptr || array->size() != 3) {
        throw std::runtime_error(
            std::string("TOML field must be a numeric array of length 3: ") + name);
    }

    Eigen::Vector3d result;
    for (size_t i = 0; i < 3; ++i) {
        const auto value = (*array)[i].value<double>();
        if (!value.has_value()) {
            throw std::runtime_error(
                std::string("TOML field must contain only numbers: ") + name);
        }
        result[static_cast<Eigen::Index>(i)] = *value;
    }
    return result;
}

Eigen::Quaterniond requiredQuaternionXyzw(
    const toml::array* array, const char* name)
{
    if (array == nullptr || array->size() != 4) {
        throw std::runtime_error(
            std::string("TOML field must be a numeric array of length 4: ") + name);
    }

    double values[4]{};
    for (size_t i = 0; i < 4; ++i) {
        const auto value = (*array)[i].value<double>();
        if (!value.has_value()) {
            throw std::runtime_error(
                std::string("TOML field must contain only numbers: ") + name);
        }
        values[i] = *value;
    }

    Eigen::Quaterniond quaternion(values[3], values[0], values[1], values[2]);
    if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1e-12) {
        throw std::runtime_error(
            std::string("TOML quaternion is non-finite or has zero norm: ") + name);
    }
    quaternion.normalize();
    return quaternion;
}

void requireFiniteNonnegative(double value, const char* name) {
    if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(
            std::string(name) + " must be finite and nonnegative");
    }
}

}  // namespace

Config loadConfig(const std::filesystem::path& path) {
    toml::table root;
    try {
        root = toml::parse_file(path.string());
    } catch (const toml::parse_error& error) {
        std::ostringstream message;
        message << "failed to parse TOML config '" << path.string() << "': " << error;
        throw std::runtime_error(message.str());
    }

    Config config;
    config.imu.init.gravity_mag = requiredNumber(
        root["imu"]["gravity"].value<double>(), "imu.gravity");

    config.imu.noise.accel_noise_density = requiredNumber(
        root["imu"]["noise"]["accel_noise_density"].value<double>(),
        "imu.noise.accel_noise_density");
    config.imu.noise.gyro_noise_density = requiredNumber(
        root["imu"]["noise"]["gyro_noise_density"].value<double>(),
        "imu.noise.gyro_noise_density");
    config.imu.noise.accel_random_walk = requiredNumber(
        root["imu"]["noise"]["accel_random_walk"].value<double>(),
        "imu.noise.accel_random_walk");
    config.imu.noise.gyro_random_walk = requiredNumber(
        root["imu"]["noise"]["gyro_random_walk"].value<double>(),
        "imu.noise.gyro_random_walk");

    if (const auto voxel_size =
            root["odometry"]["voxel_size"].value<double>()) {
        config.odometry.voxel_size = *voxel_size;
    }
    if (const auto velocity_gain =
            root["odometry"]["observer"]["velocity_gain"].value<double>()) {
        config.odometry.observer.velocity_gain = *velocity_gain;
    }
    if (const auto splitting_distance =
            root["odometry"]["submap"]["splitting_distance"].value<double>()) {
        config.odometry.submap.splitting_distance = *splitting_distance;
    }
    if (const auto splitting_rotation =
            root["odometry"]["submap"]["splitting_rotation"].value<double>()) {
        config.odometry.submap.splitting_rotation = *splitting_rotation;
    }
    if (const auto max_keyframes =
            root["odometry"]["submap"]["max_keyframes"].value<int>()) {
        config.odometry.submap.max_keyframes = *max_keyframes;
    }
    if (const auto voxel_size =
            root["odometry"]["submap"]["voxel_size"].value<double>()) {
        config.odometry.submap.voxel_size = *voxel_size;
    }

    const auto time_offset = root["deskew"]["time_offset"].value<bool>();
    if (!time_offset.has_value()) {
        throw std::runtime_error(
            "missing or non-boolean TOML field: deskew.time_offset");
    }
    config.deskew.time_offset = *time_offset;

    const auto extrinsics = root["extrinsics"]["T_imu_lidar"];
    const Eigen::Vector3d translation = requiredVector3(
        extrinsics["translation"].as_array(),
        "extrinsics.T_imu_lidar.translation");
    const Eigen::Quaterniond rotation = requiredQuaternionXyzw(
        extrinsics["rotation_xyzw"].as_array(),
        "extrinsics.T_imu_lidar.rotation_xyzw");
    config.extrinsics.T_imu_lidar = Isometry3d::Identity();
    config.extrinsics.T_imu_lidar.linear() = rotation.toRotationMatrix();
    config.extrinsics.T_imu_lidar.translation() = translation;

    validateConfig(config);
    return config;
}

void validateConfig(const Config& config) {
    if (!std::isfinite(config.imu.init.gravity_mag)
        || config.imu.init.gravity_mag <= 0.0) {
        throw std::invalid_argument("imu.gravity must be finite and positive");
    }

    requireFiniteNonnegative(
        config.imu.noise.accel_noise_density,
        "imu.noise.accel_noise_density");
    requireFiniteNonnegative(
        config.imu.noise.gyro_noise_density,
        "imu.noise.gyro_noise_density");
    requireFiniteNonnegative(
        config.imu.noise.accel_random_walk,
        "imu.noise.accel_random_walk");
    requireFiniteNonnegative(
        config.imu.noise.gyro_random_walk,
        "imu.noise.gyro_random_walk");
    requireFiniteNonnegative(
        config.odometry.observer.velocity_gain,
        "odometry.observer.velocity_gain");
    if (!std::isfinite(config.odometry.voxel_size)
        || config.odometry.voxel_size <= 0.0) {
        throw std::invalid_argument(
            "odometry.voxel_size must be finite and positive");
    }
    if (!std::isfinite(config.odometry.submap.splitting_distance)
        || config.odometry.submap.splitting_distance <= 0.0) {
        throw std::invalid_argument(
            "odometry.submap.splitting_distance must be finite and positive");
    }
    if (!std::isfinite(config.odometry.submap.splitting_rotation)
        || config.odometry.submap.splitting_rotation <= 0.0) {
        throw std::invalid_argument(
            "odometry.submap.splitting_rotation must be finite and positive");
    }
    if (config.odometry.submap.max_keyframes <= 0) {
        throw std::invalid_argument(
            "odometry.submap.max_keyframes must be positive");
    }
    if (!std::isfinite(config.odometry.submap.voxel_size)
        || config.odometry.submap.voxel_size <= 0.0) {
        throw std::invalid_argument(
            "odometry.submap.voxel_size must be finite and positive");
    }

    const Isometry3d& extrinsic = config.extrinsics.T_imu_lidar;
    if (!extrinsic.matrix().allFinite()) {
        throw std::invalid_argument(
            "extrinsics.T_imu_lidar must contain only finite values");
    }
    const Eigen::Matrix3d should_be_identity =
        extrinsic.rotation().transpose() * extrinsic.rotation();
    if (!should_be_identity.isApprox(Eigen::Matrix3d::Identity(), 1e-9)
        || std::abs(extrinsic.rotation().determinant() - 1.0) > 1e-9) {
        throw std::invalid_argument(
            "extrinsics.T_imu_lidar rotation must be a proper rotation");
    }
}

}  // namespace sapphire
