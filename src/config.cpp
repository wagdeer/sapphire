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

void requireFinitePositive(double value, const char* name) {
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(
            std::string(name) + " must be finite and positive");
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
    if (const auto value =
            root["imu"]["init"]["convergence_gyro_std"].value<double>()) {
        config.imu.init.convergence_gyro_std = *value;
    }
    if (const auto value =
            root["imu"]["init"]["convergence_accel_std"].value<double>()) {
        config.imu.init.convergence_accel_std = *value;
    }
    if (const auto value =
            root["imu"]["init"]["timeout_sec"].value<double>()) {
        config.imu.init.timeout_sec = *value;
    }
    if (const auto value =
            root["imu"]["init"]["check_interval_sec"].value<double>()) {
        config.imu.init.check_interval_sec = *value;
    }
    if (const auto value =
            root["imu"]["init"]["min_samples"].value<int>()) {
        config.imu.init.min_samples = *value;
    }

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
    if (const auto fusion =
            root["odometry"]["fusion"].value<std::string>()) {
        config.odometry.fusion = *fusion;
    }
    if (const auto value =
            root["odometry"]["crop_box"]["min_x"].value<double>()) {
        config.odometry.crop_box.min_x = *value;
    }
    if (const auto value =
            root["odometry"]["crop_box"]["min_y"].value<double>()) {
        config.odometry.crop_box.min_y = *value;
    }
    if (const auto value =
            root["odometry"]["crop_box"]["min_z"].value<double>()) {
        config.odometry.crop_box.min_z = *value;
    }
    if (const auto value =
            root["odometry"]["crop_box"]["max_x"].value<double>()) {
        config.odometry.crop_box.max_x = *value;
    }
    if (const auto value =
            root["odometry"]["crop_box"]["max_y"].value<double>()) {
        config.odometry.crop_box.max_y = *value;
    }
    if (const auto value =
            root["odometry"]["crop_box"]["max_z"].value<double>()) {
        config.odometry.crop_box.max_z = *value;
    }
    if (const auto velocity_gain =
            root["odometry"]["observer"]["velocity_gain"].value<double>()) {
        config.odometry.observer.velocity_gain = *velocity_gain;
    }
    if (const auto position_gain =
            root["odometry"]["observer"]["position_gain"].value<double>()) {
        config.odometry.observer.position_gain = *position_gain;
    }
    if (const auto orientation_gain =
            root["odometry"]["observer"]["orientation_gain"].value<double>()) {
        config.odometry.observer.orientation_gain = *orientation_gain;
    }
    if (const auto accel_bias_gain =
            root["odometry"]["observer"]["accel_bias_gain"].value<double>()) {
        config.odometry.observer.accel_bias_gain = *accel_bias_gain;
    }
    if (const auto gyro_bias_gain =
            root["odometry"]["observer"]["gyro_bias_gain"].value<double>()) {
        config.odometry.observer.gyro_bias_gain = *gyro_bias_gain;
    }
    if (const auto accel_bias_max =
            root["odometry"]["observer"]["accel_bias_max"].value<double>()) {
        config.odometry.observer.accel_bias_max = *accel_bias_max;
    }
    if (const auto gyro_bias_max =
            root["odometry"]["observer"]["gyro_bias_max"].value<double>()) {
        config.odometry.observer.gyro_bias_max = *gyro_bias_max;
    }
    if (const auto value =
            root["odometry"]["eskf"]["sigma_rotation"].value<double>()) {
        config.odometry.eskf.sigma_rotation = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["sigma_translation"].value<double>()) {
        config.odometry.eskf.sigma_translation = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["icp_covariance_scale"].value<double>()) {
        config.odometry.eskf.icp_covariance_scale = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["use_hessian"].value<bool>()) {
        config.odometry.eskf.use_hessian = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["hessian_min_information"]
                .value<double>()) {
        config.odometry.eskf.hessian_min_information = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["hessian_max_condition"]
                .value<double>()) {
        config.odometry.eskf.hessian_max_condition = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["hessian_degenerate_sigma"]
                .value<double>()) {
        config.odometry.eskf.hessian_degenerate_sigma = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["hessian_max_sigma"].value<double>()) {
        config.odometry.eskf.hessian_max_sigma = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["mahalanobis_threshold"].value<double>()) {
        config.odometry.eskf.mahalanobis_threshold = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["inject_full_pose"].value<bool>()) {
        config.odometry.eskf.inject_full_pose = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["inject_directional_pose"].value<bool>()) {
        config.odometry.eskf.inject_directional_pose = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["bias_update_scale"].value<double>()) {
        config.odometry.eskf.bias_update_scale = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["velocity_correction_gain"]
                .value<double>()) {
        config.odometry.eskf.velocity_correction_gain = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["accel_bias_max"].value<double>()) {
        config.odometry.eskf.accel_bias_max = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["gyro_bias_max"].value<double>()) {
        config.odometry.eskf.gyro_bias_max = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["init_sigma_theta"].value<double>()) {
        config.odometry.eskf.init_sigma_theta = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["init_sigma_velocity"].value<double>()) {
        config.odometry.eskf.init_sigma_velocity = *value;
    }
    if (const auto value =
            root["odometry"]["eskf"]["init_sigma_position"].value<double>()) {
        config.odometry.eskf.init_sigma_position = *value;
    }
    if (const auto value =
            root["imu"]["noise"]["accel_bias_rw_sigma"].value<double>()) {
        config.imu.noise.accel_bias_rw_sigma = *value;
    }
    if (const auto value =
            root["imu"]["noise"]["gyro_bias_rw_sigma"].value<double>()) {
        config.imu.noise.gyro_bias_rw_sigma = *value;
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

    if (const auto value = root["registration"]["type"].value<std::string>()) {
        config.registration.type = *value;
    }
    if (const auto value =
            root["registration"]["gicp"]["max_iterations"].value<int>()) {
        config.registration.gicp.max_iterations = *value;
    }
    if (const auto value =
            root["registration"]["gicp"]["transformation_epsilon"].value<double>()) {
        config.registration.gicp.transformation_epsilon = *value;
    }
    if (const auto value =
            root["registration"]["gicp"]["rotation_epsilon"].value<double>()) {
        config.registration.gicp.rotation_epsilon = *value;
    }
    if (const auto value =
            root["registration"]["gicp"]["max_correspondence_dist"].value<double>()) {
        config.registration.gicp.max_correspondence_dist = *value;
    }
    if (const auto value =
            root["registration"]["gicp"]["k_correspondences"].value<int>()) {
        config.registration.gicp.k_correspondences = *value;
    }
    if (const auto value =
            root["registration"]["gicp"]["min_num_points"].value<int>()) {
        config.registration.gicp.min_num_points = *value;
    }
    if (const auto value =
            root["registration"]["gicp"]["max_correction_trans"].value<double>()) {
        config.registration.gicp.max_correction_trans = *value;
    }
    if (const auto value =
            root["registration"]["gicp"]["max_correction_rot_deg"].value<double>()) {
        config.registration.gicp.max_correction_rot_deg = *value;
    }
    if (const auto value =
            root["registration"]["vgicp"]["voxel_resolution"].value<double>()) {
        config.registration.vgicp.voxel_resolution = *value;
    }
    if (const auto value = root["pgo"]["enabled"].value<bool>()) {
        config.pgo.enabled = *value;
    }
    if (const auto value = root["pgo"]["keyframe_distance"].value<double>()) {
        config.pgo.keyframe_distance = *value;
    }
    if (const auto value = root["pgo"]["keyframe_rotation"].value<double>()) {
        config.pgo.keyframe_rotation = *value;
    }
    if (const auto value =
            root["pgo"]["loop_min_time_separation"].value<double>()) {
        config.pgo.loop_min_time_separation = *value;
    }
    if (const auto value =
            root["pgo"]["loop_min_travel_distance"].value<double>()) {
        config.pgo.loop_min_travel_distance = *value;
    }
    if (const auto value = root["pgo"]["loop_max_rotation"].value<double>()) {
        config.pgo.loop_max_rotation = *value;
    }
    if (const auto value = root["pgo"]["loop_search_radius"].value<double>()) {
        config.pgo.loop_search_radius = *value;
    }
    if (const auto value = root["pgo"]["loop_search_stride"].value<int>()) {
        config.pgo.loop_search_stride = *value;
    }
    if (const auto value = root["pgo"]["target_frame_count"].value<int>()) {
        config.pgo.target_frame_count = *value;
    }
    if (const auto value = root["pgo"]["source_voxel_size"].value<double>()) {
        config.pgo.source_voxel_size = *value;
    }
    if (const auto value = root["pgo"]["target_voxel_size"].value<double>()) {
        config.pgo.target_voxel_size = *value;
    }
    if (const auto value = root["pgo"]["fitness_threshold"].value<double>()) {
        config.pgo.fitness_threshold = *value;
    }
    if (const auto value = root["pgo"]["update_period_sec"].value<double>()) {
        config.pgo.update_period_sec = *value;
    }
    if (const auto value = root["pgo"]["map_frame_stride"].value<int>()) {
        config.pgo.map_frame_stride = *value;
    }
    if (const auto value = root["pgo"]["map_voxel_size"].value<double>()) {
        config.pgo.map_voxel_size = *value;
    }
    if (const auto value = root["pgo"]["occupancy"]["enabled"].value<bool>()) {
        config.pgo.occupancy.enabled = *value;
    }
    if (const auto value =
            root["pgo"]["occupancy"]["resolution"].value<double>()) {
        config.pgo.occupancy.resolution = *value;
    }
    if (const auto value =
            root["pgo"]["occupancy"]["h_clearance"].value<double>()) {
        config.pgo.occupancy.h_clearance = *value;
    }
    if (const auto value = root["pgo"]["occupancy"]["d_max"].value<double>()) {
        config.pgo.occupancy.d_max = *value;
    }
    if (const auto value =
            root["pgo"]["occupancy"]["occ_threshold"].value<double>()) {
        config.pgo.occupancy.occ_threshold = *value;
    }
    if (const auto value =
            root["pgo"]["occupancy"]["usable_range"].value<double>()) {
        config.pgo.occupancy.usable_range = *value;
    }
    if (const auto value =
            root["pgo"]["occupancy"]["min_range"].value<double>()) {
        config.pgo.occupancy.min_range = *value;
    }
    if (const auto value =
            root["pgo"]["occupancy"]["cloud_voxel_size"].value<double>()) {
        config.pgo.occupancy.cloud_voxel_size = *value;
    }
    if (const auto value =
            root["pgo"]["occupancy"]["margin"].value<double>()) {
        config.pgo.occupancy.margin = *value;
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

    requireFinitePositive(
        config.imu.noise.accel_noise_density,
        "imu.noise.accel_noise_density");
    requireFinitePositive(
        config.imu.noise.gyro_noise_density,
        "imu.noise.gyro_noise_density");
    requireFinitePositive(
        config.imu.noise.accel_random_walk,
        "imu.noise.accel_random_walk");
    requireFinitePositive(
        config.imu.noise.gyro_random_walk,
        "imu.noise.gyro_random_walk");
    requireFinitePositive(
        config.imu.init.convergence_gyro_std,
        "imu.init.convergence_gyro_std");
    requireFinitePositive(
        config.imu.init.convergence_accel_std,
        "imu.init.convergence_accel_std");
    requireFinitePositive(
        config.imu.init.timeout_sec,
        "imu.init.timeout_sec");
    requireFinitePositive(
        config.imu.init.check_interval_sec,
        "imu.init.check_interval_sec");
    if (config.imu.init.min_samples <= 0) {
        throw std::invalid_argument("imu.init.min_samples must be positive");
    }
    requireFiniteNonnegative(
        config.odometry.observer.position_gain,
        "odometry.observer.position_gain");
    requireFiniteNonnegative(
        config.odometry.observer.velocity_gain,
        "odometry.observer.velocity_gain");
    requireFiniteNonnegative(
        config.odometry.observer.orientation_gain,
        "odometry.observer.orientation_gain");
    requireFiniteNonnegative(
        config.odometry.observer.accel_bias_gain,
        "odometry.observer.accel_bias_gain");
    requireFiniteNonnegative(
        config.odometry.observer.gyro_bias_gain,
        "odometry.observer.gyro_bias_gain");
    requireFiniteNonnegative(
        config.odometry.observer.accel_bias_max,
        "odometry.observer.accel_bias_max");
    requireFiniteNonnegative(
        config.odometry.observer.gyro_bias_max,
        "odometry.observer.gyro_bias_max");
    if (config.odometry.fusion != "observer"
        && config.odometry.fusion != "eskf") {
        throw std::invalid_argument(
            "odometry.fusion must be \"observer\" or \"eskf\"");
    }
    requireFinitePositive(
        config.odometry.eskf.sigma_rotation,
        "odometry.eskf.sigma_rotation");
    requireFinitePositive(
        config.odometry.eskf.sigma_translation,
        "odometry.eskf.sigma_translation");
    requireFinitePositive(
        config.odometry.eskf.icp_covariance_scale,
        "odometry.eskf.icp_covariance_scale");
    requireFinitePositive(
        config.odometry.eskf.hessian_min_information,
        "odometry.eskf.hessian_min_information");
    requireFinitePositive(
        config.odometry.eskf.hessian_max_condition,
        "odometry.eskf.hessian_max_condition");
    requireFinitePositive(
        config.odometry.eskf.hessian_degenerate_sigma,
        "odometry.eskf.hessian_degenerate_sigma");
    requireFinitePositive(
        config.odometry.eskf.hessian_max_sigma,
        "odometry.eskf.hessian_max_sigma");
    if (config.odometry.eskf.hessian_degenerate_sigma
        > config.odometry.eskf.hessian_max_sigma) {
        throw std::invalid_argument(
            "odometry.eskf.hessian_degenerate_sigma must not exceed "
            "hessian_max_sigma");
    }
    if (!std::isfinite(config.odometry.eskf.mahalanobis_threshold)) {
        throw std::invalid_argument(
            "odometry.eskf.mahalanobis_threshold must be finite "
            "(use <=0 to disable)");
    }
    requireFiniteNonnegative(
        config.odometry.eskf.bias_update_scale,
        "odometry.eskf.bias_update_scale");
    requireFiniteNonnegative(
        config.odometry.eskf.velocity_correction_gain,
        "odometry.eskf.velocity_correction_gain");
    requireFiniteNonnegative(
        config.odometry.eskf.accel_bias_max,
        "odometry.eskf.accel_bias_max");
    requireFiniteNonnegative(
        config.odometry.eskf.gyro_bias_max,
        "odometry.eskf.gyro_bias_max");
    requireFinitePositive(
        config.odometry.eskf.init_sigma_theta,
        "odometry.eskf.init_sigma_theta");
    requireFinitePositive(
        config.odometry.eskf.init_sigma_velocity,
        "odometry.eskf.init_sigma_velocity");
    requireFinitePositive(
        config.odometry.eskf.init_sigma_position,
        "odometry.eskf.init_sigma_position");
    if (!std::isfinite(config.odometry.voxel_size)
        || config.odometry.voxel_size <= 0.0) {
        throw std::invalid_argument(
            "odometry.voxel_size must be finite and positive");
    }
    const auto& crop = config.odometry.crop_box;
    if (!std::isfinite(crop.min_x) || !std::isfinite(crop.min_y)
        || !std::isfinite(crop.min_z) || !std::isfinite(crop.max_x)
        || !std::isfinite(crop.max_y) || !std::isfinite(crop.max_z)
        || crop.min_x >= crop.max_x || crop.min_y >= crop.max_y
        || crop.min_z >= crop.max_z) {
        throw std::invalid_argument(
            "odometry.crop_box bounds must be finite and ordered");
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
    if (config.registration.type != "GICP"
        && config.registration.type != "VGICP") {
        throw std::invalid_argument(
            "registration.type must be GICP or VGICP");
    }
    const auto& gicp = config.registration.gicp;
    if (gicp.max_iterations <= 0 || gicp.k_correspondences <= 0
        || gicp.min_num_points <= 0) {
        throw std::invalid_argument(
            "registration.gicp integer limits must be positive");
    }
    requireFinitePositive(
        gicp.transformation_epsilon,
        "registration.gicp.transformation_epsilon");
    requireFinitePositive(
        gicp.rotation_epsilon,
        "registration.gicp.rotation_epsilon");
    requireFinitePositive(
        gicp.max_correspondence_dist,
        "registration.gicp.max_correspondence_dist");
    requireFinitePositive(
        gicp.max_correction_trans,
        "registration.gicp.max_correction_trans");
    requireFinitePositive(
        gicp.max_correction_rot_deg,
        "registration.gicp.max_correction_rot_deg");
    requireFinitePositive(
        config.registration.vgicp.voxel_resolution,
        "registration.vgicp.voxel_resolution");
    requireFinitePositive(
        config.pgo.keyframe_distance,
        "pgo.keyframe_distance");
    requireFinitePositive(
        config.pgo.keyframe_rotation,
        "pgo.keyframe_rotation");
    requireFiniteNonnegative(
        config.pgo.loop_min_time_separation,
        "pgo.loop_min_time_separation");
    requireFiniteNonnegative(
        config.pgo.loop_min_travel_distance,
        "pgo.loop_min_travel_distance");
    requireFinitePositive(
        config.pgo.loop_max_rotation,
        "pgo.loop_max_rotation");
    requireFinitePositive(
        config.pgo.loop_search_radius,
        "pgo.loop_search_radius");
    if (config.pgo.loop_search_stride <= 0
        || config.pgo.target_frame_count < 0) {
        throw std::invalid_argument(
            "pgo loop_search_stride must be positive and "
            "target_frame_count must be nonnegative");
    }
    requireFinitePositive(
        config.pgo.source_voxel_size,
        "pgo.source_voxel_size");
    requireFinitePositive(
        config.pgo.target_voxel_size,
        "pgo.target_voxel_size");
    requireFinitePositive(
        config.pgo.fitness_threshold,
        "pgo.fitness_threshold");
    requireFinitePositive(
        config.pgo.update_period_sec,
        "pgo.update_period_sec");
    if (config.pgo.map_frame_stride <= 0) {
        throw std::invalid_argument(
            "pgo.map_frame_stride must be positive");
    }
    requireFinitePositive(
        config.pgo.map_voxel_size,
        "pgo.map_voxel_size");
    if (config.pgo.occupancy.enabled) {
        requireFinitePositive(
            config.pgo.occupancy.resolution,
            "pgo.occupancy.resolution");
        requireFiniteNonnegative(
            config.pgo.occupancy.h_clearance,
            "pgo.occupancy.h_clearance");
        requireFiniteNonnegative(
            config.pgo.occupancy.d_max,
            "pgo.occupancy.d_max");
        if (config.pgo.occupancy.occ_threshold <= 0.0
            || config.pgo.occupancy.occ_threshold > 1.0
            || !std::isfinite(config.pgo.occupancy.occ_threshold)) {
            throw std::invalid_argument(
                "pgo.occupancy.occ_threshold must be in (0, 1]");
        }
        requireFinitePositive(
            config.pgo.occupancy.usable_range,
            "pgo.occupancy.usable_range");
        requireFiniteNonnegative(
            config.pgo.occupancy.min_range,
            "pgo.occupancy.min_range");
        if (config.pgo.occupancy.min_range
            >= config.pgo.occupancy.usable_range) {
            throw std::invalid_argument(
                "pgo.occupancy.min_range must be < usable_range");
        }
        requireFiniteNonnegative(
            config.pgo.occupancy.cloud_voxel_size,
            "pgo.occupancy.cloud_voxel_size");
        requireFinitePositive(
            config.pgo.occupancy.margin,
            "pgo.occupancy.margin");
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
