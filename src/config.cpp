#include <sapphire/config.hpp>

#include <toml++/toml.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace sapphire {
namespace {

struct ParseContext {
    std::unordered_set<std::string> known;
};

using NodeView = toml::node_view<toml::node>;

void remember(ParseContext& ctx, const char* path) {
    ctx.known.insert(path);
}

[[noreturn]] void throwFieldType(
    const toml::node& node, const char* path, const char* expected)
{
    std::ostringstream message;
    message << "TOML field " << path << " at line " << node.source().begin.line
            << " must be " << expected;
    throw std::runtime_error(message.str());
}

[[noreturn]] void throwUnknownKey(const toml::node& node, const std::string& path)
{
    std::ostringstream message;
    message << "unknown TOML key '" << path << "' at line "
            << node.source().begin.line;
    throw std::runtime_error(message.str());
}

[[noreturn]] void throwMissing(const char* path) {
    throw std::runtime_error(
        std::string("missing required TOML field: ") + path);
}

std::optional<double> optionalNumber(
    ParseContext& ctx, const NodeView& node, const char* path)
{
    remember(ctx, path);
    if (!node) {
        return std::nullopt;
    }
    if (!node.is_number()) {
        throwFieldType(*node.node(), path, "a number");
    }
    const auto value = node.value<double>();
    if (!value.has_value()) {
        throwFieldType(*node.node(), path, "a number");
    }
    return value;
}

double requiredNumber(
    ParseContext& ctx, const NodeView& node, const char* path)
{
    const auto value = optionalNumber(ctx, node, path);
    if (!value.has_value()) {
        throwMissing(path);
    }
    return *value;
}

std::optional<int> optionalInt(
    ParseContext& ctx, const NodeView& node, const char* path)
{
    remember(ctx, path);
    if (!node) {
        return std::nullopt;
    }
    if (!node.is_integer()) {
        throwFieldType(*node.node(), path, "an integer");
    }
    const auto value = node.value_exact<int64_t>();
    if (!value.has_value()
        || *value < static_cast<int64_t>(std::numeric_limits<int>::min())
        || *value > static_cast<int64_t>(std::numeric_limits<int>::max())) {
        throwFieldType(*node.node(), path, "an integer");
    }
    return static_cast<int>(*value);
}

std::optional<bool> optionalBool(
    ParseContext& ctx, const NodeView& node, const char* path)
{
    remember(ctx, path);
    if (!node) {
        return std::nullopt;
    }
    if (!node.is_boolean()) {
        throwFieldType(*node.node(), path, "a boolean");
    }
    const auto value = node.value_exact<bool>();
    if (!value.has_value()) {
        throwFieldType(*node.node(), path, "a boolean");
    }
    return value;
}

std::optional<std::string> optionalString(
    ParseContext& ctx, const NodeView& node, const char* path)
{
    remember(ctx, path);
    if (!node) {
        return std::nullopt;
    }
    if (!node.is_string()) {
        throwFieldType(*node.node(), path, "a string");
    }
    const auto value = node.value_exact<std::string>();
    if (!value.has_value()) {
        throwFieldType(*node.node(), path, "a string");
    }
    return value;
}

double readNumberElement(const toml::node& element, const char* path) {
    if (!element.is_number()) {
        throwFieldType(element, path, "a number");
    }
    const auto value = element.value<double>();
    if (!value.has_value()) {
        throwFieldType(element, path, "a number");
    }
    return *value;
}

const toml::array& requireNumericArray(
    ParseContext& ctx, const NodeView& node, const char* path, size_t length)
{
    remember(ctx, path);
    if (!node) {
        throwMissing(path);
    }
    if (!node.is_array() || node.as_array() == nullptr
        || node.as_array()->size() != length) {
        std::ostringstream message;
        message << "TOML field " << path << " at line "
                << node.node()->source().begin.line
                << " must be a numeric array of length " << length;
        throw std::runtime_error(message.str());
    }
    return *node.as_array();
}

Eigen::Vector3d requiredVector3(
    ParseContext& ctx, const NodeView& node, const char* path)
{
    const toml::array& array = requireNumericArray(ctx, node, path, 3);
    Eigen::Vector3d result;
    for (size_t i = 0; i < 3; ++i) {
        result[static_cast<Eigen::Index>(i)] =
            readNumberElement(array[i], path);
    }
    return result;
}

Eigen::Quaterniond requiredQuaternionXyzw(
    ParseContext& ctx, const NodeView& node, const char* path)
{
    const toml::array& array = requireNumericArray(ctx, node, path, 4);
    double values[4]{};
    for (size_t i = 0; i < 4; ++i) {
        values[i] = readNumberElement(array[i], path);
    }

    Eigen::Quaterniond quaternion(values[3], values[0], values[1], values[2]);
    if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1e-12) {
        std::ostringstream message;
        message << "TOML field " << path << " at line "
                << node.node()->source().begin.line
                << " is non-finite or has zero norm";
        throw std::runtime_error(message.str());
    }
    quaternion.normalize();
    return quaternion;
}

bool isKnownTable(const ParseContext& ctx, const std::string& path) {
    const std::string prefix = path + ".";
    for (const std::string& field : ctx.known) {
        if (field.compare(0, prefix.size(), prefix) == 0) {
            return true;
        }
    }
    return false;
}

void rejectUnknownKeys(
    const ParseContext& ctx,
    const toml::table& table,
    const std::string& prefix = {})
{
    for (auto&& [key, node] : table) {
        const std::string path = prefix.empty()
            ? std::string(key.str())
            : prefix + "." + std::string(key.str());
        if (node.is_table()) {
            if (!isKnownTable(ctx, path)) {
                throwUnknownKey(node, path);
            }
            rejectUnknownKeys(ctx, *node.as_table(), path);
            continue;
        }
        if (ctx.known.count(path) == 0) {
            throwUnknownKey(node, path);
        }
    }
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

void requireRelativeTrajectoryFile(const std::string& file) {
    const std::filesystem::path path(file);
    const bool has_parent_escape = [&path]() {
        for (const std::filesystem::path& part : path) {
            if (part == "..") {
                return true;
            }
        }
        return false;
    }();
    if (file.empty() || path.is_absolute() || !path.has_filename()
        || path.filename() == "." || path.filename() == ".."
        || has_parent_escape) {
        throw std::invalid_argument(
            "output.trajectory_file must be a relative path inside "
            "output.trajectory_dir");
    }
}

void requireChoice(
    const std::string& value,
    const char* name,
    std::initializer_list<const char*> allowed)
{
    for (const char* item : allowed) {
        if (value == item) {
            return;
        }
    }
    std::ostringstream message;
    message << name << " must be one of:";
    for (const char* item : allowed) {
        message << ' ' << item;
    }
    throw std::invalid_argument(message.str());
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
    ParseContext ctx;
    config.imu.init.gravity_mag = requiredNumber(
        ctx, root["imu"]["gravity"],
        "imu.gravity");
    if (const auto value =
            optionalNumber(
                ctx, root["imu"]["init"]["convergence_gyro_std"],
                "imu.init.convergence_gyro_std")) {
        config.imu.init.convergence_gyro_std = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["imu"]["init"]["convergence_accel_std"],
                "imu.init.convergence_accel_std")) {
        config.imu.init.convergence_accel_std = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["imu"]["init"]["timeout_sec"],
                "imu.init.timeout_sec")) {
        config.imu.init.timeout_sec = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["imu"]["init"]["check_interval_sec"],
                "imu.init.check_interval_sec")) {
        config.imu.init.check_interval_sec = *value;
    }
    if (const auto value =
            optionalInt(
                ctx, root["imu"]["init"]["min_samples"],
                "imu.init.min_samples")) {
        config.imu.init.min_samples = *value;
    }

    config.imu.noise.accel_noise_density = requiredNumber(
        ctx, root["imu"]["noise"]["accel_noise_density"],
        "imu.noise.accel_noise_density");
    config.imu.noise.gyro_noise_density = requiredNumber(
        ctx, root["imu"]["noise"]["gyro_noise_density"],
        "imu.noise.gyro_noise_density");
    config.imu.noise.accel_random_walk = requiredNumber(
        ctx, root["imu"]["noise"]["accel_random_walk"],
        "imu.noise.accel_random_walk");
    config.imu.noise.gyro_random_walk = requiredNumber(
        ctx, root["imu"]["noise"]["gyro_random_walk"],
        "imu.noise.gyro_random_walk");

    if (const auto voxel_size =
            optionalNumber(
                ctx, root["odometry"]["voxel_size"],
                "odometry.voxel_size")) {
        config.odometry.voxel_size = *voxel_size;
    }
    if (const auto fusion =
            optionalString(
                ctx, root["odometry"]["fusion"],
                "odometry.fusion")) {
        config.odometry.fusion = *fusion;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["crop_box"]["min_x"],
                "odometry.crop_box.min_x")) {
        config.odometry.crop_box.min_x = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["crop_box"]["min_y"],
                "odometry.crop_box.min_y")) {
        config.odometry.crop_box.min_y = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["crop_box"]["min_z"],
                "odometry.crop_box.min_z")) {
        config.odometry.crop_box.min_z = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["crop_box"]["max_x"],
                "odometry.crop_box.max_x")) {
        config.odometry.crop_box.max_x = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["crop_box"]["max_y"],
                "odometry.crop_box.max_y")) {
        config.odometry.crop_box.max_y = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["crop_box"]["max_z"],
                "odometry.crop_box.max_z")) {
        config.odometry.crop_box.max_z = *value;
    }
    if (const auto velocity_gain =
            optionalNumber(
                ctx, root["odometry"]["observer"]["velocity_gain"],
                "odometry.observer.velocity_gain")) {
        config.odometry.observer.velocity_gain = *velocity_gain;
    }
    if (const auto position_gain =
            optionalNumber(
                ctx, root["odometry"]["observer"]["position_gain"],
                "odometry.observer.position_gain")) {
        config.odometry.observer.position_gain = *position_gain;
    }
    if (const auto orientation_gain =
            optionalNumber(
                ctx, root["odometry"]["observer"]["orientation_gain"],
                "odometry.observer.orientation_gain")) {
        config.odometry.observer.orientation_gain = *orientation_gain;
    }
    if (const auto accel_bias_gain =
            optionalNumber(
                ctx, root["odometry"]["observer"]["accel_bias_gain"],
                "odometry.observer.accel_bias_gain")) {
        config.odometry.observer.accel_bias_gain = *accel_bias_gain;
    }
    if (const auto gyro_bias_gain =
            optionalNumber(
                ctx, root["odometry"]["observer"]["gyro_bias_gain"],
                "odometry.observer.gyro_bias_gain")) {
        config.odometry.observer.gyro_bias_gain = *gyro_bias_gain;
    }
    if (const auto accel_bias_max =
            optionalNumber(
                ctx, root["odometry"]["observer"]["accel_bias_max"],
                "odometry.observer.accel_bias_max")) {
        config.odometry.observer.accel_bias_max = *accel_bias_max;
    }
    if (const auto gyro_bias_max =
            optionalNumber(
                ctx, root["odometry"]["observer"]["gyro_bias_max"],
                "odometry.observer.gyro_bias_max")) {
        config.odometry.observer.gyro_bias_max = *gyro_bias_max;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["sigma_rotation"],
                "odometry.eskf.sigma_rotation")) {
        config.odometry.eskf.sigma_rotation = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["sigma_translation"],
                "odometry.eskf.sigma_translation")) {
        config.odometry.eskf.sigma_translation = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["icp_covariance_scale"],
                "odometry.eskf.icp_covariance_scale")) {
        config.odometry.eskf.icp_covariance_scale = *value;
    }
    if (const auto value =
            optionalBool(
                ctx, root["odometry"]["eskf"]["use_hessian"],
                "odometry.eskf.use_hessian")) {
        config.odometry.eskf.use_hessian = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["hessian_min_information"],
                "odometry.eskf.hessian_min_information")) {
        config.odometry.eskf.hessian_min_information = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["hessian_max_condition"],
                "odometry.eskf.hessian_max_condition")) {
        config.odometry.eskf.hessian_max_condition = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["hessian_degenerate_sigma"],
                "odometry.eskf.hessian_degenerate_sigma")) {
        config.odometry.eskf.hessian_degenerate_sigma = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["hessian_max_sigma"],
                "odometry.eskf.hessian_max_sigma")) {
        config.odometry.eskf.hessian_max_sigma = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["mahalanobis_threshold"],
                "odometry.eskf.mahalanobis_threshold")) {
        config.odometry.eskf.mahalanobis_threshold = *value;
    }
    if (const auto value =
            optionalBool(
                ctx, root["odometry"]["eskf"]["inject_full_pose"],
                "odometry.eskf.inject_full_pose")) {
        config.odometry.eskf.inject_full_pose = *value;
    }
    if (const auto value =
            optionalBool(
                ctx, root["odometry"]["eskf"]["inject_directional_pose"],
                "odometry.eskf.inject_directional_pose")) {
        config.odometry.eskf.inject_directional_pose = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["bias_update_scale"],
                "odometry.eskf.bias_update_scale")) {
        config.odometry.eskf.bias_update_scale = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["velocity_correction_gain"],
                "odometry.eskf.velocity_correction_gain")) {
        config.odometry.eskf.velocity_correction_gain = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["accel_bias_max"],
                "odometry.eskf.accel_bias_max")) {
        config.odometry.eskf.accel_bias_max = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["gyro_bias_max"],
                "odometry.eskf.gyro_bias_max")) {
        config.odometry.eskf.gyro_bias_max = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["init_sigma_theta"],
                "odometry.eskf.init_sigma_theta")) {
        config.odometry.eskf.init_sigma_theta = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["init_sigma_velocity"],
                "odometry.eskf.init_sigma_velocity")) {
        config.odometry.eskf.init_sigma_velocity = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["odometry"]["eskf"]["init_sigma_position"],
                "odometry.eskf.init_sigma_position")) {
        config.odometry.eskf.init_sigma_position = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["imu"]["noise"]["accel_bias_rw_sigma"],
                "imu.noise.accel_bias_rw_sigma")) {
        config.imu.noise.accel_bias_rw_sigma = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["imu"]["noise"]["gyro_bias_rw_sigma"],
                "imu.noise.gyro_bias_rw_sigma")) {
        config.imu.noise.gyro_bias_rw_sigma = *value;
    }
    if (const auto splitting_distance =
            optionalNumber(
                ctx, root["odometry"]["submap"]["splitting_distance"],
                "odometry.submap.splitting_distance")) {
        config.odometry.submap.splitting_distance = *splitting_distance;
    }
    if (const auto splitting_rotation =
            optionalNumber(
                ctx, root["odometry"]["submap"]["splitting_rotation"],
                "odometry.submap.splitting_rotation")) {
        config.odometry.submap.splitting_rotation = *splitting_rotation;
    }
    if (const auto max_keyframes =
            optionalInt(
                ctx, root["odometry"]["submap"]["max_keyframes"],
                "odometry.submap.max_keyframes")) {
        config.odometry.submap.max_keyframes = *max_keyframes;
    }
    if (const auto voxel_size =
            optionalNumber(
                ctx, root["odometry"]["submap"]["voxel_size"],
                "odometry.submap.voxel_size")) {
        config.odometry.submap.voxel_size = *voxel_size;
    }

    if (const auto value = optionalString(
        ctx, root["registration"]["type"],
        "registration.type")) {
        config.registration.type = *value;
    }
    if (const auto value =
            optionalInt(
                ctx, root["registration"]["gicp"]["max_iterations"],
                "registration.gicp.max_iterations")) {
        config.registration.gicp.max_iterations = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["registration"]["gicp"]["transformation_epsilon"],
                "registration.gicp.transformation_epsilon")) {
        config.registration.gicp.transformation_epsilon = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["registration"]["gicp"]["rotation_epsilon"],
                "registration.gicp.rotation_epsilon")) {
        config.registration.gicp.rotation_epsilon = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["registration"]["gicp"]["max_correspondence_dist"],
                "registration.gicp.max_correspondence_dist")) {
        config.registration.gicp.max_correspondence_dist = *value;
    }
    if (const auto value =
            optionalInt(
                ctx, root["registration"]["gicp"]["k_correspondences"],
                "registration.gicp.k_correspondences")) {
        config.registration.gicp.k_correspondences = *value;
    }
    if (const auto value =
            optionalInt(
                ctx, root["registration"]["gicp"]["min_num_points"],
                "registration.gicp.min_num_points")) {
        config.registration.gicp.min_num_points = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["registration"]["gicp"]["max_correction_trans"],
                "registration.gicp.max_correction_trans")) {
        config.registration.gicp.max_correction_trans = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["registration"]["gicp"]["max_correction_rot_deg"],
                "registration.gicp.max_correction_rot_deg")) {
        config.registration.gicp.max_correction_rot_deg = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["registration"]["vgicp"]["voxel_resolution"],
                "registration.vgicp.voxel_resolution")) {
        config.registration.vgicp.voxel_resolution = *value;
    }
    if (const auto value = optionalBool(
        ctx, root["pgo"]["enabled"],
        "pgo.enabled")) {
        config.pgo.enabled = *value;
    }
    if (const auto value = optionalNumber(
        ctx, root["pgo"]["keyframe_distance"],
        "pgo.keyframe_distance")) {
        config.pgo.keyframe_distance = *value;
    }
    if (const auto value = optionalNumber(
        ctx, root["pgo"]["keyframe_rotation"],
        "pgo.keyframe_rotation")) {
        config.pgo.keyframe_rotation = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["loop_min_time_separation"],
                "pgo.loop_min_time_separation")) {
        config.pgo.loop_min_time_separation = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["loop_min_travel_distance"],
                "pgo.loop_min_travel_distance")) {
        config.pgo.loop_min_travel_distance = *value;
    }
    if (const auto value = optionalNumber(
        ctx, root["pgo"]["loop_max_rotation"],
        "pgo.loop_max_rotation")) {
        config.pgo.loop_max_rotation = *value;
    }
    if (const auto value = optionalNumber(
        ctx, root["pgo"]["loop_search_radius"],
        "pgo.loop_search_radius")) {
        config.pgo.loop_search_radius = *value;
    }
    if (const auto value = optionalInt(
        ctx, root["pgo"]["loop_search_stride"],
        "pgo.loop_search_stride")) {
        config.pgo.loop_search_stride = *value;
    }
    if (const auto value = optionalInt(
        ctx, root["pgo"]["target_frame_count"],
        "pgo.target_frame_count")) {
        config.pgo.target_frame_count = *value;
    }
    if (const auto value = optionalNumber(
        ctx, root["pgo"]["source_voxel_size"],
        "pgo.source_voxel_size")) {
        config.pgo.source_voxel_size = *value;
    }
    if (const auto value = optionalNumber(
        ctx, root["pgo"]["target_voxel_size"],
        "pgo.target_voxel_size")) {
        config.pgo.target_voxel_size = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["source_travel_radius_m"],
                "pgo.source_travel_radius_m")) {
        config.pgo.source_travel_radius_m = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["source_max_range_m"],
                "pgo.source_max_range_m")) {
        config.pgo.source_max_range_m = *value;
    }
    if (const auto value = optionalInt(
        ctx, root["pgo"]["source_max_frames"],
        "pgo.source_max_frames")) {
        config.pgo.source_max_frames = *value;
    }
    if (const auto value = optionalInt(
        ctx, root["pgo"]["lidar_min_inliers"],
        "pgo.lidar_min_inliers")) {
        config.pgo.lidar_min_inliers = *value;
    }
    if (const auto value = optionalInt(
        ctx, root["pgo"]["lidar_max_iterations"],
        "pgo.lidar_max_iterations")) {
        config.pgo.lidar_max_iterations = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["lidar_max_correspondence_m"],
                "pgo.lidar_max_correspondence_m")) {
        config.pgo.lidar_max_correspondence_m = *value;
    }
    if (const auto value = optionalNumber(
        ctx, root["pgo"]["lidar_min_overlap"],
        "pgo.lidar_min_overlap")) {
        config.pgo.lidar_min_overlap = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["lidar_max_fitness_m2"],
                "pgo.lidar_max_fitness_m2")) {
        config.pgo.lidar_max_fitness_m2 = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["lidar_translation_sigma_m"],
                "pgo.lidar_translation_sigma_m")) {
        config.pgo.lidar_translation_sigma_m = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["lidar_rotation_sigma_rad"],
                "pgo.lidar_rotation_sigma_rad")) {
        config.pgo.lidar_rotation_sigma_rad = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["max_constraint_error_sigma"],
                "pgo.max_constraint_error_sigma")) {
        config.pgo.max_constraint_error_sigma = *value;
    }
    if (const auto value = optionalNumber(
        ctx, root["pgo"]["update_period_sec"],
        "pgo.update_period_sec")) {
        config.pgo.update_period_sec = *value;
    }
    if (const auto value = optionalInt(
        ctx, root["pgo"]["map_frame_stride"],
        "pgo.map_frame_stride")) {
        config.pgo.map_frame_stride = *value;
    }
    if (const auto value = optionalNumber(
        ctx, root["pgo"]["map_voxel_size"],
        "pgo.map_voxel_size")) {
        config.pgo.map_voxel_size = *value;
    }
    if (const auto value = optionalBool(
        ctx, root["pgo"]["occupancy"]["enabled"],
        "pgo.occupancy.enabled")) {
        config.pgo.occupancy.enabled = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["occupancy"]["resolution"],
                "pgo.occupancy.resolution")) {
        config.pgo.occupancy.resolution = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["occupancy"]["h_clearance"],
                "pgo.occupancy.h_clearance")) {
        config.pgo.occupancy.h_clearance = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["occupancy"]["ground_margin"],
                "pgo.occupancy.ground_margin")) {
        config.pgo.occupancy.ground_margin = *value;
    }
    if (const auto value = optionalNumber(
        ctx, root["pgo"]["occupancy"]["d_max"],
        "pgo.occupancy.d_max")) {
        config.pgo.occupancy.d_max = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["occupancy"]["clear_height_eps"],
                "pgo.occupancy.clear_height_eps")) {
        config.pgo.occupancy.clear_height_eps = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["occupancy"]["occ_threshold"],
                "pgo.occupancy.occ_threshold")) {
        config.pgo.occupancy.occ_threshold = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["occupancy"]["usable_range"],
                "pgo.occupancy.usable_range")) {
        config.pgo.occupancy.usable_range = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["occupancy"]["min_range"],
                "pgo.occupancy.min_range")) {
        config.pgo.occupancy.min_range = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["occupancy"]["cloud_voxel_size"],
                "pgo.occupancy.cloud_voxel_size")) {
        config.pgo.occupancy.cloud_voxel_size = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["pgo"]["occupancy"]["margin"],
                "pgo.occupancy.margin")) {
        config.pgo.occupancy.margin = *value;
    }

    if (const auto value =
            optionalString(
                ctx, root["sensor"]["lidar"]["time_field"],
                "sensor.lidar.time_field")) {
        config.sensor.lidar.time_field = *value;
    }
    if (const auto value =
            optionalString(
                ctx, root["sensor"]["lidar"]["time_unit"],
                "sensor.lidar.time_unit")) {
        config.sensor.lidar.time_unit = *value;
    }
    if (const auto value =
            optionalString(
                ctx, root["sensor"]["lidar"]["time_ref"],
                "sensor.lidar.time_ref")) {
        config.sensor.lidar.time_ref = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["sensor"]["lidar"]["max_scan_duration"],
                "sensor.lidar.max_scan_duration")) {
        config.sensor.lidar.max_scan_duration = *value;
    }
    if (const auto value =
            optionalNumber(
                ctx, root["sensor"]["lidar"]["header_tolerance"],
                "sensor.lidar.header_tolerance")) {
        config.sensor.lidar.header_tolerance = *value;
    }
    if (const auto value =
            optionalBool(
                ctx, root["sensor"]["lidar"]["require_point_time"],
                "sensor.lidar.require_point_time")) {
        config.sensor.lidar.require_point_time = *value;
    }
    if (const auto value =
            optionalString(
                ctx, root["sensor"]["imu"]["accel_unit"],
                "sensor.imu.accel_unit")) {
        config.sensor.imu.accel_unit = *value;
    }
    if (const auto value =
            optionalString(
                ctx, root["output"]["trajectory_dir"],
                "output.trajectory_dir")) {
        config.output.trajectory_dir = *value;
    }
    if (const auto value =
            optionalString(
                ctx, root["output"]["trajectory_file"],
                "output.trajectory_file")) {
        config.output.trajectory_file = *value;
    }

    const auto extrinsics = root["extrinsics"]["T_imu_lidar"];
    const Eigen::Vector3d translation = requiredVector3(
        ctx, extrinsics["translation"],
        "extrinsics.T_imu_lidar.translation");
    const Eigen::Quaterniond rotation = requiredQuaternionXyzw(
        ctx, extrinsics["rotation_xyzw"],
        "extrinsics.T_imu_lidar.rotation_xyzw");
    config.extrinsics.T_imu_lidar = Isometry3d::Identity();
    config.extrinsics.T_imu_lidar.linear() = rotation.toRotationMatrix();
    config.extrinsics.T_imu_lidar.translation() = translation;

    rejectUnknownKeys(ctx, root);
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
    requireFiniteNonnegative(
        config.pgo.source_travel_radius_m,
        "pgo.source_travel_radius_m");
    requireFinitePositive(
        config.pgo.source_max_range_m,
        "pgo.source_max_range_m");
    if (config.pgo.source_max_frames < 1) {
        throw std::invalid_argument(
            "pgo.source_max_frames must be positive");
    }
    if (config.pgo.lidar_min_inliers < 6
        || config.pgo.lidar_max_iterations < 1) {
        throw std::invalid_argument(
            "pgo.lidar_min_inliers must be >= 6 and "
            "lidar_max_iterations must be positive");
    }
    requireFinitePositive(
        config.pgo.lidar_max_correspondence_m,
        "pgo.lidar_max_correspondence_m");
    if (!std::isfinite(config.pgo.lidar_min_overlap)
        || config.pgo.lidar_min_overlap <= 0.0
        || config.pgo.lidar_min_overlap > 1.0) {
        throw std::invalid_argument(
            "pgo.lidar_min_overlap must be in (0, 1]");
    }
    requireFinitePositive(
        config.pgo.lidar_max_fitness_m2,
        "pgo.lidar_max_fitness_m2");
    requireFinitePositive(
        config.pgo.lidar_translation_sigma_m,
        "pgo.lidar_translation_sigma_m");
    requireFinitePositive(
        config.pgo.lidar_rotation_sigma_rad,
        "pgo.lidar_rotation_sigma_rad");
    requireFinitePositive(
        config.pgo.max_constraint_error_sigma,
        "pgo.max_constraint_error_sigma");
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
            config.pgo.occupancy.ground_margin,
            "pgo.occupancy.ground_margin");
        if (config.pgo.occupancy.ground_margin
            >= config.pgo.occupancy.h_clearance
            && config.pgo.occupancy.h_clearance > 0.0) {
            throw std::invalid_argument(
                "pgo.occupancy.ground_margin must be < h_clearance");
        }
        requireFiniteNonnegative(
            config.pgo.occupancy.d_max,
            "pgo.occupancy.d_max");
        requireFiniteNonnegative(
            config.pgo.occupancy.clear_height_eps,
            "pgo.occupancy.clear_height_eps");
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

    requireChoice(
        config.sensor.lidar.time_field,
        "sensor.lidar.time_field",
        {"auto", "none", "offset_time", "t", "timestamp", "time", "time_stamp"});
    requireChoice(
        config.sensor.lidar.time_unit,
        "sensor.lidar.time_unit",
        {"auto", "s", "ms", "us", "ns"});
    requireChoice(
        config.sensor.lidar.time_ref,
        "sensor.lidar.time_ref",
        {"auto", "relative", "absolute"});
    requireFinitePositive(
        config.sensor.lidar.max_scan_duration,
        "sensor.lidar.max_scan_duration");
    requireFiniteNonnegative(
        config.sensor.lidar.header_tolerance,
        "sensor.lidar.header_tolerance");
    requireChoice(
        config.sensor.imu.accel_unit,
        "sensor.imu.accel_unit",
        {"auto", "g", "m_s2"});
    requireRelativeTrajectoryFile(config.output.trajectory_file);
    if (!config.pgo.enabled && !trajectoryOutputPath(config).empty()) {
        throw std::invalid_argument(
            "output.trajectory_dir is set but pgo.enabled is false; "
            "the TUM dump is the pose-graph map trajectory and stays empty "
            "unless PGO is enabled");
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

std::filesystem::path trajectoryOutputPath(const Config& config) {
    if (config.output.trajectory_dir.empty()) {
        return {};
    }
    return std::filesystem::path(config.output.trajectory_dir)
        / config.output.trajectory_file;
}

}  // namespace sapphire
