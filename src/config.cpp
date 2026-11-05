#include <sapphire/config.hpp>

#include <toml++/toml.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
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

bool isAllowedChoice(const std::string& value, const char* const* allowed) {
    for (const char* const* item = allowed; *item != nullptr; ++item) {
        if (value == *item) {
            return true;
        }
    }
    return false;
}

[[noreturn]] void throwChoice(const char* name, const char* const* allowed) {
    std::ostringstream message;
    message << name << " must be one of:";
    for (const char* const* item = allowed; *item != nullptr; ++item) {
        message << ' ' << *item;
    }
    throw std::invalid_argument(message.str());
}

// Scalar schema. Each TOML key is one row: the loader assigns it, and the
// same row carries the range or choice check. Cross-field rules stay in
// validateConfig because one message covers more than one key.
enum class NumberRule {
    None,
    Positive,
    Nonnegative,
    UnitInterval,
};

enum class IntRule {
    None,
    Positive,
};

enum class Gate {
    Always,
    Occupancy,
};

struct NumberField {
    const char* path;
    void (*set)(Config&, double);
    double (*get)(const Config&);
    bool required;
    NumberRule rule;
    Gate gate;
};

struct IntField {
    const char* path;
    void (*set)(Config&, int);
    int (*get)(const Config&);
    IntRule rule;
};

struct BoolField {
    const char* path;
    void (*set)(Config&, bool);
};

struct StringField {
    const char* path;
    void (*set)(Config&, const std::string&);
    const std::string& (*get)(const Config&);
    const char* const* choices;
    const char* reject_message;
};

#define FIELD(path, member, required, rule, gate) \
    NumberField{ \
        path, \
        [](Config& config, double value) { config.member = value; }, \
        [](const Config& config) { return config.member; }, \
        required, \
        rule, \
        gate \
    }

#define OPT(path, member, rule) \
    FIELD(path, member, false, rule, Gate::Always)
#define REQ(path, member, rule) \
    FIELD(path, member, true, rule, Gate::Always)
#define WHEN_OCC(path, member, rule) \
    FIELD(path, member, false, rule, Gate::Occupancy)

#define INT(path, member, rule) \
    IntField{ \
        path, \
        [](Config& config, int value) { config.member = value; }, \
        [](const Config& config) { return config.member; }, \
        rule \
    }

#define BOOL_FIELD(path, member) \
    BoolField{ \
        path, \
        [](Config& config, bool value) { config.member = value; } \
    }

#define STR(path, member, choices, reject_message) \
    StringField{ \
        path, \
        [](Config& config, const std::string& value) { config.member = value; }, \
        [](const Config& config) -> const std::string& { return config.member; }, \
        choices, \
        reject_message \
    }

constexpr const char* const kFusionChoices[] = {
    "observer", "eskf", nullptr};
constexpr const char* const kRegistrationChoices[] = {
    "GICP", "VGICP", nullptr};
constexpr const char* const kAccelUnitChoices[] = {
    "auto", "g", "m_s2", nullptr};

const NumberField kNumbers[] = {
    REQ("imu.gravity", imu.init.gravity_mag, NumberRule::Positive),
    OPT("imu.init.convergence_gyro_std",
        imu.init.convergence_gyro_std, NumberRule::Positive),
    OPT("imu.init.convergence_accel_std",
        imu.init.convergence_accel_std, NumberRule::Positive),
    OPT("imu.init.timeout_sec", imu.init.timeout_sec, NumberRule::Positive),
    OPT("imu.init.check_interval_sec",
        imu.init.check_interval_sec, NumberRule::Positive),
    REQ("imu.noise.accel_noise_density",
        imu.noise.accel_noise_density, NumberRule::Positive),
    REQ("imu.noise.gyro_noise_density",
        imu.noise.gyro_noise_density, NumberRule::Positive),
    REQ("imu.noise.accel_random_walk",
        imu.noise.accel_random_walk, NumberRule::Positive),
    REQ("imu.noise.gyro_random_walk",
        imu.noise.gyro_random_walk, NumberRule::Positive),
    // Zero is the stationary-bias default. Negative values stay legal.
    OPT("imu.noise.accel_bias_rw_sigma",
        imu.noise.accel_bias_rw_sigma, NumberRule::None),
    OPT("imu.noise.gyro_bias_rw_sigma",
        imu.noise.gyro_bias_rw_sigma, NumberRule::None),

    OPT("odometry.voxel_size", odometry.voxel_size, NumberRule::Positive),
    // Finite and ordered as a group in validateConfig.
    OPT("odometry.crop_box.min_x", odometry.crop_box.min_x, NumberRule::None),
    OPT("odometry.crop_box.min_y", odometry.crop_box.min_y, NumberRule::None),
    OPT("odometry.crop_box.min_z", odometry.crop_box.min_z, NumberRule::None),
    OPT("odometry.crop_box.max_x", odometry.crop_box.max_x, NumberRule::None),
    OPT("odometry.crop_box.max_y", odometry.crop_box.max_y, NumberRule::None),
    OPT("odometry.crop_box.max_z", odometry.crop_box.max_z, NumberRule::None),
    OPT("odometry.submap.splitting_distance",
        odometry.submap.splitting_distance, NumberRule::Positive),
    OPT("odometry.submap.splitting_rotation",
        odometry.submap.splitting_rotation, NumberRule::Positive),
    OPT("odometry.submap.voxel_size",
        odometry.submap.voxel_size, NumberRule::Positive),
    OPT("odometry.observer.position_gain",
        odometry.observer.position_gain, NumberRule::Nonnegative),
    OPT("odometry.observer.velocity_gain",
        odometry.observer.velocity_gain, NumberRule::Nonnegative),
    OPT("odometry.observer.orientation_gain",
        odometry.observer.orientation_gain, NumberRule::Nonnegative),
    OPT("odometry.observer.accel_bias_gain",
        odometry.observer.accel_bias_gain, NumberRule::Nonnegative),
    OPT("odometry.observer.gyro_bias_gain",
        odometry.observer.gyro_bias_gain, NumberRule::Nonnegative),
    OPT("odometry.observer.accel_bias_max",
        odometry.observer.accel_bias_max, NumberRule::Nonnegative),
    OPT("odometry.observer.gyro_bias_max",
        odometry.observer.gyro_bias_max, NumberRule::Nonnegative),
    OPT("odometry.eskf.sigma_rotation",
        odometry.eskf.sigma_rotation, NumberRule::Positive),
    OPT("odometry.eskf.sigma_translation",
        odometry.eskf.sigma_translation, NumberRule::Positive),
    OPT("odometry.eskf.icp_covariance_scale",
        odometry.eskf.icp_covariance_scale, NumberRule::Positive),
    OPT("odometry.eskf.hessian_min_information",
        odometry.eskf.hessian_min_information, NumberRule::Positive),
    OPT("odometry.eskf.hessian_max_condition",
        odometry.eskf.hessian_max_condition, NumberRule::Positive),
    OPT("odometry.eskf.hessian_degenerate_sigma",
        odometry.eskf.hessian_degenerate_sigma, NumberRule::Positive),
    OPT("odometry.eskf.hessian_max_sigma",
        odometry.eskf.hessian_max_sigma, NumberRule::Positive),
    // Finite, including negatives, which disable the gate.
    OPT("odometry.eskf.mahalanobis_threshold",
        odometry.eskf.mahalanobis_threshold, NumberRule::None),
    OPT("odometry.eskf.bias_update_scale",
        odometry.eskf.bias_update_scale, NumberRule::Nonnegative),
    OPT("odometry.eskf.velocity_correction_gain",
        odometry.eskf.velocity_correction_gain, NumberRule::Nonnegative),
    OPT("odometry.eskf.accel_bias_max",
        odometry.eskf.accel_bias_max, NumberRule::Nonnegative),
    OPT("odometry.eskf.gyro_bias_max",
        odometry.eskf.gyro_bias_max, NumberRule::Nonnegative),
    OPT("odometry.eskf.init_sigma_theta",
        odometry.eskf.init_sigma_theta, NumberRule::Positive),
    OPT("odometry.eskf.init_sigma_velocity",
        odometry.eskf.init_sigma_velocity, NumberRule::Positive),
    OPT("odometry.eskf.init_sigma_position",
        odometry.eskf.init_sigma_position, NumberRule::Positive),

    OPT("registration.gicp.transformation_epsilon",
        registration.gicp.transformation_epsilon, NumberRule::Positive),
    OPT("registration.gicp.rotation_epsilon",
        registration.gicp.rotation_epsilon, NumberRule::Positive),
    OPT("registration.gicp.max_correspondence_dist",
        registration.gicp.max_correspondence_dist, NumberRule::Positive),
    OPT("registration.gicp.max_correction_trans",
        registration.gicp.max_correction_trans, NumberRule::Positive),
    OPT("registration.gicp.max_correction_rot_deg",
        registration.gicp.max_correction_rot_deg, NumberRule::Positive),
    OPT("registration.vgicp.voxel_resolution",
        registration.vgicp.voxel_resolution, NumberRule::Positive),

    OPT("pgo.keyframe_distance", pgo.keyframe_distance, NumberRule::Positive),
    OPT("pgo.keyframe_rotation", pgo.keyframe_rotation, NumberRule::Positive),
    OPT("pgo.loop_min_time_separation",
        pgo.loop_min_time_separation, NumberRule::Nonnegative),
    OPT("pgo.loop_min_travel_distance",
        pgo.loop_min_travel_distance, NumberRule::Nonnegative),
    OPT("pgo.loop_max_rotation", pgo.loop_max_rotation, NumberRule::Positive),
    OPT("pgo.loop_search_radius", pgo.loop_search_radius, NumberRule::Positive),
    OPT("pgo.source_voxel_size", pgo.source_voxel_size, NumberRule::Positive),
    OPT("pgo.target_voxel_size", pgo.target_voxel_size, NumberRule::Positive),
    OPT("pgo.source_travel_radius_m",
        pgo.source_travel_radius_m, NumberRule::Nonnegative),
    OPT("pgo.source_max_range_m", pgo.source_max_range_m, NumberRule::Positive),
    OPT("pgo.lidar_max_correspondence_m",
        pgo.lidar_max_correspondence_m, NumberRule::Positive),
    OPT("pgo.lidar_min_overlap", pgo.lidar_min_overlap, NumberRule::UnitInterval),
    OPT("pgo.lidar_max_fitness_m2",
        pgo.lidar_max_fitness_m2, NumberRule::Positive),
    OPT("pgo.lidar_translation_sigma_m",
        pgo.lidar_translation_sigma_m, NumberRule::Positive),
    OPT("pgo.lidar_rotation_sigma_rad",
        pgo.lidar_rotation_sigma_rad, NumberRule::Positive),
    OPT("pgo.max_constraint_error_sigma",
        pgo.max_constraint_error_sigma, NumberRule::Positive),
    OPT("pgo.update_period_sec", pgo.update_period_sec, NumberRule::Positive),
    OPT("pgo.map_voxel_size", pgo.map_voxel_size, NumberRule::Positive),
    WHEN_OCC("pgo.occupancy.resolution",
        pgo.occupancy.resolution, NumberRule::Positive),
    WHEN_OCC("pgo.occupancy.h_clearance",
        pgo.occupancy.h_clearance, NumberRule::Nonnegative),
    WHEN_OCC("pgo.occupancy.ground_margin",
        pgo.occupancy.ground_margin, NumberRule::Nonnegative),
    WHEN_OCC("pgo.occupancy.d_max",
        pgo.occupancy.d_max, NumberRule::Nonnegative),
    WHEN_OCC("pgo.occupancy.clear_height_eps",
        pgo.occupancy.clear_height_eps, NumberRule::Nonnegative),
    WHEN_OCC("pgo.occupancy.occ_threshold",
        pgo.occupancy.occ_threshold, NumberRule::UnitInterval),
    WHEN_OCC("pgo.occupancy.usable_range",
        pgo.occupancy.usable_range, NumberRule::Positive),
    WHEN_OCC("pgo.occupancy.min_range",
        pgo.occupancy.min_range, NumberRule::Nonnegative),
    WHEN_OCC("pgo.occupancy.cloud_voxel_size",
        pgo.occupancy.cloud_voxel_size, NumberRule::Nonnegative),
    WHEN_OCC("pgo.occupancy.margin",
        pgo.occupancy.margin, NumberRule::Positive),

    OPT("sensor.lidar.max_scan_duration",
        sensor.lidar.max_scan_duration, NumberRule::Positive),
    OPT("sensor.lidar.header_tolerance",
        sensor.lidar.header_tolerance, NumberRule::Nonnegative),
};

const IntField kInts[] = {
    INT("imu.init.min_samples", imu.init.min_samples, IntRule::Positive),
    INT("odometry.submap.max_keyframes",
        odometry.submap.max_keyframes, IntRule::Positive),
    // None: one message covers the whole integer group.
    INT("registration.gicp.max_iterations",
        registration.gicp.max_iterations, IntRule::None),
    INT("registration.gicp.k_correspondences",
        registration.gicp.k_correspondences, IntRule::None),
    INT("registration.gicp.min_num_points",
        registration.gicp.min_num_points, IntRule::None),
    INT("pgo.loop_search_stride", pgo.loop_search_stride, IntRule::None),
    INT("pgo.target_frame_count", pgo.target_frame_count, IntRule::None),
    INT("pgo.source_max_frames", pgo.source_max_frames, IntRule::Positive),
    INT("pgo.lidar_min_inliers", pgo.lidar_min_inliers, IntRule::None),
    INT("pgo.lidar_max_iterations", pgo.lidar_max_iterations, IntRule::None),
    INT("pgo.map_frame_stride", pgo.map_frame_stride, IntRule::Positive),
};

const BoolField kBools[] = {
    BOOL_FIELD("odometry.eskf.use_hessian", odometry.eskf.use_hessian),
    BOOL_FIELD("odometry.eskf.inject_full_pose", odometry.eskf.inject_full_pose),
    BOOL_FIELD("odometry.eskf.inject_directional_pose",
        odometry.eskf.inject_directional_pose),
    BOOL_FIELD("pgo.enabled", pgo.enabled),
    BOOL_FIELD("pgo.occupancy.enabled", pgo.occupancy.enabled),
    BOOL_FIELD("sensor.lidar.require_point_time",
        sensor.lidar.require_point_time),
};

const StringField kStrings[] = {
    STR("odometry.fusion", odometry.fusion, kFusionChoices,
        "odometry.fusion must be \"observer\" or \"eskf\""),
    STR("registration.type", registration.type, kRegistrationChoices,
        "registration.type must be GICP or VGICP"),
    STR("sensor.lidar.time_field", sensor.lidar.time_field,
        sensor::timeFieldChoices(), nullptr),
    STR("sensor.lidar.time_unit", sensor.lidar.time_unit,
        sensor::timeUnitChoices(), nullptr),
    STR("sensor.lidar.time_ref", sensor.lidar.time_ref,
        sensor::timeRefChoices(), nullptr),
    STR("sensor.imu.accel_unit", sensor.imu.accel_unit,
        kAccelUnitChoices, nullptr),
    STR("output.trajectory_dir", output.trajectory_dir, nullptr, nullptr),
    STR("output.trajectory_file", output.trajectory_file, nullptr, nullptr),
};

#undef FIELD
#undef OPT
#undef REQ
#undef WHEN_OCC
#undef INT
#undef BOOL_FIELD
#undef STR

void applyNumberRule(const NumberField& field, double value) {
    switch (field.rule) {
    case NumberRule::None:
        break;
    case NumberRule::Positive:
        requireFinitePositive(value, field.path);
        break;
    case NumberRule::Nonnegative:
        requireFiniteNonnegative(value, field.path);
        break;
    case NumberRule::UnitInterval:
        if (!std::isfinite(value) || value <= 0.0 || value > 1.0) {
            throw std::invalid_argument(
                std::string(field.path) + " must be in (0, 1]");
        }
        break;
    }
}

void loadSchema(ParseContext& ctx, toml::table& root, Config& config) {
    for (const NumberField& field : kNumbers) {
        const NodeView node = root.at_path(field.path);
        if (field.required) {
            field.set(config, requiredNumber(ctx, node, field.path));
        } else if (const auto value = optionalNumber(ctx, node, field.path)) {
            field.set(config, *value);
        }
    }
    for (const IntField& field : kInts) {
        const NodeView node = root.at_path(field.path);
        if (const auto value = optionalInt(ctx, node, field.path)) {
            field.set(config, *value);
        }
    }
    for (const BoolField& field : kBools) {
        const NodeView node = root.at_path(field.path);
        if (const auto value = optionalBool(ctx, node, field.path)) {
            field.set(config, *value);
        }
    }
    for (const StringField& field : kStrings) {
        const NodeView node = root.at_path(field.path);
        if (const auto value = optionalString(ctx, node, field.path)) {
            field.set(config, *value);
        }
    }

    const NodeView extrinsics = root.at_path("extrinsics.T_imu_lidar");
    const Eigen::Vector3d translation = requiredVector3(
        ctx, extrinsics["translation"],
        "extrinsics.T_imu_lidar.translation");
    const Eigen::Quaterniond rotation = requiredQuaternionXyzw(
        ctx, extrinsics["rotation_xyzw"],
        "extrinsics.T_imu_lidar.rotation_xyzw");
    config.extrinsics.T_imu_lidar = Isometry3d::Identity();
    config.extrinsics.T_imu_lidar.linear() = rotation.toRotationMatrix();
    config.extrinsics.T_imu_lidar.translation() = translation;
}

void validateSchema(const Config& config) {
    const bool occupancy_enabled = config.pgo.occupancy.enabled;
    for (const NumberField& field : kNumbers) {
        if (field.gate == Gate::Occupancy && !occupancy_enabled) {
            continue;
        }
        applyNumberRule(field, field.get(config));
    }
    for (const IntField& field : kInts) {
        if (field.rule == IntRule::Positive && field.get(config) <= 0) {
            throw std::invalid_argument(
                std::string(field.path) + " must be positive");
        }
    }
    for (const StringField& field : kStrings) {
        if (field.choices == nullptr
            || isAllowedChoice(field.get(config), field.choices)) {
            continue;
        }
        if (field.reject_message != nullptr) {
            throw std::invalid_argument(field.reject_message);
        }
        throwChoice(field.path, field.choices);
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
    ParseContext ctx;
    loadSchema(ctx, root, config);
    rejectUnknownKeys(ctx, root);
    validateConfig(config);
    return config;
}

void validateConfig(const Config& config) {
    validateSchema(config);

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
    const auto& crop = config.odometry.crop_box;
    if (!std::isfinite(crop.min_x) || !std::isfinite(crop.min_y)
        || !std::isfinite(crop.min_z) || !std::isfinite(crop.max_x)
        || !std::isfinite(crop.max_y) || !std::isfinite(crop.max_z)
        || crop.min_x >= crop.max_x || crop.min_y >= crop.max_y
        || crop.min_z >= crop.max_z) {
        throw std::invalid_argument(
            "odometry.crop_box bounds must be finite and ordered");
    }

    const auto& gicp = config.registration.gicp;
    if (gicp.max_iterations <= 0 || gicp.k_correspondences <= 0
        || gicp.min_num_points <= 0) {
        throw std::invalid_argument(
            "registration.gicp integer limits must be positive");
    }
    if (config.pgo.loop_search_stride <= 0
        || config.pgo.target_frame_count < 0) {
        throw std::invalid_argument(
            "pgo loop_search_stride must be positive and "
            "target_frame_count must be nonnegative");
    }
    if (config.pgo.lidar_min_inliers < 6
        || config.pgo.lidar_max_iterations < 1) {
        throw std::invalid_argument(
            "pgo.lidar_min_inliers must be >= 6 and "
            "lidar_max_iterations must be positive");
    }
    if (config.pgo.occupancy.enabled) {
        if (config.pgo.occupancy.ground_margin
            >= config.pgo.occupancy.h_clearance
            && config.pgo.occupancy.h_clearance > 0.0) {
            throw std::invalid_argument(
                "pgo.occupancy.ground_margin must be < h_clearance");
        }
        if (config.pgo.occupancy.min_range
            >= config.pgo.occupancy.usable_range) {
            throw std::invalid_argument(
                "pgo.occupancy.min_range must be < usable_range");
        }
    }

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
