#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <sapphire/config.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <toml++/toml.hpp>
#include <unordered_set>

namespace sapphire {
namespace {

struct ParseContext {
  std::unordered_set<std::string> known;
};

using NodeView = toml::node_view<toml::node>;

void remember(ParseContext& ctx, const char* path) { ctx.known.insert(path); }

[[noreturn]] void throwFieldType(const toml::node& node, const char* path, const char* expected) {
  std::ostringstream message;
  message << "TOML field " << path << " at line " << node.source().begin.line << " must be " << expected;
  throw std::runtime_error(message.str());
}

[[noreturn]] void throwUnknownKey(const toml::node& node, const std::string& path) {
  std::ostringstream message;
  message << "unknown TOML key '" << path << "' at line " << node.source().begin.line;
  throw std::runtime_error(message.str());
}

[[noreturn]] void throwMissing(const char* path) { throw std::runtime_error(std::string("missing required TOML field: ") + path); }

std::optional<double> optionalNumber(ParseContext& ctx, const NodeView& node, const char* path) {
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

double requiredNumber(ParseContext& ctx, const NodeView& node, const char* path) {
  const auto value = optionalNumber(ctx, node, path);
  if (!value.has_value()) {
    throwMissing(path);
  }
  return *value;
}

std::optional<int> optionalInt(ParseContext& ctx, const NodeView& node, const char* path) {
  remember(ctx, path);
  if (!node) {
    return std::nullopt;
  }
  if (!node.is_integer()) {
    throwFieldType(*node.node(), path, "an integer");
  }
  const auto value = node.value_exact<int64_t>();
  if (!value.has_value() || *value < static_cast<int64_t>(std::numeric_limits<int>::min()) ||
      *value > static_cast<int64_t>(std::numeric_limits<int>::max())) {
    throwFieldType(*node.node(), path, "an integer");
  }
  return static_cast<int>(*value);
}

std::optional<bool> optionalBool(ParseContext& ctx, const NodeView& node, const char* path) {
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

std::optional<std::string> optionalString(ParseContext& ctx, const NodeView& node, const char* path) {
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

const toml::array& requireNumericArray(ParseContext& ctx, const NodeView& node, const char* path, size_t length) {
  remember(ctx, path);
  if (!node) {
    throwMissing(path);
  }
  if (!node.is_array() || node.as_array() == nullptr || node.as_array()->size() != length) {
    std::ostringstream message;
    message << "TOML field " << path << " at line " << node.node()->source().begin.line << " must be a numeric array of length " << length;
    throw std::runtime_error(message.str());
  }
  return *node.as_array();
}

Eigen::Vector3d requiredVector3(ParseContext& ctx, const NodeView& node, const char* path) {
  const toml::array& array = requireNumericArray(ctx, node, path, 3);
  Eigen::Vector3d result;
  for (size_t i = 0; i < 3; ++i) {
    result[static_cast<Eigen::Index>(i)] = readNumberElement(array[i], path);
  }
  return result;
}

Eigen::Quaterniond requiredQuaternionXyzw(ParseContext& ctx, const NodeView& node, const char* path) {
  const toml::array& array = requireNumericArray(ctx, node, path, 4);
  double values[4]{};
  for (size_t i = 0; i < 4; ++i) {
    values[i] = readNumberElement(array[i], path);
  }

  Eigen::Quaterniond quaternion(values[3], values[0], values[1], values[2]);
  if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1e-12) {
    std::ostringstream message;
    message << "TOML field " << path << " at line " << node.node()->source().begin.line << " is non-finite or has zero norm";
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

void rejectUnknownKeys(const ParseContext& ctx, const toml::table& table, const std::string& prefix = {}) {
  for (auto&& [key, node] : table) {
    const std::string path = prefix.empty() ? std::string(key.str()) : prefix + "." + std::string(key.str());
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
    throw std::invalid_argument(std::string(name) + " must be finite and nonnegative");
  }
}

void requireFinitePositive(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
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
  if (file.empty() || path.is_absolute() || !path.has_filename() || path.filename() == "." || path.filename() == ".." || has_parent_escape) {
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
  NonnegativeFloat,
  UnitInterval,
};

enum class Gate {
  Always,
  Occupancy,
};

enum class Required { Never, Always };

struct NumberField {
  const char* path;
  void (*set)(Config&, double);
  double (*get)(const Config&);
  Required required;
  NumberRule rule;
  Gate gate;
};

struct IntField {
  const char* path;
  void (*set)(Config&, int);
  int (*get)(const Config&);
  int minimum;
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
};

#define FIELD(path, member, required, rule, gate)                                                                                               \
  NumberField {                                                                                                                                 \
    path, [](Config& config, double value) { config.member = value; }, [](const Config& config) { return config.member; }, required, rule, gate \
  }

#define OPT(path, member, rule) FIELD(path, member, Required::Never, rule, Gate::Always)
#define REQ(path, member, rule) FIELD(path, member, Required::Always, rule, Gate::Always)
#define WHEN_OCC(path, member, rule) FIELD(path, member, Required::Never, rule, Gate::Occupancy)

#define BOOL_FIELD(path, member)                                    \
  BoolField {                                                       \
    path, [](Config& config, bool value) { config.member = value; } \
  }

#define STR(path, member, choices)                                                        \
  StringField {                                                                           \
    path, [](Config& config, const std::string& value) { config.member = value; },        \
        [](const Config& config) -> const std::string& { return config.member; }, choices \
  }

const NumberField kNumbers[] = {
    REQ("sensor.imu.gravity", sensor.imu.init.gravity_mag, NumberRule::Positive),
    OPT("sensor.imu.gyro_var", sensor.imu.noise.gyro_var, NumberRule::NonnegativeFloat),
    OPT("sensor.imu.accel_var", sensor.imu.noise.accel_var, NumberRule::NonnegativeFloat),
    OPT("sensor.imu.gyro_bias_var", sensor.imu.noise.gyro_bias_var, NumberRule::NonnegativeFloat),
    OPT("sensor.imu.accel_bias_var", sensor.imu.noise.accel_bias_var, NumberRule::NonnegativeFloat),

    OPT("odometry.voxel_size", odometry.voxel_size, NumberRule::Positive),
    // Finite and ordered as a group in validateConfig.
    OPT("odometry.crop_box.min_x", odometry.crop_box.min_x, NumberRule::None),
    OPT("odometry.crop_box.min_y", odometry.crop_box.min_y, NumberRule::None),
    OPT("odometry.crop_box.min_z", odometry.crop_box.min_z, NumberRule::None),
    OPT("odometry.crop_box.max_x", odometry.crop_box.max_x, NumberRule::None),
    OPT("odometry.crop_box.max_y", odometry.crop_box.max_y, NumberRule::None),
    OPT("odometry.crop_box.max_z", odometry.crop_box.max_z, NumberRule::None),

    OPT("io.global_map_voxel_size", io.global_map_voxel_size, NumberRule::Positive),
    OPT("pgo.relinearize_threshold", pgo.relinearize_threshold, NumberRule::Nonnegative),
    OPT("submap.max_range_m", submap.max_range_m, NumberRule::Positive),
    OPT("submap.max_deformation_m", submap.max_deformation_m, NumberRule::Nonnegative),
    OPT("submap.ba.relative_cost_tolerance", submap.ba.relative_cost_tolerance, NumberRule::Nonnegative),
    OPT("pgo.loop_closure.local_search_radius_m", pgo.loop_closure.local_search_radius_m, NumberRule::Positive),
    OPT("pgo.loop_closure.local_departure_radius_m", pgo.loop_closure.local_departure_radius_m, NumberRule::Positive),
    OPT("pgo.loop_closure.local_min_overlap", pgo.loop_closure.local_min_overlap, NumberRule::UnitInterval),
    OPT("pgo.loop_closure.global_search_radius_min_m", pgo.loop_closure.global_search_radius_min_m, NumberRule::Positive),
    OPT("pgo.loop_closure.global_search_radius_max_m", pgo.loop_closure.global_search_radius_max_m, NumberRule::Positive),
    OPT("pgo.loop_closure.global_search_radius_growth", pgo.loop_closure.global_search_radius_growth, NumberRule::Nonnegative),
    OPT("pgo.loop_closure.global_departure_radius_m", pgo.loop_closure.global_departure_radius_m, NumberRule::Positive),
    OPT("pgo.loop_closure.fft_min_score", pgo.loop_closure.fft_min_score, NumberRule::UnitInterval),
    OPT("pgo.loop_closure.fft_min_ambiguity", pgo.loop_closure.fft_min_ambiguity, NumberRule::Positive),
    OPT("pgo.loop_closure.global_min_overlap", pgo.loop_closure.global_min_overlap, NumberRule::UnitInterval),
    OPT("pgo.loop_closure.global_max_rms_m", pgo.loop_closure.global_max_rms_m, NumberRule::Positive),
    WHEN_OCC("pgo.occupancy.resolution", pgo.occupancy.resolution, NumberRule::Positive),
    WHEN_OCC("pgo.occupancy.h_clearance", pgo.occupancy.h_clearance, NumberRule::Nonnegative),
    WHEN_OCC("pgo.occupancy.ground_margin", pgo.occupancy.ground_margin, NumberRule::Nonnegative),
    WHEN_OCC("pgo.occupancy.d_max", pgo.occupancy.d_max, NumberRule::Nonnegative),
    WHEN_OCC("pgo.occupancy.clear_height_eps", pgo.occupancy.clear_height_eps, NumberRule::Nonnegative),
    WHEN_OCC("pgo.occupancy.occ_threshold", pgo.occupancy.occ_threshold, NumberRule::UnitInterval),
    WHEN_OCC("pgo.occupancy.usable_range", pgo.occupancy.usable_range, NumberRule::Positive),
    WHEN_OCC("pgo.occupancy.min_range", pgo.occupancy.min_range, NumberRule::Nonnegative),
    WHEN_OCC("pgo.occupancy.cloud_voxel_size", pgo.occupancy.cloud_voxel_size, NumberRule::Nonnegative),
    WHEN_OCC("pgo.occupancy.margin", pgo.occupancy.margin, NumberRule::Positive),

    OPT("sensor.lidar.max_range_m", sensor.lidar.max_range_m, NumberRule::Nonnegative),
    OPT("sensor.lidar.max_scan_duration", sensor.lidar.max_scan_duration, NumberRule::Positive),
    OPT("sensor.lidar.header_tolerance", sensor.lidar.header_tolerance, NumberRule::Nonnegative),
};

#define INT_FIELD(path, member, minimum) \
  IntField { path, [](Config& config, int value) { config.member = value; }, [](const Config& config) { return config.member; }, minimum }

const IntField kInts[] = {
    INT_FIELD("odometry.voxel_capacity", odometry.voxel_capacity, 2),
    INT_FIELD("pgo.relinearize_skip", pgo.relinearize_skip, 1),
    INT_FIELD("submap.min_voxel_num", submap.min_voxel_num, 1),
    INT_FIELD("submap.min_frames", submap.min_frames, 1),
    INT_FIELD("submap.ba.min_frames", submap.ba.min_frames, 3),
    INT_FIELD("submap.ba.min_planes", submap.ba.min_planes, 1),
    INT_FIELD("submap.ba.max_iterations", submap.ba.max_iterations, 1),
    INT_FIELD("pgo.loop_closure.local_max_candidates", pgo.loop_closure.local_max_candidates, 1),
    INT_FIELD("pgo.loop_closure.global_max_candidates", pgo.loop_closure.global_max_candidates, 1),
    INT_FIELD("pgo.loop_closure.global_refine_updates", pgo.loop_closure.global_refine_updates, 0),
};

const BoolField kBools[] = {
    BOOL_FIELD("odometry.enable_map_feedback", odometry.enable_map_feedback),
    BOOL_FIELD("odometry.map_feedback_ba", odometry.map_feedback_ba),
    BOOL_FIELD("pgo.enabled", pgo.enabled),
    BOOL_FIELD("pgo.loop_closure.enabled", pgo.loop_closure.enabled),
    BOOL_FIELD("pgo.occupancy.enabled", pgo.occupancy.enabled),
    BOOL_FIELD("sensor.lidar.require_point_time", sensor.lidar.require_point_time),
};

const StringField kStrings[] = {
    STR("io.map_directory", io.map_directory, nullptr),
    STR("sensor.lidar.time_field", sensor.lidar.time_field, sensor::timeFieldChoices()),
    STR("sensor.lidar.time_unit", sensor.lidar.time_unit, sensor::timeUnitChoices()),
    STR("sensor.lidar.time_ref", sensor.lidar.time_ref, sensor::timeRefChoices()),
    STR("output.trajectory_dir", output.trajectory_dir, nullptr),
    STR("output.trajectory_file", output.trajectory_file, nullptr),
};

#undef INT_FIELD
#undef FIELD
#undef OPT
#undef REQ
#undef WHEN_OCC
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
    case NumberRule::NonnegativeFloat:
      requireFiniteNonnegative(value, field.path);
      if (value > std::numeric_limits<float>::max()) {
        throw std::invalid_argument(std::string(field.path) + " is outside the float range");
      }
      break;
    case NumberRule::UnitInterval:
      if (!std::isfinite(value) || value <= 0.0 || value > 1.0) {
        throw std::invalid_argument(std::string(field.path) + " must be in (0, 1]");
      }
      break;
  }
}

void loadSchema(ParseContext& ctx, toml::table& root, Config& config) {
  for (const BoolField& field : kBools) {
    const NodeView node = root.at_path(field.path);
    if (const auto value = optionalBool(ctx, node, field.path)) {
      field.set(config, *value);
    }
  }
  for (const NumberField& field : kNumbers) {
    const NodeView node = root.at_path(field.path);
    if (field.required == Required::Always) {
      field.set(config, requiredNumber(ctx, node, field.path));
    } else if (const auto value = optionalNumber(ctx, node, field.path)) {
      field.set(config, *value);
    }
  }
  for (const IntField& field : kInts) {
    if (const auto value = optionalInt(ctx, root.at_path(field.path), field.path)) {
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
  const Eigen::Vector3d translation = requiredVector3(ctx, extrinsics["translation"], "extrinsics.T_imu_lidar.translation");
  const Eigen::Quaterniond rotation = requiredQuaternionXyzw(ctx, extrinsics["rotation_xyzw"], "extrinsics.T_imu_lidar.rotation_xyzw");
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
    if (field.get(config) < field.minimum) {
      throw std::invalid_argument(std::string(field.path) + " must be at least " + std::to_string(field.minimum));
    }
  }
  for (const StringField& field : kStrings) {
    if (field.choices == nullptr || isAllowedChoice(field.get(config), field.choices)) {
      continue;
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

  const auto& crop = config.odometry.crop_box;
  if (!std::isfinite(crop.min_x) || !std::isfinite(crop.min_y) || !std::isfinite(crop.min_z) || !std::isfinite(crop.max_x) ||
      !std::isfinite(crop.max_y) || !std::isfinite(crop.max_z) || crop.min_x >= crop.max_x || crop.min_y >= crop.max_y || crop.min_z >= crop.max_z) {
    throw std::invalid_argument("odometry.crop_box bounds must be finite and ordered");
  }

  if (config.pgo.occupancy.enabled) {
    if (config.pgo.occupancy.ground_margin >= config.pgo.occupancy.h_clearance && config.pgo.occupancy.h_clearance > 0.0) {
      throw std::invalid_argument("pgo.occupancy.ground_margin must be < h_clearance");
    }
    if (config.pgo.occupancy.min_range >= config.pgo.occupancy.usable_range) {
      throw std::invalid_argument("pgo.occupancy.min_range must be < usable_range");
    }
  }

  requireRelativeTrajectoryFile(config.output.trajectory_file);
  if (!config.pgo.enabled) {
    throw std::invalid_argument("pgo.enabled must be true for Method B");
  }
  if (config.io.map_directory.empty()) {
    throw std::invalid_argument("io.map_directory must not be empty");
  }
  const auto& loop = config.pgo.loop_closure;
  if (loop.local_departure_radius_m < loop.local_search_radius_m) {
    throw std::invalid_argument("pgo.loop_closure.local_departure_radius_m must be >= local_search_radius_m");
  }
  if (loop.global_search_radius_min_m > loop.global_search_radius_max_m) {
    throw std::invalid_argument("pgo.loop_closure.global_search_radius_max_m must be >= global_search_radius_min_m");
  }

  const Isometry3d& extrinsic = config.extrinsics.T_imu_lidar;
  if (!extrinsic.matrix().allFinite()) {
    throw std::invalid_argument("extrinsics.T_imu_lidar must contain only finite values");
  }
  const Eigen::Matrix3d should_be_identity = extrinsic.rotation().transpose() * extrinsic.rotation();
  if (!should_be_identity.isApprox(Eigen::Matrix3d::Identity(), 1e-9) || std::abs(extrinsic.rotation().determinant() - 1.0) > 1e-9) {
    throw std::invalid_argument("extrinsics.T_imu_lidar rotation must be a proper rotation");
  }
}

std::filesystem::path trajectoryOutputPath(const Config& config) {
  if (config.output.trajectory_dir.empty()) {
    return {};
  }
  return std::filesystem::path(config.output.trajectory_dir) / config.output.trajectory_file;
}

}  // namespace sapphire
