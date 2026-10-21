#pragma once
#include <toml++/toml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace sapphire::backend::io {

struct CameraBinding {
  std::string calibration_key;
  std::vector<int64_t> cam_overlaps;
  std::string mask;
  std::string rostopic, depth;
  std::string rectify_key;
  int64_t device_index = -1; // Absent means this camera is not in the device batch.
  // Explicit capture/binning contract, [width,height]. Both absent preserves
  // legacy full-calibration pixels. Cropping and anisotropic scaling are not inferred.
  std::array<int, 2> calibration_image_size{}, raw_image_size{};
};

struct SensorConfig {
  std::filesystem::path directory;
  std::string calibration_key, left, right, depth, batch_endpoint;
  std::filesystem::path file;
  std::map<int64_t, CameraBinding> cameras;
  std::array<int64_t, 2> stereo_cameras{0, 1};
  int64_t depth_camera = 0;
  std::map<int64_t, std::string> ros_topics;
  std::vector<int64_t> device_cameras; // Batch slot -> camera ID; -1 explicitly ignores a slot.
  std::string rectify_key;
  // Appended to preserve existing aggregate initialization of sensor bindings.
  // Explicit per-camera dimensions take precedence over transport scaling.
  double ros_calibration_scale = 1.0, device_calibration_scale = 1.0;
  bool device_input = false;
  std::string rectificationKey() const {
    if (!rectify_key.empty()) return rectify_key;
    if (cameras.empty()) return calibration_key;
    throw std::invalid_argument("Selected camera has no rectify_key in sensors.toml");
  }
};

inline std::filesystem::path calibrationDirectory() {
  const char* path = std::getenv("PDR_CALIBRATION_FILE_PATH");
  return path && *path ? path : "/userdata/hardware";
}

inline std::filesystem::path sensorConfigFile() {
  if (const char* prefix = std::getenv("EXTERIOR_INSTALL_PREFIX"); prefix && *prefix)
    return std::filesystem::path(prefix) / "config/backend/sensors.toml";
  const auto bin = std::filesystem::read_symlink("/proc/self/exe").parent_path();
  const auto prefix = bin.filename() == "exterior" && bin.parent_path().filename() == "libexec"
                          ? bin.parent_path().parent_path() : bin.parent_path();
  return prefix / "config/backend/sensors.toml";
}

inline SensorConfig loadSensorConfig(const std::filesystem::path& path = sensorConfigFile(),
                                  const std::filesystem::path& directory = calibrationDirectory(),
                                  const std::array<int64_t, 2>& pair = {0, 1}) {
  try {
    const auto root = toml::parse_file(path.string());
    auto keys = [](const toml::table& node, const std::set<std::string>& allowed, const char* location) {
      for (const auto& item : node) {
        const std::string key(item.first.str());
        if (!allowed.count(key)) throw std::runtime_error(std::string(location) + ": unknown key " + key);
      }
    };
    auto table = [](const toml::table& node, const std::string& key) -> const toml::table& {
      const auto* value = node[key].as_table();
      if (!value) throw std::runtime_error(key + " must be a table");
      return *value;
    };
    auto text = [](const toml::table& node, const char* key) {
      const auto value = node[key].value<std::string>();
      if (!value) throw std::runtime_error(std::string("Missing text field: ") + key);
      const auto& result = *value;
      if (result.empty() || result.find_first_of(" \t\r\n") != std::string::npos)
        throw std::runtime_error(std::string("Empty or whitespace-containing field: ") + key);
      return result;
    };
    auto ids = [](const toml::table& node, const char* key) {
      const auto* values = node[key].as_array();
      if (!values) throw std::runtime_error(std::string(key) + " must be an array of camera IDs");
      std::vector<int64_t> result;
      std::set<int64_t> seen;
      for (const auto& value : *values) {
        const auto id = value.value<int64_t>();
        if (!value.is_integer() || !id || *id < 0 || !seen.insert(*id).second)
          throw std::runtime_error(std::string(key) + " requires distinct nonnegative integer camera IDs");
        result.push_back(*id);
      }
      return result;
    };
    SensorConfig config;
    config.directory = std::filesystem::absolute(directory).lexically_normal();
    config.file = std::filesystem::absolute(path).lexically_normal();
    auto scale = [](const toml::table& node) {
      if (!node.contains("calibration_scale")) return 1.0;
      const auto value = node["calibration_scale"].value<double>();
      if (!value || !std::isfinite(*value) || *value <= 0.0)
        throw std::runtime_error("calibration_scale must be finite and positive");
      return *value;
    };
    if (root.contains("ros")) {
      const auto& ros = table(root, "ros");
      keys(ros, {"calibration_scale"}, "ros");
      config.ros_calibration_scale = scale(ros);
    }
    for (const auto& [key, value] : root) {
      const std::string name(key.str());
      if (name == "device" || name == "ros") continue;
      if (name.size() <= 3 || name.compare(0, 3, "cam") != 0 ||
          name.find_first_not_of("0123456789", 3) != std::string::npos)
        throw std::runtime_error("sensors: unknown key " + name);
      const auto id = std::stoll(name.substr(3));
      if (name != "cam" + std::to_string(id)) throw std::runtime_error("Noncanonical camera ID: " + name);
      const auto& camera = table(root, name);
      keys(camera, {"calibration_key", "rectify_key", "rostopic", "depth", "cam_overlaps", "mask", "device_index", "calibration_image_size", "raw_image_size"}, name.c_str());
      CameraBinding binding{text(camera, "calibration_key"), ids(camera, "cam_overlaps"), {}};
      if (camera.contains("calibration_image_size") || camera.contains("raw_image_size")) {
        auto size = [&](const char* field) {
          const auto* a = camera[field].as_array();
          if (!a || a->size() != 2) throw std::runtime_error(name + "." + field + " requires [width,height]");
          std::array<int, 2> result{};
          for (int j = 0; j < 2; ++j) {
            const auto v = (*a)[j].value<int64_t>();
            if (!(*a)[j].is_integer() || !v || *v <= 0 || *v > 32768)
              throw std::runtime_error(name + "." + field + " requires positive integer dimensions <=32768");
            result[j] = int(*v);
          }
          return result;
        };
        binding.calibration_image_size = size("calibration_image_size");
        binding.raw_image_size = size("raw_image_size");
        const auto a = binding.calibration_image_size, b = binding.raw_image_size;
        if (int64_t(a[0]) * b[1] != int64_t(a[1]) * b[0])
          throw std::runtime_error(name + ": raw/calibration aspect ratio differs; crop/stretch requires a new calibration");
      }
      if (camera.contains("mask")) {
        const auto mask = camera["mask"].value<std::string>();
        if (!mask || mask->find_first_of("\r\n") != std::string::npos || mask->find('\0') != std::string::npos)
          throw std::runtime_error(name + ".mask must be a file path string (empty disables masking)");
        binding.mask = *mask;
      }
      for (const char* field : {"rostopic", "depth", "rectify_key"}) {
        if (!camera.contains(field)) continue;
        const auto value = camera[field].value<std::string>();
        if (!value || value->find_first_of(" \t\r\n") != std::string::npos || value->find('\0') != std::string::npos)
          throw std::runtime_error(name + "." + field + " must be an address string (empty disables this input)");
        if (std::string(field) == "rostopic") binding.rostopic = *value;
        else if (std::string(field) == "depth") binding.depth = *value;
        else binding.rectify_key = *value;
      }
      if (camera.contains("device_index")) {
        const auto index = camera["device_index"].value<int64_t>();
        if (!camera["device_index"].is_integer() || !index || *index < 0 || *index >= 64)
          throw std::runtime_error(name + ".device_index must be an integer in [0, 63]");
        binding.device_index = *index;
      }
      config.cameras.emplace(id, std::move(binding));
    }
    for (const auto& [id, camera] : config.cameras) {
      for (const auto overlap : camera.cam_overlaps) {
        if (overlap == id || !config.cameras.count(overlap))
          throw std::runtime_error("cam" + std::to_string(id) + ": cam_overlaps references itself or an undefined camera");
      }
    }
    const auto& device = table(root, "device");
    keys(device, {"batch_endpoint", "calibration_scale"}, "device");
    config.device_calibration_scale = scale(device);
    if (pair[0] == pair[1] || !config.cameras.count(pair[0]))
      throw std::runtime_error("Selected reference camera must exist and differ from the optional stereo partner");
    for (size_t i = 0; config.cameras.count(pair[1]) && i < 2; ++i) {
      const auto& overlaps = config.cameras.at(pair[i]).cam_overlaps;
      if (std::find(overlaps.begin(), overlaps.end(), pair[1-i]) == overlaps.end())
        throw std::runtime_error("Stereo cameras must declare mutual cam_overlaps");
    }
    config.stereo_cameras = {pair[0], pair[1]};
    config.depth_camera = pair[0];
    const auto& left = config.cameras.at(pair[0]);
    const auto right = config.cameras.find(pair[1]);
    if (right != config.cameras.end() && left.rectify_key != right->second.rectify_key)
      throw std::runtime_error("Selected overlapping cameras must use the same rectify_key");
    config.calibration_key = left.calibration_key;
    config.rectify_key = left.rectify_key;
    std::set<std::string> topics;
    for (const auto& [id, camera] : config.cameras) {
      if (!camera.rostopic.empty()) config.ros_topics.emplace(id, camera.rostopic);
      for (const auto& topic : {camera.rostopic, camera.depth})
        if (!topic.empty() && !topics.insert(topic).second)
          throw std::runtime_error("Camera image and depth topics must be distinct");
    }
    config.left = left.rostopic;
    config.right = right == config.cameras.end() ? std::string{} : right->second.rostopic;
    config.depth = left.depth;
    config.batch_endpoint = text(device, "batch_endpoint");
    for (const auto& [id, camera] : config.cameras) {
      if (camera.device_index < 0) continue;
      const auto index = size_t(camera.device_index);
      if (config.device_cameras.size() <= index) config.device_cameras.resize(index + 1, -1);
      if (config.device_cameras[index] >= 0) throw std::runtime_error("Duplicate camera device_index");
      config.device_cameras[index] = id;
    }
    return config;
  } catch (const std::exception& error) {
    throw std::runtime_error(path.string() + ": " + error.what());
  }
}
}  // namespace sapphire::backend::io
