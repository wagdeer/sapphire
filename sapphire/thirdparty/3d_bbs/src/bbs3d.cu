#include <bbs3d.hpp>

#include <cuda_runtime.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <discrete_transformation.hpp>

namespace gpu {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 2.0F * kPi;
constexpr std::size_t kBranchStockSize = 10000;
constexpr int kCudaBlockSize = 128;

struct YawInfo {
  int num_divisions = 1;
  float resolution = 0.0F;
  float minimum = 0.0F;
};

struct LeafIndexWindow {
  std::array<std::int64_t, 3> minimum{};
  std::array<std::int64_t, 3> maximum{};
};

struct DeviceNode {
  int level;
  int x;
  int y;
  int z;
  int yaw;
};

struct DevicePose {
  float rotation[9];
  float translation[3];
};

void check_cuda(cudaError_t error, const char* operation) {
  if (error != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(error));
  }
}

template <typename T>
class DeviceBuffer {
 public:
  DeviceBuffer() = default;

  explicit DeviceBuffer(std::size_t size) { allocate(size); }

  ~DeviceBuffer() {
    if (data_ != nullptr) {
      cudaFree(data_);
    }
  }

  DeviceBuffer(const DeviceBuffer&) = delete;
  DeviceBuffer& operator=(const DeviceBuffer&) = delete;

  DeviceBuffer(DeviceBuffer&& other) noexcept : data_(std::exchange(other.data_, nullptr)), size_(other.size_) {
    other.size_ = 0;
  }

  DeviceBuffer& operator=(DeviceBuffer&& other) noexcept {
    if (this != &other) {
      if (data_ != nullptr) {
        cudaFree(data_);
      }
      data_ = std::exchange(other.data_, nullptr);
      size_ = other.size_;
      other.size_ = 0;
    }
    return *this;
  }

  void allocate(std::size_t size) {
    if (size <= size_) {
      return;
    }
    if (data_ != nullptr) {
      check_cuda(cudaFree(data_), "cudaFree");
      data_ = nullptr;
      size_ = 0;
    }
    size_ = size;
    check_cuda(cudaMalloc(reinterpret_cast<void**>(&data_), size * sizeof(T)), "cudaMalloc");
  }

  void copy_from_host(const T* source, std::size_t size) {
    if (size > size_) {
      throw std::out_of_range("CUDA copy exceeds device buffer");
    }
    check_cuda(cudaMemcpy(data_, source, size * sizeof(T), cudaMemcpyHostToDevice), "cudaMemcpy host-to-device");
  }

  void copy_to_host(T* destination, std::size_t size) const {
    if (size > size_) {
      throw std::out_of_range("CUDA copy exceeds device buffer");
    }
    check_cuda(cudaMemcpy(destination, data_, size * sizeof(T), cudaMemcpyDeviceToHost), "cudaMemcpy device-to-host");
  }

  T* data() { return data_; }
  const T* data() const { return data_; }

 private:
  T* data_ = nullptr;
  std::size_t size_ = 0;
};

struct DeviceTarget {
  explicit DeviceTarget(const cpu::VoxelMapsData& target)
      : bucket_storage(static_cast<std::size_t>(target.max_level + 1)),
        bucket_pointers(static_cast<std::size_t>(target.max_level + 1)),
        bucket_sizes(static_cast<std::size_t>(target.max_level + 1)),
        resolutions(static_cast<std::size_t>(target.max_level + 1)) {
    std::vector<const int4*> host_pointers(bucket_storage.size());
    std::vector<int> host_sizes(bucket_storage.size());
    std::vector<float> host_resolutions(bucket_storage.size());
    for (int level = 0; level <= target.max_level; ++level) {
      const cpu::VoxelBuckets& buckets = target.buckets_for_level(level);
      std::vector<int4> packed;
      packed.reserve(buckets.size());
      for (const Eigen::Vector4i& bucket : buckets) {
        packed.push_back(make_int4(bucket.x(), bucket.y(), bucket.z(), bucket.w()));
      }
      DeviceBuffer<int4>& device_buckets = bucket_storage[static_cast<std::size_t>(level)];
      device_buckets.allocate(packed.size());
      device_buckets.copy_from_host(packed.data(), packed.size());
      host_pointers[static_cast<std::size_t>(level)] = device_buckets.data();
      host_sizes[static_cast<std::size_t>(level)] = static_cast<int>(packed.size());
      host_resolutions[static_cast<std::size_t>(level)] = target.resolution(level);
    }
    bucket_pointers.copy_from_host(host_pointers.data(), host_pointers.size());
    bucket_sizes.copy_from_host(host_sizes.data(), host_sizes.size());
    resolutions.copy_from_host(host_resolutions.data(), host_resolutions.size());
  }

  std::vector<DeviceBuffer<int4>> bucket_storage;
  DeviceBuffer<const int4*> bucket_pointers;
  DeviceBuffer<int> bucket_sizes;
  DeviceBuffer<float> resolutions;
};

struct ScoreWorkspace {
  ScoreWorkspace(std::size_t capacity, const std::vector<YawInfo>& yaw_info)
      : device_nodes(capacity), device_scores(capacity), device_yaw(yaw_info.size()) {
    host_nodes.reserve(capacity);
    host_scores.reserve(capacity);
    device_yaw.copy_from_host(yaw_info.data(), yaw_info.size());
  }

  void ensure_capacity(std::size_t capacity) {
    device_nodes.allocate(capacity);
    device_scores.allocate(capacity);
    if (host_nodes.capacity() < capacity) {
      host_nodes.reserve(capacity);
      host_scores.reserve(capacity);
    }
  }

  DeviceBuffer<DeviceNode> device_nodes;
  DeviceBuffer<int> device_scores;
  DeviceBuffer<YawInfo> device_yaw;
  std::vector<DeviceNode> host_nodes;
  std::vector<int> host_scores;
};

float normalize_yaw(float yaw) {
  double normalized = std::fmod(static_cast<double>(yaw) + static_cast<double>(kPi),
                               static_cast<double>(kTwoPi));
  if (normalized < 0.0) {
    normalized += static_cast<double>(kTwoPi);
  }
  return static_cast<float>(normalized - static_cast<double>(kPi));
}

Eigen::Vector3f rotation_to_rpy(const Eigen::Matrix3f& rotation) {
  return {std::atan2(rotation(2, 1), rotation(2, 2)),
          std::asin(std::clamp(-rotation(2, 0), -1.0F, 1.0F)),
          std::atan2(rotation(1, 0), rotation(0, 0))};
}

std::vector<float3> finite_points(const Eigen::Vector3f* points, std::size_t point_count) {
  std::vector<float3> finite;
  finite.reserve(point_count);
  for (std::size_t index = 0; index < point_count; ++index) {
    if (points[index].allFinite()) {
      finite.push_back(make_float3(points[index].x(), points[index].y(), points[index].z()));
    }
  }
  return finite;
}

std::vector<YawInfo> make_yaw_info(const cpu::VoxelMapsData& voxelmaps,
                                   const std::vector<float3>& source_points, float initial_yaw,
                                   float min_yaw_offset, float max_yaw_offset) {
  float maximum_norm = 0.0F;
  for (const float3& point : source_points) {
    maximum_norm = std::max(maximum_norm, std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z));
  }

  const float yaw_width = max_yaw_offset - min_yaw_offset;
  std::vector<YawInfo> levels(static_cast<std::size_t>(voxelmaps.max_level + 1));
  for (int level = voxelmaps.max_level; level >= 0; --level) {
    const float parent_piece =
        level == voxelmaps.max_level || levels[static_cast<std::size_t>(level + 1)].resolution == 0.0F
            ? yaw_width
            : levels[static_cast<std::size_t>(level + 1)].resolution;
    float angular_resolution = 0.0F;
    if (maximum_norm > std::numeric_limits<float>::epsilon() && yaw_width > 0.0F) {
      const float ratio = voxelmaps.resolution(level) / maximum_norm;
      const float cosine = std::clamp(1.0F - ratio * ratio * 0.5F, -1.0F, 1.0F);
      angular_resolution = std::floor(std::acos(cosine) * 10000.0F) / 10000.0F;
    }

    YawInfo& info = levels[static_cast<std::size_t>(level)];
    if (yaw_width > 0.0F && angular_resolution > 0.0F && angular_resolution <= yaw_width &&
        parent_piece > 0.0F) {
      info.num_divisions =
          std::max(1, static_cast<int>(std::ceil(parent_piece / angular_resolution)));
    }
    info.resolution =
        info.num_divisions == 1 ? 0.0F : parent_piece / static_cast<float>(info.num_divisions);
    const float offset =
        yaw_width == 0.0F
            ? 0.0F
            : (info.resolution == 0.0F ? 0.5F * (min_yaw_offset + max_yaw_offset)
                                      : min_yaw_offset);
    info.minimum = normalize_yaw(initial_yaw + offset);
  }
  return levels;
}

LeafIndexWindow make_leaf_window(const Eigen::Vector3f& center, const Eigen::Vector3f& half_window,
                                 float resolution) {
  LeafIndexWindow window;
  for (int dimension = 0; dimension < 3; ++dimension) {
    const double minimum =
        (static_cast<double>(center[dimension]) - static_cast<double>(half_window[dimension])) /
        static_cast<double>(resolution);
    const double maximum =
        (static_cast<double>(center[dimension]) + static_cast<double>(half_window[dimension])) /
        static_cast<double>(resolution);
    window.minimum[static_cast<std::size_t>(dimension)] =
        static_cast<std::int64_t>(std::ceil(minimum));
    window.maximum[static_cast<std::size_t>(dimension)] =
        static_cast<std::int64_t>(std::floor(maximum));
  }
  return window;
}

bool leaf_inside_window(const DiscreteTransformation<float>& node,
                        const LeafIndexWindow& window) {
  return static_cast<std::int64_t>(node.x) >= window.minimum[0] &&
         static_cast<std::int64_t>(node.x) <= window.maximum[0] &&
         static_cast<std::int64_t>(node.y) >= window.minimum[1] &&
         static_cast<std::int64_t>(node.y) <= window.maximum[1] &&
         static_cast<std::int64_t>(node.z) >= window.minimum[2] &&
         static_cast<std::int64_t>(node.z) <= window.maximum[2];
}

Eigen::Isometry3f node_pose(const DiscreteTransformation<float>& node,
                            const cpu::VoxelMapsData& voxelmaps,
                            const std::vector<YawInfo>& yaw_info, float fixed_roll,
                            float fixed_pitch) {
  const YawInfo& yaw = yaw_info[static_cast<std::size_t>(node.level)];
  const float translation_resolution = voxelmaps.resolution(node.level);
  const Eigen::Translation3f translation(node.x * translation_resolution,
                                         node.y * translation_resolution,
                                         node.z * translation_resolution);
  const Eigen::AngleAxisf roll(fixed_roll, Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitch(fixed_pitch, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf rotation(
      normalize_yaw(yaw.minimum + node.yaw * yaw.resolution), Eigen::Vector3f::UnitZ());
  return Eigen::Isometry3f(translation * rotation * pitch * roll);
}

DevicePose pack_pose(const Eigen::Isometry3f& pose) {
  DevicePose packed{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      packed.rotation[row * 3 + column] = pose.linear()(row, column);
    }
    packed.translation[row] = pose.translation()[row];
  }
  return packed;
}

__device__ bool contains_voxel(const int4* buckets, int bucket_count, int max_scan_count, int x,
                               int y, int z) {
  const std::uint32_t hash =
      static_cast<std::uint32_t>(x) * 73856093U ^
      static_cast<std::uint32_t>(y) * 19349669U ^
      static_cast<std::uint32_t>(z) * 83492791U;
  for (int scan = 0; scan < max_scan_count; ++scan) {
    const int index =
        static_cast<int>((static_cast<std::uint64_t>(hash) + static_cast<std::uint64_t>(scan)) %
                         static_cast<std::uint64_t>(bucket_count));
    const int4 bucket = buckets[index];
    if (bucket.w != 0 && bucket.x == x && bucket.y == y && bucket.z == z) {
      return true;
    }
  }
  return false;
}

__global__ void score_nodes_kernel(const float3* points, int point_count,
                                   const int4* const* level_buckets,
                                   const int* level_bucket_sizes, const float* level_resolutions,
                                   int max_scan_count, const DeviceNode* nodes,
                                   const YawInfo* yaw_info, float fixed_roll, float fixed_pitch,
                                   int node_count, int* scores) {
  const int node_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (node_index >= node_count) {
    return;
  }

  const DeviceNode node = nodes[node_index];
  const YawInfo yaw = yaw_info[node.level];
  const float angle = yaw.minimum + static_cast<float>(node.yaw) * yaw.resolution;
  float sr = 0.0F;
  float cr = 0.0F;
  float sp = 0.0F;
  float cp = 0.0F;
  float sy = 0.0F;
  float cy = 0.0F;
  sincosf(fixed_roll, &sr, &cr);
  sincosf(fixed_pitch, &sp, &cp);
  sincosf(angle, &sy, &cy);
  const float r00 = cy * cp;
  const float r01 = cy * sp * sr - sy * cr;
  const float r02 = cy * sp * cr + sy * sr;
  const float r10 = sy * cp;
  const float r11 = sy * sp * sr + cy * cr;
  const float r12 = sy * sp * cr - cy * sr;
  const float r20 = -sp;
  const float r21 = cp * sr;
  const float r22 = cp * cr;

  const float resolution = level_resolutions[node.level];
  const float inverse_resolution = 1.0F / resolution;
  const float tx = static_cast<float>(node.x) * resolution;
  const float ty = static_cast<float>(node.y) * resolution;
  const float tz = static_cast<float>(node.z) * resolution;
  const int4* buckets = level_buckets[node.level];
  const int bucket_count = level_bucket_sizes[node.level];
  int score = 0;
  for (int point_index = 0; point_index < point_count; ++point_index) {
    const float3 point = points[point_index];
    const int x = __float2int_rd((r00 * point.x + r01 * point.y + r02 * point.z + tx) *
                                inverse_resolution);
    const int y = __float2int_rd((r10 * point.x + r11 * point.y + r12 * point.z + ty) *
                                inverse_resolution);
    const int z = __float2int_rd((r20 * point.x + r21 * point.y + r22 * point.z + tz) *
                                inverse_resolution);
    score += contains_voxel(buckets, bucket_count, max_scan_count, x, y, z);
  }
  scores[node_index] = score;
}

__global__ void score_pose_kernel(const float3* points, int point_count, const int4* buckets,
                                  int bucket_count, int max_scan_count, float inverse_resolution,
                                  DevicePose pose, int* score) {
  const int point_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_index >= point_count) {
    return;
  }
  const float3 point = points[point_index];
  const int x = __float2int_rd(
      (pose.rotation[0] * point.x + pose.rotation[1] * point.y +
       pose.rotation[2] * point.z + pose.translation[0]) *
      inverse_resolution);
  const int y = __float2int_rd(
      (pose.rotation[3] * point.x + pose.rotation[4] * point.y +
       pose.rotation[5] * point.z + pose.translation[1]) *
      inverse_resolution);
  const int z = __float2int_rd(
      (pose.rotation[6] * point.x + pose.rotation[7] * point.y +
       pose.rotation[8] * point.z + pose.translation[2]) *
      inverse_resolution);
  if (contains_voxel(buckets, bucket_count, max_scan_count, x, y, z)) {
    atomicAdd(score, 1);
  }
}

__global__ void warmup_kernel() {}

const std::vector<int>& score_nodes(const std::vector<DiscreteTransformation<float>>& nodes,
                                    const DeviceBuffer<float3>& device_points, int point_count,
                                    const DeviceTarget& target, int max_scan_count,
                                    float fixed_roll, float fixed_pitch,
                                    ScoreWorkspace& workspace) {
  if (nodes.empty()) {
    workspace.host_scores.clear();
    return workspace.host_scores;
  }
  workspace.ensure_capacity(nodes.size());
  workspace.host_nodes.clear();
  for (const DiscreteTransformation<float>& node : nodes) {
    workspace.host_nodes.push_back({node.level, node.x, node.y, node.z, node.yaw});
  }
  workspace.device_nodes.copy_from_host(workspace.host_nodes.data(), workspace.host_nodes.size());
  const int block_count =
      (static_cast<int>(nodes.size()) + kCudaBlockSize - 1) / kCudaBlockSize;
  score_nodes_kernel<<<block_count, kCudaBlockSize>>>(
      device_points.data(), point_count, target.bucket_pointers.data(),
      target.bucket_sizes.data(), target.resolutions.data(), max_scan_count,
      workspace.device_nodes.data(), workspace.device_yaw.data(), fixed_roll, fixed_pitch,
      static_cast<int>(nodes.size()), workspace.device_scores.data());
  check_cuda(cudaGetLastError(), "score_nodes_kernel launch");
  workspace.host_scores.resize(nodes.size());
  workspace.device_scores.copy_to_host(workspace.host_scores.data(), workspace.host_scores.size());
  return workspace.host_scores;
}

PoseScore score_pose_device(const std::vector<float3>& points,
                            const DeviceBuffer<float3>& device_points,
                            const cpu::VoxelMapsData& target, const DeviceTarget& device_target,
                            const Eigen::Isometry3f& pose) {
  PoseScore result;
  if (points.empty()) {
    return result;
  }
  DeviceBuffer<int> device_score(1);
  check_cuda(cudaMemset(device_score.data(), 0, sizeof(int)), "cudaMemset score");
  const int block_count =
      (static_cast<int>(points.size()) + kCudaBlockSize - 1) / kCudaBlockSize;
  score_pose_kernel<<<block_count, kCudaBlockSize>>>(
      device_points.data(), static_cast<int>(points.size()),
      device_target.bucket_storage.front().data(),
      static_cast<int>(target.buckets_for_level(0).size()), target.max_bucket_scan_count,
      1.0F / target.min_level_resolution, pack_pose(pose), device_score.data());
  check_cuda(cudaGetLastError(), "score_pose_kernel launch");
  check_cuda(cudaDeviceSynchronize(), "score_pose_kernel execution");
  int matched = 0;
  device_score.copy_to_host(&matched, 1);
  result.available = true;
  result.matched_points = static_cast<std::size_t>(matched);
  result.evaluated_points = points.size();
  result.overlap =
      static_cast<double>(result.matched_points) / static_cast<double>(result.evaluated_points);
  return result;
}

}  // namespace

void initialize_device() {
  check_cuda(cudaFree(nullptr), "initialize CUDA context");
  warmup_kernel<<<1, 1>>>();
  check_cuda(cudaGetLastError(), "warmup_kernel launch");
  check_cuda(cudaDeviceSynchronize(), "warmup_kernel execution");
}

const char* search_termination_name(SearchTermination termination) noexcept {
  switch (termination) {
    case SearchTermination::kNoCandidates:
      return "no_candidates";
    case SearchTermination::kQueueExhausted:
      return "queue_exhausted";
  }
  return "unknown";
}

PoseScore BBS3D::score_pose(const Eigen::Vector3f* source_points, std::size_t point_count,
                            const cpu::VoxelMapsData& target,
                            const Eigen::Isometry3f& target_T_source) const {
  if ((source_points == nullptr && point_count != 0) || point_count == 0 || !target.valid() ||
      !target_T_source.matrix().allFinite()) {
    return {};
  }
  const std::vector<float3> points = finite_points(source_points, point_count);
  if (points.empty() || points.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return {};
  }
  DeviceBuffer<float3> device_points(points.size());
  device_points.copy_from_host(points.data(), points.size());
  const DeviceTarget device_target(target);
  return score_pose_device(points, device_points, target, device_target, target_T_source);
}

LocalSearchResult BBS3D::search_local(const Eigen::Vector3f* source_points,
                                      std::size_t point_count,
                                      const LocalSearchTarget& target,
                                      const LocalSearchOptions& options) const {
  if ((source_points == nullptr && point_count != 0) ||
      !options.translation_half_window.allFinite() ||
      (options.translation_half_window.array() < 0.0F).any() ||
      !std::isfinite(options.min_yaw_offset) || !std::isfinite(options.max_yaw_offset) ||
      options.min_yaw_offset > options.max_yaw_offset ||
      options.max_yaw_offset - options.min_yaw_offset > kTwoPi + 1e-5F ||
      !std::isfinite(options.minimum_overlap) || options.minimum_overlap < 0.0 ||
      options.minimum_overlap > 1.0) {
    throw std::invalid_argument("Invalid GPU BBS local search options");
  }

  const auto start_time = std::chrono::steady_clock::now();
  LocalSearchResult result;
  if (target.voxelmaps != nullptr && target.voxelmaps->max_level > 3) {
    throw std::invalid_argument("GPU BBS supports voxel levels 0 through 3");
  }
  if (target.voxelmaps == nullptr || !target.voxelmaps->valid() ||
      !target.initial_target_T_source.matrix().allFinite()) {
    return result;
  }
  const std::vector<float3> points = finite_points(source_points, point_count);
  if (points.empty()) {
    return result;
  }
  if (points.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    throw std::length_error("GPU BBS source point count exceeds score representation");
  }

  const cpu::VoxelMapsData& voxelmaps = *target.voxelmaps;
  DeviceBuffer<float3> device_points(points.size());
  device_points.copy_from_host(points.data(), points.size());
  const DeviceTarget device_target(voxelmaps);
  const Eigen::Vector3f initial_rpy = rotation_to_rpy(target.initial_target_T_source.linear());
  const std::vector<YawInfo> yaw_info =
      make_yaw_info(voxelmaps, points, initial_rpy.z(), options.min_yaw_offset,
                    options.max_yaw_offset);
  const LeafIndexWindow leaf_window =
      make_leaf_window(target.initial_target_T_source.translation(),
                       options.translation_half_window, voxelmaps.min_level_resolution);
  result.initial_overlap =
      score_pose_device(points, device_points, voxelmaps, device_target,
                        target.initial_target_T_source)
          .overlap;

  const int top_level = voxelmaps.max_level;
  const float top_resolution = voxelmaps.resolution(top_level);
  std::array<int, 3> root_minimum{};
  std::array<int, 3> root_maximum{};
  for (int dimension = 0; dimension < 3; ++dimension) {
    root_minimum[static_cast<std::size_t>(dimension)] = static_cast<int>(std::floor(
        (target.initial_target_T_source.translation()[dimension] -
         options.translation_half_window[dimension]) /
        top_resolution));
    root_maximum[static_cast<std::size_t>(dimension)] = static_cast<int>(std::ceil(
        (target.initial_target_T_source.translation()[dimension] +
         options.translation_half_window[dimension]) /
        top_resolution));
  }

  std::vector<DiscreteTransformation<float>> roots;
  const int yaw_divisions = yaw_info[static_cast<std::size_t>(top_level)].num_divisions;
  for (int x = root_minimum[0]; x <= root_maximum[0]; ++x) {
    for (int y = root_minimum[1]; y <= root_maximum[1]; ++y) {
      for (int z = root_minimum[2]; z <= root_maximum[2]; ++z) {
        for (int yaw = 0; yaw < yaw_divisions; ++yaw) {
          roots.emplace_back(0, top_level, x, y, z, yaw);
        }
      }
    }
  }
  result.root_nodes = roots.size();
  if (roots.empty()) {
    result.elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                 start_time)
            .count();
    return result;
  }

  ScoreWorkspace score_workspace(std::max(roots.size(), 2U * kBranchStockSize), yaw_info);
  const std::vector<int>& root_scores =
      score_nodes(roots, device_points, static_cast<int>(points.size()), device_target,
                  voxelmaps.max_bucket_scan_count, initial_rpy.x(), initial_rpy.y(),
                  score_workspace);
  std::priority_queue<DiscreteTransformation<float>> queue;
  for (std::size_t index = 0; index < roots.size(); ++index) {
    roots[index].score = root_scores[index];
    queue.push(roots[index]);
  }

  const int score_threshold =
      static_cast<int>(std::floor(static_cast<double>(points.size()) * options.minimum_overlap));
  int best_score = score_threshold;
  int best_observed_leaf_score = -1;
  float best_pose_deviation = std::numeric_limits<float>::infinity();
  DiscreteTransformation<float> best_node;
  std::vector<DiscreteTransformation<float>> branch_stock;
  branch_stock.reserve(kBranchStockSize + 64U);
  std::vector<DiscreteTransformation<float>> children;

  const auto flush_branch_stock = [&]() {
    if (branch_stock.empty()) {
      return;
    }
    const std::vector<int>& scores =
        score_nodes(branch_stock, device_points, static_cast<int>(points.size()), device_target,
                    voxelmaps.max_bucket_scan_count, initial_rpy.x(), initial_rpy.y(),
                    score_workspace);
    for (std::size_t index = 0; index < branch_stock.size(); ++index) {
      branch_stock[index].score = scores[index];
      if (branch_stock[index].score >= best_score) {
        queue.push(branch_stock[index]);
      } else {
        ++result.pruned_nodes[static_cast<std::size_t>(branch_stock[index].level)];
      }
    }
    branch_stock.clear();
  };

  result.termination = SearchTermination::kQueueExhausted;
  while (!queue.empty() || !branch_stock.empty()) {
    if (queue.empty() || branch_stock.size() >= kBranchStockSize) {
      flush_branch_stock();
      if (queue.empty()) {
        break;
      }
    }

    DiscreteTransformation<float> node = queue.top();
    queue.pop();
    if (node.score < best_score) {
      ++result.pruned_nodes[static_cast<std::size_t>(node.level)];
      continue;
    }
    ++result.expanded_nodes[static_cast<std::size_t>(node.level)];
    if (node.is_leaf()) {
      if (!leaf_inside_window(node, leaf_window)) {
        ++result.pruned_nodes[0];
        continue;
      }
      best_observed_leaf_score = std::max(best_observed_leaf_score, node.score);
      const Eigen::Isometry3f pose =
          node_pose(node, voxelmaps, yaw_info, initial_rpy.x(), initial_rpy.y());
      const float pose_yaw = rotation_to_rpy(pose.linear()).z();
      const float yaw_deviation = normalize_yaw(pose_yaw - initial_rpy.z());
      const float pose_deviation =
          (pose.translation() - target.initial_target_T_source.translation()).squaredNorm() +
          yaw_deviation * yaw_deviation;
      if (node.score > best_score ||
          (node.score == best_score && pose_deviation < best_pose_deviation)) {
        best_score = node.score;
        best_pose_deviation = pose_deviation;
        best_node = node;
      }
      continue;
    }

    const int child_level = node.level - 1;
    const int child_yaw_divisions =
        yaw_info[static_cast<std::size_t>(child_level)].num_divisions;
    node.branch(children, child_level, child_yaw_divisions);
    for (const DiscreteTransformation<float>& child : children) {
      if (child_level == 0 && !leaf_inside_window(child, leaf_window)) {
        ++result.pruned_nodes[0];
        continue;
      }
      branch_stock.push_back(child);
    }
  }

  const bool passed = best_score > score_threshold && best_observed_leaf_score > score_threshold;
  if (passed) {
    result.accepted = true;
    result.target_id = target.target_id;
    result.target_T_source =
        node_pose(best_node, voxelmaps, yaw_info, initial_rpy.x(), initial_rpy.y());
    result.score.available = true;
    result.score.matched_points = static_cast<std::size_t>(best_score);
    result.score.evaluated_points = points.size();
    result.score.overlap =
        static_cast<double>(best_score) / static_cast<double>(points.size());
  } else if (best_observed_leaf_score >= 0) {
    result.score.available = true;
    result.score.matched_points = static_cast<std::size_t>(best_observed_leaf_score);
    result.score.evaluated_points = points.size();
    result.score.overlap = static_cast<double>(best_observed_leaf_score) /
                           static_cast<double>(points.size());
  }
  result.elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time)
          .count();
  return result;
}

}  // namespace gpu
