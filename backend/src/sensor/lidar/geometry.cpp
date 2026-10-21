#include "backend/sensor/lidar/geometry.hpp"

#include <zlib.h>

#include <Eigen/Cholesky>
#include <cstring>
#include <stdexcept>
#include <type_traits>

#include "backend/database/compression.hpp"

namespace sapphire::backend::lidar {
namespace {
constexpr char magic[] = "SAPLID01";
constexpr std::size_t record_bytes = 85, header_bytes = 16;
template <class T>
void put(unsigned char*& out, T value) {
  static_assert(sizeof(T) == 4 || sizeof(T) == 8);
  using Bits = std::conditional_t<sizeof(T) == 4, uint32_t, uint64_t>;
  Bits bits;
  std::memcpy(&bits, &value, sizeof(T));
  for (size_t i = 0; i < sizeof(T); ++i) *out++ = static_cast<unsigned char>(bits >> (8 * i));
}
template <class T>
T get(const unsigned char*& in) {
  using Bits = std::conditional_t<sizeof(T) == 4, uint32_t, uint64_t>;
  Bits bits = 0;
  for (size_t i = 0; i < sizeof(T); ++i) bits |= Bits(*in++) << (8 * i);
  T value;
  std::memcpy(&value, &bits, sizeof(T));
  return value;
}
}  // namespace
void validate(const GaussianCloud& cloud) {
  if (cloud.empty() || cloud.size() > max_points) throw std::invalid_argument("LiDAR Gaussian count outside node budget");
  for (const auto& p : cloud)
    if (!p.valid() || p.voxel_key.level < 0 || p.voxel_key.level > 30 || !p.covariance.isApprox(p.covariance.transpose(), 1e-5f) ||
        p.covariance.llt().info() != Eigen::Success)
      throw std::invalid_argument("LiDAR node requires finite positive-definite Gaussian geometry");
}
cv::Mat encode(const GaussianCloud& cloud) {
  validate(cloud);
  cv::Mat raw(1, int(header_bytes + record_bytes * cloud.size()), CV_8UC1);
  auto* out = raw.ptr();
  std::memcpy(out, magic, 8);
  out += 8;
  put(out, uint64_t(cloud.size()));
  for (const auto& p : cloud) {
    put(out, p.voxel_key.x);
    put(out, p.voxel_key.y);
    put(out, p.voxel_key.z);
    put(out, int32_t(p.voxel_key.level));
    put(out, int32_t(p.N));
    for (int i = 0; i < 3; ++i) put(out, p.mean[i]);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) put(out, p.covariance(r, c));
    put(out, p.radius);
    *out++ = p.is_plane ? 1 : 0;
  }
  return compressData(raw);
}
GaussianCloudPtr decode(const cv::Mat& archive) {
  if (archive.empty()) return {};
  // Check the matrix trailer before allocating decompression output.
  if (archive.type() != CV_8UC1 || !archive.isContinuous() || archive.total() < 12) throw std::runtime_error("Invalid LiDAR archive");
  int shape[3];
  std::memcpy(shape, archive.ptr() + archive.total() - 12, 12);
  if (shape[0] != 1 || shape[1] < int(header_bytes) || shape[1] > int(header_bytes + max_points * record_bytes) || shape[2] != CV_8UC1)
    throw std::runtime_error("LiDAR archive exceeds bounded geometry schema");
  cv::Mat raw(1, shape[1], CV_8UC1);
  uLongf bytes = raw.total();
  if (uncompress(raw.data, &bytes, archive.data, archive.total() - 12) != Z_OK || bytes != raw.total())
    throw std::runtime_error("Corrupt LiDAR geometry compression");
  if (raw.type() != CV_8UC1 || !raw.isContinuous() || raw.total() < header_bytes || std::memcmp(raw.ptr(), magic, 8))
    throw std::runtime_error("Invalid LiDAR geometry header");
  const auto* in = raw.ptr() + 8;
  const auto count = get<uint64_t>(in);
  if (!count || count > max_points || raw.total() != header_bytes + count * record_bytes) throw std::runtime_error("Invalid LiDAR geometry length");
  auto cloud = std::make_shared<GaussianCloud>(size_t(count));
  for (auto& p : *cloud) {
    p.voxel_key.x = get<int64_t>(in);
    p.voxel_key.y = get<int64_t>(in);
    p.voxel_key.z = get<int64_t>(in);
    p.voxel_key.level = get<int32_t>(in);
    p.N = get<int32_t>(in);
    for (int i = 0; i < 3; ++i) p.mean[i] = get<float>(in);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) p.covariance(r, c) = get<float>(in);
    p.radius = get<float>(in);
    const auto plane = *in++;
    if (plane > 1) throw std::runtime_error("Invalid LiDAR plane flag");
    p.is_plane = plane;
  }
  validate(*cloud);
  return cloud;
}
}  // namespace sapphire::backend::lidar
