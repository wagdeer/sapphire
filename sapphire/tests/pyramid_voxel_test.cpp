#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "backend/storage/map_database.hpp"

namespace {

using Clock = std::chrono::steady_clock;

void check(bool condition, const char* message) {
  if (condition) {
    return;
  }
  std::cerr << "check failed: " << message << '\n';
  std::exit(1);
}

std::shared_ptr<sapphire::GaussianCloud> make_cloud() {
  auto cloud = std::make_shared<sapphire::GaussianCloud>();
  cloud->reserve(32U * 32U * 8U);
  for (int x = 0; x < 32; ++x) {
    for (int y = 0; y < 32; ++y) {
      for (int z = 0; z < 8; ++z) {
        sapphire::GaussianPoint point;
        point.mean = Eigen::Vector3f(static_cast<float>((x - 16) * 0.37 + y * 0.003),
                                     static_cast<float>((y - 16) * 0.41 + z * 0.005),
                                     static_cast<float>((z - 4) * 0.43 + static_cast<double>((x * y) % 7) * 0.007));
        point.covariance = Eigen::Matrix3f::Identity() / 9.0F;
        point.voxel_key = sapphire::VOXEL_LOC(x, y, z, 0);
        point.N = 1;
        point.regularize();
        cloud->emplace_back(std::move(point));
      }
    }
  }
  return cloud;
}

bool equivalent(const cpu::VoxelMapsData& left, const cpu::VoxelMapsData& right) {
  if (left.min_level_resolution != right.min_level_resolution || left.max_level != right.max_level ||
      left.max_bucket_scan_count != right.max_bucket_scan_count ||
      left.level_buckets.size() != right.level_buckets.size()) {
    return false;
  }
  for (std::size_t level = 0; level < left.level_buckets.size(); ++level) {
    const cpu::VoxelBuckets& left_buckets = left.level_buckets[level];
    const cpu::VoxelBuckets& right_buckets = right.level_buckets[level];
    if (left_buckets.size() != right_buckets.size()) {
      return false;
    }
    for (std::size_t index = 0; index < left_buckets.size(); ++index) {
      if ((left_buckets[index].array() != right_buckets[index].array()).any()) {
        return false;
      }
    }
  }
  return true;
}

double elapsed_milliseconds(Clock::time_point begin, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

std::size_t occupied_bucket_count(const cpu::VoxelBuckets& buckets) {
  std::size_t count = 0;
  for (const Eigen::Vector4i& bucket : buckets) {
    count += bucket.w() != 0;
  }
  return count;
}

}  // namespace

int main() {
  const std::shared_ptr<sapphire::GaussianCloud> cloud = make_cloud();
  sapphire::vvec<float, 3> means;
  means.reserve(cloud->size());
  for (const sapphire::GaussianPoint& point : *cloud) {
    means.emplace_back(point.mean);
  }
  cpu::VoxelMaps voxelmaps;
  voxelmaps.set_min_res(0.5F);
  voxelmaps.set_max_level(3);
  voxelmaps.create_voxelmaps(means.data(), means.size());
  const cpu::VoxelMapsData original = voxelmaps.release_data();
  cpu::VoxelMaps accessor_voxelmaps;
  accessor_voxelmaps.set_min_res(0.5F);
  accessor_voxelmaps.set_max_level(3);
  accessor_voxelmaps.create_voxelmaps(
      cloud->size(), [cloud](std::size_t index) { return (*cloud)[index].mean; });
  const cpu::VoxelMapsData accessor_built = accessor_voxelmaps.release_data();
  check(equivalent(original, accessor_built),
        "Gaussian-mean accessor builds the same occupancy as the legacy point range");
  check(original.level_buckets.size() == static_cast<std::size_t>(original.max_level + 1),
        "original-style tolerant occupancy stores every pyramid level");
  check(original.resolution(0) == 0.5F && original.resolution(3) == 4.0F,
        "pyramid resolutions are derived from a fixed factor of two");

  std::unordered_set<Eigen::Vector3i, cpu::VoxelMaps::VectorHash, cpu::VoxelMaps::VctorEqual> unique_exact;
  for (const sapphire::GaussianPoint& point : *cloud) {
    unique_exact.insert((point.mean.array() / original.min_level_resolution).floor().cast<int>());
  }
  check(occupied_bucket_count(original.level_buckets.front()) > unique_exact.size(),
        "level zero includes the original neighborhood tolerance");

  std::size_t dense_bytes = 0;
  for (const cpu::VoxelBuckets& buckets : original.level_buckets) {
    dense_bytes += buckets.size() * sizeof(Eigen::Vector4i);
  }

  constexpr int kCodecIterations = 20;
  std::string data;
  const Clock::time_point encode_begin = Clock::now();
  for (int iteration = 0; iteration < kCodecIterations; ++iteration) {
    std::ostringstream stream(std::ios::binary);
    cpu::save_voxelmaps(stream, original);
    data = stream.str();
  }
  const double encode_ms = elapsed_milliseconds(encode_begin, Clock::now()) / kCodecIterations;

  cpu::VoxelMapsData restored;
  const Clock::time_point decode_begin = Clock::now();
  for (int iteration = 0; iteration < kCodecIterations; ++iteration) {
    std::istringstream stream(data, std::ios::binary);
    restored = cpu::load_voxelmaps(stream);
  }
  const double decode_ms = elapsed_milliseconds(decode_begin, Clock::now()) / kCodecIterations;

  check(equivalent(original, restored), "sparse codec restores every pyramid level and bucket exactly");
  bool rejected_unversioned_payload = false;
  try {
    std::istringstream unversioned_stream(std::string(64, '\0'), std::ios::binary);
    static_cast<void>(cpu::load_voxelmaps(unversioned_stream));
  } catch (const std::runtime_error&) {
    rejected_unversioned_payload = true;
  }
  check(rejected_unversioned_payload, "unversioned pyramid payload is rejected without fallback");
  check(data.size() * 5U < dense_bytes * 4U, "pyramid voxel payload is at least 20 percent smaller than dense buckets");
  check(encode_ms < 50.0, "pyramid voxel encoding stays below 50 ms");
  check(decode_ms < 50.0, "pyramid voxel decoding stays below 50 ms");

  const std::filesystem::path database_path =
      std::filesystem::temp_directory_path() / ("sapphire-pyramid-voxel-test-" + std::to_string(Clock::now().time_since_epoch().count()) + ".db");
  double save_ms = 0.0;
  double load_ms = 0.0;
  {
    sapphire::LioFrame lio;
    lio.pcd = cloud;
    lio.timestamp = 1.0;
    sapphire::NavigationPath navigation;
    navigation.samples.push_back({1.0, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F});
    sapphire::OdomPoses odom_poses;
    odom_poses.push_back({1.0, lio.T_odom_base});
    sapphire::SubmapFrame submap(0, std::move(lio), 0, 0, 1, 0.0, 15.0, original, std::move(odom_poses),
                                 std::move(navigation), {});
    sapphire::LocalGrid local_grid(0.1F);
    sapphire::MapDatabase database(database_path.string());

    const Clock::time_point save_begin = Clock::now();
    database.saveSubmap(submap, local_grid);
    save_ms = elapsed_milliseconds(save_begin, Clock::now());

    constexpr int kLoadIterations = 20;
    const Clock::time_point load_begin = Clock::now();
    for (int iteration = 0; iteration < kLoadIterations; ++iteration) {
      restored = database.loadPyramidVoxel(1);
    }
    load_ms = elapsed_milliseconds(load_begin, Clock::now()) / kLoadIterations;
  }

  sqlite3* sqlite = nullptr;
  check(sqlite3_open_v2(database_path.c_str(), &sqlite, SQLITE_OPEN_READONLY, nullptr) == SQLITE_OK, "open database for pyramid voxel size check");
  sqlite3_stmt* size_statement = nullptr;
  check(sqlite3_prepare_v2(sqlite, "SELECT COUNT(*),COALESCE(SUM(length(data)),0) FROM PyramidVoxel;", -1, &size_statement, nullptr) == SQLITE_OK,
        "prepare pyramid voxel size query");
  check(sqlite3_step(size_statement) == SQLITE_ROW, "read pyramid voxel size");
  const sqlite3_int64 voxel_rows = sqlite3_column_int64(size_statement, 0);
  const sqlite3_int64 stored_bytes = sqlite3_column_int64(size_statement, 1);
  sqlite3_finalize(size_statement);
  sqlite3_close_v2(sqlite);

  check(equivalent(original, restored), "SQLite round trip restores the exact pyramid voxel structure");
  check(voxel_rows == 1 && stored_bytes == static_cast<sqlite3_int64>(data.size()), "SQLite stores one compact pyramid voxel payload");
  check(save_ms < 500.0, "submap and pyramid voxel persistence stays below 500 ms");
  check(load_ms < 50.0, "SQLite pyramid voxel load stays below 50 ms");

  const double storage_ratio = dense_bytes == 0 ? 0.0 : 100.0 * static_cast<double>(data.size()) / static_cast<double>(dense_bytes);
  std::cout << "Pyramid voxel benchmark: dense=" << dense_bytes << " B, sparse=" << data.size() << " B (" << storage_ratio
            << "%), encode=" << encode_ms << " ms, decode=" << decode_ms << " ms, save=" << save_ms << " ms, load=" << load_ms << " ms\n";

  std::filesystem::remove(database_path);
  std::filesystem::remove(database_path.string() + "-wal");
  std::filesystem::remove(database_path.string() + "-shm");
  return 0;
}
