#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

#include <sapphire/backend/submap.hpp>
#include <sapphire/backend/submap_arxiv.hpp>

namespace {

using namespace sapphire;

struct ScratchDirectory {
  ScratchDirectory() {
    auto pattern = (std::filesystem::temp_directory_path() / "sapphire-submap-test-XXXXXX").string();
    if (!mkdtemp(pattern.data())) throw std::runtime_error("cannot create submap test directory");
    path = pattern;
  }

  ~ScratchDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }

  std::filesystem::path path;
};

void expect(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}

PointCloudConstPtr planes(int cells) {
  auto cloud = std::make_shared<PointCloud>();
  for (int cell = 0; cell < cells; ++cell) {
    for (float x : {.1f, .5f, .9f}) {
      for (float y : {.1f, .5f, .9f}) {
        Point point{};
        point.x = float(cell % 10) + x;
        point.y = float(cell / 10) + y;
        point.z = .5f;
        cloud->push_back(point);
      }
    }
  }
  return cloud;
}

InertialFrame frame(size_t stamp, const PointCloudConstPtr& cloud) {
  return {{double(stamp), Isometry3d::Identity(), cloud},
          {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), -9.8 * Eigen::Vector3d::UnitZ()},
          {}};
}

}  // namespace

int main() {
  ScratchDirectory scratch;
  Config config;
  config.pgo.loop_closure.enabled = false;
  SubmapArxiv arxiv(scratch.path, config.pgo.loop_closure);
  Submap submap(arxiv, config.odometry, config.submap, config.pgo);
  const auto thin = planes(2);
  const auto dense = planes(80);

  // Cross both the old 20-frame seal trigger and the dense BA workspace limit.
  for (size_t i = 1; i <= 68; ++i) {
    expect(!submap.update(frame(i, thin)), "weak geometry must keep the block open");
  }
  expect(!submap.update(frame(69, dense)), "latest frame is the lookahead");
  expect(!submap.update(frame(70, dense)), "one dense view cannot certify a plane");
  expect(submap.update(frame(71, dense)), "seal as soon as both thresholds and the lookahead are present");
  submap.seal();
  expect(submap.stats().optimized_poses == 70, "first block must retain all seventy committed poses");
  expect(arxiv.read(0)->size() == 80, "the long block must archive its fitted geometry");

  for (size_t i = 72; i <= 90; ++i) {
    expect(!submap.update(frame(i, dense)), "minimum keyframe count still applies to dense blocks");
  }
  expect(submap.update(frame(91, dense)), "next block must reuse the lookahead");
  submap.seal();
  expect(submap.stats().optimized_poses == 90, "different block lengths must preserve distinct graph keys");
  expect(arxiv.read(1)->size() == 80, "the next block must archive its own geometry");

  submap.seal();
  const auto poses = submap.optimizedFrames();
  expect(poses.size() == 91 && poses.back().first == 91., "shutdown must retain the incomplete tail");
  expect(!std::filesystem::exists(scratch.path / "blocks" / "2.gauss"), "empty geometry must stay out of the archive");
}
