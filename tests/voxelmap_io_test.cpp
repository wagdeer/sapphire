#include <sstream>
#include <stdexcept>

#include <sapphire/mapping/voxelmap.hpp>

namespace {
using namespace sapphire;

void check(bool condition) {
  if (!condition) throw std::runtime_error("Gaussian voxel map I/O assertion failed");
}

void reject(const std::string& encoded) {
  std::istringstream input(encoded, std::ios::binary);
  try {
    GaussianVoxelMap::read(input);
  } catch (const std::runtime_error&) {
    return;
  }
  throw std::runtime_error("Expected malformed voxel map to be rejected");
}
}  // namespace

int main() {
  GaussianVoxelMap live;
  live.center = Eigen::Vector3f(-.5f, .5f, .25f);
  live.stamp = 1234567.25;
  live.version = 6;

  Moments data;
  for (std::size_t frame = 0; frame < 2; ++frame) {
    for (int x = 0; x < 4; ++x) {
      for (int y = 0; y < 4; ++y) data.add(Eigen::Vector3f(-.95f + x * .25f, .05f + y * .25f, .25f), frame);
    }
  }
  const auto voxel = data.fit();
  check(voxel.has_value());
  const VoxelKey slot{-1, 0, 0};
  check(live.insert(slot, *voxel));
  check(!live.insert(slot, *voxel));
  check(live.supportedPoints() == 32);

  std::ostringstream output(std::ios::binary);
  live.write(output);
  const std::string blob = output.str();
  check(blob.size() == 44 + 76);

  std::istringstream input(blob, std::ios::binary);
  const auto restored = GaussianVoxelMap::read(input);
  check(restored->size() == 1 && restored->cell(0) == slot);
  check(restored->center == live.center && restored->stamp == live.stamp && restored->version == live.version);
  check(restored->voxel(0).gaussian.covariance == voxel->gaussian.covariance);
  check(restored->supportedPoints() == 32 && restored->residentBytes() >= sizeof(GaussianVoxelMap));
  for (const Eigen::Vector3f query : {Eigen::Vector3f(-.5f, .5f, .3f), Eigen::Vector3f(.01f, .5f, .3f), Eigen::Vector3f(-.5f, .5f, 1.f)}) {
    const auto* before = live.lookup(query);
    const auto* after = restored->lookup(query);
    check((before == nullptr) == (after == nullptr));
    if (before) check(before->normal == after->normal && before->sigma == after->sigma);
  }
  std::ostringstream recoded(std::ios::binary);
  restored->write(recoded);
  check(recoded.str() == blob);

  reject(blob.substr(0, blob.size() - 1));
  reject(blob + "x");
  std::string wrong = blob;
  wrong[0] = 'X';
  reject(wrong);
  wrong = blob;
  wrong[40] = 2;
  reject(wrong + blob.substr(44));
}
