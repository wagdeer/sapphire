#include <sapphire/backend/submap_arxiv.hpp>
#include <sapphire/odometry/registration.hpp>

#include <stdexcept>

namespace sapphire {
struct RegistrationTestAccess {
  static const GaussianVoxelMap* update(Registration& registration, double stamp, const Isometry3d& pose, const VV3& scan) {
    return registration.updateRevisit(stamp, pose, scan);
  }
};
}  // namespace sapphire

namespace {

using namespace sapphire;

struct RevisitFixture {
  RevisitExchange exchange;
  LocalPlaneExchange local_planes;
  Registration tracking{Config{}, exchange, local_planes};
  VV3 scanner;
  std::unique_ptr<GaussianVoxelMap> map = std::make_unique<GaussianVoxelMap>();

  RevisitFixture() {
    // Three mutually orthogonal planes constrain all six pose directions.
    for (int normal = 0; normal < 3; ++normal) {
      for (int a = -3; a <= 3; ++a) {
        for (int b = -3; b <= 3; ++b) {
          Eigen::Vector3f center;
          center[normal] = -4.5f;
          center[(normal + 1) % 3] = a + .5f;
          center[(normal + 2) % 3] = b + .5f;
          scanner.push_back(center);
          if (!map->insert(key(center), {{center, Eigen::Vector3f::Unit(normal), .45f, .05f}, {}})) {
            throw std::runtime_error("test planes overlap");
          }
        }
      }
    }
    tracking.rememberKeyframe(10., Isometry3d::Identity());
  }

  void publish() {
    exchange.publish(std::make_unique<RevisitTarget>(RevisitTarget{std::move(map), 10., Isometry3d::Identity(), Isometry3d::Identity(), 1, 0, 0, 1}));
  }
};

}  // namespace

int main() {
  const auto identity = sapphire::Isometry3d::Identity();
  RevisitFixture immediate;
  immediate.publish();
  if (!sapphire::RegistrationTestAccess::update(immediate.tracking, 10.1, identity, immediate.scanner)) {
    throw std::runtime_error("first independent scan should verify the map");
  }

  RevisitFixture delayed;
  sapphire::RegistrationTestAccess::update(delayed.tracking, 10.1, identity, delayed.scanner);
  delayed.publish();
  auto moved = identity;
  moved.translation().x() = 2.;
  if (!sapphire::RegistrationTestAccess::update(delayed.tracking, 11., moved, {})) {
    throw std::runtime_error("late delivery must use the recorded scan");
  }
  if (!immediate.tracking.globalFromOdom().matrix().isApprox(delayed.tracking.globalFromOdom().matrix(), 1e-12)) {
    throw std::runtime_error("delivery time changed the alignment");
  }

  RevisitFixture expired;
  sapphire::RegistrationTestAccess::update(expired.tracking, 10.1, identity, expired.scanner);
  expired.publish();
  if (sapphire::RegistrationTestAccess::update(expired.tracking, 41., identity, expired.scanner)) {
    throw std::runtime_error("expired scan must not activate historical geometry");
  }
}
