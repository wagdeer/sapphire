#include <sapphire/backend/submap_arxiv.hpp>
#include <sapphire/odometry/registration.hpp>

#include <cmath>
#include <stdexcept>

namespace sapphire {
struct RegistrationTestAccess {
  static iEskf::COV predictCovariance(Registration& registration) {
    MeasureGroup scan;
    FilterImuSample imu;
    imu.acc.z() = 9.80665f;
    for (int i = 0; i < 50; ++i) {
      imu.secs = i * 0.01;
      scan.imu.push_back(imu);
    }
    registration.process(std::move(scan));
    registration.filter_.StartTracking(imu);
    imu.secs += 0.01;
    registration.filter_.SetObsTime(imu.secs);
    if (!registration.filter_.Predict(imu)) {
      throw std::runtime_error("expected IMU propagation after initialization");
    }
    return registration.filter_.GetCov();
  }

  static const GaussianVoxelMap* update(Registration& registration, double stamp, const Isometry3d& pose, const VV3& scan) {
    return registration.updateRevisit(stamp, pose, scan);
  }
};
}  // namespace sapphire

namespace {

using namespace sapphire;

void testImuProcessNoise() {
  RevisitExchange exchange;
  LocalPlaneExchange local_planes;
  Config config;
  config.sensor.imu.noise = {0., 0., 0., 0.};
  Registration baseline(config, exchange, local_planes);
  config.sensor.imu.noise = {0.1, 0.2, 0.3, 0.4};
  Registration tuned(config, exchange, local_planes);

  const iEskf::COV difference = RegistrationTestAccess::predictCovariance(tuned) - RegistrationTestAccess::predictCovariance(baseline);
  const double expected[] = {0.1, 0., 0.2, 0.3, 0.4, 0.};
  for (int block = 0; block < 6; ++block) {
    for (int axis = 0; axis < 3; ++axis) {
      const auto index = block * 3 + axis;
      if (std::abs(difference(index, index) - expected[block] * 0.01 * 0.01) > 1e-8) {
        throw std::runtime_error("IMU process noise must change the corresponding covariance block");
      }
    }
  }
}

void testPosteriorCovarianceIsSymmetric() {
  iEskf filter;
  filter.SetInitialConditions({}, V3::Zero(), V3::Zero(), 1.f, V3(0.f, 0.f, -9.8f));
  auto prior = iEskf::COV::Identity().eval();
  // Matrix inversion can leave slightly different upper and lower entries.
  prior(0, 3) = .2f;
  prior(3, 0) = .2001f;
  filter.SetCov(prior);
  filter.UpdateObserve([](const iEskf::KFState&, Mat_66<scalar>& h, Vec_6<scalar>& r) {
    h.setIdentity();
    r.setZero();
  });
  const auto covariance = filter.GetCov();
  if (covariance != covariance.transpose()) {
    throw std::runtime_error("the observation update must leave a symmetric covariance");
  }
}

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
  testImuProcessNoise();
  testPosteriorCovarianceIsSymmetric();
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
