#include <sapphire/mapping/voxelmap.hpp>
#include <sapphire/utils.hpp>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <iostream>
#include <istream>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace sapphire {

VoxelKey key(const Eigen::Vector3f& p) {
  return {static_cast<int>(std::floor(p.x() / leaf)), static_cast<int>(std::floor(p.y() / leaf)), static_cast<int>(std::floor(p.z() / leaf))};
}

void Moments::add(const Eigen::Vector3f& p, std::size_t id) {
  const auto q = p.cast<double>();
  sum += q;
  second.noalias() += q * q.transpose();
  ++count;
  if (last_frame != id) {
    ++frames;
    last_frame = id;
  }
}

std::optional<FittedVoxel> Moments::fit() const {
  if (count < 6) return {};
  const Eigen::Vector3d center = sum / count;
  const Eigen::Matrix3d cov = second / count - center * center.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(cov);
  if (eigen.info() != Eigen::Success) return {};
  const auto& e = eigen.eigenvalues();
  if (e[1] < .012 || e[0] > .007 || e[0] > .18 * e[1]) return {};
  const VoxelPlane plane{center.cast<float>(), eigen.eigenvectors().col(0).cast<float>(), std::clamp(float(2.8 * std::sqrt(e[2])), .28f, 1.15f),
                         float(std::sqrt(std::max(.0009, e[0])))};
  const auto c = cov.cast<float>().eval();
  return FittedVoxel{plane,
                     {static_cast<std::uint32_t>(count), static_cast<std::uint32_t>(frames), {c(0, 0), c(0, 1), c(0, 2), c(1, 1), c(1, 2), c(2, 2)}}};
}

bool supported(const VoxelPlane& plane, const Eigen::Vector3f& point) {
  const Eigen::Vector3f offset = point - plane.center;
  const float normal = offset.dot(plane.normal);
  return std::abs(normal) <= .18f && offset.squaredNorm() - normal * normal <= plane.radius * plane.radius;
}

void* GaussianVoxelMap::IndexMemory::do_allocate(std::size_t bytes, std::size_t alignment) {
  void* memory = std::pmr::new_delete_resource()->allocate(bytes, alignment);
  bytes_ += bytes;
  return memory;
}

void GaussianVoxelMap::IndexMemory::do_deallocate(void* p, std::size_t bytes, std::size_t alignment) {
  std::pmr::new_delete_resource()->deallocate(p, bytes, alignment);
  bytes_ -= bytes;
}

void GaussianVoxelMap::reserve(std::size_t count) {
  index_.reserve(count);
  planes_.reserve(count);
  geometry_.reserve(count);
}

bool GaussianVoxelMap::insert(const VoxelKey& cell, const FittedVoxel& voxel) {
  if (planes_.size() >= std::numeric_limits<std::uint32_t>::max()) throw std::length_error("plane index overflow");
  const auto [where, inserted] = index_.try_emplace(cell, static_cast<std::uint32_t>(planes_.size()));
  if (!inserted) return false;
  try {
    geometry_.push_back({cell, voxel.gaussian});
    try {
      planes_.push_back(voxel.plane);
    } catch (...) {
      geometry_.pop_back();
      throw;
    }
  } catch (...) {
    index_.erase(where);
    throw;
  }
  return true;
}

std::size_t GaussianVoxelMap::residentBytes() const {
  return sizeof(GaussianVoxelMap) + index_memory_.bytes() + planes_.capacity() * sizeof(VoxelPlane) + geometry_.capacity() * sizeof(Geometry);
}

std::uint64_t GaussianVoxelMap::supportedPoints() const {
  std::uint64_t total = 0;
  for (const auto& entry : geometry_) total += entry.gaussian.points;
  return total;
}

const VoxelPlane* GaussianVoxelMap::lookup(const Eigen::Vector3f& p) const {
  const auto cell = key(p);
  const auto test = [&](const VoxelKey& k) -> const VoxelPlane* {
    const auto hit = index_.find(k);
    if (hit == index_.end()) return nullptr;
    const VoxelPlane& plane = planes_[hit->second];
    return supported(plane, p) ? &plane : nullptr;
  };
  if (const auto* local = test(cell)) return local;
  const Eigen::Vector3f fraction = p / leaf - Eigen::Vector3f(cell.x, cell.y, cell.z);
  const std::array<VoxelKey, 3> neighbors = {VoxelKey{cell.x + (fraction.x() < .5f ? -1 : 1), cell.y, cell.z},
                                             VoxelKey{cell.x, cell.y + (fraction.y() < .5f ? -1 : 1), cell.z},
                                             VoxelKey{cell.x, cell.y, cell.z + (fraction.z() < .5f ? -1 : 1)}};
  const VoxelPlane* best = nullptr;
  float distance = std::numeric_limits<float>::infinity();
  for (const VoxelKey& k : neighbors) {
    const auto* other = test(k);
    if (!other) continue;
    const float candidate = std::abs(other->normal.dot(p - other->center));
    if (candidate < distance) {
      distance = candidate;
      best = other;
    }
  }
  return best;
}

const VoxelPlane* GaussianVoxelMap::coarseLookup(const Eigen::Vector3f& p) const {
  const auto k = key(p);
  const VoxelPlane* best = nullptr;
  float cost = std::numeric_limits<float>::infinity();
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        const auto entry = index_.find({k.x + x, k.y + y, k.z + z});
        if (entry == index_.end()) continue;
        const auto& plane = planes_[entry->second];
        const auto offset = (p - plane.center).eval();
        const float plane_distance = offset.dot(plane.normal);
        const float tangential = std::max(0.f, offset.squaredNorm() - plane_distance * plane_distance);
        const float radius = plane.radius + .25f;
        if (std::abs(plane_distance) > .5f || tangential > radius * radius) continue;
        const float weight = plane_distance * plane_distance + .10f * tangential;
        if (weight < cost) {
          cost = weight;
          best = &plane;
        }
      }
    }
  }
  return best;
}

namespace {
#if defined(__BYTE_ORDER__)
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "Gaussian voxel maps use little-endian encoding");
#endif
static_assert(sizeof(float) == 4 && sizeof(double) == 8 && std::numeric_limits<float>::is_iec559 && std::numeric_limits<double>::is_iec559);
constexpr std::array<char, 8> magic{'S', 'A', 'P', 'G', 'A', 'U', 'S', '1'};
// A 1 m cell grid over the current 35 m radius block fits inside a 72-cell cube.
constexpr std::uint32_t max_cells = 72 * 72 * 72;

template <class T>
void writeScalar(std::ostream& out, T value) {
  if (!out.write(reinterpret_cast<const char*>(&value), sizeof value)) throw std::runtime_error("Cannot write Gaussian voxel map");
}

template <class T>
T readScalar(std::istream& in) {
  T value;
  if (!in.read(reinterpret_cast<char*>(&value), sizeof value)) throw std::runtime_error("Truncated Gaussian voxel map");
  return value;
}

void writePoint(std::ostream& out, const Eigen::Vector3f& point) {
  for (int i = 0; i < 3; ++i) writeScalar(out, point[i]);
}

Eigen::Vector3f readPoint(std::istream& in) {
  Eigen::Vector3f point;
  for (int i = 0; i < 3; ++i) point[i] = readScalar<float>(in);
  return point;
}

bool validVoxel(const FittedVoxel& voxel) {
  const auto& plane = voxel.plane;
  const auto& gaussian = voxel.gaussian;
  return plane.center.allFinite() && plane.normal.allFinite() && std::abs(plane.normal.squaredNorm() - 1.f) < .01f && std::isfinite(plane.radius) &&
         plane.radius > 0.f && std::isfinite(plane.sigma) && plane.sigma > 0.f && gaussian.points >= 6 && gaussian.frames >= 2 &&
         gaussian.frames <= gaussian.points &&
         std::all_of(gaussian.covariance.begin(), gaussian.covariance.end(), [](float v) { return std::isfinite(v); });
}
}  // namespace

void GaussianVoxelMap::write(std::ostream& out) const {
  if (!size() || size() > max_cells || !center.allFinite() || !std::isfinite(stamp))
    throw std::runtime_error("Invalid Gaussian map metadata");
  out.write(magic.data(), magic.size());
  writeScalar(out, leaf);
  writePoint(out, center);
  writeScalar(out, stamp);
  writeScalar(out, static_cast<std::uint64_t>(version));
  writeScalar(out, static_cast<std::uint32_t>(size()));
  for (std::size_t i = 0; i < size(); ++i) {
    const VoxelKey& voxel_key = cell(i);
    const auto fitted = voxel(i);
    if (!validVoxel(fitted)) throw std::runtime_error("Invalid fitted voxel");
    writeScalar(out, static_cast<std::int32_t>(voxel_key.x));
    writeScalar(out, static_cast<std::int32_t>(voxel_key.y));
    writeScalar(out, static_cast<std::int32_t>(voxel_key.z));
    writeScalar(out, fitted.gaussian.points);
    writeScalar(out, fitted.gaussian.frames);
    writePoint(out, fitted.plane.center);
    for (float value : fitted.gaussian.covariance) writeScalar(out, value);
    writePoint(out, fitted.plane.normal);
    writeScalar(out, fitted.plane.radius);
    writeScalar(out, fitted.plane.sigma);
  }
}

std::unique_ptr<GaussianVoxelMap> GaussianVoxelMap::read(std::istream& in) {
  std::array<char, magic.size()> header{};
  if (!in.read(header.data(), header.size()) || header != magic) throw std::runtime_error("Unknown Gaussian voxel map format");
  if (readScalar<float>(in) != leaf) throw std::runtime_error("Unsupported voxel resolution");
  const Eigen::Vector3f center = readPoint(in);
  const double stamp = readScalar<double>(in);
  const auto version = readScalar<std::uint64_t>(in);
  const auto cells = readScalar<std::uint32_t>(in);
  if (!center.allFinite() || !std::isfinite(stamp) || version > std::numeric_limits<std::size_t>::max() || !cells || cells > max_cells)
    throw std::runtime_error("Invalid Gaussian map metadata");

  auto map = std::make_unique<GaussianVoxelMap>();
  map->center = center;
  map->stamp = stamp;
  map->version = static_cast<std::size_t>(version);
  map->reserve(cells);
  for (std::uint32_t i = 0; i < cells; ++i) {
    const VoxelKey cell{readScalar<std::int32_t>(in), readScalar<std::int32_t>(in), readScalar<std::int32_t>(in)};
    FittedVoxel voxel;
    voxel.gaussian.points = readScalar<std::uint32_t>(in);
    voxel.gaussian.frames = readScalar<std::uint32_t>(in);
    voxel.plane.center = readPoint(in);
    for (auto& value : voxel.gaussian.covariance) value = readScalar<float>(in);
    voxel.plane.normal = readPoint(in);
    voxel.plane.radius = readScalar<float>(in);
    voxel.plane.sigma = readScalar<float>(in);
    if (!validVoxel(voxel) || !map->insert(cell, voxel)) throw std::runtime_error("Invalid or duplicate fitted voxel");
  }
  if (in.peek() != std::char_traits<char>::eof()) throw std::runtime_error("Trailing data after Gaussian voxel map");
  return map;
}

namespace {
constexpr int kInertialDimension = 15;
constexpr int kPoseDimension = 6;
constexpr double kGravityMagnitude = 9.8;
constexpr double kImuFactorWeight = .0002;
const Eigen::Matrix3d I33 = Eigen::Matrix3d::Identity();
const Mat_66<double> noiseMeas = (Mat_61<double>() << .01, .01, .01, 2., 2., 2.).finished().asDiagonal();
const Mat_66<double> noiseWalk = .0001 * Mat_66<double>::Identity();
#define PLM(a) std::vector<Eigen::Matrix<double, a, a>, Eigen::aligned_allocator<Eigen::Matrix<double, a, a>>>
#define PLV(a) std::vector<Eigen::Matrix<double, a, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, a, 1>>>
#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

Eigen::Matrix3d bundleRotationExp(const Eigen::Vector3d &ang) {
  double ang_norm = ang.norm();
  if (ang_norm >= 1e-11) {
    Eigen::Vector3d r_axis = ang / ang_norm;
    Eigen::Matrix3d K;
    K << SKEW_SYM_MATRX(r_axis);
    return I33 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
  }

  return I33;
}

Eigen::Matrix3d bundleRotationExp(const Eigen::Vector3d &ang_vel, const double &dt) {
  double ang_vel_norm = ang_vel.norm();
  if (ang_vel_norm > 1e-7) {
    Eigen::Vector3d r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix3d K;

    K << SKEW_SYM_MATRX(r_axis);
    double r_ang = ang_vel_norm * dt;

    return I33 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
  }

  return I33;
}

Eigen::Vector3d bundleRotationLog(const Eigen::Matrix3d &R) {
  double theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  Eigen::Vector3d K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d Omega;
  Omega << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return Omega;
}

Eigen::Matrix3d rightJacobian(Eigen::Vector3d vec) {
  double ang = vec.norm();

  if (ang < 1e-9) {
    return I33;
  } else {
    vec /= ang;
    double ra = sin(ang) / ang;
    return ra * I33 + (1 - ra) * vec * vec.transpose() - (1 - cos(ang)) / ang * skew(vec);
  }
}

Eigen::Matrix3d inverseRightJacobian(const Eigen::Matrix3d &rotR) {
  Eigen::AngleAxisd rot_vec(rotR);
  Eigen::Vector3d axi = rot_vec.axis();
  double ang = rot_vec.angle();

  if (ang < 1e-9) {
    return I33;
  } else {
    double ctt = ang / 2 / tan(ang / 2);
    return ctt * I33 + (1 - ctt) * axi * axi.transpose() + ang / 2 * skew(axi);
  }
}

struct BundleState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double t;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d bg;
  Eigen::Vector3d ba;
  Eigen::Vector3d g;
  Eigen::Matrix<double, kInertialDimension, kInertialDimension> cov;

  BundleState() { setZero(); }

  BundleState(double _t, const Eigen::Matrix3d &_R, const Eigen::Vector3d &_p, const Eigen::Vector3d &_v, const Eigen::Vector3d &_bg,
              const Eigen::Vector3d &_ba, const Eigen::Vector3d &_g = Eigen::Vector3d(0, 0, -kGravityMagnitude))
      : t(_t), R(_R), p(_p), v(_v), bg(_bg), ba(_ba), g(_g) {}

  BundleState &operator+=(const Eigen::Matrix<double, kInertialDimension, 1> &ist) {
    this->R = this->R * bundleRotationExp(ist.block<3, 1>(0, 0));
    this->p += ist.block<3, 1>(3, 0);
    this->v += ist.block<3, 1>(6, 0);
    this->bg += ist.block<3, 1>(9, 0);
    this->ba += ist.block<3, 1>(12, 0);
    return *this;
  }

  Eigen::Matrix<double, kInertialDimension, 1> operator-(const BundleState &b) {
    Eigen::Matrix<double, kInertialDimension, 1> a;
    a.block<3, 1>(0, 0) = bundleRotationLog(b.R.transpose() * this->R);
    a.block<3, 1>(3, 0) = this->p - b.p;
    a.block<3, 1>(6, 0) = this->v - b.v;
    a.block<3, 1>(9, 0) = this->bg - b.bg;
    a.block<3, 1>(12, 0) = this->ba - b.ba;
    return a;
  }

  BundleState &operator=(const BundleState &b) {
    this->R = b.R;
    this->p = b.p;
    this->v = b.v;
    this->bg = b.bg;
    this->ba = b.ba;
    this->g = b.g;
    this->t = b.t;
    this->cov = b.cov;
    return *this;
  }

  void setZero() {
    t = 0;
    R.setIdentity();
    p.setZero();
    v.setZero();
    bg.setZero();
    ba.setZero();
    cov.setIdentity();
    cov *= 0.0001;
    cov.block<6, 6>(9, 9) = Mat_66<double>::Identity() * 0.00001;
  }
};

class PointCluster {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d P;
  Eigen::Vector3d v;
  int N;

  PointCluster() {
    P.setZero();
    v.setZero();
    N = 0;
  }

  void clear() {
    P.setZero();
    v.setZero();
    N = 0;
  }

  void push(const Eigen::Vector3d &vec) {
    N++;
    P += vec * vec.transpose();
    v += vec;
  }

  Eigen::Matrix3d cov() {
    Eigen::Vector3d center = v / N;
    return P / N - center * center.transpose();
  }

  PointCluster &operator+=(const PointCluster &sigv) {
    this->P += sigv.P;
    this->v += sigv.v;
    this->N += sigv.N;

    return *this;
  }

  PointCluster &operator-=(const PointCluster &sigv) {
    this->P -= sigv.P;
    this->v -= sigv.v;
    this->N -= sigv.N;

    return *this;
  }

  void transform(const PointCluster &sigv, const BundleState &stat) {
    N = sigv.N;
    v = stat.R * sigv.v + N * stat.p;
    Eigen::Matrix3d rp = stat.R * sigv.v * stat.p.transpose();
    P = stat.R * sigv.P * stat.R.transpose() + rp + rp.transpose() + N * stat.p * stat.p.transpose();
  }
};

class BundlePreintegration {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d R_delta;
  Eigen::Vector3d p_delta, v_delta;
  Eigen::Vector3d bg, ba;

  Eigen::Matrix3d R_bg;
  Eigen::Matrix3d p_bg, p_ba;
  Eigen::Matrix3d v_bg, v_ba;

  double dtime;

  Eigen::Vector3d dbg, dba;
  Eigen::Vector3d dbg_buf, dba_buf;

  Eigen::Matrix<double, kInertialDimension, kInertialDimension> cov, information;

  BundlePreintegration(const Eigen::Vector3d &bg1 = Eigen::Vector3d::Zero(), const Eigen::Vector3d &ba1 = Eigen::Vector3d::Zero()) {
    bg = bg1;
    ba = ba1;
    R_delta.setIdentity();
    p_delta.setZero();
    v_delta.setZero();

    R_bg.setZero();
    p_bg.setZero();
    p_ba.setZero();
    v_bg.setZero();
    v_ba.setZero();

    dtime = 0;

    dbg.setZero();
    dba.setZero();
    dbg_buf.setZero();
    dba_buf.setZero();

    cov.setZero();
  }

  void add_imu(Eigen::Vector3d &cur_gyr, Eigen::Vector3d &cur_acc, double dt) {
    dtime += dt;
    Eigen::Matrix3d R_inc = bundleRotationExp(cur_gyr, dt);
    Eigen::Matrix3d R_jr(rightJacobian(cur_gyr * dt));

    Eigen::Matrix3d R_dt = dt * R_delta;
    Eigen::Matrix3d R_dt2_2 = 0.5 * dt * dt * R_delta;

    Eigen::Matrix3d acc_skew;
    acc_skew << SKEW_SYM_MATRX(cur_acc);

    p_ba = p_ba + v_ba * dt - R_dt2_2;
    p_bg = p_bg + v_bg * dt - R_dt2_2 * acc_skew * R_bg;
    v_ba = v_ba - R_dt;
    v_bg = v_bg - R_dt * acc_skew * R_bg;
    R_bg = R_inc.transpose() * R_bg - R_jr * dt;

    Eigen::Matrix<double, 9, 9> A;
    Eigen::Matrix<double, 9, 6> B;
    A.setIdentity();
    B.setZero();

    A.block<3, 3>(0, 0) = R_inc.transpose();
    A.block<3, 3>(3, 0) = -R_dt2_2 * acc_skew;
    A.block<3, 3>(3, 6) = I33 * dt;
    A.block<3, 3>(6, 0) = -R_dt * acc_skew;

    B.block<3, 3>(0, 0) = R_jr * dt;
    B.block<3, 3>(3, 3) = R_dt2_2;
    B.block<3, 3>(6, 3) = R_dt;

    cov.block<9, 9>(0, 0) = A * cov.block<9, 9>(0, 0) * A.transpose() + B * noiseMeas * B.transpose();
    cov.block<6, 6>(9, 9) += noiseWalk * dt;

    p_delta += v_delta * dt + R_dt2_2 * cur_acc;
    v_delta += R_dt * cur_acc;
    R_delta = R_delta * R_inc;
  }

  double give_evaluate(BundleState &st1, BundleState &st2, Eigen::MatrixXd &jtj, Eigen::VectorXd &gg, bool jac_enable) {
    Eigen::Matrix<double, kInertialDimension, kInertialDimension> joca, jocb;
    Eigen::Matrix<double, kInertialDimension, 1> rr;
    joca.setZero();
    jocb.setZero();
    rr.setZero();

    Eigen::Matrix3d R_correct = R_delta * bundleRotationExp(R_bg * dbg);
    Eigen::Vector3d t_correct = p_delta + p_bg * dbg + p_ba * dba;
    Eigen::Vector3d v_correct = v_delta + v_bg * dbg + v_ba * dba;

    Eigen::Matrix3d res_r = R_correct.transpose() * st1.R.transpose() * st2.R;
    Eigen::Vector3d exp_v = st1.R.transpose() * (st2.v - st1.v - dtime * st1.g);
    Eigen::Vector3d res_v = exp_v - v_correct;
    Eigen::Vector3d exp_t = st1.R.transpose() * (st2.p - st1.p - st1.v * dtime - 0.5 * dtime * dtime * st1.g);
    Eigen::Vector3d res_t = exp_t - t_correct;

    Eigen::Vector3d res_bg = st2.bg - st1.bg;
    Eigen::Vector3d res_ba = st2.ba - st1.ba;

    double b_wei = 1;

    rr.block<3, 1>(0, 0) = bundleRotationLog(res_r);
    rr.block<3, 1>(3, 0) = res_t;
    rr.block<3, 1>(6, 0) = res_v;
    rr.block<3, 1>(9, 0) = res_bg * b_wei;
    rr.block<3, 1>(12, 0) = res_ba * b_wei;

    const auto &cov_inv = information;

    if (jac_enable) {
      Eigen::Matrix3d JR_inv = inverseRightJacobian(res_r);
      joca.block<3, 3>(0, 0) = -JR_inv * st2.R.transpose() * st1.R;
      jocb.block<3, 3>(0, 0) = JR_inv;
      joca.block<3, 3>(0, 9) = -JR_inv * res_r.transpose() * rightJacobian(R_bg * dbg) * R_bg;

      joca.block<3, 3>(3, 0) = skew(exp_t);
      joca.block<3, 3>(3, 3) = -st1.R.transpose();
      joca.block<3, 3>(3, 6) = -st1.R.transpose() * dtime;
      joca.block<3, 3>(3, 9) = -p_bg;
      joca.block<3, 3>(3, 12) = -p_ba;
      jocb.block<3, 3>(3, 3) = st1.R.transpose();

      joca.block<3, 3>(6, 0) = skew(exp_v);
      joca.block<3, 3>(6, 6) = -st1.R.transpose();
      joca.block<3, 3>(6, 9) = -v_bg;
      joca.block<3, 3>(6, 12) = -v_ba;
      jocb.block<3, 3>(6, 6) = st1.R.transpose();

      joca.block<3, 3>(9, 9) = -I33 * b_wei;
      joca.block<3, 3>(12, 12) = -I33 * b_wei;
      jocb.block<3, 3>(9, 9) = I33 * b_wei;
      jocb.block<3, 3>(12, 12) = I33 * b_wei;

      Eigen::Matrix<double, kInertialDimension, 2 * kInertialDimension> joc;
      joc.block<kInertialDimension, kInertialDimension>(0, 0) = joca;
      joc.block<kInertialDimension, kInertialDimension>(0, kInertialDimension) = jocb;

      jtj = joc.transpose() * cov_inv * joc;
      gg = joc.transpose() * cov_inv * rr;
    }

    return rr.dot(cov_inv * rr);
  }

  double give_evaluate_g(BundleState &st1, BundleState &st2, Eigen::MatrixXd &jtj, Eigen::VectorXd &gg, bool jac_enable) {
    Eigen::Matrix<double, kInertialDimension, kInertialDimension> joca, jocb;
    Eigen::Matrix<double, kInertialDimension, 1> rr;
    joca.setZero();
    jocb.setZero();
    rr.setZero();
    Eigen::Matrix<double, kInertialDimension, 3> jocg;
    jocg.setZero();

    Eigen::Matrix3d R_correct = R_delta * bundleRotationExp(R_bg * dbg);
    Eigen::Vector3d t_correct = p_delta + p_bg * dbg + p_ba * dba;
    Eigen::Vector3d v_correct = v_delta + v_bg * dbg + v_ba * dba;

    Eigen::Matrix3d res_r = R_correct.transpose() * st1.R.transpose() * st2.R;
    Eigen::Vector3d exp_v = st1.R.transpose() * (st2.v - st1.v - dtime * st1.g);
    Eigen::Vector3d res_v = exp_v - v_correct;
    Eigen::Vector3d exp_t = st1.R.transpose() * (st2.p - st1.p - st1.v * dtime - 0.5 * dtime * dtime * st1.g);
    Eigen::Vector3d res_t = exp_t - t_correct;

    Eigen::Vector3d res_bg = st2.bg - st1.bg;
    Eigen::Vector3d res_ba = st2.ba - st1.ba;

    double b_wei = 1;

    rr.block<3, 1>(0, 0) = bundleRotationLog(res_r);
    rr.block<3, 1>(3, 0) = res_t;
    rr.block<3, 1>(6, 0) = res_v;
    rr.block<3, 1>(9, 0) = res_bg * b_wei;
    rr.block<3, 1>(12, 0) = res_ba * b_wei;

    Eigen::Matrix<double, 15, 15> cov_inv = cov.inverse();

    if (jac_enable) {
      Eigen::Matrix3d JR_inv = inverseRightJacobian(res_r);
      joca.block<3, 3>(0, 0) = -JR_inv * st2.R.transpose() * st1.R;
      jocb.block<3, 3>(0, 0) = JR_inv;
      joca.block<3, 3>(0, 9) = -JR_inv * res_r.transpose() * rightJacobian(R_bg * dbg) * R_bg;

      joca.block<3, 3>(3, 0) = skew(exp_t);
      joca.block<3, 3>(3, 3) = -st1.R.transpose();
      joca.block<3, 3>(3, 6) = -st1.R.transpose() * dtime;
      joca.block<3, 3>(3, 9) = -p_bg;
      joca.block<3, 3>(3, 12) = -p_ba;
      jocb.block<3, 3>(3, 3) = st1.R.transpose();

      joca.block<3, 3>(6, 0) = skew(exp_v);
      joca.block<3, 3>(6, 6) = -st1.R.transpose();
      joca.block<3, 3>(6, 9) = -v_bg;
      joca.block<3, 3>(6, 12) = -v_ba;
      jocb.block<3, 3>(6, 6) = st1.R.transpose();

      joca.block<3, 3>(9, 9) = -I33 * b_wei;
      joca.block<3, 3>(12, 12) = -I33 * b_wei;
      jocb.block<3, 3>(9, 9) = I33 * b_wei;
      jocb.block<3, 3>(12, 12) = I33 * b_wei;

      jocg.block<3, 3>(3, 0) = st1.R.transpose() * (-0.5 * dtime * dtime);
      jocg.block<3, 3>(6, 0) = st1.R.transpose() * (-dtime);

      Eigen::Matrix<double, kInertialDimension, 2 * kInertialDimension + 3> joc;
      joc.block<kInertialDimension, kInertialDimension>(0, 0) = joca;
      joc.block<kInertialDimension, kInertialDimension>(0, kInertialDimension) = jocb;
      joc.block<kInertialDimension, 3>(0, 2 * kInertialDimension) = jocg;

      jtj = joc.transpose() * cov_inv * joc;
      gg = joc.transpose() * cov_inv * rr;
    }

    return rr.dot(cov_inv * rr);
  }

  void update_state(const Eigen::Matrix<double, kInertialDimension, 1> &dxi) {
    dbg_buf = dbg;
    dba_buf = dba;

    dbg += dxi.block<3, 1>(9, 0);
    dba += dxi.block<3, 1>(12, 0);
  }

  void merge(BundlePreintegration &imu2) {
    p_bg += v_bg * imu2.dtime + R_delta * (imu2.p_bg - skew(imu2.p_delta) * R_bg);
    p_ba += v_ba * imu2.dtime + R_delta * imu2.p_ba;
    v_bg += R_delta * (imu2.v_bg - skew(imu2.v_delta) * R_bg);
    v_ba += R_delta * imu2.v_ba;
    R_bg = imu2.R_delta.transpose() * R_bg + imu2.R_bg;

    Eigen::Matrix<double, kInertialDimension, kInertialDimension> Ai, Bi;
    Ai.setIdentity();
    Bi.setIdentity();
    Ai.block<3, 3>(0, 0) = imu2.R_delta.transpose();
    Ai.block<3, 3>(3, 0) = -R_delta * skew(imu2.p_delta);
    Ai.block<3, 3>(3, 6) = I33 * imu2.dtime;
    Ai.block<3, 3>(6, 0) = -R_delta * skew(imu2.v_delta);

    Bi.block<3, 3>(3, 3) = R_delta;
    Bi.block<3, 3>(6, 6) = R_delta;
    cov = Ai * cov * Ai.transpose() + Bi * imu2.cov * Bi.transpose();

    p_delta += v_delta * imu2.dtime + R_delta * imu2.p_delta;
    v_delta += R_delta * imu2.v_delta;
    R_delta = R_delta * imu2.R_delta;

    dtime += imu2.dtime;
  }
};

struct FrameMoments {
  std::size_t frame;
  PointCluster moment;
};

class VoxelLidarFactors {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<PointCluster> sig_vecs;
  std::vector<std::vector<FrameMoments>> plvec_voxels;
  std::vector<double> coeffs;
  PLV(3) eig_values;
  PLM(3) eig_vectors;
  std::vector<PointCluster> pcr_adds;
  int win_size;

  VoxelLidarFactors(int _w) : win_size(_w) {}

  void push_voxel(std::vector<FrameMoments> &&vec_orig, PointCluster &fix, double coe, Eigen::Vector3d &eig_value, Eigen::Matrix3d &eig_vector,
                  PointCluster &pcr_add) {
    plvec_voxels.push_back(std::move(vec_orig));
    sig_vecs.push_back(fix);
    coeffs.push_back(coe);
    eig_values.push_back(eig_value);
    eig_vectors.push_back(eig_vector);
    pcr_adds.push_back(pcr_add);
  }

  void acc_evaluate2(const std::vector<BundleState> &xs, int head, int end, Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT, double &residual) {
    Hess.setZero();
    JacT.setZero();
    residual = 0;
    const int kk = 0;

    PLV(3) viRiTuk(win_size);
    PLM(3) viRiTukukT(win_size);

    std::vector<Eigen::Matrix<double, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6>>> Auk(win_size);
    Eigen::Matrix3d umumT;

    for (int a = head; a < end; a++) {
      const auto &groups = plvec_voxels[a];
      double coe = coeffs[a];

      Eigen::Vector3d lmbd = eig_values[a];
      Eigen::Matrix3d U = eig_vectors[a];
      int NN = pcr_adds[a].N;
      Eigen::Vector3d vBar = pcr_adds[a].v / NN;

      Eigen::Vector3d u[3] = {U.col(0), U.col(1), U.col(2)};
      Eigen::Vector3d &uk = u[kk];
      Eigen::Matrix3d ukukT = uk * uk.transpose();
      umumT.setZero();
      for (int i = 0; i < 3; i++)
        if (i != kk) umumT += 2.0 / (lmbd[kk] - lmbd[i]) * u[i] * u[i].transpose();

      for (const auto &group : groups) {
        const auto i = group.frame;
        Eigen::Matrix3d Pi = group.moment.P;
        Eigen::Vector3d vi = group.moment.v;
        Eigen::Matrix3d Ri = xs[i].R;
        double ni = group.moment.N;

        Eigen::Matrix3d vihat;
        vihat << SKEW_SYM_MATRX(vi);
        Eigen::Vector3d RiTuk = Ri.transpose() * uk;
        Eigen::Matrix3d RiTukhat;
        RiTukhat << SKEW_SYM_MATRX(RiTuk);

        Eigen::Vector3d PiRiTuk = Pi * RiTuk;
        viRiTuk[i] = vihat * RiTuk;
        viRiTukukT[i] = viRiTuk[i] * uk.transpose();

        Eigen::Vector3d ti_v = xs[i].p - vBar;
        double ukTti_v = uk.dot(ti_v);

        Eigen::Matrix3d combo1 = skew(PiRiTuk) + vihat * ukTti_v;
        Eigen::Vector3d combo2 = Ri * vi + ni * ti_v;
        Auk[i].block<3, 3>(0, 0) = (Ri * Pi + ti_v * vi.transpose()) * RiTukhat - Ri * combo1;
        Auk[i].block<3, 3>(0, 3) = combo2 * uk.transpose() + combo2.dot(uk) * I33;
        Auk[i] /= NN;

        const Mat_61<double> &jjt = Auk[i].transpose() * uk;
        JacT.block<6, 1>(6 * i, 0) += coe * jjt;

        const Eigen::Matrix3d &HRt = 2.0 / NN * (1.0 - ni / NN) * viRiTukukT[i];
        Mat_66<double> Hb = Auk[i].transpose() * umumT * Auk[i];
        Hb.block<3, 3>(0, 0) += 2.0 / NN * (combo1 - RiTukhat * Pi) * RiTukhat - 2.0 / NN / NN * viRiTuk[i] * viRiTuk[i].transpose() -
                                0.5 * skew(jjt.block<3, 1>(0, 0));
        Hb.block<3, 3>(0, 3) += HRt;
        Hb.block<3, 3>(3, 0) += HRt.transpose();
        Hb.block<3, 3>(3, 3) += 2.0 / NN * (ni - ni * ni / NN) * ukukT;

        Hess.block<6, 6>(6 * i, 6 * i) += coe * Hb;
      }

      for (std::size_t begin = 0; begin + 1 < groups.size(); ++begin) {
        const auto &first = groups[begin];
        const auto i = first.frame;
        double ni = first.moment.N;
        for (std::size_t next = begin + 1; next < groups.size(); ++next) {
          const auto &second = groups[next];
          const auto j = second.frame;
          double nj = second.moment.N;
          Mat_66<double> Hb = Auk[i].transpose() * umumT * Auk[j];
          Hb.block<3, 3>(0, 0) += -2.0 / NN / NN * viRiTuk[i] * viRiTuk[j].transpose();
          Hb.block<3, 3>(0, 3) += -2.0 * nj / NN / NN * viRiTukukT[i];
          Hb.block<3, 3>(3, 0) += -2.0 * ni / NN / NN * viRiTukukT[j].transpose();
          Hb.block<3, 3>(3, 3) += -2.0 * ni * nj / NN / NN * ukukT;

          Hess.block<6, 6>(6 * i, 6 * j) += coe * Hb;
        }
      }

      residual += coe * lmbd[kk];
    }

    for (int i = 1; i < win_size; i++)
      for (int j = 0; j < i; j++) Hess.block<6, 6>(6 * i, 6 * j) = Hess.block<6, 6>(6 * j, 6 * i).transpose();
  }

  double cached_residual(int head, int end) const {
    double residual = 0.;
    for (int i = head; i < end; ++i) residual += coeffs[i] * eig_values[i][0];
    return residual;
  }

  void evaluate_only_residual(const std::vector<BundleState> &xs, int head, int end, double &residual) {
    residual = 0;
    int kk = 0;  // The kk-th lambda value

    PointCluster pcr;

    for (int a = head; a < end; a++) {
      PointCluster sig = sig_vecs[a];
      for (const auto &group : plvec_voxels[a]) {
        pcr.transform(group.moment, xs[group.frame]);
        sig += pcr;
      }

      Eigen::Vector3d vBar = sig.v / sig.N;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sig.P / sig.N - vBar * vBar.transpose());
      Eigen::Vector3d lmbd = saes.eigenvalues();

      eig_values[a] = saes.eigenvalues();
      eig_vectors[a] = saes.eigenvectors();
      pcr_adds[a] = sig;

      residual += coeffs[a] * lmbd[kk];
    }
  }

  void clear() {
    sig_vecs.clear();
    plvec_voxels.clear();
    eig_values.clear();
    eig_vectors.clear();
    pcr_adds.clear();
    coeffs.clear();
  }

  ~VoxelLidarFactors() {}
};

class VoxelBundleOptimizer {
 public:
  int win_size, jac_leng, imu_leng;

  void hess_plus(Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT, Eigen::MatrixXd &hs, Eigen::VectorXd &js) {
    for (int i = 0; i < win_size; i++) {
      JacT.block<kPoseDimension, 1>(i * kInertialDimension, 0) += js.block<kPoseDimension, 1>(i * kPoseDimension, 0);
      for (int j = 0; j < win_size; j++)
        Hess.block<kPoseDimension, kPoseDimension>(i * kInertialDimension, j * kInertialDimension) +=
            hs.block<kPoseDimension, kPoseDimension>(i * kPoseDimension, j * kPoseDimension);
    }
  }

  double divide_thread(std::vector<BundleState> &x_stats, VoxelLidarFactors &voxhess, std::deque<BundlePreintegration *> &imus_factor,
                       Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT) {
    int thd_num = 5;
    double residual = 0;
    Hess.setZero();
    JacT.setZero();
    PLM(-1) hessians(thd_num);
    PLV(-1) jacobins(thd_num);
    std::vector<double> resis(thd_num, 0);

    for (int i = 0; i < thd_num; i++) {
      hessians[i].resize(jac_leng, jac_leng);
      jacobins[i].resize(jac_leng);
    }

    int tthd_num = thd_num;
    int g_size = voxhess.plvec_voxels.size();
    if (g_size < tthd_num) tthd_num = 1;
    double part = 1.0 * g_size / tthd_num;

    std::vector<std::thread *> mthreads(tthd_num);
    for (int i = 1; i < tthd_num; i++)
      mthreads[i] = new std::thread(&VoxelLidarFactors::acc_evaluate2, &voxhess, std::cref(x_stats), part * i, part * (i + 1), std::ref(hessians[i]),
                                    std::ref(jacobins[i]), std::ref(resis[i]));

    Eigen::MatrixXd jtj(2 * kInertialDimension, 2 * kInertialDimension);
    Eigen::VectorXd gg(2 * kInertialDimension);

    for (int i = 0; i < win_size - 1; i++) {
      jtj.setZero();
      gg.setZero();
      residual += imus_factor[i]->give_evaluate(x_stats[i], x_stats[i + 1], jtj, gg, true);
      Hess.block<kInertialDimension * 2, kInertialDimension * 2>(i * kInertialDimension, i * kInertialDimension) += jtj;
      JacT.block<kInertialDimension * 2, 1>(i * kInertialDimension, 0) += gg;
    }

    Hess *= kImuFactorWeight;
    JacT *= kImuFactorWeight;
    residual *= (kImuFactorWeight * 0.5);

    for (int i = 0; i < tthd_num; i++) {
      if (i != 0)
        mthreads[i]->join();
      else
        voxhess.acc_evaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);
      hess_plus(Hess, JacT, hessians[i], jacobins[i]);
      residual += resis[i];
      delete mthreads[i];
    }

    return residual;
  }

  double only_residual(std::vector<BundleState> &x_stats, VoxelLidarFactors &voxhess, std::deque<BundlePreintegration *> &imus_factor,
                       bool update_voxels = true) {
    double residual1 = 0, residual2 = 0;
    Eigen::MatrixXd jtj(2 * kInertialDimension, 2 * kInertialDimension);
    Eigen::VectorXd gg(2 * kInertialDimension);

    int thd_num = 5;
    std::vector<double> residuals(thd_num, 0);
    int g_size = voxhess.plvec_voxels.size();
    if (g_size < thd_num) {
      thd_num = 1;
    }
    std::vector<std::thread *> mthreads(thd_num, nullptr);
    double part = 1.0 * g_size / thd_num;
    if (update_voxels) {
      for (int i = 1; i < thd_num; i++)
        mthreads[i] = new std::thread(&VoxelLidarFactors::evaluate_only_residual, &voxhess, std::cref(x_stats), part * i, part * (i + 1),
                                      std::ref(residuals[i]));
    }

    for (int i = 0; i < win_size - 1; i++) residual1 += imus_factor[i]->give_evaluate(x_stats[i], x_stats[i + 1], jtj, gg, false);
    residual1 *= (kImuFactorWeight * 0.5);

    for (int i = 0; i < thd_num; i++) {
      if (!update_voxels) {
        residuals[i] = voxhess.cached_residual(part * i, part * (i + 1));
      } else if (i != 0) {
        mthreads[i]->join();
        delete mthreads[i];
      } else
        voxhess.evaluate_only_residual(x_stats, part * i, part * (i + 1), residuals[i]);
      residual2 += residuals[i];
    }

    return (residual1 + residual2);
  }

  double damping_iter(std::vector<BundleState> &x_stats, VoxelLidarFactors &voxhess, std::deque<BundlePreintegration *> &imus_factor,
                      double accepted_cost) {
    win_size = voxhess.win_size;
    jac_leng = win_size * 6;
    imu_leng = win_size * kInertialDimension;
    double u = 0.01, v = 2;
    Eigen::MatrixXd D(imu_leng, imu_leng), Hess(imu_leng, imu_leng);
    Eigen::VectorXd JacT(imu_leng), dxi(imu_leng);

    D.setIdentity();
    double residual1, residual2, q;
    bool is_calc_hess = true;
    bool residual_at_accepted_state = true;
    std::vector<BundleState> x_stats_temp = x_stats;

    for (int i = 0; i < 3; i++) {
      if (is_calc_hess) {
        residual1 = divide_thread(x_stats, voxhess, imus_factor, Hess, JacT);
      }

      Hess.topRows(kInertialDimension).setZero();
      Hess.leftCols(kInertialDimension).setZero();
      Hess.block<kInertialDimension, kInertialDimension>(0, 0).setIdentity();
      JacT.head(kInertialDimension).setZero();

      D.diagonal() = Hess.diagonal();
      dxi = (Hess + u * D).ldlt().solve(-JacT);

      for (int j = 0; j < win_size; j++) {
        x_stats_temp[j].R = x_stats[j].R * bundleRotationExp(dxi.block<3, 1>(kInertialDimension * j, 0));
        x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(kInertialDimension * j + 3, 0);
        x_stats_temp[j].v = x_stats[j].v + dxi.block<3, 1>(kInertialDimension * j + 6, 0);
        x_stats_temp[j].bg = x_stats[j].bg + dxi.block<3, 1>(kInertialDimension * j + 9, 0);
        x_stats_temp[j].ba = x_stats[j].ba + dxi.block<3, 1>(kInertialDimension * j + 12, 0);
      }

      for (int j = 0; j < win_size - 1; j++) imus_factor[j]->update_state(dxi.block<kInertialDimension, 1>(kInertialDimension * j, 0));

      double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);

      residual2 = only_residual(x_stats_temp, voxhess, imus_factor);

      q = (residual1 - residual2);

      residual_at_accepted_state = q > 0;
      if (residual_at_accepted_state) {
        accepted_cost = residual2;
        x_stats = x_stats_temp;
        double one_three = 1.0 / 3;

        q = q / q1;
        v = 2;
        q = 1 - pow(2 * q - 1, 3);
        u *= (q < one_three ? one_three : q);
        is_calc_hess = true;
      } else {
        u = u * v;
        v = 2 * v;
        is_calc_hess = false;

        for (int j = 0; j < win_size - 1; j++) {
          imus_factor[j]->dbg = imus_factor[j]->dbg_buf;
          imus_factor[j]->dba = imus_factor[j]->dba_buf;
        }
      }

      if (fabs((residual1 - residual2) / residual1) < 1e-6) break;
    }
    // Rebuild the cached voxel eigensystems only if the last trial was rejected.
    return residual_at_accepted_state ? accepted_cost : only_residual(x_stats, voxhess, imus_factor);
  }
};

#undef SKEW_SYM_MATRX
#undef PLV
#undef PLM

bool integrate(const InertialFrame &a, const InertialFrame &b, BundlePreintegration &pre) {
  const auto &data = b.imu;
  const double start = a.frame.stamp;
  const double stop = b.frame.stamp;
  if (data.size() < 2 || data.front().stamp > start || data.back().stamp < stop) return false;
  for (std::size_t i = 1; i < data.size(); ++i) {
    const auto &x = data[i - 1];
    const auto &y = data[i];
    const auto from = std::max(start, x.stamp);
    const auto to = std::min(stop, y.stamp);
    if (to <= from) continue;
    const auto span = y.stamp - x.stamp;
    if (span > .08) return false;
    const double interp = ((from - x.stamp) + (to - x.stamp)) / (2. * span);
    Eigen::Vector3d gyro = (1. - interp) * x.gyro + interp * y.gyro - a.nav.gyro_bias;
    Eigen::Vector3d acc = ((1. - interp) * x.accel + interp * y.accel) * a.nav.scale - a.nav.acc_bias;
    pre.add_imu(gyro, acc, to - from);
  }
  return std::abs(pre.dtime - (stop - start)) < 1.e-6;
}

struct Cell {
  std::vector<FrameMoments> live;
  PointCluster fixed;
};

void addLidarFactors(const std::vector<InertialFrame> &frames, const std::vector<MappingFrame> &fixed, const std::vector<BundleState> &states,
                     VoxelLidarFactors &factor) {
  std::unordered_map<VoxelKey, Cell, VoxelKeyHash> cells;
  struct CachedCell {
    VoxelKey key{};
    Cell *cell = nullptr;
  };
  // References to map entries stay valid across rehashes.
  std::array<CachedCell, 256> recent{};
  auto within = [](const auto &p) { return p.squaredNorm() < 625.f; };
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const auto &f = frames[i].frame;
    for (const auto &p : *f.cloud) {
      if (!within(p.getVector3fMap())) continue;
      const Eigen::Vector3d pt = p.getVector3fMap().cast<double>();
      const auto cell = key((f.odom * pt).cast<float>());
      auto &cached = recent[VoxelKeyHash{}(cell) % recent.size()];
      if (!cached.cell || !(cached.key == cell)) cached = {cell, &cells[cell]};
      auto &groups = cached.cell->live;
      if (groups.empty() || groups.back().frame != i) groups.push_back({i, {}});
      groups.back().moment.push(pt);
    }
  }
  for (const auto &f : fixed) {
    if ((f.odom.translation() - frames.front().frame.odom.translation()).norm() > 22.) continue;
    for (const auto &p : *f.cloud) {
      if (!within(p.getVector3fMap())) continue;
      const Eigen::Vector3d world = f.odom * p.getVector3fMap().cast<double>();
      const auto it = cells.find(key(world.cast<float>()));
      if (it != cells.end()) it->second.fixed.push(world);
    }
  }
  for (auto &[k, cell] : cells) {
    if (cell.live.size() < 2 && cell.fixed.N < 6) continue;
    PointCluster total = cell.fixed;
    for (const auto &group : cell.live) {
      PointCluster transformed;
      transformed.transform(group.moment, states[group.frame]);
      total += transformed;
    }
    if (total.N < 10) continue;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(total.cov());
    if (solver.info() != Eigen::Success) continue;
    Eigen::Vector3d values = solver.eigenvalues();
    if (values[1] < .012 || values[0] > .12 * values[1] || values[0] > .007) continue;
    Eigen::Matrix3d axes = solver.eigenvectors();
    factor.push_voxel(std::move(cell.live), cell.fixed, 1., values, axes, total);
  }
}
}  // namespace

Mat_66<double> relativeOdometryCovariance(const std::vector<Isometry3d> &poses, std::size_t first, std::size_t end) {
  Mat_66<double> covariance = Mat_66<double>::Zero();
  Mat_66<double> per_frame = Mat_66<double>::Zero();
  per_frame.diagonal() << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
  for (std::size_t i = first + 1; i < end; ++i) {
    const Isometry3d delta = poses[i].inverse() * poses[i - 1];
    Mat_66<double> adjoint = Mat_66<double>::Zero();
    adjoint.topLeftCorner<3, 3>() = delta.linear();
    adjoint.bottomRightCorner<3, 3>() = delta.linear();
    adjoint.bottomLeftCorner<3, 3>() = skew(delta.translation()) * delta.linear();
    covariance = (adjoint * covariance * adjoint.transpose() + per_frame).eval();
  }
  return covariance;
}

BaStats adjustInertial(const std::vector<InertialFrame> &frames, const std::vector<MappingFrame> &fixed, std::vector<Isometry3d> &poses,
                       std::vector<InertialEstimate> &nav, BoundarySummary *boundary) {
  BaStats result;
  if (frames.size() < 3) return result;

  std::vector<BundleState> states;
  std::deque<BundlePreintegration> storage;
  std::deque<BundlePreintegration *> imu;
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const auto &f = frames[i];
    BundleState x;
    x.t = f.frame.stamp;
    x.R = poses[i].linear();
    x.p = poses[i].translation();
    x.v = f.nav.velocity;
    x.bg = f.nav.gyro_bias;
    x.ba = f.nav.acc_bias;
    x.g = frames.front().nav.gravity;
    states.push_back(x);
    if (i == 0) continue;
    const auto &prev = frames[i - 1];
    auto &pre = storage.emplace_back(prev.nav.gyro_bias, prev.nav.acc_bias);
    if (!integrate(prev, f, pre)) return result;
    // Bias updates change the residual, while the integrated covariance stays fixed.
    pre.information = pre.cov.inverse();
    imu.push_back(&pre);
  }
  VoxelLidarFactors factors(static_cast<int>(states.size()));
  addLidarFactors(frames, fixed, states, factors);
  result.planes = factors.plvec_voxels.size();
  if (result.planes < 16) return result;

  VoxelBundleOptimizer opt;
  opt.win_size = static_cast<int>(states.size());
  // The factor builder already evaluated these moments at the initial poses.
  result.cost_first = opt.only_residual(states, factors, imu, false);
  result.cost_last = opt.damping_iter(states, factors, imu, result.cost_first);
  if (!std::isfinite(result.cost_last) || result.cost_last >= result.cost_first) return result;

  auto corrected = poses;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    corrected[i].linear() = states[i].R;
    corrected[i].translation() = states[i].p;
    const auto delta = poses[i].inverse() * corrected[i];
    if (!corrected[i].matrix().allFinite() || delta.translation().norm() > .5 || Eigen::AngleAxisd(delta.linear()).angle() > .15) return result;
  }
  if (boundary) {
    // Use the final undamped model. The first 15 coordinates are fixed by
    // local BA; the last pose separates the active area from this submap.
    Eigen::MatrixXd joint(states.size() * kInertialDimension, states.size() * kInertialDimension);
    Eigen::VectorXd gradient(joint.rows());
    opt.divide_thread(states, factors, imu, joint, gradient);
    const Eigen::Index dim = joint.rows() - kInertialDimension;
    const Eigen::MatrixXd h = joint.bottomRightCorner(dim, dim);
    if (h.allFinite() && (h.diagonal().array() > 0.).all()) {
      const Eigen::VectorXd scale = h.diagonal().cwiseSqrt().cwiseInverse();
      const Eigen::MatrixXd normal = scale.asDiagonal() * (0.5 * (h + h.transpose())) * scale.asDiagonal();
      const Eigen::LDLT<Eigen::MatrixXd> elimination(normal);
      if (elimination.info() == Eigen::Success && (elimination.vectorD().array() > 1e-9).all()) {
        const Eigen::Index offset = dim - kInertialDimension;
        Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(dim, 6);
        rhs.middleRows(offset, 6) = scale.segment(offset, 6).asDiagonal();
        const Eigen::MatrixXd columns = scale.asDiagonal() * elimination.solve(rhs);
        const Mat_66<double> endpoint = columns.middleRows(offset, 6);
        const Eigen::LLT<Mat_66<double>> end_factor((0.5 * (endpoint + endpoint.transpose())).eval());
        if (end_factor.info() == Eigen::Success && columns.allFinite()) {
          Mat_66<double> tangent = Mat_66<double>::Identity();
          tangent.bottomRightCorner<3, 3>() = states.back().R.transpose();
          boundary->covariance = tangent * endpoint * tangent.transpose();
          boundary->position_sigma = std::sqrt(endpoint.bottomRightCorner<3, 3>().diagonal().maxCoeff());
          boundary->rotation_sigma = std::sqrt(endpoint.topLeftCorner<3, 3>().diagonal().maxCoeff());
          // Use the uncertainty of the same adjacent constraint exported
          // to PGO. Convert its right translation tangent into BA's world
          // position coordinates before testing the 3-sigma ellipsoid.
          Mat_66<double> motion_covariance = relativeOdometryCovariance(poses, 0, poses.size());
          motion_covariance = tangent.transpose() * motion_covariance * tangent;
          const Eigen::LLT<Mat_66<double>> motion_noise(motion_covariance);
          if (motion_noise.info() != Eigen::Success) return result;
          const Mat_66<double> correction = 3. * motion_noise.matrixL().toDenseMatrix();
          Mat_61<double> restore_endpoint;
          const Eigen::AngleAxisd turn(states.back().R.transpose() * poses.back().linear());
          restore_endpoint << turn.angle() * turn.axis(), poses.back().translation() - states.back().p;
          const auto original_end = states.back();
          std::vector<Mat_66<double>> motions(states.size(), Mat_66<double>::Zero());
          for (Eigen::Index i = 0; i < offset; i += kInertialDimension) {
            const std::size_t frame = static_cast<std::size_t>(i / kInertialDimension + 1);
            const Eigen::Matrix<double, kInertialDimension, 6> gain =
                end_factor.solve(columns.middleRows(i, kInertialDimension).transpose()).transpose();
            motions[frame] = gain.topRows<6>() * correction;
            const Eigen::Matrix<double, kInertialDimension, 1> delta = gain * restore_endpoint;
            corrected[frame].linear() = states[frame].R * bundleRotationExp(delta.head<3>());
            corrected[frame].translation() = states[frame].p + delta.segment<3>(3);
            states[frame].v += delta.segment<3>(6);
            states[frame].bg += delta.segment<3>(9);
            states[frame].ba += delta.segment<3>(12);
            if (frame < imu.size()) imu[frame]->update_state(delta);
          }
          // Point-to-plane change from conditional interior deformation.
          // Evaluate the whole support from the voxel's sufficient moments.
          std::vector<double> cost(states.size(), 0.);
          std::vector<std::size_t> points(states.size(), 0);
          for (std::size_t cell = 0; cell < factors.plvec_voxels.size(); ++cell) {
            const auto normal = factors.eig_vectors[cell].col(0);
            const auto &cluster = factors.plvec_voxels[cell];
            for (const auto &group : cluster) {
              const auto i = group.frame;
              if (i == 0 || i + 1 == states.size()) continue;
              const auto &xyz = group.moment;
              const Eigen::Vector3d direction = states[i].R.transpose() * normal;
              points[i] += xyz.N;
              for (Eigen::Index axis = 0; axis < 6; ++axis) {
                const Eigen::Vector3d gradient = direction.cross(motions[i].col(axis).head<3>());
                const double translation = normal.dot(motions[i].col(axis).tail<3>());
                cost[i] += gradient.dot(xyz.P * gradient) + 2. * translation * gradient.dot(xyz.v) + xyz.N * translation * translation;
              }
            }
          }
          for (std::size_t i = 1; i + 1 < states.size(); ++i) {
            if (points[i]) boundary->interior_shift_m = std::max(boundary->interior_shift_m, std::sqrt(std::max(0., cost[i]) / (6 * points[i])));
          }
          corrected.back() = poses.back();
          states.back() = original_end;
          states.back().v = nav.back().velocity;
          states.back().bg = nav.back().gyro_bias;
          states.back().ba = nav.back().acc_bias;
          boundary->valid = true;
        }
      }
    }
  }
  if (boundary) {
    if (!boundary->valid) return result;
    for (std::size_t i = 0; i < states.size(); ++i) {
      const auto delta = poses[i].inverse() * corrected[i];
      if (!corrected[i].matrix().allFinite() || delta.translation().norm() > .5 || Eigen::AngleAxisd(delta.linear()).angle() > .15) {
        boundary->valid = false;
        return result;
      }
      states[i].R = corrected[i].linear();
      states[i].p = corrected[i].translation();
    }
    result.cost_last = opt.only_residual(states, factors, imu);
    if (!std::isfinite(result.cost_last) || result.cost_last >= result.cost_first) {
      boundary->valid = false;
      return result;
    }
  }
  poses.swap(corrected);
  for (std::size_t i = 0; i < poses.size(); ++i) {
    nav[i].velocity = states[i].v;
    nav[i].gyro_bias = states[i].bg;
    nav[i].acc_bias = states[i].ba;
  }
  result.updates = 1;
  return result;
}

}  // namespace sapphire
