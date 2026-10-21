#include <backend/io/trajectory.hpp>

#include <Eigen/SparseCholesky>
#include <unsupported/Eigen/Splines>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <limits>
#include <locale>
#include <ostream>

namespace sapphire::backend::io {
namespace {
constexpr size_t maxControls = 64, maxOutput = 128;
constexpr double minimumDistance = .02, outputSpacing = .02, smoothingLambda = .1;
using Spline = Eigen::Spline<double, 3, Eigen::Dynamic>;
using Pose = sapphire::backend::StateGroupf;

Eigen::Matrix<double, 3, Eigen::Dynamic> smoothPositions(const Eigen::Matrix<double, 3, Eigen::Dynamic>& input) {
  const int n = input.cols();
  if (n < 3) return input;
  Eigen::MatrixXd system = Eigen::MatrixXd::Identity(n, n);
  const Eigen::Vector3d stencil(1, -2, 1);
  for (int i = 0; i + 2 < n; ++i) system.block<3, 3>(i, i) += smoothingLambda * stencil * stencil.transpose();
  Eigen::SparseMatrix<double> interior = system.block(1, 1, n - 2, n - 2).sparseView();
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(interior);
  if (solver.info() != Eigen::Success) return input;
  Eigen::MatrixXd rhs = input.middleCols(1, n - 2).transpose();
  rhs -= system.block(1, 0, n - 2, 1) * input.col(0).transpose();
  rhs -= system.block(1, n - 1, n - 2, 1) * input.col(n - 1).transpose();
  const Eigen::MatrixXd fitted = solver.solve(rhs);
  if (solver.info() != Eigen::Success || !fitted.allFinite()) return input;
  auto output = input;
  output.middleCols(1, n - 2) = fitted.transpose();
  return output;
}
}  // namespace

OdomFileFormat odometry_format(std::string_view name) {
  if (name == "tum") return OdomFileFormat::Tum;
  if (name == "txt") return OdomFileFormat::Txt;
  throw std::invalid_argument("Unsupported pose format; choose txt or tum");
}

std::vector<std::string> trajectoryColumns(OdomFileFormat format) {
  switch (format) {
    case OdomFileFormat::Tum: return {"timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"};
    case OdomFileFormat::Txt: return {"timestamp", "r00", "r01", "r02", "tx", "r10", "r11", "r12", "ty", "r20", "r21", "r22", "tz"};
  }
  throw std::invalid_argument("Unsupported pose format; choose txt or tum");
}

void writeOdomFile(std::ostream& output, OdomFileFormat format,
                   const std::vector<OdometryRecord>& records, const std::string& metadata) {
  const auto columns = trajectoryColumns(format);
  if (records.empty()) throw std::invalid_argument("No exportable poses");
  if (metadata.find_first_of("\r\n") != std::string::npos) throw std::invalid_argument("Trajectory metadata must be a single line");
  for (const auto& record : records) {
    if (!std::isfinite(record.stamp)) throw std::invalid_argument("Non-finite timestamp");
    if (!std::all_of(record.matrix.begin(), record.matrix.end(), [](double v) { return std::isfinite(v); }))
      throw std::invalid_argument("Non-finite pose");
    const Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> matrix(record.matrix.data());
    const Eigen::Matrix3d rotation = matrix.leftCols<3>();
    if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).squaredNorm() > 1e-4 ||
        std::abs(rotation.determinant() - 1) > .01)
      throw std::invalid_argument("Invalid pose rotation (including null pose)");
  }
  // Restore caller stream formatting, including locales that use decimal commas.
  struct Restore {
    std::ostream& output;
    std::ios::fmtflags flags;
    std::streamsize precision;
    std::locale locale;
    ~Restore() { output.flags(flags); output.precision(precision); output.imbue(locale); }
  } restore{output, output.flags(), output.precision(), output.getloc()};
  output.imbue(std::locale::classic());
  output << std::defaultfloat << std::setprecision(std::numeric_limits<double>::max_digits10);
  output << '#';
  for (const auto& column : columns) output << ' ' << column;
  output << '\n';
  if (!metadata.empty()) output << "# " << metadata << '\n';
  for (const auto& record : records) {
    output << record.stamp;
    if (format == OdomFileFormat::Txt) {
      for (double value : record.matrix) output << ' ' << value;
    } else {
      const Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> matrix(record.matrix.data());
      const Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::Matrix3d(matrix.leftCols<3>())).normalized();
      output << ' ' << matrix(0, 3) << ' ' << matrix(1, 3) << ' ' << matrix(2, 3)
             << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w();
    }
    output << '\n';
  }
  if (!output) throw std::runtime_error("Cannot write trajectory");
}

void PathConstructor::clear() {
  samples_.clear(); session_ = 0;
}

void PathConstructor::setDuration(double seconds) {
  if (!std::isfinite(seconds) || seconds <= 0) throw std::invalid_argument("path_duration_sec must be finite and positive");
  duration_ = seconds;
  clear();
}

void PathConstructor::observe(uint64_t session, double keyframe_stamp, double stamp, const Pose& odom) {
  if (!session || !std::isfinite(stamp) || !std::isfinite(keyframe_stamp) || keyframe_stamp <= 0 ||
      stamp < keyframe_stamp || odom.isNull() || !odom.matrix().allFinite()) return;
  if (session != session_) {
    clear(); session_ = session;
  }
  if (!samples_.empty() && stamp <= samples_.back().stamp) return;
  // Keep both endpoints. Replace only a redundant moving endpoint.
  if (samples_.size() > 1 &&
      samples_.back().pose.translationDistance(samples_[samples_.size() - 2].pose) < minimumDistance &&
      odom.translationDistance(samples_[samples_.size() - 2].pose) < minimumDistance)
    samples_.back() = {stamp, odom};
  else samples_.push_back({stamp, odom});
  samples_.erase(samples_.begin(), std::lower_bound(samples_.begin(), samples_.end(), stamp - duration_,
      [](const Sample& sample, double cutoff) { return sample.stamp < cutoff; }));
  if (samples_.size() > maxControls) {
    std::vector<Sample> reduced{samples_.front()};
    for (size_t i = 2; i + 1 < samples_.size(); i += 2) reduced.push_back(samples_[i]);
    reduced.push_back(samples_.back()); samples_.swap(reduced);
  }
}

std::vector<PathConstructor::Sample> PathConstructor::smooth() const {
  if (samples_.size() < 2) return samples_;
  std::vector<Sample> samples{samples_.front()};
  for (size_t i = 1; i < samples_.size(); ++i) {
    if (samples_[i].pose.translationDistance(samples.back().pose) >= minimumDistance) samples.push_back(samples_[i]);
    else if (i + 1 == samples_.size()) {
      if (samples.size() > 1) samples.back() = samples_[i];
      else if (samples_[i].pose.translationDistance(samples.back().pose) > 1e-6) samples.push_back(samples_[i]);
      else samples.back() = samples_[i];
    }
  }
  if (samples.size() < 2) return {samples_.back()};
  // Translate near the origin before fitting, retaining raw odometry orientation.
  const Eigen::Vector3d origin = samples.front().pose.translationVector().cast<double>();
  const int n = samples.size();
  Eigen::Matrix<double, 3, Eigen::Dynamic> positions(3, n);
  for (int i = 0; i < n; ++i) {
    positions.col(i) = samples[i].pose.translationVector().cast<double>() - origin;
  }
  const auto fitted = smoothPositions(positions);
  // Approximate interior observations instead of forcing interpolation through
  // nearly coincident points. Clamped uniform knots keep the curve in the
  // controls' convex hull, avoiding ill-conditioned chord-length interpolation
  // that can turn millimetre odom jitter into metre-scale excursions.
  const int degree = std::min(3, n - 1);
  Eigen::RowVectorXd knots(n + degree + 1), parameters(n);
  for (int i = 0; i < knots.size(); ++i)
    knots[i] = i <= degree ? 0 : i >= n ? 1 : double(i - degree) / (n - degree);
  for (int i = 0; i < n; ++i) parameters[i] = knots.segment(i + 1, degree).mean();
  const Spline spline(knots, fitted);
  if (!spline.ctrls().allFinite()) return samples_;
  // Arc-length table: construct Eigen's spline once, rather than rebuilding it
  // on every evaluation as in the general-purpose reference tool.
  constexpr int divisions = 256;
  std::vector<double> lengths(divisions + 1, 0);
  Eigen::Vector3d previous = spline(0).matrix();
  for (int i = 1; i <= divisions; ++i) {
    const Eigen::Vector3d point = spline(double(i) / divisions).matrix();
    lengths[i] = lengths[i - 1] + (point - previous).norm(); previous = point;
  }
  if (!std::isfinite(lengths.back())) return samples_;
  const size_t count = std::max<size_t>(2, std::min<double>(maxOutput, std::ceil(lengths.back() / outputSpacing) + 1));
  std::vector<Sample> result; result.reserve(count);
  result.push_back(samples_.front());
  for (size_t i = 1; i + 1 < count; ++i) {
    const double length = lengths.back() * i / (count - 1);
    const auto upper = std::lower_bound(lengths.begin() + 1, lengths.end(), length);
    const size_t segment = upper - lengths.begin();
    const double span = lengths[segment] - lengths[segment - 1];
    const double u = (segment - 1 + (span > 1e-12 ? (length - lengths[segment - 1]) / span : 0)) / divisions;
    const int k = std::min<int>(n - 1, std::upper_bound(parameters.data(), parameters.data() + n, u) - parameters.data());
    const double fraction = (u - parameters[k - 1]) / (parameters[k] - parameters[k - 1]);
    const auto q = Eigen::Quaternionf(samples[k - 1].pose.rotationMatrix()).normalized().slerp(
        float(fraction), Eigen::Quaternionf(samples[k].pose.rotationMatrix()).normalized());
    result.push_back({samples[k - 1].stamp + fraction * (samples[k].stamp - samples[k - 1].stamp),
                      Pose::fromRotationTranslation(q.toRotationMatrix(), (origin + spline(u).matrix()).cast<float>())});
  }
  result.push_back(samples_.back());
  return result;
}
}  // namespace sapphire::backend::io
