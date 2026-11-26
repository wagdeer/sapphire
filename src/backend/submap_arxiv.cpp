#include <sapphire/backend/submap_arxiv.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unsupported/Eigen/FFT>
#include <utility>

namespace sapphire {
namespace {
constexpr int width = 128;
constexpr float spacing = .75f;
constexpr int channels = 4;
constexpr int area = width * width;
static_assert(channels * area == std::numeric_limits<std::uint16_t>::max() + 1);
constexpr std::size_t shortlist_size = 15;
using Complex = std::complex<float>;
using Spectrum = std::vector<Complex>;
using Descriptor = Eigen::Matrix<float, 64, 1>;

struct Surface {
  Eigen::Vector3f offset;
  Eigen::Vector3f normal;
};

struct RasterFrame {
  Eigen::Vector3f center;
  std::vector<Surface> surfaces;
};

struct Hint {
  std::size_t target;
  Eigen::Vector2f shift, correction;
  float score;
};

struct Entry {
  Isometry3d track;
  Descriptor key;
  std::size_t bins;
  bool departed = false;
};

class Frequencies {
 public:
  void transform(Spectrum& grid, bool inverse) {
    for (int x = 0; x < width; ++x) {
      for (int y = 0; y < width; ++y) from_[y] = grid[x * width + y];
      apply(inverse);
      for (int y = 0; y < width; ++y) grid[x * width + y] = line_[y];
    }
    for (int y = 0; y < width; ++y) {
      for (int x = 0; x < width; ++x) from_[x] = grid[x * width + y];
      apply(inverse);
      for (int x = 0; x < width; ++x) grid[x * width + y] = line_[x];
    }
  }

 private:
  void apply(bool inverse) {
    if (inverse)
      fft_.inv(line_, from_);
    else
      fft_.fwd(line_, from_);
  }

  Eigen::FFT<float> fft_;
  std::vector<Complex> line_ = std::vector<Complex>(width);
  std::vector<Complex> from_ = std::vector<Complex>(width);
};

using Raster = std::array<Spectrum, channels>;

int wrap(int cell) { return (cell + width + width / 2) % width - width / 2; }

}  // namespace

class SubmapArxiv::Impl {
 public:
  explicit Impl(const std::filesystem::path& directory)
      : blocks_(directory / "blocks"), places_(directory / "places") {
    std::filesystem::create_directories(directory);
    if (!std::filesystem::create_directory(blocks_)) throw std::runtime_error("map directory already contains blocks");
    if (!std::filesystem::create_directory(places_)) throw std::runtime_error("submap place index already exists");
  }

  void write(std::size_t id, const GaussianVoxelMap& map) {
    if (map.size() == 0) return;
    const auto path = blockPath(id);
    auto partial = path;
    partial += ".partial";
    std::ofstream output(partial, std::ios::binary);
    map.write(output);
    output.close();
    if (!output) throw std::runtime_error("cannot finish Gaussian block");
    std::filesystem::rename(partial, path);
  }

  std::unique_ptr<GaussianVoxelMap> read(std::size_t id, std::size_t* bytes) const {
    const auto path = blockPath(id);
    std::ifstream stream(path, std::ios::binary);
    auto map = GaussianVoxelMap::read(stream);
    if (bytes) *bytes = std::filesystem::file_size(path);
    return map;
  }

  std::vector<PlaceProposal> indexAndQuery(const GaussianVoxelMap& map, const Isometry3d& track) {
    const auto id = entries_.size();
    const Eigen::Vector3f position = track.translation().cast<float>();
    if (!entries_.empty()) travel_ += (track.translation() - entries_.back().track.translation()).norm();
    auto [key, bins] = deposit(map, track);
    entries_.push_back({track, key, bins.size()});
    save(id, bins);

    std::vector<std::pair<float, std::size_t>> pool;
    const double radius = std::min(90., 20. + .014 * travel_);
    for (std::size_t old = 0; old + 4 < id; ++old) {
      auto& target = entries_[old];
      const double distance = (track.translation() - target.track.translation()).norm();
      if (distance > 25.) target.departed = true;
      if (target.departed && distance < radius && target.bins && !bins.empty()) pool.emplace_back(key.dot(target.key), old);
    }
    if (pool.size() > shortlist_size) {
      std::partial_sort(pool.begin(), pool.begin() + shortlist_size, pool.end(), std::greater<>());
      pool.resize(shortlist_size);
    }

    std::vector<Hint> hints;
    std::vector<PlaceProposal> out;
    if (!pool.empty()) {
      const auto source = spectrum(bins);
      for (const auto& [quality, old] : pool) {
        const auto& target = hot(old);
        auto peak = correlate(source, target, entries_[old].bins, bins.size());
        if (peak.score < .10f || peak.ambiguity < 1.18f) continue;
        const Eigen::Vector2f correction = entries_[old].track.translation().head<2>().cast<float>() + peak.shift - position.head<2>();
        // Seek agreement with independently sealed query groups observed
        // earlier. Frames contributing to adjacent coarse rasters may overlap.
        bool supported = false;
        for (const auto& [past, previous] : recent_hints_) {
          if (past + 1 >= id) continue;
          for (const auto& h : previous) {
            if ((correction - h.correction).squaredNorm() < 4.f &&
                (entries_[old].track.translation() - entries_[h.target].track.translation()).squaredNorm() < 196.) {
              supported = true;
              break;
            }
          }
          if (supported) break;
        }
        hints.push_back({old, peak.shift, correction, peak.score});
        if (!supported) continue;
        Isometry3d relative = entries_[old].track.inverse() * track;
        Eigen::Vector3d world_shift = track.translation() - entries_[old].track.translation();
        world_shift.head<2>() = peak.shift.cast<double>();
        relative.translation() = entries_[old].track.linear().transpose() * world_shift;
        out.push_back({old, relative, peak.score});
      }
    }
    recent_hints_.emplace_back(id, std::move(hints));
    if (recent_hints_.size() > 5) recent_hints_.pop_front();
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.score > b.score; });
    return out;
  }

 private:
  struct Peak {
    Eigen::Vector2f shift;
    float score, ambiguity;
  };

  std::pair<Descriptor, std::vector<std::uint16_t>> deposit(const GaussianVoxelMap& map, const Isometry3d& track) {
    const Eigen::Matrix3f rotation = track.linear().cast<float>();
    RasterFrame frame{track.translation().cast<float>(), {}};
    frame.surfaces.reserve(map.size());
    for (std::size_t i = 0; i < map.size(); ++i) {
      const auto plane = map.voxel(i).plane;
      const Eigen::Vector3f normal = rotation * plane.normal;
      if (std::abs(normal.z()) < .65f) frame.surfaces.push_back({rotation * plane.center, normal});
    }
    recent_.push_back(std::move(frame));
    if (recent_.size() > 4) recent_.pop_front();
    auto center = recent_.back().center;
    std::array<std::uint8_t, channels * area> grid{};
    std::vector<std::uint16_t> bins;
    Descriptor key = Descriptor::Zero();
    for (const auto& previous : recent_) {
      const Eigen::Vector3f delta = previous.center - center;
      if (delta.squaredNorm() > 225.f) continue;
      for (const auto& point : previous.surfaces) {
        const Eigen::Vector3f offset = point.offset + delta;
        const int x = static_cast<int>(std::round(offset.x() / spacing));
        const int y = static_cast<int>(std::round(offset.y() / spacing));
        if (std::abs(x) >= width / 2 - 3 || std::abs(y) >= width / 2 - 3) continue;
        constexpr float pi = 3.14159265358979323846f;
        const int angle = (static_cast<int>(std::round(std::atan2(point.normal.y(), point.normal.x()) * channels / pi)) + channels) % channels;
        const int bin = angle * area + (x + width / 2) * width + y + width / 2;
        if (grid[bin]) continue;
        grid[bin] = 1;
        bins.push_back(static_cast<std::uint16_t>(bin));
        const auto ring = std::min(15, static_cast<int>(std::hypot(float(x), float(y)) * spacing / 3.));
        ++key(ring * channels + angle);
      }
    }
    if (bins.size() < 45) return {Descriptor::Zero().eval(), std::vector<std::uint16_t>{}};
    key = key.cwiseSqrt();
    key.normalize();
    return {key, std::move(bins)};
  }

  Raster spectrum(const std::vector<std::uint16_t>& bins) {
    Raster out;
    for (auto& field : out) field.resize(area);
    for (const auto bin : bins) out[bin / area][bin % area] = 1.f;
    for (auto& field : out) fft_.transform(field, false);
    return out;
  }

  Peak correlate(const Raster& a, const Raster& b, std::size_t nb, std::size_t na) {
    Spectrum mixed(area);
    for (int k = 0; k < channels; ++k)
      for (int i = 0; i < area; ++i) mixed[i] += b[k][i] * std::conj(a[k][i]);
    fft_.transform(mixed, true);
    int best = 0;
    for (int i = 1; i < area; ++i)
      if (mixed[i].real() > mixed[best].real()) best = i;
    const int x = best / width, y = best % width;
    const float peak = mixed[best].real();
    float second = 0.f;
    for (int i = 0; i < area; ++i) {
      const int dx = wrap(i / width - x), dy = wrap(i % width - y);
      if (dx * dx + dy * dy > 16) second = std::max(second, mixed[i].real());
    }
    return {Eigen::Vector2f(wrap(x) * spacing, wrap(y) * spacing), peak / std::sqrt(float(na) * float(nb)), peak / std::max(1.f, second)};
  }

  std::filesystem::path blockPath(std::size_t id) const { return blocks_ / (std::to_string(id) + ".gauss"); }
  std::filesystem::path path(std::size_t id) const { return places_ / (std::to_string(id) + ".bins"); }

  void save(std::size_t id, const std::vector<std::uint16_t>& bins) {
    if (bins.empty()) return;
    std::ofstream file(path(id), std::ios::binary);
    file.write(reinterpret_cast<const char*>(bins.data()), bins.size() * sizeof(bins.front()));
    file.close();
    if (!file) throw std::runtime_error("cannot save submap correlation raster");
  }

  const Raster& hot(std::size_t id) {
    const auto it = std::find_if(spectra_.begin(), spectra_.end(), [id](const auto& e) { return e.first == id; });
    if (it != spectra_.end()) {
      std::rotate(spectra_.begin(), it, std::next(it));
      return spectra_.front().second;
    }
    std::vector<std::uint16_t> bins(entries_[id].bins);
    std::ifstream file(path(id), std::ios::binary);
    if (!file.read(reinterpret_cast<char*>(bins.data()), bins.size() * sizeof(bins.front())))
      throw std::runtime_error("cannot read submap correlation raster");
    if (spectra_.size() == 8) spectra_.pop_back();
    spectra_.emplace_front(id, spectrum(bins));
    return spectra_.front().second;
  }

  std::filesystem::path blocks_, places_;
  std::vector<Entry> entries_;
  std::deque<RasterFrame> recent_;
  std::deque<std::pair<std::size_t, std::vector<Hint>>> recent_hints_;
  std::deque<std::pair<std::size_t, Raster>> spectra_;
  Frequencies fft_;
  double travel_ = 0.;
};

SubmapArxiv::SubmapArxiv(std::filesystem::path directory) : impl_(std::make_unique<Impl>(std::move(directory))) {}
SubmapArxiv::~SubmapArxiv() = default;
void SubmapArxiv::write(std::size_t id, const GaussianVoxelMap& map) { impl_->write(id, map); }
std::unique_ptr<GaussianVoxelMap> SubmapArxiv::read(std::size_t id, std::size_t* bytes) const { return impl_->read(id, bytes); }

std::vector<PlaceProposal> SubmapArxiv::indexAndQuery(const GaussianVoxelMap& map, const Isometry3d& track_anchor) {
  return impl_->indexAndQuery(map, track_anchor);
}

void SubmapArxiv::publishRevisit(std::size_t source, double stamp, const Isometry3d& source_pose, const std::vector<RevisitBlock>& blocks) {
  if (blocks.empty()) return;
  const auto& target = blocks.front();
  const Isometry3d target_from_global = target.global_from_block.inverse();
  std::unordered_map<VoxelKey, Moments, VoxelKeyHash> voxels;
  std::size_t bytes = 0;
  const Eigen::Vector3d center_in_target = target_from_global * source_pose.translation();
  for (const auto& block : blocks) {
    const Isometry3d pose = target_from_global * block.global_from_block;
    std::size_t block_bytes = 0;
    const auto geometry = read(block.id, &block_bytes);
    bytes += block_bytes;
    for (std::size_t i = 0; i < geometry->size(); ++i) {
      const auto voxel = geometry->voxel(i);
      const Eigen::Vector3d center = pose * voxel.plane.center.cast<double>();
      if ((center - center_in_target).squaredNorm() > 35. * 35.) continue;
      const auto cell = key(center.cast<float>());
      if (voxels.size() >= 24000 && voxels.find(cell) == voxels.end()) continue;
      const auto& c = voxel.gaussian.covariance;
      Eigen::Matrix3d covariance;
      covariance << c[0], c[1], c[2], c[1], c[3], c[4], c[2], c[4], c[5];
      const auto points = voxel.gaussian.points;
      auto& moments = voxels[cell];
      moments.sum += points * center;
      moments.second += points * (pose.linear() * covariance * pose.linear().transpose() + center * center.transpose());
      moments.count += points;
      moments.frames += voxel.gaussian.frames;
    }
  }
  auto map = std::make_unique<GaussianVoxelMap>();
  map->center = center_in_target.cast<float>();
  map->reserve(voxels.size());
  for (const auto& [cell, moments] : voxels) {
    if (const auto fitted = moments.fit()) map->insert(cell, *fitted);
  }
  if (map->size() < 72) return;
  if (spdlog::should_log(spdlog::level::debug)) {
    std::string ids;
    for (const auto& block : blocks) ids += " " + std::to_string(block.id);
    spdlog::debug("[methodB] map {} {} {:.9f}{}", source, target.id, stamp, ids);
  }
  revisit_.publish(std::make_unique<RevisitTarget>(
      RevisitTarget{std::move(map), stamp, source_pose, target.global_from_block, source, target.id, bytes, blocks.size()}));
}
}  // namespace sapphire
