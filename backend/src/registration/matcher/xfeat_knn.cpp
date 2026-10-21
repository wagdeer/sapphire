#include <backend/registration/matcher/descriptor_match.hpp>
#include <backend/registration/descriptor/landmark_descriptors.hpp>
#include "backend/registration/utils/npu_core.hpp"
#include "backend/registration/matcher/nearest.hpp"
#include "backend/registration/utils/xfeat_network.hpp"
#include <array>
#include <cmath>
#include <chrono>
#include <ctime>
#include <limits>
#include <backend/tools/logging.hpp>
#if EXTERIOR_WITH_RKNN && defined(__aarch64__)
#include <arm_neon.h>
#endif
namespace sapphire::backend::feature {
namespace {
using Clock = std::chrono::steady_clock;
double owner_cpu_ms() { timespec t{}; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t); return 1000.*t.tv_sec + 1e-6*t.tv_nsec; }
#if EXTERIOR_WITH_RKNN
constexpr int slots = 512, dimensions = 64;
void require(bool ok, const char* reason) { if (!ok) throw std::runtime_error(reason); }
#endif
}
struct XFeatKnn::Impl {
  uint64_t identity = 0;
  int core = 1;
#if EXTERIOR_WITH_RKNN
  detail::XFeatNetwork matcher;
  std::unique_ptr<cv::float16_t[]> similarities = std::make_unique<cv::float16_t[]>(slots*slots);
  std::array<float, slots> forward_max{}, reverse_max{};
  std::array<featmap::detail::DescriptorNearest<Float64_F32>, slots> forward{}, reverse{};
  void initialize(const std::string& matches, uint64_t descriptor_identity, int selected_core) {
    require(selected_core >= 0 && selected_core < 3, "Invalid XFeat NPU core");
    core = selected_core;
    identity = descriptor_identity;
    std::lock_guard<std::mutex> lock(npu_core_mutexes[core]);
    matcher.initialize(matches, core, 2, slots*dimensions, similarities.get(), slots*slots*sizeof(cv::float16_t), false);
  }
  featmap::MatchResult match(const featmap::LandmarkMap& map, const bow::FeatureBlock& query, const MatchOptions& options) {
    featmap::MatchResult result;
    if (options.min_matches == 0) throw std::invalid_argument("Zero submap match count");
    query.validate();
    if (map.descriptor_model_id() != identity || query.descriptor_model_id != identity || map.descriptor_dimensions() != dimensions || query.descriptors.cols != dimensions) {
      result.status = featmap::MatchStatus::vocabulary_mismatch; return result;
    }
    const size_t na = map.points().size(), nb = query.keypoints.size();
    if (na > slots || nb > slots) { result.status = featmap::MatchStatus::budget_exceeded; return result; }
    if (na < options.min_matches || nb < options.min_matches) return result;
    std::lock_guard<std::mutex> lock(npu_core_mutexes[core]);
    auto* a = matcher.data(0); auto* b = matcher.data(1);
    std::fill_n(a, slots*dimensions, cv::float16_t(0)); std::fill_n(b, slots*dimensions, cv::float16_t(0));
    const landmark_descriptors<Float64_F32> rows(map);
    for (size_t i = 0; i < na; ++i) {
      const float* src = rows.row(i).data();
      for (int c = 0; c < dimensions; ++c) { a[i*dimensions+c] = cv::float16_t(src[c]); }
    }
    for (size_t i = 0; i < nb; ++i) {
      const auto& p = query.keypoints[i].pt;
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
      const float* src = query.descriptors.ptr<float>(int(i));
      for (int c = 0; c < dimensions; ++c) b[i*dimensions+c] = cv::float16_t(src[c]);
    }
    matcher.run();
    forward_max.fill(-1); reverse_max.fill(-1); forward.fill({}); reverse.fill({});
    for (size_t i = 0; i < na; ++i) {
      float maximum = -1;
#if defined(__aarch64__)
      auto maxima = vdupq_n_f32(-1);
      for (size_t j = 0; j < nb; j += 4) {
        const auto v = vcvt_f32_f16(vreinterpret_f16_u8(vld1_u8(reinterpret_cast<const uint8_t*>(similarities.get()+i*slots+j))));
        maxima = vmaxq_f32(maxima,v);
        vst1q_f32(reverse_max.data()+j,vmaxq_f32(vld1q_f32(reverse_max.data()+j),v));
      }
      maximum = vmaxvq_f32(maxima);
#else
      for (size_t j = 0; j < nb; ++j) {
        const float v = float(similarities[i*slots+j]);
        maximum = std::max(maximum,v); reverse_max[j] = std::max(reverse_max[j],v);
      }
#endif
      forward_max[i] = maximum;
    }
    // The approximate matrix only selects candidates. Distances use the actual
    // stored FP32 values (not a second quantized copy); both directions are unique.
    for (size_t i = 0; i < na; ++i) {
      const descriptor_distance<Float64_F32> distance{rows.row(i)};
      for (size_t base = 0; base < nb; base += 4) {
        uint32_t active[4]{};
#if defined(__aarch64__)
        const auto score = vcvt_f32_f16(vreinterpret_f16_u8(vld1_u8(reinterpret_cast<const uint8_t*>(similarities.get()+i*slots+base))));
        const auto mask = vandq_u32(vcgeq_f32(score,vdupq_n_f32(.754f)),
            vorrq_u32(vcgeq_f32(score,vdupq_n_f32(forward_max[i]-.001f)),
                      vcgeq_f32(score,vsubq_f32(vld1q_f32(reverse_max.data()+base),vdupq_n_f32(.001f)))));
        if (vmaxvq_u32(mask) == 0) continue;
        vst1q_u32(active,mask);
#else
        for (size_t lane = 0; lane < 4 && base+lane < nb; ++lane) {
          const size_t j = base+lane; const float v = float(similarities[i*slots+j]);
          active[lane] = v >= .754f && (v >= forward_max[i]-.001f || v >= reverse_max[j]-.001f);
        }
#endif
        for (size_t lane = 0; lane < 4 && base+lane < nb; ++lane) {
          if (!active[lane]) continue;
          const size_t j = base+lane;
          if (result.comparisons >= std::min<uint64_t>(4096, options.max_comparisons)) { result.status = featmap::MatchStatus::budget_exceeded; return result; }
          ++result.comparisons;
          const float d = distance.squared_distance(descriptor_row<Float64_F32>(query.descriptors.ptr<float>(int(j))));
          forward[i].add(uint32_t(j), d); reverse[j].add(uint32_t(i), d);
        }
      }
    }
    result.pairs.reserve(std::min(na,nb));
    const auto gate = UnitL2MatchPolicy::gate();
    for (size_t i = 0; i < na; ++i) {
      const auto f = forward[i].distances();
      if (!f.accepts(gate)) continue;
      const auto r = reverse[f.first.id].distances();
      if (r.first.id == i && r.accepts(gate)) result.pairs.push_back({uint32_t(i), f.first.id, f.first.distance});
    }
    result.status = result.pairs.size() >= options.min_matches ? featmap::MatchStatus::success : featmap::MatchStatus::insufficient_matches;
    return result;
  }
#else
  void initialize(const std::string&, uint64_t, int) { throw std::runtime_error("XFeat requires EXTERIOR_WITH_RKNN"); }
  featmap::MatchResult match(const featmap::LandmarkMap&, const bow::FeatureBlock&, const MatchOptions&) { throw std::runtime_error("XFeat requires EXTERIOR_WITH_RKNN"); }
#endif
};
XFeatKnn::XFeatKnn(const std::string& model, uint64_t identity, int core) : impl_(std::make_unique<Impl>()) { impl_->initialize(model,identity,core); }
XFeatKnn::~XFeatKnn() = default;
featmap::MatchResult XFeatKnn::match(const featmap::LandmarkMap& map, const bow::FeatureBlock& query, const MatchOptions& options) {
  const auto begin = Clock::now(); const double cpu = owner_cpu_ms();
  auto result = impl_->match(map,query,options);
  EXT_LOG_INFO("[xfeat_match] map={} query={} pairs={} refined={} status={} wall_ms={:.3f} owner_cpu_ms={:.3f}", map.points().size(), query.keypoints.size(), result.pairs.size(), result.comparisons, int(result.status), std::chrono::duration<double,std::milli>(Clock::now()-begin).count(), owner_cpu_ms()-cpu);
  return result;
}
}
