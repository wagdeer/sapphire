#include <backend/registration/descriptor/xfeat.hpp>
#include "backend/registration/utils/npu_core.hpp"
#include "backend/registration/utils/xfeat_network.hpp"
#include <array>
#include <cmath>
#include <fstream>
#include <chrono>
#include <ctime>
#include <backend/tools/logging.hpp>
#include <limits>
namespace sapphire::backend::feature {
namespace {
using Clock = std::chrono::steady_clock;
double owner_cpu_ms() { timespec t{}; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t); return 1000.*t.tv_sec + 1e-6*t.tv_nsec; }
#if EXTERIOR_WITH_RKNN
constexpr int slots = 512, dimensions = 64, pixels = 28 * 40;
void require(bool ok, const char* reason) { if (!ok) throw std::runtime_error(reason); }
float cubic(float x) {
  x = std::abs(x);
  return x <= 1 ? ((1.25f*x-2.25f)*x)*x+1 : x < 2 ? ((-.75f*x+3.75f)*x-6)*x+3 : 0;
}
uint64_t fingerprint(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  uint64_t hash = 14695981039346656037ULL;
  auto mix = [&](unsigned char x) { hash ^= x; hash *= 1099511628211ULL; };
  std::array<char, 65536> buffer;
  while (file.read(buffer.data(), buffer.size()) || file.gcount())
    for (std::streamsize i = 0; i < file.gcount(); ++i) mix(buffer[i]);
  require(file.eof(), "Cannot read XFeat model");
  for (unsigned char c : std::string("xfeat64-gray320x240-resize320x224-instance-norm-bicubic-official-unit-f32-v1")) mix(c);
  return hash ? hash : 1;
}
#endif
}
struct XFeatDescriptor::Impl {
  uint64_t identity = 0;
  int core = 1;
#if EXTERIOR_WITH_RKNN
  detail::XFeatNetwork extractor;
  std::array<float, 320*224> resized{};
  std::array<float, dimensions*pixels> output{}, dense{};
  void initialize(const std::string& descriptor, int selected_core) {
    require(selected_core >= 0 && selected_core < 3, "Invalid XFeat NPU core");
    core = selected_core;
    std::lock_guard<std::mutex> lock(npu_core_mutexes[core]);
    identity = fingerprint(descriptor);
    extractor.initialize(descriptor, core, 1, 320*224, output.data(), sizeof(output), true);
  }
  cv::Mat describe(const cv::Mat& gray, const std::vector<cv::KeyPoint>& points) {
    require(gray.type() == CV_8UC1 && gray.size() == cv::Size(320,240) && points.size() <= slots, "XFeat requires 320x240 gray and at most 512 points");
    for (const auto& p : points) require(std::isfinite(p.pt.x) && std::isfinite(p.pt.y) && p.pt.x >= 0 && p.pt.x < 320 && p.pt.y >= 0 && p.pt.y < 240, "Invalid XFeat sampling point");
    cv::Mat descriptors(int(points.size()), dimensions, CV_32F);
    if (points.empty()) return descriptors;
    std::lock_guard<std::mutex> lock(npu_core_mutexes[core]);
    double sum = 0;
    for (int y = 0; y < 224; ++y) {
      const float sy = (y+.5f)*(240.f/224)-.5f;
      const int low = int(std::floor(sy)); const float fy = sy-low;
      const auto* a = gray.ptr<uint8_t>(std::max(0,low)); const auto* b = gray.ptr<uint8_t>(std::min(239,low+1));
      for (int x = 0; x < 320; ++x) { const float v = (a[x]*(1-fy)+b[x]*fy)*(1.f/255); resized[y*320+x] = v; sum += v; }
    }
    const float mean = sum/resized.size(); double variance = 0;
    for (float v : resized) { const float d = v-mean; variance += double(d)*d; }
    const float scale = 1/std::sqrt(variance/resized.size()+1e-5);
    auto* input = extractor.data(0);
    for (size_t i = 0; i < resized.size(); ++i) input[i] = cv::float16_t((resized[i]-mean)*scale);
    extractor.run();
    for (int p = 0; p < pixels; ++p) {
      float norm = 0;
      for (int c = 0; c < dimensions; ++c) { float v = output[c*pixels+p]; norm += v*v; }
      require(std::isfinite(norm), "Nonfinite XFeat output");
      const float inv = 1/std::max(1e-12f,std::sqrt(norm));
      for (int c = 0; c < dimensions; ++c) dense[p*dimensions+c] = output[c*pixels+p]*inv;
    }
    descriptors.setTo(0);
    for (size_t i = 0; i < points.size(); ++i) {
      // Exactly the official grid_sample convention, including zero padding.
      const float x = points[i].pt.x/319.f*40-.5f, y = (points[i].pt.y*224.f/240.f)/223.f*28-.5f;
      const int ix = std::floor(x), iy = std::floor(y); float* dst = descriptors.ptr<float>(int(i));
      for (int by = iy-1; by <= iy+2; ++by) for (int bx = ix-1; bx <= ix+2; ++bx) {
        if (bx < 0 || bx >= 40 || by < 0 || by >= 28) continue;
        const float weight = cubic(x-bx)*cubic(y-by); const float* src = dense.data()+(by*40+bx)*dimensions;
        for (int c = 0; c < dimensions; ++c) dst[c] += weight*src[c];
      }
      float norm = 0; for (int c = 0; c < dimensions; ++c) norm += dst[c]*dst[c];
      require(norm > 1e-12f && std::isfinite(norm), "Degenerate XFeat descriptor");
      const float inv = 1/std::sqrt(norm); for (int c = 0; c < dimensions; ++c) dst[c] *= inv;
    }
    return descriptors;
  }
#else
  void initialize(const std::string&, int) { throw std::runtime_error("XFeat requires EXTERIOR_WITH_RKNN"); }
  cv::Mat describe(const cv::Mat&, const std::vector<cv::KeyPoint>&) { throw std::runtime_error("XFeat requires EXTERIOR_WITH_RKNN"); }
#endif
};
XFeatDescriptor::XFeatDescriptor(const std::string& descriptor, int core) : impl_(std::make_unique<Impl>()) { impl_->initialize(descriptor,core); }
XFeatDescriptor::~XFeatDescriptor() = default;
uint64_t XFeatDescriptor::model_id() const { return impl_->identity; }
cv::Mat XFeatDescriptor::describe(const cv::Mat& image, const std::vector<cv::KeyPoint>& points) {
  const auto begin = Clock::now(); const double cpu = owner_cpu_ms();
  auto result = impl_->describe(image,points);
  EXT_LOG_INFO("[xfeat_extract] points={} wall_ms={:.3f} owner_cpu_ms={:.3f}", points.size(), std::chrono::duration<double,std::milli>(Clock::now()-begin).count(), owner_cpu_ms()-cpu);
  return result;
}
}
