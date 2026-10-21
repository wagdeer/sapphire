#include "backend/registration/utils/npu_core.hpp"
#include <backend/registration/descriptor/siglip2.hpp>
#include <array>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <fstream>
#include <sstream>
#if EXTERIOR_WITH_RKNN
#include <rknn_api.h>
#endif

namespace sapphire::backend::feature {
struct Siglip2Descriptor::Impl {
  cv::Mat identity;
#if EXTERIOR_WITH_RKNN
  int selected_core = 0;
  rknn_context context = 0;
  rknn_tensor_mem *input = nullptr, *output = nullptr;
  std::array<cv::float16_t, 256> lut;
  cv::Mat resized;
  ~Impl() {
    if (output) rknn_destroy_mem(context, output);
    if (input) rknn_destroy_mem(context, input);
    if (context) rknn_destroy(context);
  }
  static void check(bool ok, const char* what) {
    if (!ok) throw std::runtime_error(what);
  }
  void initialize(const std::string& model, int core) {
    if (core < 0 || core > 2) throw std::invalid_argument("Global descriptor requires one NPU core: 0, 1, or 2");
    selected_core = core;
    std::lock_guard<std::mutex> lock(npu_core_mutexes.at(core));
    // Content fingerprint plus preprocessing contract, independent of deployment
    // path. This is an accidental model-mixing guard, not a security checksum.
    std::ifstream file(model, std::ios::binary);
    uint64_t fingerprint = 14695981039346656037ULL;
    std::array<char, 65536> block;
    while (file.read(block.data(), block.size()) || file.gcount())
      for (std::streamsize i = 0; i < file.gcount(); ++i) { fingerprint ^= uint8_t(block[i]); fingerprint *= 1099511628211ULL; }
    check(file.eof(), "Global descriptor model read failed");
    std::ostringstream version;
    version << "siglip2-b32-gray320x240-area256-rgb-fp16-v1:" << std::hex << fingerprint;
    const auto tag = version.str();
    identity = cv::Mat(1, int(tag.size()), CV_8U, const_cast<char*>(tag.data())).clone();
    check(rknn_init(&context, const_cast<char*>(model.c_str()), 0, 0, nullptr) == 0, "Global descriptor RKNN init failed");
    check(rknn_set_core_mask(context, static_cast<rknn_core_mask>(1 << core)) == 0, "Global descriptor core binding failed");
    rknn_input_output_num count{};
    check(rknn_query(context, RKNN_QUERY_IN_OUT_NUM, &count, sizeof(count)) == 0 && count.n_input == 1 && count.n_output == 1,
          "Global descriptor requires one input and one output");
    rknn_tensor_attr ia{}, oa{};
    check(rknn_query(context, RKNN_QUERY_NATIVE_INPUT_ATTR, &ia, sizeof(ia)) == 0, "Global descriptor input query failed");
    check(ia.type == RKNN_TENSOR_FLOAT16 && ia.fmt == RKNN_TENSOR_NHWC && ia.n_dims == 4 && ia.dims[0] == 1 &&
          ia.dims[1] == 256 && ia.dims[2] == 256 && ia.dims[3] == 3 && ia.n_elems == 256*256*3 &&
          ia.size_with_stride == 256*256*3*2, "Unsupported global descriptor native input layout");
    check(rknn_query(context, RKNN_QUERY_NATIVE_OUTPUT_ATTR, &oa, sizeof(oa)) == 0, "Global descriptor output query failed");
    check(oa.type == RKNN_TENSOR_FLOAT16 && oa.n_elems == 768 && oa.size_with_stride == 768*2 &&
          (oa.fmt == RKNN_TENSOR_NHWC || oa.fmt == RKNN_TENSOR_NCHW || oa.fmt == RKNN_TENSOR_UNDEFINED),
          "Unsupported global descriptor native output layout");
    input = rknn_create_mem(context, ia.size_with_stride);
    output = rknn_create_mem(context, oa.size_with_stride);
    check(input && output, "Global descriptor native allocation failed");
    ia.pass_through = 1;
    check(rknn_set_io_mem(context, input, &ia) == 0 && rknn_set_io_mem(context, output, &oa) == 0,
          "Global descriptor native binding failed");
    for (int i = 0; i < 256; ++i) lut[i] = cv::float16_t(float(i)*(1.f/127.5f)-1.f);
  }
  cv::Mat describe(const cv::Mat& gray) {
    if (gray.empty() || gray.type() != CV_8UC1 || gray.size()!=cv::Size(320,240))
      throw std::invalid_argument("Global descriptor requires a 320x240 mono8 reference image");
    std::lock_guard<std::mutex> lock(npu_core_mutexes.at(selected_core));
    cv::resize(gray, resized, {256, 256}, 0, 0, cv::INTER_AREA);
    auto* target = static_cast<cv::float16_t*>(input->virt_addr);
    for (int y = 0; y < 256; ++y) {
      const auto* row = resized.ptr<uint8_t>(y);
      for (int x = 0; x < 256; ++x) {
        const auto v = lut[row[x]];
        const int offset = 3*(256*y+x);
        target[offset] = target[offset+1] = target[offset+2] = v;
      }
    }
    // Synchronous execution is intentional. No async fence/wait driver path.
    check(rknn_run(context, nullptr) == 0, "Global descriptor inference failed");
    cv::Mat result(1, 768, CV_32F);
    const auto* values = static_cast<const cv::float16_t*>(output->virt_addr);
    auto* row = result.ptr<float>();
    float norm = 0;
    for (int i = 0; i < 768; ++i) { row[i] = float(values[i]); norm += row[i]*row[i]; }
    check(std::isfinite(norm) && norm > 1e-12f, "Invalid global descriptor output");
    const float scale = std::sqrt(norm);
    for (int i = 0; i < 768; ++i) row[i] /= scale;
    return result;
  }
#else
  void initialize(const std::string&, int) { throw std::runtime_error("Global descriptor requires EXTERIOR_WITH_RKNN"); }
  cv::Mat describe(const cv::Mat&) { throw std::runtime_error("Global descriptor requires EXTERIOR_WITH_RKNN"); }
#endif
};
Siglip2Descriptor::Siglip2Descriptor(const std::string& model, int core) : impl_(std::make_unique<Impl>()) {
  impl_->initialize(model, core);
}
Siglip2Descriptor::~Siglip2Descriptor() = default;
cv::Mat Siglip2Descriptor::describe(const cv::Mat& gray) { return impl_->describe(gray); }
GlobalDescriptor Siglip2Descriptor::encode(const cv::Mat& gray) {
  // Type 2 stays separate from the historical unversioned type-1 similarity.
  return GlobalDescriptor(2, describe(gray), impl_->identity);
}
}  // namespace sapphire::backend::feature
