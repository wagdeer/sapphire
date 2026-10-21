#pragma once
#if EXTERIOR_WITH_RKNN
#include <rknn_api.h>

#include <array>
#include <backend/tools/logging.hpp>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
namespace sapphire::backend::feature::detail {
inline void xfeat_require(bool ok, const char* reason) {
  if (!ok) throw std::runtime_error(reason);
}
struct XFeatNetwork {
  XFeatNetwork() = default;
  XFeatNetwork(const XFeatNetwork&) = delete;
  XFeatNetwork& operator=(const XFeatNetwork&) = delete;
  rknn_context context = 0;
  std::array<rknn_tensor_mem*, 2> input{};
  rknn_output output{};
  ~XFeatNetwork() {
    for (auto* memory : input)
      if (memory) rknn_destroy_mem(context, memory);
    if (context) rknn_destroy(context);
  }
  void initialize(const std::string& path, int core, int inputs, int elements, void* output_buffer, int output_bytes, bool float_output) {
    xfeat_require(rknn_init(&context, const_cast<char*>(path.c_str()), 0, 0, nullptr) == 0, "XFeat RKNN initialization failed");
    xfeat_require(rknn_set_core_mask(context, static_cast<rknn_core_mask>(1 << core)) == 0, "XFeat core binding failed");
    rknn_input_output_num counts{};
    xfeat_require(rknn_query(context, RKNN_QUERY_IN_OUT_NUM, &counts, sizeof(counts)) == 0 && counts.n_input == unsigned(inputs) && counts.n_output == 1,
                  "Unexpected XFeat model interface");
    for (int i = 0; i < inputs; ++i) {
      rknn_tensor_attr attr{};
      attr.index = i;
      xfeat_require(rknn_query(context, RKNN_QUERY_NATIVE_INPUT_ATTR, &attr, sizeof(attr)) == 0 && attr.type == RKNN_TENSOR_FLOAT16 &&
                        attr.n_elems == unsigned(elements) && attr.size == unsigned(elements * 2) && attr.size_with_stride == attr.size,
                    "Unexpected XFeat native input layout");
      if (inputs == 1) xfeat_require(attr.fmt == RKNN_TENSOR_NHWC, "XFeat extractor requires NHWC input");
      input[i] = rknn_create_mem(context, attr.size_with_stride);
      xfeat_require(input[i] != nullptr, "XFeat input allocation failed");
      attr.pass_through = 1;
      xfeat_require(rknn_set_io_mem(context, input[i], &attr) == 0, "XFeat native input binding failed");
    }
    rknn_tensor_attr attr{};
    xfeat_require(rknn_query(context, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr)) == 0 && attr.n_elems == unsigned(output_bytes / (float_output ? 4 : 2)),
                  "Unexpected XFeat output shape");
    if (float_output)
      xfeat_require(attr.n_dims == 4 && attr.dims[1] == 64 && attr.dims[2] == 28 && attr.dims[3] == 40, "XFeat output must be NCHW64x28x40");
    else
      xfeat_require(attr.type == RKNN_TENSOR_FLOAT16, "Matcher output must be FP16");
    output.buf = output_buffer;
    output.size = output_bytes;
    output.is_prealloc = 1;
    output.want_float = float_output;
    rknn_mem_size memory{};
    if (rknn_query(context, RKNN_QUERY_MEM_SIZE, &memory, sizeof(memory)) == 0)
      EXT_LOG_INFO("[xfeat_memory] model={} weights={} internal={} dma_reported={} host_output={} (overlapping accounting)",
                   inputs == 1 ? "extractor" : "matcher", memory.total_weight_size, memory.total_internal_size, memory.total_dma_allocated_size, output_bytes);
  }
  void run() {
    xfeat_require(rknn_run(context, nullptr) == 0, "XFeat inference failed");
    // Fixed caller-owned output. Extractor uses the runtime's explicit NCHW
    // conversion; no asynchronous fence path or per-call output allocation.
    xfeat_require(rknn_outputs_get(context, 1, &output, nullptr) == 0, "XFeat output failed");
    xfeat_require(rknn_outputs_release(context, 1, &output) == 0, "XFeat output release failed");
  }
  cv::float16_t* data(int i) { return static_cast<cv::float16_t*>(input[i]->virt_addr); }
};
}  // namespace sapphire::backend::feature::detail
#endif
