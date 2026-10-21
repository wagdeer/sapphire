#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <backend/common/visual_frame.hpp>

namespace sapphire::backend::feature {
// Serialized backend worker only. This model consumes 320x240 gray replicated to RGB,
// resized to 256x256 with INTER_AREA and normalized to [-1, 1]. It must not be
// mixed with descriptors from a different model/preprocessing version.
class Siglip2Descriptor final {
 public:
  explicit Siglip2Descriptor(const std::string& model, int npu_core = 0);
  ~Siglip2Descriptor();
  Siglip2Descriptor(const Siglip2Descriptor&) = delete;
  Siglip2Descriptor& operator=(const Siglip2Descriptor&) = delete;
  // Returns an owned, unit-normalized 1x768 CV_32F row. Native RKNN buffers
  // remain private and reusable; callers may retain the returned descriptor.
  cv::Mat describe(const cv::Mat& gray);
  GlobalDescriptor encode(const cv::Mat& gray);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace sapphire::backend::feature
