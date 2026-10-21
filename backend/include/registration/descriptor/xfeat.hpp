#pragma once
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace sapphire::backend::feature {
// Descriptor inference only. Prepared once; synchronous execution, fixed buffers.
class XFeatDescriptor final {
 public:
  XFeatDescriptor(const std::string& descriptor_model, int core);
  ~XFeatDescriptor();
  uint64_t model_id() const;
  cv::Mat describe(const cv::Mat& gray, const std::vector<cv::KeyPoint>& points);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
