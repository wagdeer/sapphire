#pragma once
#include <backend/params/feature.hpp>
#include <backend/io/runtime_resources.hpp>
#include <memory>
namespace sapphire::backend::feature {
class XFeatDescriptor;
class XFeatKnn;
class Siglip2Descriptor;
// Owner-thread resources. Construct before accepting callbacks. No worker,
// polling, map references or descriptor cache; only fixed model/bound buffers.
class RknnModels final {
 public:
  explicit RknnModels(const FeatureParams& config, const RuntimeResources& resources = RuntimeResources::defaults());
  void validate(const FeatureParams& config, const RuntimeResources& resources) const;
  void validate(const FeatureParams& config) const;
  const std::shared_ptr<XFeatDescriptor>& xfeat() const { return xfeat_; }
  const std::shared_ptr<XFeatKnn>& xfeat_matcher() const { return xfeat_matcher_; }
  const std::shared_ptr<Siglip2Descriptor>& global() const { return global_; }

 private:
  ParametersMap parameters_;
  int npu_core_ = 0;
  std::shared_ptr<XFeatDescriptor> xfeat_;
  std::shared_ptr<XFeatKnn> xfeat_matcher_;
  std::shared_ptr<Siglip2Descriptor> global_;
};
}  // namespace sapphire::backend::feature
