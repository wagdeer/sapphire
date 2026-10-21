#include <backend/registration/descriptor/siglip2.hpp>
#include <backend/registration/descriptor/xfeat.hpp>
#include <backend/registration/matcher/descriptor_match.hpp>
#include <backend/registration/utils/rknn_models.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/timer.hpp>
#include <stdexcept>
namespace sapphire::backend::feature {
RknnModels::RknnModels(const FeatureParams& config, const RuntimeResources& resources)
    : parameters_(config.parameters()), npu_core_(resources.npu_core) {
  config.validate();
  if (!config.uses_xfeat()) return;
  if (npu_core_ < 0 || npu_core_ > 2) throw std::invalid_argument("Runtime NPU core must be 0, 1 or 2");
  const auto directory = RuntimeResources::weight_directory();
  const auto descriptor = directory / "xfeat_descriptor.rknn";
  const auto matcher = directory / "match512_matrix.rknn";
  const auto scene = directory / "siglip2_b32_split4.rknn";
  for (const auto& file : {descriptor, matcher, scene})
    if (!std::filesystem::is_regular_file(file))
      throw std::runtime_error("Backend model package is incomplete: " + file.string());
  tools::Timer timer;
  // Disposable deterministic input; never inserted into any map or cache.
  cv::Mat gray(240, 320, CV_8UC1);
  for (int y = 0; y < gray.rows; ++y)
    for (int x = 0; x < gray.cols; ++x) gray.at<unsigned char>(y, x) = (x * 17 + y * 31) % 256;
  {
    xfeat_ = std::make_shared<XFeatDescriptor>(descriptor.string(), npu_core_);
    xfeat_matcher_ = std::make_shared<XFeatKnn>(matcher.string(), xfeat_->model_id(), npu_core_);
    const std::vector<cv::KeyPoint> keys{cv::KeyPoint(160.f, 120.f, 1.f)};
    auto descriptors = xfeat_->describe(gray, keys);
    auto frame = bow::FeatureBlock::create_learned(keys, {cv::Point3f(0, 0, 1)}, descriptors, xfeat_->model_id());
    auto map = featmap::LandmarkMap::from_frame(*frame, 512);
    const MatchOptions options{1.f, 50, 1, 1};  // One disposable warmup pair.
    if (xfeat_matcher_->match(*map, *frame, options).pairs.size() != 1) throw std::runtime_error("XFeat warmup identity match failed");
  }
  {
    global_ = std::make_shared<Siglip2Descriptor>(scene.string(), npu_core_);
    global_->encode(gray);
  }
  EXT_LOG_INFO("[models] prepared before Ready in {:.3f}s; xfeat={} global={}; no idle inference", timer.elapsed_seconds(), bool(xfeat_), bool(global_));
}
void RknnModels::validate(const FeatureParams& config, const RuntimeResources& resources) const {
  validate(config);
  if (npu_core_ != resources.npu_core) throw std::invalid_argument("Prepared NPU core differs from startup resources");
}
void RknnModels::validate(const FeatureParams& config) const {
  if (parameters_ != config.parameters()) throw std::invalid_argument("Prepared model configuration differs from startup snapshot");
}
}  // namespace sapphire::backend::feature
