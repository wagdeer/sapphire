#pragma once
#include <cstdlib>
#include <charconv>
#include <stdexcept>
#include <memory>
namespace sapphire::backend::bow { class FbowVocabulary; }
#include <filesystem>
#include <string>

namespace sapphire::backend {
// Deployment paths are resolved at startup or supplied by the task/service.
// They are not algorithm tuning and are never restored from parameter metadata.
struct RuntimeResources {
  // Immutable tree shared across sequential online tasks; postings remain map-local.
  std::shared_ptr<const bow::FbowVocabulary> prepared_vocabulary;
  std::string vocabulary_path;
  std::string vocabulary_hash;
  std::string working_directory;
  int npu_core = 0;  // Runtime affinity, shared by XFeat and SigLIP.

  static std::filesystem::path weight_directory();

  static RuntimeResources defaults() {
    const auto env = [](const char* name) {
      const auto* value = std::getenv(name);
      return value ? std::string(value) : std::string{};
    };
    RuntimeResources resources;
    const auto core = env("PDR_BACKEND_NPU_CORE");
    if (!core.empty()) {
      const auto result = std::from_chars(core.data(), core.data() + core.size(), resources.npu_core);
      if (result.ec != std::errc{} || result.ptr != core.data() + core.size() || resources.npu_core < 0 || resources.npu_core > 2)
        throw std::invalid_argument("PDR_BACKEND_NPU_CORE must be 0, 1 or 2");
    }
    resources.vocabulary_path = env("PDR_VOCABULARY_PATH");
    resources.vocabulary_hash = env("PDR_VOCABULARY_HASH");
    resources.working_directory = env("PDR_WORKING_DIRECTORY");
    if (resources.vocabulary_path.empty())
      resources.vocabulary_path = (weight_directory() / "orb_vocab.fbow").string();
    return resources;
  }
};
}  // namespace sapphire::backend
