#include <dlfcn.h>

#include <backend/io/runtime_resources.hpp>
#include <cstdlib>

namespace sapphire::backend {
namespace {
const char resource_anchor = 0;
}
std::filesystem::path RuntimeResources::weight_directory() {
  if (const char* directory = std::getenv("SAPPHIRE_MODEL_DIR"); directory && *directory) return directory;
  Dl_info info{};
  if (!dladdr(&resource_anchor, &info) || !info.dli_fname) throw std::runtime_error("Cannot locate backend model resources");
  const auto library_dir = std::filesystem::canonical(info.dli_fname).parent_path();
  const auto relative = library_dir.lexically_relative(EXTERIOR_BUILD_LIBRARY_DIR);
  if (!relative.empty() && *relative.begin() != "..") return EXTERIOR_SOURCE_MODEL_DIR;
  // Static core executables may live in bin or lib/<ros-package>; shared cores
  // live in lib. Resolve the same installed resource tree from either layout.
  auto prefix = library_dir;
  for (int level = 0; level < 4 && !prefix.empty(); ++level, prefix = prefix.parent_path()) {
    const auto candidate = prefix / "share/sapphire/data/models";
    if (std::filesystem::is_regular_file(candidate / "orb_vocab.fbow")) return candidate;
  }
  // Disabled vision may run without any visual assets. Enabled consumers report
  // the missing specific resource when opening it.
  return (library_dir / EXTERIOR_MODELS_FROM_LIB).lexically_normal();
}
}  // namespace sapphire::backend
