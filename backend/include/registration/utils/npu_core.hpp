#pragma once
#include <array>
#include <mutex>
namespace sapphire::backend::feature {
// Shared by all production model contexts, including startup warmup. This
// serializes this process only; deployments must reserve the selected core.
inline std::array<std::mutex, 3> npu_core_mutexes;
}
