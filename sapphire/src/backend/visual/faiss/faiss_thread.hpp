#pragma once
#include <omp.h>

namespace sapphire::mapping {
// Restore the caller thread's OpenMP setting, including on exceptions.
struct SingleThread {
  int before = omp_get_max_threads();
  SingleThread() { omp_set_num_threads(1); }
  ~SingleThread() { omp_set_num_threads(before); }
};
}  // namespace sapphire::mapping
