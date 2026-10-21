#pragma once

#include <backend/common/common_type.hpp>

namespace sapphire::backend {

struct DatabaseParams {
  unsigned int sqlite_cache_size_pages = 10000;
  bool sqlite_in_memory = false;
  int sqlite_journal_mode = 2;
  int sqlite_synchronous_mode = 2;
  int sqlite_temp_store = 2;
  unsigned int writer_batch_max_bytes = 33554432;
  unsigned int writer_batch_max_items = 128;
  unsigned int writer_batch_max_latency_ms = 50;
  unsigned int writer_commit_timeout_ms = 2000;
  unsigned int writer_max_bytes = 268435456;
  unsigned int writer_max_items = 1024;
  unsigned int writer_producer_wait_ms = 0;

  DatabaseParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

}  // namespace sapphire::backend
