#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace sapphire::backend {

// Advisory route evidence, independent of node residency and Bayes likelihoods.
// Mapping samples seed routes; only committed geometric revisits anchor a query.
class VisitHistory {
 public:
  using Resolve = std::function<int(int)>;  // Current owner, or zero if unavailable.
  struct Visit {
    uint64_t segment = 0;
    int session = 0, query = 0, place = 0;
    double stamp = 0;  // Sensor time, never cache access time.
    bool verified = false;
  };

  VisitHistory(size_t capacity = 4096, double max_gap_seconds = 5);
  void record(int session, int query, double stamp, int place, bool verified);
  void break_sequence();
  std::vector<int> predict(int session, double stamp, const Resolve& resolve, size_t limit) const;
  // Preserve the global appearance leader; never introduce an unobserved candidate.
  static bool reorder(std::vector<int>& candidates, const std::vector<int>& predicted);
  std::string encode() const;
  void restore(const std::string& archive);
  size_t size() const { return visits_.size(); }
  size_t memory_size() const { return sizeof(*this) + visits_.size() * sizeof(Visit); }
  const std::deque<Visit>& visits() const { return visits_; }

 private:
  size_t capacity_;
  double max_gap_seconds_;
  std::deque<Visit> visits_;
  uint64_t segment_ = 0;
  int session_ = -1, query_ = 0;
  double stamp_ = 0;
  bool active_ = false, verified_ = false;
  int previous_place_ = 0, current_place_ = 0;
};
}  // namespace sapphire::backend
