#pragma once
#include <backend/common/visual_frame.hpp>
#include <functional>
#include <memory>
#include <vector>
namespace sapphire::backend::feature {
// Faiss float inner-product index, partitioned by model/preprocessing identity.
// One vector per canonical reference node; never indexes local ORB descriptors.
class FaissCosineIndex final {
 public:
  FaissCosineIndex();
  ~FaissCosineIndex();
  void set(int id, const std::vector<GlobalDescriptor>& descriptors);
  void erase(int id);
  void clear();
  size_t memory_size() const;
  // Optional scores are the exact Faiss inner products, aligned with returned IDs.
  // Every call clears the output, including empty/incompatible searches.
  std::vector<int> search(const std::vector<GlobalDescriptor>& descriptors, size_t limit,
                         const std::function<bool(int)>& eligible, std::vector<float>* scores = nullptr) const;
  // Score additional resident IDs needed by the temporal filter.
  std::vector<float> scores(const std::vector<GlobalDescriptor>& descriptors, const std::vector<int>& ids) const;
  static bool valid(const GlobalDescriptor& descriptor);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace sapphire::backend::feature
