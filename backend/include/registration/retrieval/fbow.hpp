#pragma once

#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace fbow {
class Vocabulary;
}

namespace sapphire::backend {
class FbowIndex;

namespace bow {
struct WordCount;
// Immutable loaded tree + cached fingerprint; no per-map postings or workers.
class FbowVocabulary final {
 public:
  ~FbowVocabulary();
 private:
  friend class sapphire::backend::FbowIndex;
  FbowVocabulary() = default;
  std::unique_ptr<fbow::Vocabulary> tree_;
  unsigned long long hash_ = 0;
};
}

// One vocabulary loaded at construction; clear() only removes map-node references. FBoW owns vocabulary
// loading, descriptor validation and quantization; node lifetime stays here.
class FbowIndex final {
 public:
  using References = std::map<int, int>;
 private:
  struct Posting {
    References references;
    int totalReferences = 0;
  };
 public:
  class ReferenceUpdate {
   public:
    ReferenceUpdate(ReferenceUpdate&&) = default;
    ReferenceUpdate& operator=(ReferenceUpdate&&) = default;
    ReferenceUpdate(const ReferenceUpdate&) = delete;
    ReferenceUpdate& operator=(const ReferenceUpdate&) = delete;
   private:
    friend class FbowIndex;
    ReferenceUpdate() = default;
    const FbowIndex* owner = nullptr;
    std::unordered_map<int, Posting> postings;
    int total = 0;
  };
  // Stage only touched postings without changing visible references. Discard on
  // storage failure; apply after commit. The owner thread must not mutate the
  // dictionary between preparation and publication. Reservation happens here.
  ReferenceUpdate prepare_replacement(int retained, const std::vector<bow::WordCount>& old_retained,
                                      int removed, const std::vector<bow::WordCount>& old_removed,
                                      const std::vector<bow::WordCount>& replacement, int second_removed = 0, const std::vector<bow::WordCount>& second_words = {});
  void apply_replacement(ReferenceUpdate&& update) noexcept;

  static const int ID_INVALID;

  explicit FbowIndex(const std::string& path, const std::string& expected_hash = {});
  explicit FbowIndex(std::shared_ptr<const bow::FbowVocabulary> prepared);
  static std::shared_ptr<const bow::FbowVocabulary> prepare(const std::string& path, const std::string& expected_hash = {});
  ~FbowIndex();

  // Read-only preparation. Register references only after the node's final
  // frame/landmark representation has been constructed successfully.
  std::vector<int> quantize(const cv::Mat &descriptors, std::vector<uint32_t>* node_ids = nullptr) const;

  void addWordRef(int wordId, int nodeId);
  void add_word_references(int word_id, int node_id, uint32_t count);

  int removeAllWordRef(int wordId, int nodeId);

  int getTotalActiveReferences() const { return _totalActiveReferences; }

  const References *getReferences(int wordId) const;
  bool contains(int wordId) const;
  unsigned int getIndexedWordsCount() const;

  unsigned int getIndexMemoryUsed() const;

  unsigned long getMemoryUsed() const;

  bool isFBoW() const;

  unsigned long long getFBoWVocabularyHash() const;

  void clear();

 private:
  std::unordered_map<int, Posting> _postings;
  int _totalActiveReferences;
  std::shared_ptr<const bow::FbowVocabulary> prepared_;
  unsigned long long _fbowVocabularyHash;
};

}  // namespace sapphire::backend
