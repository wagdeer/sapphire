#include <backend/parameters.hpp>
#include "backend/registration/retrieval/fbow.hpp"

#include <fbow/vocabulary.h>
#include <backend/registration/descriptor/feature_block.hpp>

#include <backend/tools/logging.hpp>
#include <stdexcept>

namespace sapphire::backend {

const int FbowIndex::ID_INVALID = 0;

bow::FbowVocabulary::~FbowVocabulary() = default;

std::shared_ptr<const bow::FbowVocabulary> FbowIndex::prepare(const std::string& path, const std::string& expected_hash) {
  if (path.empty()) throw std::invalid_argument("FBoW vocabulary path must not be empty.");
  auto vocabulary = fbow::Vocabulary::load_orb(path);
  const auto hash = vocabulary->hash();
  if (!expected_hash.empty() && expected_hash != std::to_string(hash))
    throw std::runtime_error(fmt::format("FBoW vocabulary hash mismatch (expected={} actual={} path={}).", expected_hash, hash, path));
  EXT_LOG_INFO("Loaded FBoW vocabulary {} (hash={}, blocks={}, memory={} MB).", path, hash,
               static_cast<int>(vocabulary->size()), static_cast<long>(vocabulary->memoryUsage() / (1024 * 1024)));
  auto prepared = std::shared_ptr<bow::FbowVocabulary>(new bow::FbowVocabulary);
  prepared->tree_ = std::move(vocabulary);
  prepared->hash_ = hash;
  return prepared;
}
FbowIndex::FbowIndex(const std::string& path, const std::string& expected_hash)
    : FbowIndex(prepare(path, expected_hash)) {}
FbowIndex::FbowIndex(std::shared_ptr<const bow::FbowVocabulary> prepared)
    : _totalActiveReferences(0), prepared_(std::move(prepared)) {
  if (!prepared_ || !prepared_->tree_ || !prepared_->tree_->isValid() || prepared_->tree_->getDescType() != CV_8UC1 || prepared_->tree_->getDescSize() != 32)
    throw std::invalid_argument("Prepared vocabulary must contain valid ORB-256 descriptors");
  _fbowVocabularyHash = prepared_->hash_;
}

FbowIndex::~FbowIndex() = default;

bool FbowIndex::isFBoW() const { return prepared_->tree_->isValid(); }

unsigned long long FbowIndex::getFBoWVocabularyHash() const { return _fbowVocabularyHash; }

std::vector<int> FbowIndex::quantize(const cv::Mat &descriptors, std::vector<uint32_t>* node_ids) const {
  if (descriptors.empty()) throw std::invalid_argument("Cannot quantize empty descriptors.");
  std::vector<int> wordIds;
  std::vector<uint32_t> local_nodes;
  prepared_->tree_->quantize_orb(descriptors, 4, wordIds, node_ids ? *node_ids : local_nodes);
  return wordIds;
}

FbowIndex::ReferenceUpdate FbowIndex::prepare_replacement(int retained, const std::vector<bow::WordCount>& old_retained,
                                                                int removed, const std::vector<bow::WordCount>& old_removed,
                                                                const std::vector<bow::WordCount>& replacement, int second_removed, const std::vector<bow::WordCount>& second_words) {
  if (retained <= 0 || removed <= 0 || retained == removed) throw std::invalid_argument("Invalid retrieval replacement owners");
  ReferenceUpdate update;
  update.owner = this;
  const auto touch = [&](int word) {
    if (word <= 0 || update.postings.count(word)) return;
    const auto found = _postings.find(word);
    update.postings.emplace(word, found == _postings.end() ? Posting{} : found->second);
  };
  for (const auto& word : old_retained) touch(word.word_id);
  for (const auto& word : old_removed) touch(word.word_id);
  for (const auto& word : second_words) touch(word.word_id);
  for (const auto& word : replacement) touch(word.word_id);
  for (auto& [word, posting] : update.postings) {
    posting.references.erase(retained);
    posting.references.erase(removed);
    if (second_removed) posting.references.erase(second_removed);
  }
  for (const auto& word : replacement)
    if (word.word_id > 0) update.postings.at(word.word_id).references[retained] = word.count;
  update.total = _totalActiveReferences;
  for (auto& [word, posting] : update.postings) {
    const auto old = _postings.find(word);
    if (old != _postings.end()) update.total -= old->second.totalReferences;
    posting.totalReferences = 0;
    for (const auto& [id, count] : posting.references) posting.totalReferences += count;
    update.total += posting.totalReferences;
  }
  _postings.reserve(_postings.size() + update.postings.size());
  return update;
}
void FbowIndex::apply_replacement(ReferenceUpdate&& update) noexcept {
  if (update.owner != this) std::terminate();
  // No re-quantization, new posting allocation or whole-dictionary copy after
  // the database commit: prepared node handles and existing references transfer.
  while (!update.postings.empty()) {
    auto node = update.postings.extract(update.postings.begin());
    const auto old = _postings.find(node.key());
    if (node.mapped().references.empty()) {
      if (old != _postings.end()) _postings.erase(old);
    } else if (old == _postings.end()) {
      _postings.insert(std::move(node));
    } else {
      old->second.references.swap(node.mapped().references);
      old->second.totalReferences = node.mapped().totalReferences;
    }
  }
  _totalActiveReferences = update.total;
  update.owner = nullptr;
}

void FbowIndex::addWordRef(int wordId, int nodeId) { add_word_references(wordId, nodeId, 1); }

void FbowIndex::add_word_references(int word_id, int node_id, uint32_t count) {
    if (word_id <= ID_INVALID || node_id == 0 || !count)
        throw std::invalid_argument("Invalid vocabulary reference");
    auto& posting = _postings[word_id];
    posting.references[node_id] += count;
    posting.totalReferences += count;
    _totalActiveReferences += count;
}

int FbowIndex::removeAllWordRef(int wordId, int nodeId) {
  const auto posting = _postings.find(wordId);
  if (posting == _postings.end()) {
    return 0;
  }
  const auto reference = posting->second.references.find(nodeId);
  if (reference == posting->second.references.end()) {
    return 0;
  }
  const int removed = reference->second;
  posting->second.references.erase(reference);
  posting->second.totalReferences -= removed;
  _totalActiveReferences -= removed;
  if (posting->second.references.empty()) {
    _postings.erase(posting);
  }
  return removed;
}

const FbowIndex::References *FbowIndex::getReferences(int wordId) const {
  const auto posting = _postings.find(wordId);
  return posting == _postings.end() ? nullptr : &posting->second.references;
}

bool FbowIndex::contains(int wordId) const { return _postings.find(wordId) != _postings.end(); }

unsigned int FbowIndex::getIndexedWordsCount() const { return static_cast<unsigned int>(_postings.size()); }

unsigned int FbowIndex::getIndexMemoryUsed() const { return static_cast<unsigned int>(prepared_->tree_->memoryUsage()); }

unsigned long FbowIndex::getMemoryUsed() const {
  unsigned long memory = sizeof(FbowIndex) + getIndexMemoryUsed() + _postings.bucket_count() * sizeof(void *);
  for (const auto &posting : _postings) {
    memory += sizeof(posting) + sizeof(Posting) + posting.second.references.size() * (sizeof(int) * 2 + sizeof(References::const_iterator));
  }
  return memory;
}

void FbowIndex::clear() {
  _postings.clear();
  _totalActiveReferences = 0;
}

}  // namespace sapphire::backend
