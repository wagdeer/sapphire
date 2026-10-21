#include "backend/maplayer/node.hpp"

#include <algorithm>
#include <atomic>
#include <backend/maplayer/navimap.hpp>
#include <backend/tools/associative.hpp>
#include <backend/tools/logging.hpp>
#include <stdexcept>

namespace sapphire::backend {

Node::Node()
    : _id(0),  // invalid id
      _mapId(-1),
      _stamp(0.0),
      _weight(0),
      _saved(false),
      _modified(true),
      _linksModified(true),
      _enabled(false),
      _invalidWordsCount(0) {}

Node::Node(int id, int mapId, int weight, double stamp, const std::string& label, const StateGroupf& pose, VisualFrame frame)
    : _id(id),
      _mapId(mapId),
      _stamp(stamp),
      _weight(weight),
      _label(label),
      _saved(false),
      _modified(true),
      _linksModified(true),
      _enabled(false),
      _invalidWordsCount(0),
      _pose(pose) {
  set_reference_frame(std::move(frame));
}

void Node::set_reference_frame(VisualFrame frame) { feature_map_.set_reference_frame(std::move(frame)); }

void Node::addLinks(const std::list<Link>& links) {
  for (std::list<Link>::const_iterator iter = links.begin(); iter != links.end(); ++iter) {
    addLink(*iter);
  }
}
void Node::addLinks(const std::map<int, Link>& links) {
  for (std::map<int, Link>::const_iterator iter = links.begin(); iter != links.end(); ++iter) {
    addLink(iter->second);
  }
}
void Node::addLink(const Link& link) {
  EXT_ASSERT_MSG(link.from() == this->id(), "{}", fmt::format("{}->{} for node {} (type={})", link.from(), link.to(), this->id(), link.type()));
  EXT_ASSERT_MSG((link.to() != this->id()) || link.type() == Link::kPosePrior || link.type() == Link::kGravity, "{}",
                 fmt::format("{}->{} for node {} (type={})", link.from(), link.to(), this->id(), link.type()));
  EXT_ASSERT_MSG(link.to() == this->id() || _links.find(link.to()) == _links.end(), "{}",
                 fmt::format("Link {} (type={}) already added to node {}!", link.to(), link.type(), this->id()));
  _links.insert(std::make_pair(link.to(), link));
  _linksModified = true;
}

bool Node::hasLink(int idTo, std::optional<Link::Type> type) const {
  if (!type) {
    return _links.find(idTo) != _links.end();
  }
  if (idTo == 0) {
    for (std::multimap<int, Link>::const_iterator iter = _links.begin(); iter != _links.end(); ++iter) {
      if (*type == iter->second.type()) {
        return true;
      }
    }
  } else {
    for (std::multimap<int, Link>::const_iterator iter = _links.find(idTo); iter != _links.end() && iter->first == idTo; ++iter) {
      if (*type == iter->second.type()) {
        return true;
      }
    }
  }
  return false;
}

void Node::changeLinkIds(int idFrom, int idTo) {
  std::multimap<int, Link>::iterator iter = _links.find(idFrom);
  while (iter != _links.end() && iter->first == idFrom) {
    Link link = iter->second;
    _links.erase(iter++);
    link.setTo(idTo);
    _links.insert(std::make_pair(idTo, link));
    _linksModified = true;
    EXT_LOG_DEBUG("({}) neighbor ids changed from {} to {}", _id, idFrom, idTo);
  }
}

void Node::removeLinks(bool keepSelfReferringLinks) {
  size_t sizeBefore = _links.size();
  if (keepSelfReferringLinks) {
    for (std::multimap<int, Link>::iterator iter = _links.begin(); iter != _links.end();) {
      if (iter->second.from() == iter->second.to()) {
        ++iter;
      } else {
        _links.erase(iter++);
      }
    }
  } else {
    _links.clear();
  }
  if (_links.size() != sizeBefore) _linksModified = true;
}

void Node::removeLink(int idTo) {
  int count = (int)_links.erase(idTo);
  if (count) {
    EXT_LOG_DEBUG("Removed link {} from {}", idTo, this->id());
    _linksModified = true;
  }
}

float Node::compareTo(const Node& s) const {
  // Optional/versioned semantic descriptors may be absent in an older node.
  // Type-2 retrieval never participates in this legacy rehearsal score.

  float similarity = 0.0f;
  int totalDescs = 0;

  for (size_t i = 0; i < std::min(featmap().global_descriptors().size(), s.featmap().global_descriptors().size()); ++i) {
    if (featmap().global_descriptors()[i].type() == 1 && s.featmap().global_descriptors()[i].type() == 1) {
      // rescale dot product from -1<->1 to 0<->1 (we assume normalized vectors!)
      float dotProd = (featmap().global_descriptors()[i].data().dot(s.featmap().global_descriptors()[i].data()) + 1.0f) / 2.0f;
      EXT_ASSERT_MSG(dotProd >= 0, "Global descriptors should be normalized!");
      similarity += dotProd;
      totalDescs += 1;
    }
  }

  if (totalDescs) {
    similarity /= totalDescs;
  } else {
    if (!s.isBadNode() && !isBadNode() && retrieval_mass() && s.retrieval_mass()) {
      const auto& other = s.retrieval_words();
      const int total = std::max(feature_count_ - _invalidWordsCount, s.feature_count() - s.getInvalidWordsCount());
      size_t a = 0, b = 0;
      double common = 0;
      const bool submap_comparison = mapped_features_ || s.has_map_features();
      const double left_mass = retrieval_mass(), right_mass = s.retrieval_mass();
      while (a < retrieval_words_.size() && b < other.size()) {
        const auto& left = retrieval_words_[a];
        const auto& right = other[b];
        if (left.word_id < right.word_id) {
          ++a;
          continue;
        }
        if (right.word_id < left.word_id) {
          ++b;
          continue;
        }
        if (left.word_id > 0)
          common +=
              submap_comparison ? std::min(double(left.count) / left_mass, double(right.count) / right_mass) : std::min(left.count, right.count);
        ++a;
        ++b;
      }
      similarity = submap_comparison ? common : common / total;
    }
  }
  return similarity;
}

const std::multimap<int, int>& Node::getWords() const {
  // Legacy inspection boundary. Retrieval and matching consume compact storage directly.
  if (_words.empty() && feature_count_) {
    if (!mapped_features_ && loop_features()) {
      for (size_t i = 0; i < loop_features()->word_ids.size(); ++i) _words.emplace(loop_features()->word_ids[i], static_cast<int>(i));
    } else {
      for (const auto& word : retrieval_words_)
        for (uint32_t i = 0; i < word.count; ++i) _words.emplace_hint(_words.end(), word.word_id, -1);
    }
  }
  return _words;
}

void Node::set_retrieval_words(std::vector<bow::WordCount> words, uint64_t vocabulary_hash) {
  uint32_t count = 0, invalid = 0;
  int previous = std::numeric_limits<int>::min();
  for (size_t i = 0; i < words.size(); ++i) {
    const auto& word = words[i];
    if (!word.count || word.count > bow::FeatureBlock::max_features - count || (i && word.word_id <= previous))
      throw std::invalid_argument("Invalid retrieval word summary");
    count += word.count;
    if (word.word_id <= 0) invalid += word.count;
    previous = word.word_id;
  }
  features_erased_ = false;
  mapped_features_ = false;
  feature_modified_ = false;
  clear_feature_cache();
  _words.clear();
  retrieval_words_ = std::move(words);
  retrieval_mass_ = count - invalid;
  feature_count_ = count;
  _invalidWordsCount = invalid;
  vocabulary_hash_ = vocabulary_hash;
}

void Node::set_raw_summary(uint32_t count) {
  if (count > bow::FeatureBlock::max_features) throw std::invalid_argument("Raw ORB feature count exceeded");
  set_retrieval_words({}, 0);
  feature_count_ = retrieval_mass_ = count;
}

void Node::set_loop_features(std::shared_ptr<const bow::FeatureBlock> features) {
  if (mapped_features_) {
    if (features && !features->is_raw() && features->vocabulary_hash != vocabulary_hash_)
      throw std::invalid_argument("Query/submap vocabulary mismatch");
  } else if (features && features->is_raw()) {
    set_raw_summary(features->keypoints.size());
  } else {
    std::vector<bow::WordCount> words;
    if (features) {
      auto ids = features->word_ids;
      std::sort(ids.begin(), ids.end());
      for (int id : ids) {
        if (words.empty() || words.back().word_id != id) words.push_back({id, 0});
        ++words.back().count;
      }
    }
    set_retrieval_words(std::move(words), features ? features->vocabulary_hash : 0);
  }
  feature_map_.set_loop_features(std::move(features));
}

void Node::set_feature_summary(std::vector<bow::WordCount> words, uint64_t hash, uint32_t point_count) {
  static std::atomic<uint64_t> next_token{0};
  if (point_count > max_natural_points) throw std::invalid_argument("Node point count exceeded");
  if (!hash && !words.empty()) throw std::invalid_argument("Raw submap cannot have visual words");
  uint32_t mass = 0;
  int previous = 0;
  for (const auto& word : words) {
    if (word.word_id <= previous || !word.count || word.count > point_count * featmap::word_credits - mass)
      throw std::invalid_argument("Invalid submap retrieval summary");
    mass += word.count;
    previous = word.word_id;
  }
  if (mass % featmap::word_credits) throw std::invalid_argument("Incomplete submap point votes");
  _words.clear();
  clear_feature_cache();
  features_erased_ = false;
  mapped_features_ = true;
  feature_token_ = ++next_token;
  feature_modified_ = false;
  retrieval_words_ = std::move(words);
  vocabulary_hash_ = hash;
  feature_count_ = point_count;
  retrieval_mass_ = hash ? mass : point_count;
  _invalidWordsCount = hash ? point_count - mass / featmap::word_credits : 0;
}

void Node::set_map_features(std::shared_ptr<const featmap::LandmarkMap> features) {
  if (!features) throw std::invalid_argument("Missing submap features");
  if (features->max_points() > max_natural_points) throw std::invalid_argument("Node point capacity exceeded");
  set_feature_summary(features->retrieval_words(), features->vocabulary_hash(), features->points().size());
  feature_map_.set_landmarks(std::move(features));
  _modified = true;
  feature_modified_ = true;
}

void Node::cache_map_features(std::shared_ptr<const featmap::LandmarkMap> features) {
  if (!mapped_features_ || !features || features->max_points() > max_natural_points || features->vocabulary_hash() != vocabulary_hash_ ||
      features->points().size() != feature_count_)
    throw std::runtime_error("Node cache metadata mismatch");
  const auto words = features->retrieval_words();
  if (words.size() != retrieval_words_.size() || !std::equal(words.begin(), words.end(), retrieval_words_.begin(), [](const auto& a, const auto& b) {
        return a.word_id == b.word_id && a.count == b.count;
      }))
    throw std::runtime_error("Node cache retrieval mismatch");
  feature_map_.set_landmarks(std::move(features));
}

void Node::setWords(const std::multimap<int, int>& words, const std::vector<cv::KeyPoint>& keypoints, const std::vector<cv::Point3f>& points,
                    const cv::Mat& descriptors) {
  EXT_ASSERT_MSG(descriptors.empty() || descriptors.rows == (int)words.size(), "{}",
                 fmt::format("words={}, descriptors={}", (int)words.size(), descriptors.rows));
  EXT_ASSERT_MSG(points.empty() || points.size() == words.size(), "{}", fmt::format("words={}, points={}", (int)words.size(), (int)points.size()));
  EXT_ASSERT_MSG(keypoints.empty() || keypoints.size() == words.size(), "{}",
                 fmt::format("words={}, descriptors={}", (int)words.size(), (int)keypoints.size()));
  // EXT_ASSERT(words.empty() || !keypoints.empty() || !points.empty() || !descriptors.empty());

  _invalidWordsCount = 0;
  for (std::multimap<int, int>::const_iterator iter = words.begin(); iter != words.end(); ++iter) {
    if (iter->first <= 0) {
      ++_invalidWordsCount;
    }
    // make sure indexes are all valid!
    EXT_ASSERT_MSG(iter->second < 0 || iter->second < (int)words.size(), "{}",
                   fmt::format("iter->second={} words.size()={}", iter->second, (int)words.size()));
  }

  _enabled = false;
  std::vector<bow::WordCount> summary;
  for (const auto& word : words) {
    if (summary.empty() || summary.back().word_id != word.first) summary.push_back({word.first, 0});
    ++summary.back().count;
  }
  set_retrieval_words(std::move(summary), 0);
  _words = words;
  auto frame = visualFrame();
  frame.setFeatures(keypoints, points, descriptors);
  feature_map_.set_reference_frame(std::move(frame));
}

bool Node::isBadNode() const { return !has_lidar() && !has_natural_features(); }
bool Node::has_natural_features() const {
  if (loop_features() && loop_features()->is_raw()) return !loop_features()->keypoints.empty();
  if (mapped_features_ && loop_features())
    return std::any_of(loop_features()->word_ids.begin(), loop_features()->word_ids.end(), [](int word) { return word > 0; });
  return feature_count_ > static_cast<uint32_t>(_invalidWordsCount);
}

void Node::removeAllWords() {
  set_retrieval_words({}, 0);
  features_erased_ = true;
  _modified = true;
  feature_map_.clear_reference_features();
  _invalidWordsCount = 0;
}

unsigned long Node::getMemoryUsed() const  // Return memory usage in Bytes
{
  unsigned long total = sizeof(Node) + (lidar_cloud_ ? lidar_cloud_->capacity() * sizeof(GaussianPoint) : 0);
  total += feature_map_.memory_size();
  total += retrieval_words_.capacity() * sizeof(bow::WordCount);
  total += _words.size() * (sizeof(int) * 2 + sizeof(std::multimap<int, cv::KeyPoint>::iterator)) + sizeof(std::multimap<int, cv::KeyPoint>);

  total += sizeof(_pose);
  total += _velocity.size() * sizeof(float);
  total += _links.size() * (sizeof(int) + sizeof(Link) + sizeof(std::multimap<int, Link>::iterator)) + sizeof(std::multimap<int, Link>);
  return total;
}

}  // namespace sapphire::backend

namespace sapphire::backend {
void Node::set_navimap(std::shared_ptr<const LocalMap> map) {
  if (map && map->anchor_id != id()) throw std::invalid_argument("Navigation layer anchor mismatch");
  navimap_map_ = std::move(map);
}
void Node::sync_feature_summary(featmap::FeatureMap content) {
  const auto& points = content.landmarks();
  set_feature_summary(points->retrieval_words(), points->vocabulary_hash(), points->points().size());
  feature_map_ = std::move(content);
  _modified = feature_modified_ = true;
}
void Node::set_feature_content(std::shared_ptr<const featmap::LandmarkMap> points, featmap::NodeObservations history) {
  if (!points || points->max_points() > max_natural_points) throw std::invalid_argument("Invalid node point capacity");
  if (history.reference_node && history.reference_node != id()) throw std::invalid_argument("Node observation datum mismatch");
  feature_map_.set_content(std::move(points), std::move(history));
  sync_feature_summary(feature_map_);
}
void Node::replace_landmark(uint64_t removed, uint64_t retained) {
  auto content = feature_map_;
  content.replace_landmark(removed, retained);
  sync_feature_summary(std::move(content));
}
void Node::retire_unsupported(const std::set<uint64_t>& observed) {
  feature_map_.retire_unsupported(observed);
  sync_feature_summary(feature_map_);
}
void Node::fuse_landmarks(const VisualFrame& query, const std::shared_ptr<const bow::FeatureBlock>& query_features, const StateGroupf& query_from_map,
                          const std::vector<featmap::Match>& seeds) {
  auto content = feature_map_;
  content.fuse_landmarks(query, query_features, query_from_map, seeds);
  if (content.landmarks() == feature_map_.landmarks()) return;
  // Prepare the retrieval summary before replacing the live content. A failed
  // allocation must not expose a new bank/history with the previous summary.
  sync_feature_summary(std::move(content));
}
void Node::maintain_landmarks(const VisualFrame& query, const std::shared_ptr<const bow::FeatureBlock>& features, int capture,
                              const StateGroupf& query_from_map, const std::vector<double>& pixel_variances,
                              const std::map<uint64_t, uint32_t>& verified_rows) {
  auto content = feature_map_;
  content.maintain_landmarks(query, features, capture, query_from_map, pixel_variances, verified_rows);
  if (&content.history() == &feature_map_.history()) return;
  if (content.landmarks() == feature_map_.landmarks()) {
    feature_map_ = std::move(content);
    _modified = feature_modified_ = true;
    return;
  }
  sync_feature_summary(std::move(content));
}
void Node::prune_views(float scale, const std::set<int>& protected_observers) {
  feature_map_.prune_views(id(), scale, protected_observers);
  _modified = feature_modified_ = true;
}
void Node::update_descriptors(const std::map<int, StateGroupf>& poses) {
  feature_map_.update_descriptors(poses);
  sync_feature_summary(feature_map_);
}
void Node::reject_observations(const std::vector<std::pair<int, uint64_t>>& rejected) {
  if (rejected.empty()) return;
  feature_map_.reject_observations(rejected);
  sync_feature_summary(feature_map_);
}
}  // namespace sapphire::backend
