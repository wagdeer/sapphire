#include <backend/common/common_type.hpp>
#include <backend/params/topograph.hpp>
#include "backend/maplayer/topograph/bayes_filter.hpp"

#include <algorithm>
#include <cmath>
#include <backend/tools/logging.hpp>
#include <backend/tools/strings.hpp>
#include <backend/tools/timer.hpp>
#include <set>
#include <stdexcept>
#include <unordered_map>

#include "backend/maplayer/node.hpp"
#include "backend/maplayer/topograph.hpp"
#include "backend/parameters.hpp"

namespace sapphire::backend {

BayesFilter::BayesFilter(const ParametersMap& parameters)
    : BayesFilter(TopographParams{}.withOverrides(parameters)) {}
BayesFilter::BayesFilter(const TopographParams& parameters) { configure(parameters); }
BayesFilter::~BayesFilter() = default;

void BayesFilter::configure(const TopographParams& parameters) {
  parameters.validate();
  _predictionLC = parameters.transitionWeights();
  _virtualPlacePrior = parameters.belief_virtual_place_prior_threshold;
  _fullPredictionUpdate = parameters.belief_rebuild_prediction_each_update;
  _totalPredictionLCValues = 0;
  _predictionEpsilon = static_cast<float>(*std::min_element(_predictionLC.begin(), _predictionLC.end()));
  for (double value : _predictionLC) _totalPredictionLCValues += value;
  invalidate_prediction();
}

const std::vector<double> &BayesFilter::getPredictionLC() const {
  // {Vp, Lc, l1, l2, l3, l4...}
  return _predictionLC;
}

void BayesFilter::reset() {
  _posterior.clear();
  invalidate_prediction();
  _posteriorWorkspace.clear();
  _priorWorkspace.clear();
  _backgroundWorkspace.clear();
}

void BayesFilter::clear_prediction_columns() {
  lookahead_ready_ = false;
  _predictionIds.clear();
  _predictionRowGenerations.clear();
  _predictionColumns.clear();
  _predictionGeneration = 0U;
}

void BayesFilter::invalidate_prediction() {
  clear_prediction_columns();
  _neighborsIndex.clear();
  graph_snapshot_.clear();
  excluded_snapshot_.clear();
}

bool BayesFilter::merge_places(int retained, int removed) {
  if (retained <= 0 || removed <= 0 || retained == removed) throw std::invalid_argument("Invalid Bayes place merge");
  lookahead_ready_ = false;
  const auto source = _posterior.find(removed);
  const bool moved = source != _posterior.end();
  if (moved) {
    const auto target = _posterior.find(retained);
    if (target == _posterior.end()) {
      // Rename in place; no allocation is needed at the merge commit boundary.
      auto node = _posterior.extract(source);
      node.key() = retained;
      _posterior.insert(std::move(node));
    } else {
      // A and B were exclusive hypotheses; their union adds mass, not evidence.
      target->second = static_cast<float>(static_cast<double>(target->second) + source->second);
      _posterior.erase(source);
    }
  }
  // Changed adjacency repairs affected neighborhoods at the next update.
  // This also covers removed STM places absent from belief. Keep the old graph
  // snapshot for that repair and do not reset the virtual-place probability.
  return moved;
}

const std::map<int, float> &BayesFilter::computePosterior(std::map<int, std::vector<int>> adjacency, const std::set<int> &excluded, const std::vector<int> &ids,
                                                          const std::vector<float> &likelihood) {
  lookahead_ready_ = false;
  EXT_LOG_DEBUG("");

  if (likelihood.empty() || ids.size() != likelihood.size()) {
    EXT_LOG_ERROR("likelihood is empty!");
    return _posterior;
  }

  if (_predictionLC.size() < 2) {
    EXT_LOG_ERROR("Prediction is not valid!");
    return _posterior;
  }

  tools::Timer timer;
  timer.reset();

  this->updatePrediction(std::move(adjacency), excluded, ids);
  return updateObservation(ids, likelihood);
}
const std::map<int, float>& BayesFilter::computePosterior(const Topograph& graph, const std::set<int>& excluded,
                                                        const std::vector<int>& ids, const std::vector<float>& likelihood) {
  if (likelihood.empty() || ids.size() != likelihood.size() || _predictionLC.size() < 2) return _posterior;
  neighbor_computations_ = 0;
  if (excluded != excluded_snapshot_) {
    // STM membership changes which destinations receive transition mass, not
    // the graph distances. Relation edits invalidate affected distances separately.
    clear_prediction_columns();
    excluded_snapshot_ = excluded;
  }
  updateColumns(excluded, ids, &graph);
  return updateObservation(ids, likelihood);
}
void BayesFilter::relations_changed(int source) noexcept {
  // The first changed edge on any new path was reachable in the old graph.
  // Inspect OLD cached depths before deleting them: this covers deletions,
  // contractions and insertions without reconstructing an old/new graph union.
  for (auto it = _neighborsIndex.begin(); it != _neighborsIndex.end();) {
    const auto reached = it->second.find(source);
    if (it->first == source || (reached != it->second.end() && unsigned(reached->second) <= _predictionLC.size()-2U))
      it = _neighborsIndex.erase(it);
    else ++it;
  }
  clear_prediction_columns();
}
const std::map<int, float>& BayesFilter::updateObservation(const std::vector<int>& ids, const std::vector<float>& likelihood) {
  lookahead_ready_ = false;
  tools::Timer timer;
  EXT_ASSERT(_predictionColumns.size() == ids.size() && _predictionRowGenerations.size() == ids.size());
  EXT_LOG_DEBUG("STEP1-update sparse prediction={:.6f}s, size={}", timer.lap_seconds(), ids.size());

  this->updatePosterior(ids);
  _posteriorWorkspace.clear();
  _posteriorWorkspace.reserve(ids.size());
  for (const auto &entry : _posterior) {
    _posteriorWorkspace.push_back(entry.second);
  }
  EXT_ASSERT(_posteriorWorkspace.size() == ids.size());

  const bool virtualPlaceUsed = ids.front() < 0;
  const std::size_t realBegin = virtualPlaceUsed ? 1U : 0U;
  _backgroundWorkspace.assign(static_cast<std::size_t>(_predictionGeneration) + 2U, 0.0);
  _priorWorkspace.assign(ids.size(), 0.0);
  for (std::size_t column = 0U; column < _predictionColumns.size(); ++column) {
    const PredictionColumn &prediction = _predictionColumns[column];
    EXT_ASSERT(prediction.backgroundGeneration <= _predictionGeneration);
    EXT_ASSERT(std::isfinite(prediction.background) && prediction.background >= 0.0f);
    EXT_ASSERT(std::isfinite(prediction.toVirtual) && prediction.toVirtual >= 0.0f);
    const double sourceProbability = static_cast<double>(_posteriorWorkspace[column]);
    if (prediction.background != 0.0f) {
      const double contribution = static_cast<double>(prediction.background) * sourceProbability;
      _backgroundWorkspace[0] += contribution;
      _backgroundWorkspace[static_cast<std::size_t>(prediction.backgroundGeneration) + 1U] -= contribution;
    }
    if (virtualPlaceUsed) {
      _priorWorkspace[0] += static_cast<double>(prediction.toVirtual) * sourceProbability;
    }
    std::uint32_t previousRow = 0U;
    bool firstOverride = true;
    for (const auto &overrideValue : prediction.overrides) {
      const std::size_t row = overrideValue.first;
      EXT_ASSERT(row >= realBegin && row < ids.size());
      EXT_ASSERT(firstOverride || previousRow < overrideValue.first);
      EXT_ASSERT(std::isfinite(overrideValue.second) && overrideValue.second >= 0.0f);
      firstOverride = false;
      previousRow = overrideValue.first;
      const float baseline = _predictionRowGenerations[row] <= prediction.backgroundGeneration ? prediction.background : 0.0f;
      _priorWorkspace[row] += static_cast<double>(overrideValue.second - baseline) * sourceProbability;
    }
  }
  for (std::size_t generation = 1U; generation < _backgroundWorkspace.size(); ++generation) {
    _backgroundWorkspace[generation] += _backgroundWorkspace[generation - 1U];
  }
  for (std::size_t row = realBegin; row < ids.size(); ++row) {
    _priorWorkspace[row] += _backgroundWorkspace[static_cast<std::size_t>(_predictionRowGenerations[row])];
  }
  EXT_LOG_DEBUG("STEP1-sparse multiply time={:.6f}s", timer.lap_seconds());

  double sum = 0.0;
  auto posterior = _posterior.begin();
  for (std::size_t i = 0U; i < ids.size(); ++i, ++posterior) {
    EXT_ASSERT(posterior != _posterior.end() && posterior->first == ids[i]);
    posterior->second = likelihood[i] * static_cast<float>(_priorWorkspace[i]);
    sum += posterior->second;
  }
  EXT_LOG_DEBUG("STEP2-likelihood time={:.6f}s", timer.lap_seconds());

  // Normalize
  EXT_LOG_DEBUG("sum={:.6f}", sum);
  if (sum != 0.0) {
    for (std::map<int, float>::iterator i = _posterior.begin(); i != _posterior.end(); ++i) {
      (*i).second = static_cast<float>(static_cast<double>((*i).second) / sum);
    }
    double normalizedSum = 0.0;
    for (const auto &entry : _posterior) {
      normalizedSum += entry.second;
    }
    EXT_ASSERT(std::abs(normalizedSum - 1.0) <= 1.0e-5);
  }
  EXT_LOG_DEBUG("normalize time={:.6f}s", timer.lap_seconds());
  // std::cout << "Posterior=" << _posterior << std::endl;
  lookahead_ready_ = true;
  return _posterior;
}

BayesFilter::PredictionColumn BayesFilter::buildPredictionColumn(unsigned int column, const std::map<int, int> &neighbors, const std::set<int>& excluded,
                                                                 const std::unordered_map<int, int> &idToIndex, unsigned int count,
                                                                 bool virtualPlaceUsed) const {
  PredictionColumn result;
  result.backgroundGeneration = _predictionGeneration;
  result.toVirtual = virtualPlaceUsed ? static_cast<float>(_predictionLC[0]) : 0.0f;

  std::map<std::uint32_t, float> explicitValues;
  float addedProbabilitiesSum = 0.0f;
  for (const auto &neighbor : neighbors) {
    if (neighbor.first < 0 || excluded.count(neighbor.first)) {
      continue;
    }
    const auto index = idToIndex.find(neighbor.first);
    if (index == idToIndex.end()) {
      continue;
    }
    EXT_ASSERT(neighbor.second + 1 < static_cast<int>(_predictionLC.size()));
    const float probability = static_cast<float>(_predictionLC[neighbor.second + 1]);
    explicitValues[static_cast<std::uint32_t>(index->second)] = probability;
    addedProbabilitiesSum += probability;
  }

  const float expectedLocalMass = _totalPredictionLCValues - static_cast<float>(_predictionLC[0]);
  if (addedProbabilitiesSum < expectedLocalMass) {
    const float delta = expectedLocalMass - addedProbabilitiesSum;
    explicitValues[column] += delta;
    addedProbabilitiesSum += delta;
  }

  for (auto iter = explicitValues.begin(); iter != explicitValues.end();) {
    if (iter->second == 0.0f) {
      iter = explicitValues.erase(iter);
    } else {
      ++iter;
    }
  }

  const unsigned int realBegin = virtualPlaceUsed ? 1U : 0U;
  const unsigned int realCount = count - realBegin;
  float background = 0.0f;
  if (_totalPredictionLCValues < 1.0f && count > 1U) {
    background = (1.0f - _totalPredictionLCValues) / static_cast<float>(count - 1U);
    addedProbabilitiesSum += background * static_cast<float>(realCount - explicitValues.size());
  }

  const float maxNorm = 1.0f - (virtualPlaceUsed ? static_cast<float>(_predictionLC[0]) : 0.0f);
  if (addedProbabilitiesSum < maxNorm - 0.0001f || addedProbabilitiesSum > maxNorm + 0.0001f) {
    const float scale = maxNorm / addedProbabilitiesSum;
    background *= scale;
    if (background < _predictionEpsilon) {
      background = 0.0f;
    }
    for (auto &entry : explicitValues) {
      entry.second *= scale;
      if (entry.second < _predictionEpsilon) {
        entry.second = 0.0f;
      }
    }
  }
  result.background = background;
  result.overrides.reserve(explicitValues.size());
  double columnSum = static_cast<double>(result.toVirtual) +
                     static_cast<double>(result.background) * static_cast<double>(realCount - explicitValues.size());
  for (const auto &entry : explicitValues) {
    EXT_ASSERT(std::isfinite(entry.second) && entry.second >= 0.0f);
    if (entry.second != background) {
      result.overrides.push_back(entry);
    }
    columnSum += entry.second;
  }
  EXT_ASSERT(std::isfinite(result.background) && result.background >= 0.0f);
  EXT_ASSERT(std::isfinite(result.toVirtual) && result.toVirtual >= 0.0f);
  if (columnSum < 0.99 || columnSum > 1.01) {
    EXT_LOG_WARN("Prediction is not normalized sum={:.6f}", columnSum);
  }
  return result;
}

void BayesFilter::updatePrediction(std::map<int, std::vector<int>> adjacency, const std::set<int> &excluded, const std::vector<int> &ids) {
  neighbor_computations_ = 0;
  EXT_ASSERT(_predictionLC.size() >= 2U && !ids.empty());
  auto graph = std::move(adjacency);
  if (graph != graph_snapshot_ || excluded != excluded_snapshot_) {
    auto reusable = std::move(_neighborsIndex);
    if (excluded != excluded_snapshot_ || _fullPredictionUpdate)
      reusable.clear();
    else {
      // A changed directed adjacency can affect only origins reaching its source
      // within the model horizon. Walk reverse edges in the OLD/NEW union so
      // deleted shortcuts, newly introduced shortcuts and contractions are covered.
      std::set<int> affected;
      for (const auto& entry : graph_snapshot_) {
        const auto now = graph.find(entry.first);
        if (now == graph.end() || now->second != entry.second) affected.insert(entry.first);
      }
      for (const auto& entry : graph) {
        const auto old = graph_snapshot_.find(entry.first);
        if (old == graph_snapshot_.end() || old->second != entry.second) affected.insert(entry.first);
      }
      std::map<int,std::vector<int>> reverse;
      for (const auto* snapshot : {&graph_snapshot_, &graph})
        for (const auto& entry : *snapshot)
          for (int to : entry.second) reverse[to].push_back(entry.first);
      std::vector<std::pair<int,unsigned>> queue;
      for (int id : affected) queue.emplace_back(id, 0);
      for (size_t i=0; i<queue.size(); ++i) {
        const auto [id,depth] = queue[i];
        if (depth >= _predictionLC.size()-2U) continue;
        for (int from : reverse[id]) if (affected.insert(from).second) queue.emplace_back(from,depth+1);
      }
      for (int id : affected) reusable.erase(id);
    }
    invalidate_prediction();
    _neighborsIndex = std::move(reusable);
    excluded_snapshot_ = excluded;
  }
  graph_snapshot_ = std::move(graph);
  updateColumns(excluded, ids);
}
void BayesFilter::updateColumns(const std::set<int>& excluded, const std::vector<int>& ids, const Topograph* relations) {
  if (_predictionIds == ids && !_fullPredictionUpdate) {
    return;
  }

  ++_predictionGeneration;
  EXT_ASSERT(_predictionGeneration != 0U);
  const bool virtualPlaceUsed = ids.front() < 0;
  std::unordered_map<int, int> newIdToIndex;
  newIdToIndex.reserve(ids.size());
  for (std::size_t i = 0U; i < ids.size(); ++i) {
    EXT_ASSERT(i == 0U || ids[i - 1U] < ids[i]);
    if (ids[i] > 0) {
      newIdToIndex.emplace(ids[i], static_cast<int>(i));
    }
  }

  const std::vector<int> oldIds = _predictionIds;
  const std::vector<std::uint64_t> oldGenerations = _predictionRowGenerations;
  std::vector<PredictionColumn> oldColumns = std::move(_predictionColumns);
  std::unordered_map<int, int> oldIdToIndex;
  oldIdToIndex.reserve(oldIds.size());
  for (std::size_t i = 0U; i < oldIds.size(); ++i) {
    oldIdToIndex.emplace(oldIds[i], static_cast<int>(i));
  }

  _predictionIds = ids;
  _predictionRowGenerations.resize(ids.size());
  _predictionColumns.resize(ids.size());
  std::set<int> idsToUpdate;
  if (_fullPredictionUpdate) {
    _neighborsIndex.clear();
    for (const int id : ids) {
      if (id > 0) {
        idsToUpdate.insert(id);
      }
    }
  }
  bool removed = false;
  for (const int oldId : oldIds) {
    if (oldId > 0 && newIdToIndex.find(oldId) == newIdToIndex.end()) {
      _neighborsIndex.erase(oldId);
      removed = true;
    }
  }
  if (removed) {
    // Cached graph depths can change when an intermediate place is contracted.
    // Rebuilding columns with the old neighbor maps is not a topology rebuild.
    _neighborsIndex.clear();
    for (const int id : ids) {
      if (id > 0 && oldIdToIndex.find(id) != oldIdToIndex.end()) {
        idsToUpdate.insert(id);
      }
    }
  }

  for (std::size_t i = 0U; i < ids.size(); ++i) {
    const auto old = oldIdToIndex.find(ids[i]);
    _predictionRowGenerations[i] = old == oldIdToIndex.end() ? _predictionGeneration : oldGenerations[old->second];
    if (ids[i] <= 0 || old != oldIdToIndex.end()) {
      continue;
    }
    const auto saved = _neighborsIndex.find(ids[i]);
    std::map<int, int> neighbors = saved == _neighborsIndex.end() ? graph_distances(ids[i], relations) : saved->second;
    for (const auto &neighbor : neighbors) {
      auto cached = _neighborsIndex.find(neighbor.first);
      if (!oldIds.empty() && cached != _neighborsIndex.end()) {
        cached->second.insert_or_assign(ids[i], neighbor.second);
      }
      if (oldIdToIndex.find(neighbor.first) != oldIdToIndex.end()) {
        idsToUpdate.insert(neighbor.first);
      }
    }
    _neighborsIndex.insert_or_assign(ids[i], std::move(neighbors));
  }

  const auto virtualColumn = [this](std::size_t count) {
    PredictionColumn column;
    column.backgroundGeneration = _predictionGeneration;
    if (count == 1U) {
      column.toVirtual = 1.0f;
    } else if (_virtualPlacePrior > 0.0f) {
      column.toVirtual = _virtualPlacePrior;
      column.background = (1.0f - _virtualPlacePrior) / static_cast<float>(count - 1U);
    } else {
      column.toVirtual = 1.0f / static_cast<float>(count);
      column.background = column.toVirtual;
    }
    return column;
  };

  for (std::size_t i = 0U; i < ids.size(); ++i) {
    if (ids[i] <= 0) {
      _predictionColumns[i] = virtualColumn(ids.size());
      continue;
    }
    const auto old = oldIdToIndex.find(ids[i]);
    const bool rebuild = _fullPredictionUpdate || old == oldIdToIndex.end() || idsToUpdate.find(ids[i]) != idsToUpdate.end();
    if (rebuild) {
      auto cached = _neighborsIndex.find(ids[i]);
      if (cached == _neighborsIndex.end()) {
        _neighborsIndex.insert_or_assign(ids[i], graph_distances(ids[i], relations));
        cached = _neighborsIndex.find(ids[i]);
      }
      _predictionColumns[i] = buildPredictionColumn(static_cast<unsigned int>(i), cached->second, excluded, newIdToIndex, ids.size(), virtualPlaceUsed);
      continue;
    }

    PredictionColumn copied = std::move(oldColumns[old->second]);
    copied.toVirtual = virtualPlaceUsed ? static_cast<float>(_predictionLC[0]) : 0.0f;
    std::vector<std::pair<std::uint32_t, float>> remapped;
    remapped.reserve(copied.overrides.size());
    for (const auto &entry : copied.overrides) {
      EXT_ASSERT(entry.first < oldIds.size());
      const auto row = newIdToIndex.find(oldIds[entry.first]);
      if (row != newIdToIndex.end()) {
        remapped.emplace_back(static_cast<std::uint32_t>(row->second), entry.second);
      }
    }
    copied.overrides = std::move(remapped);
    _predictionColumns[i] = std::move(copied);
  }
  if (removed || _fullPredictionUpdate) {
    _predictionGeneration = 1U;
    std::fill(_predictionRowGenerations.begin(), _predictionRowGenerations.end(), _predictionGeneration);
    for (PredictionColumn &column : _predictionColumns) {
      column.backgroundGeneration = _predictionGeneration;
    }
  }
  EXT_LOG_INFO("[bayes_prediction] columns={} bfs={} cached={}", ids.size(), neighbor_computations_, _neighborsIndex.size());
}

std::map<int,int> BayesFilter::graph_distances(int origin, const Topograph* relations) const {
  ++neighbor_computations_;
  if (relations) return relations->place_distances(origin, _predictionLC.size()-2U);
  std::map<int,int> distance;
  if (!graph_snapshot_.count(origin)) return distance;
  std::vector<int> queue{origin}; distance.emplace(origin,0);
  for (size_t i=0; i<queue.size(); ++i) {
    const int id=queue[i], depth=distance.at(id);
    if (unsigned(depth)>=_predictionLC.size()-2U) continue;
    for (int next : graph_snapshot_.at(id))
      if (distance.emplace(next,depth+1).second) queue.push_back(next);
  }
  return distance;
}

std::vector<int> BayesFilter::rank_next_places(size_t limit) const {
  if (!lookahead_ready_ || !limit || _predictionColumns.empty()) return {};
  std::vector<double> prior(_predictionIds.size(),0), background(_predictionGeneration+2,0);
  for (size_t col=0; col<_predictionColumns.size(); ++col) {
    const auto p=_posterior.find(_predictionIds[col]);
    if (p==_posterior.end()) continue;
    const auto& c=_predictionColumns[col];
    const double mass=p->second;
    background[0]+=mass*c.background;
    background[c.backgroundGeneration+1]-=mass*c.background;
    for (const auto& value : c.overrides) {
      const float base=_predictionRowGenerations[value.first]<=c.backgroundGeneration ? c.background : 0;
      prior[value.first]+=mass*(value.second-base);
    }
  }
  for (size_t i=1; i<background.size(); ++i) background[i]+=background[i-1];
  std::vector<std::pair<double,int>> ranked;
  for (size_t i=0; i<prior.size(); ++i) {
    if (_predictionIds[i]<=0) continue;
    const double probability=prior[i]+background[_predictionRowGenerations[i]];
    if (probability>0) ranked.emplace_back(probability,_predictionIds[i]);
  }
  std::sort(ranked.begin(),ranked.end(),[](const auto& a,const auto& b){return a.first!=b.first ? a.first>b.first : a.second<b.second;});
  std::vector<int> ids;
  for (size_t i=0; i<std::min(limit,ranked.size()); ++i) ids.push_back(ranked[i].second);
  return ids;
}

cv::Mat BayesFilter::generatePrediction(std::map<int, std::vector<int>> adjacency, const std::set<int> &excluded, const std::vector<int> &ids) {
  updatePrediction(adjacency, excluded, ids);
  cv::Mat prediction = cv::Mat::zeros(ids.size(), ids.size(), CV_32FC1);
  const bool virtualPlaceUsed = !ids.empty() && ids.front() < 0;
  const std::size_t realBegin = virtualPlaceUsed ? 1U : 0U;
  for (std::size_t column = 0U; column < ids.size(); ++column) {
    const PredictionColumn &sparse = _predictionColumns[column];
    for (std::size_t row = realBegin; row < ids.size(); ++row) {
      if (_predictionRowGenerations[row] <= sparse.backgroundGeneration) {
        prediction.at<float>(row, column) = sparse.background;
      }
    }
    if (virtualPlaceUsed) {
      prediction.at<float>(0, column) = sparse.toVirtual;
    }
    for (const auto &entry : sparse.overrides) {
      prediction.at<float>(entry.first, column) = entry.second;
    }
  }
  return prediction;
}

unsigned long BayesFilter::getMemoryUsed() const {
  long memoryUsage = sizeof(BayesFilter);
  for (const auto& entry : graph_snapshot_) memoryUsage += sizeof(entry) + entry.second.capacity() * sizeof(int);
  memoryUsage += _posterior.size() * (sizeof(float) + sizeof(int) + sizeof(std::map<int, float>::iterator)) + sizeof(std::map<int, float>);
  memoryUsage += _predictionIds.capacity() * sizeof(int);
  memoryUsage += _predictionRowGenerations.capacity() * sizeof(std::uint64_t);
  memoryUsage += _predictionColumns.capacity() * sizeof(PredictionColumn);
  memoryUsage += _posteriorWorkspace.capacity() * sizeof(float);
  memoryUsage += _priorWorkspace.capacity() * sizeof(double);
  memoryUsage += _backgroundWorkspace.capacity() * sizeof(double);
  for (const PredictionColumn &column : _predictionColumns) {
    memoryUsage += column.overrides.capacity() * sizeof(std::pair<std::uint32_t, float>);
  }
  memoryUsage += _predictionLC.size() * sizeof(double);
  memoryUsage += _neighborsIndex.size() * (sizeof(int) + sizeof(std::map<int, int>) + sizeof(std::map<int, std::map<int, int> >::iterator)) +
                 sizeof(std::map<int, std::map<int, int> >);
  for (std::map<int, std::map<int, int> >::const_iterator iter = _neighborsIndex.begin(); iter != _neighborsIndex.end(); ++iter) {
    memoryUsage += iter->second.size() * (sizeof(int) * 2 + sizeof(std::map<int, int>::iterator)) + sizeof(std::map<int, int>);
  }
  return memoryUsage;
}

void BayesFilter::updatePosterior(const std::vector<int> &likelihoodIds) {
  EXT_LOG_DEBUG("");
  std::map<int, float> newPosterior;
  for (std::vector<int>::const_iterator i = likelihoodIds.begin(); i != likelihoodIds.end(); ++i) {
    std::map<int, float>::iterator post = _posterior.find(*i);
    if (post == _posterior.end()) {
      if (_posterior.size() == 0) {
        newPosterior.insert(std::pair<int, float>(*i, 1));
      } else {
        newPosterior.insert(std::pair<int, float>(*i, 0));
      }
    } else {
      newPosterior.insert(std::pair<int, float>((*post).first, (*post).second));
    }
  }
  _posterior = newPosterior;
}

}  // namespace sapphire::backend
