#include <backend/maplayer/topograph/visit_history.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

namespace sapphire::backend {
VisitHistory::VisitHistory(size_t capacity, double max_gap_seconds)
    : capacity_(capacity), max_gap_seconds_(max_gap_seconds) {
  if (capacity < 4 || capacity > 100000 || !std::isfinite(max_gap_seconds) || max_gap_seconds <= 0)
    throw std::invalid_argument("Invalid visit-history bounds");
}

void VisitHistory::break_sequence() {
  active_ = false;
  previous_place_ = current_place_ = 0;
}

void VisitHistory::record(int session, int query, double stamp, int place, bool verified) {
  if (session < 0 || query <= 0 || place <= 0 || !std::isfinite(stamp) || stamp <= 0) {
    break_sequence();
    return;
  }
  // A repeated callback must not create extra route support.
  if (active_ && session == session_ && query == query_ && stamp == stamp_) return;
  if (!active_ || session != session_ || stamp <= stamp_ || stamp - stamp_ > max_gap_seconds_ || verified != verified_) {
    break_sequence();
    if (segment_ == std::numeric_limits<uint64_t>::max()) throw std::overflow_error("Visit segment overflow");
    ++segment_;
  }
  active_ = true;
  session_ = session;
  query_ = query;
  stamp_ = stamp;
  verified_ = verified;
  if (verified && place != current_place_) {
    previous_place_ = current_place_;
    current_place_ = place;
  }
  // One sample per consecutive place; dwelling does not outvote other routes.
  if (visits_.empty() || visits_.back().segment != segment_ || visits_.back().place != place)
    visits_.push_back({segment_, session, query, place, stamp, verified});
  while (visits_.size() > capacity_) visits_.pop_front();
}

std::vector<int> VisitHistory::predict(int session, double stamp, const Resolve& resolve, size_t limit) const {
  if (!limit || !active_ || !verified_ || !previous_place_ || session != session_ ||
      !std::isfinite(stamp) || stamp < stamp_ || stamp - stamp_ > max_gap_seconds_) return {};
  const int previous = resolve(previous_place_), current = resolve(current_place_);
  if (previous <= 0 || current <= 0 || previous == current) return {};
  struct Step { uint64_t segment; int place; };
  std::vector<Step> route;
  route.reserve(visits_.size());
  for (const auto& visit : visits_) {
    const int place = resolve(visit.place);
    if (route.empty() || route.back().segment != visit.segment || route.back().place != place)
      route.push_back({visit.segment, place});
  }
  std::map<int, double> votes;
  // Bounded sequence matching supports reverse travel and up to two skipped places.
  constexpr int horizon = 3;
  for (int i = 0; i < static_cast<int>(route.size()); ++i) {
    if (route[i].place != current) continue;
    for (int direction : {-1, 1}) {
      bool matched = false;
      for (int hop = 1; hop <= horizon; ++hop) {
        const int j = i - direction * hop;
        if (j < 0 || j >= static_cast<int>(route.size()) || route[j].segment != route[i].segment || route[j].place <= 0) break;
        if (route[j].place == previous) { matched = true; break; }
      }
      if (!matched) continue;
      for (int hop = 1; hop <= horizon; ++hop) {
        const int j = i + direction * hop;
        if (j < 0 || j >= static_cast<int>(route.size()) || route[j].segment != route[i].segment || route[j].place <= 0) break;
        const int place = route[j].place;
        if (place != previous && place != current) votes[place] += 1.0 / hop;
      }
    }
  }
  std::vector<std::pair<int, double>> ranked(votes.begin(), votes.end());
  std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
    return a.second != b.second ? a.second > b.second : a.first < b.first;
  });
  std::vector<int> result;
  for (size_t i = 0; i < std::min(limit, ranked.size()); ++i) result.push_back(ranked[i].first);
  return result;
}

bool VisitHistory::reorder(std::vector<int>& candidates, const std::vector<int>& predicted) {
  if (candidates.size() < 3 || predicted.empty()) return false;
  const auto before = candidates;
  std::map<int, size_t> priority;
  for (size_t i = 0; i < predicted.size(); ++i) priority.emplace(predicted[i], i);
  const auto rank = [&](int id) { const auto it = priority.find(id); return it == priority.end() ? predicted.size() : it->second; };
  std::stable_sort(candidates.begin() + 1, candidates.end(), [&](int a, int b) { return rank(a) < rank(b); });
  return candidates != before;
}

std::string VisitHistory::encode() const {
  auto rows = nlohmann::json::array();
  for (const auto& v : visits_) rows.push_back({v.segment, v.session, v.query, v.place, v.stamp, v.verified});
  return nlohmann::json{{"version", 1}, {"visits", std::move(rows)}}.dump();
}

void VisitHistory::restore(const std::string& archive) {
  VisitHistory restored(capacity_, max_gap_seconds_);
  if (!archive.empty()) {
    if (archive.size() > 16 * 1024 * 1024) throw std::invalid_argument("Visit archive too large");
    const auto data = nlohmann::json::parse(archive);
    if (data.at("version") != 1 || !data.at("visits").is_array() || data.at("visits").size() > 100000)
      throw std::invalid_argument("Unsupported visit archive");
    const auto integer = [](const nlohmann::json& n, uint64_t maximum) {
      if (!n.is_number_integer() || (n.is_number_integer() && !n.is_number_unsigned() && n.get<int64_t>() < 0) ||
          n.get<uint64_t>() > maximum) throw std::invalid_argument("Invalid visit integer");
      return n.get<uint64_t>();
    };
    Visit last;
    for (const auto& row : data.at("visits")) {
      if (!row.is_array() || row.size() != 6 || !row[4].is_number() || !row[5].is_boolean())
        throw std::invalid_argument("Invalid visit row");
      Visit v{integer(row[0], std::numeric_limits<uint64_t>::max() - 1),
              int(integer(row[1], std::numeric_limits<int>::max())),
              int(integer(row[2], std::numeric_limits<int>::max())),
              int(integer(row[3], std::numeric_limits<int>::max())), row[4].get<double>(), row[5].get<bool>()};
      if (!v.segment || v.query <= 0 || v.place <= 0 || !std::isfinite(v.stamp) || v.stamp <= 0 ||
          v.segment < last.segment || (v.segment == last.segment &&
          (v.session != last.session || v.verified != last.verified || v.stamp <= last.stamp)))
        throw std::invalid_argument("Invalid visit sequence");
      restored.visits_.push_back(v);
      if (restored.visits_.size() > capacity_) restored.visits_.pop_front();
      restored.segment_ = v.segment;
      last = v;
    }
  }
  // History is restored; localization context must be established afresh.
  *this = std::move(restored);
}
}  // namespace sapphire::backend
