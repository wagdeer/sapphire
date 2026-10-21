#include "backend/maplayer/topograph.hpp"

#include <algorithm>
#include <backend/tools/associative.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/timer.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <set>
#include <unordered_map>

#include "nanoflann/nanoflann.hpp"

namespace sapphire::backend {
namespace {

class PoseMapNanoflannAdapter {
 public:
  using Entry = std::map<int, StateGroupf>::value_type;

  explicit PoseMapNanoflannAdapter(const std::map<int, StateGroupf>& poses, const int* excludedId = nullptr) {
    entries_.reserve(poses.size() - (excludedId && poses.find(*excludedId) != poses.end() ? 1U : 0U));
    for (const Entry& entry : poses) {
      if (!excludedId || entry.first != *excludedId) {
        EXT_ASSERT_MSG(entry.second.translationVector().allFinite(), "Invalid pose ({}) {}", entry.first, entry.second.debugString().c_str());
        entries_.push_back(&entry);
      }
    }
  }

  std::size_t kdtree_get_point_count() const { return entries_.size(); }

  float kdtree_get_pt(std::size_t index, std::size_t dimension) const {
    const StateGroupf& pose = entries_[index]->second;
    return dimension == 0U ? pose.x() : (dimension == 1U ? pose.y() : pose.z());
  }

  template <class BoundingBox>
  bool kdtree_get_bbox(BoundingBox&) const {
    return false;
  }

  const Entry& entry(std::size_t index) const { return *entries_[index]; }

 private:
  std::vector<const Entry*> entries_;
};

using PoseMapDistance = nanoflann::L2_Simple_Adaptor<float, PoseMapNanoflannAdapter>;
using PoseMapIndex = nanoflann::KDTreeSingleIndexAdaptor<PoseMapDistance, PoseMapNanoflannAdapter, 3, std::uint32_t>;

std::map<int, float> findNearestNodesImpl(const StateGroupf& targetPose, const std::map<int, StateGroupf>& poses, const int* excludedId, float radius,
                                          float angle, int k) {
  EXT_ASSERT(radius >= 0.0f);
  EXT_ASSERT(k >= 0);
  EXT_ASSERT(radius > 0.0f || k > 0);
  EXT_ASSERT(targetPose.translationVector().allFinite());
  std::map<int, float> foundNodes;
  const PoseMapNanoflannAdapter adapter(poses, excludedId);
  if (adapter.kdtree_get_point_count() == 0U) {
    return foundNodes;
  }

  const PoseMapIndex index(3, adapter, nanoflann::KDTreeSingleIndexAdaptorParams(15));
  const float query[3] = {targetPose.x(), targetPose.y(), targetPose.z()};
  std::vector<std::uint32_t> indices;
  std::vector<float> squaredDistances;

  if (radius > 0.0f) {
    std::vector<nanoflann::ResultItem<std::uint32_t, float>> matches;
    (void)index.radiusSearch(query, radius * radius, matches, nanoflann::SearchParameters());
    if (k > 0 && matches.size() > static_cast<std::size_t>(k)) {
      matches.resize(static_cast<std::size_t>(k));
    }
    indices.reserve(matches.size());
    squaredDistances.reserve(matches.size());
    for (const auto& match : matches) {
      indices.push_back(match.first);
      squaredDistances.push_back(match.second);
    }
  } else {
    const std::size_t count = std::min<std::size_t>(static_cast<std::size_t>(k), adapter.kdtree_get_point_count());
    indices.resize(count);
    squaredDistances.resize(count);
    const std::size_t found = index.knnSearch(query, count, indices.data(), squaredDistances.data());
    indices.resize(found);
    squaredDistances.resize(found);
  }

  const Eigen::Vector3f targetForward = targetPose.rotateVector(Eigen::Vector3f::UnitX());
  const float minimumOrientationDot = angle > 0.0f ? std::cos(angle) : -1.0f;
  for (std::size_t i = 0; i < indices.size(); ++i) {
    const PoseMapNanoflannAdapter::Entry& entry = adapter.entry(indices[i]);
    if (angle > 0.0f) {
      const Eigen::Vector3f candidateForward = entry.second.rotateVector(Eigen::Vector3f::UnitX());
      if (targetForward.dot(candidateForward) < minimumOrientationDot) {
        continue;
      }
    }
    foundNodes.emplace(entry.first, squaredDistances[i]);
  }
  return foundNodes;
}

}  // namespace

namespace graph {

MaxGraphErrors computeMaxGraphErrors(const std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& links) {
  MaxGraphErrors maxError;

  EXT_LOG_DEBUG("poses={} links={}", (int)poses.size(), (int)links.size());
  for (std::multimap<int, Link>::const_iterator iter = links.begin(); iter != links.end(); ++iter) {
    // ignore priors
    if (iter->second.from() != iter->second.to()) {
      StateGroupf t1 = sapphire::backend::tools::value_or(poses, iter->second.from(), StateGroupf());
      StateGroupf t2 = sapphire::backend::tools::value_or(poses, iter->second.to(), StateGroupf());

      if (!t1.isValid() || !t2.isValid()) {
        EXT_LOG_WARN("Poses are null or not invertible, aborting optimized graph max error check! (Pose {}={} Pose {}={})", iter->second.from(),
                     t1.debugString().c_str(), iter->second.to(), t2.debugString().c_str());

        return MaxGraphErrors();
      }

      const StateGroupf t = t1.between(t2);
      const StateGroupf linkT = iter->second.transform();

      float linearError = std::max({fabs(linkT.x() - t.x()), fabs(linkT.y() - t.y()), fabs(linkT.z() - t.z())});
      EXT_ASSERT(iter->second.transVariance(false) > 0.0);
      float stddevLinear = sqrt(iter->second.transVariance(false));
      float linearErrorRatio = linearError / stddevLinear;
      if (linearErrorRatio > maxError.linearRatio) {
        maxError.linear = linearError;
        maxError.linearRatio = linearErrorRatio;
        maxError.linearLink = iter->second;
      }

      {
        float angularError = t.rotationDistance(linkT);

        EXT_ASSERT(iter->second.rotVariance(false) > 0.0);
        float stddevAngular = sqrt(iter->second.rotVariance(false));
        float angularErrorRatio = angularError / stddevAngular;
        if (angularErrorRatio > maxError.angularRatio) {
          maxError.angular = angularError;
          maxError.angularRatio = angularErrorRatio;
          maxError.angularLink = iter->second;
        }
      }
    }
  }
  return maxError;
}

std::vector<double> getMaxOdomInf(const std::multimap<int, Link>& links) {
  std::vector<double> maxOdomInf(6, 0.0);
  maxOdomInf.resize(6, 0.0);
  for (std::multimap<int, Link>::const_iterator iter = links.begin(); iter != links.end(); ++iter) {
    if (iter->second.type() == Link::kOdometry) {
      for (int i = 0; i < 6; ++i) {
        const double& v = iter->second.infMatrix()(i, i);
        if (maxOdomInf[i] < v) {
          maxOdomInf[i] = v;
        }
      }
    }
  }
  if (maxOdomInf[0] == 0.0) {
    maxOdomInf.clear();
  }
  return maxOdomInf;
}

////////////////////////////////////////////
// Graph utilities
////////////////////////////////////////////
std::multimap<int, Link>::iterator findLink(std::multimap<int, Link>& links, int from, int to, bool checkBothWays, std::optional<Link::Type> type) {
  std::multimap<int, Link>::iterator iter = links.find(from);
  while (iter != links.end() && iter->first == from) {
    if (iter->second.to() == to && (!type || *type == iter->second.type())) {
      return iter;
    }
    ++iter;
  }

  if (checkBothWays) {
    // let's try to -> from
    iter = links.find(to);
    while (iter != links.end() && iter->first == to) {
      if (iter->second.to() == from && (!type || *type == iter->second.type())) {
        return iter;
      }
      ++iter;
    }
  }
  return links.end();
}

std::multimap<int, std::pair<int, Link::Type>>::iterator findLink(std::multimap<int, std::pair<int, Link::Type>>& links, int from, int to,
                                                                  bool checkBothWays, std::optional<Link::Type> type) {
  std::multimap<int, std::pair<int, Link::Type>>::iterator iter = links.find(from);
  while (iter != links.end() && iter->first == from) {
    if (iter->second.first == to && (!type || *type == iter->second.second)) {
      return iter;
    }
    ++iter;
  }

  if (checkBothWays) {
    // let's try to -> from
    iter = links.find(to);
    while (iter != links.end() && iter->first == to) {
      if (iter->second.first == from && (!type || *type == iter->second.second)) {
        return iter;
      }
      ++iter;
    }
  }
  return links.end();
}

std::multimap<int, int>::iterator findLink(std::multimap<int, int>& links, int from, int to, bool checkBothWays) {
  std::multimap<int, int>::iterator iter = links.find(from);
  while (iter != links.end() && iter->first == from) {
    if (iter->second == to) {
      return iter;
    }
    ++iter;
  }

  if (checkBothWays) {
    // let's try to -> from
    iter = links.find(to);
    while (iter != links.end() && iter->first == to) {
      if (iter->second == from) {
        return iter;
      }
      ++iter;
    }
  }
  return links.end();
}
std::multimap<int, Link>::const_iterator findLink(const std::multimap<int, Link>& links, int from, int to, bool checkBothWays,
                                                  std::optional<Link::Type> type) {
  std::multimap<int, Link>::const_iterator iter = links.find(from);
  while (iter != links.end() && iter->first == from) {
    if (iter->second.to() == to && (!type || *type == iter->second.type())) {
      return iter;
    }
    ++iter;
  }

  if (checkBothWays) {
    // let's try to -> from
    iter = links.find(to);
    while (iter != links.end() && iter->first == to) {
      if (iter->second.to() == from && (!type || *type == iter->second.type())) {
        return iter;
      }
      ++iter;
    }
  }
  return links.end();
}

std::multimap<int, std::pair<int, Link::Type>>::const_iterator findLink(const std::multimap<int, std::pair<int, Link::Type>>& links, int from, int to,
                                                                        bool checkBothWays, std::optional<Link::Type> type) {
  std::multimap<int, std::pair<int, Link::Type>>::const_iterator iter = links.find(from);
  while (iter != links.end() && iter->first == from) {
    if (iter->second.first == to && (!type || *type == iter->second.second)) {
      return iter;
    }
    ++iter;
  }

  if (checkBothWays) {
    // let's try to -> from
    iter = links.find(to);
    while (iter != links.end() && iter->first == to) {
      if (iter->second.first == from && (!type || *type == iter->second.second)) {
        return iter;
      }
      ++iter;
    }
  }
  return links.end();
}

std::multimap<int, int>::const_iterator findLink(const std::multimap<int, int>& links, int from, int to, bool checkBothWays) {
  std::multimap<int, int>::const_iterator iter = links.find(from);
  while (iter != links.end() && iter->first == from) {
    if (iter->second == to) {
      return iter;
    }
    ++iter;
  }

  if (checkBothWays) {
    // let's try to -> from
    iter = links.find(to);
    while (iter != links.end() && iter->first == to) {
      if (iter->second == from) {
        return iter;
      }
      ++iter;
    }
  }
  return links.end();
}

std::list<Link> findLinks(const std::multimap<int, Link>& links, int from) {
  std::list<Link> output;
  for (std::multimap<int, Link>::const_iterator iter = links.begin(); iter != links.end(); ++iter) {
    if (iter->second.from() == from) {
      output.push_back(iter->second);
    } else if (iter->second.to() == from) {
      output.push_back(iter->second.inverse());
    }
  }
  return output;
}

std::multimap<int, Link> filterDuplicateLinks(const std::multimap<int, Link>& links) {
  std::multimap<int, Link> output;
  std::set<const Link*> paired;
  for (const auto& [id, edge] : links) {
    bool inverse_copy = false;
    if (edge.from() != edge.to()) {
      const auto reverse = edge.inverse();
      const auto range = output.equal_range(edge.to());
      for (auto it = range.first; it != range.second; ++it) {
        const auto& old = it->second;
        if (paired.count(&old) || old.to() != edge.from() || old.type() != edge.type()) continue;
        const bool same_transform = old.transform().isNull()
                                        ? reverse.transform().isNull()
                                        : !reverse.transform().isNull() && old.transform().matrix().isApprox(reverse.transform().matrix(), 1e-5f);
        if (same_transform &&
            (old.infMatrix() - reverse.infMatrix()).cwiseAbs().maxCoeff() <= 1e-5 * std::max(1.0, old.infMatrix().cwiseAbs().maxCoeff())) {
          paired.insert(&old);
          inverse_copy = true;
          break;
        }
      }
    }
    if (!inverse_copy) output.emplace(edge.from(), edge);
  }
  return output;
}

std::map<int, StateGroupf> radiusPosesFiltering(const std::map<int, StateGroupf>& poses, float radius, float angle, bool keepLatest) {
  if (poses.size() > 2 && radius > 0.0f) {
    const PoseMapNanoflannAdapter adapter(poses);
    const PoseMapIndex index(3, adapter, nanoflann::KDTreeSingleIndexAdaptorParams(15));
    std::set<std::uint32_t> indicesChecked;
    std::set<std::uint32_t> indicesKept;
    std::vector<nanoflann::ResultItem<std::uint32_t, float>> neighbors;

    for (std::uint32_t i = 0; i < adapter.kdtree_get_point_count(); ++i) {
      if (indicesChecked.find(i) == indicesChecked.end()) {
        const StateGroupf& currentPose = adapter.entry(i).second;
        const float query[3] = {currentPose.x(), currentPose.y(), currentPose.z()};
        neighbors.clear();
        (void)index.radiusSearch(query, radius * radius, neighbors, nanoflann::SearchParameters(0.0f, false));

        std::set<std::uint32_t> cloudIndices;
        const Eigen::Vector3f vA = currentPose.rotateVector(Eigen::Vector3f::UnitX());
        const float minimumOrientationDot = angle > 0.0f ? std::cos(angle) : -1.0f;
        for (const auto& neighbor : neighbors) {
          if (indicesChecked.find(neighbor.first) == indicesChecked.end()) {
            if (angle > 0.0f) {
              const StateGroupf& checkT = adapter.entry(neighbor.first).second;
              // same orientation?
              const Eigen::Vector3f vB = checkT.rotateVector(Eigen::Vector3f::UnitX());
              if (vA.dot(vB) >= minimumOrientationDot) {
                cloudIndices.insert(neighbor.first);
              }
            } else {
              cloudIndices.insert(neighbor.first);
            }
          }
        }

        if (keepLatest) {
          bool lastAdded = false;
          for (auto iter = cloudIndices.rbegin(); iter != cloudIndices.rend(); ++iter) {
            if (!lastAdded) {
              indicesKept.insert(*iter);
              lastAdded = true;
            }
            indicesChecked.insert(*iter);
          }
        } else {
          bool firstAdded = false;
          for (auto iter = cloudIndices.begin(); iter != cloudIndices.end(); ++iter) {
            if (!firstAdded) {
              indicesKept.insert(*iter);
              firstAdded = true;
            }
            indicesChecked.insert(*iter);
          }
        }
      }
    }

    EXT_LOG_INFO("Cloud filtered In = {}, Out = {} (radius={:.6f} angle={:.6f} keepLatest={})", adapter.kdtree_get_point_count(), indicesKept.size(),
                 radius, angle, keepLatest ? 1 : 0);

    std::map<int, StateGroupf> keptPoses;
    for (const std::uint32_t kept : indicesKept) {
      keptPoses.insert(adapter.entry(kept));
    }

    // make sure the first and last poses are still here
    keptPoses.insert(*poses.begin());
    keptPoses.insert(*poses.rbegin());

    return keptPoses;
  } else {
    return poses;
  }
}

std::multimap<int, int> radiusPosesClustering(const std::map<int, StateGroupf>& poses, float radius, float angle) {
  std::multimap<int, int> clusters;
  if (poses.size() > 1 && radius > 0.0f) {
    const PoseMapNanoflannAdapter adapter(poses);
    const PoseMapIndex index(3, adapter, nanoflann::KDTreeSingleIndexAdaptorParams(15));
    std::vector<nanoflann::ResultItem<std::uint32_t, float>> neighbors;

    for (std::uint32_t i = 0; i < adapter.kdtree_get_point_count(); ++i) {
      const auto& currentEntry = adapter.entry(i);
      const StateGroupf& currentT = currentEntry.second;
      const float query[3] = {currentT.x(), currentT.y(), currentT.z()};
      neighbors.clear();
      (void)index.radiusSearch(query, radius * radius, neighbors, nanoflann::SearchParameters(0.0f, false));

      const Eigen::Vector3f vA = currentT.rotateVector(Eigen::Vector3f::UnitX());
      const float minimumOrientationDot = angle > 0.0f ? std::cos(angle) : -1.0f;
      for (const auto& neighbor : neighbors) {
        if (i != neighbor.first) {
          const auto& candidateEntry = adapter.entry(neighbor.first);
          if (angle > 0.0f) {
            const StateGroupf& checkT = candidateEntry.second;
            // same orientation?
            const Eigen::Vector3f vB = checkT.rotateVector(Eigen::Vector3f::UnitX());
            if (vA.dot(vB) >= minimumOrientationDot) {
              clusters.emplace(currentEntry.first, candidateEntry.first);
            }
          } else {
            clusters.emplace(currentEntry.first, candidateEntry.first);
          }
        }
      }
    }
  }
  return clusters;
}

class Node {
 public:
  Node(int id, int fromId, const StateGroupf& pose) : id_(id), costSoFar_(0.0f), distToEnd_(0.0f), fromId_(fromId), closed_(false), pose_(pose) {}

  int id() const { return id_; }
  int fromId() const { return fromId_; }
  bool isClosed() const { return closed_; }
  bool isOpened() const { return !closed_; }
  float costSoFar() const { return costSoFar_; }               // Dijkstra cost
  float distToEnd() const { return distToEnd_; }               // Breath-first cost
  float totalCost() const { return costSoFar_ + distToEnd_; }  // A* cost
  StateGroupf pose() const { return pose_; }
  float distFrom(const StateGroupf& pose) const { return pose_.translationDistance(pose); }

  void setClosed(bool closed) { closed_ = closed; }
  void setFromId(int fromId) { fromId_ = fromId; }
  void setCostSoFar(float costSoFar) { costSoFar_ = costSoFar; }
  void setDistToEnd(float distToEnd) { distToEnd_ = distToEnd; }
  void setPose(const StateGroupf& pose) { pose_ = pose; }

 private:
  int id_;
  float costSoFar_;
  float distToEnd_;
  int fromId_;
  bool closed_;
  StateGroupf pose_;
};

typedef std::pair<int, float> Pair;  // first is id, second is cost
struct Order {
  bool operator()(Pair const& a, Pair const& b) const { return a.second > b.second; }
};

// A*
std::list<std::pair<int, StateGroupf>> computePath(const std::map<int, StateGroupf>& poses, const std::multimap<int, int>& links, int from, int to,
                                                   bool updateNewCosts) {
  std::list<std::pair<int, StateGroupf>> path;

  int startNode = from;
  int endNode = to;
  StateGroupf endPose = poses.at(endNode);
  // Accessed only by ID; neighbor iteration and queue ordering determine the path.
  // References remain valid across unordered_map rehashes.
  std::unordered_map<int, Node> nodes;
  nodes.insert(std::make_pair(startNode, Node(startNode, 0, poses.at(startNode))));
  std::priority_queue<Pair, std::vector<Pair>, Order> pq;
  std::multimap<float, int> pqmap;
  if (updateNewCosts) {
    pqmap.insert(std::make_pair(0, startNode));
  } else {
    pq.push(Pair(startNode, 0));
  }

  while ((updateNewCosts && pqmap.size()) || (!updateNewCosts && pq.size())) {
    Node* currentNode;
    if (updateNewCosts) {
      currentNode = &nodes.find(pqmap.begin()->second)->second;
      pqmap.erase(pqmap.begin());
    } else {
      currentNode = &nodes.find(pq.top().first)->second;
      pq.pop();
    }

    currentNode->setClosed(true);

    if (currentNode->id() == endNode) {
      while (currentNode->id() != startNode) {
        path.push_front(std::make_pair(currentNode->id(), currentNode->pose()));
        currentNode = &nodes.find(currentNode->fromId())->second;
      }
      path.push_front(std::make_pair(startNode, poses.at(startNode)));
      break;
    }

    // lookup neighbors
    for (std::multimap<int, int>::const_iterator iter = links.find(currentNode->id()); iter != links.end() && iter->first == currentNode->id();
         ++iter) {
      auto nodeIter = nodes.find(iter->second);
      if (nodeIter == nodes.end()) {
        std::map<int, StateGroupf>::const_iterator poseIter = poses.find(iter->second);
        if (poseIter == poses.end()) {
          EXT_LOG_ERROR("Next pose {} (from {}) should be found in poses! Ignoring it!", iter->second, iter->first);
        } else {
          Node n(iter->second, currentNode->id(), poseIter->second);
          n.setCostSoFar(currentNode->costSoFar() + currentNode->distFrom(poseIter->second));
          n.setDistToEnd(n.distFrom(endPose));

          nodes.insert(std::make_pair(iter->second, n));
          if (updateNewCosts) {
            pqmap.insert(std::make_pair(n.totalCost(), n.id()));
          } else {
            pq.push(Pair(n.id(), n.totalCost()));
          }
        }
      } else if (updateNewCosts && nodeIter->second.isOpened()) {
        float newCostSoFar = currentNode->costSoFar() + currentNode->distFrom(nodeIter->second.pose());
        if (nodeIter->second.costSoFar() > newCostSoFar) {
          // update the cost in the priority queue
          for (std::multimap<float, int>::iterator mapIter = pqmap.begin(); mapIter != pqmap.end(); ++mapIter) {
            if (mapIter->second == nodeIter->first) {
              pqmap.erase(mapIter);
              nodeIter->second.setFromId(currentNode->id());
              nodeIter->second.setCostSoFar(newCostSoFar);
              pqmap.insert(std::make_pair(nodeIter->second.totalCost(), nodeIter->first));
              break;
            }
          }
        }
      }
    }
  }
  return path;
}

// Dijksta
std::list<int> computePath(const std::multimap<int, Link>& links, int from, int to, bool updateNewCosts, bool useSameCostForAllLinks) {
  std::list<int> path;

  int startNode = from;
  int endNode = to;
  std::map<int, Node> nodes;
  nodes.insert(std::make_pair(startNode, Node(startNode, 0, StateGroupf())));
  std::priority_queue<Pair, std::vector<Pair>, Order> pq;
  std::multimap<float, int> pqmap;
  if (updateNewCosts) {
    pqmap.insert(std::make_pair(0, startNode));
  } else {
    pq.push(Pair(startNode, 0));
  }

  while ((updateNewCosts && pqmap.size()) || (!updateNewCosts && pq.size())) {
    Node* currentNode;
    if (updateNewCosts) {
      currentNode = &nodes.find(pqmap.begin()->second)->second;
      pqmap.erase(pqmap.begin());
    } else {
      currentNode = &nodes.find(pq.top().first)->second;
      pq.pop();
    }

    currentNode->setClosed(true);

    if (currentNode->id() == endNode) {
      while (currentNode->id() != startNode) {
        path.push_front(currentNode->id());
        currentNode = &nodes.find(currentNode->fromId())->second;
      }
      path.push_front(startNode);
      break;
    }

    // lookup neighbors
    for (std::multimap<int, Link>::const_iterator iter = links.find(currentNode->id()); iter != links.end() && iter->first == currentNode->id();
         ++iter) {
      std::map<int, Node>::iterator nodeIter = nodes.find(iter->second.to());
      float cost = 1;
      if (!useSameCostForAllLinks) {
        cost = iter->second.transform().translationNorm();
      }
      if (nodeIter == nodes.end()) {
        Node n(iter->second.to(), currentNode->id(), StateGroupf());

        n.setCostSoFar(currentNode->costSoFar() + cost);
        nodes.insert(std::make_pair(iter->second.to(), n));
        if (updateNewCosts) {
          pqmap.insert(std::make_pair(n.totalCost(), n.id()));
        } else {
          pq.push(Pair(n.id(), n.totalCost()));
        }
      } else if (updateNewCosts && nodeIter->second.isOpened()) {
        float newCostSoFar = currentNode->costSoFar() + cost;
        if (nodeIter->second.costSoFar() > newCostSoFar) {
          // update the cost in the priority queue
          for (std::multimap<float, int>::iterator mapIter = pqmap.begin(); mapIter != pqmap.end(); ++mapIter) {
            if (mapIter->second == nodeIter->first) {
              pqmap.erase(mapIter);
              nodeIter->second.setFromId(currentNode->id());
              nodeIter->second.setCostSoFar(newCostSoFar);
              pqmap.insert(std::make_pair(nodeIter->second.totalCost(), nodeIter->first));
              break;
            }
          }
        }
      }
    }
  }
  return path;
}

// return path starting from "fromId" (Identity pose for the first node)

int findNearestNode(const std::map<int, StateGroupf>& poses, const StateGroupf& targetPose, float* distance) {
  int id = 0;
  std::map<int, float> nearestNodes = findNearestNodes(targetPose, poses, 0, 0, 1);
  if (!nearestNodes.empty()) {
    id = nearestNodes.begin()->first;
    if (distance) {
      *distance = nearestNodes.begin()->second;
    }
  }
  return id;
}

// return <id, sqrd distance>, excluding query
std::map<int, float> findNearestNodes(int nodeId, const std::map<int, StateGroupf>& poses, float radius, float angle, int k) {
  EXT_ASSERT(((poses).find(nodeId) != (poses).end()));
  return findNearestNodesImpl(poses.at(nodeId), poses, &nodeId, radius, angle, k);
}

// return <id, sqrd distance>
std::map<int, float> findNearestNodes(const StateGroupf& targetPose, const std::map<int, StateGroupf>& poses, float radius, float angle, int k) {
  EXT_ASSERT(radius >= 0.0f);
  EXT_ASSERT(k >= 0);
  EXT_ASSERT(radius > 0.0f || k > 0);
  std::map<int, float> foundNodes = findNearestNodesImpl(targetPose, poses, nullptr, radius, angle, k);
  EXT_LOG_DEBUG("found nodes={}", (int)foundNodes.size());
  return foundNodes;
}

// return <id, StateGroupf>, excluding query
std::map<int, StateGroupf> findNearestPoses(int nodeId, const std::map<int, StateGroupf>& poses, float radius, float angle, int k) {
  EXT_ASSERT(((poses).find(nodeId) != (poses).end()));
  const std::map<int, float> nearestNodes = findNearestNodes(nodeId, poses, radius, angle, k);
  std::map<int, StateGroupf> foundPoses;
  for (const auto& nearest : nearestNodes) {
    foundPoses.insert(*poses.find(nearest.first));
  }
  return foundPoses;
}
// return <id, StateGroupf>
std::map<int, StateGroupf> findNearestPoses(const StateGroupf& targetPose, const std::map<int, StateGroupf>& poses, float radius, float angle,
                                            int k) {
  std::map<int, float> nearestNodes = findNearestNodes(targetPose, poses, radius, angle, k);
  std::map<int, StateGroupf> foundPoses;
  for (std::map<int, float>::iterator iter = nearestNodes.begin(); iter != nearestNodes.end(); ++iter) {
    foundPoses.insert(*poses.find(iter->first));
  }
  EXT_LOG_DEBUG("found nodes={}/{} (radius={:.6f}, angle={:.6f}, k={})", (int)foundPoses.size(), (int)poses.size(), radius, angle, k);
  return foundPoses;
}

float computePathLength(const std::vector<std::pair<int, StateGroupf>>& path) {
  float length = 0.0f;
  if (path.size() > 1) {
    for (unsigned int i = 0; i < path.size() - 1; ++i) {
      length += path[i].second.translationDistance(path[i + 1].second);
    }
  }
  return length;
}

float computePathLength(const std::map<int, StateGroupf>& path) {
  float length = 0.0f;
  if (path.size() > 1) {
    std::map<int, StateGroupf>::const_iterator iter = path.begin();
    StateGroupf previousPose = iter->second;
    ++iter;
    for (; iter != path.end(); ++iter) {
      const StateGroupf& currentPose = iter->second;
      length += previousPose.translationDistance(currentPose);
      previousPose = currentPose;
    }
  }
  return length;
}

void computeMinMax(const std::map<int, StateGroupf>& poses, cv::Vec3f& min, cv::Vec3f& max) {
  if (!poses.empty()) {
    min[0] = max[0] = poses.begin()->second.x();
    min[1] = max[1] = poses.begin()->second.y();
    min[2] = max[2] = poses.begin()->second.z();
    for (std::map<int, StateGroupf>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter) {
      if (min[0] > iter->second.x()) min[0] = iter->second.x();
      if (max[0] < iter->second.x()) max[0] = iter->second.x();
      if (min[1] > iter->second.y()) min[1] = iter->second.y();
      if (max[1] < iter->second.y()) max[1] = iter->second.y();
      if (min[2] > iter->second.z()) min[2] = iter->second.z();
      if (max[2] < iter->second.z()) max[2] = iter->second.z();
    }
  }
}

} /* namespace graph */

} /* namespace sapphire::backend */
