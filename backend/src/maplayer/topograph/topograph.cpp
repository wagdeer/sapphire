#include <Eigen/Cholesky>
#include <algorithm>
#include <backend/common/common_type.hpp>
#include <backend/maplayer/topograph.hpp>
#include <backend/maplayer/node.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/timer.hpp>
#include <stdexcept>
namespace sapphire::backend {
Topograph::~Topograph() {
  for (auto& [id, node] : _nodes) delete node;
}
const Node* Topograph::getNode(int id) const {
  auto it = _nodes.find(id);
  return it == _nodes.end() ? nullptr : it->second;
}
void Topograph::erase_graph_edge(Node& a, Node& b, int& last_loop) {
  auto& older = a.id() < b.id() ? a : b;
  auto& newer = a.id() < b.id() ? b : a;
  if (!older.hasLink(newer.id()) || !newer.hasLink(older.id())) return;
  if (older.getLinks().find(newer.id())->second.type() == Link::kLoopClosure && newer.getWeight() > 0) {
    older.setWeight(older.getWeight() + 1);
    newer.setWeight(newer.getWeight() - 1);
  }
  older.removeLink(newer.id());
  newer.removeLink(older.id());
  const bool has_children = std::any_of(newer.getLinks().begin(), newer.getLinks().end(), [&](const auto& edge) {
    return !edge.second.isOdometry() && edge.second.from() != edge.second.to() && edge.first < newer.id();
  });
  if (!has_children && newer.id() == last_loop) last_loop = 0;
}
void Topograph::insert_graph_edge(Node& from, Node& to, const Link& edge, bool reduce, int& last_loop) {
  if (from.hasLink(to.id())) return;
  from.addLink(edge);
  to.addLink(edge.inverse());
  if (edge.isLoopClosure()) {
    last_loop = std::max(edge.from(), edge.to());
    if ((reduce && edge.from() < edge.to()) || (!reduce && edge.from() > edge.to())) {
      from.setWeight(from.getWeight() + to.getWeight());
      to.setWeight(0);
    } else {
      to.setWeight(to.getWeight() + from.getWeight());
      from.setWeight(0);
    }
  }
}
std::map<int, int> Topograph::place_distances(int origin, unsigned max_hops) const {
  std::map<int, int> distance;
  if (origin <= 0 || !getNode(origin)) return distance;
  std::vector<int> queue{origin};
  distance.emplace(origin, 0);
  for (size_t i = 0; i < queue.size(); ++i) {
    const int id = queue[i], hops = distance.at(id);
    if (unsigned(hops) >= max_hops) continue;
    for (const auto& entry : getNode(id)->getLinks()) {
      const int next = entry.first;
      const bool geometric = entry.second.isOdometry() || entry.second.isLoopClosure();
      if (!geometric || next <= 0 || next == id || !getNode(next)) continue;
      if (distance.emplace(next, hops + 1).second) queue.push_back(next);
    }
  }
  return distance;
}
std::map<int, std::vector<int>> Topograph::resident_adjacency() const {
  std::map<int, std::vector<int>> graph;
  for (const auto& item : _nodes) {
    if (item.first <= 0) continue;
    auto& edges = graph[item.first];
    for (const auto& edge : item.second->getLinks()) {
      const bool geometric = edge.second.isOdometry() || edge.second.isLoopClosure();
      if (geometric && edge.first > 0 && edge.first != item.first && getNode(edge.first)) edges.push_back(edge.first);
    }
    std::sort(edges.begin(), edges.end());
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
  }
  return graph;
}
void Topograph::admit(Node* node, bool restored) {
  if (!node || _nodes.count(node->id())) throw std::invalid_argument("Duplicate resident node");
  // Existing nodes may retain directed links to a cold endpoint. Loading it
  // enables those edges; invalidate their OLD source neighborhoods as well.
  if (restored)
    for (const auto& [id, resident] : _nodes)
      if (resident->hasLink(node->id())) belief_.relations_changed(id);
  if (restored) belief_.relations_changed(node->id());
  _nodes.emplace(node->id(), node);
  ++topology_revision_;
}
void Topograph::release(int id) {
  belief_.relations_changed(id);
  remap_last_loop(id, 0);
  _nodes.erase(id);  // Memory owns transfer/destruction after detaching from the graph.
  ++topology_revision_;
}
void Topograph::apply_links(const std::vector<Link>& added, const std::vector<std::pair<int, int>>& removed) {
  if (added.empty() && removed.empty()) return;
  std::map<int, Node> drafts;
  int last_loop = _lastLoopClosureId;
  prepare_links(added, removed, drafts, last_loop);
  apply_links(drafts, last_loop);
}
void Topograph::prepare_links(const std::vector<Link>& added, const std::vector<std::pair<int, int>>& removed, std::map<int, Node>& drafts,
                              int& last_loop) const {
  last_loop = _lastLoopClosureId;
  const auto edit = [&](int id) -> Node& {
    const auto* node = getNode(id);
    if (!node) throw std::invalid_argument("Graph edit requires a resident endpoint");
    return drafts.try_emplace(id, *node).first->second;
  };
  for (const auto& edge : removed) {
    if (edge.first <= 0 || edge.second <= 0) throw std::invalid_argument("Invalid graph endpoint");
    auto& from = edit(edge.first);
    auto& to = edit(edge.second);
    erase_graph_edge(from, to, last_loop);
  }
  for (const auto& edge : added) {
    if (!edge.isValid() || edge.from() <= 0 || edge.to() <= 0) throw std::invalid_argument("Invalid accepted graph edge");
    auto& from = edit(edge.from());
    auto& to = edit(edge.to());
    insert_graph_edge(from, to, edge, _reduceGraph, last_loop);
  }
}
void Topograph::apply_links(std::map<int, Node>& drafts, int last_loop) noexcept {
  for (const auto& [id, node] : drafts) belief_.relations_changed(id);
  // Preserve resident object addresses held by the owner-thread pipeline.
  for (auto& [id, node] : drafts) std::swap(*_nodes.at(id), node);
  _lastLoopClosureId = last_loop;
  _linksChanged = true;
  ++topology_revision_;
}
void Topograph::removeLink(int oldId, int newId) {
  if ((oldId > 0 && !getNode(oldId)) || (newId > 0 && !getNode(newId)) || (oldId < 0 && newId < 0)) {
    EXT_LOG_ERROR("Cannot remove edge {}->{}: missing resident endpoint", oldId, newId);
    return;
  }
  apply_links({}, {{oldId, newId}});
}
const RootFrame* Topograph::rootFrame(const std::map<int, StateGroupf>& poses) const {
  if (!persistent_root_frames_) return nullptr;
  const RootFrame* oldest = nullptr;
  for (const auto& [anchor, frame] : root_frames_)
    if (poses.count(anchor) && (!oldest || frame.creation_order < oldest->creation_order)) oldest = &frame;
  return oldest;
}
void Topograph::validateFramePose(std::map<int, StateGroupf>& poses) const {
  if (const auto* frame = rootFrame(poses)) {
    // Numeric gauge tolerance, not observation uncertainty. Reject rather than
    // silently rebase a graph whose root was moved by an incompatible caller.
    if ((poses.at(frame->anchor).matrix() - frame->frame_from_anchor.matrix()).norm() > 2e-5f)
      throw std::runtime_error("Component datum moved; refusing a silent frame change: anchor=" + std::to_string(frame->anchor) +
                               " delta=" + std::to_string((poses.at(frame->anchor).matrix() - frame->frame_from_anchor.matrix()).norm()) +
                               " actual=" + poses.at(frame->anchor).debugString() + " expected=" + frame->frame_from_anchor.debugString());
    poses.at(frame->anchor) = frame->frame_from_anchor;
  }
}
std::string Topograph::root_frame_id() const {
  if (!active_frame_anchor_) return {};
  return root_frames_.at(active_frame_anchor_).id;
}
std::map<std::string, StateGroupf> Topograph::root_frame_aliases() const {
  std::map<std::string, StateGroupf> aliases;
  if (!active_frame_anchor_) return aliases;
  for (const auto& [anchor, frame] : root_frames_) {
    if (anchor != active_frame_anchor_ && world_poses_.count(anchor))
      aliases.emplace(frame.id, world_poses_.at(anchor).compose(frame.frame_from_anchor.inverse()));
  }
  return aliases;
}
void Topograph::select_world_component(const std::set<int>& connected) {
  std::map<int, StateGroupf> selected;
  for (int id : connected) {
    if (stored_poses_.count(id)) selected.emplace(id, stored_poses_.at(id));
    if (world_poses_.count(id)) selected.insert_or_assign(id, world_poses_.at(id));
    if (_optimizedPoses.count(id)) selected.insert_or_assign(id, _optimizedPoses.at(id));
  }

  // Component selection can expose archived anchors absent from this solver run.
  for (const auto& [id, pose] : world_poses_) changed_poses_.insert(id);
  for (const auto& [id, pose] : selected) changed_poses_.insert(id);
  world_poses_ = std::move(selected);
  if (const auto* frame = rootFrame(world_poses_)) active_frame_anchor_ = frame->anchor;
}
const std::map<int, float>& Topograph::observe(const std::set<int>& excluded, const std::vector<int>& ids, const std::vector<float>& likelihood) {
  return belief_.computePosterior(*this, excluded, ids, likelihood);
}
void Topograph::commit_contraction(int retained, int removed) noexcept {
  if (retained <= 0 || removed <= 0 || retained == removed) std::terminate();
  belief_.merge_places(retained, removed);
  if (hypothesis_.place_id == removed) hypothesis_.place_id = retained;
  if (hypothesis_.place_id == retained) {
    const auto probability = belief_.getPosterior().find(retained);
    if (probability != belief_.getPosterior().end()) hypothesis_.posterior = probability->second;
  }
  if (committed_loop_ == removed) committed_loop_ = retained;
  remap_last_loop(removed, retained);
  if (_lastRejectedLoopClosureIds.first == removed) _lastRejectedLoopClosureIds.first = retained;
  if (_lastRejectedLoopClosureIds.second == removed) _lastRejectedLoopClosureIds.second = retained;
}
std::map<int, float> Topograph::getNeighborsIdRadius(int nodeId,
                                                     float radius,  // 0 means ignore radius
                                                     const std::map<int, StateGroupf>& optimizedPoses,
                                                     int maxGraphDepth  // 0 means infinite margin
) const {
  EXT_ASSERT(maxGraphDepth >= 0);
  EXT_ASSERT(((optimizedPoses).find(nodeId) != (optimizedPoses).end()));
  EXT_ASSERT(nodeId > 0);
  std::map<int, float> ids;
  std::map<int, float> checkedIds;
  std::list<int> curentMarginList;
  std::set<int> currentMargin;
  std::set<int> nextMargin;
  nextMargin.insert(nodeId);
  int m = 0;
  StateGroupf referential = optimizedPoses.at(nodeId);
  EXT_ASSERT_MSG(!referential.isNull(), "nodeId={:d}", nodeId);
  float radiusSqrd = radius * radius;
  while ((maxGraphDepth == 0 || m < maxGraphDepth) && nextMargin.size()) {
    curentMarginList = std::list<int>(nextMargin.begin(), nextMargin.end());
    nextMargin.clear();

    for (std::list<int>::iterator jter = curentMarginList.begin(); jter != curentMarginList.end(); ++jter) {
      if (checkedIds.find(*jter) == checkedIds.end()) {
        // EXT_LOG_DEBUG("Added %d with margin %d", *jter, m);
        //  Look up in STM/WM if all ids are here, if not... load them from the database
        const Node* s = this->getNode(*jter);
        if (s) {
          const StateGroupf& t = optimizedPoses.at(*jter);
          EXT_ASSERT(!t.isNull());
          float distanceSqrd = referential.translationSquaredDistance(t);
          if (radiusSqrd == 0 || distanceSqrd < radiusSqrd) {
            ids.insert(std::pair<int, float>(*jter, distanceSqrd));
          }

          // links
          for (std::multimap<int, Link>::const_iterator iter = s->getLinks().begin(); iter != s->getLinks().end(); ++iter) {
            if (!((ids).find(iter->first) != (ids).end()) && ((optimizedPoses).find(iter->first) != (optimizedPoses).end())) {
              nextMargin.insert(iter->first);
            }
          }
        }
      }
    }
    ++m;
  }
  return ids;
}
std::set<int> Topograph::metric_component(int root, const std::set<int>& nodes,
                                          const std::function<std::multimap<int, Link>(int)>& load_links) const {
  if (!nodes.count(root)) throw std::runtime_error("Missing metric component root");
  std::map<int, std::set<int>> adjacency;
  for (int id : nodes) {
    for (const auto& [neighbor, edge] : load_links(id)) {
      if (edge.from() == edge.to() || !edge.isValid() || (neighbor > 0 && !nodes.count(neighbor))) continue;
      adjacency[id].insert(neighbor);
      adjacency[neighbor].insert(id);
    }
  }
  std::set<int> visited, component;

  std::vector<int> pending{root};
  while (!pending.empty()) {
    int id = pending.back();
    pending.pop_back();
    if (!visited.insert(id).second) continue;
    if (id > 0) component.insert(id);
    for (int next : adjacency[id])
      if (!visited.count(next)) pending.push_back(next);
  }
  return component;
}

namespace {
void add_unique(Node& node, const Link& link) {
  // Existing constraints win; do not count correlated measurements twice.
  if (!node.hasLink(link.to(), link.from() == link.to() ? std::optional<Link::Type>(link.type()) : std::nullopt)) node.addLink(link);
}
}  // namespace
void Topograph::redirect_edges(std::map<int, std::unique_ptr<Node>>& nodes, const Node& source, int retained, const StateGroupf& transform,
                               const std::multimap<int, Link>& constraints) {
  const int removed = source.id();
  auto& merged = *nodes.at(retained);
  for (auto& entry : nodes) {
    auto& node = *entry.second;
    std::vector<Link> redirected;
    const auto range = node.getLinks().equal_range(removed);
    for (auto it = range.first; it != range.second; ++it) {
      if (node.id() != retained) redirected.push_back(it->second.reanchor(retained, removed, transform));
    }
    node.removeLink(removed);
    for (const auto& edge : redirected) add_unique(node, edge);
  }
  for (const auto& entry : source.getLinks()) {
    if (entry.first == retained) continue;
    add_unique(merged, entry.second.reanchor(retained, removed, transform));
  }

  // Keep the two RAM directions consistent with that authoritative factor.
  for (const auto& entry : constraints) {
    const auto& edge = entry.second;
    if (edge.from() == edge.to() || edge.from() < 0 || edge.to() < 0 || !nodes.count(edge.from()) || !nodes.count(edge.to())) continue;
    auto& from = *nodes.at(edge.from());
    auto& to = *nodes.at(edge.to());
    from.removeLink(edge.to());
    from.addLink(edge);
    auto reverse = edge.inverse();
    to.removeLink(edge.from());
    to.addLink(reverse);
  }
}
void Topograph::start_session() {
  belief_.reset();
  _mapCorrection = StateGroupf::identity();
  _lastMappedPose = {};
  map_aligned_ = false;
  hypothesis_ = {};
  commit_loop(0, 0);
  _optimizedPoses.clear();
  _constraints.clear();
  _lastRejectedLoopClosureIds = {0, 0};
}
bool Topograph::canBeReduced(const Link& link, float maxDistance, int direction) {
  return link.isValid() && link.from() > 0 && link.to() > 0 && link.to() != link.from() && link.isLoopClosure() &&
         (maxDistance == 0.0f || link.transform().translationNorm() < maxDistance) &&
         (direction == 0 || (direction == -1 && link.to() < link.from()) || (direction == 1 && link.to() > link.from()));
}
int Topograph::reduceNode(int id, float maxDistance, int direction) {
  Node* source = mutable_node(id);
  if (!source || source->getWeight() < 0 || !source->getLabel().empty() || root_frames_.count(id)) return 0;
  // Graph-only contraction cannot transfer scene content. Those nodes must go
  // through the owner transaction that merges features, observations and maps.
  if (source->has_map_features() || source->has_observation_history() || source->navimap()) return 0;

  const auto links = source->getLinks();
  const Link* selected = nullptr;
  for (const auto& [other, edge] : links) {
    // A unary prior needs its own marginalization policy. Do not silently drop
    // it or reinterpret it as a relative measurement during node reduction.
    const auto* neighbor = getNode(other);
    if (other == id || !edge.isValid() || !neighbor || neighbor->getWeight() < 0 || (!edge.isOdometry() && !edge.isLoopClosure())) return 0;
    if (canBeReduced(edge, maxDistance, direction)) selected = &edge;
  }
  if (!selected) return 0;
  const int retained = selected->to();
  Node& target = *mutable_node(retained);
  const auto retained_from_removed = selected->transform().inverse();

  // Identifying the two nodes fixes their relative coordinate frames, just as
  // scene fusion does. Re-express each incident measurement; do not compose
  // uncertain paths and mislabel the resulting edges as fresh loop detections.
  std::vector<Link> redirected;
  for (const auto& [other, edge] : links) {
    if (other != retained && !target.hasLink(other)) redirected.push_back(edge.reanchor(retained, id, retained_from_removed));
  }

  belief_.invalidate_prediction();
  for (const auto& [other, edge] : links) mutable_node(other)->removeLink(id);
  for (const auto& edge : redirected) {
    target.addLink(edge);
    mutable_node(edge.to())->addLink(edge.inverse());
  }
  commit_contraction(retained, id);
  _linksChanged = true;
  ++topology_revision_;
  return retained;
}
std::map<int, int> Topograph::getNeighborsId(int nodeId,
                                             int maxGraphDepth,             // 0 means infinite margin
                                             int maxCheckedInDatabase,      // default -1 (no limit)
                                             bool incrementMarginOnLoop,    // default false
                                             bool ignoreLoopIds,            // default false
                                             bool ignoreIntermediateNodes,  // default false, ignored if ignoreLoopIds=true
                                             const std::set<int>& nodesSet, double* dbAccessTime, const ColdLinks& load) const {
  EXT_ASSERT(maxGraphDepth >= 0);
  if (dbAccessTime) {
    *dbAccessTime = 0;
  }
  std::map<int, int> ids;
  if (nodeId <= 0) {
    return ids;
  }
  int nbLoadedFromDb = 0;
  std::list<int> curentMarginList;
  std::set<int> currentMargin;
  std::set<int> nextMargin;
  nextMargin.insert(nodeId);
  int m = 0;
  std::set<int> ignoredIds;
  while ((maxGraphDepth == 0 || m < maxGraphDepth) && nextMargin.size()) {
    // insert more recent first (priority to be loaded first from the database below if set)
    curentMarginList = std::list<int>(nextMargin.rbegin(), nextMargin.rend());
    nextMargin.clear();

    for (std::list<int>::iterator jter = curentMarginList.begin(); jter != curentMarginList.end(); ++jter) {
      if (ids.find(*jter) == ids.end() && (nodesSet.empty() || nodesSet.find(*jter) != nodesSet.end())) {
        // EXT_LOG_DEBUG("Added %d with margin %d", *jter, m);
        //  Look up in STM/WM if all ids are here, if not... load them from the database
        const Node* s = this->getNode(*jter);
        std::multimap<int, Link> tmpLinks;
        const std::multimap<int, Link>* links = &tmpLinks;
        bool isIntermediateNode = false;
        if (s) {
          isIntermediateNode = s->getWeight() == -1;
          if (!ignoreIntermediateNodes || !isIntermediateNode) {
            int effectiveMargin = m > 0 && isIntermediateNode > 0 ? m - 1 : m;
            ids.insert(std::pair<int, int>(s->id(), effectiveMargin));
          } else {
            ignoredIds.insert(s->id());
          }
          links = &s->getLinks();

        } else if (load && (maxCheckedInDatabase == -1 || (maxCheckedInDatabase > 0 && nbLoadedFromDb < maxCheckedInDatabase))) {
          int weight = 0;
          tools::Timer timer;
          load(*jter, weight, tmpLinks);
          isIntermediateNode = weight == -1;
          if (!ignoreIntermediateNodes || !isIntermediateNode) {
            int effectiveMargin = m > 0 && isIntermediateNode > 0 ? m - 1 : m;
            ids.insert(std::pair<int, int>(*jter, effectiveMargin));
            if (!isIntermediateNode) {
              ++nbLoadedFromDb;
            }
          } else {
            ignoredIds.insert(*jter);
          }

          if (tmpLinks.empty()) {
            EXT_LOG_WARN("[graph] Node {} loaded without graph links", *jter);
          }
          if (!ignoreLoopIds) {
            for (std::multimap<int, Link>::iterator kter = tmpLinks.begin(); kter != tmpLinks.end();) {
              if (kter->second.from() == kter->second.to()) {
                // ignore self-referring links
                tmpLinks.erase(kter++);
              } else {
                ++kter;
              }
            }
          }
          if (dbAccessTime) {
            *dbAccessTime += timer.elapsed_seconds();
          }
        }

        // links
        for (std::multimap<int, Link>::const_iterator iter = links->begin(); iter != links->end(); ++iter) {
          if (!((ids).find(iter->first) != (ids).end()) && ignoredIds.find(iter->first) == ignoredIds.end()) {
            EXT_ASSERT(iter->second.type() != Link::kUndef);
            if (iter->second.type() == Link::kOdometry) {
              if (isIntermediateNode) {
                // stay on the same margin
                if (currentMargin.insert(iter->first).second) {
                  curentMarginList.push_back(iter->first);
                }
              } else {
                nextMargin.insert(iter->first);
              }
            } else if (!ignoreLoopIds && iter->second.isLoopClosure()) {
              if (incrementMarginOnLoop) {
                nextMargin.insert(iter->first);
              } else {
                if (currentMargin.insert(iter->first).second) {
                  curentMarginList.push_back(iter->first);
                }
              }
            }
          }
        }
      }
    }
    ++m;
  }
  return ids;
}
void Topograph::updateLink(const Link& link) {
  auto* fromS = mutable_node(link.from());
  auto* toS = mutable_node(link.to());
  if (!fromS || !toS) throw std::invalid_argument("Missing resident graph endpoint");

  if (fromS->hasLink(link.to()) && toS->hasLink(link.from())) {
    Link::Type oldType = fromS->getLinks().find(link.to())->second.type();

    if (oldType != link.type()) {
      belief_.relations_changed(link.from());
      belief_.relations_changed(link.to());
    }
    fromS->removeLink(link.to());
    toS->removeLink(link.from());

    fromS->addLink(link);
    toS->addLink(link.inverse());

    if (oldType != link.type() && link.from() != link.to()) ++topology_revision_;

    _linksChanged = true;
  } else {
    EXT_LOG_ERROR("fromId={:d} and toId={:d} are not linked!", link.from(), link.to());
  }
}
void Topograph::commit_solution(std::map<int, StateGroupf>& poses, std::multimap<int, Link>& constraints, const StateGroupf& correction,
                                const std::optional<Mat66_t<double>>& covariance, const StateGroupf& mapped) noexcept {
  _optimizedPoses.swap(poses);
  _constraints.swap(constraints);
  _mapCorrection = correction;
  _mapCorrectionCovariance = covariance;
  _lastMappedPose = mapped;
}

void Topograph::detach(Node& node) {
  belief_.invalidate_prediction();
  auto* s = &node;
  if (root_frames_.count(node.id())) throw std::runtime_error("Cannot delete a component datum");
  const std::multimap<int, Link>& links = s->getLinks();
  for (std::multimap<int, Link>::const_iterator iter = links.begin(); iter != links.end(); ++iter) {
    if (iter->second.from() != iter->second.to() && iter->first > 0) {
      Node* sTo = mutable_node(iter->first);
      // neighbor to s
      EXT_ASSERT_MSG(sTo != 0,
                     "A neighbor ({:d}) of the deleted location {:d} is "
                     "not found in WM/STM! Are you deleting a location "
                     "outside the WM/STM?",
                     iter->first, s->id());

      if (iter->first > s->id() && links.size() > 1 && sTo->hasLink(s->id())) {
        EXT_LOG_WARN(
            "Link {:d} of {:d} is newer, removing neighbor link "
            "may split the map!",
            iter->first, s->id());
      }

      // child
      if (iter->second.type() == Link::kLoopClosure && s->getWeight() > 0) {
        sTo->setWeight(sTo->getWeight() + s->getWeight());  // copy weight
      }

      sTo->removeLink(s->id());
    }
  }
  s->removeLinks(true);  // remove all links, but keep self referring link
  s->setWeight(-9);      // invalid
  s->setLabel("");       // reset label
}
std::map<int, int> Topograph::redirect_members(int retained, int source, bool consolidate) const {
  auto members = submap_members_;
  if (consolidate)
    for (auto& [id, owner] : members)
      if (owner == source) owner = retained;
  members[source] = retained;
  return members;
}
Node& Topograph::apply_contraction(int retained, int removed, std::map<int, std::unique_ptr<Node>>& nodes, std::map<int, int>& members,
                                   const StateGroupf& odom_from_anchor, bool saved) noexcept {
  auto* source = mutable_node(removed);
  belief_.relations_changed(removed);
  for (const auto& [id, node] : nodes) belief_.relations_changed(id);
  for (auto& [id, node] : nodes) {
    auto found = _nodes.find(id);
    if (found == _nodes.end()) continue;  // prepared cold neighbors remain cold
    node->setSaved(saved);
    if (saved) node->setModified(false);
    auto* old = found->second;
    found->second = node.release();
    delete old;
  }
  _nodes.erase(removed);
  delete source;
  odom_from_active_anchor_ = odom_from_anchor;
  submap_members_.swap(members);
  commit_contraction(retained, removed);
  _lastLoopClosureId = retained;
  _linksChanged = true;
  ++topology_revision_;
  return *_nodes.at(retained);
}
std::optional<std::pair<Link, Link>> Topograph::odometry_links(const Node& current, const Node* previous, bool same_session,
                                                               const Mat66_t<double>& covariance, bool diagonal_only) const {
  const auto* s = &current;
  Mat66_t<double> infMatrix;
  std::optional<std::pair<Link, Link>> neighbor;
  if (previous && same_session && !s->getPose().isNull() && !previous->getPose().isNull()) {
    if (diagonal_only) {
      infMatrix.setZero();
      infMatrix(0, 0) = 1.0 / covariance(0, 0);
      infMatrix(1, 1) = 1.0 / covariance(1, 1);
      infMatrix(2, 2) = 1.0 / covariance(2, 2);
      infMatrix(3, 3) = 1.0 / covariance(3, 3);
      infMatrix(4, 4) = 1.0 / covariance(4, 4);
      infMatrix(5, 5) = 1.0 / covariance(5, 5);
    } else {
      const Eigen::LLT<Mat66_t<double>> factor(covariance);
      if (!covariance.allFinite() || !covariance.isApprox(covariance.transpose(), 1e-8) || factor.info() != Eigen::Success)
        throw std::invalid_argument("Odometry covariance must be finite, symmetric and positive definite");
      infMatrix = factor.solve(Mat66_t<double>::Identity());
    }
    const auto anchor = odom_from_active_anchor_.isNull() ? previous->getPose() : odom_from_active_anchor_;
    const Link edge(previous->id(), s->id(), Link::kOdometry, anchor.between(s->getPose()), infMatrix);
    neighbor.emplace(edge, edge.inverse());
  }
  return neighbor;
}

void Topograph::connect_odometry(Node& current, Node& prior, const Mat66_t<double>& covariance, const std::pair<Link, Link>* neighbor) {
  // A new node was not reachable in the old resident graph. Only the prior
  // source gains an edge; do not scan every cached origin again for the new ID.
  belief_.relations_changed(prior.id());
  auto* node = &current;
  auto* previous = &prior;
  if (!node->getPose().isNull() && !previous->getPose().isNull()) {
    double maxAngVar = std::max({covariance(3, 3), covariance(4, 4), covariance(5, 5)});

    if (maxAngVar != 1.0 && maxAngVar > 0.1) {
      static bool warned = false;
      if (!warned) {
        EXT_LOG_WARN(
            "Very large angular variance ({:.6f}) detected! Please fix odometry "
            "covariance, otherwise poor graph optimizations are "
            "expected and wrong loop closure detections creating a lot "
            "of errors in the map could be accepted. This message is only "
            "printed once.",
            maxAngVar);
        warned = true;
      }
    }

    const auto& infMatrix = neighbor->first.infMatrix();
    if (_odomMaxInf.empty()) {
      _odomMaxInf.resize(6, 0.0);
    }
    for (int i = 0; i < 6; ++i) {
      const double& v = infMatrix(i, i);
      if (_odomMaxInf[i] < v) {
        _odomMaxInf[i] = v;
      }
    }

    previous->addLink(neighbor->first);
    node->addLink(neighbor->second);
  } else {
    previous->addLink(Link(previous->id(), node->id(), Link::kOdometry, StateGroupf()));
    node->addLink(Link(node->id(), previous->id(), Link::kOdometry, StateGroupf()));
  }
}
void Topograph::rehearse_edges(Node& older, Node& newer, bool update_id) {
  auto* oldS = &older;
  auto* newS = &newer;
  const int oldId = oldS->id(), newId = newS->id();
  belief_.relations_changed(oldId);
  belief_.relations_changed(newId);
  for (const auto& link : oldS->getLinks()) belief_.relations_changed(link.first);
  ++topology_revision_;
  // remove mutual links
  Link newToOldLink = newS->getLinks().find(oldS->id())->second;
  oldS->removeLink(newId);
  newS->removeLink(oldId);

  if (update_id) {
    // redirect neighbor links
    const std::multimap<int, Link>& links = oldS->getLinks();
    for (std::multimap<int, Link>::const_iterator iter = links.begin(); iter != links.end(); ++iter) {
      if (iter->second.from() != iter->second.to()) {
        Link link = iter->second;
        Link mergedLink = link;
        if (newToOldLink.transform().isNull()) {
          // Appearance-only ownership links have no metric coordinate frame.
          EXT_ASSERT(link.transform().isNull());
          mergedLink.setFrom(newId);
        } else {
          mergedLink = link.reanchor(newId, oldId, newToOldLink.transform());
        }
        EXT_ASSERT(mergedLink.from() == newS->id() && mergedLink.to() == link.to());

        Node* s = mutable_node(link.to());
        if (s) {
          // modify neighbor "from"
          s->removeLink(oldS->id());
          s->addLink(mergedLink.inverse());

          newS->addLink(mergedLink);
        } else {
          EXT_LOG_ERROR("Didn't find neighbor {:d} of {:d} in RAM...", link.to(), oldS->id());
        }
      }
    }
    newS->setLabel(oldS->getLabel());
    oldS->setLabel("");
    oldS->removeLinks();  // remove all links
    oldS->addLink(
        Link(oldS->id(), newS->id(), Link::kLoopClosure, StateGroupf(), Mat66_t<double>::Identity()));  // to keep track of the merged location

  } else {
    newS->addLink(Link(newS->id(), oldS->id(), Link::kLoopClosure, StateGroupf(), Mat66_t<double>::Identity()));
  }
}

void Topograph::track_observation(const Node& observation, const Mat66_t<double>& odomCovariance, bool keep_depth) {
  const auto* node = &observation;
  // Update optimizedPoses with the newly added node
  StateGroupf newPose = _mapCorrection.compose(node->getPose());

  EXT_LOG_DEBUG("[mapping] Added pose {} (odom={})", newPose.debugString().c_str(), node->getPose().debugString().c_str());
  // Update Poses and Constraints
  _optimizedPoses.insert(std::make_pair(node->id(), newPose));

  // Add the odometry neighbor link to the growing graph.
  if (node->getLinks().size() && node->getLinks().begin()->second.type() == Link::kOdometry) {
    // link should be old to new
    EXT_ASSERT_MSG(node->id() > node->getLinks().begin()->second.to(), "Only forward links should be added.");

    Link tmp = node->getLinks().begin()->second.inverse();

    // if the previous node is an intermediate node, remove it from the local graph
    if (_constraints.size() && _constraints.rbegin()->second.to() == node->getLinks().begin()->second.to()) {
      const Node* s = getNode(node->getLinks().begin()->second.to());
      EXT_ASSERT(s != 0);
      if (s->getWeight() == -1 && !keep_depth) {
        tmp = _constraints.rbegin()->second.merge(tmp);
        _optimizedPoses.erase(s->id());
        _constraints.erase(--_constraints.end());
      }
    }

    _constraints.insert(std::make_pair(tmp.from(), tmp));
  }
  if (node->getWeight() >= 0 && odomCovariance(0, 0) < 1) {
    if (!_mapCorrectionCovariance.has_value()) {
      _mapCorrectionCovariance = Mat66_t<double>::Zero();
    }
    // Rotation-only transport from the base frame into the map frame.
    const Mat66_t<double> adjoint = _lastMappedPose.rotationPart().adjoint().cast<double>();
    *_mapCorrectionCovariance += adjoint * odomCovariance * adjoint.transpose();
  }

  _lastMappedPose = newPose;  // keep in cache the latest corrected pose
}
void Topograph::remove_reduced(const std::map<int, int>& reduced) {
  if (reduced.size()) {
    for (std::map<int, int>::const_iterator iter = reduced.begin(); iter != reduced.end(); ++iter) {
      int erased = (int)_optimizedPoses.erase(iter->first);
      if (erased) {
        for (std::multimap<int, Link>::iterator jter = _constraints.begin(); jter != _constraints.end();) {
          if (jter->second.from() == iter->first || jter->second.to() == iter->first) {
            _constraints.erase(jter++);
          } else {
            ++jter;
          }
        }
      }
    }
  }
}
int Topograph::publication_component(bool use_previous, bool solver_enabled) {
  int selection = 0;
  if (persistent_root_frames_) {
    validateFramePose(_optimizedPoses);
    const auto* frame = rootFrame(_optimizedPoses);
    map_aligned_ = frame != nullptr;
    if (frame) {
      if (active_frame_anchor_ != frame->anchor) {
        for (const auto& entry : world_poses_) stored_poses_.insert_or_assign(entry.first, entry.second);
        selection = frame->anchor;
      }
      active_frame_anchor_ = frame->anchor;
    }
  }
  if (!persistent_root_frames_ && !map_aligned_ && solver_enabled) {
    for (const auto& entry : _optimizedPoses) {
      if (entry.first > 0 && (world_poses_.count(entry.first) || (use_previous && stored_poses_.count(entry.first)))) {
        map_aligned_ = true;
        break;
      }
    }
  }
  if (map_aligned_) {
    // Re-select only when joining a previously hidden component. Walking cold
    // adjacency on every frame would turn normal mapping into repeated DB reads.
    for (const auto& entry : _optimizedPoses) {
      if (!persistent_root_frames_ && use_previous && stored_poses_.count(entry.first) && !world_poses_.count(entry.first)) {
        selection = entry.first;
        break;
      }
    }
  }
  return selection;
}
void Topograph::publish_current(const std::map<int, int>& reduced, const std::list<int>& removed) {
  for (const auto& [id, retained] : reduced) changed_poses_.insert(id);
  changed_poses_.insert(removed.begin(), removed.end());
  if (map_aligned_) {
    for (const auto& entry : reduced) world_poses_.erase(entry.first);
    for (int id : removed) world_poses_.erase(id);
    for (const auto& entry : _optimizedPoses) {
      if (std::find(removed.begin(), removed.end(), entry.first) == removed.end()) world_poses_.insert_or_assign(entry.first, entry.second);
    }
    world_last_pose_ = _lastMappedPose;
  }
}
void Topograph::accept_explicit_solution(const std::map<int, StateGroupf>& poses, const std::multimap<int, Link>& constraints, int last_id,
                                         bool priors_ignored) {
  auto checked = poses;
  validateFramePose(checked);
  if (persistent_root_frames_ && !poses.empty()) {
    const auto* frame = rootFrame(poses);
    if (!frame) throw std::runtime_error("Explicit graph solution has no persistent datum");
    std::set<int> nodes;
    for (const auto& [id, pose] : poses)
      if (id > 0) nodes.insert(id);
    const auto connected = metric_component(frame->anchor, nodes, [&](int id) {
      std::multimap<int, Link> edges;
      for (const auto& [key, link] : constraints) {
        if (link.from() == id)
          edges.emplace(link.to(), link);
        else if (link.to() == id)
          edges.emplace(link.from(), link.inverse());
      }
      return edges;
    });
    for (const auto& entry : poses)
      if (entry.first > 0 && !connected.count(entry.first)) throw std::runtime_error("Cannot publish independent components in one world frame");
  }
  const auto* last = getNode(last_id);
  if (last && poses.count(last->id()) && _optimizedPoses.count(last->id())) {
    const auto correction = poses.at(last->id()).compose(_optimizedPoses.at(last->id()).inverse());
    if (!_lastMappedPose.isNull()) _lastMappedPose = correction.compose(_lastMappedPose);
    _mapCorrection = correction.compose(_mapCorrection);
  }
  for (const auto& [id, pose] : _optimizedPoses) changed_poses_.insert(id);
  for (const auto& [id, pose] : world_poses_) changed_poses_.insert(id);
  for (const auto& [id, pose] : poses) changed_poses_.insert(id);
  _optimizedPoses = poses;
  _constraints = constraints;
  // This API supplies an explicit world-frame solution, also used by offline tools.
  world_poses_ = poses;
  if (const auto* frame = rootFrame(poses)) active_frame_anchor_ = frame->anchor;
  world_last_pose_ = _lastMappedPose;
}

void Topograph::restore_frames(bool persistent, std::map<int, RootFrame> frames) {
  if (!persistent && !frames.empty())
    throw std::runtime_error("This database has persistent component frames; enable topograph.persistent_root_frames");
  persistent_root_frames_ = persistent;
  root_frames_ = std::move(frames);
}
const RootFrame& Topograph::restore_frame(const RootFrame& frame) { return root_frames_.emplace(frame.anchor, frame).first->second; }
void Topograph::restore_members(std::map<int, int> members) { submap_members_ = std::move(members); }
void Topograph::clear_residents() {
  belief_.reset();
  for (auto& entry : _nodes) delete entry.second;
  _nodes.clear();
  ++topology_revision_;
}
void Topograph::reset_storage() {
  odom_from_active_anchor_ = {};
  _lastLoopClosureId = 0;
  _linksChanged = false;
  _odomMaxInf.clear();
  submap_members_.clear();
  submap_local_poses_.clear();
  changed_poses_.clear();
}
void Topograph::remap_last_loop(int removed, int retained) {
  if (_lastLoopClosureId == removed) _lastLoopClosureId = retained;
}
void Topograph::begin_odometry(const StateGroupf& pose) { odom_from_active_anchor_ = pose; }
Node& Topograph::apply_feature_content(int retained, std::unique_ptr<Node>& replacement, std::map<int, int>& members,
                                       std::map<int, StateGroupf>* local_poses) noexcept {
  // The retained draft also carries accepted edge edits and is excluded from
  // apply_links()'s other endpoints. Invalidate its old reachable neighborhoods.
  belief_.relations_changed(retained);
  auto* old = _nodes.at(retained);
  _nodes.at(retained) = replacement.release();
  delete old;
  submap_members_.swap(members);
  if (local_poses) submap_local_poses_.swap(*local_poses);
  ++topology_revision_;
  return *_nodes.at(retained);
}
std::map<int, Node> Topograph::prepare_cold_link(const Link& link, bool update) const {
  if (!link.isValid() || link.from() <= 0 || link.to() <= 0) throw std::invalid_argument("Invalid cold graph edge");
  std::map<int, Node> drafts;
  for (const auto& edge : {link, link.inverse()}) {
    if (const auto* node = getNode(edge.from())) {
      auto& draft = drafts.try_emplace(edge.from(), *node).first->second;
      if (update) draft.removeLink(edge.to());
      draft.addLink(edge);
    }
  }
  return drafts;
}
void Topograph::commit_cold_link(std::map<int, Node>& drafts) noexcept {
  for (auto& [id, draft] : drafts) {
    belief_.relations_changed(id);
    std::swap(*_nodes.at(id), draft);
  }
  _linksChanged = true;
  ++topology_revision_;
}
void Topograph::make_intermediate(Node& node) {
  node.setWeight(-1);
  ++topology_revision_;
}
void Topograph::prepare_edge_edits(std::map<int, std::unique_ptr<Node>>& nodes, Node& source, const std::vector<Link>& added,
                                   const std::vector<std::pair<int, int>>& removed_links) const {
  const auto edit = [&](int id) -> Node& {
    if (id == source.id()) return source;
    return *nodes.at(id);
  };
  int last_loop = _lastLoopClosureId;
  for (const auto& edge : removed_links) {
    if (edge.first > 0 && edge.second > 0)
      Topograph::erase_graph_edge(edit(edge.first), edit(edge.second), last_loop);
    else
      throw std::invalid_argument("Invalid graph endpoint");
  }
  for (const auto& edge : added) Topograph::insert_graph_edge(edit(edge.from()), edit(edge.to()), edge, _reduceGraph, last_loop);
}

void Topograph::restore_solution(std::map<int, StateGroupf>& poses, std::multimap<int, Link>& constraints, const StateGroupf& last) {
  _optimizedPoses.swap(poses);
  _constraints.swap(constraints);
  _lastMappedPose = last;
  stored_poses_ = _optimizedPoses;
  world_poses_ = _optimizedPoses;
  active_frame_anchor_ = 0;
  map_aligned_ = false;
}
void Topograph::activate_restored_world(bool use_previous) {
  if (use_previous && !stored_poses_.empty())
    _optimizedPoses = world_poses_;
  else {
    world_poses_.clear();
    clear_projection();
  }
  world_last_pose_ = _lastMappedPose;
}
void Topograph::clear_projection() {
  _optimizedPoses.clear();
  _constraints.clear();
}
void Topograph::reset_projection() {
  clear_projection();
  _mapCorrection = StateGroupf::identity();
  map_aligned_ = false;
}
void Topograph::clear_publication() {
  clear_projection();
  world_poses_.clear();
  stored_poses_.clear();
  world_last_pose_ = {};
  _lastMappedPose = {};
  map_aligned_ = false;
}
void Topograph::end_observations() {
  hypothesis_ = {};
  commit_loop(0, 0);
  _constraints.clear();
  _mapCorrection = StateGroupf::identity();
  _mapCorrectionCovariance.reset();
}
void Topograph::begin_mapping(bool new_session) {
  if (new_session && world_poses_.empty()) map_aligned_ = true;
}
void Topograph::on_rehearsal(int removed) {
  _optimizedPoses.erase(removed);
  changed_poses_.insert(removed);
}
void Topograph::remove_reduced_world(const std::map<int, int>& reduced) {
  for (const auto& entry : reduced) {
    world_poses_.erase(entry.first);
    changed_poses_.insert(entry.first);
  }
}
void Topograph::prepare_archive(const std::set<int>& surviving, bool no_current_odometry) {
  if (map_aligned_ || no_current_odometry) {
    for (const auto& entry : _optimizedPoses) world_poses_.insert_or_assign(entry.first, entry.second);
    if (!_lastMappedPose.isNull()) world_last_pose_ = _lastMappedPose;
  }
  for (auto it = world_poses_.begin(); it != world_poses_.end();)
    if (it->first > 0 && !surviving.count(it->first))
      it = world_poses_.erase(it);
    else
      ++it;
}
void Topograph::finish_archive(const std::set<int>& surviving, const std::map<int, int>& reduced) {
  for (const auto& entry : reduced) {
    _optimizedPoses.erase(entry.first);
    world_poses_.erase(entry.first);
    stored_poses_.erase(entry.first);
  }
  for (const auto& entry : world_poses_) stored_poses_.insert_or_assign(entry.first, entry.second);
  for (auto it = stored_poses_.begin(); it != stored_poses_.end();)
    if (it->first > 0 && !surviving.count(it->first))
      it = stored_poses_.erase(it);
    else
      ++it;
}
void Topograph::remove_last_projection(int lastId) {
  EXT_LOG_DEBUG("[mapping] Detected that only last node has been removed (lastId={:d})", lastId);
  _optimizedPoses.erase(lastId);
  changed_poses_.insert(lastId);
  for (std::multimap<int, Link>::iterator iter = _constraints.find(lastId); iter != _constraints.end() && iter->first == lastId; ++iter) {
    if (iter->second.to() != iter->second.from()) {
      std::multimap<int, Link>::iterator jter = graph::findLink(_constraints, iter->second.to(), iter->second.from(), false);
      if (jter != _constraints.end()) {
        _constraints.erase(jter);
      }
    }
  }
  _constraints.erase(lastId);
}
void Topograph::retain_component(const std::map<int, int>& ids) {
  for (std::map<int, StateGroupf>::iterator iter = _optimizedPoses.begin(); iter != _optimizedPoses.end();) {
    if (iter->first > 0 && !((ids).find(iter->first) != (ids).end())) {
      EXT_LOG_DEBUG("[mapping] Removed {:d} from local map", iter->first);
      changed_poses_.insert(iter->first);
      _optimizedPoses.erase(iter++);
    } else {
      ++iter;
    }
  }
  for (std::multimap<int, Link>::iterator iter = _constraints.begin(); iter != _constraints.end();) {
    if (iter->first > 0 && (!((ids).find(iter->second.from()) != (ids).end()) || !((ids).find(iter->second.to()) != (ids).end()))) {
      _constraints.erase(iter++);
    } else {
      ++iter;
    }
  }
}
void Topograph::remember_solution(const std::map<int, StateGroupf>& poses) {
  for (const auto& entry : poses) stored_poses_.insert_or_assign(entry.first, entry.second);
}
void Topograph::apply_offline_solution(const std::map<int, StateGroupf>& poses, std::multimap<int, Link>& constraints, bool correct_odometry) {
  if (persistent_root_frames_) {
    if (const auto* frame = rootFrame(poses)) active_frame_anchor_ = frame->anchor;
  }
  for (auto& entry : _optimizedPoses) {
    const auto found = poses.find(entry.first);
    if (found != poses.end() && entry.second != found->second) {
      changed_poses_.insert(entry.first);
      entry.second = found->second;
    }
  }
  // Offline raster/publication consumers run before close(). Keep the selected
  // world component on the same accepted revision as its graph projection.
  // Other solved components stay in storage, not in this world's publication.
  for (auto& entry : world_poses_) {
    const auto found = poses.find(entry.first);
    if (found != poses.end() && entry.second != found->second) {
      changed_poses_.insert(entry.first);
      entry.second = found->second;
    }
  }
  if (correct_odometry && !_optimizedPoses.empty()) {
    const auto& last = *_optimizedPoses.rbegin();
    _mapCorrection = last.second.compose(getNode(last.first)->getPose().inverse());
  }
  _constraints.swap(constraints);
}
std::set<int> Topograph::contraction_neighbors(const Node& source, const std::vector<Link>& added,
                                               const std::vector<std::pair<int, int>>& removed_links) const {
  const int removed = source.id();
  if (root_frames_.count(removed)) throw std::runtime_error("Cannot contract a component datum");
  std::set<int> neighbors;
  for (const auto& edge : added) {
    neighbors.insert(edge.from());
    neighbors.insert(edge.to());
  }
  for (const auto& edge : removed_links) {
    if (edge.first > 0) neighbors.insert(edge.first);
    if (edge.second > 0) neighbors.insert(edge.second);
  }
  for (const auto& edge : source.getLinks())
    if (edge.first > 0) neighbors.insert(edge.first);
  for (const auto& entry : _nodes)
    if (entry.second->hasLink(removed)) neighbors.insert(entry.first);
  return neighbors;
}
void Topograph::metric_constraints(const std::set<int>& ids, std::map<int, StateGroupf>& poses, std::multimap<int, Link>& links,
                                   bool include_intermediate, bool keep_depth, const ColdLinks& load_links) const {
  for (std::set<int>::const_iterator iter = ids.begin(); iter != ids.end(); ++iter) {
    if (((poses).find(*iter) != (poses).end())) {
      const Node* s = include_intermediate ? 0 : getNode(*iter);  // If we look in db, we don't ignore intermediate nodes
      if (s && s->getWeight() == -1 && !keep_depth) {
        poses.erase(*iter);
        continue;
      }

      std::multimap<int, Link> tmpLinks;
      if (const auto* resident = getNode(*iter)) {
        tmpLinks = resident->getLinks();
      } else if (load_links) {
        int unused_weight = 0;
        load_links(*iter, unused_weight, tmpLinks);
      }
      for (std::multimap<int, Link>::iterator jter = tmpLinks.begin(); jter != tmpLinks.end(); ++jter) {
        // Capture edges are stored with their inverse at the other endpoint.
        // Read one orientation, preserving every measurement in that orientation.
        if (jter->second.isValid() && (*iter <= jter->first) && ((poses).find(jter->first) != (poses).end())) {
          if (!include_intermediate && !keep_depth && (jter->second.type() == Link::kOdometry)) {
            s = getNode(jter->first);
            EXT_ASSERT(s != 0);
            if (s->getWeight() == -1) {
              bool validLink = false;
              Link link = jter->second;
              while (s && s->getWeight() == -1) {
                // skip to next neighbor, well we assume that intermediate nodes
                // are only linked by max 2 neighbor links.
                std::multimap<int, Link> n;
                for (const auto& edge : s->getLinks())
                  if (edge.second.isOdometry()) n.insert(edge);
                EXT_ASSERT(n.size() <= 2);
                std::multimap<int, Link>::iterator uter = n.upper_bound(s->id());
                if (uter != n.end()) {
                  const Node* s2 = getNode(uter->first);
                  if (s2) {
                    link = link.merge(uter->second);
                    poses.erase(s->id());
                    s = s2;
                    validLink = s->getWeight() != -1;
                  } else {
                    validLink = false;
                    break;
                  }

                } else {
                  validLink = false;
                  break;
                }
              }
              if (validLink) {
                links.insert(std::make_pair(*iter, link));
              } else {
                poses.erase(s->id());
              }
            } else {
              links.insert(std::make_pair(*iter, jter->second));
            }
          } else {
            links.insert(std::make_pair(*iter, jter->second));
          }
        }
      }
    }
  }
}
}  // namespace sapphire::backend

namespace sapphire::backend {
void Topograph::restore_local_poses(int owner, const featmap::NodeObservations& history) {
  if (history.reference_node != owner) throw std::runtime_error("Node placement datum mismatch");
  history.validate();
  for (const auto& [capture, pose] : history.observation_poses)
    if (capture != owner && submap_owner(capture) == owner) submap_local_poses_.insert_or_assign(capture, pose);
}
int Topograph::project_submaps(int root, std::map<int, StateGroupf>& poses, std::multimap<int, Link>& links,
                               const featmap::NodeObservations* candidate) const {
  const auto placement = [&](int id) -> std::pair<int, StateGroupf> {
    if (candidate) {
      auto p = candidate->observation_poses.find(id);
      if (p != candidate->observation_poses.end()) return {candidate->reference_node, p->second};
    }
    auto p = submap_local_poses_.find(id);
    if (is_submap_member(id) && p == submap_local_poses_.end()) throw std::runtime_error("Missing committed submap capture placement");
    return p == submap_local_poses_.end() ? std::make_pair(id, StateGroupf::identity()) : std::make_pair(submap_owner(id), p->second);
  };
  const auto datum = placement(root);
  const auto fixed = poses.at(root).compose(datum.second.inverse());
  // Compute the owner seeds before erasing member captures. Owners may be cold
  // or outside the original active projection, but never gain a second variable.
  for (auto it = poses.begin(); it != poses.end(); ++it) {
    auto p = placement(it->first);
    if (p.first != it->first && !poses.count(p.first)) poses.emplace(p.first, it->second.compose(p.second.inverse()));
  }
  for (auto it = poses.begin(); it != poses.end();)
    if (placement(it->first).first != it->first)
      it = poses.erase(it);
    else
      ++it;
  poses.at(datum.first) = fixed;
  std::multimap<int, Link> projected;
  for (const auto& [id, original] : links) {
    auto a = placement(original.from()), b = placement(original.to());
    // Internal measurements are constants in the rigid approximation. They are
    // retained in the capture graph and validated there, never fed as self priors.
    if (original.from() != original.to() && a.first == b.first) continue;
    auto edge = original;
    if (a.first != original.from()) edge = edge.reanchor(a.first, original.from(), a.second);
    if (original.to() != original.from() && b.first != original.to()) edge = edge.reanchor(b.first, original.to(), b.second);
    projected.emplace(edge.from(), std::move(edge));  // Independent measurements are not endpoint-deduplicated.
  }
  links.swap(projected);
  return datum.first;
}
void Topograph::expand_submaps(std::map<int, StateGroupf>& poses, const featmap::NodeObservations* candidate) const {
  for (const auto& [capture, local] : submap_local_poses_) {
    if (candidate && candidate->observation_poses.count(capture)) continue;
    const auto owner = poses.find(submap_owner(capture));
    if (owner != poses.end()) poses.insert_or_assign(capture, owner->second.compose(local));
  }
  if (candidate) {
    auto owner = poses.find(candidate->reference_node);
    if (owner != poses.end())
      for (const auto& [capture, local] : candidate->observation_poses) poses.insert_or_assign(capture, owner->second.compose(local));
  }
}
}  // namespace sapphire::backend
