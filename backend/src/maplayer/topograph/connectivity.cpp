#include <backend/maplayer/topograph.hpp>
#include <backend/tools/logging.hpp>
namespace sapphire::backend::graph {
namespace {
class LinkIdKey {
 public:
  LinkIdKey(int id, Link::Type type) : id_(id), type_(type) {}
  bool operator<(const LinkIdKey& k) const {
    if (type_ == Link::kOdometry && k.type_ != Link::kOdometry) {
      return true;
    } else if (type_ != Link::kOdometry && k.type_ == Link::kOdometry) {
      return false;
    } else {
      // normal link, sort by smallest to largest id
      return id_ < k.id_;
    }
  }
  int id_;
  Link::Type type_;
};

}  // namespace
void getConnectedGraph(int fromId, const std::map<int, StateGroupf>& posesIn, const std::multimap<int, Link>& linksIn,
                       std::map<int, StateGroupf>& posesOut, std::multimap<int, Link>& linksOut, bool priors_ignored) {
  EXT_LOG_DEBUG("IN: fromId={} poses={} links={} priorsIgnored={}", fromId, (int)posesIn.size(), (int)linksIn.size(), priors_ignored ? 1 : 0);
  EXT_ASSERT(fromId > 0);
  EXT_ASSERT_MSG(((posesIn).find(fromId) != (posesIn).end()), "{}",
                 fmt::format("poses={} (first={} last={}) fromId={}", posesIn.size(), posesIn.empty() ? 0 : posesIn.begin()->first,
                             posesIn.empty() ? 0 : posesIn.rbegin()->first, fromId));

  posesOut.clear();
  linksOut.clear();

  std::map<LinkIdKey, StateGroupf> nextPoses;
  nextPoses.insert(std::make_pair(LinkIdKey(fromId, Link::kUndef), posesIn.find(fromId)->second));
  std::multimap<int, std::pair<int, Link::Type> > biLinks;
  for (std::multimap<int, Link>::const_iterator iter = linksIn.begin(); iter != linksIn.end(); ++iter) {
    if (iter->second.from() != iter->second.to()) {
      if (graph::findLink(biLinks, iter->second.from(), iter->second.to(), true, iter->second.type()) == biLinks.end()) {
        biLinks.insert(std::make_pair(iter->second.from(), std::make_pair(iter->second.to(), iter->second.type())));
        biLinks.insert(std::make_pair(iter->second.to(), std::make_pair(iter->second.from(), iter->second.type())));
      }
    }
  }

  while (!nextPoses.empty()) {
    // For nodes, fill up all neightbor nodes before loop closure ones
    int currentId = nextPoses.begin()->first.id_;
    StateGroupf currentPose = nextPoses.begin()->second;
    nextPoses.erase(nextPoses.begin());

    if (posesOut.find(currentId) != posesOut.end()) {
      // Already added from priority list
      continue;
    }

    posesOut.insert(std::make_pair(currentId, currentPose));

    // add prior links
    for (std::multimap<int, Link>::const_iterator pter = linksIn.find(currentId); pter != linksIn.end() && pter->first == currentId; ++pter) {
      if (pter->second.from() == pter->second.to() && (!priors_ignored || pter->second.type() != Link::kPosePrior)) {
        linksOut.insert(*pter);
      }
    }

    for (std::multimap<int, std::pair<int, Link::Type> >::const_iterator iter = biLinks.find(currentId);
         iter != biLinks.end() && iter->first == currentId; ++iter) {
      int toId = iter->second.first;
      Link::Type type = iter->second.second;
      if (posesIn.find(toId) != posesIn.end()) {
        std::multimap<int, Link>::const_iterator kter = graph::findLink(linksIn, currentId, toId, true, type);
        EXT_ASSERT(kter != linksIn.end());
        if (!((posesOut).find(toId) != (posesOut).end())) {
          StateGroupf t = kter->second.from() == currentId ? kter->second.transform() : kter->second.transform().inverse();
          StateGroupf pose = posesOut.at(currentId).compose(t);

          nextPoses.insert(std::make_pair(LinkIdKey(toId, type), pose));
        }

        // only add unique links
        if (graph::findLink(linksOut, currentId, toId, true, kter->second.type()) == linksOut.end()) {
          linksOut.insert(*kter);
        }
      }
    }
  }
  linksOut.clear();
  for (const auto& [id, edge] : graph::filterDuplicateLinks(linksIn)) {
    if (!posesOut.count(edge.from()) || !posesOut.count(edge.to())) continue;
    if (priors_ignored && edge.type() == Link::kPosePrior) continue;
    linksOut.emplace(edge.from(), edge);
  }
  EXT_LOG_DEBUG("OUT: poses={} links={}", (int)posesOut.size(), (int)linksOut.size());
}

}  // namespace sapphire::backend::graph
