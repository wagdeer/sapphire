#include "maplayer/topograph/node.hpp"

#include <algorithm>
#include <tools/logging.hpp>
#include <stdexcept>

namespace sapphire {

Node::Node()
    : _id(0),  // invalid id
      _mapId(-1),
      _stamp(0.0),
      _weight(0),
      _saved(false),
      _modified(true),
      _linksModified(true) {}

Node::Node(int id, int mapId, int weight, double stamp, const std::string& label, const SE3f& pose)
    : _id(id), _mapId(mapId), _stamp(stamp), _weight(weight), _label(label), _saved(false), _modified(true), _linksModified(true), _pose(pose) {}

Node::Node(GaussianCloudPtr cloud, double stamp, const SE3f& odom_pose)
    : Node(0, -1, 0, stamp, "", odom_pose) {
  if (!std::isfinite(stamp) || !odom_pose.isValid() || !cloud)
    throw std::invalid_argument("Node requires a finite odometry anchor, timestamp and geometry");
  set_lidar_cloud(std::move(cloud));
}

void Node::assignIdentity(int id, int map_id) {
  if (id <= 0 || map_id < 0 || _mapId != -1 || _saved || !_links.empty())
    throw std::invalid_argument("Only a detached node can receive a map identity");
  _id = id;
  _mapId = map_id;
}

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

bool Node::isBadNode() const { return !has_lidar(); }

unsigned long Node::getMemoryUsed() const  // Return memory usage in Bytes
{
  unsigned long total = sizeof(Node) + (lidar_cloud_ ? lidar_cloud_->capacity() * sizeof(GaussianPoint) : 0);

  total += sizeof(_pose);
  total += _velocity.size() * sizeof(float);
  total += _links.size() * (sizeof(int) + sizeof(Link) + sizeof(std::multimap<int, Link>::iterator)) + sizeof(std::multimap<int, Link>);
  return total;
}

}  // namespace sapphire
