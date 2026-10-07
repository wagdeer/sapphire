#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <vector>

#include "common/aabb.hpp"

namespace boost::geometry::traits {
template <>
struct tag<sapphire::AABB> {
  using type = box_tag;
};

template <>
struct point_type<sapphire::AABB> {
  using type = model::point<float, 3, cs::cartesian>;
};

template <std::size_t Dimension>
struct indexed_access<sapphire::AABB, min_corner, Dimension> {
  static float get(const sapphire::AABB &box) { return box.minimum[Dimension]; }
  static void set(sapphire::AABB &box, float value) { box.minimum[Dimension] = value; }
};

template <std::size_t Dimension>
struct indexed_access<sapphire::AABB, max_corner, Dimension> {
  static float get(const sapphire::AABB &box) { return box.maximum[Dimension]; }
  static void set(sapphire::AABB &box, float value) { box.maximum[Dimension] = value; }
};
}  // namespace boost::geometry::traits

namespace sapphire {

class DenseAABBIndex final {
 private:
  using Tree = boost::geometry::index::rtree<std::pair<AABB, std::uint64_t>, boost::geometry::index::quadratic<16>>;

 public:
  void upsert(std::uint64_t id, const AABB &bounds, bool active = true) {
    if (!bounds.valid()) {
      throw std::invalid_argument("Spatial index requires finite bounds");
    }
    const std::pair<AABB, std::uint64_t> value{bounds, id};
    if (id < values_.size()) {
      if (active_[id]) tree_.remove(values_[id]);
      values_[id] = value;
      active_[id] = false;
      if (active) { tree_.insert(value); active_[id] = true; }
      return;
    }
    if (id != values_.size()) {
      throw std::invalid_argument("Dense spatial index requires contiguous IDs");
    }
    if (values_.size() == values_.capacity()) values_.reserve(values_.empty() ? 1 : values_.size() * 2);
    if (active_.size() == active_.capacity()) active_.reserve(active_.empty() ? 1 : active_.size() * 2);
    if (active) tree_.insert(value);
    values_.push_back(value);
    active_.push_back(active);
  }

  void setActive(std::uint64_t id, bool active) {
    if (!contains(id)) throw std::out_of_range("Unknown spatial identity");
    if (active_[id] == active) return;
    if (active) tree_.insert(values_[id]);
    else tree_.remove(values_[id]);
    active_[id] = active;
  }

  bool active(std::uint64_t id) const noexcept { return id < active_.size() && active_[id]; }
  std::size_t activeSize() const noexcept { return tree_.size(); }

  template <typename Visitor>
  void visitIntersections(const AABB &bounds, Visitor &&visitor) const {
    if (!bounds.valid()) {
      return;
    }
    std::vector<std::pair<AABB, std::uint64_t>> values;
    tree_.query(boost::geometry::index::intersects(bounds), std::back_inserter(values));
    for (const std::pair<AABB, std::uint64_t> &value : values) {
      visitor(value.second, value.first);
    }
  }

  bool contains(std::uint64_t id) const noexcept { return id < values_.size(); }

  std::size_t size() const noexcept { return values_.size(); }

 private:
  Tree tree_;
  std::vector<std::pair<AABB, std::uint64_t>> values_;
  std::vector<bool> active_;
};

}  // namespace sapphire
