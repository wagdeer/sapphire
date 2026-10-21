#pragma once
#include <backend/maplayer/featmap/landmark_map.hpp>

#include "distance.hpp"
namespace sapphire::backend::feature {
// Borrowed view over an immutable LandmarkMap, which owns and validates its
// appearances. This view neither stores descriptors nor manages a cache/index.
// The map must outlive the view. Check its format once; rows are branch-free.
template <class Format>
class landmark_descriptors {
 public:
  explicit landmark_descriptors(const featmap::LandmarkMap& map) : map_(map) {
    static_assert(std::is_same_v<Format, Binary_U8> || std::is_same_v<typename descriptor_traits<Format>::scalar, float>);
    if (map.descriptor_dimensions() != descriptor_traits<Format>::dimensions) throw std::invalid_argument("Map does not match compiled descriptor format");
  }
  descriptor_row<Format> row(size_t index) const noexcept { return map_.points()[index].appearance.descriptor.template row_unchecked<Format>(); }

 private:
  const featmap::LandmarkMap& map_;
};
// Draft/observation ranges cross the checked boundary once. The projector is
// statically bound and borrowed storage is never repacked for matching.
template <class Format, class Range, class Projector>
class descriptor_range {
 public:
  descriptor_range(const Range& values, Projector project) : values_(values), project_(project) {
    if (values.empty()) return;
    const auto model = project_(values.front()).model_id();
    if (project_(values.front()).dimensions() != descriptor_traits<Format>::dimensions) throw std::invalid_argument("Descriptor range format mismatch");
    for (const auto& value : values)
      if (project_(value).model_id() != model || project_(value).dimensions() != descriptor_traits<Format>::dimensions) throw std::invalid_argument("Mixed descriptor range models");
  }
  descriptor_row<Format> row(size_t index) const noexcept { return project_(values_[index]).template row_unchecked<Format>(); }

 private:
  const Range& values_;
  Projector project_;
};
template <class Format, class Range, class Projector>
auto make_descriptor_range(const Range& values, Projector project) {
  return descriptor_range<Format, Range, Projector>(values, project);
}
}  // namespace sapphire::backend::feature
