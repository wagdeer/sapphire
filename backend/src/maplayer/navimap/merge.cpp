#include <backend/maplayer/navimap.hpp>
#include <stdexcept>

namespace sapphire::backend {
std::shared_ptr<LocalMap> fuse_localmaps(const LocalMap& target, const LocalMap& source, const StateGroupf& target_from_source) {
  if (!target.sealed || target.anchor_id <= 0 || source.anchor_id <= 0 || target.anchor_id == source.anchor_id || !target_from_source.isValid() ||
      !target.observation_count || !source.observation_count || target.last_observation.sequence >= source.first_observation.sequence)
    throw std::invalid_argument("Invalid localmap visit fusion");
  target.validate_summary();
  source.validate_summary();
  auto result = std::make_shared<LocalMap>();
  result->anchor_id = target.anchor_id;
  result->first_observation = target.first_observation;
  result->observation_count = target.observation_count + source.observation_count;
  result->visit_count = target.visit_count + source.visit_count;
  result->last_observation = source.last_observation;
  result->last_observation.anchor_id = target.anchor_id;
  result->last_observation.anchor_from_body = target_from_source.compose(source.last_observation.anchor_from_body);
  auto& merged = result->navimap;
  merged.cell_size = target.navimap.cell_size;
  merged.settings = source.navimap.settings;
  merged.anchor_from_grid = target_from_source.compose(source.navimap.anchor_from_grid);
  // Borrow resident evidence; cold maps load once. Neither input is copied into
  // a temporary destination before projection into the current horizontal plane.
  if (target.navimap_resident()) merged.merge(target.navimap, StateGroupf::identity());
  else merged.merge(target.navimap_snapshot(), StateGroupf::identity());
  if (source.navimap_resident()) merged.merge(source.navimap, target_from_source);
  else merged.merge(source.navimap_snapshot(), target_from_source);
  result->navimap_enabled = source.navimap_enabled;
  // The current odometry segment defines future append() transforms. Historical
  // poses are reanchored separately by the database submap transaction.
  result->odom_from_anchor = source.odom_from_anchor.compose(target_from_source.inverse());
  result->sealed = source.sealed;
  result->revision = std::max(target.revision, source.revision) + 1;
  return result;
}

}  // namespace sapphire::backend
