#pragma once

#include <backend/common/state_group.hpp>

namespace sapphire::backend {

struct OdomObservation {
  int anchor_id = 0;
  int sequence = 0;
  double stamp = 0;
  // Body pose in the anchor frame; transformed when the anchor changes.
  StateGroupf anchor_from_body;
  // Original input odometry pose; never rewritten by map correction.
  StateGroupf odom_from_body;
};

}  // namespace sapphire::backend
