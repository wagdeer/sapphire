#pragma once

#include <cmath>
#include <cstdint>
#include <backend/common/state_group.hpp>
#include <stdexcept>
#include <string>

namespace sapphire::backend {

struct RootFrame {
  std::string id;
  int anchor = 0;
  int64_t creation_order = 0;
  StateGroupf frame_from_anchor;

  void validate() const {
    if (anchor <= 0 || creation_order <= 0 || id.size() != 35 || id.substr(0, 3) != "mf_" ||
        id.substr(3).find_first_not_of("0123456789abcdef") != std::string::npos || frame_from_anchor.isNull() || !frame_from_anchor.matrix().allFinite())
      throw std::runtime_error("Invalid persistent component frame");
    const auto r = frame_from_anchor.rotationMatrix();
    if ((r.transpose() * r - Eigen::Matrix3f::Identity()).norm() > 1e-3f || std::abs(r.determinant() - 1.f) > 1e-3f)
      throw std::runtime_error("Invalid component frame rotation");
  }
};

}  // namespace sapphire::backend
