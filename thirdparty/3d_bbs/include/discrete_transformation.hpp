#pragma once
#include <cstddef>
#include <vector>

template <typename T>
class DiscreteTransformation {
 public:
  DiscreteTransformation() = default;

  DiscreteTransformation(std::size_t target_index, int level, int x, int y, int z, int yaw)
      : target_index(target_index), level(level), x(x), y(y), z(z), yaw(yaw) {}

  bool operator<(const DiscreteTransformation& rhs) const {
    if (score != rhs.score) {
      return score < rhs.score;
    }
    if (level != rhs.level) {
      return level > rhs.level;
    }
    if (target_index != rhs.target_index) {
      return target_index > rhs.target_index;
    }
    if (x != rhs.x) {
      return x > rhs.x;
    }
    if (y != rhs.y) {
      return y > rhs.y;
    }
    if (z != rhs.z) {
      return z > rhs.z;
    }
    return yaw > rhs.yaw;
  }

  bool is_leaf() const { return level == 0; }

  void branch(std::vector<DiscreteTransformation>& children, int child_level, int yaw_divisions) const {
    children.clear();
    children.reserve(static_cast<std::size_t>(8 * yaw_divisions));
    for (int dx = 0; dx < 2; ++dx) {
      for (int dy = 0; dy < 2; ++dy) {
        for (int dz = 0; dz < 2; ++dz) {
          for (int dyaw = 0; dyaw < yaw_divisions; ++dyaw) {
            children.emplace_back(target_index, child_level, x * 2 + dx, y * 2 + dy, z * 2 + dz,
                                  yaw * yaw_divisions + dyaw);
          }
        }
      }
    }
  }

  int score = 0;
  std::size_t target_index = 0;
  int level = 0;
  int x = 0;
  int y = 0;
  int z = 0;
  int yaw = 0;
};
