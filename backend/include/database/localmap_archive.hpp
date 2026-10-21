#pragma once
#include <backend/maplayer/navimap.hpp>
namespace sapphire::backend {
class DBDriver;
class Node;
// Archive format, cold reads and immutable snapshot persistence. The map itself
// receives geometry from its loader and does not interpret encoded bytes.
class LocalMapArchive final {
 public:
  static cv::Mat encode(const LocalMap&);
  static std::shared_ptr<LocalMap> decode(const cv::Mat&);
  static void unload_navimap(LocalMap&, const cv::Mat& archive, const std::shared_ptr<DBDriver>&);
  static void prepare_append(LocalMap&);
  static void seal(LocalMap&);
};
}  // namespace sapphire::backend
