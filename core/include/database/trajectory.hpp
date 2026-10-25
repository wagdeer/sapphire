#pragma once

#include <common/se3_state.hpp>
#include <array>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <string_view>
#include <vector>

namespace sapphire {

struct OdomObservation {
  int anchor_id = 0;
  int sequence = 0;
  double stamp = 0;
  // Body pose in the anchor frame; transformed when the anchor changes.
  SE3f anchor_from_body;
  // Original input odometry pose; never rewritten by map correction.
  SE3f odom_from_body;
};


enum class OdomFileFormat { Tum, Txt };

OdomFileFormat odometry_format(std::string_view name);

std::vector<std::string> trajectoryColumns(OdomFileFormat format);

struct OdometryRecord {
  double stamp;
  std::array<double, 12> matrix;
};

void writeOdomFile(std::ostream& output, OdomFileFormat format,
                     const std::vector<OdometryRecord>& records, const std::string& metadata = {});

// build smoothed path from odom records
class PathConstructor {
 public:
  struct Sample {
    double stamp;
    SE3f pose;
  };
  void clear();
  void setDuration(double seconds);
  void observe(uint64_t session, double keyframe_stamp, double stamp, const SE3f& odom);
  std::vector<Sample> smooth() const;
  size_t size() const { return samples_.size(); }
 private:
  uint64_t session_ = 0;
  double duration_ = 5;
  std::vector<Sample> samples_;
};

}  // namespace sapphire
