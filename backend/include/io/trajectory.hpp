#pragma once

#include <backend/common/state_group.hpp>
#include <array>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <string_view>
#include <vector>

namespace sapphire::backend::io {

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
    StateGroupf pose;
  };
  void clear();
  void setDuration(double seconds);
  void observe(uint64_t session, double keyframe_stamp, double stamp, const StateGroupf& odom);
  std::vector<Sample> smooth() const;
  size_t size() const { return samples_.size(); }
 private:
  uint64_t session_ = 0;
  double duration_ = 5;
  std::vector<Sample> samples_;
};

}  // namespace sapphire::backend::io
