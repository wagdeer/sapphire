#include "sapphire_ros2/lidar_handler.hpp"
#include <cassert>
#include <iostream>
using F = sensor_msgs::msg::PointField;
using Cloud = sensor_msgs::msg::PointCloud2;
template<class T>
void store(Cloud &m, size_t i, size_t field, T value) {
  std::array<uint8_t, sizeof(T)> bytes;
  std::memcpy(bytes.data(), &value, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  if (m.is_bigendian) std::reverse(bytes.begin(), bytes.end());
#else
  if (!m.is_bigendian) std::reverse(bytes.begin(), bytes.end());
#endif
  std::memcpy(m.data.data() + (i / m.width) * m.row_step + (i % m.width) * m.point_step + field, bytes.data(), sizeof(T));
}
int main() {
  for (int type : {0, 1, 2, 3}) for (bool big : {false, true}) {
    sapphire::SensorParameters params;
    params.lidar_type = type; params.point_filter_num = 2;
    sapphire_ros::LidarHandler handler(params);
    auto m = std::make_shared<Cloud>();
    m->header.stamp.sec = 99;
    m->width = 2; m->height = 2; m->point_step = 32; m->row_step = 80; m->is_bigendian = big;
    m->data.resize(160, 0x5a);
    const auto field = [&](const char *name, int offset, int datatype) {
      F f; f.name = name; f.offset = offset; f.datatype = datatype; f.count = 1; m->fields.push_back(f);
    };
    field("x", 0, F::FLOAT32); field("y", 4, F::FLOAT32); field("z", 8, F::FLOAT32);
    field(type == 0 ? "reflectivity" : "intensity", 12, type == 0 ? F::UINT8 : F::FLOAT32);
    field(type == 0 ? "offset_time" : type == 3 ? "time" : "timestamp", 16,
          type == 0 ? F::UINT32 : type == 3 ? F::FLOAT32 : F::FLOAT64);
    const std::array<double, 4> dt{.06, 0, .04, .1};
    for (size_t i = 0; i < 4; ++i) {
      store(*m, i, 0, i == 1 ? std::numeric_limits<float>::quiet_NaN() : 1.F);
      store(*m, i, 4, 0.F); store(*m, i, 8, 0.F);
      if (type == 0) {
        store(*m, i, 12, uint8_t(42)); store(*m, i, 16, uint32_t(dt[i] * 1e9));
      } else {
        store(*m, i, 12, 42.F);
        if (type == 3) store(*m, i, 16, float(dt[i] - .1));
        else store(*m, i, 16, 99. + dt[i]);
      }
    }
    std::reverse(m->fields.begin(), m->fields.end());
    double begin;
    std::optional<double> end;
    std::vector<sapphire::LidarPoint> points;
    assert(handler.process(m, begin, points, end));
    assert(points.size() == 2 && end && std::abs(begin - (type == 3 ? 98.9 : 99.)) < 1e-7);
    assert(std::abs(*end - (type == 3 ? 99. : 99.1)) < 1e-7);
    assert(points[0].intensity == 42 && std::abs(points[0].time_offset - .06) < 1e-7);
    if (type == 1 || type == 2) {
      const auto raw = sapphire_ros::decode_lidar_scan(m);
      assert(raw.size() == 4 && raw[0].intensity == 42 && std::abs(raw[0].timestamp - 99.06) < 1e-9);
      assert(std::isnan(raw[1].x)); // Decoding leaves filtering to core.
    }
    if (type == 0) store(*m, 3, 16, uint32_t(1000000000));
    else if (type == 3) store(*m, 3, 16, std::numeric_limits<float>::quiet_NaN());
    else store(*m, 3, 16, std::numeric_limits<double>::quiet_NaN());
    assert(!handler.process(m, begin, points, end) && points.empty() && !end);
  }
  for (const auto &name : {"livox", "robosense", "airy", "hesaixt", "hesai", "velodyne"})
    assert(sapphire::lidar_type_name(int(sapphire::parse_lidar_type(name))));
  std::cout << "PASS four ROS point formats, endian/padding, full bounds and invalid skipped timestamps\n";
}
