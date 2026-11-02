#include "maplayer/topograph/node.hpp"
#include "module/mapping.hpp"
#include "parameters.hpp"
#include "database/memory.hpp"
#include <iostream>
#include <stdexcept>

#include "maplayer/node_builder.hpp"
using namespace sapphire;
void check(bool ok, const char* why) {
  if (!ok) throw std::runtime_error(why);
}
LidarFrame makeFrame(double stamp, double x) {
  const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-.1, Eigen::Vector3d::UnitY())).toRotationMatrix();
  const Eigen::Vector3d position(x, -2, .8);
  StateGroup state;
  state.R = rotation;
  state.p = position;
  GaussianCloud cloud;
  for (int i = 0; i < 100; ++i) {
    GaussianPoint p;
    p.mean = {i * .02f, .3f, -.1f};
    p.covariance = Eigen::Vector3f(.001f, .02f, .03f).asDiagonal();
    p.N = 20;
    p.radius = .5;
    p.voxel_key = {i, 0, 0, 0};
    cloud.push_back(p.transformed(rotation, position));
  }
  return LidarFrame(state, std::move(cloud), stamp, stamp);
}
int main() try {
  PoseGraphParameters parameters;
  NodeBuilder builder(parameters.submap_travel_distance);
  auto first = makeFrame(1, 3);
  const auto rotation = first.x.R;
  check(!builder.push(std::move(first)), "first frame waits for explicit flush");
  auto one = builder.flush();
  check(bool(one), "submap emits");
  for (const auto& p : *one->node->lidar_cloud()) {
    check((p.mean - Eigen::Vector3f(p.voxel_key.x * .02f, .3f, -.1f)).norm() < 1e-5, "world Gaussian mean frozen in anchor coordinates");
    check((p.covariance - Eigen::Vector3f(.001f, .02f, .03f).asDiagonal().toDenseMatrix()).norm() < 1e-6, "covariance rotates with the mean");
  }
  NodeBuilder distant;
  auto far = makeFrame(1, 3);
  far.gaussians.front().mean += Eigen::Vector3f(100, 20, -10);
  const auto original_far = far.gaussians.front();
  check(!distant.push(std::move(far)), "distant evidence accepted");
  auto far_node = distant.flush();
  check(far_node && far_node->node->lidar_cloud()->size() == 100, "archive keeps all distant Gaussians");
  bool found_far = false;
  for (const auto &p : *far_node->node->lidar_cloud()) if (p.voxel_key == original_far.voxel_key) {
    const Eigen::Vector3d restored = rotation * p.mean.cast<double>() + Eigen::Vector3d(3,-2,.8);
    check((restored-original_far.mean.cast<double>()).norm()<1e-4 && p.N == original_far.N,
          "distant geometry and sample ownership survive anchor transform");
    found_far = true;
  }
  check(found_far, "distant identity retained");
  NodeBuilder bounded(10000);
  for (int i = 0; i < 256; ++i)
    check(!bounded.push(makeFrame(i + 1, 3)), "buffer holds bounded frame history");
  auto closed = bounded.push(makeFrame(257, 3));
  check(closed && closed->trajectory.size() == 255 && bounded.buffered_frame_count() == 1,
        "frame budget flushes before retaining the next frame");
  auto large = makeFrame(258, 3);
  const auto exemplar = large.gaussians.front();
  large.gaussians.assign(30000, exemplar);
  for (size_t i = 0; i < large.gaussians.size(); ++i) large.gaussians[i].voxel_key.x = i;
  check(!bounded.push(std::move(large)), "Gaussian count does not split a submap");
  auto complete = bounded.flush();
  check(complete && complete->node->lidar_cloud()->size() == 30000 && complete->trajectory.size() == 1,
        "large cloud is retained in full with its original trajectory");
  check(!bounded.flush(), "tail is emitted once");
  NodeBuilder bytes(2, 150 * sizeof(GaussianPoint));
  check(!bytes.push(makeFrame(1, 3)), "first frame fits payload");
  check(!bytes.push(makeFrame(2, 3)), "same identities merge within budget");
  auto different = makeFrame(3, 5);
  for (auto &p : different.gaussians) p.voxel_key.x += 100;
  auto byte_closed = bytes.push(std::move(different));
  check(byte_closed && byte_closed->node->lidar_cloud()->size() == 100 &&
        byte_closed->node->lidar_cloud()->front().N == 40 &&
        byte_closed->trajectory.size() == 1 && bytes.buffered_frame_count() == 1,
        "byte boundary wins before distance crossing without dropping either node");
  auto too_large = makeFrame(4, 6);
  too_large.gaussians.resize(151);
  bool size_rejected = false;
  try { (void)bytes.push(std::move(too_large)); }
  catch (const std::length_error &) { size_rejected = true; }
  check(size_rejected && bytes.buffered_frame_count() == 1, "oversize input is rejected before mutation");
  auto byte_tail = bytes.flush();
  check(byte_tail && byte_tail->node->getStamp() == 3 &&
        byte_tail->node->lidar_cloud()->size() == 100 && !bytes.flush(), "bounded tail preserved once");
  NodeBuilder by_distance(2);
  check(!by_distance.push(makeFrame(1, 3)), "distance starts at first observation");
  auto distance_closed = by_distance.push(makeFrame(3, 5));
  check(distance_closed && distance_closed->trajectory.size() == 1 && by_distance.empty(),
        "distance threshold emits the complete interval");
  check(distance_closed->node->lidar_cloud()->size() == 100 && distance_closed->node->lidar_cloud()->front().N == 40,
        "repeated voxel identities merge without duplicating units");
  check(distance_closed->node->getStamp() == 1 && distance_closed->trajectory.back().timestamp == 3,
        "original ordered trajectory is retained");
  by_distance.clear();
  auto invalid = makeFrame(4, 6);
  invalid.timestamp = std::numeric_limits<double>::quiet_NaN();
  bool rejected = false;
  try { (void)by_distance.push(std::move(invalid)); } catch (const std::invalid_argument&) { rejected = true; }
  check(rejected && by_distance.empty(), "invalid observation rejected before accumulation");
  Mapping mapping;
  MappingParameters mapping_parameters;
  mapping_parameters.relocalizer.spatial_enabled = false;
  mapping.init(mapping_parameters, "");
  const Node* built = one->node.get();
  const auto cloud = built->lidar_cloud();
  const auto pose = built->getPose();
  const auto stamp = built->getStamp();
  check(built->id() == 0 && built->mapId() == -1 && built->getLinks().empty() && one->trajectory.empty(),
        "builder creates a complete detached node; the anchor is not repeated in the trajectory");
  check(mapping.process_node(std::move(one->node), Mat66_t<double>::Identity()), "built node admission");
  const auto* admitted = mapping.getMemory()->getNode(mapping.getLastLocationId());
  check(!one->node && admitted == built, "the exact built node is transferred, never reconstructed");
  check(admitted->id() > 0 && admitted->mapId() >= 0, "map identity is assigned only on admission");
  check(admitted->lidar_cloud() == cloud && admitted->getPose().matrix() == pose.matrix(), "geometry and anchor remain unchanged");
  check(admitted->getStamp() == stamp, "original observation stamp preserved");
  const int id = admitted->id();
  rejected = false;
  try { (void)mapping.process_node(std::make_unique<Node>(*admitted), Mat66_t<double>::Identity()); }
  catch (const std::invalid_argument&) { rejected = true; }
  check(rejected && mapping.getLastLocationId() == id, "map-bound nodes cannot be admitted again");
  check(mapping.process_node(std::move(complete->node), Mat66_t<double>::Identity()), "next detached node accepted");
  check(mapping.getLastLocationId() == id + 1, "rejected duplicate did not reserve an ID");
  mapping.close();
  std::cout << "PASS frozen Gaussian input\n";
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
