#include "pipeline/backend.hpp"
#include "database/memory.hpp"
#include <filesystem>
#include <future>
#include <iostream>
#include <thread>

#include "maplayer/node_builder.hpp"
#include "slam.hpp"
namespace sapphire {
struct SlamPipelineTestAccess {
  static const Node* resident(SlamPipeline& p, int id) { return p.backend_->pose_graph_->getMemory()->getNode(id); }
  static void submit(SlamPipeline& p, NodeBuilder::Result frame) { p.backend_->submitNode(std::move(frame)); }
};
}  // namespace sapphire
using namespace sapphire;
void check(bool ok, const char* why) {
  if (!ok) throw std::runtime_error(why);
}
LidarFrame frame(double stamp, double x) {
  LidarFrame f(StateGroup{}, GaussianCloud{}, stamp, stamp);
  f.x.R = Eigen::AngleAxisd(.12, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  f.x.p = {x, .2, .5};
  for (int i = 0; i < 400; ++i) {
    GaussianPoint p;
    p.mean = Eigen::Vector3f(std::sin(i * .3f) * 3, std::cos(i * .7f) * 2, std::sin(i * .17f));
    p.covariance = Eigen::Vector3f(.01, .02, .03).asDiagonal();
    p.N = 20;
    p.radius = .5;
    p.voxel_key = {i, 0, 0, 0};
    f.gaussians.push_back(p.transformed(f.x.R, f.x.p));
  }
  return f;
}
int main(int argc, char** argv) try {
  check(argc == 4, "usage CONFIG DB write|reopen");
  auto params = load_parameters(argv[1], load_sensor_parameters(std::filesystem::path(argv[1]).parent_path() / "sensors.toml").lidars.front());
  params.pose_graph.database_path = argv[2];
  params.general.save_path = (std::filesystem::path(argv[2]).parent_path() / "unused-save-path").string();
  check(!std::filesystem::exists(params.general.save_path), "fresh unused output path");
  params.pose_graph.mapping.registration.lidar_bbs_enabled = false;
  params.pose_graph.mapping.relocalizer.spatial_enabled = false;
  const bool cancel = std::string(argv[3]) == "cancel";
  const bool resume = cancel || std::string(argv[3]) == "reopen";
  params.pose_graph.map_mode = resume ? "resume" : "new";
  params.pose_graph.mapping.relocalizer.global_enabled=false; // This fixture checks independent resume persistence.
  SlamPipeline pipeline(params);
  if(cancel) {pipeline.drain();pipeline.close();std::cout<<"PASS no-input resume close\n";return 0;}
  check(!std::filesystem::exists(params.general.save_path), "explicit database path creates no unrelated output folder");
  NodeBuilder builder(params.pose_graph.submap_travel_distance);
  const auto f = frame(1, 3);
  (void)builder.push(LidarFrame(f));
  (void)builder.push(frame(2, 3.2));
  auto submap = builder.flush();
  check(bool(submap), "submap emitted");
  check((submap->node->lidar_cloud()->front().covariance - submap->node->lidar_cloud()->front().covariance.transpose()).norm() < 1e-5, "frozen covariance symmetric");
  const auto before = f.x.p;
  const Node* built = submap->node.get();
  const auto geometry = built->lidar_cloud();
  SlamPipelineTestAccess::submit(pipeline, std::move(*submap));
  auto status = pipeline.drain();
  check(status.accepted == 1 && status.completed == 1 && status.ready_available, "durable ready publication");
  check(status.completed_revision && SlamPipelineTestAccess::resident(pipeline, int(*status.completed_revision)) == built,
        "resume and normal admission preserve the built Node object");
  check(status.last_completed == 0, "submission sequence remains unchanged across retry");
  double stamp;
  Eigen::Isometry3d correction, pose;
  const auto captured = pipeline.captureReady(false, stamp, correction, pose);
  check(captured.has_value(), "capture committed map");
  check(!captured->map_id.empty() && captured->ready_revision == status.ready_revision &&
            (pose.matrix() - (correction * Eigen::Translation3d(f.x.p) * Eigen::Quaterniond(f.x.R)).matrix()).norm() < 1e-4,
        "pose/correction revision agreement");
  check(f.x.p == before, "backend leaves odometry unchanged");
  pipeline.close();
  std::cout << "PASS\n";
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
