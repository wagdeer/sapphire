#include <slam.hpp>
#include <sensor/lidar_sync.hpp>
#include <database/compression.hpp>
#include <database/memory.hpp>
#include <maplayer/node_builder.hpp>
#include <maplayer/local_map.hpp>
#include <iostream>
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  const std::filesystem::path config(argv[1]);
  const auto sensors = sapphire::load_sensor_parameters(config / "sensors.toml");
  auto parameters = sapphire::load_parameters(config / "params.toml", sensors.lidars.front());
  parameters.pose_graph.enabled = false;
  {
    sapphire::tools::ParallelExecutor<> executor(parameters.local_submap.thread_num);
    sapphire::LocalMap local_map(parameters, executor);
    local_map.reset();
    if (local_map.size() != 0 || local_map.capacity() != parameters.local_submap.win_size) return 1;
  }
  sapphire::GaussianPoint point;
  point.mean = {1, 2, 3};
  point.covariance = Eigen::Matrix3f::Identity() * .01f;
  point.N = 12; point.radius = .2f;
  const sapphire::GaussianCloud cloud{point};
  const auto decoded = sapphire::lidar::decode(sapphire::lidar::encode(cloud));
  if (decoded->size() != 1 || decoded->front().mean != point.mean) return 1;
  {
    sapphire::Memory memory;
    if (!memory.init("")) return 1;
    auto* node = memory.admit_node(std::make_unique<sapphire::Node>(decoded, 1, sapphire::SE3f::identity()),
                                   sapphire::Mat66_t<double>::Identity());
    if (!node || memory.getNode(node->id()) != node) return 1;
    memory.close(false);
  }
  sapphire::NodeBuilder builder;
  (void)builder.push(sapphire::LidarFrame(sapphire::StateGroup{}, sapphire::GaussianCloud(cloud), 1, 0));
  const auto built = builder.flush();
  if (!built || built->node->lidar_cloud()->size() != 1 || !built->trajectory.empty() || built->node->id() != 0) return 1;
  sapphire::LidarSync fusion(sapphire::LidarSyncParameters{});
  std::vector<sapphire::TimedLidarPoint> raw{{1, 0, 0, 1, 10}, {1, 0, 0, 1, 10.05}, {1, 0, 0, 1, 10.1}};
  if (!fusion.push(false, raw, "front") || !fusion.push(true, raw, "rear") || !fusion.pop()) return 1;
  sapphire::SlamPipeline::Callbacks callbacks;
  callbacks.odom_state = [](const sapphire::StateGroup &) {};
  sapphire::SlamPipeline pipeline(parameters, std::move(callbacks));
  pipeline.shutdown();
  const sapphire::SlamPipeline::ContinuationProgress progress = pipeline.drain();
  if (progress.first_failure) std::rethrow_exception(progress.first_failure);
  pipeline.close();
  std::cout << "PASS installed native core, config, Gaussian codec and pipeline lifecycle\n";
}
