#include <cassert>
#include <chrono>
#include <future>
#include <filesystem>
#include <unistd.h>
#include <iostream>

#include "slam.hpp"
namespace sapphire {
struct SlamPipelineTestAccess {
  static LidarFrame frame(double stamp) {
    GaussianCloud cloud(37);
    cloud.front().mean = {1, 2, 3};
    return {StateGroup{}, std::move(cloud), stamp, 4};
  }
  static void verify(SlamPipeline& pipeline) {
    pipeline.shutdown();  // isolate the production handoff from its consumer
    auto first = frame(1);
    const auto* allocation = first.gaussians.data();
    pipeline.transfer(std::move(first));
    assert(pipeline.marginal_frames_.front().gaussians.data() == allocation);
    LidarFrame consumed(std::move(pipeline.marginal_frames_.front()));
    pipeline.marginal_frames_.pop_front();
    assert(consumed.gaussians.data() == allocation);
    assert(consumed.gaussians.front().mean == Eigen::Vector3f(1, 2, 3));
    assert(consumed.timestamp == 1 && consumed.journey == 4);
    pipeline.transfer(frame(2));
    pipeline.transfer(frame(3));
    bool bounded = false;
    try {
      pipeline.transfer(frame(4));
    } catch (const std::logic_error&) {
      bounded = true;
    }
    assert(bounded && pipeline.marginal_frames_.size() == 2);
    // Re-enable only the isolated queue's producer wait, not either runtime.
    pipeline.stopping_.store(false);
    auto waiting = std::async(std::launch::async, [&] { return pipeline.waitForCapacity(); });
    assert(waiting.wait_for(std::chrono::milliseconds(30)) == std::future_status::timeout);
    pipeline.shutdown();
    assert(waiting.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
    assert(!waiting.get() && pipeline.marginal_frames_.size() == 2);
    assert(!pipeline.requestReset(1)); // A stopped owner cannot reopen a session.
    assert(pipeline.marginal_frames_.size() == 2);
  }
};
}  // namespace sapphire
int main() {
  sapphire::SapphireParameters parameters;
  parameters.pose_graph.enabled = false;
  parameters.general.save_path = (std::filesystem::temp_directory_path() / ("sapphire-disabled-map-" + std::to_string(getpid()))).string();
  assert(!std::filesystem::exists(parameters.general.save_path));
  parameters.local_submap.thread_num = 1;
  sapphire::SlamPipeline pipeline(parameters);
  sapphire::SlamPipelineTestAccess::verify(pipeline);
  assert(!std::filesystem::exists(parameters.general.save_path));
  std::cout << "PASS allocation identity, bounded backpressure, stop wakeup and reset ownership\n";
}
