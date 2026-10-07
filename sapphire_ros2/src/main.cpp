#include <csignal>
#include <exception>
#include <memory>
#include <opencv2/core/utility.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sapphire_ros2/sapphire_node.hpp>

namespace {
volatile std::sig_atomic_t stop_signal = 0;
extern "C" void request_stop(int signal) { stop_signal = signal; }
}
int main(int argc, char **argv) {
  // The application owns OpenCV's process-wide policy. Small visual frames run
  // on the mapping worker alongside LIO; nested image-kernel pools add overhead.
  // Set this before any worker starts, never inside a core-library callback.
  cv::setNumThreads(2);
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  RCLCPP_INFO(rclcpp::get_logger("cmn_sapphire"), "OpenCV internal threads: %d", cv::getNumThreads());
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  int result = 1;
  try {
    auto node = std::make_shared<sapphire_ros::SapphireNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2);
    executor.add_node(node);
    // Existing executor timer consumes the signal-safe bridge; no signal worker.
    auto signal_bridge = node->create_wall_timer(std::chrono::milliseconds(50), [&] {
      if (stop_signal || node->stop_requested()) { node->request_stop(); executor.cancel(); }
    });
    executor.spin(); // Returning joins active callbacks before checked finish.
    signal_bridge.reset();
    node->request_stop();
    executor.remove_node(node);
    result = node->finish_checked() ? 0 : 1;
    node.reset();
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("cmn_sapphire"), "%s", error.what());
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("cmn_sapphire"), "Non-standard checked shutdown failure");
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
