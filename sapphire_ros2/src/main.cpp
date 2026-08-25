#include <exception>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sapphire_ros2/sapphire_node.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sapphire_ros::SapphireNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 0;
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("cmn_sapphire"), "%s", error.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
