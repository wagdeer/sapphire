#pragma once
#include <rcl/publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <stdexcept>

namespace sapphire_ros {
// Local adapter completion only. The caller owns context and publisher through
// the interval; no subscriber/delivery/enqueue promise is implied.
template<class Message>
void checked_publish(const typename rclcpp::Publisher<Message>::SharedPtr &publisher,
                     const rclcpp::Context::SharedPtr &context, const Message &message,
                     std::size_t cap = 65536, double *serialization_ms = nullptr,
                     double *publish_ms = nullptr, std::size_t *copy_high_water = nullptr,
                     std::size_t retained_bytes = 0) {
  if (!publisher || !context || !context->is_valid()) throw std::runtime_error("Required ROS context unavailable");
  auto start = std::chrono::steady_clock::now();
  rclcpp::Serialization<Message> serializer;
  rclcpp::SerializedMessage encoded;
  serializer.serialize_message(&message, &encoded);
  if (encoded.size() > cap) throw std::length_error("Required encoded payload exceeds B3 envelope");
  if (serialization_ms) *serialization_ms += std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  if (copy_high_water) *copy_high_water = std::max(*copy_high_water, retained_bytes + encoded.capacity());
  start = std::chrono::steady_clock::now();
  const auto result = rcl_publish(publisher->get_publisher_handle().get(), &message, nullptr);
  if (publish_ms) *publish_ms += std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  if (result != RCL_RET_OK || !context->is_valid()) {
    rcl_reset_error();
    throw std::runtime_error("Required local ROS submission failed or context lost");
  }
}
}
