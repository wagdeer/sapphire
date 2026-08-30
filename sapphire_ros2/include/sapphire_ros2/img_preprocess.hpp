#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

#include "visual_frame.hpp"

namespace sapphire_ros {

class ImageProcessor {
 public:
  std::optional<sapphire::ImageMeas> process(const sensor_msgs::msg::Image::ConstSharedPtr &message) const {
    const std::string &encoding = message->encoding;
    if (encoding == sensor_msgs::image_encodings::MONO8 || encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
      if (!has_storage(*message, 1)) {
        return std::nullopt;
      }
      cv::Mat gray(static_cast<int>(message->height), static_cast<int>(message->width), CV_8UC1, const_cast<uint8_t *>(message->data.data()),
                   message->step);
      std::shared_ptr<const void> owner = message;
      return sapphire::ImageMeas(timestamp(*message), std::move(gray), std::move(owner));
    }

    int source_type = 0;
    int conversion = 0;
    size_t channels = 0;
    if (encoding == sensor_msgs::image_encodings::BGR8) {
      source_type = CV_8UC3;
      conversion = cv::COLOR_BGR2GRAY;
      channels = 3;
    } else if (encoding == sensor_msgs::image_encodings::RGB8) {
      source_type = CV_8UC3;
      conversion = cv::COLOR_RGB2GRAY;
      channels = 3;
    } else if (encoding == sensor_msgs::image_encodings::BGRA8) {
      source_type = CV_8UC4;
      conversion = cv::COLOR_BGRA2GRAY;
      channels = 4;
    } else if (encoding == sensor_msgs::image_encodings::RGBA8) {
      source_type = CV_8UC4;
      conversion = cv::COLOR_RGBA2GRAY;
      channels = 4;
    } else {
      return std::nullopt;
    }
    if (!has_storage(*message, channels)) {
      return std::nullopt;
    }

    try {
      const cv::Mat source(static_cast<int>(message->height), static_cast<int>(message->width), source_type,
                           const_cast<uint8_t *>(message->data.data()), message->step);
      cv::Mat gray;
      cv::cvtColor(source, gray, conversion);
      return sapphire::ImageMeas(timestamp(*message), std::move(gray));
    } catch (const cv::Exception &) {
      return std::nullopt;
    }
  }

  std::optional<sapphire::ImageMeas> process(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &message) const {
    if (message->data.empty() || message->data.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
      return std::nullopt;
    }
    try {
      const cv::Mat encoded(1, static_cast<int>(message->data.size()), CV_8UC1, const_cast<uint8_t *>(message->data.data()));
      cv::Mat gray = cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
      if (gray.empty()) {
        return std::nullopt;
      }
      return sapphire::ImageMeas(timestamp(*message), std::move(gray));
    } catch (const cv::Exception &) {
      return std::nullopt;
    }
  }

 private:
  static bool has_storage(const sensor_msgs::msg::Image &message, size_t channels) {
    if (message.width == 0 || message.height == 0 || message.step < static_cast<size_t>(message.width) * channels) {
      return false;
    }
    return message.step <= std::numeric_limits<size_t>::max() / message.height &&
           message.data.size() >= static_cast<size_t>(message.step) * message.height;
  }

  template <typename Message>
  static double timestamp(const Message &message) {
    return static_cast<double>(message.header.stamp.sec) + static_cast<double>(message.header.stamp.nanosec) * 1e-9;
  }
};

}  // namespace sapphire_ros
