#pragma once

#include <cmath>
#include <cstddef>
#include <list>
#include <utility>
#include <vector>

namespace sapphire::backend::tools {

template <typename T>
T mean(const std::vector<T>& values) {
  T result = 0;
  for (const T& value : values) {
    result += value;
  }
  if (!values.empty()) {
    result /= values.size();
  }
  return result;
}

template <typename T>
T mean(const std::list<T>& values) {
  T result = 0;
  for (const T& value : values) {
    result += value;
  }
  if (!values.empty()) {
    result /= values.size();
  }
  return result;
}

template <typename T>
T sample_variance(const std::vector<T>& values, T mean_value) {
  if (values.size() <= 1) {
    return T{};
  }

  // Keep the legacy accumulation precision to preserve existing results.
  float sum = 0.0f;
  for (const T& value : values) {
    sum += (value - mean_value) * (value - mean_value);
  }
  return static_cast<T>(sum / (values.size() - 1));
}

template <typename T>
T sample_variance(const std::list<T>& values, T mean_value) {
  if (values.size() <= 1) {
    return T{};
  }

  float sum = 0.0f;
  for (const T& value : values) {
    sum += (value - mean_value) * (value - mean_value);
  }
  return static_cast<T>(sum / (values.size() - 1));
}

template <typename T>
T nan_aware_max(const std::vector<T>& values) {
  if (values.empty()) {
    return T{};
  }

  T result = values.front();
  for (std::size_t i = 1; i < values.size(); ++i) {
    if (std::isnan(result) || (result < values[i] && !std::isnan(values[i]))) {
      result = values[i];
    }
  }
  return result;
}

template <typename T>
std::pair<T, T> nan_aware_minmax(const std::vector<T>& values) {
  if (values.empty()) {
    return {T{}, T{}};
  }

  T minimum = values.front();
  T maximum = values.front();
  for (std::size_t i = 1; i < values.size(); ++i) {
    if (std::isnan(minimum) || (minimum > values[i] && !std::isnan(values[i]))) {
      minimum = values[i];
    }
    if (std::isnan(maximum) || (maximum < values[i] && !std::isnan(values[i]))) {
      maximum = values[i];
    }
  }
  return {minimum, maximum};
}

}  // namespace sapphire::backend::tools
