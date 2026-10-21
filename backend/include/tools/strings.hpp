#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sapphire::backend::tools {

namespace detail {
inline bool is_ascii_digit(char character) noexcept { return character >= '0' && character <= '9'; }

inline bool is_integer(std::string_view text, bool allow_sign = true) noexcept {
  if (text.empty()) {
    return false;
  }
  for (std::size_t index = 0; index < text.size(); ++index) {
    if (!((allow_sign && index == 0 && text[index] == '-') || is_ascii_digit(text[index]))) {
      return false;
    }
  }
  return true;
}
}  // namespace detail

inline std::string replace_char(std::string_view text, char before, char after) {
  std::string result(text);
  for (char& character : result) {
    if (character == before) {
      character = after;
    }
  }
  return result;
}

inline std::string replace_char(std::string_view text, char before, std::string_view after) {
  std::size_t replacement_count = 0;
  for (const char character : text) {
    if (character == before) {
      ++replacement_count;
    }
  }

  std::string result;
  result.reserve(text.size() - replacement_count + replacement_count * after.size());
  for (const char character : text) {
    if (character == before) {
      result.append(after.data(), after.size());
    } else {
      result.push_back(character);
    }
  }
  return result;
}

// 有意丢弃空字段以保留配置和轨迹解析合同的原ustl函数接口
inline std::vector<std::string> split(std::string_view input, char separator = ' ') {
  std::vector<std::string> parts;
  std::size_t begin = 0;
  while (begin < input.size()) {
    const std::size_t end = input.find(separator, begin);
    const std::size_t count = (end == std::string_view::npos ? input.size() : end) - begin;
    if (count != 0) {
      parts.emplace_back(input.substr(begin, count));
    }
    if (end == std::string_view::npos) {
      break;
    }
    begin = end + 1;
  }
  return parts;
}

template <typename Range>
std::string join(const Range& strings, std::string_view separator = {}) {
  std::string result;
  bool first = true;
  for (const auto& string : strings) {
    if (!first) {
      result.append(separator.data(), separator.size());
    }
    result.append(string);
    first = false;
  }
  return result;
}

inline std::string pad_right(std::string_view text, std::size_t width = 20) {
  std::string result(text);
  if (result.size() < width) {
    result.append(width - result.size(), ' ');
  }
  return result;
}

inline bool is_number(std::string_view text) {
  const auto parts = split(text, '.');
  if (parts.size() == 1) {
    return detail::is_integer(text);
  }
  return parts.size() == 2 && detail::is_integer(parts.front()) && detail::is_integer(parts.back(), false);
}

std::optional<int> parse_int(std::string_view text) noexcept;
std::optional<float> parse_float(std::string_view text) noexcept;
std::optional<double> parse_double(std::string_view text) noexcept;

int parse_int_or(std::string_view text, int fallback = 0) noexcept;
float parse_float_or(std::string_view text, float fallback = 0.0f) noexcept;
double parse_double_or(std::string_view text, double fallback = 0.0) noexcept;

// Compatibility parsing: strings containing "false" (ASCII case-insensitive)
// and exactly "0" are false; every other string is true.
bool parse_bool(std::string_view text) noexcept;

std::string format_float(float value, int precision = 6, bool fixed = false);
std::string format_double(double value, int precision = 6, bool fixed = false);

std::string hex_to_bytes(std::string_view hex);

}  // namespace sapphire::backend::tools
