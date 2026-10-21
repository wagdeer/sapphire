#include <array>
#include <charconv>
#include <backend/tools/strings.hpp>
#include <limits>
#include <system_error>
#include <vector>

namespace sapphire::backend::tools {
namespace {

unsigned char hex_nibble(char character) noexcept {
  if (character >= '0' && character <= '9') {
    return static_cast<unsigned char>(character - '0');
  }
  if (character >= 'A' && character <= 'F') {
    return static_cast<unsigned char>(character - 'A' + 10);
  }
  if (character >= 'a' && character <= 'f') {
    return static_cast<unsigned char>(character - 'a' + 10);
  }
  return 0;
}

constexpr std::size_t kStackBufferSize = 128;

char ascii_lower(char value) noexcept {
  if (value >= 'A' && value <= 'Z') {
    return static_cast<char>(value - 'A' + 'a');
  }
  return value;
}

bool contains_ascii_case_insensitive(std::string_view text, std::string_view needle) noexcept {
  if (needle.size() > text.size()) {
    return false;
  }
  for (std::size_t i = 0; i <= text.size() - needle.size(); ++i) {
    std::size_t j = 0;
    while (j < needle.size() && ascii_lower(text[i + j]) == needle[j]) {
      ++j;
    }
    if (j == needle.size()) {
      return true;
    }
  }
  return false;
}

std::string_view trim_numeric_prefix(std::string_view text) noexcept {
  while (!text.empty() &&
         (text.front() == ' ' || text.front() == '\t' || text.front() == '\n' || text.front() == '\r' || text.front() == '\f' || text.front() == '\v')) {
    text.remove_prefix(1);
  }
  if (!text.empty() && text.front() == '+') {
    text.remove_prefix(1);
  }
  return text;
}

template <typename Number>
std::optional<Number> parse_floating(std::string_view text) noexcept {
  text = trim_numeric_prefix(text);
  if (text.empty()) {
    return std::nullopt;
  }

  auto parse = [](std::string_view normalized) -> std::optional<Number> {
    Number value = 0;
    const auto result = std::from_chars(normalized.data(), normalized.data() + normalized.size(), value, std::chars_format::general);
    if (result.ec != std::errc() || result.ptr == normalized.data()) {
      return std::nullopt;
    }
    return value;
  };

  if (text.find(',') == std::string_view::npos) {
    return parse(text);
  }

  if (text.size() <= kStackBufferSize) {
    std::array<char, kStackBufferSize> buffer{};
    for (std::size_t i = 0; i < text.size(); ++i) {
      buffer[i] = text[i] == ',' ? '.' : text[i];
    }
    return parse(std::string_view(buffer.data(), text.size()));
  }

  std::string normalized(text);
  for (char& value : normalized) {
    if (value == ',') {
      value = '.';
    }
  }
  return parse(normalized);
}

template <typename Number>
std::string format_floating(Number value, int precision, bool fixed) {
  if (precision < 0) {
    precision = 6;
  }
  const std::chars_format format = fixed ? std::chars_format::fixed : std::chars_format::general;

  std::array<char, kStackBufferSize> stackBuffer{};
  auto result = std::to_chars(stackBuffer.data(), stackBuffer.data() + stackBuffer.size(), value, format, precision);
  if (result.ec == std::errc()) {
    return std::string(stackBuffer.data(), result.ptr);
  }

  std::size_t size = static_cast<std::size_t>(precision) + static_cast<std::size_t>(std::numeric_limits<Number>::max_exponent10) + 32;
  if (size < kStackBufferSize * 2) {
    size = kStackBufferSize * 2;
  }
  while (true) {
    std::vector<char> buffer(size);
    result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value, format, precision);
    if (result.ec == std::errc()) {
      return std::string(buffer.data(), result.ptr);
    }
    if (result.ec != std::errc::value_too_large) {
      return {};
    }
    size *= 2;
  }
}

}  // namespace

std::optional<int> parse_int(std::string_view text) noexcept {
  if (contains_ascii_case_insensitive(text, "true")) {
    return 1;
  }
  if (contains_ascii_case_insensitive(text, "false")) {
    return 0;
  }

  text = trim_numeric_prefix(text);
  if (text.empty()) {
    return std::nullopt;
  }
  int value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc() || result.ptr == text.data()) {
    return std::nullopt;
  }
  return value;
}

std::optional<float> parse_float(std::string_view text) noexcept { return parse_floating<float>(text); }

std::optional<double> parse_double(std::string_view text) noexcept { return parse_floating<double>(text); }

int parse_int_or(std::string_view text, int fallback) noexcept {
  const std::optional<int> value = parse_int(text);
  return value.value_or(fallback);
}

float parse_float_or(std::string_view text, float fallback) noexcept {
  const std::optional<float> value = parse_float(text);
  return value.value_or(fallback);
}

double parse_double_or(std::string_view text, double fallback) noexcept {
  const std::optional<double> value = parse_double(text);
  return value.value_or(fallback);
}

bool parse_bool(std::string_view text) noexcept { return !contains_ascii_case_insensitive(text, "false") && text != "0"; }

std::string format_float(float value, int precision, bool fixed) { return format_floating(value, precision, fixed); }

std::string format_double(double value, int precision, bool fixed) { return format_floating(value, precision, fixed); }

std::string hex_to_bytes(std::string_view hex) {
  if (hex.empty() || hex.size() % 2 != 0) {
    return {};
  }

  std::string bytes(hex.size() / 2, '\0');
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    const unsigned char high = hex_nibble(hex[index * 2]);
    const unsigned char low = hex_nibble(hex[index * 2 + 1]);
    bytes[index] = static_cast<char>((high << 4) | low);
  }
  return bytes;
}

}  // namespace sapphire::backend::tools
