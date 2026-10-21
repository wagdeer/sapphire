#include <charconv>
#include <backend/tools/process.hpp>
#include <fstream>
#include <limits>
#include <string>
#include <string_view>
#include <system_error>

namespace sapphire::backend::tools {
namespace {

constexpr const char* kSelfStatusPath = "/proc/self/status";

std::string_view trim_left(std::string_view value) noexcept {
  while (!value.empty() && (value.front() == ' ' || value.front() == '\t')) {
    value.remove_prefix(1);
  }
  return value;
}

}  // namespace

std::uint64_t resident_set_size() {
  std::error_code error;
  const std::optional<std::uint64_t> bytes = resident_set_size(error);
  if (!bytes) {
    throw std::system_error(error, "cannot read current process RSS");
  }
  return *bytes;
}

std::optional<std::uint64_t> resident_set_size(const std::filesystem::path& status_path, std::error_code& error) noexcept {
  error.clear();
  std::ifstream input(status_path);
  if (!input) {
    error = std::make_error_code(std::errc::no_such_file_or_directory);
    return std::nullopt;
  }

  std::string line;
  while (std::getline(input, line)) {
    constexpr std::string_view prefix = "VmRSS:";
    if (line.compare(0, prefix.size(), prefix) != 0) {
      continue;
    }
    std::string_view value = trim_left(std::string_view(line).substr(prefix.size()));
    std::uint64_t kibibytes = 0;
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), kibibytes);
    if (parsed.ec != std::errc{}) {
      error = std::make_error_code(parsed.ec);
      return std::nullopt;
    }
    value.remove_prefix(static_cast<std::size_t>(parsed.ptr - value.data()));
    value = trim_left(value);
    if (value != "kB") {
      error = std::make_error_code(std::errc::protocol_error);
      return std::nullopt;
    }
    if (kibibytes > std::numeric_limits<std::uint64_t>::max() / 1024U) {
      error = std::make_error_code(std::errc::value_too_large);
      return std::nullopt;
    }
    return kibibytes * 1024U;
  }

  error = std::make_error_code(std::errc::no_message);
  return std::nullopt;
}

std::optional<std::uint64_t> resident_set_size(std::error_code& error) noexcept { return resident_set_size(kSelfStatusPath, error); }

}  // namespace sapphire::backend::tools
