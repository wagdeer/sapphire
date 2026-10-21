#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <system_error>

namespace sapphire::backend::tools {

// Returns the current process's Linux VmRSS in bytes.
std::uint64_t resident_set_size();
std::optional<std::uint64_t> resident_set_size(std::error_code& error) noexcept;

// The path overload supports deterministic parser and error tests.
std::optional<std::uint64_t> resident_set_size(const std::filesystem::path& status_path, std::error_code& error) noexcept;

}  // namespace sapphire::backend::tools
