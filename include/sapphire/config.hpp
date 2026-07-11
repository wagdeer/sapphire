#pragma once

#include <sapphire/types.hpp>

#include <filesystem>

namespace sapphire {

/// Load and validate a Sapphire TOML configuration file.
/// Throws std::runtime_error when the file cannot be parsed or is invalid.
Config loadConfig(const std::filesystem::path& path);

/// Validate a fully constructed configuration.
/// Throws std::invalid_argument when a value is invalid.
void validateConfig(const Config& config);

}  // namespace sapphire
