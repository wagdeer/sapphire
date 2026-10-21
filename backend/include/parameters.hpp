#pragma once
#include <backend/common/common_type.hpp>
#include <optional>
#include <string>
#include <string_view>

namespace sapphire::backend {

class Parameters {
 public:
  enum class scalar_type_t { Bool, Int, UnsignedInt, Float, Double, String };

  static const ParametersMap& getDefaultParameters();

  // Unknown names have no type; startup configuration rejects them.
  static std::optional<scalar_type_t> getType(const std::string& paramKey);

  static std::string getDescription(const std::string& paramKey);

  // Validate current catalog names and scalar encoding; module constraints belong to *Params::validate().
  static void validate(const ParametersMap& parameters);

  // Supported scalar types: bool, int, unsigned int, float, double and string.
  template <typename T>
  static bool parse(const ParametersMap& parameters, std::string_view key, T& value);
  static void parse(const ParametersMap& parameters, ParametersMap& parametersOut);

  static std::string getDefaultDatabaseName();

  // Historical records are provenance, not startup configuration. Validate only
  // the name:value;... storage grammar, preserving names and values verbatim.
  static void validateRecord(const ParametersMap& parameters);
  static std::string serialize(const ParametersMap& parameters);
  static ParametersMap deserialize(const std::string& parameters);

 private:
  Parameters() = delete;
};

class LoadParameters final {
 public:
  LoadParameters() = delete;

  // Read explicit file values; no command-line overlays or database fallback.
  static ParametersMap read(const std::string& path);

  // Complete and validate programmatic startup values.
  static ParametersMap from_map(const ParametersMap& values);

  // Runtime owns input/lifecycle fields; the file must not duplicate them.
  static ParametersMap from_file(const std::string& path, const ParametersMap& runtime = {});

 private:
  static void validate_modules(const ParametersMap& values);
};

}  // namespace sapphire::backend
