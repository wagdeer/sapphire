#include <toml++/toml.h>

#include <backend/common/common_type.hpp>
#include <backend/parameters.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/strings.hpp>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iterator>
#include <limits>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace sapphire::backend {

std::string Parameters::getDefaultDatabaseName() { return "rtabmap.db"; }

void Parameters::validateRecord(const ParametersMap& parameters) {
  for (const auto& [key, value] : parameters) {
    if (key.empty() || key.find_first_of(":;") != std::string::npos || key.find('\0') != std::string::npos || value.find(';') != std::string::npos ||
        value.find('\0') != std::string::npos)
      throw std::invalid_argument("Parameter record cannot encode entry: " + key);
  }
}

std::string Parameters::serialize(const ParametersMap& parameters) {
  validateRecord(parameters);
  std::stringstream output;
  for (ParametersMap::const_iterator iter = parameters.begin(); iter != parameters.end(); ++iter) {
    if (iter != parameters.begin()) {
      output << ";";
    }
    output << iter->first << ":" << iter->second;
  }
  EXT_LOG_DEBUG("output={}", output.str().c_str());
  return output.str();
}

ParametersMap Parameters::deserialize(const std::string& parameters) {
  EXT_LOG_DEBUG("parameters={}", parameters.c_str());
  ParametersMap output;
  for (const auto& entry : tools::split(parameters, ';')) {
    const auto colon = entry.find(':');
    if (colon == std::string::npos || colon == 0) throw std::invalid_argument("Invalid parameter record: " + entry);
    // Empty string values are valid; a parameter name may occur only once.
    const auto key = entry.substr(0, colon);
    if (!output.emplace(key, entry.substr(colon + 1)).second) throw std::invalid_argument("Duplicate parameter: " + key);
  }
  validateRecord(output);
  return output;
}

std::string_view scalar_text(const std::string& text) {
  const auto first = text.find_first_not_of(" \t\n\r\f\v");
  if (first == std::string::npos) return {};
  const auto last = text.find_last_not_of(" \t\n\r\f\v");
  return std::string_view(text).substr(first, last - first + 1);
}
bool equals_ascii(std::string_view value, std::string_view word) {
  if (value.size() != word.size()) return false;
  for (size_t i = 0; i < word.size(); ++i) {
    const char c = value[i] >= 'A' && value[i] <= 'Z' ? value[i] + ('a' - 'A') : value[i];
    if (c != word[i]) return false;
  }
  return true;
}
[[noreturn]] void invalid_scalar(std::string_view key, const std::string& text) {
  throw std::invalid_argument("Invalid parameter " + std::string(key) + "='" + text + "': expected a complete, finite value in range");
}
template <class T>
T scalar(std::string_view key, const std::string& text) {
  auto input = scalar_text(text);
  if constexpr (std::is_same_v<T, bool>) {
    if (equals_ascii(input, "true") || input == "1") return true;
    if (equals_ascii(input, "false") || input == "0") return false;
    invalid_scalar(key, text);
  } else {
    if (!input.empty() && input.front() == '+') {
      input.remove_prefix(1);
      if (!input.empty() && (input.front() == '+' || input.front() == '-')) invalid_scalar(key, text);
    }
    if (input.empty()) invalid_scalar(key, text);
    T result{};
    const auto parsed = std::from_chars(input.data(), input.data() + input.size(), result);
    if (parsed.ec != std::errc() || parsed.ptr != input.data() + input.size()) invalid_scalar(key, text);
    if constexpr (std::is_floating_point_v<T>) {
      if (!std::isfinite(result)) invalid_scalar(key, text);
    }
    return result;
  }
}
template <class T>
bool parse_scalar(const ParametersMap& parameters, std::string_view key, T& value) {
  const auto found = parameters.find(key);
  if (found == parameters.end()) return false;
  const T draft = scalar<T>(key, found->second);
  value = draft;
  return true;
}

template <class T>
bool Parameters::parse(const ParametersMap& parameters, std::string_view key, T& value) {
  if constexpr (std::is_same_v<T, std::string>) {
    const auto found = parameters.find(key);
    if (found == parameters.end()) return false;
    value = found->second;
    return true;
  } else
    return parse_scalar(parameters, key, value);
}
template bool Parameters::parse<bool>(const ParametersMap&, std::string_view, bool&);
template bool Parameters::parse<int>(const ParametersMap&, std::string_view, int&);
template bool Parameters::parse<unsigned int>(const ParametersMap&, std::string_view, unsigned int&);
template bool Parameters::parse<float>(const ParametersMap&, std::string_view, float&);
template bool Parameters::parse<double>(const ParametersMap&, std::string_view, double&);
template bool Parameters::parse<std::string>(const ParametersMap&, std::string_view, std::string&);

void Parameters::validate(const ParametersMap& parameters) {
  for (const auto& [key, value] : parameters) {
    const auto type = getType(key);
    if (!type) throw std::invalid_argument("Unknown algorithm parameter: " + key);
    switch (*type) {
      case scalar_type_t::Bool:
        scalar<bool>(key, value);
        break;
      case scalar_type_t::Int:
        scalar<int>(key, value);
        break;
      case scalar_type_t::UnsignedInt:
        scalar<unsigned int>(key, value);
        break;
      case scalar_type_t::Float:
        scalar<float>(key, value);
        break;
      case scalar_type_t::Double:
        scalar<double>(key, value);
        break;
      case scalar_type_t::String:
        break;
    }
  }
}

namespace {
void read_table(const toml::table& table, const std::string& prefix, ParametersMap& output) {
  using scalar_type_t = Parameters::scalar_type_t;
  for (const auto& [name, node] : table) {
    const auto key = prefix + std::string(name.str());
    const auto invalid = [&](const std::string& reason) {
      throw std::invalid_argument("line " + std::to_string(node.source().begin.line) + ": " + key + ": " + reason);
    };
    if (const auto* child = node.as_table()) {
      const auto& defaults = Parameters::getDefaultParameters();
      const auto first = defaults.lower_bound(key + '.');
      if (first == defaults.end() || first->first.compare(0, key.size() + 1, key + '.') != 0) invalid("unknown parameter table");
      read_table(*child, key + '.', output);
      continue;
    }
    const auto type = Parameters::getType(key);
    if (!type) invalid("unknown algorithm parameter");
    std::string value;
    switch (*type) {
      case scalar_type_t::Bool:
        if (!node.is_boolean()) invalid("requires a TOML boolean");
        value = node.as_boolean()->get() ? "true" : "false";
        break;
      case scalar_type_t::String:
        if (!node.is_string()) invalid("requires a TOML string");
        value = node.as_string()->get();
        break;
      case scalar_type_t::Int:
      case scalar_type_t::UnsignedInt:
        if (!node.is_integer()) invalid("requires a TOML integer");
        value = std::to_string(node.as_integer()->get());
        break;
      case scalar_type_t::Float:
      case scalar_type_t::Double:
        if (const auto* integer = node.as_integer())
          value = std::to_string(integer->get());
        else if (const auto* number = node.as_floating_point()) {
          std::ostringstream text;
          text.imbue(std::locale::classic());
          text << std::setprecision(std::numeric_limits<double>::max_digits10) << number->get();
          value = text.str();
        } else
          invalid("requires a TOML number");
        break;
    }
    if (!output.emplace(key, std::move(value)).second) invalid("duplicate parameter key");
  }
}
}  // namespace

ParametersMap LoadParameters::read(const std::string& path) {
  try {
    ParametersMap values;
    read_table(toml::parse_file(path), "", values);
    validate_modules(values);
    return values;
  } catch (const toml::parse_error& error) {
    std::ostringstream message;
    message << error;
    throw std::invalid_argument(path + ": " + message.str());
  } catch (const std::exception& error) {
    throw std::invalid_argument(path + ": " + error.what());
  }
}

ParametersMap LoadParameters::from_map(const ParametersMap& values) {
  auto result = Parameters::getDefaultParameters();
  for (const auto& [key, value] : values) result.insert_or_assign(key, value);
  // Validate typed module constraints without converting numbers back to text.
  // Keep the exact input representation for diagnostics and database records.
  validate_modules(result);
  return result;
}

ParametersMap LoadParameters::from_file(const std::string& path, const ParametersMap& runtime) {
  auto values = read(path);
  for (const auto& [key, value] : runtime) {
    if (!Parameters::getDefaultParameters().count(key)) throw std::invalid_argument("Unknown runtime-owned algorithm parameter: " + key);
    if (values.count(key)) throw std::invalid_argument(path + ": " + key + " is owned by runtime/input and must not appear in algorithm tuning");
    values.emplace(key, value);
  }
  return from_map(values);
}
}  // namespace sapphire::backend
