#pragma once

#include <spdlog/fmt/fmt.h>

#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace sapphire::backend::tools {

enum class LogLevel {
  Debug,
  Info,
  Warn,
  Error,
  Fatal,
  Off,
};

struct SourceLocation {
  const char* file;
  int line;
  const char* function;
};

// The string views are valid only for the duration of the callback.
struct LogRecordView {
  LogLevel level;
  std::string_view message;
  SourceLocation source;
};

struct ConsoleSinkConfig {
  bool enabled = true;
  bool colored = true;
};

struct FileSinkConfig {
  std::filesystem::path path;
  bool truncate = false;
};

struct LogPatternConfig {
  bool timestamp = true;
  bool source = true;
  bool thread_id = false;
};

struct LoggingConfig {
  // The threshold applies to every configured sink, including callbacks.
  // Fatal is always processed so it can flush and throw.
  LogLevel level = LogLevel::Info;
  ConsoleSinkConfig console;
  // Console and file may be enabled together. The file's parent must exist.
  std::optional<FileSinkConfig> file;
  // The level is always printed.
  LogPatternConfig pattern;
};

bool configure(const LoggingConfig& config, std::string* error = nullptr) noexcept;
LoggingConfig configuration();

void set_level(LogLevel level) noexcept;
LogLevel level() noexcept;
bool should_log(LogLevel level) noexcept;
void flush() noexcept;

using LogCallback = std::function<void(const LogRecordView&)>;

class CallbackRegistration final {
 public:
  CallbackRegistration() noexcept;
  ~CallbackRegistration();

  CallbackRegistration(CallbackRegistration&& other) noexcept;
  CallbackRegistration& operator=(CallbackRegistration&& other) noexcept;

  CallbackRegistration(const CallbackRegistration&) = delete;
  CallbackRegistration& operator=(const CallbackRegistration&) = delete;

  explicit operator bool() const noexcept;
  // Prevents future calls and waits for callbacks already in flight. Calling
  // reset() from this registration's own callback only prevents future calls;
  // the current invocation already owns its record view.
  void reset() noexcept;

 private:
  class Impl;
  explicit CallbackRegistration(std::unique_ptr<Impl> impl) noexcept;

  std::unique_ptr<Impl> impl_;

  friend CallbackRegistration register_callback(LogLevel, LogCallback);
};

CallbackRegistration register_callback(LogLevel minimum_level, LogCallback callback);

namespace detail {

void write_log(LogLevel level, SourceLocation source, std::string_view message) noexcept;

template <typename... Args>
void log(LogLevel level, SourceLocation source, fmt::format_string<Args...> format, Args&&... args) noexcept {
  try {
    write_log(level, source, fmt::format(format, std::forward<Args>(args)...));
  } catch (...) {
    // Logging must never break the caller's control flow.
  }
}

template <typename... Args>
[[noreturn]] void fatal(SourceLocation source, fmt::format_string<Args...> format, Args&&... args) {
  std::string message;
  try {
    message = fmt::format(format, std::forward<Args>(args)...);
  } catch (const std::exception& exception) {
    message = std::string("fatal log formatting failed: ") + exception.what();
  } catch (...) {
    message = "fatal log formatting failed";
  }

  write_log(LogLevel::Fatal, source, message);
  flush();
  throw std::runtime_error(message);
}

}  // namespace detail

}  // namespace sapphire::backend::tools

#define EXT_LOG_DETAIL(level_value, ...)                                                        \
  do {                                                                                          \
    if (::sapphire::backend::tools::should_log(level_value)) {                                           \
      ::sapphire::backend::tools::detail::log(level_value, {__FILE__, __LINE__, __func__}, __VA_ARGS__); \
    }                                                                                           \
  } while (false)

#define EXT_LOG_DEBUG(...) EXT_LOG_DETAIL(::sapphire::backend::tools::LogLevel::Debug, __VA_ARGS__)
#define EXT_LOG_INFO(...) EXT_LOG_DETAIL(::sapphire::backend::tools::LogLevel::Info, __VA_ARGS__)
#define EXT_LOG_WARN(...) EXT_LOG_DETAIL(::sapphire::backend::tools::LogLevel::Warn, __VA_ARGS__)
#define EXT_LOG_ERROR(...) EXT_LOG_DETAIL(::sapphire::backend::tools::LogLevel::Error, __VA_ARGS__)
#define EXT_LOG_FATAL(...)                                                         \
  do {                                                                             \
    ::sapphire::backend::tools::detail::fatal({__FILE__, __LINE__, __func__}, __VA_ARGS__); \
  } while (false)

#define EXT_ASSERT(condition)                                                                               \
  do {                                                                                                      \
    if (!(condition)) {                                                                                     \
      ::sapphire::backend::tools::detail::fatal({__FILE__, __LINE__, __func__}, "Assertion failed: {}", #condition); \
    }                                                                                                       \
  } while (false)

#define EXT_ASSERT_MSG(condition, ...)                                               \
  do {                                                                               \
    if (!(condition)) {                                                              \
      ::sapphire::backend::tools::detail::fatal({__FILE__, __LINE__, __func__}, __VA_ARGS__); \
    }                                                                                \
  } while (false)
