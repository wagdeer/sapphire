#include <spdlog/logger.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <backend/tools/logging.hpp>
#include <mutex>
#include <vector>

namespace sapphire::backend::tools {
namespace {

std::string make_pattern(const LogPatternConfig& config) {
  std::string pattern;
  if (config.timestamp) {
    pattern += "%Y-%m-%d %H:%M:%S.%e ";
  }
  pattern += "[%^%l%$] ";
  if (config.thread_id) {
    pattern += "[%t] ";
  }
  if (config.source) {
    pattern += "[%s:%# %!] ";
  }
  pattern += "%v";
  return pattern;
}

spdlog::level::level_enum to_spdlog_level(LogLevel level) noexcept {
  switch (level) {
    case LogLevel::Debug:
      return spdlog::level::debug;
    case LogLevel::Info:
      return spdlog::level::info;
    case LogLevel::Warn:
      return spdlog::level::warn;
    case LogLevel::Error:
      return spdlog::level::err;
    case LogLevel::Fatal:
      return spdlog::level::critical;
    case LogLevel::Off:
      return spdlog::level::off;
  }
  return spdlog::level::off;
}

LogLevel from_spdlog_level(spdlog::level::level_enum level) noexcept {
  switch (level) {
    case spdlog::level::trace:
    case spdlog::level::debug:
      return LogLevel::Debug;
    case spdlog::level::info:
      return LogLevel::Info;
    case spdlog::level::warn:
      return LogLevel::Warn;
    case spdlog::level::err:
      return LogLevel::Error;
    case spdlog::level::critical:
      return LogLevel::Fatal;
    case spdlog::level::off:
    case spdlog::level::n_levels:
      return LogLevel::Off;
  }
  return LogLevel::Off;
}

bool passes(LogLevel level, LogLevel minimum) noexcept { return minimum != LogLevel::Off && static_cast<int>(level) >= static_cast<int>(minimum); }

struct CallbackEntry {
  CallbackEntry(LogLevel minimum_level, LogCallback callback_value) : minimum(minimum_level), callback(std::move(callback_value)) {}

  std::mutex mutex;
  std::condition_variable idle;
  bool active = true;
  std::size_t in_flight = 0;
  LogLevel minimum;
  LogCallback callback;
};

thread_local CallbackEntry* current_callback = nullptr;
thread_local bool dispatching_callbacks = false;

class CallbackRegistry {
 public:
  CallbackRegistry() : snapshot_(std::make_shared<const EntryList>()) {}

  std::shared_ptr<CallbackEntry> add(LogLevel minimum, LogCallback callback) {
    auto entry = std::make_shared<CallbackEntry>(minimum, std::move(callback));
    std::lock_guard<std::mutex> lock(mutex_);
    auto next = std::make_shared<EntryList>(*std::atomic_load_explicit(&snapshot_, std::memory_order_acquire));
    next->push_back(entry);
    std::atomic_store_explicit(&snapshot_, std::const_pointer_cast<const EntryList>(next), std::memory_order_release);
    return entry;
  }

  void remove(const std::shared_ptr<CallbackEntry>& entry) noexcept {
    if (!entry) {
      return;
    }

    {
      std::lock_guard<std::mutex> registry_lock(mutex_);
      {
        std::lock_guard<std::mutex> entry_lock(entry->mutex);
        entry->active = false;
      }

      const auto current = std::atomic_load_explicit(&snapshot_, std::memory_order_acquire);
      try {
        auto next = std::make_shared<EntryList>();
        next->reserve(current->size());
        std::copy_if(current->begin(), current->end(), std::back_inserter(*next),
                     [&entry](const std::shared_ptr<CallbackEntry>& candidate) { return candidate != entry; });
        std::atomic_store_explicit(&snapshot_, std::const_pointer_cast<const EntryList>(next), std::memory_order_release);
      } catch (...) {
        // The inactive flag still prevents future invocation. Keeping the
        // old snapshot is safe and avoids throwing from RAII destruction.
      }
    }

    if (current_callback == entry.get()) {
      return;
    }

    std::unique_lock<std::mutex> lock(entry->mutex);
    entry->idle.wait(lock, [&entry] { return entry->in_flight == 0; });
  }

  void dispatch(const LogRecordView& record) noexcept {
    if (dispatching_callbacks) {
      return;
    }

    struct DispatchGuard {
      DispatchGuard() { dispatching_callbacks = true; }
      ~DispatchGuard() {
        current_callback = nullptr;
        dispatching_callbacks = false;
      }
    } guard;

    const auto entries = std::atomic_load_explicit(&snapshot_, std::memory_order_acquire);
    for (const auto& entry : *entries) {
      {
        std::lock_guard<std::mutex> lock(entry->mutex);
        if (!entry->active || !passes(record.level, entry->minimum)) {
          continue;
        }
        ++entry->in_flight;
      }

      current_callback = entry.get();
      try {
        entry->callback(record);
      } catch (...) {
        // A sink callback is observational and cannot break logging.
      }
      current_callback = nullptr;

      {
        std::lock_guard<std::mutex> lock(entry->mutex);
        --entry->in_flight;
        if (!entry->active && entry->in_flight == 0) {
          entry->idle.notify_all();
        }
      }
    }
  }

 private:
  using EntryList = std::vector<std::shared_ptr<CallbackEntry>>;

  std::mutex mutex_;
  std::shared_ptr<const EntryList> snapshot_;
};

CallbackRegistry& callback_registry() {
  static CallbackRegistry* registry = new CallbackRegistry;
  return *registry;
}

class CallbackSink final : public spdlog::sinks::sink {
 public:
  void log(const spdlog::details::log_msg& message) override {
    callback_registry().dispatch({from_spdlog_level(message.level),
                                  std::string_view(message.payload.data(), message.payload.size()),
                                  {message.source.filename, message.source.line, message.source.funcname}});
  }

  void flush() override {}
  void set_pattern(const std::string&) override {}
  void set_formatter(std::unique_ptr<spdlog::formatter>) override {}
};

struct LoggingState {
  LoggingState() {
    config.console.enabled = false;
    callback_sink = std::make_shared<CallbackSink>();
    auto initial = std::make_shared<spdlog::logger>("exterior", spdlog::sinks_init_list{callback_sink});
    initial->set_level(spdlog::level::trace);
    logger = std::move(initial);
  }

  std::mutex configure_mutex;
  std::atomic<LogLevel> minimum{LogLevel::Info};
  LoggingConfig config;
  std::shared_ptr<CallbackSink> callback_sink;
  std::shared_ptr<spdlog::logger> logger;
};

LoggingState& logging_state() {
  static LoggingState* state = new LoggingState;
  return *state;
}

void set_error(std::string* destination, std::string message) noexcept {
  if (!destination) {
    return;
  }
  try {
    *destination = std::move(message);
  } catch (...) {
  }
}

}  // namespace

class CallbackRegistration::Impl {
 public:
  explicit Impl(std::shared_ptr<CallbackEntry> entry_value) : entry(std::move(entry_value)) {}

  ~Impl() { callback_registry().remove(entry); }

  std::shared_ptr<CallbackEntry> entry;
};

bool configure(const LoggingConfig& config, std::string* error) noexcept {
  try {
    auto& state = logging_state();
    std::lock_guard<std::mutex> lock(state.configure_mutex);

    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(state.callback_sink);
    const std::string pattern = make_pattern(config.pattern);

    if (config.console.enabled) {
      spdlog::sink_ptr console;
      if (config.console.colored) {
        console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      } else {
        console = std::make_shared<spdlog::sinks::stdout_sink_mt>();
      }
      console->set_pattern(pattern);
      sinks.push_back(std::move(console));
    }

    if (config.file) {
      if (config.file->path.empty()) {
        set_error(error, "log file path is empty");
        return false;
      }
      auto file = std::make_shared<spdlog::sinks::basic_file_sink_mt>(config.file->path.string(), config.file->truncate);
      file->set_pattern(pattern);
      sinks.push_back(std::move(file));
    }

    auto logger = std::make_shared<spdlog::logger>("exterior", sinks.begin(), sinks.end());
    logger->set_level(spdlog::level::trace);
    logger->set_error_handler([](const std::string&) {});

    state.config = config;
    state.minimum.store(config.level, std::memory_order_release);
    std::atomic_store_explicit(&state.logger, std::move(logger), std::memory_order_release);
    if (error) {
      error->clear();
    }
    return true;
  } catch (const std::exception& exception) {
    set_error(error, exception.what());
  } catch (...) {
    set_error(error, "unknown spdlog configuration error");
  }
  return false;
}

LoggingConfig configuration() {
  auto& state = logging_state();
  std::lock_guard<std::mutex> lock(state.configure_mutex);
  return state.config;
}

void set_level(LogLevel level) noexcept {
  auto& state = logging_state();
  std::lock_guard<std::mutex> lock(state.configure_mutex);
  state.config.level = level;
  state.minimum.store(level, std::memory_order_release);
}

LogLevel level() noexcept { return logging_state().minimum.load(std::memory_order_acquire); }

bool should_log(LogLevel message_level) noexcept {
  if (message_level == LogLevel::Fatal) {
    return true;
  }
  return passes(message_level, level());
}

void flush() noexcept {
  try {
    const auto logger = std::atomic_load_explicit(&logging_state().logger, std::memory_order_acquire);
    if (logger) {
      logger->flush();
    }
  } catch (...) {
  }
}

CallbackRegistration::CallbackRegistration() noexcept = default;
CallbackRegistration::~CallbackRegistration() = default;

CallbackRegistration::CallbackRegistration(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}

CallbackRegistration::CallbackRegistration(CallbackRegistration&& other) noexcept = default;

CallbackRegistration& CallbackRegistration::operator=(CallbackRegistration&& other) noexcept = default;

CallbackRegistration::operator bool() const noexcept { return static_cast<bool>(impl_); }

void CallbackRegistration::reset() noexcept { impl_.reset(); }

CallbackRegistration register_callback(LogLevel minimum_level, LogCallback callback) {
  if (!callback) {
    throw std::invalid_argument("log callback is empty");
  }
  return CallbackRegistration(std::make_unique<CallbackRegistration::Impl>(callback_registry().add(minimum_level, std::move(callback))));
}

namespace detail {

void write_log(LogLevel message_level, SourceLocation source, std::string_view message) noexcept {
  if (message_level != LogLevel::Fatal && !should_log(message_level)) {
    return;
  }

  try {
    const auto logger = std::atomic_load_explicit(&logging_state().logger, std::memory_order_acquire);
    if (!logger) {
      return;
    }

    logger->log(spdlog::source_loc(source.file, source.line, source.function), to_spdlog_level(message_level),
                spdlog::string_view_t(message.data(), message.size()));
    if (message_level == LogLevel::Fatal) {
      logger->flush();
    }
  } catch (...) {
    if (message_level == LogLevel::Fatal) {
      std::fwrite(message.data(), 1, message.size(), stderr);
      std::fputc('\n', stderr);
      std::fflush(stderr);
    }
  }
}

}  // namespace detail
}  // namespace sapphire::backend::tools
