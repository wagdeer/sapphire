#include <pwd.h>
#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <backend/tools/filesystem.hpp>
#include <system_error>

namespace sapphire::backend::tools {
namespace {

bool is_digit(char value) noexcept { return value >= '0' && value <= '9'; }

bool is_hidden(std::string_view name) noexcept { return !name.empty() && name.front() == '.'; }

std::string normalized_extension(std::string extension_value) {
  if (!extension_value.empty() && extension_value.front() == '.') {
    extension_value.erase(extension_value.begin());
  }
  return extension_value;
}

bool extension_matches(const fs::path& path, const std::vector<std::string>& extensions) {
  if (extensions.empty()) {
    return true;
  }
  const std::string path_extension = extension(path);
  return std::find(extensions.begin(), extensions.end(), path_extension) != extensions.end();
}

}  // namespace

std::string filename(const fs::path& path) { return path.filename().string(); }

std::string extension(const fs::path& path) { return normalized_extension(path.extension().string()); }

bool path_exists(const fs::path& path) noexcept {
  std::error_code error;
  return path_exists(path, error);
}

bool path_exists(const fs::path& path, std::error_code& error) noexcept {
  error.clear();
  const bool exists = fs::exists(path, error);
  return !error && exists;
}

bool directory_exists(const fs::path& path) noexcept {
  std::error_code error;
  return directory_exists(path, error);
}

bool directory_exists(const fs::path& path, std::error_code& error) noexcept {
  error.clear();
  const bool exists = fs::is_directory(path, error);
  return !error && exists;
}

std::uintmax_t file_size(const fs::path& path) { return fs::file_size(path); }

std::uintmax_t file_size(const fs::path& path, std::error_code& error) noexcept {
  error.clear();
  return fs::file_size(path, error);
}

bool remove_file(const fs::path& path) { return fs::remove(path); }

bool remove_file(const fs::path& path, std::error_code& error) noexcept {
  error.clear();
  return fs::remove(path, error);
}

void rename_file(const fs::path& from, const fs::path& to) { fs::rename(from, to); }

void rename_file(const fs::path& from, const fs::path& to, std::error_code& error) noexcept {
  error.clear();
  fs::rename(from, to, error);
}

fs::path current_directory() { return fs::current_path(); }

fs::path current_directory(std::error_code& error) noexcept {
  error.clear();
  return fs::current_path(error);
}

fs::path home_directory() {
  std::error_code error;
  fs::path result = home_directory(error);
  if (error) {
    throw std::system_error(error, "cannot determine home directory");
  }
  return result;
}

fs::path home_directory(std::error_code& error) noexcept {
  error.clear();
  if (const char* home = std::getenv("HOME"); home != nullptr && *home != '\0') {
    return fs::path(home);
  }

  const long suggested_size = sysconf(_SC_GETPW_R_SIZE_MAX);
  const std::size_t buffer_size = suggested_size > 0 ? static_cast<std::size_t>(suggested_size) : 16384U;
  std::vector<char> buffer(buffer_size);
  passwd entry{};
  passwd* result = nullptr;
  const int status = getpwuid_r(getuid(), &entry, buffer.data(), buffer.size(), &result);
  if (status != 0) {
    error = std::error_code(status, std::generic_category());
    return {};
  }
  if (result == nullptr || entry.pw_dir == nullptr || *entry.pw_dir == '\0') {
    error = std::make_error_code(std::errc::no_such_file_or_directory);
    return {};
  }
  return fs::path(entry.pw_dir);
}

bool NaturalLess::operator()(std::string_view left, std::string_view right) const noexcept {
  std::size_t left_index = 0;
  std::size_t right_index = 0;
  while (left_index < left.size() && right_index < right.size()) {
    const bool left_digit = is_digit(left[left_index]);
    const bool right_digit = is_digit(right[right_index]);
    if (left_digit && right_digit) {
      const std::size_t left_begin = left_index;
      const std::size_t right_begin = right_index;
      while (left_index < left.size() && is_digit(left[left_index])) {
        ++left_index;
      }
      while (right_index < right.size() && is_digit(right[right_index])) {
        ++right_index;
      }

      std::size_t left_significant = left_begin;
      std::size_t right_significant = right_begin;
      while (left_significant < left_index && left[left_significant] == '0') {
        ++left_significant;
      }
      while (right_significant < right_index && right[right_significant] == '0') {
        ++right_significant;
      }
      const std::size_t left_length = left_index - left_significant;
      const std::size_t right_length = right_index - right_significant;
      if (left_length != right_length) {
        return left_length < right_length;
      }
      const int number_order = left.substr(left_significant, left_length).compare(right.substr(right_significant, right_length));
      if (number_order != 0) {
        return number_order < 0;
      }
      const std::size_t left_run_length = left_index - left_begin;
      const std::size_t right_run_length = right_index - right_begin;
      if (left_run_length != right_run_length) {
        return left_run_length < right_run_length;
      }
      continue;
    }

    if (left[left_index] != right[right_index]) {
      return static_cast<unsigned char>(left[left_index]) < static_cast<unsigned char>(right[right_index]);
    }
    ++left_index;
    ++right_index;
  }
  return left.size() < right.size();
}

std::vector<DirectoryEntry> list_directory(const fs::path& directory, const DirectoryOptions& options) {
  std::error_code error;
  std::vector<DirectoryEntry> result = list_directory(directory, options, error);
  if (error) {
    throw std::system_error(error, "cannot list directory " + directory.string());
  }
  return result;
}

std::vector<DirectoryEntry> list_directory(const fs::path& directory, const DirectoryOptions& options, std::error_code& error) noexcept {
  error.clear();
  std::vector<DirectoryEntry> result;
  fs::directory_iterator iterator(directory, error);
  const fs::directory_iterator end;
  for (; !error && iterator != end; iterator.increment(error)) {
    const fs::directory_entry& native_entry = *iterator;
    const std::string name = native_entry.path().filename().string();
    if (!options.include_hidden && is_hidden(name)) {
      continue;
    }

    std::error_code metadata_error;
    const fs::file_status status = native_entry.status(metadata_error);
    if (metadata_error) {
      error = metadata_error;
      break;
    }
    const fs::file_type type = status.type();
    const bool is_file = type == fs::file_type::regular;
    const bool is_directory_entry = type == fs::file_type::directory;
    if ((!is_file && !is_directory_entry) || (is_file && !options.include_files) || (is_directory_entry && !options.include_directories) ||
        !extension_matches(native_entry.path(), options.extensions)) {
      continue;
    }
    result.push_back({native_entry.path(), name, type});
  }
  if (error) {
    result.clear();
    return result;
  }

  if (options.sort == DirectorySort::lexical) {
    std::sort(result.begin(), result.end(), [](const DirectoryEntry& left, const DirectoryEntry& right) { return left.name < right.name; });
  } else if (options.sort == DirectorySort::natural) {
    std::sort(result.begin(), result.end(), [](const DirectoryEntry& left, const DirectoryEntry& right) { return NaturalLess{}(left.name, right.name); });
  }
  return result;
}

}  // namespace sapphire::backend::tools
