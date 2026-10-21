#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace sapphire::backend::tools {

namespace fs = std::filesystem;

// 纯词法辅助函数；这些函数不访问文件系统。
std::string filename(const fs::path& path);
std::string extension(const fs::path& path);

bool path_exists(const fs::path& path) noexcept;
bool path_exists(const fs::path& path, std::error_code& error) noexcept;

bool directory_exists(const fs::path& path) noexcept;
bool directory_exists(const fs::path& path, std::error_code& error) noexcept;

std::uintmax_t file_size(const fs::path& path);
std::uintmax_t file_size(const fs::path& path, std::error_code& error) noexcept;

bool remove_file(const fs::path& path);
bool remove_file(const fs::path& path, std::error_code& error) noexcept;

void rename_file(const fs::path& from, const fs::path& to);
void rename_file(const fs::path& from, const fs::path& to, std::error_code& error) noexcept;

fs::path current_directory();
fs::path current_directory(std::error_code& error) noexcept;

fs::path home_directory();
fs::path home_directory(std::error_code& error) noexcept;

// 数字段先按数值比较，再按前导零的宽度比较。比较与区域设置无关，不进行内存分配或文件系统访问。
struct NaturalLess {
  bool operator()(std::string_view left, std::string_view right) const noexcept;
};

enum class DirectorySort {
  none,
  lexical,
  natural,
};

struct DirectoryOptions {
  // 扩展名省略前导点，并且区分大小写匹配。
  std::vector<std::string> extensions;
  bool include_hidden = true;
  bool include_files = true;
  bool include_directories = true;
  DirectorySort sort = DirectorySort::natural;
};

struct DirectoryEntry {
  fs::path path;
  std::string name;
  fs::file_type type = fs::file_type::none;

  bool is_regular_file() const noexcept { return type == fs::file_type::regular; }
  bool is_directory() const noexcept { return type == fs::file_type::directory; }
};

std::vector<DirectoryEntry> list_directory(const fs::path& directory, const DirectoryOptions& options = {});
std::vector<DirectoryEntry> list_directory(const fs::path& directory, const DirectoryOptions& options, std::error_code& error) noexcept;

}  // namespace sapphire::backend::tools
