#include <backend/database/statistics.hpp>
#include <backend/tools/strings.hpp>
#include <backend/tools/filesystem.hpp>
#include <filesystem>
#include <iomanip>

namespace sapphire::backend {
std::map<std::string, float> Statistics::_defaultData;
bool Statistics::_defaultDataInitialized = false;

const std::map<std::string, float>& Statistics::defaultData() {
  Statistics stat;
  return _defaultData;
}

std::string Statistics::serializeData(const std::map<std::string, float>& data) {
  std::stringstream output;
  for (std::map<std::string, float>::const_iterator iter = data.begin(); iter != data.end(); ++iter) {
    if (iter != data.begin()) {
      output << ";";
    }
    output << iter->first << ":" << tools::format_float(iter->second);
  }
  return output.str();
}

std::map<std::string, float> Statistics::deserializeData(const std::string& data) {
  std::map<std::string, float> output;
  std::vector<std::string> tuplets = sapphire::backend::tools::split(data, ';');
  for (auto iter = tuplets.begin(); iter != tuplets.end(); ++iter) {
    std::vector<std::string> p = sapphire::backend::tools::split(*iter, ':');
    if (p.size() == 2) {
      std::string key = p.front();
      std::string value = p.back();
      (output).insert_or_assign(key, tools::parse_float_or(value));
    }
  }
  return output;
}

Statistics::Statistics()
    : _extended(0),
      _refImageId(0),
      _refImageMapId(-1),
      _loopClosureId(0),
      _loopClosureMapId(-1),
      _stamp(0.0f) {
  _defaultDataInitialized = true;
}

Statistics::~Statistics() {}

// name format = "Grp/Name/unit"
void Statistics::addStatistic(const std::string& name, float value) { (_data).insert_or_assign(name, value); }

void StatisticsLog::open(const std::string& directory, const StatisticsParams& params, bool overwrite) {
  close();
  if (!params.log_enabled) return;
  if (directory.empty()) throw std::invalid_argument("Statistics logging requires a working directory");
  auto path = directory;
  if (path.front() == '~') path = tools::home_directory().string() + path.substr(1);
  const auto file = std::filesystem::path(path) / "statistics.tsv";
  const bool header = params.log_headers && (overwrite || !std::filesystem::exists(file) || std::filesystem::file_size(file) == 0);
  stream_.clear();
  stream_.exceptions(std::ios::failbit | std::ios::badbit);
  stream_.open(file, overwrite ? std::ios::out | std::ios::trunc : std::ios::out | std::ios::app);
  stream_ << std::setprecision(17);
  flush_each_record_ = !params.buffer_logs;
  if (header) stream_ << "stamp\tnode\tmetric\tvalue\n";
}
void StatisticsLog::write(const Statistics& statistics) {
  if (!stream_.is_open()) return;
  // std::filebuf has bounded buffering; never accumulate a session's records in RAM.
  for (const auto& [key, value] : statistics.data())
    stream_ << statistics.stamp() << '\t' << statistics.refImageId() << '\t' << key << '\t' << value << '\n';
  if (flush_each_record_) stream_.flush();
}
void StatisticsLog::close() {
  if (!stream_.is_open()) return;
  stream_.close();
}

}  // namespace sapphire::backend
