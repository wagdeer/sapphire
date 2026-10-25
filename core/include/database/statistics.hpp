#pragma once

#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace sapphire {
struct StatisticsParams;

// One observation's diagnostic record. Graph and cloud ownership stays in Topograph.
class Statistics {
 public:
  static std::string serializeData(const std::map<std::string, float>& data);
  static std::map<std::string, float> deserializeData(const std::string& data);

  void addStatistic(const std::string& name, float value) { data_.insert_or_assign(name, value); }
  void setRefNodeId(int id) { node_id_ = id; }
  void setStamp(double stamp) { stamp_ = stamp; }
  void setWmState(const std::vector<int>& ids) { resident_ids_ = ids; }

  int refNodeId() const { return node_id_; }
  double stamp() const { return stamp_; }
  const std::vector<int>& wmState() const { return resident_ids_; }
  const std::map<std::string, float>& data() const { return data_; }

 private:
  int node_id_ = 0;
  double stamp_ = 0;
  std::vector<int> resident_ids_;
  std::map<std::string, float> data_;
};

class StatisticsLog final {
 public:
  void open(const std::string& directory, const StatisticsParams&, bool overwrite);
  void write(const Statistics&);
  void close();

 private:
  std::ofstream stream_;
  bool flush_each_record_ = false;
};

}  // namespace sapphire
