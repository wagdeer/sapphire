#include "database/sqlite.hpp"
#include "database/statistics.hpp"
#include "database/memory.hpp"

#include <cassert>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iterator>

using namespace sapphire;
using namespace sapphire;

int main() {
  const std::map<std::string, float> values{{"Relocalization/Id/", 7}, {"Timing/Total/ms", 1.25f}};
  assert(Statistics::deserializeData(Statistics::serializeData(values)) == values);
  std::vector<std::future<void>> workers;
  for (int i = 0; i < 4; ++i) workers.push_back(std::async(std::launch::async, [&, i] {
    for (int j = 0; j < 100; ++j) {
      Statistics record;
      assert(record.data().empty() && record.refNodeId() == 0);
      record.setRefNodeId(i + 1);
      record.addStatistic("Relocalization/Id/", float(j));
      assert(record.data().at("Relocalization/Id/") == j && record.refNodeId() == i + 1);
    }
  }));
  for (auto& worker : workers) worker.get();

  auto pattern = (std::filesystem::temp_directory_path() / "sapphire-statistics-XXXXXX").string();
  const char* created = mkdtemp(pattern.data());
  assert(created);
  const std::filesystem::path directory(created);
  const auto path = (directory / "map.db").string();
  {
    Memory memory;
    assert(memory.init(path));
    auto cloud = std::make_shared<GaussianCloud>(1);
    cloud->front().N = 20;
    cloud->front().radius = .3f;
    cloud->front().mean = {1, 2, 3};
    cloud->front().covariance = .01f * Eigen::Matrix3f::Identity();
    for (int id = 1; id <= 2; ++id) {
      assert(memory.admit_node(std::make_unique<Node>(cloud, id, SE3f::fromTranslation(Eigen::Vector3f(id, 0, 0))), Mat66_t<double>::Identity()));
      Statistics record;
      record.setRefNodeId(id);
      record.setStamp(id + .125);
      for (const auto& [key, value] : values) record.addStatistic(key, value);
      record.setWmState({999}); // Memory must capture actual resident IDs at admission.
      assert(memory.save_keyframe(id, {}, &record, true));
      assert(record.wmState() == std::vector<int>{999}); // caller record remains independent
    }
    memory.joinTrashThread();
    memory.close();
  }
  {
    DBDriverSqlite3 database;
    assert(database.openConnection(path));
    for (int id = 1; id <= 2; ++id) {
      double stamp = 0;
      std::vector<int> residents;
      assert(database.getStatistics(id, stamp, &residents) == values);
      assert(stamp == id + .125);
      assert(residents == (id == 1 ? std::vector<int>{1} : std::vector<int>{1, 2}));
    }
    database.closeConnection(false);
  }
  {
    StatisticsLog log;
    StatisticsParams parameters;
    parameters.log_enabled = true;
    parameters.buffer_logs = false;
    log.open(directory.string(), parameters, true);
    Statistics record;
    record.setRefNodeId(7);
    record.setStamp(1.25);
    record.addStatistic("Timing/Total/ms", 2.5f);
    log.write(record);
    log.close();
    std::ifstream input(directory / "statistics.tsv");
    const std::string text((std::istreambuf_iterator<char>(input)), {});
    assert(text == "stamp\tnode\tmetric\tvalue\n1.25\t7\tTiming/Total/ms\t2.5\n");
  }
  std::filesystem::remove_all(directory);
}
