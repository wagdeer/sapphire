#include <sqlite3.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <maplayer/topograph/node.hpp>
#include <database/sqlite.hpp>
#include <maplayer/topograph.hpp>
#include <solver/pgo_solver.hpp>
#include <database/memory.hpp>
#include <filesystem>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>

using namespace sapphire;
namespace {
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
template <class F>
void rejects(F action) {
  bool rejected = false;
  try {
    action();
  } catch (const std::exception&) {
    rejected = true;
  }
  require(rejected, "Unsupported type was accepted");
}
struct DatabaseFile {
  std::string path;
  DatabaseFile() {
    char name[] = "/tmp/exterior-link-XXXXXX";
    int fd = mkstemp(name);
    require(fd >= 0, "mkstemp failed");
    close(fd);
    path = name;
  }
  ~DatabaseFile() {
    for (const auto& suffix : {"", "-wal", "-shm"}) std::filesystem::remove(path + suffix);
  }
};
void sql(const std::string& path, const std::string& command) {
  sqlite3* raw = nullptr;
  require(sqlite3_open(path.c_str(), &raw) == SQLITE_OK, "SQL open failed");
  std::unique_ptr<sqlite3, decltype(&sqlite3_close)> db(raw, sqlite3_close);
  require(sqlite3_exec(db.get(), command.c_str(), nullptr, nullptr, nullptr) == SQLITE_OK, "SQL fixture update failed");
}


class HeldDriver : public DBDriverSqlite3 {
 public:
  ~HeldDriver() {
    release();
    emptyTrashes();
  }
  void wait() { require(entered.get_future().wait_for(std::chrono::seconds(5)) == std::future_status::ready, "Writer did not enter in time"); }
  void release() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      released = true;
    }
    changed.notify_all();
  }

 protected:
  void beforeAsyncBatchWrite() override {
    entered.set_value();
    std::unique_lock<std::mutex> lock(mutex);
    changed.wait(lock, [&] { return released; });
  }

 private:
  std::promise<void> entered;
  std::mutex mutex;
  std::condition_variable changed;
  bool released = false;
};
void memory_graph_boundary() {
  DatabaseFile file;
  MappingParameters config;
  config.topograph.use_odometry_gravity = true;
  config.topograph.ignore_odometry_cross_covariance = false;
  config.memory.recent_nodes = 1;
  const auto a = SE3f::fromRotationTranslation(Eigen::AngleAxisf(.29f, Eigen::Vector3f::UnitZ()).toRotationMatrix(), {1, -2, .3f});
  const auto b = SE3f::fromRotationTranslation(Eigen::AngleAxisf(-.21f, Eigen::Vector3f::UnitY()).toRotationMatrix(), {2, .4f, 1});
  Mat66_t<double> covariance = Mat66_t<double>::Identity();
  covariance(0, 4) = covariance(4, 0) = .2;
  int initial_map = 0;
  {
    Memory memory(config);
    require(memory.init(file.path, true), "Memory boundary fixture failed to open");
    initial_map = memory.current_map_id();
    auto* first = memory.admit_node(std::make_unique<Node>(0, -1, 0, 1, "", a), covariance);
    auto* second = memory.admit_node(std::make_unique<Node>(0, -1, 0, 2, "", b), covariance);
    require(first->getLabel() == "map" + std::to_string(initial_map) && second->getLabel().empty(), "Session label moved to the wrong observation");
    require(first->hasLink(first->id(), Link::kGravity) && second->hasLink(second->id(), Link::kGravity), "Graph admission lost gravity");
    const auto& forward = first->getLinks().find(second->id())->second;
    const auto& reverse = second->getLinks().find(first->id())->second;
    require(forward.isOdometry() && (forward.transform().matrix() - a.between(b).matrix()).norm() < 1e-5f &&
            forward.infMatrix().isApprox(covariance.inverse(), 1e-9), "Graph admission changed odometry direction or cross covariance");
    require(reverse.infMatrix().isApprox(forward.inverse().infMatrix(), 1e-9) &&
            (reverse.transform().matrix() - forward.inverse().transform().matrix()).norm() < 1e-5f, "Reverse odometry lost tangent transport");
    const int last = second->id();
    auto invalid = covariance;
    invalid(0, 0) = -1;
    rejects([&] { memory.admit_node(std::make_unique<Node>(0, -1, 0, 3, "", a), invalid); });
    require(memory.getAllNodeIds().size() == 2 && memory.getStMem().size() == 2 && second->getLinks().size() == 2 &&
                memory.topograph().current_odom_pose() == b, "Invalid covariance partially published an observation");
    require(memory.promote_recent_nodes() == std::vector<int>{first->id()} && memory.getStMem() == std::set<int>{last},
            "Storage promotion changed the recent-observation boundary");
    require(memory.incrementMapId() == initial_map + 1 && memory.incrementMapId() == initial_map + 1, "Empty session advanced identity twice");
    auto* next = memory.admit_node(std::make_unique<Node>(0, -1, 0, 4, "", a), covariance);
    require(next->mapId() == initial_map + 1 && next->getLabel() == "map" + std::to_string(initial_map + 1) && !next->hasLink(last) && next->getLinks().size() == 1,
            "New session synthesized cross-session odometry");
    memory.close();
  }
  {
    Memory restored(config);
    require(restored.init(file.path), "Memory boundary fixture failed to reopen");
    require(restored.topograph().current_odom_pose().isNull(), "Restored map established a live odometry session");
    const auto* previous = restored.getLastWorkingNode(false);
    require(previous != nullptr, "Restored working set missing");
    const int old_id = previous->id();
    auto* next = restored.admit_node(std::make_unique<Node>(0, -1, 0, 5, "", b), covariance);
    require(next->id() > old_id && !next->hasLink(old_id) && next->mapId() == initial_map + 2, "Restored admission changed session/ID semantics");
    restored.close();
  }
  DatabaseFile reducible;
  config.topograph.use_odometry_gravity = false;
  config.topograph.automatic_map_labels = false;
  config.topograph.reduce_graph = true;
  {
    Memory storage(config);
    require(storage.init(reducible.path, true), "Reduction fixture failed to open");
    for (int id = 1; id <= 4; ++id)
      require(storage.admit_node(std::make_unique<Node>(0, -1, 0, id, "", SE3f::fromTranslation({float(id), 0, 0})), covariance),
              "Reduction fixture admission failed");
    require(storage.addLink(Link(3, 1, Link::kRelocalization, SE3f::fromTranslation({-2, 0, 0}))), "Reduction fixture loop missing");
    require(storage.promote_recent_nodes() == std::vector<int>({1, 2, 3}) && storage.getNode(3)->hasLink(4),
            "Storage promotion secretly contracted the graph");
    // Leave a recent node at close too. A storage lifecycle must not decide reduction.
    storage.close();
  }
  {
    Memory restored(config);
    require(restored.init(reducible.path), "Reduction fixture failed to reopen");
    require(restored.getAllNodeIds().size() == 4 && restored.getLinks(3, true).count(4), "Storage close changed graph topology");
    require(restored.topograph().reduceNode(3, 0, 0) == 1, "Fixture was not reducible by an explicit graph operation");
    restored.deleteLocation(3, true);
    require(!restored.getNode(3) && restored.getNode(1)->hasLink(4), "Explicit graph reduction and storage retirement disagreed");
    restored.close(false);
  }
}

void pose_graph_contract() {
  const auto a = SE3f::fromRotationTranslation(
      (Eigen::AngleAxisf(.23f, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-.17f, Eigen::Vector3f::UnitX())).toRotationMatrix(), {1, 2, -.4f});
  const auto b = SE3f::fromRotationTranslation(
      (Eigen::AngleAxisf(-.31f, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(.41f, Eigen::Vector3f::UnitZ())).toRotationMatrix(), {2, -.8f, 1.3f});
  const auto c = SE3f::fromRotationTranslation(
      Eigen::AngleAxisf(.19f, Eigen::Vector3f::UnitX()).toRotationMatrix(), {-1, 3, 2});
  const auto information = Mat66_t<double>::Identity() * 100;
  const Link ab(1, 2, Link::kOdometry, a.between(b), information);
  const Link bc(2, 3, Link::kOdometry, b.between(c), information);
  const Link ac(1, 3, Link::kRelocalization, a.between(c), information);
  const auto roundtrip = ab.inverse().inverse();
  require(roundtrip.transform().translationDistance(ab.transform()) < 1e-5f &&
          roundtrip.infMatrix().isApprox(ab.infMatrix(), 1e-5), "Inverse transport changed SE3 factor");
  for (bool incremental : {false, true}) {
    PGOParams params; params.incremental_enabled = incremental; params.max_iterations = 20;
    solver::PoseGraphSolver solver(params); solver.fixed_root = true;
    std::map<int, SE3f> poses{{1, a}, {2, a}, {3, a}};
    std::multimap<int, Link> links{{1, ab}, {2, bc}, {3, ac.inverse()}};
    std::optional<Mat66_t<double>> covariance;
    require(solver.optimizeInPlace(1, poses, links, covariance), "SE3 graph optimization failed");
    require(poses.at(2).translationDistance(b) < 1e-4f && poses.at(3).translationDistance(c) < 1e-4f &&
            poses.at(2).rotationDistance(b) < 1e-3f && poses.at(3).rotationDistance(c) < 1e-3f,
            "Graph optimization inverted a measurement or changed coordinates");
    require(covariance && covariance->allFinite(), "Graph covariance missing");
    if (incremental) {
      // Unchanged factors must read the existing iSAM2 estimate, not new guesses.
      poses.at(2) = a;
      poses.at(3) = a;
      require(solver.optimizeInPlace(1, poses, links, covariance) && poses.at(3).translationDistance(c) < 1e-4f,
              "Unchanged graph accepted caller guesses instead of its numerical estimate");
      solver.swap_results();
      const auto shifted = SE3f::fromTranslation({.7f, -.3f, .2f}).compose(c);
      auto replacement = links;
      replacement.erase(2);
      replacement.erase(3);
      replacement.emplace(2, Link(2, 3, Link::kOdometry, b.between(shifted), information));
      {
        solver::PoseGraphSolver::Trial trial(solver);
        auto trial_poses = poses;
        require(solver.optimizeInPlace(1, trial_poses, replacement, covariance) &&
                    trial_poses.at(3).translationDistance(shifted) < 1e-4f,
                "Trial did not replace the numerical factor");
      }
      require(solver.swap_results().empty(), "Rejected trial published pose changes");
      require(solver.optimizeInPlace(1, poses, links, covariance) && poses.at(3).translationDistance(c) < 1e-4f,
              "Rejected trial contaminated the committed numerical graph");
      {
        solver::PoseGraphSolver::Trial trial(solver);
        require(solver.optimizeInPlace(1, poses, replacement, covariance), "Accepted trial failed");
        trial.commit();
      }
      require(solver.swap_results().count(3), "Committed trial omitted its pose changes");
      poses.at(3) = a;
      require(solver.optimizeInPlace(1, poses, replacement, covariance) && poses.at(3).translationDistance(shifted) < 1e-4f,
              "Accepted trial was not retained by iSAM2");
      links.erase(3);
      require(solver.optimizeInPlace(1, poses, links, covariance) && poses.at(3).translationDistance(c) < 1e-4f,
              "Incremental graph retained a removed constraint incorrectly");
      poses = {{1, a}};
      require(solver.optimizeInPlace(1, poses, {}, covariance), "Root-only graph removal failed");
      poses = {{101, a}, {102, c}};
      links = {{101, Link(101, 102, Link::kOdometry, a.between(b), information)}};
      require(solver.optimizeInPlace(101, poses, links, covariance) && poses.at(102).translationDistance(b) < 1e-4f,
              "Disjoint session retained stale factors or estimates");
    }
  }
}
void types() {
  require(!Link().isValid(), "Default link should be invalid");
  const Link::Type valid[] = {Link::kOdometry, Link::kRelocalization, Link::kPosePrior, Link::kGravity};
  const int ids[] = {0, 1, 7, 9};
  for (size_t i = 0; i < 4; ++i) require(int(valid[i]) == ids[i], "Persisted type ID changed");
  Link edge(1, 2, Link::kOdometry, SE3f::identity());
  for (int id : {-1, 2, 3, 4, 5, 6, 8, 10, 97, 98, 99, 10000}) {
    const auto type = static_cast<Link::Type>(id);
    require(!Link::isConstraintType(type), "Non-constraint type recognized");
    rejects([&] { Link invalid(1, 2, type, SE3f::identity()); });
    rejects([&] { edge.setType(type); });
    require(edge.type() == Link::kOdometry, "Failed setter changed the link");
  }
  std::multimap<int, Link> graph;
  graph.emplace(1, edge);
  graph.emplace(1, Link(1, 1, Link::kPosePrior, SE3f::identity()));
  graph.emplace(1, Link(1, 1, Link::kGravity, SE3f::identity()));
  require(graph::findLink(graph, 2, 1) != graph.end(), "Unfiltered reverse lookup failed");
  require(graph::findLink(graph, 1, 1, false, Link::kGravity)->second.type() == Link::kGravity, "Exact self-link lookup failed");
  require(graph::findLink(graph, 1, 2, false, Link::kUndef) == graph.end(), "Invalid sentinel still means any type");
  require(!Link(1, 1, Link::kPosePrior, SE3f::identity()).isRelocalization(), "Prior classified as reducible loop");

}
std::unique_ptr<Node> fixture() {
  auto node = std::make_unique<Node>(1, 0, 1, 1, "link-fixture", SE3f::identity());
  const Link::Type types[] = {Link::kOdometry, Link::kRelocalization, Link::kRelocalization, Link::kRelocalization, Link::kOdometry};
  sapphire::Mat66_t<double> info = sapphire::Mat66_t<double>::Identity() * 3;
  info(0, 1) = info(1, 0) = .2;
  int target = 2;
  for (auto type : types) node->addLink(Link(1, target++, type, SE3f::identity(), info));
  node->addLink(Link(1, 1, Link::kPosePrior, SE3f::identity()));
  node->addLink(Link(1, 1, Link::kGravity, SE3f::identity()));
  node->addLink(Link(1, 20, Link::kOdometry, SE3f()));  // Intentional connectivity placeholder.
  return node;
}
void contraction() {
  Topograph graph;
  for (int id : {1, 2, 3, 4}) graph.admit(new Node(id, 0, 1, 0, "", SE3f::identity()));
  const auto add = [&](const Link& edge) {
    graph.nodes().at(edge.from())->addLink(edge);
    if (edge.from() != edge.to()) graph.nodes().at(edge.to())->addLink(edge.inverse());
  };
  const auto anchor = SE3f::fromTranslation({.4f, .1f, 0});
  const Link original(3, 4, Link::kOdometry, SE3f::fromTranslation({1, 0, 0}));
  add(Link(3, 1, Link::kRelocalization, anchor.inverse()));
  add(Link(2, 3, Link::kOdometry, SE3f::identity()));
  add(original);
  add(Link(3, 3, Link::kPosePrior, SE3f::identity()));
  require(graph.reduceNode(3, 0, -1) == 0 && graph.getNode(2)->hasLink(3), "Reduction dropped a unary prior");
  graph.nodes().at(3)->removeLink(3);
  require(graph.reduceNode(3, 0, -1) == 1, "Verified loop did not select retained node");
  auto* removed = graph.nodes().at(3);
  graph.release(3); delete removed;
  const auto& rewritten = graph.getNode(1)->getLinks().find(4)->second;
  const auto expected = original.reanchor(1, 3, anchor);
  require(rewritten.isOdometry(), "Node contraction invented a loop from odometry");
  require((rewritten.infMatrix() - expected.infMatrix()).norm() < 1e-9 &&
          rewritten.transform().translationDistance(expected.transform()) < 1e-6,
          "Contraction composed uncertain paths instead of changing coordinates");
  for (int id : {1, 2, 4}) {
    require(!graph.getNode(id)->hasLink(3), "Contraction left a retired endpoint");
    require(graph.reduceNode(id, 0, 0) == 0, "Redirected odometry triggered cascading reduction");
  }
  require(graph.getNeighborsId(1, 0, 0, false, true, false, {}, nullptr, {}).size() == 3, "Contraction disconnected the odometry backbone");
}
void repeatedContraction() {
  for (int direction : {-1, 1}) {
    Topograph graph;
    constexpr int count = 120;
    for (int id = 1; id <= count; ++id) graph.admit(new Node(id, 0, 1, 0, "", SE3f::identity()));
    const auto add = [&](int from, int to, Link::Type type) {
      const Link edge(from, to, type, SE3f::identity());
      graph.nodes().at(from)->addLink(edge);
      graph.nodes().at(to)->addLink(edge.inverse());
    };
    for (int id = 1; id < count; ++id) add(id, id + 1, Link::kOdometry);
    int loops = 0;
    for (int id = 20; id <= count; id += 4) { add(id, id - 16, Link::kRelocalization); ++loops; }
    int removed = 0;
    for (int step = 1; step <= count; ++step) {
      const int id = direction < 0 ? step : count + 1 - step;
      if (!graph.reduceNode(id, 0, direction)) continue;
      auto* node = graph.nodes().at(id); graph.release(id); delete node; ++removed;
    }
    require(removed == loops, "Contraction expanded beyond verified loop relations");
    const int first = graph.nodes().begin()->first;
    require(graph.getNeighborsId(first, 0, 0, false, false, false, {}, nullptr, {}).size() == graph.nodes().size(),
            "Repeated contraction lost graph connectivity");
    size_t edges = 0;
    for (const auto& [id, node] : graph.nodes()) {
      edges += node->getLinks().size();
      for (const auto& [other, edge] : node->getLinks()) {
        require(graph.getNode(other) && graph.getNode(other)->hasLink(id), "Contraction left a dangling or one-way edge");
        require(edge.isOdometry(), "Contraction invented a new loop from odometry");
      }
    }
    require(edges / 2 <= size_t(count - 1 + loops), "Contraction grew the graph");
  }
}

void queries(DBDriver& db) {
  std::multimap<int, Link> links;
  links.emplace(99, Link(99, 100, Link::kOdometry, SE3f::identity()));
  db.loadLinks(1, links);
  require(links.size() == 8 && !links.count(-7) && !links.count(99), "Default query includes landmarks or stale output");
  db.loadLinks(1, links, Link::kGravity);
  require(links.size() == 1 && links.begin()->second.type() == Link::kGravity, "Exact gravity query failed");
  db.loadLinks(1, links, Link::kRelocalization);
  require(links.size() == 3 && links.count(3) && links.count(4) && links.count(5), "Unified loop query failed");
  const auto& edge = links.find(5)->second;
  require(edge.infMatrix()(0, 1) == .2, "Information cross term changed");
  require(edge.isRelocalization(), "Ordinary loop has unexpected metadata");
  rejects([&] { db.loadLinks(1, links, Link::kUndef); });
}
void storage() {
  DatabaseFile file;
  {
    HeldDriver writer;
    require(writer.openConnection(file.path, true), "Writer open failed");
    require(writer.asyncSave(fixture()), "Fixture enqueue failed");
    writer.wait();
    queries(writer);  // Guaranteed in-flight snapshot, before disk write.
    writer.release();
    writer.emptyTrashes();
    queries(writer);  // Same API, persisted path.
    writer.closeConnection();
  }
  DBDriverSqlite3 reader;
  require(reader.openConnection(file.path), "Reader open failed");
  queries(reader);
  rejects([&] { reader.addLink(Link()); });
  rejects([&] { reader.updateLink(Link()); });
  std::multimap<int, Link> all;
  reader.getAllLinks(all, true);
  require(all.size() == 7, "Null-transform filtering lost a real link");
  reader.getAllLinks(all, false);
  require(all.size() == 8, "Null placeholder not preserved");
  auto loaded = std::unique_ptr<Node>(reader.loadNode(1));
  require(loaded && loaded->getLinks().size() == 8, "Node lost links");
  reader.closeConnection();
  for (int bad : {2, 3, 4, 5, 6, 8, 97, 99, 42}) {
    sql(file.path, "UPDATE Link SET type=" + std::to_string(bad) + " WHERE from_id=1 AND to_id=2");
    require(reader.openConnection(file.path), "Invalid fixture open failed");
    rejects([&] { reader.getAllLinks(all, false); });
    rejects([&] { reader.loadLinks(1, all); });
    rejects([&] { std::unique_ptr<Node> invalid(reader.loadNode(1)); });
    Node sentinel(100);
    std::list<Node*> output{&sentinel};
    rejects([&] { reader.loadNodes({1}, output); });
    require(output.size() == 1 && output.front() == &sentinel, "Failed read published partial nodes");
    reader.closeConnection();  // Statements must also finalize when decoding throws.
  }
  sql(file.path, "UPDATE Link SET type=0 WHERE from_id=1 AND to_id=2");
  require(reader.openConnection(file.path), "Repaired fixture open failed");
  queries(reader);
  reader.closeConnection();
  sql(file.path, "PRAGMA user_version=11");
  require(!reader.openConnection(file.path), "Old Link payload schema was accepted");
  sql(file.path, "PRAGMA user_version=14");
  require(reader.openConnection(file.path), "Current schema was not preserved after rejection");
  queries(reader);
  reader.closeConnection();
}
}  // namespace
int main() {
  try {
    types();
    pose_graph_contract();
    memory_graph_boundary();
    contraction();
    repeatedContraction();

    storage();
    std::cout << "Link types, filters, all storage paths and retired-type rejection passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
