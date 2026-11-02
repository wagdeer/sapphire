#include <common/common_type.hpp>
#include <unistd.h>
#include <sys/resource.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "maplayer/topograph/node.hpp"
#include "database/sqlite.hpp"

namespace {

using namespace std::chrono_literals;

class TestDriver : public sapphire::DBDriverSqlite3 {
 public:
  explicit TestDriver(const sapphire::ParametersMap &parameters = {}) : sapphire::DBDriverSqlite3(parameters) {}

  std::atomic<int> delayMilliseconds{0};
  std::atomic<bool> throwBeforeWrite{false};
  std::atomic<bool> throwAfterNodeWrite{false};

 protected:
  void beforeAsyncBatchWrite() override {
    const int delay = delayMilliseconds.load();
    if (delay > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }
    if (throwBeforeWrite.load()) {
      throw std::runtime_error("injected DB worker failure");
    }
  }

  void saveQuery(const std::list<sapphire::Node *> &nodes) override {
    sapphire::DBDriverSqlite3::saveQuery(nodes);
    if (throwAfterNodeWrite.load()) {
      throw std::runtime_error("injected failure after node write");
    }
  }
};

std::unique_ptr<sapphire::Node> node(int id, const std::string &label, int weight = 1, int mapId = 0) {
  return std::make_unique<sapphire::Node>(id, mapId, weight, static_cast<double>(id), label);
}

bool readLabel(sapphire::DBDriver &driver, int id, std::string &label, sapphire::SE3f *outputPose = nullptr) {
  sapphire::SE3f pose;
  int mapId = -1;
  int weight = 0;
  double stamp = 0.0;
  std::vector<float> velocity;

  const bool found = driver.getNodeInfo(id, pose, mapId, weight, label, stamp, velocity);
  if (found && outputPose) {
    *outputPose = pose;
  }
  return found;
}

std::string temporaryDatabase(const char *suffix) {
  return "/tmp/exterior_db_worker_" + std::to_string(static_cast<long long>(getpid())) + "_" + suffix + ".db";
}

int runReadWriteAndSequenceTests() {
  sapphire::ParametersMap parameters;
  parameters["database.writer.batch_max_latency_ms"] = "20";
  parameters["database.writer.batch_max_items"] = "16";
  std::unique_ptr<TestDriver> driverStorage = std::make_unique<TestDriver>(parameters);
  TestDriver &driver = *driverStorage;
  driver.delayMilliseconds = 30;
  if (!driver.openConnection("", true)) {
    return 1;
  }
  if (driver.getTotalNodesSize() != 0 || driver.getLastNodesSize() != 0 || driver.getNodesMemoryUsed() != 0 ||
      driver.getLinksMemoryUsed() != 0 || driver.getStatisticsMemoryUsed() != 0) {
    throw std::runtime_error("Empty database scalar queries did not return zero");
  }

  if (!driver.asyncSave(node(1, "old")) || !driver.asyncSave(node(1, "replacement"))) {
    return 2;
  }
  std::string label;
  if (!readLabel(driver, 1, label) || label != "replacement") {
    return 3;
  }
  driver.delayMilliseconds = 5;
  if (!driver.asyncSave(node(2, "barrier"))) {
    return 4;
  }
  auto barrier = std::async(std::launch::async, [&driver] { driver.emptyTrashes(false); });
  for (int id = 3; id < 20; ++id) {
    if (!driver.asyncSave(node(id, "after-barrier-" + std::to_string(id)))) {
      return 5;
    }
  }
  if (barrier.wait_for(2s) != std::future_status::ready) {
    return 6;
  }
  barrier.get();
  driver.emptyTrashes(false);
  if (driver.getTotalNodesSize() != 19 || driver.getNodesMemoryUsed() <= 0) {
    throw std::runtime_error("Scalar queries lost committed node count or payload size");
  }

  for (int round = 0; round < 100; ++round) {
    const int id = 1000 + round;
    const std::string roundLabel = "round-" + std::to_string(round);
    if (!driver.asyncSave(node(id, roundLabel))) {
      std::fprintf(stderr, "round enqueue failed at %d: %s\n", round, driver.getWorkerError().c_str());
      return 7;
    }
    driver.emptyTrashes(false);
    label.clear();
    if (!readLabel(driver, id, label) || label != roundLabel) {
      return 8;
    }
  }

  driver.delayMilliseconds = 30;
  if (!driver.asyncSave(node(2000, "visible-last", 1, 7))) {
    return 24;
  }
  int lastNodeId = 0;
  int lastMapId = 0;
  driver.getLastNodeId(lastNodeId);
  driver.getLastMapId(lastMapId);
  if (lastNodeId != 2000 || lastMapId != 7) {
    return 25;
  }
  driver.emptyTrashes(false);

  const sapphire::DBDriver::WorkerMetrics metrics = driver.getWorkerMetrics();
  if (metrics.highWaterItems == 0 || metrics.maxBatchItems == 0 || metrics.maxBatchItems > 16 || metrics.acceptedSequence != metrics.completedSequence ||
      metrics.failed) {
    return 9;
  }
  driver.closeConnection(false);
  return 0;
}

int runBackpressureTest(bool immediate = false) {
  sapphire::ParametersMap parameters;
  parameters["database.writer.max_items"] = "1";
  parameters["database.writer.max_bytes"] = "1048576";
  if (!immediate) parameters["database.writer.producer_wait_ms"] = "10";
  parameters["database.writer.batch_max_latency_ms"] = "0";
  std::unique_ptr<TestDriver> driverStorage = std::make_unique<TestDriver>(parameters);
  TestDriver &driver = *driverStorage;
  driver.delayMilliseconds = 60;
  if (!driver.openConnection("", true) || !driver.asyncSave(node(1, "first"))) {
    return 10;
  }
  const auto started = std::chrono::steady_clock::now();
  if (driver.asyncSave(node(2, "second"))) return 11;
  const auto elapsed = std::chrono::steady_clock::now() - started;
  const auto metrics = driver.getWorkerMetrics();
  if (!metrics.failed || metrics.highWaterItems != 1) return 12;
  if (immediate) {
    if (metrics.producerWaitCount || metrics.producerWaitNanoseconds ||
        driver.getWorkerError().find("queue capacity exceeded") == std::string::npos) return 12;
  } else if (elapsed < 10ms || elapsed > 50ms || metrics.producerWaitCount != 1 ||
             driver.getWorkerError().find("capacity wait expired") == std::string::npos) return 12;

  driver.closeConnection(false);
  return 0;
}

// Incoming records are wakeups to recheck a batch, not implicit flush requests.
// With sparse arrivals below the size caps, retain the first entry's deadline.
int runBatchLatencyNotificationTest() {
  sapphire::ParametersMap parameters;
  parameters["database.writer.batch_max_latency_ms"] = "150";
  parameters["database.writer.batch_max_items"] = "128";
  struct TimedDriver : TestDriver {
    using TestDriver::TestDriver;
    std::promise<std::chrono::steady_clock::time_point> firstWrite;
    bool first = true;
    void beforeAsyncBatchWrite() override {
      if (first) { first = false; firstWrite.set_value(std::chrono::steady_clock::now()); }
    }
  } driver(parameters);
  if (!driver.openConnection("", true)) return 110;
  auto firstWrite = driver.firstWrite.get_future();
  const auto started = std::chrono::steady_clock::now();
  for (int id = 1; id <= 5; ++id) {
    auto item = node(id, "batch-notification-" + std::to_string(id));
    const auto epoch = item->write_epoch;
    if (!driver.asyncSave(std::move(item))) {
      std::fprintf(stderr, "batch enqueue failed: id=%d node_epoch=%llu driver_epoch=%llu error=%s\n", id,
                   static_cast<unsigned long long>(epoch), static_cast<unsigned long long>(driver.write_epoch()),
                   driver.getWorkerError().c_str());
      return 111;
    }
    std::this_thread::sleep_for(10ms);
  }
  if (firstWrite.wait_for(2s) != std::future_status::ready) return 112;
  const auto latency = firstWrite.get() - started;
  driver.emptyTrashes();
  const auto metrics = driver.getWorkerMetrics();
  const bool valid = driver.isWorkerHealthy() && latency >= 125ms && metrics.maxBatchItems == 5 && driver.getTotalNodesSize() == 5;
  driver.closeConnection(false);
  std::fprintf(stdout, "batch_notification_probe: first_write_ms=%.3f max_batch_items=%zu\n",
               std::chrono::duration<double, std::milli>(latency).count(), metrics.maxBatchItems);
  return valid ? 0 : 113;
}

int runCommitDeadlineTest() {
  sapphire::ParametersMap parameters;
  parameters["database.writer.commit_timeout_ms"] = "30";
  parameters["database.writer.batch_max_latency_ms"] = "0";
  struct HeldDriver : TestDriver {
    using TestDriver::TestDriver;
    std::promise<void> entered, release;
    void beforeAsyncBatchWrite() override { entered.set_value(); release.get_future().wait(); }
  };
  HeldDriver driver(parameters);
  std::promise<std::string> fault;
  auto notified = fault.get_future();
  driver.setWorkerFailureCallback([&](const std::string& message) { fault.set_value(message); });
  if (!driver.openConnection("", true) || !driver.asyncSave(node(1, "held"))) return 81;
  driver.entered.get_future().wait();
  const bool detected = notified.wait_for(300ms) == std::future_status::ready;
  const auto metrics = driver.getWorkerMetrics();
  const auto cpuMs = [] { rusage r{}; getrusage(RUSAGE_SELF, &r);
    return (r.ru_utime.tv_sec + r.ru_stime.tv_sec) * 1000. + (r.ru_utime.tv_usec + r.ru_stime.tv_usec) * .001; };
  const auto beforeCpu = cpuMs();
  std::this_thread::sleep_for(100ms); // Failed worker must not spin on retained pending entries.
  const auto idleCpu = cpuMs() - beforeCpu;
  std::fprintf(stdout, "deadline_probe: oldest_ms=%.3f retained_items=%zu fault_idle_cpu_ms=%.3f interval_ms=100\n",
               metrics.oldestAgeMs, metrics.pendingItems, idleCpu);
  auto closing = std::async(std::launch::async, [&] { driver.closeConnection(false); });
  const bool stillOwned = closing.wait_for(20ms) == std::future_status::timeout;
  driver.release.set_value();
  closing.get();
  if (!detected || !stillOwned || idleCpu > 40 || !metrics.failed || metrics.pendingItems != 1 || metrics.oldestAgeMs < 30 ||
      notified.get().find("commit deadline expired") == std::string::npos) return 82;
  // A single oversized item cannot bypass the fixed byte budget.
  parameters["database.writer.max_bytes"] = "1";
  TestDriver oversized(parameters);
  if (!oversized.openConnection("", true)) return 83;
  const bool rejected = !oversized.asyncSave(node(2, "too-large")) && !oversized.isWorkerHealthy() &&
      oversized.getWorkerMetrics().pendingItems == 0;
  oversized.closeConnection(false);
  return rejected ? 0 : 84;
}

int runBlockedCommitTest() {
  struct CommitDriver : TestDriver {
    using TestDriver::TestDriver;
    mutable std::atomic<bool> hold{false};
    mutable std::promise<void> entered, release;
    void executeNoResultQuery(const std::string& sql) const override {
      if (sql == "COMMIT;" && hold.exchange(false)) { entered.set_value(); release.get_future().wait(); }
      TestDriver::executeNoResultQuery(sql);
    }
  };
  sapphire::ParametersMap parameters;
  parameters["database.writer.commit_timeout_ms"] = "50";
  parameters["database.writer.batch_max_latency_ms"] = "0";
  parameters["database.writer.batch_max_items"] = "1";
  const auto path = temporaryDatabase("held_commit");
  std::remove(path.c_str());
  CommitDriver driver(parameters);
  std::promise<void> failed;
  auto failure = failed.get_future();
  driver.setWorkerFailureCallback([&](const std::string&) { failed.set_value(); });
  if (!driver.openConnection(path, true)) return 85;
  driver.hold = true;
  if (!driver.asyncSave(node(1, "commit-in-progress"))) return 86;
  driver.entered.get_future().wait();
  // Statistics accounting must not queue behind a durable commit. This gate
  // holds the actual transaction mutex, unlike beforeAsyncBatchWrite().
  auto accounting = std::async(std::launch::async, [&] { return driver.getMemoryUsed(); });
  const bool accountingReady = accounting.wait_for(20ms) == std::future_status::ready;
  const bool queued = driver.asyncSave(node(2, "must-not-start"));
  const bool reported = failure.wait_for(500ms) == std::future_status::ready;
  auto drain = std::async(std::launch::async, [&] { driver.emptyTrashes(false); });
  const bool releasedWaiter = drain.wait_for(100ms) == std::future_status::ready;
  auto close = std::async(std::launch::async, [&] { driver.closeConnection(false); });
  const bool owned = close.wait_for(20ms) == std::future_status::timeout;
  driver.release.set_value();
  drain.get(); close.get();
  const auto accountedBytes = accounting.get();
  if (!accountingReady || !accountedBytes || !queued || !reported || !releasedWaiter || !owned) return 87;
  // Timeout is not a rollback acknowledgement: the already executing commit
  // may finish, but no following batch may be consumed after the fault.
  TestDriver reader;
  if (!reader.openConnection(path)) return 88;
  std::string label;
  const bool preserved = readLabel(reader, 1, label) && label == "commit-in-progress" && !readLabel(reader, 2, label);
  reader.closeConnection(false); std::remove(path.c_str());
  return preserved ? 0 : 89;
}

int runShutdownTests() {
  const std::string savePath = temporaryDatabase("save");
  const std::string discardPath = temporaryDatabase("discard");
  std::remove(savePath.c_str());
  std::remove(discardPath.c_str());

  {
    std::unique_ptr<TestDriver> driverStorage = std::make_unique<TestDriver>();
    TestDriver &driver = *driverStorage;
    std::unique_ptr<sapphire::Node> saved = node(50, "saved");
    saved->setPose(sapphire::SE3f::fromRotationTranslation(Eigen::AngleAxisf(0.35f, Eigen::Vector3f::UnitZ()).toRotationMatrix(),
                                                                  Eigen::Vector3f(1.25f, -2.5f, 3.75f)));
    if (!driver.openConnection(savePath, true) || !driver.asyncSave(std::move(saved))) {
      return 13;
    }
    driver.closeConnection(true);
  }
  {
    std::unique_ptr<TestDriver> driverStorage = std::make_unique<TestDriver>();
    TestDriver &driver = *driverStorage;
    if (!driver.openConnection(savePath)) {
      return 14;
    }
    std::string label;
    sapphire::SE3f restoredPose;
    if (!readLabel(driver, 50, label, &restoredPose) || label != "saved" || restoredPose.isNull() ||
        (restoredPose.translationVector() - Eigen::Vector3f(1.25f, -2.5f, 3.75f)).norm() > 1e-6f ||
        restoredPose.rotationDistance(sapphire::SE3f::rotationExp(Eigen::Vector3f(0.0f, 0.0f, 0.35f))) > 1e-6f) {
      return 15;
    }
    driver.closeConnection(false);
  }

  {
    std::unique_ptr<TestDriver> driverStorage = std::make_unique<TestDriver>();
    TestDriver &driver = *driverStorage;
    driver.delayMilliseconds = 100;
    if (!driver.openConnection(discardPath, true) || !driver.asyncSave(node(60, "discarded"))) {
      return 16;
    }
    driver.closeConnection(false);
  }
  {
    std::unique_ptr<TestDriver> driverStorage = std::make_unique<TestDriver>();
    TestDriver &driver = *driverStorage;
    if (!driver.openConnection(discardPath)) {
      return 17;
    }
    std::string label;
    if (readLabel(driver, 60, label)) {
      return 18;
    }
    driver.closeConnection(false);
  }

  {
    std::unique_ptr<TestDriver> driverStorage = std::make_unique<TestDriver>();
    TestDriver &driver = *driverStorage;
    driver.delayMilliseconds = 100;
    if (!driver.openConnection("", true) || !driver.asyncSave(node(61, "discard-before-reopen"))) {
      return 26;
    }
    driver.closeConnection(false);
    driver.delayMilliseconds = 0;
    if (!driver.openConnection("", true) || !driver.asyncSave(node(62, "after-reopen"))) {
      return 27;
    }
    auto flush = std::async(std::launch::async, [&driver] { driver.emptyTrashes(false); });
    if (flush.wait_for(2s) != std::future_status::ready) {
      return 28;
    }
    flush.get();
    std::string label;
    if (!readLabel(driver, 62, label) || label != "after-reopen") {
      return 29;
    }
    driver.closeConnection(false);
  }

  std::remove(savePath.c_str());
  std::remove(discardPath.c_str());
  return 0;
}

int runConcurrentReaderTest() {
  const auto path = temporaryDatabase("reader");
  TestDriver writer;
  if (!writer.openConnection(path, true) || !writer.asyncSave(node(1, "before"))) return 32;
  writer.emptyTrashes(false);
  TestDriver reader;
  if (!reader.openConnection(path)) return 33;
  reader.executeNoResult("BEGIN");
  std::string label;
  if (!readLabel(reader, 1, label)) return 34;
  auto write = std::async(std::launch::async, [&] {
    writer.asyncSave(node(2, "after"));
    writer.emptyTrashes(false);
    return writer.isWorkerHealthy();
  });
  const bool waited = write.wait_for(100ms) == std::future_status::timeout;
  reader.executeNoResult("ROLLBACK");
  const bool healthy = write.get();
  reader.closeConnection(false);
  const bool saved = readLabel(writer, 2, label) && label == "after";
  writer.closeConnection(false);
  std::remove(path.c_str());
  return waited && healthy && saved ? 0 : 35;
}

int runFailureTest() {
  const std::string failurePath = temporaryDatabase("failure");
  std::remove(failurePath.c_str());
  std::unique_ptr<TestDriver> driverStorage = std::make_unique<TestDriver>();
  TestDriver &driver = *driverStorage;
  driver.throwAfterNodeWrite = true;
  if (!driver.openConnection(failurePath, true) || !driver.asyncSave(node(70, "failure"))) {
    return 19;
  }
  driver.emptyTrashes(false);
  if (driver.isWorkerHealthy() || driver.getWorkerError().find("injected") == std::string::npos) {
    return 20;
  }
  std::string label;
  if (!readLabel(driver, 70, label) || label != "failure") {
    return 23;
  }
  if (driver.asyncSave(node(71, "rejected"))) {
    return 21;
  }
  bool saveRejected = false;
  try { driver.closeConnection(true); }
  catch (const std::runtime_error& error) {
    saveRejected = std::string(error.what()).find("injected") != std::string::npos;
  }
  if (!saveRejected || driver.isConnected()) return 48;
  driver.closeConnection(false);  // Cleanup remains possible after a failed Save.
  driver.throwAfterNodeWrite = false;
  if (!driver.openConnection(failurePath)) {
    return 30;
  }
  label.clear();
  if (readLabel(driver, 70, label)) {
    return 31;
  }
  driver.closeConnection(false);
  std::remove(failurePath.c_str());
  return 0;
}

// A synchronous SQLite exception must not strand the shared database mutex.
int runFailedWriterDestructionTest() {
  TestDriver driver;
  driver.throwBeforeWrite = true;
  if (!driver.openConnection("", true) || !driver.asyncSave(node(1, "failure"))) return 49;
  driver.emptyTrashes(false);
  if (driver.isWorkerHealthy()) return 50;
  return 0;  // Destruction must neither throw nor attempt to save a failed writer.
}

int runExceptionUnlockTest() {
  class ThrowingDriver : public sapphire::DBDriverSqlite3 {
    void saveOptimizedPosesQuery(const std::map<int, sapphire::SE3f>&,
                                const sapphire::SE3f&) const override {
      throw std::runtime_error("injected synchronous write failure");
    }
  } driver;
  bool rejected = false;
  try { driver.saveOptimizedPoses({}, sapphire::SE3f{}); }
  catch (const std::runtime_error&) { rejected = true; }
  if (!rejected || driver.isConnected()) return 47;
  driver.closeConnection(false);
  return 0;
}

int runDependentWriteFlushTest() {
  sapphire::ParametersMap parameters;
  parameters["database.writer.batch_max_latency_ms"] = "2000";
  TestDriver driver(parameters);
  if (!driver.openConnection("", true)) return 40;
  if (!driver.asyncSave(node(1, "trajectory-anchor"))) return 41;
  auto trajectory = std::async(std::launch::async, [&] {
    sapphire::OdomObservation o{1, 1, 1., sapphire::SE3f::identity(), sapphire::SE3f::identity()};
    driver.save_trajectory({o});
  });
  if (trajectory.wait_for(750ms) != std::future_status::ready) return 42;
  trajectory.get();
  if (driver.load_trajectory().size() != 1) return 43;
  driver.closeConnection(false);
  return 0;
}

// Put a flush notification before the worker can enter its next batching wait.
// The first SQL write is latched, so the second node/flush cannot be consumed
// until released. No production scheduling hooks or short 50 ms timing bound.
int runFlushWhileWriterBusyTest() {
  class PausedDriver : public TestDriver {
   public:
    explicit PausedDriver(const sapphire::ParametersMap& p) : TestDriver(p) {}
    std::promise<void> entered, release;
   private:
    bool first = true;
    void beforeAsyncBatchWrite() override {
      if (first) { first = false; entered.set_value(); release.get_future().wait(); }
    }
  };
  sapphire::ParametersMap parameters;
  parameters["database.writer.batch_max_latency_ms"] = "3000";
  parameters["database.writer.commit_timeout_ms"] = "10000";
  PausedDriver driver(parameters);
  if (!driver.openConnection("", true) || !driver.asyncSave(node(1,"latched"))) return 60;
  driver.entered.get_future().wait();
  if (!driver.asyncSave(node(2,"flush-required"))) { driver.release.set_value(); return 61; }
  std::promise<void> started;
  auto flush = std::async(std::launch::async,[&] { started.set_value(); driver.emptyTrashes(false); });
  started.get_future().wait();
  const bool blocked = flush.wait_for(200ms) == std::future_status::timeout;
  driver.release.set_value();
  const bool prompt = flush.wait_for(750ms) == std::future_status::ready;
  flush.get();
  std::string label;
  const bool saved = readLabel(driver,2,label) && label == "flush-required";
  driver.closeConnection(false);
  if (!blocked || !prompt || !saved) { std::fprintf(stderr,"Busy-writer flush: blocked=%d prompt=%d saved=%d\n",blocked,prompt,saved); return 62; }
  return 0;
}

int runPendingNodeSnapshotTest() {
  using namespace sapphire;
using namespace sapphire;
  class PausedDriver : public DBDriverSqlite3 {
   public:
    std::promise<void> entered, release;
   private:
    bool first = true;
    void beforeAsyncBatchWrite() override {
      if (first) {
        first = false;
        entered.set_value();
        release.get_future().wait();
      }
    }
  } driver;
  if (!driver.openConnection("", true) || !driver.asyncSave(node(1, "blocker"))) return 50;
  driver.entered.get_future().wait();
  bool queued = driver.asyncSave(node(2, "initial"));
  auto replacement = node(2, "replacement");
  replacement->setSaved(true); // Owner already handed its initial snapshot to the writer.
  queued = queued && driver.asyncSave(std::move(replacement));
  std::unique_ptr<Node> snapshot(driver.loadNode(2));
  driver.release.set_value();
  driver.emptyTrashes();
  if (snapshot) {
    snapshot->setLabel("read-snapshot");
    queued = queued && driver.asyncSave(std::make_unique<Node>(*snapshot));
    driver.emptyTrashes();
  }
  std::string label;
  const bool valid = readLabel(driver, 2, label) && label == "read-snapshot" &&
      snapshot && snapshot->id() == 2;
  driver.closeConnection(false);
  return queued && valid ? 0 : 51;
}

// Exercise complete keyframe products under a held worker: acceptance must not
// wait for its own anchor, and readers must see the same pending snapshot.
int runKeyframeBatchTests() {
  using namespace sapphire;
using namespace sapphire;
  class BatchDriver : public TestDriver {
   public:
    std::promise<void> entered, release;
    mutable std::atomic<int> commits{0};
    mutable std::atomic<bool> producerCommit{false};
    std::thread::id producer = std::this_thread::get_id();
    bool held = true;
    bool failStatistics = false;
   protected:
    void beforeAsyncBatchWrite() override {
      if (held) { held = false; entered.set_value(); release.get_future().wait(); }
    }
    void executeNoResultQuery(const std::string& sql) const override {
      TestDriver::executeNoResultQuery(sql);
      if (sql == "COMMIT;") {
        ++commits;
        if (std::this_thread::get_id() == producer) producerCommit = true;
      }
    }
    void addStatisticsQuery(const Statistics& stats, bool wm) const override {
      TestDriver::addStatisticsQuery(stats, wm);
      if (failStatistics) throw std::runtime_error("injected complete batch failure");
    }
  };
  const auto make = [](int sequence, bool initial) {
    DBDriver::WriteBatch batch;
    auto snapshot = node(1, "version-" + std::to_string(sequence));
    snapshot->setSaved(!initial);
    batch.nodes.emplace_back(std::move(snapshot));
    OdomObservation row;
    row.sequence = sequence; row.anchor_id = 1; row.stamp = sequence;
    row.anchor_from_body = SE3f::identity(); row.odom_from_body = SE3f::identity();
    batch.trajectory.push_back(row);
    batch.statistics.emplace();
    batch.statistics->setRefNodeId(1);
    batch.statistics->setStamp(sequence);
    batch.statistics->addStatistic("Test/Version/", sequence);
    batch.statistics->setWmState({1, sequence});
    batch.saveWmState = true;
    return batch;
  };
  {
    BatchDriver driver;
    if (!driver.openConnection("", true)) return 70;
    const auto first = driver.asyncWrite(make(1, true));
    driver.entered.get_future().wait();
    const auto second = driver.asyncWrite(make(2, false));
    // Capture checks before releasing the latch, but never throw while held.
    double stamp = 0;
    std::vector<int> wm;
    const auto stats = driver.getStatistics(1, stamp, &wm);
    const auto rows = driver.load_trajectory();
    const auto all = driver.getAllStatistics();
    const bool visible = first && second > first && !driver.writeCommitted(first) && !driver.writeCommitted(second) &&
        rows.size() == 2 && rows[0].sequence == 1 && rows[1].sequence == 2 &&
        driver.load_recent_trajectory(1).back().sequence == 2 && driver.trajectory_high_water() == 2 &&
        stats.at("Test/Version/") == 2 && stamp == 2 && wm == std::vector<int>({1, 2}) && all.at(1).second == 2;
    driver.release.set_value();
    driver.emptyTrashes();
    std::string label;
    const bool committed = driver.isWorkerHealthy() && driver.writeCommitted(first) && driver.writeCommitted(second) &&
        driver.load_trajectory().size() == 2 && readLabel(driver, 1, label) && label == "version-2" &&
        driver.commits == 2 && !driver.producerCommit;
    driver.closeConnection(false);
    if (!visible || !committed) return 71;
  }
  const auto path = temporaryDatabase("complete-batch-rollback");
  std::remove(path.c_str());
  {
    BatchDriver driver;
    driver.failStatistics = true;
    if (!driver.openConnection(path, true)) return 72;
    const auto accepted = driver.asyncWrite(make(1, true));
    driver.entered.get_future().wait();
    driver.release.set_value();
    driver.emptyTrashes();
    const bool failed = !driver.isWorkerHealthy() && !driver.writeCommitted(accepted) && !driver.asyncWrite(make(2, false));
    bool saveRejected = false;
    try { driver.closeConnection(true); } catch (const std::runtime_error&) { saveRejected = true; }
    if (!failed || !saveRejected) return 73;
  }
  {
    TestDriver reader;
    if (!reader.openConnection(path)) return 74;
    std::string label;
    if (readLabel(reader, 1, label) || !reader.load_trajectory().empty() ||
        !reader.getAllStatistics().empty()) return 75;
    reader.closeConnection(false);
  }
  std::remove(path.c_str());
  for (bool save : {false, true}) {
    BatchDriver driver;
    if (!driver.openConnection(path, true)) return 76;
    const auto sequence = driver.asyncWrite(make(1, true));
    driver.entered.get_future().wait();
    auto closing = std::async(std::launch::async, [&] { driver.closeConnection(save); });
    // close must wait for the held worker in both modes; discard has been
    // requested by the time the worker is released below.
    const bool waits = closing.wait_for(50ms) == std::future_status::timeout;
    driver.release.set_value();
    closing.get();
    if (!waits || (save != driver.writeCommitted(sequence))) return 77;
    TestDriver reader;
    if (!reader.openConnection(path)) return 78;
    const auto count = reader.load_trajectory().size();
    if (count != size_t(save) || reader.getAllStatistics().size() != size_t(save)) return 79;
    reader.closeConnection(false);
    std::remove(path.c_str());
    if (!driver.openConnection("", true) || driver.writeCommitted(sequence)) return 80;
    driver.closeConnection(false);
  }
  return 0;
}

int runWorkingMemorySnapshotTests() {
  using namespace sapphire;
using namespace sapphire;
  const auto path = temporaryDatabase("working_memory");
  TestDriver db;
  if (!db.openConnection(path, true)) return 90;
  if (!db.asyncSave(node(1, "cold")) || !db.asyncSave(node(2, "resident"))) return 91;
  db.emptyTrashes();
  std::set<int> ids;
  db.getLastNodeIds(ids);
  if (!ids.empty() || db.getLastNodesSize()) return 92; // Ordinary writes are not a completed snapshot.
  std::list<Node*> loaded;
  db.loadNodes({1, 2}, loaded);
  std::vector<std::unique_ptr<Node>> owners;
  for (auto* ptr : loaded) owners.emplace_back(ptr);
  if (owners.size() != 2) return 93;
  auto* cold = owners[0].get(); auto* resident = owners[1].get();
  DBDriver::RunInfo info;
  info.lastNode = 2;
  // Recorded strings are preserved verbatim, including SQL punctuation.
  info.parameters = {{"registration.feature.pyramid.levels", "3"}, {"marker.lengths_m", "a'b"}};
  db.saveWorkingMemory({resident}, info);
  // Identical wall timestamps and a later clock reversal cannot select cold nodes
  // or make old parameter records win over the latest committed record.
  db.executeNoResult("UPDATE Node SET time_enter='2099-01-01 00:00:00'; UPDATE Info SET time_enter='2099-01-01 00:00:00';");
  info.parameters["registration.feature.pyramid.levels"] = "4";
  db.saveWorkingMemory({resident}, info);
  db.executeNoResult("UPDATE Info SET time_enter='1970-01-01 00:00:00' WHERE rowid=(SELECT MAX(rowid) FROM Info);");
  ids.clear(); db.getLastNodeIds(ids);
  if (ids != std::set<int>{2} || db.getLastNodesSize() != 1 || db.getLastParameters() != info.parameters) return 94;
  std::list<Node*> restored;
  db.loadLastNodes(restored);
  const bool correct = restored.size() == 1 && restored.front()->id() == 2;
  for (auto* ptr : restored) delete ptr;
  if (!correct) return 95;
  db.executeNoResult("CREATE TRIGGER fail_snapshot BEFORE INSERT ON Info BEGIN SELECT RAISE(ABORT,'injected snapshot failure'); END;");
  resident->setWeight(42);
  auto changed = info; changed.parameters["registration.feature.pyramid.levels"] = "5";
  bool rejected = false;
  try { db.saveWorkingMemory({cold, resident}, changed); } catch (const std::exception&) { rejected = true; }
  int weight = 0; db.getWeight(2, weight);
  ids.clear(); db.getLastNodeIds(ids);
  if (!rejected || ids != std::set<int>{2} || weight != 1 || db.getLastParameters() != info.parameters) return 96;
  db.executeNoResult("DROP TRIGGER fail_snapshot;");
  // Explicitly empty is valid, leaves historical nodes intact, and survives reopen.
  db.saveWorkingMemory({}, info);
  db.closeConnection(true);
  if (!db.openConnection(path)) return 97;
  ids.clear(); db.getLastNodeIds(ids);
  if (!ids.empty() || db.getLastNodesSize() || db.getTotalNodesSize() != 2) return 98;
  db.executeNoResult("INSERT INTO WorkingMemory VALUES(1); DELETE FROM Node WHERE id=1;");
  ids.clear(); db.getLastNodeIds(ids);
  if (!ids.empty()) return 99;
  // Reject old schema explicitly, without silently reinterpreting timestamps.
  bool no_auxiliary_table = false;
  try { db.executeNoResult("SELECT user_data FROM Data;"); }
  catch (const std::exception&) { no_auxiliary_table = true; }
  if (!no_auxiliary_table) return 101;
  db.executeNoResult("PRAGMA user_version=13;");
  db.closeConnection(false);
  if (db.openConnection(path)) return 100;
  std::remove(path.c_str());
  std::fprintf(stdout, "Explicit working-memory snapshot, rollback, empty set and schema rejection passed\n");
  return 0;
}

}  // namespace

int main() {
  if (const int result = runBatchLatencyNotificationTest()) return result;
  if (const int result = runBackpressureTest(true)) return result;
  if (const int result = runWorkingMemorySnapshotTests()) return result;
  if (const int result = runBlockedCommitTest()) return result;
  if (const int result = runCommitDeadlineTest()) return result;
  if (const int result = runKeyframeBatchTests()) return result;
  if (const int result = runFlushWhileWriterBusyTest()) return result;
  if (const int result = runPendingNodeSnapshotTest()) return result;
  if (const int result = runExceptionUnlockTest()) return result;
  if (const int result = runFailedWriterDestructionTest()) return result;
  if (const int result = runDependentWriteFlushTest()) return result;
  if (const int result = runReadWriteAndSequenceTests()) {
    return result;
  }
  if (const int result = runBackpressureTest()) {
    return result;
  }
  if (const int result = runShutdownTests()) {
    return result;
  }
  if (const int result = runConcurrentReaderTest()) return result;
  return runFailureTest();
}
