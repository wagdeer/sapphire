#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <sys/wait.h>
#include <unistd.h>

#include <condition_variable>
#include <cstdarg>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <map>
#include <thread>

#include "backend/storage/map_database.hpp"
#include "backend/visual/feature/scene_features.hpp"
#include "pipeline.hpp"
#include "backend/graph/pose_graph.hpp"
using namespace sapphire;
namespace fs = std::filesystem;
namespace sapphire {
struct MapDatabaseTestAccess {
  static sqlite3 *handle(MapDatabase &db) { return db.db_; }
  static int fd(MapDatabase &db) { return db.file_fd_; }
  static void prove(MapDatabase &db) { db.proveWritableIdentity(); }
  static struct stat observe(sqlite3 *db) { return MapDatabase::observeMainFile(db); }
};
struct SlamPipelineTestAccess {
  static void replace(SlamPipeline &pipeline, const std::string &name) {
    pipeline.filename_ = name;
    fs::create_directories(fs::path(pipeline.save_path_) / name);
    pipeline.create_pose_graph();
  }
  static PoseGraphBackend *backend(SlamPipeline &pipeline) { return pipeline.pose_graph_.get(); }
};
}  // namespace sapphire
namespace {
void check(bool ok, const std::string &message) {
  if (!ok) throw std::runtime_error(message);
}
template <class F>
MapError error(F &&f) {
  try {
    f();
  } catch (const MapError &e) {
    return e;
  }
  throw std::runtime_error("Expected explicit MapError");
}
void sql(sqlite3 *db, const char *q) { check(sqlite3_exec(db, q, nullptr, nullptr, nullptr) == SQLITE_OK, sqlite3_errmsg(db)); }
std::string scalar(sqlite3 *db, const char *q) {
  database_detail::Statement s(db, q);
  check(sqlite3_step(s.get()) == SQLITE_ROW, "scalar query");
  const auto *p = sqlite3_column_text(s.get(), 0);
  return p ? reinterpret_cast<const char *>(p) : "NULL";
}
std::string logical(sqlite3 *db) {
  std::string result;
  for (const char *table : {"Node", "SpatialRecord", "LaserRecord", "PyramidVoxel", "NaviTrajectory", "FlatGrid", "ImageRecord", "VisualScene",
                            "MetaTag", "Link", "MapState"}) {
    result += table;
    database_detail::Statement s(db, ("SELECT * FROM " + std::string(table) + " ORDER BY rowid;").c_str());
    int rc;
    while ((rc = sqlite3_step(s.get())) == SQLITE_ROW) {
      result += '[';
      for (int c = 0; c < sqlite3_column_count(s.get()); ++c) {
        const int type = sqlite3_column_type(s.get(), c);
        result += std::to_string(type) + ":";
        if (type == SQLITE_INTEGER)
          result += std::to_string(sqlite3_column_int64(s.get(), c));
        else if (type == SQLITE_FLOAT) {
          double value = sqlite3_column_double(s.get(), c);
          result.append(reinterpret_cast<const char *>(&value), sizeof(value));
        } else if (type != SQLITE_NULL) {
          const auto *p = static_cast<const char *>(sqlite3_column_blob(s.get(), c));
          const int bytes = sqlite3_column_bytes(s.get(), c);
          result += std::to_string(bytes) + ":";
          if (bytes) result.append(p, bytes);
        }
        result += ';';
      }
      result += ']';
    }
    check(rc == SQLITE_DONE, "logical row scan");
  }
  return result;
}
void writeAll(int fd, const std::string &s) {
  std::size_t n = 0;
  while (n < s.size()) {
    auto r = write(fd, s.data() + n, s.size() - n);
    check(r > 0, "pipe write");
    n += r;
  }
}
std::string readAll(int fd) {
  std::string s;
  char b[8192];
  ssize_t n;
  while ((n = read(fd, b, sizeof(b))) > 0) s.append(b, n);
  check(n == 0, "pipe read");
  return s;
}
// Measurements must not close a same-inode fd in the writer process: that would
// itself cancel POSIX locks and invalidate the regression.
std::string fileSnapshot(const fs::path &root) {
  int pipefd[2];
  check(pipe(pipefd) == 0, "snapshot pipe");
  const auto child = fork();
  check(child >= 0, "snapshot fork");
  if (child == 0) {
    close(pipefd[0]);
    std::map<std::string, std::string> files;
    for (const auto &entry : fs::directory_iterator(root)) {
      if (entry.is_symlink())
        files[entry.path().filename().string()] = "link:" + fs::read_symlink(entry.path()).string();
      else if (entry.is_regular_file()) {
        std::ifstream in(entry.path(), std::ios::binary);
        files[entry.path().filename().string()] = {std::istreambuf_iterator<char>(in), {}};
      }
    }
    std::string bytes;
    for (const auto &[name, value] : files) bytes += std::to_string(name.size()) + ":" + name + std::to_string(value.size()) + ":" + value;
    writeAll(pipefd[1], bytes);
    _exit(0);
  }
  close(pipefd[1]);
  auto bytes = readAll(pipefd[0]);
  close(pipefd[0]);
  int status;
  check(waitpid(child, &status, 0) == child && status == 0, "snapshot child");
  return bytes;
}
std::size_t fdCount() { return std::distance(fs::directory_iterator("/proc/self/fd"), fs::directory_iterator{}); }
Eigen::Isometry3d anchor(int i) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.linear() = (Eigen::AngleAxisd(.31 + .005 * i, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-.17, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(.12, Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  p.translation() = Eigen::Vector3d(10 + .4 * i, -2 + .1 * i, .5 + .02 * i);
  return p;
}
LocalGrid grid(bool large = false) {
  LocalGrid g(.1f);
  g.viewPoint = {.1f, .2f, .3f};
  g.groundCells = {{0, 0}, {.1f, 0}};
  g.obstacleCells = {{.2f, 0}};
  g.emptyCells.assign(large ? 250000 : 3, GridPoint(.3f, .1f));
  return g;
}
SubmapFrame frame(int id, const Eigen::Isometry3d &a, const std::function<void(NavigationPath &)> &adjust_navigation = {}) {
  auto cloud = std::make_shared<GaussianCloud>();
  for (int i = 0; i < 16; ++i) {
    GaussianPoint p;
    p.N = 20;
    p.voxel_key.x = i;
    p.mean = {float(i % 4) * .2f, float(i / 4) * .2f, .1f};
    p.covariance = Eigen::Matrix3f::Identity() * .001f;
    p.regularize();
    cloud->push_back(p);
  }
  cpu::VoxelMaps pyramid;
  pyramid.create_voxelmaps(cloud->size(), [&](size_t i) { return (*cloud)[i].mean; });
  LioFrame lio;
  lio.timestamp = id + 1;
  lio.T_odom_base = a;
  lio.pcd = cloud;
  NavigationPath path;
  for (int i = 0; i < 5; ++i) path.samples.push_back({double(id + 1) + .01 * i, .1f * i, 0, 0, 0, 0, 0, 1, .1f * i});
  path.spline = fit_navigation_spline(path.samples, 0, .1);
  if (adjust_navigation) adjust_navigation(path);
  VisualFrame image(id + 1, 0);
  cv::Mat descriptors(96, 32, CV_8UC1);
  cv::RNG random(900 + id);
  random.fill(descriptors, cv::RNG::UNIFORM, 0, 256);
  std::vector<VisualPoint> points;
  for (int i = 0; i < 96; ++i) points.emplace_back(Eigen::Vector2f(i + 10, i + 20), i % 4, .5f);
  image.update_features(std::move(points), descriptors);
  std::vector<VisualFrame> visuals;
  visuals.push_back(std::move(image));
  SubmapFrame f(id, std::move(lio), id, id, 1, 0, 0, pyramid.release_data(), {{double(id + 1), a}}, std::move(path), std::move(visuals));
  f.attach_tag(MetaTag(MetaTagType::kBinary, descriptors.row(0).clone()));
  f.attach_tag(MetaTag(MetaTagType::kFloat, cv::Mat(1, 4, CV_32FC1, cv::Scalar(.4f))));
  return f;
}
void seed(const fs::path &path) {
  MapDatabase db(path.string());
  std::vector<GraphLink> links;
  std::vector<std::pair<int, Eigen::Isometry3f>> poses;
  for (int i = 0; i < 6; ++i) {
    auto f = frame(i, anchor(i));
    db.saveSubmap(f, grid(), buildVisualScene(f)->encode());
    poses.emplace_back(i + 1, anchor(i).cast<float>());
    if (i) links.push_back({i, i + 1, 0, (anchor(i - 1).inverse() * anchor(i)).cast<float>()});
  }
  db.saveSubmapPoses(poses, links);
  db.finish();
}
std::string identity() { return map_config_identity(PoseGraphParameters{}, NaviMapParameters{}); }
std::uint64_t append(MapDatabase &db, bool large = false) {
  auto f = frame(6, anchor(6));
  auto first = anchor(0).cast<float>();
  first.translation().x() += .0001f;
  GraphLink base{6, 7, 0, (anchor(5).inverse() * anchor(6)).cast<float>()};
  GraphLink loop{7, 1, 1, (anchor(6).inverse() * Eigen::Isometry3d(first.cast<double>())).cast<float>()};
  return db.commitFinalizedSubmap(f, grid(large), buildVisualScene(f)->encode(), {{1, first}, {7, anchor(6).cast<float>()}}, base, loop, db.mapUuid(),
                                  db.committedRevision(), 7, identity(), 1);
}
std::function<void(const char *, void *)> hook;
std::atomic<int> raw_opens{0}, raw_closes{0}, immediate_begins{0};
bool fail_rollback = false, fail_commit_before = false, fail_commit_after = false, force_readonly = false, fail_checkpoint = false, fail_mode = false;
bool death_commit_hook = false, death_commit_return = false;
std::string capability;
sqlite3 *observing_db = nullptr;
int nested_rejections = 0;
const sqlite3_io_methods *real_methods = nullptr;
thread_local sqlite3_io_methods test_methods;
int fileSizeShim(sqlite3_file *file, sqlite3_int64 *size) {
  if (capability == "zero") {
    *size = 0;
    return SQLITE_OK;
  }
  if (capability == "failed-observation") {
    errno = EIO;
    return SQLITE_IOERR_FSTAT;
  }
  if (capability == "nested" && nested_rejections == 0) {
    const auto e = error([&] { MapDatabaseTestAccess::observe(observing_db); });
    check(e.code() == MapErrorCode::Lifecycle, "nested TLS rejection");
    ++nested_rejections;
  }
  int rc = real_methods->xFileSize(file, size);
  if (capability == "multiple") rc = real_methods->xFileSize(file, size);
  return rc;
}
int commitHook(void *) {
  if (death_commit_hook) kill(getpid(), SIGKILL);
  return 0;
}
struct Latch {
  std::mutex mutex;
  std::condition_variable cv;
  bool reached = false, released = false;
  void pause() {
    std::unique_lock<std::mutex> lock(mutex);
    reached = true;
    cv.notify_all();
    cv.wait(lock, [&] { return released; });
  }
  void wait() {
    std::unique_lock<std::mutex> lock(mutex);
    check(cv.wait_for(lock, std::chrono::seconds(10), [&] { return reached; }), "barrier not reached");
  }
  void release() {
    std::lock_guard<std::mutex> lock(mutex);
    released = true;
    cv.notify_all();
  }
};
}  // namespace
namespace sapphire::database_detail {
void writableStorageTestPoint(const char *point, void *context) {
  if (hook) hook(point, context);
}
}  // namespace sapphire::database_detail
extern "C" {
int __real_open(const char *, int, ...);
int __real_close(int);
int __real_sqlite3_exec(sqlite3 *, const char *, int (*)(void *, int, char **, char **), void *, char **);
int __real_sqlite3_open_v2(const char *, sqlite3 **, int, const char *);
int __real_sqlite3_file_control(sqlite3 *, const char *, int, void *);
sqlite3_vfs *__real_sqlite3_vfs_find(const char *);
sqlite3_mutex *__real_sqlite3_db_mutex(sqlite3 *);
sqlite3_mutex *__wrap_sqlite3_db_mutex(sqlite3 *db) { return capability == "no-mutex" ? nullptr : __real_sqlite3_db_mutex(db); }
int __real_sqlite3_step(sqlite3_stmt *);
int __wrap_sqlite3_step(sqlite3_stmt *s) {
  if (fail_mode && std::string(sqlite3_sql(s)).find("PRAGMA journal_mode") == 0) return SQLITE_BUSY;
  return __real_sqlite3_step(s);
}
int __real_sqlite3_wal_checkpoint_v2(sqlite3 *, const char *, int, int *, int *);
int __wrap_open(const char *path, int flags, ...) {
  ++raw_opens;
  mode_t mode = 0;
  if (flags & O_CREAT) {
    va_list args;
    va_start(args, flags);
    mode = va_arg(args, int);
    va_end(args);
  }
  return __real_open(path, flags, mode);
}
int __wrap_close(int fd) {
  ++raw_closes;
  return __real_close(fd);
}
int __wrap_sqlite3_exec(sqlite3 *db, const char *q, int (*cb)(void *, int, char **, char **), void *arg, char **message) {
  if (std::strcmp(q, "BEGIN IMMEDIATE;") == 0) ++immediate_begins;
  if (fail_rollback && std::strcmp(q, "ROLLBACK;") == 0) return SQLITE_IOERR;
  const bool commit = std::strcmp(q, "COMMIT;") == 0;
  if (commit && fail_commit_before) return SQLITE_IOERR;
  const int rc = __real_sqlite3_exec(db, q, cb, arg, message);
  if (commit && death_commit_return && rc == SQLITE_OK) kill(getpid(), SIGKILL);
  if (commit && fail_commit_after && rc == SQLITE_OK) return SQLITE_IOERR;
  return rc;
}
int __wrap_sqlite3_open_v2(const char *path, sqlite3 **db, int flags, const char *vfs) {
  if (force_readonly && vfs && std::strcmp(vfs, "unix") == 0) flags = (flags & ~SQLITE_OPEN_READWRITE) | SQLITE_OPEN_READONLY;
  return __real_sqlite3_open_v2(path, db, flags, vfs);
}
int __wrap_sqlite3_file_control(sqlite3 *db, const char *schema, int op, void *arg) {
  const int rc = __real_sqlite3_file_control(db, schema, op, arg);
  if (rc == SQLITE_OK && op == SQLITE_FCNTL_FILE_POINTER && !capability.empty() && capability != "unsupported" && capability != "missing") {
    auto *file = *static_cast<sqlite3_file **>(arg);
    if (file->pMethods != &test_methods) real_methods = file->pMethods;
    test_methods = *real_methods;
    test_methods.xFileSize = capability == "missing-method" ? nullptr : fileSizeShim;
    file->pMethods = &test_methods;
    observing_db = db;
  }
  return rc;
}
sqlite3_vfs *__wrap_sqlite3_vfs_find(const char *name) {
  if (capability == "unsupported" && name && std::strcmp(name, "unix") == 0) return __real_sqlite3_vfs_find("unix-none");
  return __real_sqlite3_vfs_find(name);
}
int __wrap_sqlite3_wal_checkpoint_v2(sqlite3 *db, const char *schema, int mode, int *a, int *b) {
  if (fail_checkpoint) return SQLITE_BUSY;
  return __real_sqlite3_wal_checkpoint_v2(db, schema, mode, a, b);
}
}

namespace {
fs::path copy(const fs::path &source, const fs::path &root, const std::string &name) {
  auto p = root / (name + ".db");
  fs::copy_file(source, p);
  return p;
}
void rejected(const fs::path &path, const char *mode) {
  const int opens = raw_opens, closes = raw_closes;
  check(error([&] { MapDatabase db(path.string(), mode); }).code() == MapErrorCode::WriterConflict, "admission conflict");
  check(raw_opens == opens && raw_closes == closes, "denied constructor touched raw fd");
}
void lockPresent(MapDatabase &db, bool exclusive) {
  struct flock lock {};
  lock.l_type = F_WRLCK;
  lock.l_whence = SEEK_SET;
  lock.l_start = exclusive ? 0x40000002 : 0x40000001;
  lock.l_len = exclusive ? 510 : 1;
  check(fcntl(MapDatabaseTestAccess::fd(db), F_OFD_GETLK, &lock) == 0, "kernel lock query");
  check(lock.l_type == F_WRLCK && lock.l_pid == getpid(), "SQLite POSIX lock was lost: type=" + std::to_string(lock.l_type) +
                                                              " pid=" + std::to_string(lock.l_pid) + " expected=" + std::to_string(getpid()));
}
void admission(const fs::path &source, const fs::path &root) {
  const auto p = copy(source, root, "admission"), other = copy(source, root, "unrelated");
  fs::create_hard_link(p, root / "alias-before-rename");
  fs::rename(root / "alias-before-rename", root / "alias");
  fs::create_symlink(p, root / "symlink");
  for (const char *entry : {"resume", "recover", "new"}) {
    auto writer = std::make_unique<MapDatabase>((std::string(entry) == "new" ? root / "brand-new.db" : p).string(), entry);
    if (std::string(entry) == "resume") writer->promoteToWritable();
    auto *db = MapDatabaseTestAccess::handle(*writer);
    sql(db, "PRAGMA journal_mode=DELETE;");
    for (bool ex : {false, true}) {
      sql(db, ex ? "BEGIN EXCLUSIVE;" : "BEGIN IMMEDIATE;");
      lockPresent(*writer, ex);
      auto before = fileSnapshot(root);
      for (int repeat = 0; repeat < 10; ++repeat)
        for (const auto &path : {p, other, root / "alias", root / "symlink", root / "missing"})
          for (const char *mode : {"resume", "new", "recover"}) rejected(path, mode);
      lockPresent(*writer, ex);
      check(!sqlite3_get_autocommit(db), "transaction lost");
      check(fileSnapshot(root) == before, "rejected admission changed files");
      sql(db, "ROLLBACK;");
    }
    writer->finish();
  }
  {
    MapDatabase a(p.string(), "resume"), b(other.string(), "resume");
    check(error([&] { a.promoteToWritable(); }).code() == MapErrorCode::WriterConflict, "multiple readers promoted");
    check(a.state() == MapDatabase::State::HistoricalReadOnly, "denied promotion damaged history");
    rejected(root / "new-denied", "new");
  }
  std::cout << "PASS admission: 300 rejected openers per writer mode, RESERVED/EXCLUSIVE locks and exact files preserved\n";
}
void races(const fs::path &source, const fs::path &root) {
  const auto p = copy(source, root, "races"), other = copy(source, root, "races-other");
  for (const std::string point : {"admission-reserved", "owner-opened", "validating", "before-sqlite-close"}) {
    MapDatabase target(p.string(), "resume");
    Latch latch;
    std::exception_ptr unexpected;
    const auto main = std::this_thread::get_id();
    hook = [&](const char *at, void *) {
      if (std::this_thread::get_id() == main) return;
      if (point == "before-sqlite-close" && std::string(at) == "validating") throw std::runtime_error("constructor failure");
      if (at == point) latch.pause();
    };
    std::thread worker([&] {
      try {
        MapDatabase reader(other.string(), "resume");
      } catch (...) {
        if (point != "before-sqlite-close") unexpected = std::current_exception();
      }
    });
    latch.wait();
    const auto opens = raw_opens.load(), closes = raw_closes.load();
    check(error([&] { target.promoteToWritable(); }).code() == MapErrorCode::WriterConflict, "reservation race A");
    check(raw_opens == opens && raw_closes == closes, "promotion opened while shared reservation");
    latch.release();
    worker.join();
    hook = {};
    if (unexpected) std::rethrow_exception(unexpected);
  }
  {
    Latch latch;
    std::exception_ptr failure;
    hook = [&](const char *at, void *) {
      if (std::string(at) == "admission-reserved") latch.pause();
    };
    std::thread worker([&] {
      try {
        MapDatabase db((root / "race-new").string());
      } catch (...) {
        failure = std::current_exception();
      }
    });
    latch.wait();
    rejected(p, "resume");
    latch.release();
    worker.join();
    hook = {};
    if (failure) std::rethrow_exception(failure);
  }
  // Complete close retains admission at every resource boundary, including after raw close.
  for (const std::string point : {"before-sqlite-close", "after-sqlite-close", "after-owner-close"}) {
    auto db = std::make_unique<MapDatabase>(p.string(), "resume");
    db->promoteToWritable();
    Latch latch;
    hook = [&](const char *at, void *) {
      if (at == point) latch.pause();
    };
    std::thread worker([&] { db.reset(); });
    latch.wait();
    rejected(other, "resume");
    latch.release();
    worker.join();
    hook = {};
    MapDatabase reader(other.string(), "resume");
  }
  // Real independent shared flock prevents destructive SH->EX conversion.
  {
    MapDatabase db(p.string(), "resume");
    int ready[2], done[2];
    check(pipe(ready) == 0 && pipe(done) == 0, "flock pipes");
    pid_t child = fork();
    check(child >= 0, "flock fork");
    if (child == 0) {
      close(ready[0]);
      close(done[1]);
      int fd = open(p.c_str(), O_RDONLY);
      if (fd < 0 || flock(fd, LOCK_SH | LOCK_NB)) _exit(3);
      writeAll(ready[1], "r");
      close(ready[1]);
      char c;
      if (read(done[0], &c, 1) != 1) _exit(4);
      close(fd);
      _exit(0);
    }
    close(ready[1]);
    close(done[0]);
    check(readAll(ready[0]) == "r", "flock ready");
    close(ready[0]);
    Latch latch;
    hook = [&](const char *at, void *) {
      if (std::string(at) == "before-sqlite-close") latch.pause();
    };
    MapErrorCode code = MapErrorCode::Storage;
    std::thread worker([&] { code = error([&] { db.promoteToWritable(); }).code(); });
    latch.wait();
    rejected(other, "resume");
    latch.release();
    worker.join();
    hook = {};
    check(code == MapErrorCode::WriterConflict && db.state() == MapDatabase::State::Failed, "flock failure resurrected history");
    writeAll(done[1], "d");
    close(done[1]);
    int status;
    waitpid(child, &status, 0);
    check(status == 0, "flock child");
  }
  // Partial open/proof cleanup must retain admission through all resource closes.
  for (const std::string failure : {"before-sqlite-open", "before-file-proof"})
    for (const std::string point : {"before-sqlite-close", "after-sqlite-close", "after-owner-close"}) {
      MapDatabase db(p.string(), "resume");
      Latch latch;
      hook = [&](const char *at, void *) {
        if (at == failure) throw std::runtime_error("open/proof failure");
        if (at == point) latch.pause();
      };
      std::thread worker([&] {
        try {
          db.promoteToWritable();
        } catch (...) {
        }
      });
      latch.wait();
      rejected(other, "recover");
      latch.release();
      worker.join();
      hook = {};
      check(db.state() == MapDatabase::State::Failed, "partial-open failure state");
    }
  {
    MapDatabase db(p.string(), "resume");
    db.promoteToWritable();
    sqlite3_stmt *held = nullptr;
    check(sqlite3_prepare_v2(MapDatabaseTestAccess::handle(db), "SELECT * FROM Node", -1, &held, nullptr) == SQLITE_OK, "held statement");
    check(error([&] { db.finish(); }).code() == MapErrorCode::Lifecycle, "BUSY close not visible");
    check(db.state() == MapDatabase::State::Failed && MapDatabaseTestAccess::fd(db) >= 0, "BUSY released resources");
    rejected(other, "resume");
    sqlite3_finalize(held);
    db.finish();
  }
  std::cout << "PASS deterministic admission races A-E, failed conversion and BUSY complete-close retention\n";
}
std::string recovered(const fs::path &p) {
  MapDatabase db(p.string(), "recover");
  auto *handle = MapDatabaseTestAccess::handle(db);
  check(scalar(handle, "PRAGMA integrity_check;") == "ok", "recovery integrity");
  const auto result = logical(handle);
  db.finish();
  return result;
}
void checkIndependentReopen(const fs::path &p, float resolution = .1f, bool runtime = true);
void atomic(const fs::path &source, const fs::path &root) {
  std::string old, next;
  {
    MapDatabase db(source.string(), "resume");
    old = logical(MapDatabaseTestAccess::handle(db));
  }
  const auto good = copy(source, root, "atomic-good");
  {
    MapDatabase db(good.string(), "resume");
    db.promoteToWritable();
    auto *handle = MapDatabaseTestAccess::handle(db);
    check(scalar(handle, "PRAGMA journal_mode;") == "delete", "promotion mutated mode");
    const auto original_odom = scalar(handle, "SELECT hex(odom_poses) FROM Node WHERE id=1;");
    const auto previous_pose = scalar(handle, "SELECT hex(submap_pose) FROM Node WHERE id=1;");
    check(append(db) == 2, "revision not incremented once");
    check(scalar(handle, "SELECT hex(odom_poses) FROM Node WHERE id=1;") == original_odom &&
              scalar(handle, "SELECT hex(submap_pose) FROM Node WHERE id=1;") != previous_pose,
          "changed committed pose overwrote original odometry or was not applied");
    check(scalar(handle, "PRAGMA journal_mode;") == "wal", "first commit omitted WAL policy");
    next = logical(handle);
    check(next != old, "no committed change");
    db.finish();
  }
  checkIndependentReopen(good);
  check(recovered(good) == next, "successful full rows changed on reopen");
  int index = 0;
  for (const std::string point : {"prepared-before-transaction", "after-node", "after-spatial", "after-cloud", "after-pyramid", "after-navigation",
                                  "after-grid", "after-images", "after-scene", "after-tags", "after-factors", "during-pose-updates",
                                  "during-tag-updates", "before-revision", "before-commit", "after-commit", "before-wal-activation"}) {
    auto p = copy(source, root, "atomic-" + std::to_string(index++));
    MapDatabase db(p.string(), "resume");
    db.promoteToWritable();
    hook = [&](const char *at, void *) {
      if (at == point) throw std::runtime_error("injected " + point);
    };
    auto e = error([&] { append(db); });
    hook = {};
    const bool committed = point == "after-commit" || point == "before-wal-activation";
    check(e.outcome() == (committed ? CommitOutcome::Committed : CommitOutcome::NotCommitted), "failure outcome " + point);
    check(db.state() == MapDatabase::State::Failed, "nonsticky storage failure");
    check(logical(MapDatabaseTestAccess::handle(db)) == (committed ? next : old), "full row rollback/commit mismatch " + point);
    db.finish();
    check(recovered(p) == (committed ? next : old), "reopen logical mismatch " + point);
  }
  for (int kind = 0; kind < 3; ++kind) {
    auto p = copy(source, root, "uncertain-" + std::to_string(kind));
    MapDatabase db(p.string(), "resume");
    db.promoteToWritable();
    fail_commit_before = kind == 0;
    fail_commit_after = kind == 1;
    fail_rollback = kind == 2;
    if (kind == 2)
      hook = [](const char *at, void *) {
        if (std::string(at) == "after-grid") throw std::runtime_error("rollback failure");
      };
    auto e = error([&] { append(db); });
    hook = {};
    fail_commit_before = fail_commit_after = fail_rollback = false;
    check(e.outcome() == CommitOutcome::Unknown, "uncertain COMMIT claimed rollback");
    check(error([&] { append(db); }).code() == MapErrorCode::Lifecycle, "failed writer retried");
    db.finish();
    check(recovered(p) == (kind == 1 ? next : old), "uncertain authoritative reopen");
  }
  for (int kind = 0; kind < 7; ++kind) {
    auto p = copy(source, root, "stale-" + std::to_string(kind));
    MapDatabase db(p.string(), "resume");
    db.promoteToWritable();
    auto f = frame(6, anchor(6));
    GraphLink base{6, 7, 0, (anchor(5).inverse() * anchor(6)).cast<float>()};
    std::vector<std::pair<int, Eigen::Isometry3f>> poses{{7, anchor(6).cast<float>()}};
    if (kind == 4) poses.push_back(poses.front());
    if (kind == 5) base = {7, 7, 1, Eigen::Isometry3f::Identity()};
    error([&] {
      db.commitFinalizedSubmap(f, grid(), buildVisualScene(f)->encode(), poses, base, std::nullopt, kind == 0 ? "wrong" : db.mapUuid(),
                               kind == 1 ? 0 : 1, kind == 2 ? 8 : 7, kind == 3 ? "wrong" : identity(), kind == 6 ? 7 : 1);
    });
    check(logical(MapDatabaseTestAccess::handle(db)) == old, "stale/duplicate preparation changed rows");
    db.finish();
  }
  for (bool checkpoint : {true, false}) {
    auto p = copy(source, root, checkpoint ? "finish-checkpoint" : "finish-mode");
    MapDatabase db(p.string(), "resume");
    db.promoteToWritable();
    fail_checkpoint = checkpoint;
    fail_mode = !checkpoint;
    error([&] { db.finish(); });
    fail_checkpoint = fail_mode = false;
    check(db.state() == MapDatabase::State::Failed, "finish failure invisible");
    rejected(source, "resume");
    db.finish();
  }
  std::cout
      << "PASS populated atomic append, 17 fault boundaries, full row/blob equality, stale/duplicate inputs, unknown COMMIT and checked finish\n";
}
int runChild(const std::vector<std::string> &args, bool killed = false) {
  pid_t child = fork();
  check(child >= 0, "exec fork");
  if (child == 0) {
    std::vector<char *> argv;
    argv.push_back(const_cast<char *>("/proc/self/exe"));
    for (const auto &arg : args) argv.push_back(const_cast<char *>(arg.c_str()));
    argv.push_back(nullptr);
    execv(argv[0], argv.data());
    _exit(126);
  }
  int status;
  check(waitpid(child, &status, 0) == child, "exec wait");
  if (killed)
    check(WIFSIGNALED(status) && WTERMSIG(status) == SIGKILL, "child did not die at boundary");
  else
    check(WIFEXITED(status) && WEXITSTATUS(status) == 0, "child failed");
  return status;
}
void crashChild(const std::string &point, const fs::path &p) {
  MapDatabase db(p.string(), "recover");
  auto *h = MapDatabaseTestAccess::handle(db);
  sql(h, "PRAGMA cache_size=2;");
  sql(h, "PRAGMA cache_spill=ON;");
  if (point == "wal-spill") sql(h, "PRAGMA journal_mode=WAL;");
  if (point == "during-commit") {
    death_commit_hook = true;
    sqlite3_commit_hook(h, commitHook, nullptr);
  }
  if (point == "commit-return") death_commit_return = true;
  hook = [&](const char *at, void *) {
    if (point == at || (point == "wal-spill" && std::string(at) == "after-grid")) kill(getpid(), SIGKILL);
  };
  append(db, true);
  throw std::runtime_error("missed death point");
}
void crashes(const fs::path &source, const fs::path &root) {
  std::string old, next;
  {
    MapDatabase db(source.string(), "resume");
    old = logical(MapDatabaseTestAccess::handle(db));
  }
  auto canonical = copy(source, root, "crash-next");
  {
    MapDatabase db(canonical.string(), "recover");
    append(db, true);
    next = logical(MapDatabaseTestAccess::handle(db));
    db.finish();
  }
  for (const std::string point : {"after-node", "after-grid", "wal-spill", "before-commit", "during-commit", "commit-return", "after-commit"}) {
    auto p = copy(source, root, "death-" + point);
    runChild({"--crash", point, p.string()}, true);
    if (point == "wal-spill") check(fs::file_size(p.string() + "-wal") > 0, "no real WAL spill");
    if (fs::exists(p.string() + "-wal") || fs::exists(p.string() + "-journal")) error([&] { MapDatabase historical(p.string(), "resume"); });
    auto actual = recovered(p);
    const bool committed = point == "commit-return" || point == "after-commit";
    check(actual == (committed ? next : old), "crash produced partial revision " + point);
    MapDatabase validated(p.string(), "recover");
    validated.validateHistoricalRecords(.1f);
    validated.finish();
  }
  std::cout << "PASS seven SIGKILL boundaries: rollback page spill, WAL spill, COMMIT hook/return and postcommit; actual recovery, integrity and "
               "full blobs\n";
}
}  // namespace
namespace {
using OpenCall = int (*)(const char *, int, int);
OpenCall original_open = nullptr;
std::string race_path, race_saved, race_wrong;
bool race_armed = false, race_aba = false;
int raceOpen(const char *path, int flags, int mode) {
  if (!race_armed || race_path != path) return original_open(path, flags, mode);
  race_armed = false;
  if (rename(race_path.c_str(), race_saved.c_str()) || rename(race_wrong.c_str(), race_path.c_str())) return -1;
  int fd = original_open(path, flags, mode);
  const int saved_errno = errno;
  if (race_aba) {
    if (rename(race_path.c_str(), race_wrong.c_str()) || rename(race_saved.c_str(), race_path.c_str())) std::terminate();
  }
  errno = saved_errno;
  return fd;
}
using FstatCall = int (*)(int, struct stat *);
FstatCall installed_fstat = nullptr;
int replacementFstat(int fd, struct stat *s) { return installed_fstat(fd, s); }
sqlite3_syscall_ptr (*real_get)(sqlite3_vfs *, const char *) = nullptr;
sqlite3_syscall_ptr missingFstat(sqlite3_vfs *v, const char *name) { return std::strcmp(name, "fstat") == 0 ? nullptr : real_get(v, name); }
void capabilityChild(const std::string &kind, const fs::path &p) {
  capability = kind;
  auto *vfs = __real_sqlite3_vfs_find("unix");
  if (kind == "missing") {
    real_get = vfs->xGetSystemCall;
    vfs->xGetSystemCall = missingFstat;
  }
  {
    MapDatabase history(p.string(), "resume");
    history.validateHistoricalRecords(.1f);
  }
  {
    MapDatabase db(p.string(), "resume");
    if (kind == "nested") {
      db.promoteToWritable();
      check(nested_rejections > 0, "nested observation untested");
      capability.clear();
      MapDatabaseTestAccess::prove(db);
      db.finish();
    } else {
      error([&] { db.promoteToWritable(); });
      check(db.state() == MapDatabase::State::Failed, "capability failure not closed");
      capability.clear();
    }
  }
  { MapDatabase history(p.string(), "resume"); }
}
void identities(const fs::path &source, const fs::path &root) {
  for (const std::string kind : {"unsupported", "missing", "zero", "multiple", "missing-method", "failed-observation", "no-mutex", "nested"}) {
    auto p = copy(source, root, "capability-" + kind);
    runChild({"--capability", kind, p.string()});
  }
  {
    auto p = copy(source, root, "before-promotion");
    MapDatabase db(p.string(), "resume");
    fs::rename(p, root / "before-original");
    fs::copy_file(source, p);
    error([&] { db.promoteToWritable(); });
    check(db.state() == MapDatabase::State::Failed, "pathname replacement not fail-stop");
  }
  auto *vfs = sqlite3_vfs_find("unix");
  original_open = reinterpret_cast<OpenCall>(vfs->xGetSystemCall(vfs, "open"));
  for (bool aba : {false, true}) {
    auto p = copy(source, root, aba ? "aba" : "permanent");
    auto wrong = copy(source, root, aba ? "aba-wrong" : "permanent-wrong");
    MapDatabase db(p.string(), "resume");
    race_path = p;
    race_saved = p.string() + "-saved";
    race_wrong = wrong;
    race_aba = aba;
    race_armed = true;
    check(vfs->xSetSystemCall(vfs, "open", reinterpret_cast<sqlite3_syscall_ptr>(raceOpen)) == SQLITE_OK, "open race hook");
    bool wrong_inode_observed = false;
    hook = [&](const char *at, void *) {
      if (std::string(at) == "before-file-proof") {
        struct stat owner {};
        check(fstat(MapDatabaseTestAccess::fd(db), &owner) == 0, "retained owner identity");
        const auto actual = MapDatabaseTestAccess::observe(MapDatabaseTestAccess::handle(db));
        wrong_inode_observed = actual.st_dev != owner.st_dev || actual.st_ino != owner.st_ino;
      }
    };
    auto e = error([&] { db.promoteToWritable(); });
    hook = {};
    check(wrong_inode_observed && e.code() == MapErrorCode::StaleMap, "wrong actual inode did not reach production identity proof");
    vfs->xSetSystemCall(vfs, "open", reinterpret_cast<sqlite3_syscall_ptr>(original_open));
    check(!race_armed && db.state() == MapDatabase::State::Failed, "actual SQLite open replacement escaped");
    check(!fs::exists(p.string() + "-wal") && !fs::exists(p.string() + "-journal"), "identity failure ran application SQL");
  }
  {
    auto p = copy(source, root, "readonly");
    MapDatabase db(p.string(), "resume");
    force_readonly = true;
    error([&] { db.promoteToWritable(); });
    force_readonly = false;
    check(db.state() == MapDatabase::State::Failed, "readonly fallback accepted");
  }
  installed_fstat = reinterpret_cast<FstatCall>(vfs->xGetSystemCall(vfs, "fstat"));
  {
    auto p = copy(source, root, "replaced-hook");
    MapDatabase db(p.string(), "resume");
    vfs->xSetSystemCall(vfs, "fstat", reinterpret_cast<sqlite3_syscall_ptr>(replacementFstat));
    error([&] { db.promoteToWritable(); });
    { MapDatabase history(source.string(), "resume"); }
    vfs->xSetSystemCall(vfs, "fstat", reinterpret_cast<sqlite3_syscall_ptr>(installed_fstat));
  }
  // Exercise the production connection/TLS observer concurrently. These isolated
  // test-only SQLite handles have no MapDatabase owner or live production writer.
  sqlite3 *a = nullptr, *b = nullptr;
  check(sqlite3_open_v2(source.c_str(), &a, SQLITE_OPEN_READWRITE | SQLITE_OPEN_FULLMUTEX, "unix") == SQLITE_OK, "concurrent A");
  check(sqlite3_open_v2(source.c_str(), &b, SQLITE_OPEN_READWRITE | SQLITE_OPEN_FULLMUTEX, "unix") == SQLITE_OK, "concurrent B");
  std::exception_ptr ea, eb;
  auto observe = [](sqlite3 *db, std::exception_ptr &failure) {
    try {
      for (int i = 0; i < 1000; ++i) check(S_ISREG(MapDatabaseTestAccess::observe(db).st_mode), "concurrent identity");
    } catch (...) {
      failure = std::current_exception();
    }
  };
  std::thread ta(observe, a, std::ref(ea)), tb(observe, b, std::ref(eb));
  ta.join();
  tb.join();
  sqlite3_close(a);
  sqlite3_close(b);
  if (ea) std::rethrow_exception(ea);
  if (eb) std::rethrow_exception(eb);
  std::cout
      << "PASS actual-open permanent/ABA byte-identical inode substitution, platform/hook capabilities, nested/concurrent TLS, readonly fallback\n";
}
void topology(const fs::path &source, const fs::path &root) {
  auto p = copy(source, root, "multi-chain");
  {
    MapDatabase db(p.string(), "resume");
    db.promoteToWritable();
    for (int id = 7; id <= 9; ++id) {
      auto a = anchor(id - 1);
      a.translation().x() -= 20;
      auto f = frame(id - 1, a);
      GraphLink base = id == 8 ? GraphLink{7, 8, 0, (Eigen::Isometry3d((anchor(6).matrix()).eval()).inverse() * anchor(7)).cast<float>()}
                               : GraphLink{id, id == 7 ? 6 : 1, 1, (anchor(id - 1).inverse() * anchor(id == 7 ? 5 : 0)).cast<float>()};
      // Both anchors of the real continuation share the same original odometry domain.
      if (id == 8) {
        auto previous = anchor(6);
        previous.translation().x() -= 20;
        base.transform = (previous.inverse() * a).cast<float>();
      }
      check(db.commitFinalizedSubmap(f, grid(), buildVisualScene(f)->encode(), {{id, anchor(id - 1).cast<float>()}}, base, std::nullopt, db.mapUuid(),
                                     db.committedRevision(), id, identity(), id == 8 ? 7 : id) == std::uint64_t(id - 5),
            "multi-chain revision");
    }
    db.finish();
  }
  {
    MapDatabase db(p.string(), "resume");
    db.validateHistoricalRecords(.1f);
  }
  PoseGraphParameters config;
  config.map_mode = "resume";
  PoseGraphBackend backend(config, NaviMapParameters{}, p.string(), {});
  check(backend.loopDecisionStatus() == LoopDecisionStatus::Unavailable, "invented provenance");
  check(backend.reconstructionDiagnostics().original_prior.has_value(), "lost original prior");
  backend.finish();
  checkIndependentReopen(p);
  std::cout << "PASS multiple chain roots including adjacent attachment, continuation anchors and A2 fresh graph reconstruction\n";
}
void backendFailures(const fs::path &source, const fs::path &root) {
  for (bool postcommit : {false, true}) {
    auto p = copy(source, root, postcommit ? "backend-post" : "backend-speculative");
    PoseGraphParameters config;
    config.map_mode = "resume";
    PoseGraphBackend backend(config, NaviMapParameters{}, p.string(), {});
    backend.promoteStorageToWritable();
    bool solver_changed = false;
    hook = [&](const char *point, void *context) {
      if (std::string(point) == "tentative-backend-solver") {
        auto *isam = static_cast<gtsam::ISAM2 *>(context);
        auto before = isam->getFactorsUnsafe().size();
        gtsam::NonlinearFactorGraph factors;
        factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(anchor(0).matrix()), gtsam::noiseModel::Isotropic::Sigma(6, .5)));
        isam->update(factors);
        solver_changed = isam->getFactorsUnsafe().size() == before + 1;
      }
      if (postcommit && std::string(point) == "backend-postcommit-materialization") throw std::runtime_error("runtime materialization");
    };
    auto f = frame(6, anchor(6));
    GraphLink base{6, 7, 0, (anchor(5).inverse() * anchor(6)).cast<float>()};
    auto e = error([&] {
      backend.commitFinalizedSubmap(f, grid(), buildVisualScene(f)->encode(), {{7, anchor(6).cast<float>()}}, base, std::nullopt, backend.mapUuid(),
                                    postcommit ? 1 : 0, 7, 1);
    });
    hook = {};
    check(solver_changed && backend.failed() && !backend.enabled(), "speculative failure not sticky");
    check(backend.graphRevision() == (postcommit ? 2 : 1), "backend lost committed revision");
    if (postcommit) check(e.outcome() == CommitOutcome::Committed, "runtime failure claimed rollback");
    check(error([&] { backend.addFrame(frame(7, anchor(7))); }).code() == MapErrorCode::Lifecycle, "failed backend accepted mapping");
    error([&] { backend.committedPose(0); });
    try {
      backend.finish();
      throw std::runtime_error("sticky finish missing");
    } catch (const MapError &) {
    } catch (const std::runtime_error &e) {
      check(std::string(e.what()) == "runtime materialization", "unexpected finish error");
    }
    {
      MapDatabase db(p.string(), "resume");
      db.validateHistoricalRecords(.1f);
    }
  }
  std::cout << "PASS actual tentative ISAM mutation + stale reject and committed revision + runtime failure are sticky\n";
}
void pipeline(const fs::path &root) {
  SapphireParameters parameters;
  parameters.general.save_path = root.string();
  parameters.general.save_map = 0;
  parameters.pose_graph.enabled = true;
  {
    SlamPipeline pipeline(parameters);
    bool destroyed = false;
    hook = [&](const char *at, void *) {
      if (std::string(at) == "pipeline-old-backend-destroyed") {
        MapDatabase proof((root / "replacement-admission-proof").string());
        proof.finish();
        destroyed = true;
      }
    };
    SlamPipelineTestAccess::replace(pipeline, "replacement");
    hook = {};
    check(destroyed && !pipeline.failed(), "pipeline replacement ordering");
    pipeline.shutdown();
  }
  {
    parameters.general.save_path = (root / "finish-failure").string();
    fs::create_directory(parameters.general.save_path);
    SlamPipeline pipeline(parameters);
    hook = [](const char *at, void *) {
      if (std::string(at) == "before-checkpoint") throw std::runtime_error("finish failure");
    };
    const auto opens = raw_opens.load();
    try {
      SlamPipelineTestAccess::replace(pipeline, "forbidden");
      throw std::logic_error("replacement succeeded");
    } catch (const std::runtime_error &) {
    }
    hook = {};
    check(pipeline.failed() && raw_opens == opens, "pipeline constructed after failed finish");
    pipeline.shutdown();
  }
  {
    parameters.general.save_path = (root / "construction-failure").string();
    fs::create_directory(parameters.general.save_path);
    SlamPipeline pipeline(parameters);
    fs::create_directory(fs::path(parameters.general.save_path) / "collision");
    std::ofstream(fs::path(parameters.general.save_path) / "collision/map.db") << "occupied";
    error([&] { SlamPipelineTestAccess::replace(pipeline, "collision"); });
    check(pipeline.failed() && !SlamPipelineTestAccess::backend(pipeline), "pipeline resurrected old backend");
    pipeline.shutdown();
  }
  std::cout << "PASS GPU pipeline destroys old owner before replacement; failed finish/construction stops explicitly\n";
}
}  // namespace
namespace {
void additionalBoundaries(const fs::path &source, const fs::path &root) {
  // A sidecar appearing after historical preparation is never recovered by promotion.
  {
    auto p = copy(source, root, "handoff-sidecar");
    MapDatabase db(p.string(), "resume");
    hook = [&](const char *at, void *) {
      if (std::string(at) == "before-sqlite-open") std::ofstream(p.string() + "-journal") << "appeared";
    };
    error([&] { db.promoteToWritable(); });
    hook = {};
    check(db.state() == MapDatabase::State::Failed && fs::file_size(p.string() + "-journal") == 8, "handoff silently recovered/deleted sidecar");
  }
  {
    auto p = copy(source, root, "promoted-exclusive");
    MapDatabase db(p.string(), "resume");
    Latch latch;
    hook = [&](const char *at, void *) {
      if (std::string(at) == "exclusive-promotion") latch.pause();
    };
    std::exception_ptr failure;
    std::thread worker([&] {
      try {
        db.promoteToWritable();
      } catch (...) {
        failure = std::current_exception();
      }
    });
    latch.wait();
    rejected(source, "resume");
    latch.release();
    worker.join();
    hook = {};
    if (failure) std::rethrow_exception(failure);
    db.finish();
  }
  // A genuine foreign SQLite RESERVED lock makes our BEGIN fail with BUSY.
  {
    auto p = copy(source, root, "busy-begin");
    MapDatabase db(p.string(), "resume");
    db.promoteToWritable();
    const auto old = logical(MapDatabaseTestAccess::handle(db));
    int ready[2], done[2];
    check(pipe(ready) == 0 && pipe(done) == 0, "BUSY pipes");
    pid_t child = fork();
    check(child >= 0, "BUSY fork");
    if (child == 0) {
      close(ready[0]);
      close(done[1]);
      sqlite3 *foreign = nullptr;
      if (sqlite3_open(p.c_str(), &foreign) != SQLITE_OK) _exit(3);
      if (sqlite3_exec(foreign, "BEGIN IMMEDIATE;", nullptr, nullptr, nullptr) != SQLITE_OK) _exit(4);
      writeAll(ready[1], "r");
      close(ready[1]);
      char c;
      if (read(done[0], &c, 1) != 1) _exit(5);
      sqlite3_exec(foreign, "ROLLBACK;", nullptr, nullptr, nullptr);
      sqlite3_close(foreign);
      _exit(0);
    }
    close(ready[1]);
    close(done[0]);
    check(readAll(ready[0]) == "r", "BUSY child ready");
    close(ready[0]);
    const auto e = error([&] { append(db); });
    check(e.code() == MapErrorCode::WriterConflict && e.outcome() == CommitOutcome::NotCommitted, "genuine BUSY outcome");
    check(logical(MapDatabaseTestAccess::handle(db)) == old, "BUSY modified rows");
    writeAll(done[1], "d");
    close(done[1]);
    int status;
    waitpid(child, &status, 0);
    check(status == 0, "BUSY child");
    db.finish();
  }
  {
    auto p = copy(source, root, "duplicate-node");
    MapDatabase db(p.string(), "resume");
    db.promoteToWritable();
    append(db);
    auto next = logical(MapDatabaseTestAccess::handle(db));
    auto e = error([&] { append(db); });
    check(e.code() == MapErrorCode::StaleMap && e.outcome() == CommitOutcome::NotCommitted, "duplicate Node silently replaced");
    check(logical(MapDatabaseTestAccess::handle(db)) == next, "duplicate Node/Link altered committed revision");
    db.finish();
  }
  {
    auto p = copy(source, root, "duplicate-root-slot");
    MapDatabase db(p.string(), "resume");
    db.promoteToWritable();
    const auto old = logical(MapDatabaseTestAccess::handle(db));
    auto f = frame(6, anchor(6));
    GraphLink base{7, 1, 1, Eigen::Isometry3f::Identity()};
    error([&] { db.commitFinalizedSubmap(f, grid(), {}, {{7, anchor(6).cast<float>()}}, base, base, db.mapUuid(), 1, 7, identity(), 7); });
    check(logical(MapDatabaseTestAccess::handle(db)) == old, "duplicate Link query altered rows");
    db.finish();
  }
  {
    auto p = copy(source, root, "wal-api-failure");
    MapDatabase db(p.string(), "resume");
    db.promoteToWritable();
    hook = [](const char *at, void *) {
      if (std::string(at) == "before-wal-activation") fail_mode = true;
    };
    const auto e = error([&] { append(db); });
    hook = {};
    fail_mode = false;
    check(e.outcome() == CommitOutcome::Committed && db.committedRevision() == 2, "WAL API failure hid commit");
    db.finish();
    MapDatabase reopened(p.string(), "recover");
    check(reopened.committedRevision() == 2, "WAL failure revision missing");
    reopened.finish();
  }
  // Corrupt topology through a test-only SQL handle, then require the production
  // historical validator to reject without changing any canonical bytes.
  int index = 0;
  for (const char *mutation :
       {"DELETE FROM Link WHERE from_id=2 AND to_id=3;", "UPDATE Link SET from_id=1,to_id=6,type=1 WHERE from_id=5 AND to_id=6;",
        "UPDATE Link SET from_id=6,to_id=5,type=1 WHERE from_id=1 AND to_id=2;",
        "UPDATE Link SET from_id=6,to_id=99,type=1 WHERE from_id=5 AND to_id=6;"}) {
    auto p = copy(source, root, "bad-topology-" + std::to_string(index++));
    {
      MapDatabase db(p.string(), "recover");
      sql(MapDatabaseTestAccess::handle(db), "PRAGMA foreign_keys=OFF;");
      sql(MapDatabaseTestAccess::handle(db), mutation);
      db.finish();
    }
    const auto before = fileSnapshot(root);
    {
      MapDatabase db(p.string(), "resume");
      error([&] { db.validateHistoricalRecords(.1f); });
    }
    check(fileSnapshot(root) == before, "topology rejection mutated files");
  }
  std::cout << "PASS handoff sidecar, exclusive promotion barrier, real SQLite BUSY, duplicate Node/Link, WAL API failure and malformed topology\n";
}
}  // namespace

namespace {
std::string configFor(float resolution) {
  NaviMapParameters parameters;
  if (resolution != static_cast<float>(parameters.resolution)) parameters.resolution = resolution;
  return map_config_identity(PoseGraphParameters{}, parameters);
}
void reopenFinalized(const fs::path &path, float resolution, bool runtime) {
  std::string rows;
  {
    MapDatabase history(path.string(), "resume", configFor(resolution), resolution);
    history.validateHistoricalRecords(resolution);
    rows = logical(MapDatabaseTestAccess::handle(history));
    history.finish();
  }
  {
    MapDatabase recovery(path.string(), "recover", configFor(resolution), resolution);
    check(logical(MapDatabaseTestAccess::handle(recovery)) == rows, "positive recovery changed committed rows/blobs");
    recovery.finish();
  }
  if (runtime) {
    PoseGraphParameters graph;
    graph.map_mode = "resume";
    NaviMapParameters navigation;
    if (resolution != static_cast<float>(navigation.resolution)) navigation.resolution = resolution;
    PoseGraphBackend backend(graph, navigation, path.string(), {});
    check(backend.reconstructionDiagnostics().nodes > 0, "positive A2 runtime missing history");
    backend.finish();
  }
}
void checkIndependentReopen(const fs::path &p, float resolution, bool runtime) {
  runChild({"--reopen-finalized", p.string(), std::to_string(resolution), runtime ? "runtime" : "validation"});
}
void finalizedInputDomain(const fs::path &source, const fs::path &root) {
  const auto rejectedInput = [&](const std::string &name, const SubmapFrame &f, const LocalGrid &evidence,
                                 const std::vector<std::pair<int, Eigen::Isometry3f>> &poses, const char *reason) {
    const auto p = copy(source, root, "w1-rejected-" + name);
    std::string before;
    {
      MapDatabase db(p.string(), "resume");
      db.promoteToWritable();
      auto *h = MapDatabaseTestAccess::handle(db);
      before = logical(h);
      const auto revision = db.committedRevision();
      const int begins = immediate_begins, changes = sqlite3_total_changes(h);
      const auto files = fileSnapshot(root);
      GraphLink base{6, 7, 0, (anchor(5).inverse() * anchor(6)).cast<float>()};
      const auto failure = error([&] {
        db.commitFinalizedSubmap(f, evidence, buildVisualScene(f)->encode(), poses, base, std::nullopt, db.mapUuid(), revision, 7, identity(), 1);
      });
      check(failure.code() == MapErrorCode::HistoricalData && failure.outcome() == CommitOutcome::NotCommitted,
            "invalid finalized facts had wrong failure classification: " + name);
      check(std::string(failure.what()).find(reason) != std::string::npos, "wrong rejection boundary: " + name);
      check(immediate_begins == begins && sqlite3_total_changes(h) == changes && sqlite3_get_autocommit(h), "invalid input reached BEGIN/writes");
      check(db.committedRevision() == revision && logical(h) == before, "invalid finalized input changed revision/rows/blobs: " + name);
      check(fileSnapshot(root) == files, "invalid input changed canonical bytes/sidecars: " + name);
      db.finish();
    }
    checkIndependentReopen(p);
    check(recovered(p) == before, "invalid input changed recovered rows: " + name);
    std::cout << "PASS W.1 reject before BEGIN, unchanged rows/blobs/revision, independent A2/recovery: " << name << '\n';
  };
  const auto normal = frame(6, anchor(6));
  const auto normal_pose = anchor(6).cast<float>();
  for (int family = 0; family < 3; ++family) {
    auto evidence = grid();
    auto *cells = family == 0 ? &evidence.groundCells : family == 1 ? &evidence.obstacleCells : &evidence.emptyCells;
    cells->front().x() = 1e10f;
    rejectedInput("grid-range-" + std::to_string(family), normal, evidence, {{7, normal_pose}}, "occupancy cell");
  }
  auto historical = anchor(0).cast<float>();
  historical.translation().x() = 1e10f;
  rejectedInput("historical-pose-range", normal, grid(), {{7, normal_pose}, {1, historical}}, "occupancy cell");

  // Both individual domains are in range; only their resulting union is invalid.
  historical = Eigen::Isometry3f::Identity();
  historical.translation().x() = -1.5e8f;
  auto new_pose = Eigen::Isometry3f::Identity();
  new_pose.translation().x() = 1.5e8f;
  rejectedInput("changed-historical-aggregate-width", normal, grid(), {{7, new_pose}, {1, historical}}, "occupancy dimensions");
  historical.translation() = Eigen::Vector3f(0, -1.5e8f, 0);
  new_pose.translation() = Eigen::Vector3f(0, 1.5e8f, 0);
  rejectedInput("changed-historical-aggregate-height", normal, grid(), {{1, historical}, {7, new_pose}}, "occupancy dimensions");

  LocalGrid evidence(.1f);
  evidence.groundCells = {{static_cast<float>(std::numeric_limits<int>::min()) * .1f, 0}};
  check(double(std::floor(evidence.groundCells.front().x() / .1f)) == std::numeric_limits<int>::min(), "INT_MIN fixture arithmetic");
  rejectedInput("unchanged-historical-aggregate", normal, evidence, {{7, Eigen::Isometry3f::Identity()}}, "occupancy dimensions");

  const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
  for (int position = 0; position < 3; ++position) {
    for (int value = 0; value < 3; ++value) {
      const auto f = frame(6, anchor(6), [&](NavigationPath &path) {
        const auto index = position == 0 ? 0 : position == 1 ? path.spline.knots.size() / 2 : path.spline.knots.size() - 1;
        path.spline.knots[index] = value == 0 ? nan : value == 1 ? inf : -inf;
      });
      rejectedInput("knot-" + std::to_string(position) + "-" + std::to_string(value), f, grid(), {{7, normal_pose}}, "navigation");
    }
  }

  // Valid default/empty spline (navigation samples remain required by SubmapFrame).
  const auto empty_path = copy(source, root, "w1-empty-spline");
  {
    MapDatabase db(empty_path.string(), "resume");
    db.promoteToWritable();
    const auto f = frame(6, anchor(6), [](NavigationPath &path) { path.spline = {}; });
    const GraphLink base{6, 7, 0, (anchor(5).inverse() * anchor(6)).cast<float>()};
    check(db.commitFinalizedSubmap(f, grid(), buildVisualScene(f)->encode(), {{7, normal_pose}}, base, std::nullopt, db.mapUuid(), 1, 7, identity(),
                                   1) == 2,
          "default empty spline rejected");
    db.finish();
  }
  checkIndependentReopen(empty_path);

  // Tight extent near both signed-int limits, at a nondefault occupancy resolution
  // distinct from LocalGrid.cellSize. A full independent backend remains small.
  for (bool negative : {false, true})
    for (bool recover : {false, true}) {
      constexpr float resolution = .125f;
      const auto p = root / ("w1-boundary-" + std::to_string(negative) + "-" + std::to_string(recover) + ".db");
      Eigen::Isometry3d a = Eigen::Isometry3d::Identity();
      const float cell = negative ? -2147483520.f : 2147483520.f;
      a.translation().x() = double(cell * resolution);
      check(std::floor(float(a.translation().x()) / resolution) == cell, "near-boundary cell identity");
      {
        MapDatabase seed_db(p.string(), "new", configFor(resolution), resolution);
        const auto first = frame(0, a);
        seed_db.saveSubmap(first, grid(), buildVisualScene(first)->encode());
        seed_db.saveSubmapPoses({{1, a.cast<float>()}});
        seed_db.finish();
      }
      {
        MapDatabase db(p.string(), recover ? "recover" : "resume", configFor(resolution), resolution);
        if (!recover) db.promoteToWritable(resolution);
        const auto second = frame(1, a);
        const GraphLink base{1, 2, 0, Eigen::Isometry3f::Identity()};
        check(db.commitFinalizedSubmap(second, grid(), buildVisualScene(second)->encode(), {{2, a.cast<float>()}}, base, std::nullopt, db.mapUuid(),
                                       1, 2, configFor(resolution), 1) == 2,
              "near-boundary representable geometry rejected");
        db.finish();
      }
      checkIndependentReopen(p, resolution);
    }
  std::cout << "PASS W.1 valid empty spline and near-int-boundary geometry in promoted/recovery writers at nondefault resolution\n";
}
}  // namespace

int main(int argc, char **argv) {
  char directory[] = "/tmp/sapphire-w-XXXXXX";
  fs::path root = mkdtemp(directory);
  try {
    if (argc == 5 && std::string(argv[1]) == "--reopen-finalized") {
      reopenFinalized(argv[2], std::stof(argv[3]), std::string(argv[4]) == "runtime");
      return 0;
    }
    if (argc == 4 && std::string(argv[1]) == "--capability") {
      capabilityChild(argv[2], argv[3]);
      return 0;
    }
    if (argc == 4 && std::string(argv[1]) == "--crash") {
      crashChild(argv[2], argv[3]);
      return 1;
    }
    if (argc == 2 && std::string(argv[1]) == "--pipeline-gpu") {
      pipeline(root);
      fs::remove_all(root);
      return 0;
    }
    auto source = root / "source.db";
    seed(source);
    const auto count = fdCount();
    admission(source, root);
    races(source, root);
    identities(source, root);
    atomic(source, root);
    finalizedInputDomain(source, root);
    additionalBoundaries(source, root);
    crashes(source, root);
    topology(source, root);
    backendFailures(source, root);
    check(fdCount() == count, "W leaked descriptors");
    fs::remove_all(root);
    std::cout << "W storage tests passed\n";
    return 0;
  } catch (const std::exception &e) {
    hook = {};
    std::cerr << "FAIL: " << e.what() << " (fixtures " << root << ")\n";
    return 1;
  }
}
