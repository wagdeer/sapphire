#include <execinfo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <condition_variable>
#include <cstdarg>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <map>
#include <new>
#include <set>
#include <thread>

#include "backend/registration/loop_closure.hpp"
#include "backend/storage/map_database.hpp"
#include "backend/visual/feature/scene_features.hpp"
#include "backend/grid/occ_layer.hpp"
#include "backend/graph/pose_graph.hpp"
// One allocation, on the attachment caller thread, armed only at an explicit
// preparation/update seam. All other allocations retain normal behavior.
namespace {
thread_local bool fail_attachment_allocation = false;
std::size_t failed_attachment_bytes = 0;
void *attachment_allocation_stack[24];
int attachment_allocation_depth = 0;
}  // namespace
void *operator new(std::size_t bytes) {
  if (fail_attachment_allocation) {
    fail_attachment_allocation = false;
    failed_attachment_bytes = bytes;
    attachment_allocation_depth = backtrace(attachment_allocation_stack, 24);
    throw std::bad_alloc();
  }
  if (void *p = std::malloc(bytes ? bytes : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }

using namespace sapphire;
namespace fs = std::filesystem;
namespace sapphire {
struct MapDatabaseTestAccess {
  static sqlite3 *handle(MapDatabase &db) { return db.db_; }
  static int fd(MapDatabase &db) { return db.file_fd_; }
  static void prove(MapDatabase &db) { db.proveWritableIdentity(); }
  static struct stat observe(sqlite3 *db) { return MapDatabase::observeMainFile(db); }
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
Eigen::Isometry3d relation() {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.linear() = (Eigen::AngleAxisd(.31, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-.08, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(.12, Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  p.translation() = Eigen::Vector3d(.6, -.4, .2);
  return p;
}
Eigen::Isometry3d freshAnchor() {
  auto p = anchor(0).inverse();
  p.translation() = Eigen::Vector3d(-42, 19, 3);
  return p;
}
SubmapFrame frame(int id, const Eigen::Isometry3d &a, bool target = false, int count = 120, bool mismatch = false) {
  auto cloud = std::make_shared<GaussianCloud>();
  const auto transform = target ? relation() : Eigen::Isometry3d::Identity();
  for (int i = 0; i < count; ++i) {
    const int x = i % 5, y = (i / 5) % 6, z = i / 30;
    GaussianPoint p;
    p.N = 20;
    p.voxel_key.x = i;
    p.mean = (transform * Eigen::Vector3d(.37 * x + .03 * y * y, .43 * y + .02 * x * z, .39 * z + .02 * x * y)).cast<float>();
    const Eigen::Matrix3d covariance = Eigen::Vector3d(.001, .02, .04).asDiagonal();
    p.covariance = (transform.linear() * covariance * transform.linear().transpose()).cast<float>();
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

  VisualFrame image(id + 1, 0);
  cv::Mat descriptors(96, 32, CV_8UC1);
  cv::RNG random(900 + id);
  random.fill(descriptors, cv::RNG::UNIFORM, 0, 256);
  std::vector<VisualPoint> points;
  for (int i = 0; i < 96; ++i) points.emplace_back(Eigen::Vector2f(i + 10, i + 20), i % 4, .5f);
  image.update_features(std::move(points), descriptors);
  std::vector<VisualFrame> visuals;
  visuals.push_back(std::move(image));
  auto later = a * relation();
  OdomPoses odometry{{double(id + 1), a}, {double(id + 1) + .02, later}};
  if (mismatch) odometry.front().T_odom_base.translation().x() += 1;
  SubmapFrame f(id, std::move(lio), id, id + 1, 2, 0, 0, pyramid.release_data(), std::move(odometry), std::move(path), std::move(visuals));
  f.attach_tag(MetaTag(MetaTagType::kBinary, descriptors.row(0).clone()));
  f.attach_tag(MetaTag(MetaTagType::kFloat, cv::Mat(1, 4, CV_32FC1, cv::Scalar(.4f))));
  return f;
}
PoseGraphParameters resumeConfig() {
  PoseGraphParameters p;
  p.map_mode = "resume";
  return p;
}
void seed(const fs::path &path, bool drift = true, int nodes = 6) {
  MapDatabase db(path.string());
  std::vector<GraphLink> links;
  std::vector<std::pair<int, Eigen::Isometry3f>> poses;
  for (int i = 0; i < nodes; ++i) {
    auto f = frame(i, anchor(i), true);
    db.saveSubmap(f, grid(), buildVisualScene(f)->encode());
    Eigen::Isometry3f committed = anchor(i).cast<float>();
    if (drift && i == 3) committed.translation().x() += .004f;
    poses.emplace_back(i + 1, committed);
    if (i) links.push_back({i, i + 1, 0, (anchor(i - 1).inverse() * anchor(i)).cast<float>()});
  }
  db.saveSubmapPoses(poses, links);
  db.finish();
}
std::function<void(const char *, void *)> hook;
int commit_fault = 0;
int begin_count = 0;
bool stale_revision = false;
sqlite3 *writer = nullptr;
}  // namespace
namespace sapphire::database_detail {
void writableStorageTestPoint(const char *point, void *context) {
  if (hook) hook(point, context);
}
}  // namespace sapphire::database_detail
extern "C" {
int __real_sqlite3_exec(sqlite3 *, const char *, int (*)(void *, int, char **, char **), void *, char **);
int __wrap_sqlite3_exec(sqlite3 *db, const char *q, int (*cb)(void *, int, char **, char **), void *arg, char **message) {
  if (std::strcmp(q, "BEGIN IMMEDIATE;") == 0) ++begin_count;
  const bool commit = std::strcmp(q, "COMMIT;") == 0;
  if (commit && commit_fault == 1) return SQLITE_IOERR;
  const int rc = __real_sqlite3_exec(db, q, cb, arg, message);
  if (commit && commit_fault == 2 && rc == SQLITE_OK) return SQLITE_IOERR;
  return rc;
}
sqlite3_int64 __real_sqlite3_column_int64(sqlite3_stmt *, int);
sqlite3_int64 __wrap_sqlite3_column_int64(sqlite3_stmt *s, int c) {
  auto value = __real_sqlite3_column_int64(s, c);
  if (stale_revision && c == 1 && std::strcmp(sqlite3_sql(s), "SELECT id,graph_revision,map_uuid,config_identity FROM MapState;") == 0)
    return value + 1;
  return value;
}
int __real_sqlite3_open_v2(const char *, sqlite3 **, int, const char *);
int __wrap_sqlite3_open_v2(const char *path, sqlite3 **db, int flags, const char *vfs) {
  int rc = __real_sqlite3_open_v2(path, db, flags, vfs);
  if (rc == SQLITE_OK && vfs && std::strcmp(vfs, "unix") == 0) writer = *db;
  return rc;
}
}
namespace {
fs::path fixture(const fs::path &source, const fs::path &root, const std::string &name) {
  auto dir = root / name;
  fs::create_directories(dir);
  auto p = dir / "map.db";
  fs::copy_file(source, p);
  return p;
}
void noCorrection(PoseGraphBackend &backend) {
  check(!backend.hasActiveCorrection(), "correction readiness leaked");
  const auto e = error([&] { (void)backend.T_map_odom(); });
  check(e.code() == MapErrorCode::CorrectionUnavailable || e.code() == MapErrorCode::Lifecycle, "correction error category");
}
void independentReopen(const fs::path &path, int nodes, std::uint64_t revision) {
  const auto child = fork();
  check(child >= 0, "reopen fork");
  if (child == 0) {
    const auto executable = fs::read_symlink("/proc/self/exe").string();
    const auto n = std::to_string(nodes), r = std::to_string(revision);
    execl(executable.c_str(), executable.c_str(), "--reopen", path.c_str(), n.c_str(), r.c_str(), nullptr);
    _exit(127);
  }
  int status;
  check(waitpid(child, &status, 0) == child && status == 0, "independent A2/recovery reopen failed");
}
void reopen(const fs::path &path, int nodes, std::uint64_t revision) {
  {
    PoseGraphBackend backend(resumeConfig(), {}, path.string(), {});
    check(backend.reconstructionDiagnostics().nodes == std::size_t(nodes), "A2 node count");
    check(backend.reconstructionDiagnostics().factors == std::size_t(nodes), "sole prior + connected chain/attachment factor count");
    check(backend.graphRevision() == revision, "A2 revision");
    noCorrection(backend);
    backend.finish();
  }
  MapDatabase db(path.string(), "recover");
  db.validateHistoricalRecords(.1f);
  auto *handle = MapDatabaseTestAccess::handle(db);
  check(scalar(handle, "PRAGMA integrity_check") == "ok", "recovery integrity");
  check(scalar(handle, "PRAGMA user_version") == "1", "single current format");
  check(scalar(handle, "SELECT count(*) FROM Node") == std::to_string(nodes), "complete committed node count");
  check(db.committedRevision() == revision, "recovery revision");
  for (const char *table : {"SpatialRecord", "LaserRecord", "PyramidVoxel", "NaviTrajectory", "FlatGrid", "ImageRecord", "VisualScene"})
    check(scalar(handle, ("SELECT count(*) FROM " + std::string(table)).c_str()) == std::to_string(nodes), "complete payloads");
  check(scalar(handle, "SELECT count(*) FROM MetaTag") == std::to_string(nodes * 3), "all tags including committed pose");
  if (nodes == 7) {
    check(scalar(handle, "SELECT count(*) FROM Link WHERE type=0") == "5", "no cross-session odometry");
    check(scalar(handle, "SELECT count(*) FROM Link WHERE from_id=7 AND to_id=6 AND type=1") == "1", "single adjacent-target attachment");
    check(scalar(handle, "SELECT count(*) FROM Link WHERE from_id=7 OR to_id=7") == "1", "root consumes single slot");
    const auto original_query = frame(0, freshAnchor());
    const auto timestamps = database_detail::packOdomTimestamps(original_query.odom_poses());
    const auto samples = database_detail::packOdomPoses(original_query.odom_poses());
    database_detail::Statement odom(handle, "SELECT odom_pose_count,odom_timestamps,odom_poses FROM Node WHERE id=7");
    check(sqlite3_step(odom.get()) == SQLITE_ROW && sqlite3_column_int(odom.get(), 0) == 2, "all original odometry samples persisted");
    check(sqlite3_column_bytes(odom.get(), 1) == int(timestamps.size() * sizeof(double)) &&
              std::memcmp(sqlite3_column_blob(odom.get(), 1), timestamps.data(), timestamps.size() * sizeof(double)) == 0 &&
              sqlite3_column_bytes(odom.get(), 2) == int(samples.size() * sizeof(double)) &&
              std::memcmp(sqlite3_column_blob(odom.get(), 2), samples.data(), samples.size() * sizeof(double)) == 0,
          "original Q odometry samples/timestamps preserved byte-for-byte");
    const auto links = db.loadGraphLinks();
    const auto &link = links.back();
    check(link.transform.matrix().isApprox(relation().inverse().cast<float>().matrix(), .015f), "stored inverse direction against independent truth");
    std::vector<Eigen::Isometry3d> poses;
    db.visitHistoricalNodes([&](int id, const auto &a, const auto &p, const auto &) {
      poses.push_back(p.template cast<double>());
      if (id == 7)
        check(a.matrix().isApprox(freshAnchor().matrix(), 1e-12), "fresh original odometry unchanged");
      else
        check(a.matrix().isApprox(anchor(id - 1).matrix(), 1e-12), "historical odometry unchanged");
    });
    const auto predicted = poses[6].inverse() * poses[5];
    check(predicted.matrix().isApprox(link.transform.cast<double>().matrix(), 1e-5), "independent SE3 Between(Q,H) residual");
    gtsam::BetweenFactor<gtsam::Pose3> factor(6, 5, gtsam::Pose3(link.transform.cast<double>().matrix()), gtsam::noiseModel::Unit::Create(6));
    check(factor.evaluateError(gtsam::Pose3(poses[6].matrix()), gtsam::Pose3(poses[5].matrix())).norm() < 1e-5,
          "actual reconstructed factor direction");
  }
  db.finish();
}
void success(const fs::path &source, const fs::path &root) {
  const auto p = fixture(source, root, "success");
  PoseGraphBackend backend(resumeConfig(), {}, p.string(), [](auto) { throw std::runtime_error("B3 callback must remain inactive"); });
  const auto revision = backend.graphRevision();
  const auto original = backend.committedPose(3)->matrix().eval();
  const auto old_grid = backend.latestOccupancyGrid();
  const auto retained_evidence = backend.historicalOccupancy();
  noCorrection(backend);
  int bbs = 0, gicp = 0, promotions = 0, solver = 0;
  Eigen::Isometry3d measured;
  gtsam::Values proposed;
  hook = [&](const char *stage, void *context) {
    const std::string point(stage);
    if (point == "b1-after-bbs") {
      ++bbs;
      check(static_cast<BbsResult *>(context)->accepted, "real BBS accepts");
    }
    if (point == "b1-after-gicp") {
      ++gicp;
      auto &r = *static_cast<GicpResult *>(context);
      check(r.accepted && r.inliers >= 64, "real GICP accepts");
      measured = r.T_target_query;
      check(measured.matrix().isApprox(relation().matrix(), .015), "asymmetric RPY registration truth");
    }
    if (point == "b1-after-solver") {
      ++solver;
      auto *isam = static_cast<gtsam::ISAM2 *>(context);
      proposed = isam->calculateEstimate();
      int priors = 0, between = 0;
      for (const auto &f : isam->getFactorsUnsafe()) {
        if (std::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(f)) ++priors;
        if (std::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(f)) ++between;
      }
      check(priors == 1 && between == 6, "sole original prior, no odometry predecessor or second loop");
      const auto f = std::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(isam->getFactorsUnsafe().back());
      gtsam::Vector6 v;
      v << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
      const auto noise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(10., gtsam::noiseModel::mEstimator::Base::Block), gtsam::noiseModel::Diagonal::Variances(v));
      check(f && f->keys()[0] == 6 && f->keys()[1] == 5 && f->noiseModel()->equals(*noise, 1e-15), "exact LiDAR factor direction/noise");
    }
    if (point == "exclusive-promotion") {
      ++promotions;
      check(bbs == 1 && gicp == 1 && solver == 1, "late W activation");
    }
    if (point.rfind("b1-", 0) == 0) {
      noCorrection(backend);
      if (point != "b1-postcommit-materialization" && point != "b1-runtime-ready") {
        check(backend.committedPose(3)->matrix() == original, "committed pose changed speculatively");
        check(backend.latestOccupancyGrid() == old_grid, "grid changed speculatively");
        check(!backend.committedPose(6), "query membership before commit");
      }
    }
  };
  const auto start = std::chrono::steady_clock::now();
  auto result = backend.attachFreshSession(frame(0, freshAnchor()), grid(), 6, relation());
  hook = {};
  check(result.status == AttachmentStatus::Attached && result.root_node_id == 7 && result.committed_revision == revision + 1,
        "B1 success result: " + result.message);
  check(bbs == 1 && gicp == 1 && promotions == 1, "one verification and one W promotion");
  check(begin_count > 0, "W transaction ran");
  check(backend.hasActiveCorrection() && !backend.enabled(), "correction ready, B2 worker disabled");
  for (int i = 0; i < 7; ++i)
    check(backend.committedPose(i)->matrix() == proposed.at<gtsam::Pose3>(i).matrix().cast<float>(), "all proposed serialized poses committed");
  check(backend.committedPose(3)->matrix() != original, "A2 preexisting solver/committed difference reconciled");
  const Eigen::Isometry3d expected = backend.committedPose(6)->cast<double>() * freshAnchor().inverse();
  check(result.correction && result.correction->matrix().isApprox(expected.matrix(), 1e-12) &&
            backend.T_map_odom().matrix().isApprox(expected.matrix(), 1e-12),
        "committed-derived fresh correction");
  check(!expected.matrix().isApprox(relation().matrix(), .1), "seed is not correction");
  check(backend.historicalVisualMetadata().count(7) == 1, "root target visual membership after commit");
  check(backend.historicalOccupancy().activeNodeIds().size() == 7 && retained_evidence.activeNodeIds().size() == 6,
        "root occupancy membership and safe retained diagnostic evidence");
  const auto new_grid = backend.latestOccupancyGrid();
  check(new_grid->source_graph_revision == revision + 1 && old_grid->source_graph_revision == revision,
        "immutable reader output and committed revision");
  OccupancyGrid expected_grid({});
  for (int i = 0; i < 7; ++i) check(expected_grid.append(GridFrame(i + 1, *backend.committedPose(i), grid())), "expected occupancy append");
  check(new_grid->data == expected_grid.getMap().cells && new_grid->origin_x == expected_grid.getMap().originX &&
            new_grid->origin_y == expected_grid.getMap().originY,
        "occupancy from all serialized poses");
  check(error([&] { backend.addFrame(frame(1, freshAnchor())); }).code() == MapErrorCode::ReadOnly, "B2 remains closed");
  check(error([&] { backend.attachFreshSession(frame(0, freshAnchor()), grid(), 6, relation()); }).code() == MapErrorCode::Lifecycle,
        "duplicate attachment prevented");
  check(error([&] {
          auto next = frame(7, freshAnchor());
          GraphLink edge{8, 6, 1, relation().inverse().cast<float>()};
          backend.commitFinalizedSubmap(next, grid(), buildVisualScene(next)->encode(), {{8, *backend.committedPose(6)}}, edge, std::nullopt,
                                        backend.mapUuid(), backend.graphRevision(), 8, 8);
        }).code() == MapErrorCode::Lifecycle,
        "low-level W API cannot bypass the B1-only continuation gate");
  check(backend.hasActiveCorrection() && backend.graphRevision() == revision + 1, "denied continuation retains the ready root");
  const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  backend.resetFreshSession();
  noCorrection(backend);
  check(error([&] { backend.addFrame(frame(0, freshAnchor())); }).code() == MapErrorCode::Lifecycle, "reset fences old backend");
  check(error([&] { backend.attachFreshSession(frame(0, freshAnchor()), grid(), 6, relation()); }).code() == MapErrorCode::Lifecycle,
        "reset requires owner reconstruction");
  independentReopen(p, 7, revision + 1);
  // Reset/replacement creates a genuinely fresh odometry domain. It cannot
  // start at persistent ID zero or continue the preceding session implicitly.
  {
    PoseGraphBackend replacement(resumeConfig(), {}, p.string(), {});
    noCorrection(replacement);
    auto next_anchor = freshAnchor();
    next_anchor.translation() += Eigen::Vector3d(30, 6, -2);
    check(error([&] { replacement.addFrame(frame(0, next_anchor)); }).code() == MapErrorCode::ReadOnly, "replacement requires B1");
    auto next = replacement.attachFreshSession(frame(0, next_anchor), grid(), 6, relation());
    check(next.status == AttachmentStatus::Attached && next.root_node_id == 8 && next.committed_revision == revision + 2,
          "new session requires explicit second B1 with next global root");
    check(next.correction->matrix().isApprox((replacement.committedPose(7)->cast<double>() * next_anchor.inverse()).matrix(), 1e-12),
          "replacement uses its own fresh odometry anchor");
    replacement.finish();
  }
  independentReopen(p, 8, revision + 2);
  {
    MapDatabase db(p.string(), "resume");
    check(scalar(MapDatabaseTestAccess::handle(db), "SELECT count(*) FROM Link WHERE type=0") == "5", "reset never creates cross-session type0");
    check(scalar(MapDatabaseTestAccess::handle(db), "SELECT count(*) FROM Link WHERE type=1") == "2", "one attachment per explicit fresh session");
    db.finish();
  }
  std::cout << "PASS real BBS/GICP, SE3, committed-all-key reconciliation, root topology, visual/grid, correction, reset, process reopen; total_ms="
            << elapsed << '\n';
}
void rejections(const fs::path &source, const fs::path &root) {
  const auto p = fixture(source, root, "rejections");
  const auto before = fileSnapshot(p.parent_path());
  std::uint64_t revision;
  {
    PoseGraphBackend backend(resumeConfig(), {}, p.string(), {});
    revision = backend.graphRevision();
    const auto uuid = backend.mapUuid();
    const auto metadata = backend.historicalVisualMetadata();
    const auto output = backend.latestOccupancyGrid();
    int promotions = 0, commits = begin_count;
    for (const std::string kind : {"target-zero", "target-future", "nan-seed", "nonrigid-seed", "huge-seed", "inconsistent-anchor", "far-seed-bbs",
                                   "small-query-gicp", "missing-cloud", "exception-before-registration", "exception-before-solver"}) {
      auto seed_pose = relation();
      int target = 6, count = 120;
      auto expected = AttachmentStatus::RegistrationFailure;
      if (kind == "target-zero") {
        target = 0;
        expected = AttachmentStatus::InvalidTarget;
      }
      if (kind == "target-future") {
        target = 7;
        expected = AttachmentStatus::InvalidTarget;
      }
      if (kind == "nan-seed") {
        seed_pose.translation().x() = NAN;
        expected = AttachmentStatus::InvalidSeed;
      }
      if (kind == "nonrigid-seed") {
        seed_pose.linear()(0, 0) *= 2;
        expected = AttachmentStatus::InvalidSeed;
      }
      if (kind == "huge-seed") {
        seed_pose.translation().x() = 1e30;
        expected = AttachmentStatus::InvalidSeed;
      }
      if (kind == "far-seed-bbs") {
        seed_pose.translation() = Eigen::Vector3d(100, -100, 100);
        expected = AttachmentStatus::BbsRejected;
      }
      if (kind == "small-query-gicp") {
        count = 32;
        expected = AttachmentStatus::GicpRejected;
      }
      if (kind == "missing-cloud") expected = AttachmentStatus::MissingGeometry;
      if (kind == "inconsistent-anchor") expected = AttachmentStatus::InvalidQuery;
      hook = [&](const char *stage, void *context) {
        const std::string point(stage);
        if (point == "exclusive-promotion") ++promotions;
        if (kind == "missing-cloud" && point == "b1-target-cloud") static_cast<GaussianCloudPtr *>(context)->reset();
        if ((kind == "exception-before-registration" && point == "b1-before-registration") ||
            (kind == "exception-before-solver" && point == "b1-before-solver"))
          throw std::runtime_error("injected pre-solver error");
      };
      const auto result = backend.attachFreshSession(frame(0, freshAnchor(), false, count, kind == "inconsistent-anchor"), grid(), target, seed_pose);
      hook = {};
      check(result.status == expected, "wrong rejection category: " + kind + " actual=" + std::to_string(int(result.status)) + " " + result.message);
      check(!result.correction && result.root_node_id == 0 && result.committed_revision == 0, "rejection has no durable result");
      noCorrection(backend);
      check(!backend.failed() && backend.mapUuid() == uuid && backend.graphRevision() == revision && !backend.committedPose(6),
            "healthy historical retry boundary");
      check(backend.historicalVisualMetadata() == metadata && backend.latestOccupancyGrid() == output, "no rejected query visual/grid membership");
      check(promotions == 0 && begin_count == commits, "rejection activated W");
      check(fileSnapshot(p.parent_path()) == before, "exact rejected DB/directory bytes: " + kind);
      std::cout << "PASS exact nonmutation and explicit retry after " << kind << '\n';
    }
    backend.finish();
  }
  check(fileSnapshot(p.parent_path()) == before, "writer-close/history-reject-close byte equality");
  independentReopen(p, 6, revision);
  // A2 rejects missing durable target payloads even before attachment admission.
  const auto missing = fixture(source, root, "missing-durable-pyramid");
  sqlite3 *db = nullptr;
  check(sqlite3_open(missing.c_str(), &db) == SQLITE_OK, "corrupt fixture open");
  sql(db, "DELETE FROM PyramidVoxel WHERE node_id=6");
  sqlite3_close(db);
  const auto corrupt = fileSnapshot(missing.parent_path());
  error([&] { PoseGraphBackend backend(resumeConfig(), {}, missing.string(), {}); });
  check(fileSnapshot(missing.parent_path()) == corrupt, "A2 missing-payload nonmutation");
  const auto empty = root / "empty.db";
  {
    MapDatabase db(empty.string());
    db.finish();
  }
  const auto empty_before = fileSnapshot(root);
  {
    PoseGraphBackend backend(resumeConfig(), {}, empty.string(), {});
    check(backend.attachFreshSession(frame(0, freshAnchor()), grid(), 1, relation()).status == AttachmentStatus::EmptyMap,
          "empty map rejected explicitly");
    noCorrection(backend);
    backend.finish();
  }
  check(fileSnapshot(root) == empty_before, "empty historical map not converted into new-map startup");
}
void preparationFailures(const fs::path &source, const fs::path &root, const std::string &selected = {}) {
  for (const std::string kind : {"factor", "values", "solver"}) {
    if (!selected.empty() && kind != selected) continue;
    const auto p = fixture(source, root, "allocation-" + kind);
    const auto before_files = fileSnapshot(p.parent_path());
    PoseGraphBackend backend(resumeConfig(), {}, p.string(), {});
    const auto uuid = backend.mapUuid();
    const auto revision = backend.graphRevision();
    const auto spatial = backend.historicalSpatialCandidates(5);
    const auto visual = backend.historicalVisualCandidates(5);
    const auto metadata = backend.historicalVisualMetadata();
    const auto output = backend.latestOccupancyGrid();
    std::vector<Eigen::Matrix4f> committed;
    for (int key = 0; key < 6; ++key) committed.push_back(backend.committedPose(key)->matrix());
    const int begins = begin_count;
    int bbs = 0, gicp = 0, updates = 0, promotions = 0, values_preparations = 0;
    gtsam::ISAM2 *solver = nullptr;
    gtsam::Values before_estimate;
    gtsam::NonlinearFactorGraph before_factors;
    failed_attachment_bytes = 0;
    hook = [&](const char *stage, void *context) {
      // Do observations before arming: the very next allocation must belong to
      // the production operation, not to test strings/counters/diagnostics.
      if (!std::strcmp(stage, "b1-after-bbs")) {
        check(static_cast<BbsResult *>(context)->accepted, "preparation fixture requires real BBS success");
        ++bbs;
      }
      if (!std::strcmp(stage, "b1-after-gicp")) {
        check(static_cast<GicpResult *>(context)->accepted, "preparation fixture requires real GICP success");
        ++gicp;
      }
      if (!std::strcmp(stage, "b1-before-solver")) {
        solver = static_cast<gtsam::ISAM2 *>(context);
        before_estimate = solver->calculateEstimate();
        before_factors = solver->getFactorsUnsafe();
        if (kind == "factor") fail_attachment_allocation = true;
      }
      if (!std::strcmp(stage, "b1-before-values")) {
        ++values_preparations;
        if (kind == "values") fail_attachment_allocation = true;
      }
      if (!std::strcmp(stage, "isam-update-entry")) {
        ++updates;
        if (kind == "solver") fail_attachment_allocation = true;
      }
      if (!std::strcmp(stage, "exclusive-promotion")) ++promotions;
    };
    AttachmentResult rejected;
    std::string error_message;
    bool threw = false;
    CommitOutcome outcome = CommitOutcome::NotCommitted;
    try {
      rejected = backend.attachFreshSession(frame(0, freshAnchor()), grid(), 6, relation());
    } catch (const MapError &e) {
      threw = true;
      error_message = e.what();
      outcome = e.outcome();
    }
    const bool allocation_pending = fail_attachment_allocation;
    fail_attachment_allocation = false;
    hook = {};
    std::cout << "ALLOCATION " << kind << " bytes=" << failed_attachment_bytes << " updates=" << updates << " sticky_failed=" << backend.failed()
              << " error=" << (threw ? error_message : rejected.message) << std::endl;
    backtrace_symbols_fd(attachment_allocation_stack, attachment_allocation_depth, STDOUT_FILENO);
    check(!allocation_pending && failed_attachment_bytes > 0 && bbs == 1 && gicp == 1, "real registration reached one-shot allocation fault");
    check(promotions == 0 && begin_count == begins, "preparation/first-update allocation failure activated W");
    check(backend.mapUuid() == uuid && backend.graphRevision() == revision && fileSnapshot(p.parent_path()) == before_files,
          "allocation failure changed UUID/revision/files/directory");
    noCorrection(backend);
    if (kind == "solver") {
      check(values_preparations == 1 && updates == 1 && threw && backend.failed() && outcome == CommitOutcome::NotCommitted,
            "exception inside the first mutating update must remain sticky");
      check(error([&] { backend.committedPose(0); }).code() == MapErrorCode::Lifecycle, "failed solver cannot serve historical queries");
      check(error([&] { backend.attachFreshSession(frame(0, freshAnchor()), grid(), 6, relation()); }).code() == MapErrorCode::Lifecycle,
            "first-update failure cannot retry");
      try {
        backend.finish();
      } catch (const std::exception &) {
      }
      independentReopen(p, 6, revision);
      std::cout << "PASS allocation inside first ISAM update remains sticky; clean reopen required\n";
      continue;
    }
    check(updates == 0 && solver && values_preparations == (kind == "values" ? 1 : 0), "no mutating ISAM call during local preparation");
    if (kind == "factor") check(failed_attachment_bytes == 2 * sizeof(gtsam::Key), "factor failure occurs in its two-key vector allocation");
    const auto after_estimate = solver->calculateEstimate();
    const auto &after_factors = solver->getFactorsUnsafe();
    check(before_estimate.keys() == after_estimate.keys() && before_factors.size() == after_factors.size(), "solver key/factor counts unchanged");
    for (std::size_t i = 0; i < before_factors.size(); ++i) check(before_factors[i] == after_factors[i], "existing factor identity unchanged");
    for (const auto key : before_estimate.keys())
      check(before_estimate.at<gtsam::Pose3>(key).matrix() == after_estimate.at<gtsam::Pose3>(key).matrix(), "solver estimate unchanged exactly");
    check(!backend.failed(), "B1-R1: local " + kind + " allocation incorrectly made an untouched solver sticky Failed");
    check(!threw && rejected.status == AttachmentStatus::RegistrationFailure && !rejected.message.empty() && !rejected.correction &&
              rejected.root_node_id == 0 && rejected.committed_revision == 0,
          "local allocation failure must return the existing retryable result with error detail");
    for (int key = 0; key < 6; ++key) check(backend.committedPose(key)->matrix() == committed[key], "committed lookup unchanged");
    check(!backend.committedPose(6) && backend.latestOccupancyGrid() == output && backend.historicalVisualMetadata() == metadata,
          "no query pose/grid/visual membership on preparation failure");
    const auto after_spatial = backend.historicalSpatialCandidates(5);
    const auto after_visual = backend.historicalVisualCandidates(5);
    check(after_spatial.size() == spatial.size() && after_visual.size() == visual.size(), "historical queries still work");
    for (std::size_t i = 0; i < spatial.size(); ++i) check(after_spatial[i].submap_id == spatial[i].submap_id, "spatial identity unchanged");
    for (std::size_t i = 0; i < visual.size(); ++i) check(after_visual[i].target_id == visual[i].target_id, "visual identity unchanged");
    hook = [&](const char *stage, void *) {
      if (!std::strcmp(stage, "isam-update-entry")) ++updates;
      if (!std::strcmp(stage, "exclusive-promotion")) ++promotions;
      if (!std::strcmp(stage, "b1-after-bbs")) ++bbs;
      if (!std::strcmp(stage, "b1-after-gicp")) ++gicp;
    };
    const auto retry = backend.attachFreshSession(frame(0, freshAnchor()), grid(), 6, relation());
    hook = {};
    check(retry.status == AttachmentStatus::Attached && retry.root_node_id == 7 && retry.committed_revision == revision + 1 &&
              backend.hasActiveCorrection() && updates == 1 && promotions == 1 && begin_count == begins + 1 && bbs == 2 && gicp == 2,
          "explicit retry on same backend succeeds exactly once");
    backend.finish();
    independentReopen(p, 7, revision + 1);
    std::cout << "PASS retryable " << kind << " allocation: exact state, zero ISAM updates, same-backend explicit retry and independent reopen\n";
  }
}
void failures(const fs::path &source, const fs::path &root) {
  for (const std::string point : {"b1-after-solver", "exclusive-promotion", "before-sqlite-open", "stale-revision", "after-node", "before-commit",
                                  "commit-before", "commit-after", "after-commit", "b1-postcommit-materialization", "b1-runtime-ready"}) {
    const auto p = fixture(source, root, "failure-" + point);
    std::string rows_before;
    {
      MapDatabase db(p.string(), "resume");
      rows_before = logical(MapDatabaseTestAccess::handle(db));
      db.finish();
    }
    const auto files_before = fileSnapshot(p.parent_path());
    std::uint64_t revision;
    const bool committed =
        point == "commit-after" || point == "after-commit" || point == "b1-postcommit-materialization" || point == "b1-runtime-ready";
    {
      PoseGraphBackend backend(resumeConfig(), {}, p.string(), {});
      revision = backend.graphRevision();
      bool hit = false;
      hook = [&](const char *stage, void *context) {
        if (point == "stale-revision" && std::string(stage) == "prepared-before-transaction") {
          stale_revision = true;
          hit = true;
        }
        if ((point == "commit-before" || point == "commit-after") && std::string(stage) == "before-commit") {
          commit_fault = point == "commit-before" ? 1 : 2;
          hit = true;
        }
        if (point == stage) {
          hit = true;
          if (point == "b1-after-solver") {
            auto *isam = static_cast<gtsam::ISAM2 *>(context);
            gtsam::NonlinearFactorGraph bad;
            bad.add(gtsam::BetweenFactor<gtsam::Pose3>(6, 999, gtsam::Pose3(), gtsam::noiseModel::Unit::Create(6)));
            isam->update(bad);  // Actual optimizer exception after speculative insertion.
          }
          throw std::runtime_error("injected " + point);
        }
      };
      const auto e = error([&] { backend.attachFreshSession(frame(0, freshAnchor()), grid(), 6, relation()); });
      hook = {};
      commit_fault = 0;
      stale_revision = false;
      check(hit && backend.failed(), "sticky boundary was not exercised: " + point);
      if (point == "stale-revision") check(e.code() == MapErrorCode::StaleMap, "actual W stale revision check");
      if (point == "commit-before" || point == "commit-after")
        check(e.outcome() == CommitOutcome::Unknown, "ambiguous COMMIT preserved");
      else
        check(e.outcome() == (committed ? CommitOutcome::Committed : CommitOutcome::NotCommitted), "commit outcome: " + point);
      noCorrection(backend);
      error([&] { backend.committedPose(0); });
      error([&] { backend.latestOccupancyGrid(); });
      check(error([&] { backend.attachFreshSession(frame(0, freshAnchor()), grid(), 6, relation()); }).code() == MapErrorCode::Lifecycle,
            "failed backend retried");
      try {
        backend.finish();
      } catch (const std::exception &) {
      }
    }
    if (point == "b1-after-solver" || point == "exclusive-promotion" || point == "before-sqlite-open")
      check(fileSnapshot(p.parent_path()) == files_before, "failed promotion changed prior files");
    {
      MapDatabase db(p.string(), "recover");
      db.validateHistoricalRecords(.1f);
      check(db.committedRevision() == revision + (committed ? 1 : 0), "DB authoritative outcome after " + point);
      if (!committed) check(logical(MapDatabaseTestAccess::handle(db)) == rows_before, "precommit failure changed rows/blobs");
      db.finish();
    }
    independentReopen(p, committed ? 7 : 6, revision + (committed ? 1 : 0));
    std::cout << "PASS sticky " << point << " committed=" << committed << '\n';
  }
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
    check(cv.wait_for(lock, std::chrono::seconds(10), [&] { return reached; }), "barrier timed out");
  }
  void release() {
    std::lock_guard<std::mutex> lock(mutex);
    released = true;
    cv.notify_all();
  }
};
void concurrency(const fs::path &source, const fs::path &root) {
  for (const std::string kind : {"query", "duplicate", "reset"}) {
    const auto p = fixture(source, root, "race-" + kind);
    PoseGraphBackend backend(resumeConfig(), {}, p.string(), {});
    Latch latch;
    hook = [&](const char *stage, void *) {
      if (std::string(stage) == "b1-before-promotion") latch.pause();
    };
    auto attach = std::async(std::launch::async, [&] { return backend.attachFreshSession(frame(0, freshAnchor()), grid(), 6, relation()); });
    latch.wait();
    std::promise<void> started;
    auto concurrent = std::async(std::launch::async, [&] {
      started.set_value();
      if (kind == "query") {
        const auto pose = backend.committedPose(6);
        check(pose.has_value() && backend.hasActiveCorrection(), "query saw premature runtime");
      } else if (kind == "duplicate") {
        check(error([&] { backend.attachFreshSession(frame(0, freshAnchor()), grid(), 6, relation()); }).code() == MapErrorCode::Lifecycle,
              "simultaneous second attachment admitted");
      } else
        backend.resetFreshSession();
    });
    started.get_future().wait();
    const bool blocked = concurrent.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout;
    latch.release();
    check(attach.get().status == AttachmentStatus::Attached, "first race attachment");
    concurrent.get();
    hook = {};
    check(blocked, "lifecycle race did not serialize: " + kind);
    if (kind == "reset") noCorrection(backend);
    backend.finish();
    independentReopen(p, 7, 2);
    std::cout << "PASS lifecycle serialization: " << kind << '\n';
  }
}
std::set<std::string> taskIds() {
  std::set<std::string> result;
  for (const auto &entry : fs::directory_iterator("/proc/self/task")) result.insert(entry.path().filename().string());
  return result;
}
void teardownCase(const fs::path &source, const fs::path &root, const std::string &kind) {
  // Preserve real terminate/SIGABRT behavior in this isolated child, without
  // producing a core dump. No terminate handler masks an invariant failure.
  const rlimit no_core{0, 0};
  check(setrlimit(RLIMIT_CORE, &no_core) == 0, "disable child core dumps");
  if (kind == "worker-finish-nonstandard") {
    gpu::initialize_device();
    const auto before_tasks = taskIds();
    const auto p = root / "worker-map.db";
    std::set<std::string> owned_tasks;
    std::vector<std::string> closes;
    bool injected = false;
    {
      PoseGraphBackend backend({}, {}, p.string(), {});
      for (const auto &id : taskIds())
        if (!before_tasks.count(id)) owned_tasks.insert(id);
      check(owned_tasks.size() >= 2, "new-map mapping and prefetch workers exist");
      hook = [&](const char *stage, void *) {
        if (!std::strcmp(stage, "before-sqlite-close")) {
          const auto live = taskIds();
          for (const auto &id : owned_tasks) check(!live.count(id), "owned workers must join before SQLite close");
          closes.push_back("before");
          if (!injected) {
            injected = true;
            throw 197;
          }
        }
        if (!std::strcmp(stage, "after-sqlite-close")) closes.push_back("sqlite");
        if (!std::strcmp(stage, "after-owner-close")) closes.push_back("owner");
      };
      for (int attempt = 0; attempt < 2; ++attempt) {
        bool reported = false;
        try {
          backend.finish();
        } catch (int value) {
          check(value == 197, "preserve unknown finish failure");
          reported = true;
        }
        check(reported && backend.failed(), "explicit finish must report failure and preserve sticky state");
        noCorrection(backend);
        check(error([&] { backend.addFrame(frame(0, freshAnchor())); }).code() == MapErrorCode::Lifecycle, "failed owner cannot map again");
        if (attempt == 0) {
          check(closes == std::vector<std::string>{"before"}, "failed close must retain remaining resources");
          check(error([&] { MapDatabase denied(p.string(), "recover"); }).code() == MapErrorCode::WriterConflict,
                "incomplete SQLite close must retain exclusive admission");
        }
      }
      check(closes == std::vector<std::string>({"before", "before", "sqlite", "owner"}), "safe close retry preserves SQLite/fd order");
      std::cout << "TEARDOWN before destruction worker-finish-nonstandard failed=1 explicit_failures=2 joined_workers=" << owned_tasks.size()
                << std::endl;
    }
    hook = {};
    const auto remaining_tasks = taskIds();
    for (const auto &id : owned_tasks) check(!remaining_tasks.count(id), "no owned worker leaked after destruction");
    {
      MapDatabase proof(p.string(), "recover");
      proof.validateHistoricalRecords(.1f);
      proof.finish();
    }
    independentReopen(p, 0, 0);
    std::cout << "PASS teardown worker-finish-nonstandard: workers joined, checked failure retained, safe close/admission release\n";
    return;
  }
  const bool committed = kind == "postcommit-nonstandard";
  const bool direct = kind == "direct-nonstandard";
  const bool standard = kind == "after-standard";
  const bool allocation = kind == "after-badalloc";
  const char *fault_point = committed ? "b1-postcommit-materialization" : kind == "entry-nonstandard" ? "isam-update-entry" : "b1-after-solver";
  const auto p = fixture(source, root, kind);
  const auto before_files = fileSnapshot(p.parent_path());
  std::uint64_t revision = 0;
  std::vector<std::string> closes;
  int checked_failures = 0;
  bool injected = false;
  {
    PoseGraphBackend backend(resumeConfig(), {}, p.string(), {});
    revision = backend.graphRevision();
    hook = [&](const char *stage, void *) {
      if (!std::strcmp(stage, fault_point)) {
        injected = true;
        if (standard) throw std::runtime_error("teardown standard detail");
        if (allocation) throw std::bad_alloc();
        throw 197;
      }
    };
    const auto result = error([&] { backend.attachFreshSession(frame(0, freshAnchor()), grid(), 6, relation()); });
    hook = {};
    check(injected && backend.failed() && result.outcome() == (committed ? CommitOutcome::Committed : CommitOutcome::NotCommitted),
          "teardown fixture reaches accepted sticky/commit boundary");
    noCorrection(backend);
    check(error([&] { backend.committedPose(0); }).code() == MapErrorCode::Lifecycle, "sticky failure fences service before teardown");
    check(error([&] { backend.attachFreshSession(frame(0, freshAnchor()), grid(), 6, relation()); }).code() == MapErrorCode::Lifecycle,
          "already-failed backend cannot re-enter attachment");
    hook = [&](const char *stage, void *) {
      if (!std::strcmp(stage, "before-sqlite-close")) closes.push_back("before");
      if (!std::strcmp(stage, "after-sqlite-close")) closes.push_back("sqlite");
      if (!std::strcmp(stage, "after-owner-close")) closes.push_back("owner");
    };
    if (!direct) {
      for (int attempt = 0; attempt < 2; ++attempt) {
        bool reported = false;
        try {
          backend.finish();
        } catch (int value) {
          check(!standard && !allocation && value == 197, "preserve stored integer failure");
          reported = true;
        } catch (const std::bad_alloc &) {
          check(allocation, "preserve stored allocation failure");
          reported = true;
        } catch (const std::runtime_error &e) {
          check(standard && std::string(e.what()) == "teardown standard detail", "preserve stored standard detail");
          reported = true;
        }
        check(reported && backend.failed(), "checked finish must not silently succeed on an already-failed owner");
        ++checked_failures;
        check(error([&] { backend.latestOccupancyGrid(); }).code() == MapErrorCode::Lifecycle, "finish must not revive failed service");
      }
    }
    std::cout << "TEARDOWN before destruction " << kind << " failed=1 explicit_failures=" << checked_failures << std::endl;
  }  // The old catch(std::exception) destructor aborts here for the stored int.
  hook = {};
  check(closes == std::vector<std::string>({"before", "sqlite", "owner"}), "exact SQLite-before-owner-fd close order, including direct destructor");
  if (!committed) check(fileSnapshot(p.parent_path()) == before_files, "teardown changed precommit authoritative files");
  // Same-process exclusive recovery proves descriptor admission was released,
  // not merely that a separate process can read the file after this child exits.
  {
    MapDatabase proof(p.string(), "recover");
    proof.validateHistoricalRecords(.1f);
    proof.finish();
  }
  independentReopen(p, committed ? 7 : 6, revision + (committed ? 1 : 0));
  std::cout << "PASS teardown " << kind << ": process survived; ownership reacquired; authoritative reopen succeeded\n";
}
void teardownRegressions() {
  for (const char *kind : {"entry-nonstandard", "after-nonstandard", "postcommit-nonstandard", "direct-nonstandard", "after-standard",
                           "after-badalloc", "worker-finish-nonstandard"}) {
    const auto child = fork();
    check(child >= 0, "teardown child fork");
    if (child == 0) {
      const auto executable = fs::read_symlink("/proc/self/exe").string();
      execl(executable.c_str(), executable.c_str(), "--teardown-case", kind, nullptr);
      _exit(127);
    }
    int status;
    check(waitpid(child, &status, 0) == child, "teardown waitpid");
    check(WIFEXITED(status) && WEXITSTATUS(status) == 0, std::string("teardown child must survive ") + kind + " status=" + std::to_string(status));
  }
}
void materializationMeasurement(const fs::path &root) {
  const auto path = root / "measured-map.db";
  constexpr int history_size = 128;
  seed(path, false, history_size);
  PoseGraphBackend backend(resumeConfig(), {}, path.string(), {});
  std::chrono::steady_clock::time_point start;
  double elapsed_ms = 0;
  hook = [&](const char *stage, void *) {
    if (std::string(stage) == "b1-postcommit-materialization") start = std::chrono::steady_clock::now();
    if (std::string(stage) == "b1-runtime-ready")
      elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  };
  const auto result = backend.attachFreshSession(frame(0, freshAnchor()), grid(), history_size, relation());
  hook = {};
  check(result.status == AttachmentStatus::Attached && result.root_node_id == history_size + 1, "larger map attachment");
  check(backend.historicalVisualMetadata().size() == history_size + 1 && backend.historicalOccupancy().activeNodeIds().size() == history_size + 1,
        "larger materialization membership");
  backend.finish();
  independentReopen(path, history_size + 1, 2);
  std::cout << "MEASURE committed materialization nodes=" << history_size + 1
            << " gaussians_per_node=120 descriptors_per_node=96 cells_per_node=6 elapsed_ms=" << elapsed_ms << '\n';
}
}  // namespace
int main(int argc, char **argv) {
  const auto root = fs::temp_directory_path() / ("sapphire-b1-" + std::to_string(getpid()));
  try {
    if (argc == 5 && std::string(argv[1]) == "--reopen") {
      reopen(argv[2], std::stoi(argv[3]), std::stoull(argv[4]));
      return 0;
    }
    fs::create_directories(root);
    const auto source = root / "source.db";
    seed(source);
    if (argc == 3 && std::string(argv[1]) == "--teardown-case") {
      teardownCase(source, root, argv[2]);
      fs::remove_all(root);
      return 0;
    }
    if (argc == 3 && std::string(argv[1]) == "--preparation-only") {
      preparationFailures(source, root, argv[2]);
      fs::remove_all(root);
      return 0;
    }
    teardownRegressions();
    preparationFailures(source, root);
    success(source, root);
    rejections(source, root);
    failures(source, root);
    concurrency(source, root);
    materializationMeasurement(root);
    fs::remove_all(root);
    std::cout << "B1 attachment tests passed\n";
    return 0;
  } catch (const std::exception &e) {
    fail_attachment_allocation = false;
    hook = {};
    commit_fault = 0;
    stale_revision = false;
    std::cerr << "FAIL: " << e.what() << " (fixtures " << root << ")\n";
    return 1;
  }
}
