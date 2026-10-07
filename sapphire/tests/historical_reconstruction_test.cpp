#include <cuda_runtime_api.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <sys/wait.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <set>

#include "backend/storage/map_database.hpp"
#include "backend/visual/feature/scene_features.hpp"
#include "backend/grid/occ_layer.hpp"
#include "backend/graph/pose_graph.hpp"
#include "backend/visual/visual_loop.hpp"

using namespace sapphire;
namespace fs = std::filesystem;
static bool reader_only = false;
static int reader_cuda_calls = 0;
extern "C" cudaError_t __real_cudaMalloc(void **, size_t);
extern "C" cudaError_t __real_cudaGetDeviceCount(int *);
extern "C" cudaError_t __real_cudaFree(void *);
extern "C" cudaError_t __wrap_cudaMalloc(void **p, size_t bytes) {
  if (reader_only) {
    ++reader_cuda_calls;
    return cudaErrorNoDevice;
  }
  return __real_cudaMalloc(p, bytes);
}
extern "C" cudaError_t __wrap_cudaGetDeviceCount(int *count) {
  if (reader_only) {
    ++reader_cuda_calls;
    return cudaErrorNoDevice;
  }
  return __real_cudaGetDeviceCount(count);
}
extern "C" cudaError_t __wrap_cudaFree(void *p) {
  if (reader_only) {
    ++reader_cuda_calls;
    return cudaErrorNoDevice;
  }
  return __real_cudaFree(p);
}
static std::function<void(const char *, void *)> a2_hook;
namespace sapphire::database_detail {
void writableStorageTestPoint(const char *point, void *context) {
  if (a2_hook) a2_hook(point, context);
}
}
namespace {
double fixture_offset = 0;
void check(bool value, const std::string &message) {
  if (!value) throw std::runtime_error(message);
}
template <class F>
void errorIs(MapErrorCode code, F f) {
  try {
    f();
  } catch (const MapError &e) {
    check(e.code() == code, "wrong error: " + std::string(e.what()));
    return;
  }
  throw std::runtime_error("expected MapError");
}
Eigen::Isometry3d anchor(int i) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.linear() = (Eigen::AngleAxisd(.4 + .02 * i, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-.16 + .01 * std::sin(i), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(.23, Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  p.translation() = Eigen::Vector3d(fixture_offset + 12 + .14 * i, -4 + .2 * std::sin(.5 * i), 1.2 + .05 * std::cos(i));
  return p;
}
LocalGrid localGrid(int i) {
  LocalGrid grid(.1f);
  grid.viewPoint = {0, 0, .3f};
  grid.groundCells = {{.1f * i, .1f}, {.1f * i + .1f, .2f}};
  grid.obstacleCells = {{.1f * i + .2f, .3f}};
  grid.emptyCells = {{.1f * i + .3f, .4f}};
  return grid;
}
SubmapFrame frame(int id, bool visual) {
  auto cloud = std::make_shared<GaussianCloud>();
  for (int i = 0; i < 40; ++i) {
    GaussianPoint p;
    p.N = 20;
    p.mean = {float(i % 8) * .3f, float(i / 8) * .3f, float(i % 3) * .1f};
    p.covariance = Eigen::Matrix3f::Identity() * .001f;
    p.regularize();
    cloud->push_back(p);
  }
  cpu::VoxelMaps voxels;
  voxels.set_min_res(.25f);
  voxels.create_voxelmaps(cloud->size(), [&](size_t i) { return (*cloud)[i].mean; });
  LioFrame lio;
  lio.pcd = cloud;
  lio.timestamp = id + 1;
  lio.T_odom_base = anchor(id);
  NavigationPath navigation;
  navigation.samples.push_back({double(id + 1), 0, 0, 0, 0, 0, 0, 1, 0});
  std::vector<VisualFrame> images;
  if (visual) {
    VisualFrame image(id + 1, 0);
    std::vector<VisualPoint> points;
    cv::Mat descriptors(96, 32, CV_8UC1);
    std::array<std::mt19937, 4> random{std::mt19937(123), std::mt19937(124), std::mt19937(125), std::mt19937(126)};
    for (int i = 0; i < 96; ++i) {
      points.emplace_back(Eigen::Vector2f(20 + i, 30 + i), i % 4, .5f);
      const int source = id < 4 ? id : (id == 5 ? (i < 64 ? 0 : 1) : (id == 6 ? (i < 48 ? 0 : 1) : 0));
      for (int j = 0; j < 32; ++j)
        for (int k = 0; k < 4; ++k) {
          const auto byte = random[k]() & 255;
          if (k == source) descriptors.at<uint8_t>(i, j) = byte;
        }
    }
    image.update_features(std::move(points), descriptors);
    images.push_back(std::move(image));
  }
  return SubmapFrame(id, std::move(lio), id, id, 1, 0, 0, voxels.release_data(), {{double(id + 1), anchor(id)}}, std::move(navigation),
                     std::move(images));
}
void writeMap(const fs::path &path, int n, bool visual, bool loops) {
  MapDatabase db(path.string());
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = .01;
  parameters.relinearizeSkip = 1;
  gtsam::ISAM2 isam(parameters);
  gtsam::Vector6 variances;
  variances << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
  auto odom = gtsam::noiseModel::Diagonal::Variances(variances);
  auto robust = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(10), odom);
  std::vector<GraphLink> links;
  for (int i = 0; i < n; ++i) {
    auto submap = frame(i, visual);
    db.saveSubmap(submap, localGrid(i), visual ? buildVisualScene(submap)->encode() : std::vector<uint8_t>{});
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    values.insert(i, gtsam::Pose3(anchor(i).matrix()));
    if (!i)
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(anchor(0).matrix()),
                                                 gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(1e-12))));
    else {
      Eigen::Isometry3f z = (anchor(i - 1).inverse() * anchor(i)).cast<float>();
      links.push_back({i, i + 1, 0, z});
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(i - 1, i, gtsam::Pose3(z.matrix().cast<double>().eval()), odom));
    }
    if (loops && i == n - 1 && i > 3) {
      auto z = anchor(i).inverse() * anchor(0);
      z.translation().x() += .035;
      Eigen::Isometry3f stored = z.cast<float>();
      links.push_back({i + 1, 1, 1, stored});
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(i, 0, gtsam::Pose3(stored.matrix().cast<double>().eval()), robust));
    }
    isam.update(graph, values);
    isam.calculateEstimate();
    isam.update();
    isam.calculateEstimateWithAffectedKeys();
  }
  if (n) {
    auto values = isam.calculateEstimate();
    std::vector<std::pair<int, Eigen::Isometry3f>> poses;
    for (int i = 0; i < n; ++i) poses.emplace_back(i + 1, Eigen::Isometry3f(values.at<gtsam::Pose3>(i).matrix().cast<float>()));
    db.saveSubmapPoses(poses, links);
  }
}
std::string bytes(const fs::path &path) {
  std::ifstream f(path, std::ios::binary);
  check(bool(f), "read bytes");
  return {std::istreambuf_iterator<char>(f), {}};
}
std::map<std::string, std::string> files(const fs::path &root) {
  std::map<std::string, std::string> result;
  for (auto &entry : fs::directory_iterator(root))
    if (entry.is_regular_file()) result[entry.path().filename().string()] = bytes(entry.path());
  return result;
}
void mutate(const fs::path &path, const std::string &sql) {
  sqlite3 *db = nullptr;
  check(sqlite3_open(path.c_str(), &db) == SQLITE_OK, "open mutation");
  char *message = nullptr;
  const int rc = sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &message);
  std::string error = message ? message : "";
  sqlite3_free(message);
  sqlite3_close(db);
  check(rc == SQLITE_OK, error);
}
void stageA(const fs::path &root) {
  for (int n : {0, 1, 6, 12}) {
    auto path = root / (std::to_string(n) + ".db");
    writeMap(path, n, n == 12, n == 12);
    auto before = files(root);
    {
      MapDatabase db(path.string(), "resume");
      db.validateHistoricalRecords(.1f);
      int count = 0;
      db.visitHistoricalNodes([&](int id, const auto &a, const auto &, const auto &) {
        check(id == ++count && a.matrix().isApprox(anchor(id - 1).matrix(), 1e-12), "Node enumeration/prior");
      });
      check(count == n, "Node count");
    }
    check(files(root) == before, "validation mutated files");
  }
  int index = 0;
  for (const auto &sql :
       {"DELETE FROM SpatialRecord WHERE node_id=2;", "DELETE FROM Link WHERE type=0 AND from_id=2;", "UPDATE Link SET to_id=999 WHERE type=1;",
        "UPDATE MetaTag SET tag=zeroblob(64) WHERE node_id=1 AND tag_type=2;", "UPDATE Node SET odom_pose_count=0 WHERE id=1;",
        "UPDATE FlatGrid SET ground_count=-1 WHERE node_id=2;", "UPDATE VisualScene SET payload=zeroblob(16) WHERE node_id=1;"}) {
    const auto path = root / ("bad" + std::to_string(index++) + ".db");
    fs::copy_file(root / "12.db", path);
    mutate(path, sql);
    auto before = files(root);
    errorIs(MapErrorCode::HistoricalData, [&] {
      MapDatabase db(path.string(), "resume");
      db.validateHistoricalRecords(.1f);
    });
    check(files(root) == before, "failed validation mutated files");
  }
  std::cout << "PASS historical Node/Link/payload validation and non-mutation\n";
}

void failedBackends(const fs::path &root) {
  int index = 0;
  for (const auto &[sql, code] : std::vector<std::pair<std::string, MapErrorCode>>{
           {"DELETE FROM LaserRecord WHERE node_id=2;", MapErrorCode::HistoricalData},
           {"UPDATE PyramidVoxel SET data=zeroblob(16) WHERE node_id=2;", MapErrorCode::HistoricalData},
           {"UPDATE PyramidVoxel SET data=substr(data,1,24) WHERE node_id=2;", MapErrorCode::HistoricalData},
           {"UPDATE Node SET submap_pose=zeroblob(64) WHERE id=2;", MapErrorCode::HistoricalData},
           {"UPDATE Link SET type=9 WHERE type=1;", MapErrorCode::UnsupportedSemantics},
           {"UPDATE Link SET from_id=to_id,to_id=from_id WHERE type=1;", MapErrorCode::HistoricalData},
           {"UPDATE ImageRecord SET descriptors=zeroblob(length(descriptors)) WHERE node_id=1;", MapErrorCode::HistoricalData}}) {
    const auto path = root / ("backend-bad" + std::to_string(index++) + ".db");
    fs::copy_file(root / "12.db", path);
    mutate(path, sql);
    auto before = files(root);
    PoseGraphParameters config;
    config.map_mode = "resume";
    errorIs(code, [&] { PoseGraphBackend backend(config, {}, path.string(), {}); });
    check(files(root) == before, "failed backend changed persistent bytes");
    const int fd = open(path.c_str(), O_RDONLY | O_CLOEXEC);
    check(fd >= 0 && flock(fd, LOCK_EX | LOCK_NB) == 0, "failed backend retained inode ownership");
    close(fd);
  }
  const auto path = root / "discrepancy.db";
  fs::copy_file(root / "12.db", path);
  Eigen::Isometry3f shifted;
  {
    MapDatabase facts(path.string(), "resume");
    facts.visitHistoricalNodes([&](int id, const auto &, const auto &p, const auto &) {
      if (id == 12) shifted = p;
    });
  }
  shifted.translation().x() += .2f;
  sqlite3 *db = nullptr;
  check(sqlite3_open(path.c_str(), &db) == SQLITE_OK, "open discrepancy fixture");
  {
    database_detail::Statement node(db, "UPDATE Node SET submap_pose=? WHERE id=12;");
    sqlite3_bind_blob(node.get(), 1, shifted.data(), 64, SQLITE_TRANSIENT);
    database_detail::step(db, node.get());
    MetaTag tag(shifted);
    database_detail::Statement pose_tag(db, "UPDATE MetaTag SET tag=? WHERE node_id=12 AND tag_type=2;");
    database_detail::bindMatrix(pose_tag.get(), 1, tag.descriptor());
    database_detail::step(db, pose_tag.get());
  }
  sqlite3_close(db);
  const auto before = files(root);
  PoseGraphParameters config;
  config.map_mode = "resume";
  {
    PoseGraphBackend backend(config, {}, path.string(), {});
    check(backend.reconstructionDiagnostics().max_translation > .035, "coherent displacement exceeds reference");
    check(backend.committedPose(11)->matrix() == shifted.matrix(), "coherent displacement keeps committed pose");
    check(!backend.hasActiveCorrection(), "coherent displacement creates no correction");
    OccupancyGrid expected({});
    for (int i = 0; i < 12; ++i)
      check(expected.append(GridFrame(i + 1, *backend.committedPose(i), localGrid(i))), "committed grid append");
    auto actual = backend.latestOccupancyGrid(); auto oracle = expected.getMap();
    check(actual->data == oracle.cells && actual->origin_x == oracle.originX && actual->origin_y == oracle.originY,
          "coherent displacement keeps committed occupancy");
    backend.finish();
  }
  check(files(root) == before, "solver discrepancy changed map");
  const int fd = open(path.c_str(), O_RDONLY | O_CLOEXEC);
  check(fd >= 0 && flock(fd, LOCK_EX | LOCK_NB) == 0, "diagnostic open retained inode ownership");
  close(fd);
  std::cout << "PASS historical hard failures and successful coherent 0.2 m diagnostic open, committed products, unchanged bytes and released ownership\n";
}

#include "a2_contract_cases.inc"

void rejectedStorage(const fs::path &path) {
  const auto before = files(path.parent_path());
  PoseGraphParameters config;
  config.map_mode = "resume";
  errorIs(MapErrorCode::HistoricalData, [&] { PoseGraphBackend backend(config, {}, path.string(), {}); });
  check(files(path.parent_path()) == before, "storage rejection changed persistent bytes/sidecars");
  const int fd = open(path.c_str(), O_RDONLY | O_CLOEXEC);
  check(fd >= 0 && flock(fd, LOCK_EX | LOCK_NB) == 0, "storage rejection retained inode ownership");
  close(fd);
}

void strictSqliteTypes(const fs::path &root) {
  int index = 0;
  for (const std::string sql :
       {"UPDATE FlatGrid SET ground_count=ground_count+0.5 WHERE node_id=1;",
        "UPDATE FlatGrid SET empty_count='not-a-count',empty=x'' WHERE node_id=1;",
        "UPDATE NaviTrajectory SET sample_count=sample_count+0.5 WHERE node_id=1;",
        "UPDATE LaserRecord SET gaussian_count=gaussian_count+0.5 WHERE node_id=1;",
        "UPDATE ImageRecord SET descriptors_rows=descriptors_rows+4294967296 WHERE node_id=1;",
        "UPDATE ImageRecord SET camera_id=0.5 WHERE node_id=1;", "UPDATE PyramidVoxel SET data=CAST(data AS TEXT) WHERE node_id=1;",
        "UPDATE LaserRecord SET payload=CAST(payload AS TEXT) WHERE node_id=1;",
        "UPDATE VisualScene SET payload=CAST(payload AS TEXT) WHERE node_id=1;",
        // Same conversion pattern at the other historical metadata/payload readers.
        "UPDATE Node SET odom_pose_count=odom_pose_count+0.5 WHERE id=1;", "UPDATE Node SET stamp='not-a-time' WHERE id=1;",
        "UPDATE Node SET odom_poses=CAST(odom_poses AS TEXT) WHERE id=1;",
        "UPDATE Node SET odom_timestamps=CAST(odom_timestamps AS TEXT) WHERE id=1;",
        "UPDATE Node SET submap_pose=CAST(submap_pose AS TEXT) WHERE id=1;", "UPDATE SpatialRecord SET min_x='not-a-bound' WHERE node_id=1;",
        "UPDATE FlatGrid SET obstacle_count=obstacle_count+0.5 WHERE node_id=1;", "UPDATE FlatGrid SET ground=CAST(ground AS TEXT) WHERE node_id=1;",
        "UPDATE FlatGrid SET cell_size='not-a-resolution' WHERE node_id=1;",
        "UPDATE NaviTrajectory SET control_point_count=control_point_count+0.5 WHERE node_id=1;",
        "UPDATE NaviTrajectory SET knot_count=knot_count+0.5 WHERE node_id=1;", "UPDATE NaviTrajectory SET spline_degree=4294967296 WHERE node_id=1;",
        "UPDATE NaviTrajectory SET timestamps=CAST(timestamps AS TEXT) WHERE node_id=1;",
        "UPDATE NaviTrajectory SET control_points=CAST(control_points AS TEXT) WHERE node_id=1;",
        "UPDATE ImageRecord SET frame_index=0.5 WHERE node_id=1;", "UPDATE ImageRecord SET point_count=point_count+0.5 WHERE node_id=1;",
        "UPDATE ImageRecord SET descriptors_cols=4294967328 WHERE node_id=1;",
        "UPDATE ImageRecord SET descriptors_rows=2147483647,descriptors_cols=2147483647 WHERE node_id=1;",
        "UPDATE ImageRecord SET descriptors_type=4294967296 WHERE node_id=1;", "UPDATE ImageRecord SET descriptors_type=-1 WHERE node_id=1;",
        "UPDATE ImageRecord SET points=CAST(points AS TEXT) WHERE node_id=1;",
        "UPDATE ImageRecord SET descriptors=CAST(descriptors AS TEXT) WHERE node_id=1;", "UPDATE ImageRecord SET node_id=1.5 WHERE node_id=1;",
        "UPDATE MetaTag SET tag_rows=4294967300 WHERE node_id=1;", "UPDATE MetaTag SET tag_cols=4.5 WHERE node_id=1;",
        "UPDATE MetaTag SET tag_cv_type=4294967301 WHERE node_id=1;", "UPDATE MetaTag SET tag_type=258 WHERE node_id=1;",
        "UPDATE MetaTag SET tag=CAST(tag AS TEXT) WHERE node_id=1;", "UPDATE Link SET from_id=from_id+0.5 WHERE type=1;",
        "UPDATE Link SET type=0.5 WHERE type=1;", "UPDATE Link SET transform=CAST(transform AS TEXT) WHERE type=1;"}) {
    const auto path = root / ("strict-type-" + std::to_string(index++) + ".db");
    fs::copy_file(root / "12.db", path);
    mutate(path, sql);
    rejectedStorage(path);
  }
  std::cout << "PASS " << index << " strict SQLite storage-class/range cases, unchanged bytes and released ownership\n";
}

void producerPyramid(const fs::path &root) {
  GaussianCloud cloud;
  for (int i = 0; i < 16; ++i) {
    GaussianPoint point;
    point.N = 20;
    point.mean = {float(2 * i), .25f, .5f};
    point.voxel_key.x = i;
    point.covariance = Eigen::Matrix3f::Identity() * .001f;
    point.regularize();
    cloud.push_back(point);
  }
  StateGroup state;
  state.R.setIdentity();
  state.p.setZero();
  SubmapFrameBuffer buffer(1024, 15, 40);
  buffer.push_visual(VisualFrame(1));  // Real writer uses descriptors_type=-1 for an empty ImageRecord.
  check(!buffer.push(MargiFrame(state, std::move(cloud), 1, Eigen::Matrix<double, 6, 1>::Zero(), 0)), "producer buffers short submap");
  auto submap = buffer.flush();
  check(bool(submap), "producer flushes collision fixture");
  const auto &level = submap->pyramid_voxels().level_buckets.front();
  const auto occupied = std::count_if(level.begin(), level.end(), [](const auto &b) { return b.w() != 0; });
  check(level.size() == 2048 && occupied == 80, "real producer reproduces 2048-bucket / 80-entry sparse level");
  const auto path = root / "producer-pyramid.db";
  {
    MapDatabase writer(path.string());
    writer.saveSubmap(*submap, LocalGrid(.1f));
    writer.saveSubmapPose(1, submap->lio().T_odom_base.cast<float>());
  }
  const auto before = files(root);
  for (int repeat = 0; repeat < 2; ++repeat) {
    MapDatabase facts(path.string(), "resume");
    facts.validateHistoricalRecords(.1f);
    const auto images = facts.loadVisualFrames(1);
    check(images.size() == 1 && images.front().descriptors().empty(), "producer empty ImageRecord sentinel preserved");
    const auto restored = facts.loadPyramidVoxel(1);
    const auto &original = submap->pyramid_voxels().level_buckets;
    check(restored.level_buckets == original, "producer pyramid bucket contents preserved exactly");
  }
  PoseGraphParameters config;
  config.map_mode = "resume";
  {
    PoseGraphBackend backend(config, {}, path.string(), {});
    check(backend.reconstructionDiagnostics().nodes == 1 && backend.committedPose(0)->matrix() == Eigen::Matrix4f::Identity(),
          "historical backend opens actual sparse producer map");
  }
  check(files(root) == before, "producer reopen changed persistent bytes/sidecars");
  std::ostringstream encoded(std::ios::binary);
  cpu::save_voxelmaps(encoded, submap->pyramid_voxels());
  const auto payload = encoded.str();
  constexpr std::size_t header = 8 + sizeof(std::uint32_t) + sizeof(float) + 2 * sizeof(int);
  constexpr std::size_t records = header + 2 * sizeof(std::size_t);
  const auto edit = [](std::string &bytes, std::size_t offset, auto value) { std::memcpy(bytes.data() + offset, &value, sizeof(value)); };
  std::vector<std::string> corrupt;
  const auto add = [&](auto modify) {
    auto bytes = payload;
    modify(bytes);
    corrupt.push_back(std::move(bytes));
  };
  add([&](auto &b) { edit(b, header, std::numeric_limits<std::size_t>::max()); });
  add([&](auto &b) { edit(b, header + sizeof(std::size_t), std::numeric_limits<std::size_t>::max()); });
  add([&](auto &b) { edit(b, header, std::size_t(0)); });
  add([&](auto &b) { edit(b, header, std::size_t(2049)); });  // exceeds actual input-population bound
  add([&](auto &b) { edit(b, records, std::uint32_t(2048)); });
  add([&](auto &b) { std::memcpy(b.data() + records + 16, b.data() + records, sizeof(std::uint32_t)); });
  add([&](auto &b) { edit(b, 16, int(33)); });
  add([&](auto &b) { edit(b, 20, int(0)); });
  add([&](auto &b) { edit(b, 12, std::numeric_limits<float>::infinity()); });
  add([&](auto &b) { b.resize(header + sizeof(std::size_t)); });
  add([&](auto &b) { b.pop_back(); });
  add([&](auto &b) { b.push_back('\0'); });
  for (std::size_t i = 0; i < corrupt.size(); ++i) {
    // Direct preflight rejects before the allocation-bearing native decoder is called.
    errorIs(MapErrorCode::HistoricalData, [&] { database_detail::validatePyramidPayload(corrupt[i].data(), corrupt[i].size(), 16); });
    const auto bad = root / ("pyramid-boundary-" + std::to_string(i) + ".db");
    fs::copy_file(path, bad);
    sqlite3 *db = nullptr;
    check(sqlite3_open(bad.c_str(), &db) == SQLITE_OK, "open pyramid mutation");
    {
      database_detail::Statement update(db, "UPDATE PyramidVoxel SET data=? WHERE node_id=1;");
      sqlite3_bind_blob(update.get(), 1, corrupt[i].data(), static_cast<int>(corrupt[i].size()), SQLITE_TRANSIENT);
      database_detail::step(db, update.get());
    }
    sqlite3_close(db);
    rejectedStorage(bad);
  }
  std::cout << "PASS real SubmapFrameBuffer -> MapDatabase -> historical reopen: 2048 buckets / 80 entries; " << corrupt.size()
            << " unsafe pyramid payloads rejected before decode\n";
}

void optionalAppearance(const fs::path &root) {
  const auto path = root / "absent-scene.db";
  fs::copy_file(root / "12.db", path);
  mutate(path, "DELETE FROM VisualScene WHERE node_id=1;");
  const auto before = files(root);
  PoseGraphParameters config;
  config.map_mode = "resume";
  {
    PoseGraphBackend backend(config, {}, path.string(), {});
    check(backend.historicalVisualCandidates(4).empty() && backend.historicalVisualMetadata().empty(), "absent target scene remains absent");
  }
  check(files(root) == before, "absent scene was regenerated");
  const auto empty_path = root / "empty-scene.db";
  fs::copy_file(root / "12.db", empty_path);
  mutate(empty_path, "DELETE FROM ImageRecord WHERE node_id=1;");
  const auto payload = buildVisualScene(frame(0, false))->encode();
  sqlite3 *db = nullptr;
  sqlite3_open(empty_path.c_str(), &db);
  {
    database_detail::Statement update(db, "UPDATE VisualScene SET payload=? WHERE node_id=1;");
    sqlite3_bind_blob(update.get(), 1, payload.data(), payload.size(), SQLITE_TRANSIENT);
    database_detail::step(db, update.get());
  }
  sqlite3_close(db);
  {
    PoseGraphBackend backend(config, {}, empty_path.string(), {});
    check(backend.historicalVisualCandidates(4).empty(), "empty persisted scene is valid");
  }
  const auto cell_path = root / "grid-cell-size.db";
  fs::copy_file(root / "1.db", cell_path);
  mutate(cell_path, "UPDATE FlatGrid SET cell_size=0.7;");
  {
    PoseGraphBackend backend(config, {}, cell_path.string(), {});
    check(backend.latestOccupancyGrid()->resolution == float(.1), "LocalGrid metric evidence is not rescaled by cell size");
  }
  std::cout << "PASS absent/empty appearance semantics and independent LocalGrid/occupancy resolution\n";
}

void readHistory(const fs::path &path, int n, const fs::path &report) {
  reader_only = true;
  setenv("CUDA_VISIBLE_DEVICES", "", 1);
  const auto before = files(path.parent_path());
  PoseGraphParameters config;
  config.map_mode = "resume";
  config.enabled = false;
  config.visual.enabled = false;
  config.visual.top_k = 5;
  config.visual.min_matches = 24;
  NaviMapParameters grid_config;
  grid_config.enabled = true;  // Explicit legacy schema-1 occupancy reconstruction oracle.
  std::ostringstream semantics;
  std::ofstream numerical(report.string() + ".numbers");
  int publications = 0;
  {
    PoseGraphBackend backend(config, grid_config, path.string(), [&](auto) { ++publications; });
    check(files(path.parent_path()) == before, "construction changed persistent bytes");
    check(backend.historical() && !backend.enabled() && !backend.hasActiveCorrection(), "historical mode capability gates");
    errorIs(MapErrorCode::CorrectionUnavailable, [&] { (void)backend.T_map_odom(); });
    check(backend.loopDecisionStatus() == LoopDecisionStatus::Unavailable, "loop status must be unavailable");
    for (int id : {0, n, n + 1}) errorIs(MapErrorCode::ReadOnly, [&] { backend.addFrame(frame(id, false)); });
    check(files(path.parent_path()) == before, "rejected input changed persistent bytes");
    MapDatabase facts(path.string(), "resume");
    check(backend.mapUuid() == facts.mapUuid() && backend.graphRevision() == facts.graphRevision(), "identity/revision");
    const auto diagnostics = backend.reconstructionDiagnostics();
    const auto links = facts.loadGraphLinks();
    check(diagnostics.nodes == size_t(n) && diagnostics.factors == links.size() + (n ? 1 : 0) &&
              diagnostics.odometry_factors == size_t(n ? n - 1 : 0) && diagnostics.loop_factors == links.size() - size_t(n ? n - 1 : 0),
          "graph counts");
    check(diagnostics.max_translation <= .035 && diagnostics.max_rotation <= .003 && diagnostics.max_residual_translation <= .026 &&
              diagnostics.max_residual_rotation <= .003 && std::isfinite(diagnostics.objective_delta),
          "optimizer support envelope");
    numerical << std::setprecision(17) << diagnostics.max_translation << ' ' << diagnostics.max_rotation << ' '
              << diagnostics.max_residual_translation << ' ' << diagnostics.max_residual_rotation << ' ' << diagnostics.objective_delta << '\n';
    semantics << backend.mapUuid() << ' ' << backend.graphRevision() << ' ' << diagnostics.nodes << ' ' << diagnostics.factors << '\n';
    std::vector<Eigen::Isometry3f> poses;
    std::vector<AABB> bounds;
    facts.visitHistoricalNodes([&](int id, const auto &original, const auto &committed, const auto &box) {
      check(backend.committedPose(id - 1).has_value() && (backend.committedPose(id - 1)->matrix().array() == committed.matrix().array()).all(),
            "committed pose lookup");
      if (id == 1)
        check(diagnostics.original_prior && diagnostics.original_prior->matrix().isApprox(original.matrix(), 1e-12), "original prior provenance");
      poses.push_back(committed);
      bounds.push_back(box);
      semantics << id << ' ';
      for (int j = 0; j < 16; ++j) semantics << std::hexfloat << committed.data()[j] << ' ';
      semantics << '\n';
    });
    check(!backend.committedPose(n), "out-of-range pose absent");
    if (!n) check(!diagnostics.original_prior, "empty map has no fabricated prior");
    if (n == 12)
      check((poses.back().matrix().cast<double>() - anchor(n - 1).matrix()).norm() > .001, "loop fixture must have changed committed pose");

    OccupancyGrid expected(grid_config), uninterrupted(grid_config);
    std::map<std::pair<int, int>, std::tuple<bool, bool, int>> ownership;
    for (int i = 0; i < n; ++i) {
      LocalGrid grid;
      check(facts.loadLocalGrid(i + 1, grid), "persisted grid");
      check(expected.append(GridFrame(i + 1, poses[i], grid)), "reference grid append");
      check(uninterrupted.append(GridFrame(i + 1, anchor(i).cast<float>(), grid)), "original occupancy append");
      std::map<std::pair<int, int>, bool> per_node;
      for (auto *points : {&grid.groundCells, &grid.emptyCells, &grid.obstacleCells})
        for (auto &point : *points) {
          const Eigen::Vector3f world = poses[i] * Eigen::Vector3f(point.x(), point.y(), 0);
          auto &hit =
              per_node[{int(std::floor(world.x() / float(grid_config.resolution))), int(std::floor(world.y() / float(grid_config.resolution)))}];
          hit = hit || points == &grid.obstacleCells;
        }
      for (auto &[cell, hit] : per_node) ownership[cell] = {true, hit, i + 1};
    }
    const auto output = backend.latestOccupancyGrid();
    const auto expected_map = expected.getMap();
    if (n) {
      auto update = uninterrupted.update([&](int id, LocalGrid &grid) { return facts.loadLocalGrid(id, grid); });
      for (int i = 0; i < n; ++i) {
        LocalGrid grid;
        facts.loadLocalGrid(i + 1, grid);
        update -= i + 1;
        update += GridFrame(i + 1, poses[i], grid);
      }
      check(update.commit(), "uninterrupted occupancy refusion");
      const auto live = uninterrupted.getMap();
      check(live.resolution == expected_map.resolution && live.originX == expected_map.originX && live.originY == expected_map.originY &&
                live.width == expected_map.width && live.height == expected_map.height && live.cells == expected_map.cells,
            "uninterrupted refusion differs from historical occupancy semantics");
    }
    check(output && output->map_uuid == backend.mapUuid() && output->source_graph_revision == backend.graphRevision(), "grid source identity");
    check(output->resolution == expected_map.resolution && output->origin_x == expected_map.originX && output->origin_y == expected_map.originY &&
              output->width == expected_map.width && output->height == expected_map.height && output->data == expected_map.cells,
          "exported occupancy semantics");
    check(backend.historicalOccupancy().activeNodeIds() == expected.activeNodeIds(), "active occupancy node membership");
    std::map<std::pair<int, int>, std::tuple<bool, bool, int>> reconstructed;
    backend.historicalOccupancy().visitEvidence([&](int x, int y, bool observed, bool occupied, int owner) {
      reconstructed[{x, y}] = {observed, occupied, owner};
    });
    check(reconstructed == ownership, "occupancy owner/observed/occupied evidence");
    semantics << output->width << ' ' << output->height << ' ' << output->origin_x << ' ' << output->origin_y << '\n';
    for (auto cell : output->data) semantics << int(cell) << ' ';
    semantics << '\n';

    std::vector<int> queries;
    for (int i = 0; i < n; ++i) queries.push_back(i);
    for (int i = n - 1; i >= 0; --i) queries.push_back(i);
    for (int q : queries) {
      const auto candidates = backend.historicalSpatialCandidates(q);
      std::set<uint64_t> actual, reference;
      const auto query_bounds = bounds[q].transform(poses[q]);
      for (int target = 0; target < q - 3; ++target)
        if (query_bounds.xyOverlapRatio(bounds[target].transform(poses[target])) > .5f) reference.insert(target);
      for (auto &candidate : candidates) {
        actual.insert(candidate.submap_id);
        check((candidate.map_T_submap.matrix().array() == poses[candidate.submap_id].matrix().array()).all(), "candidate committed pose");
      }
      check(actual == reference, "spatial candidate membership");
      auto visual = backend.historicalVisualCandidates(q);
      const auto metadata = backend.historicalVisualMetadata();
      mapping::DescriptorArchive oracle;
      for (int target = 0; target < q - 3; ++target) {
        auto payload = facts.loadVisualScene(target + 1);
        if (!payload.empty()) {
          auto scene = mapping::scene::FeatureMap::decode(payload.data(), payload.size());
          if (!scene->points().empty()) oracle.upsert(target + 1, *scene);
          oracle.poll_training(true);
        }
      }
      check(metadata == oracle.node_metadata(), "visual descriptor membership/fingerprint/eligible IDs");
      const auto query_frames = facts.loadVisualFrames(q + 1);
      if (!query_frames.empty()) {
        const auto &image = query_frames[0];
        std::vector<cv::KeyPoint> points;
        for (const auto &point : image.points()) points.emplace_back(point.pixel().x(), point.pixel().y(), 31, -1, point.response(), point.level());
        auto features = features::FeatureBlock::create_raw(points, {}, image.descriptors());
        const auto matches = oracle.search(*features);
        std::vector<std::pair<int, mapping::DescriptorArchive::Candidate>> ranked(matches.begin(), matches.end());
        std::sort(ranked.begin(), ranked.end(), [](const auto &a, const auto &b) {
          return a.second.score != b.second.score ? a.second.score > b.second.score : a.first < b.first;
        });
        if (ranked.size() > 5) ranked.resize(5);
        std::vector<std::pair<size_t, uint64_t>> expected_matches;
        for (const auto &[node, candidate] : ranked)
          if (candidate.matches.size() >= 24) expected_matches.emplace_back(candidate.matches.size(), node - 1);
        std::sort(expected_matches.begin(), expected_matches.end(),
                  [](auto a, auto b) { return a.first != b.first ? a.first > b.first : a.second < b.second; });
        check(visual.size() == expected_matches.size(), "historical visual candidate count");
        for (size_t i = 0; i < visual.size(); ++i)
          check(visual[i].matches == expected_matches[i].first && visual[i].target_id == expected_matches[i].second,
                "historical scores and existing ID tie order");
      }
      for (auto &match : visual) check(q > 3 && match.target_id < uint64_t(q - 3), "visual temporal exclusion");
      if (n == 12 && q == 4) check(!visual.empty() && visual.front().target_id == 0, "persisted query descriptors retrieve sole eligible target");
      if (n == 12 && q == 5)
        check(visual.size() == 2 && visual[0].target_id == 0 && visual[1].target_id == 1 && visual[0].matches > visual[1].matches,
              "strict visual score order");
      if (n == 12 && q == 6)
        check(visual.size() == 2 && visual[0].target_id == 0 && visual[1].target_id == 1 && visual[0].matches == visual[1].matches,
              "existing visual tie rule");
      if (n != 12) check(visual.empty(), "LiDAR-only history yields no appearance candidates");
      semantics << "query " << q << " spatial";
      for (auto id : actual) semantics << ' ' << id;
      semantics << " visual";
      for (auto &match : visual) semantics << ' ' << match.target_id << ':' << match.matches;
      semantics << '\n';
    }
    check(files(path.parent_path()) == before, "historical queries changed persistent bytes");
  }
  check(publications == 0 && reader_cuda_calls == 0, "historical reader published or called CUDA");
  check(files(path.parent_path()) == before, "destruction changed persistent bytes/sidecars");
  std::ofstream(report) << semantics.str();
}

void child(const fs::path &exe, const std::vector<std::string> &args) {
  const auto pid = fork();
  check(pid >= 0, "fork");
  if (!pid) {
    std::vector<char *> argv;
    argv.push_back(const_cast<char *>(exe.c_str()));
    for (auto &a : args) argv.push_back(const_cast<char *>(a.c_str()));
    argv.push_back(nullptr);
    execv(exe.c_str(), argv.data());
    _exit(90);
  }
  int status;
  check(waitpid(pid, &status, 0) == pid && WIFEXITED(status) && WEXITSTATUS(status) == 0, "independent child failed");
}
void processes(const fs::path &root, const fs::path &exe) {
  fs::create_directory(root / "maps");
  for (int n : {0, 1, 6, 12, 32}) {
    auto map = root / "maps" / (std::to_string(n) + ".db");
    child(exe, {"--write", map.string(), std::to_string(n)});
    const auto before = files(root / "maps");
    fs::permissions(map, fs::perms::owner_read, fs::perm_options::replace);
    fs::permissions(root / "maps", fs::perms::owner_read | fs::perms::owner_exec, fs::perm_options::replace);
    auto first = root / (std::to_string(n) + "-first.txt"), second = root / (std::to_string(n) + "-second.txt");
    child(exe, {"--read", map.string(), std::to_string(n), first.string()});
    child(exe, {"--read", map.string(), std::to_string(n), second.string()});
    check(bytes(first) == bytes(second), "cross-process persistent and derived semantic equality");
    check(files(root / "maps") == before, "cross-process reconstruction changed persistent directory");
    fs::permissions(root / "maps", fs::perms::owner_all, fs::perm_options::replace);
    std::cout << "PASS cross-process history n=" << n << " numerical B=" << bytes(first.string() + ".numbers")
              << " numerical C=" << bytes(second.string() + ".numbers");
  }
}
void gpuWriter(const fs::path &path, int n) {
  PoseGraphParameters config;
  config.update_period_sec = .01;
  NaviMapParameters grid;
  grid.enabled = true;  // Match the legacy grid oracle used by gpuCycle's readers.
  PoseGraphBackend backend(config, grid, path.string(), {});
  for (int i = 0; i < n; ++i) backend.addFrame(frame(i, false));
}
void gpuCycle(const fs::path &root, const fs::path &exe) {
  fs::create_directory(root / "maps");
  const auto path = root / "maps" / "gpu.db";
  child(exe, {"--gpu-write", path.string(), "6"});
  const auto before = files(root / "maps");
  child(exe, {"--read", path.string(), "6", (root / "gpu-first.txt").string()});
  child(exe, {"--read", path.string(), "6", (root / "gpu-second.txt").string()});
  check(bytes(root / "gpu-first.txt") == bytes(root / "gpu-second.txt") && files(root / "maps") == before, "GPU-writer/CPU-reader history");
  std::cout << "PASS GPU backend writer then two independent CPU-only historical readers\n";
}
}  // namespace
int main(int argc, char **argv) {
  if (argc > 1 && std::string(argv[1]) != "--gpu-cycle") try {
      if (std::stoi(argv[3]) == 32) fixture_offset = 19970;
      if (std::string(argv[1]) == "--write")
        writeMap(argv[2], std::stoi(argv[3]), std::stoi(argv[3]) == 12, std::stoi(argv[3]) == 12);
      else if (std::string(argv[1]) == "--read")
        readHistory(argv[2], std::stoi(argv[3]), argv[4]);
      else if (std::string(argv[1]) == "--gpu-write")
        gpuWriter(argv[2], std::stoi(argv[3]));
      else
        throw std::runtime_error("unknown mode");
      return 0;
    } catch (const std::exception &e) {
      std::cerr << e.what() << '\n';
      return 1;
    }
  auto pattern = (fs::temp_directory_path() / "sapphire-a2-XXXXXX").string();
  check(mkdtemp(pattern.data()) != nullptr, "create unique test directory");
  const fs::path root(pattern);
  try {
    if (argc > 1)
      gpuCycle(root, fs::canonical("/proc/self/exe"));
    else {
      stageA(root);
      failedBackends(root);
      amendmentContract(root);
      strictSqliteTypes(root);
      producerPyramid(root);
      optionalAppearance(root);
      processes(root, fs::canonical("/proc/self/exe"));
    }
    fs::remove_all(root);
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL " << e.what() << " fixtures " << root << '\n';
    return 1;
  }
}
