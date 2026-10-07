#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <locale>
#include <map>
#include <string>

#include "backend/storage/map_database.hpp"
#include "pipeline.hpp"
#include "backend/graph/pose_graph.hpp"

using namespace sapphire;
namespace fs = std::filesystem;

namespace {
void check(bool value, const std::string &message) {
  if (!value) throw std::runtime_error(message);
}
template <class F>
void errorIs(MapErrorCode code, F operation) {
  try {
    operation();
  } catch (const MapError &error) {
    check(error.code() == code, "wrong MapError code: " + std::string(error.what()));
    return;
  }
  throw std::runtime_error("expected a typed map error");
}
std::string bytes(const fs::path &path) {
  std::ifstream input(path, std::ios::binary);
  check(bool(input), "read file bytes");
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}
std::map<std::string, std::string> files(const fs::path &root) {
  std::map<std::string, std::string> result;
  for (const auto &entry : fs::recursive_directory_iterator(root)) {
    const auto key = entry.path().lexically_relative(root).string();
    const auto type = entry.symlink_status().type();
    if (type == fs::file_type::symlink)
      result[key] = "<symlink>" + fs::read_symlink(entry.path()).string();
    else if (type == fs::file_type::regular)
      result[key] = bytes(entry.path());
    else
      result[key] = "<type:" + std::to_string(static_cast<int>(type)) + ">";
  }
  return result;
}
template <class F>
void unchanged(const fs::path &root, F operation) {
  const auto before = files(root);
  operation();
  check(files(root) == before, "open/rejection changed database bytes or directory contents");
}
void sql(sqlite3 *db, const std::string &query) {
  char *message = nullptr;
  const int rc = sqlite3_exec(db, query.c_str(), nullptr, nullptr, &message);
  const std::string error = message ? message : "";
  sqlite3_free(message);
  check(rc == SQLITE_OK, "SQL: " + error);
}
void mutate(const fs::path &path, const std::string &query) {
  sqlite3 *db = nullptr;
  check(sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READWRITE, nullptr) == SQLITE_OK, "open mutation fixture");
  sql(db, query);
  check(sqlite3_close(db) == SQLITE_OK, "close mutation fixture");
}
std::string rows(sqlite3 *db, const std::string &query) {
  database_detail::Statement statement(db, query.c_str());
  std::ostringstream out;
  int rc;
  while ((rc = sqlite3_step(statement.get())) == SQLITE_ROW) {
    for (int i = 0; i < sqlite3_column_count(statement.get()); ++i) {
      const int n = sqlite3_column_bytes(statement.get(), i);
      out << sqlite3_column_type(statement.get(), i) << ':' << n << ':';
      if (n) out.write(static_cast<const char *>(sqlite3_column_blob(statement.get(), i)), n);
      out << '|';
    }
    out << '\n';
  }
  check(rc == SQLITE_DONE, "read verification rows");
  return out.str();
}
std::string contents(sqlite3 *db) {
  std::string result = rows(db, "PRAGMA user_version;") + rows(db, "SELECT type,name,tbl_name,sql FROM sqlite_master ORDER BY type,name;");
  for (const char *table : {"MapState", "Node", "Link", "SpatialRecord", "MetaTag", "NaviTrajectory", "LaserRecord", "ImageRecord", "VisualScene",
                            "PyramidVoxel", "FlatGrid"})
    result += rows(db, std::string("SELECT * FROM ") + table + " ORDER BY rowid;");
  return result;
}
SubmapFrame submap(std::uint64_t id) {
  auto cloud = std::make_shared<GaussianCloud>();
  for (int i = 0; i < 80; ++i) {
    GaussianPoint point;
    point.N = 20;
    point.mean = {i * .1f, (i % 3) * .1f, 0};
    point.covariance = Eigen::Matrix3f::Identity() * .001f;
    point.regularize();
    cloud->push_back(point);
  }
  cpu::VoxelMaps voxels;
  voxels.set_min_res(.25f);
  voxels.create_voxelmaps(cloud->size(), [&](std::size_t i) { return (*cloud)[i].mean; });
  LioFrame lio;
  lio.pcd = cloud;
  lio.timestamp = id + 1;
  NavigationPath navigation;
  navigation.samples.push_back({double(id + 1), 0, 0, 0, 0, 0, 0, 1, 0});
  return SubmapFrame(id, std::move(lio), id, id, 1, 0, 0, voxels.release_data(), {{double(id + 1), Eigen::Isometry3d::Identity()}},
                     std::move(navigation), {});
}
void create(const fs::path &path) { MapDatabase database(path.string()); }

void pathSemantics(const fs::path &root) {
  const auto paths = root / "paths";
  fs::create_directories(paths / "real" / "inner");
  fs::create_directory_symlink(paths / "real" / "inner", paths / "alias");
  const auto requested = paths / "alias" / ".." / "map.db";
  const auto actual = paths / "real" / "map.db";
  const auto wrong = paths / "map.db";
  std::ofstream(wrong) << "lexical destination must not be touched";
  const auto wrong_bytes = bytes(wrong);
  std::string uuid;
  {
    MapDatabase writer(requested.string());
    uuid = writer.mapUuid();
    check(fs::equivalent(requested, actual) && fs::equivalent(writer.path(), actual), "open and reported path name actual requested inode");
    check(!fs::equivalent(actual, wrong) && bytes(wrong) == wrong_bytes, "new must not use lexically normalized destination");
    errorIs(MapErrorCode::WriterConflict, [&] { MapDatabase reader(actual.string(), "resume"); });
    errorIs(MapErrorCode::WriterConflict, [&] { MapDatabase reader(requested.string(), "resume"); });
  }
  unchanged(paths, [&] {
    errorIs(MapErrorCode::DestinationExists, [&] { create(requested); });
    MapDatabase reader(requested.string(), "resume");
    check(reader.mapUuid() == uuid && fs::equivalent(reader.path(), actual), "resume must read actual requested map");
  });
  // With the lexical destination absent, a collision must not create it either.
  fs::remove(wrong);
  unchanged(paths, [&] { errorIs(MapErrorCode::DestinationExists, [&] { create(requested); }); });
  check(!fs::exists(wrong), "rejected new created a second map");

  for (const char *name : {"missing-map.db", "map.db"}) {
    const auto missing_parent = paths / "absent" / ".." / name;
    unchanged(paths, [&] {
      errorIs(MapErrorCode::Storage, [&] { create(missing_parent); });
      errorIs(MapErrorCode::MissingMap, [&] { MapDatabase reader(missing_parent.string(), "resume"); });
    });
  }
  // A valid map at the accidentally-normalized name must not make this request valid.
  create(wrong);
  unchanged(paths, [&] {
    errorIs(MapErrorCode::Storage, [&] { create(paths / "absent" / ".." / "map.db"); });
    errorIs(MapErrorCode::MissingMap, [&] { MapDatabase reader((paths / "absent" / ".." / "map.db").string(), "resume"); });
  });
  for (const bool relative : {false, true}) {
    const auto target = paths / (relative ? "relative.db" : "absolute.db");
    const auto input = relative ? target.lexically_relative(fs::current_path()) : target;
    {
      MapDatabase writer(input.string());
      uuid = writer.mapUuid();
      check(fs::equivalent(input, target) && fs::equivalent(writer.path(), target), "ordinary path opens intended inode");
    }
    unchanged(paths, [&] {
      MapDatabase reader(input.string(), "resume");
      check(reader.mapUuid() == uuid, "ordinary path reopens same map");
      errorIs(MapErrorCode::WriterConflict, [&] { create(input); });
    });
  }
  fs::create_symlink(actual, paths / "leaf-link");
  unchanged(paths, [&] {
    errorIs(MapErrorCode::Storage, [&] { MapDatabase reader((paths / "leaf-link").string(), "resume"); });
    errorIs(MapErrorCode::DestinationExists, [&] { create(paths / "leaf-link"); });
  });
  std::cout << "PASS filesystem traversal, actual inode ownership, relative/absolute paths and rejected-path non-mutation\n";
}

void specialFiles(const fs::path &root) {
  const auto special = root / "special";
  fs::create_directory(special);
  const auto fifo = special / "fifo";
  check(mkfifo(fifo.c_str(), 0600) == 0, "create FIFO with no writer");
  const auto socket_path = special / "socket";
  const int socket_fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  check(socket_fd >= 0, "create socket");
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  check(socket_path.string().size() < sizeof(address.sun_path), "socket path length");
  std::strcpy(address.sun_path, socket_path.c_str());
  check(bind(socket_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0, "bind socket fixture");
  for (const auto &target : {fifo, special, socket_path}) {
    struct stat before {};
    check(lstat(target.c_str(), &before) == 0, "inspect non-regular inode");
    unchanged(special, [&] {
      const pid_t child = fork();
      check(child >= 0, "fork bounded special-file check");
      if (child == 0) {
        alarm(2);  // Test watchdog only; production open must return without it.
        try {
          errorIs(MapErrorCode::Storage, [&] { MapDatabase reader(target.string(), "resume"); });
          _exit(0);
        } catch (...) {
          _exit(1);
        }
      }
      int status = 0;
      check(waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
            "non-regular resume must return Storage before the two-second watchdog");
    });
    struct stat after {};
    check(lstat(target.c_str(), &after) == 0 && before.st_ino == after.st_ino && before.st_dev == after.st_dev && before.st_mode == after.st_mode,
          "non-regular rejection changed the target inode/type");
  }
  close(socket_fd);
  std::cout << "PASS bounded FIFO, directory and socket rejection without map mutation\n";
}

void compatibility(const fs::path &root, const fs::path &valid) {
  const PoseGraphParameters graph;
  const NaviMapParameters grid;
  const auto identity = map_config_identity(graph, grid);
  check(identity == "c2:9477d0eb7e4b7acd", "fixed config-2 encoding vector");
  for (auto field : {&NaviMapParameters::clear_height_eps, &NaviMapParameters::margin, &NaviMapParameters::d_max,
                     &NaviMapParameters::ground_plane_tolerance, &NaviMapParameters::ground_max_correction, &NaviMapParameters::h_clearance,
                     &NaviMapParameters::ground_margin, &NaviMapParameters::usable_range, &NaviMapParameters::min_range}) {
    auto changed = grid;
    changed.*field *= 2;
    unchanged(root, [&] { MapDatabase reader(valid.string(), "resume", map_config_identity(graph, changed)); });
  }
  auto policy = graph;
  policy.submap_voxel_size *= 2;
  policy.submap_travel_distance *= 2;
  policy.submap_max_point_range *= 2;
  policy.visual.enabled = true;
  policy.visual.mode = "stereo";
  policy.visual.global_xy_window *= 2;
  policy.visual.global_z_window *= 2;
  policy.visual.top_k *= 2;
  policy.visual.min_matches *= 2;
  policy.visual.image_interval *= 2;
  policy.visual.max_features *= 2;
  policy.visual.max_frames *= 2;
  for (auto *camera : {&policy.visual.left, &policy.visual.right}) {
    camera->width = 640;
    camera->height = 480;
    camera->input_width = 1280;
    camera->input_height = 960;
    camera->time_offset = .01;
    camera->intrinsics = {400, 400, 320, 240};
    camera->distortion = {.1, 0, 0, 0, 0};
    camera->camera_to_imu_rotation = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    camera->camera_to_imu_translation = {.1, .2, .3};
  }
  auto switches = grid;
  switches.enabled = false;
  switches.adaptive_ground = false;
  // Disabled navigation now explicitly selects geometry-only schema 2; it must
  // not reinterpret a legacy archive as if its grid evidence never existed.
  unchanged(root, [&] {
    errorIs(MapErrorCode::UnsupportedFormat, [&] { MapDatabase reader(valid.string(), "resume", map_config_identity(policy, switches)); });
  });
  for (auto field : {&NaviMapParameters::resolution, &NaviMapParameters::occ_threshold, &NaviMapParameters::hit_probability,
                     &NaviMapParameters::miss_probability, &NaviMapParameters::clamp_min_probability, &NaviMapParameters::clamp_max_probability}) {
    auto changed = grid;
    changed.*field *= .9;
    unchanged(root, [&] {
      errorIs(MapErrorCode::IncompatibleConfig, [&] { MapDatabase reader(valid.string(), "resume", map_config_identity(graph, changed)); });
    });
  }
  const auto old = root / "old-config.db";
  fs::copy_file(valid, old);
  mutate(old, "UPDATE MapState SET config_identity='a1:0123456789abcdef';");
  unchanged(root, [&] { errorIs(MapErrorCode::IncompatibleConfig, [&] { MapDatabase reader(old.string(), "resume"); }); });
  const pid_t child = fork();
  check(child >= 0, "fork encoding check");
  if (child == 0) {
    struct CommaNumbers : std::numpunct<char> {
      char do_decimal_point() const override { return ','; }
      char do_thousands_sep() const override { return '.'; }
      std::string do_grouping() const override { return "\3"; }
    };
    std::locale::global(std::locale(std::locale::classic(), new CommaNumbers));
    _exit(map_config_identity(graph, grid) == identity ? 0 : 1);
  }
  int status = 0;
  check(waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0, "encoding differs by process or locale");
  std::cout << "PASS compatibility-only fields, critical occupancy mismatches, encoding version and deterministic locale-independent identity\n";
}
}  // namespace

int main() {
  char directory[] = "/tmp/sapphire-a1-XXXXXX";
  const fs::path root = mkdtemp(directory);
  fs::create_directory(root);
  try {
    const auto valid = root / "valid ?# map.db";
    const auto first = submap(0), second = submap(1);
    std::string uuid;
    {
      MapDatabase database(valid.string());
      uuid = database.mapUuid();
      check(uuid.size() == 36 && database.graphRevision() == 0, "new supported identity/revision");
      database.saveSubmap(first, LocalGrid{});
      database.saveSubmap(second, LocalGrid{});
      database.saveLink(1, 2, 0, Eigen::Isometry3f::Identity());
      sqlite3 *reader = nullptr;
      check(sqlite3_open(valid.c_str(), &reader) == SQLITE_OK, "open logical verifier");
      const auto before = contents(reader);
      errorIs(MapErrorCode::DuplicateNode, [&] { database.saveSubmap(first, LocalGrid{}); });
      check(contents(reader) == before, "duplicate changed payloads, links, identity or revision");
      errorIs(MapErrorCode::WriterConflict, [&] { MapDatabase other(valid.string(), "resume"); });
      check(contents(reader) == before, "conflicting opener changed canonical state");
      sql(reader, "BEGIN IMMEDIATE;");
      errorIs(MapErrorCode::WriterConflict, [&] { database.saveSubmapPose(1, Eigen::Isometry3f::Identity()); });
      sql(reader, "ROLLBACK;");
      check(contents(reader) == before, "conflicting SQLite writer changed canonical state");
      check(sqlite3_close(reader) == SQLITE_OK, "close logical verifier");
    }
    unchanged(root, [&] {
      MapDatabase eligible(valid.string(), "resume");
      check(!eligible.writable() && eligible.mapUuid() == uuid && eligible.graphRevision() == 1, "resume eligibility only");
      errorIs(MapErrorCode::ReadOnly, [&] { eligible.saveSubmap(submap(2), LocalGrid{}); });
      errorIs(MapErrorCode::ReadOnly, [&] { eligible.saveLink(1, 2, 1, Eigen::Isometry3f::Identity()); });
    });
    std::cout << "PASS new insertion, duplicate preservation, writer conflict, read-only eligibility\n";
    pathSemantics(root);
    specialFiles(root);
    compatibility(root, valid);

    const auto empty = root / "empty";
    std::ofstream(empty).close();
    const auto unrelated = root / "unrelated";
    std::ofstream(unrelated) << "not a SQLite map";
    for (const auto &path : {valid, empty, unrelated}) {
      unchanged(root, [&] { errorIs(MapErrorCode::DestinationExists, [&] { create(path); }); });
      unchanged(root, [&] { errorIs(MapErrorCode::UnsupportedMode, [&] { MapDatabase db(path.string(), "localize"); }); });
    }
    unchanged(root, [&] { errorIs(MapErrorCode::MissingMap, [&] { MapDatabase db((root / "missing").string(), "resume"); }); });
    unchanged(root, [&] { errorIs(MapErrorCode::UnsupportedMode, [&] { MapDatabase db((root / "missing").string(), "localize"); }); });
    unchanged(root, [&] { errorIs(MapErrorCode::Storage, [&] { create(root / "absent-parent" / "map.db"); }); });
    for (const auto &path : {empty, unrelated})
      unchanged(root, [&] { errorIs(MapErrorCode::UnsupportedFormat, [&] { MapDatabase db(path.string(), "resume"); }); });

    const auto legacy = root / "legacy.db";
    fs::copy_file(valid, legacy);
    mutate(legacy, "PRAGMA user_version=0;");
    unchanged(root, [&] { errorIs(MapErrorCode::UnsupportedFormat, [&] { MapDatabase db(legacy.string(), "resume"); }); });
    const auto unknown = root / "unknown.db";
    fs::copy_file(valid, unknown);
    mutate(unknown, "PRAGMA user_version=999;");
    unchanged(root, [&] { errorIs(MapErrorCode::UnsupportedFormat, [&] { MapDatabase db(unknown.string(), "resume"); }); });
    const auto missing_table = root / "missing-table.db";
    fs::copy_file(valid, missing_table);
    mutate(missing_table, "DROP TABLE VisualScene;");
    unchanged(root, [&] { errorIs(MapErrorCode::UnsupportedFormat, [&] { MapDatabase db(missing_table.string(), "resume"); }); });
    NaviMapParameters other_grid;
    other_grid.hit_probability = .8;
    unchanged(root, [&] {
      errorIs(MapErrorCode::IncompatibleConfig,
              [&] { MapDatabase db(valid.string(), "resume", map_config_identity(PoseGraphParameters{}, other_grid)); });
    });
    int malformed_id = 0;
    for (const auto &query : {"DELETE FROM MapState;", "UPDATE MapState SET graph_revision=-1;", "UPDATE MapState SET graph_revision='bad';",
                              "UPDATE MapState SET map_uuid='bad';", "UPDATE MapState SET config_identity='bad';"}) {
      const auto malformed = root / ("malformed-" + std::to_string(malformed_id++) + ".db");
      fs::copy_file(valid, malformed);
      mutate(malformed, query);
      unchanged(root, [&] { errorIs(MapErrorCode::MalformedMetadata, [&] { MapDatabase db(malformed.string(), "resume"); }); });
    }
    std::cout << "PASS mode, version, configuration, metadata and byte-for-byte non-mutation matrix\n";

    fs::permissions(valid, fs::perms::owner_read, fs::perm_options::replace);
    fs::permissions(root, fs::perms::owner_read | fs::perms::owner_exec, fs::perm_options::replace);
    unchanged(root, [&] {
      MapDatabase eligible(valid.string(), "resume");
      check(eligible.mapUuid() == uuid, "read-only metadata");
    });
    fs::permissions(root, fs::perms::owner_all, fs::perm_options::replace);
    fs::permissions(valid, fs::perms::owner_read | fs::perms::owner_write, fs::perm_options::replace);
    std::ofstream(valid.string() + "-wal") << "unrecovered journal fixture";
    unchanged(root, [&] { errorIs(MapErrorCode::WriterConflict, [&] { MapDatabase db(valid.string(), "resume"); }); });
    fs::remove(valid.string() + "-wal");
    std::cout << "PASS read-only directory and non-mutating journal rejection\n";

    for (const auto &change :
         {"UPDATE MapState SET graph_revision=graph_revision+1;", "UPDATE MapState SET map_uuid='01234567-89ab-4cde-8fab-0123456789ab';"}) {
      const auto stale = root / ("stale-" + std::to_string(malformed_id++) + ".db");
      auto writer = std::make_unique<MapDatabase>(stale.string());
      writer->saveSubmap(first, LocalGrid{});
      sqlite3 *external = nullptr;
      check(sqlite3_open(stale.c_str(), &external) == SQLITE_OK, "stale fixture connection");
      sql(external, change);
      const auto before = contents(external);
      errorIs(MapErrorCode::StaleMap, [&] { writer->saveSubmap(second, LocalGrid{}); });
      errorIs(MapErrorCode::StaleMap, [&] { writer->saveSubmapPose(1, Eigen::Isometry3f::Identity()); });
      check(contents(external) == before, "stale writer changed committed records");
      writer.reset();
      check(contents(external) == before, "stale writer destruction changed committed records");
      sqlite3_close(external);
    }
    {
      const auto path = root / "replaced.db";
      MapDatabase writer(path.string());
      const auto renamed = root / "original-inode.db";
      fs::rename(path, renamed);
      std::ofstream(path) << "replacement must not be touched";
      const auto before = bytes(path);
      errorIs(MapErrorCode::StaleMap, [&] { writer.saveSubmap(first, LocalGrid{}); });
      check(bytes(path) == before, "replacement path was mutated");
      fs::remove(path);
      fs::rename(renamed, path);
    }
    std::cout << "PASS stale revision, UUID, and path identity\n";

    // Two independent processes start at a pipe barrier; exactly one can create the destination.
    const auto race = root / "race.db";
    int gate[2];
    check(pipe(gate) == 0, "create race barrier");
    pid_t children[2];
    for (int i = 0; i < 2; ++i) {
      children[i] = fork();
      check(children[i] >= 0, "fork create racer");
      if (children[i] == 0) {
        close(gate[1]);
        char token;
        if (read(gate[0], &token, 1) != 1) _exit(90);
        try {
          create(race);
          _exit(0);
        } catch (const MapError &e) {
          _exit(e.code() == MapErrorCode::DestinationExists ? 10 : 91);
        } catch (...) {
          _exit(92);
        }
      }
    }
    close(gate[0]);
    check(write(gate[1], "go", 2) == 2, "release create racers");
    close(gate[1]);
    int winners = 0, collisions = 0;
    for (pid_t child : children) {
      int status = 0;
      check(waitpid(child, &status, 0) == child && WIFEXITED(status), "join create racer");
      winners += WEXITSTATUS(status) == 0;
      collisions += WEXITSTATUS(status) == 10;
    }
    check(winners == 1 && collisions == 1, "exclusive creation must have exactly one winner");
    unchanged(root, [&] {
      MapDatabase eligible(race.string(), "resume");
      check(eligible.graphRevision() == 0, "race winner is intact");
    });
    std::cout << "PASS cross-process exclusive-create collision\n";

    PoseGraphParameters graph;
    graph.map_mode = "resume";
    const auto thread_count = [] { return std::distance(fs::directory_iterator("/proc/self/task"), fs::directory_iterator{}); };
    const auto before_threads = thread_count();
    unchanged(root, [&] {
      PoseGraphBackend backend(graph, {}, valid.string(), {});
      check(backend.historical() && !backend.hasActiveCorrection(), "read-only historical backend");
    });
    check(thread_count() == before_threads, "resume eligibility started a backend worker");
    graph.map_mode = "localize";
    unchanged(root, [&] { errorIs(MapErrorCode::UnsupportedMode, [&] { PoseGraphBackend backend(graph, {}, valid.string(), {}); }); });
    graph.map_mode = "new";
    unchanged(root, [&] { errorIs(MapErrorCode::DestinationExists, [&] { PoseGraphBackend backend(graph, {}, valid.string(), {}); }); });
    SapphireParameters parameters;
    parameters.general.save_path = (root / "must-not-create").string();
    parameters.general.save_map = 1;
    parameters.pose_graph.map_mode = "resume";
    parameters.pose_graph.database_path = valid.string();
    unchanged(root, [&] { errorIs(MapErrorCode::ResumeUnavailable, [&] { SlamPipeline pipeline(parameters); }); });
    parameters.pose_graph.map_mode = "localize";
    unchanged(root, [&] { errorIs(MapErrorCode::UnsupportedMode, [&] { SlamPipeline pipeline(parameters); }); });
    parameters.pose_graph.map_mode = "new";
    unchanged(root, [&] { errorIs(MapErrorCode::DestinationExists, [&] { SlamPipeline pipeline(parameters); }); });
    std::cout << "PASS historical backend and pipeline continuation guards without output directories or continuation\n";

    // The former output-directory helper recursively removed an existing map before safe open.
    // Populate the next minute's timestamp directories so a wall-clock second boundary is harmless.
    const auto output = root / "output";
    const auto now = std::time(nullptr);
    for (int offset = 0; offset < 60; ++offset) {
      const auto time = now + offset;
      std::tm local{};
      localtime_r(&time, &local);
      std::ostringstream name;
      name << std::put_time(&local, "%Y-%m-%d_%H-%M-%S");
      fs::create_directories(output / name.str());
      std::ofstream(output / name.str() / "map.db") << "existing map must survive";
    }
    SapphireParameters output_parameters;
    output_parameters.general.save_path = output.string();
    output_parameters.general.save_map = 1;
    output_parameters.pose_graph.enabled = false;  // Exercise directory handling without a GPU/backend.
    unchanged(output, [&] {
      SlamPipeline pipeline(output_parameters);
      pipeline.shutdown();
    });
    std::cout << "PASS existing timestamp output directories are never cleared\n";

    const auto config = root / "mode.toml";
    std::ofstream(config) << "[map]\nmode = 'resume'\ndatabase_path = '" << valid.string() << "'\n";
    const auto loaded = load_parameters(config);
    check(loaded.pose_graph.map_mode == "resume" && loaded.pose_graph.database_path == valid.string(), "load map entry options");
    auto scheduling = loaded.pose_graph;
    scheduling.update_period_sec *= 2;
    scheduling.storage.cloud_cache_mb *= 2;
    check(map_config_identity(scheduling, loaded.navi_map) == map_config_identity(loaded.pose_graph, loaded.navi_map),
          "scheduling/cache budgets do not change map compatibility identity");
    std::cout << "PASS configuration entry and compatibility identity\n";
    fs::remove_all(root);
    std::cout << "A1 map open tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << " (fixtures retained at " << root << ")\n";
    return 1;
  }
}
