#include "backend/storage/map_database.hpp"

#include <set>
#include <map>
#include <tuple>
#include "backend/graph/loop_policy.hpp"

#include "backend/visual/feature/scene_features.hpp"
#include "backend/visual/visual_loop.hpp"
#include "backend/registration/revisit_coverage.hpp"

namespace sapphire {
namespace {
void requireHistory(bool valid, const std::string &message) {
  if (!valid) throw MapError(MapErrorCode::HistoricalData, message);
}
void requireSemantics(bool valid, const std::string &message) {
  if (!valid) throw MapError(MapErrorCode::UnsupportedSemantics, message);
}
void validateSceneGeometry(const mapping::scene::FeatureMap &scene, const mapping::scene::FeatureMap &expected) {
  requireHistory(scene.points().size() == expected.points().size(), "Visual geometry membership mismatch");
  for (std::size_t i = 0; i < scene.points().size(); ++i) {
    const auto &a = scene.points()[i]; const auto &b = expected.points()[i];
    requireHistory(a.has_position == b.has_position, "Scene/ImageRecord metric presence mismatch");
    if (a.has_position) {
      const Eigen::Vector3d x(a.position.x, a.position.y, a.position.z), y(b.position.x, b.position.y, b.position.z);
      requireHistory((x - y).cwiseAbs().maxCoeff() <= 1e-5 * (1 + y.cwiseAbs().maxCoeff()),
                     "Scene/ImageRecord anchor geometry mismatch");
    }
  }
}
void done(sqlite3 *db, int rc) {
  if (rc != SQLITE_DONE) throw MapError(MapErrorCode::Storage, sqlite3_errmsg(db));
}
Eigen::Isometry3f pose(sqlite3_stmt *s, int column) {
  requireHistory(sqlite3_column_type(s, column) == SQLITE_BLOB && sqlite3_column_bytes(s, column) == 16 * sizeof(float),
                 "Invalid historical transform size");
  Eigen::Isometry3f value;
  std::memcpy(value.data(), sqlite3_column_blob(s, column), 16 * sizeof(float));
  const Eigen::Matrix3d r = value.rotation().cast<double>();
  requireHistory(value.matrix().allFinite() && value.matrix().row(3) == Eigen::RowVector4f(0, 0, 0, 1) &&
                     (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() <= 1e-6 && std::abs(r.determinant() - 1) <= 1e-6,
                 "Malformed historical rigid transform");
  return value;
}
template <class A, class B>
bool floatEquivalent(const A &a, const B &b) {
  for (Eigen::Index i = 0; i < a.size(); ++i)
    if (std::abs(double(a(i)) - double(b(i))) >
        4 * std::numeric_limits<float>::epsilon() * std::max({1., std::abs(double(a(i))), std::abs(double(b(i)))}) + 1e-12)
      return false;
  return true;
}
// Every edge points within the contiguous SQL identity domain. Induction over Q
// proves connectivity: its predecessor edge or root attachment reaches an older Node.
std::vector<int> validateTopology(const std::vector<Eigen::Isometry3d> &anchors, const std::vector<GraphLink> &links) {
  std::set<int> odometry, type1;
  for (const auto &link : links) {
    requireHistory(link.from_id > 0 && link.to_id > 0 && std::size_t(link.from_id) <= anchors.size() && std::size_t(link.to_id) <= anchors.size(),
                   "Dangling Link endpoint");
    requireSemantics(link.type == 0 || link.type == 1, "Unsupported historical Link type");
    if (link.type == 0) {
      requireHistory(link.to_id == sqlite3_int64(link.from_id) + 1 && odometry.insert(link.to_id).second, "Invalid/duplicate odometry edge");
      const Eigen::Matrix4d expected = (anchors[link.from_id - 1].inverse() * anchors[link.to_id - 1]).matrix();
      requireHistory(floatEquivalent(expected, link.transform.matrix()), "Stored odometry Link differs from original anchors");
    } else {
      requireHistory(link.from_id > link.to_id && type1.insert(link.from_id).second, "Invalid type-1 direction or duplicate query");
    }
  }
  std::vector<int> roots(anchors.size(), 1);
  for (std::size_t i = 1; i < anchors.size(); ++i) {
    const int q = static_cast<int>(i + 1);
    if (odometry.count(q))
      roots[i] = roots[i - 1];
    else {
      requireHistory(type1.count(q) == 1, "Missing predecessor or root attachment");
      roots[i] = q;
    }
  }
  for (const auto &link : links)
    if (link.type == 1 && odometry.count(link.from_id))
      requireHistory(loop_policy::eligible(link.from_id - 1, link.to_id - 1, roots[link.from_id - 1] - 1,
                                            roots[link.to_id - 1] - 1, anchors.size()),
                     "Ordinary type-1 edge violates temporal separation");
  return roots;
}
// Shared A2/W representability check only: no occupancy fusion or raster allocation.
// Bounds are min-x, min-y, max-x, max-y; infinities denote no cell evidence yet.
void includeOccupancyEvidence(const LocalGrid &grid, const Eigen::Isometry3f &map_pose, float resolution, Eigen::Vector4d &bounds) {
  requireHistory(std::isfinite(resolution) && resolution > 0, "Invalid occupancy resolution");
  for (const auto *points : {&grid.groundCells, &grid.obstacleCells, &grid.emptyCells})
    for (const auto &point : *points) {
      requireHistory(point.allFinite(), "Nonfinite LocalGrid evidence");
      const Eigen::Vector3f world = map_pose * Eigen::Vector3f(point.x(), point.y(), 0);
      // Keep A2's float transform/division and floor semantics. Check in double
      // before any integer narrowing, including float rounding at INT_MAX.
      const double x = std::floor(world.x() / resolution), y = std::floor(world.y() / resolution);
      requireHistory(std::isfinite(x) && std::isfinite(y) && x >= std::numeric_limits<int>::min() && y >= std::numeric_limits<int>::min() &&
                         x <= std::numeric_limits<int>::max() && y <= std::numeric_limits<int>::max(),
                     "Unrepresentable occupancy cell");
      bounds[0] = std::min(bounds[0], x);
      bounds[1] = std::min(bounds[1], y);
      bounds[2] = std::max(bounds[2], x);
      bounds[3] = std::max(bounds[3], y);
    }
}
void validateOccupancyExtent(const Eigen::Vector4d &bounds) {
  if (!std::isfinite(bounds[0])) return;  // Empty evidence preserves A2 semantics.
  // Checked coordinates are integral doubles in the signed-int domain. Their
  // differences are exact here, even if they would overflow signed int.
  const double width = bounds[2] - bounds[0] + 1, height = bounds[3] - bounds[1] + 1;
  requireHistory(width >= 1 && height >= 1 && width <= std::numeric_limits<int>::max() && height <= std::numeric_limits<int>::max(),
                 "Unrepresentable occupancy dimensions");
  requireHistory(static_cast<std::size_t>(width) <= std::vector<int8_t>().max_size() / static_cast<std::size_t>(height),
                 "Unrepresentable occupancy dimensions");
}
double scalar(const void *bytes, std::size_t i) {
  double value;
  std::memcpy(&value, static_cast<const char *>(bytes) + i * sizeof(double), sizeof(double));
  return value;
}
}  // namespace

void database_detail::validatePyramidPayload(const void *data, std::size_t size, std::size_t source_point_count) {
  // PYRVOX04's current POD header, followed by per-level bucket count and sparse records.
  // Preflight the entire payload without allocating buckets or sparse-record vectors.
  constexpr std::size_t level_offset = 8 + sizeof(std::uint32_t) + sizeof(float);
  constexpr std::size_t header_size = level_offset + 2 * sizeof(int);
  constexpr std::size_t record_size = 4 * sizeof(std::uint32_t);
  requireHistory(data && size >= header_size, "Truncated pyramid header");
  const auto *bytes = static_cast<const char *>(data);
  requireHistory(size <= std::size_t(std::numeric_limits<int>::max()) && size <= std::size_t(std::numeric_limits<std::streamsize>::max()),
                 "Unrepresentable pyramid payload length");
  std::uint32_t version;
  float resolution;
  int max_level, scans;
  std::memcpy(&version, bytes + 8, sizeof(version));
  std::memcpy(&resolution, bytes + 8 + sizeof(version), sizeof(resolution));
  std::memcpy(&max_level, bytes + level_offset, sizeof(int));
  std::memcpy(&scans, bytes + level_offset + sizeof(int), sizeof(int));
  requireHistory(std::memcmp(bytes, "PYRVOX04", 8) == 0 && version == 4 && std::isfinite(resolution) && resolution > 0 && scans > 0,
                 "Invalid pyramid header");
  requireHistory(max_level >= 0 && max_level <= 32, "Invalid pyramid level count");
  // The real builder inserts at most eight voxel coordinates per source Gaussian,
  // then grows to at most 16 * that INPUT population. Collision-limited output
  // may retain far fewer entries; stored sparse occupancy is not an allocation bound.
  requireHistory(source_point_count <= std::numeric_limits<std::size_t>::max() / 8 / 16, "Pyramid source population overflow");
  const auto bucket_limit = std::max(std::size_t(1), source_point_count * 8 * 16);
  std::size_t allocation_bytes = 0;
  std::size_t cursor = header_size;
  for (int level = 0; level <= max_level; ++level) {
    requireHistory(size - cursor >= 2 * sizeof(std::size_t), "Truncated pyramid counts");
    std::size_t buckets, occupied;
    std::memcpy(&buckets, bytes + cursor, sizeof(buckets));
    std::memcpy(&occupied, bytes + cursor + sizeof(buckets), sizeof(occupied));
    cursor += 2 * sizeof(std::size_t);
    requireHistory(occupied <= (size - cursor) / record_size, "Truncated pyramid sparse records");
    requireHistory(buckets > 0 && occupied <= buckets && buckets <= bucket_limit && buckets - 1 <= std::numeric_limits<std::uint32_t>::max() &&
                       buckets <= cpu::VoxelBuckets().max_size() &&
                       buckets <= (std::numeric_limits<std::size_t>::max() - allocation_bytes) / sizeof(Eigen::Vector4i),
                   "Invalid pyramid bucket allocation");
    allocation_bytes += buckets * sizeof(Eigen::Vector4i);
    requireHistory(occupied <= (std::numeric_limits<std::size_t>::max() - allocation_bytes) / record_size, "Pyramid sparse allocation overflow");
    std::uint32_t previous = 0;
    for (std::size_t i = 0; i < occupied; ++i) {
      std::uint32_t index;
      std::memcpy(&index, bytes + cursor + i * record_size, sizeof(index));
      requireHistory(index < buckets && (i == 0 || index > previous), "Invalid pyramid sparse index/order");
      previous = index;
    }
    cursor += occupied * record_size;
  }
  requireHistory(cursor == size, "Trailing pyramid bytes");
}

Eigen::Isometry3d MapDatabase::loadOriginalAnchor(int id) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  requireReadable();
  database_detail::Statement query(db_, "SELECT odom_pose_count,odom_poses FROM Node WHERE id=?;");
  sqlite3_bind_int(query.get(), 1, id);
  const int status = sqlite3_step(query.get());
  if (status != SQLITE_ROW && status != SQLITE_DONE) throw MapError(MapErrorCode::Storage, sqlite3_errmsg(db_));
  requireHistory(status == SQLITE_ROW, "Missing original query anchor");
  const auto count = database_detail::readCount(query.get(), 0, 1, 7 * sizeof(double));
  requireHistory(count > 0, "Empty original query odometry");
  const auto *data = sqlite3_column_blob(query.get(), 1);
  Eigen::Vector3d t(scalar(data, 0), scalar(data, 1), scalar(data, 2));
  Eigen::Quaterniond q(scalar(data, 6), scalar(data, 3), scalar(data, 4), scalar(data, 5));
  requireHistory(t.allFinite() && q.coeffs().allFinite() && std::abs(q.norm() - 1) <= 1e-12,
                 "Invalid original query anchor");
  Eigen::Isometry3d anchor = Eigen::Isometry3d::Identity();
  anchor.translation() = t; anchor.linear() = q.toRotationMatrix();
  return anchor;
}

void MapDatabase::visitHistoricalNodes(const NodeVisitor &visitor) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  requireReadable();
  database_detail::Statement query(db_,
                                   "SELECT n.id,n.stamp,n.odom_pose_count,n.odom_timestamps,n.odom_poses,n.submap_pose,"
                                   "s.min_x,s.min_y,s.min_z,s.max_x,s.max_y,s.max_z,"
                                   "m.tag_rows,m.tag_cols,m.tag_cv_type,m.tag FROM Node n "
                                   "LEFT JOIN SpatialRecord s ON s.node_id=n.id "
                                   "LEFT JOIN MetaTag m ON m.node_id=n.id AND m.tag_type=2 ORDER BY n.id;");
  auto *s = query.get();
  sqlite3_int64 expected = 1;
  int rc;
  while ((rc = sqlite3_step(s)) == SQLITE_ROW) {
    const auto id = database_detail::readInteger(s, 0, 1, std::numeric_limits<int>::max());
    const auto count = database_detail::readCount(s, 2, 4, 7 * sizeof(double));
    (void)database_detail::readCount(s, 2, 3, sizeof(double));
    requireHistory(id == expected++ && id <= std::numeric_limits<int>::max(), "Historical Node IDs must be contiguous from 1");
    requireHistory(count > 0, "Missing or malformed original odometry for Node " + std::to_string(id));
    const auto *times = sqlite3_column_blob(s, 3), *poses = sqlite3_column_blob(s, 4);
    const double stamp = database_detail::readReal(s, 1);
    requireHistory(std::isfinite(stamp) && std::abs(stamp - scalar(times, 0)) <= 1e-9, "Node stamp differs from original anchor timestamp");
    Eigen::Isometry3d anchor = Eigen::Isometry3d::Identity();
    double previous = -std::numeric_limits<double>::infinity();
    for (sqlite3_int64 j = 0; j < count; ++j) {
      const double time = scalar(times, j);
      Eigen::Vector3d t(scalar(poses, 7 * j), scalar(poses, 7 * j + 1), scalar(poses, 7 * j + 2));
      Eigen::Quaterniond q(scalar(poses, 7 * j + 6), scalar(poses, 7 * j + 3), scalar(poses, 7 * j + 4), scalar(poses, 7 * j + 5));
      requireHistory(std::isfinite(time) && time >= previous && t.allFinite() && q.coeffs().allFinite() && std::abs(q.norm() - 1) <= 1e-12,
                     "Invalid original odometry sample");
      previous = time;
      if (!j) {
        anchor.translation() = t;
        anchor.linear() = q.toRotationMatrix();
      }
    }
    const auto committed = pose(s, 5);
    const AABB bounds({database_detail::readFloat(s, 6), database_detail::readFloat(s, 7), database_detail::readFloat(s, 8)},
                      {database_detail::readFloat(s, 9), database_detail::readFloat(s, 10), database_detail::readFloat(s, 11)});
    requireHistory(bounds.valid() && bounds.transform(committed).valid(), "Invalid historical spatial bounds");
    requireHistory(database_detail::readInteger(s, 12) == 4 && database_detail::readInteger(s, 13) == 4 &&
                       database_detail::readInteger(s, 14) == CV_32FC1 && database_detail::blobBytes(s, 15) == 16 * sizeof(float),
                   "Missing or malformed committed pose MetaTag");
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> tag;
    std::memcpy(tag.data(), sqlite3_column_blob(s, 15), 16 * sizeof(float));
    requireHistory((tag.array() == committed.matrix().array()).all(), "Pose MetaTag disagrees with committed Node pose");
    visitor(static_cast<int>(id), anchor, committed, bounds);
  }
  done(db_, rc);
}

std::vector<GraphLink> MapDatabase::loadGraphLinks() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  requireReadable();
  database_detail::Statement query(db_, "SELECT from_id,to_id,type,transform FROM Link ORDER BY type,from_id,to_id;");
  std::vector<GraphLink> links;
  int rc;
  while ((rc = sqlite3_step(query.get())) == SQLITE_ROW) {
    auto *s = query.get();
    const auto from = database_detail::readInteger(s, 0, 1, std::numeric_limits<int>::max());
    const auto to = database_detail::readInteger(s, 1, 1, std::numeric_limits<int>::max());
    const auto type = database_detail::readInteger(s, 2, std::numeric_limits<sqlite3_int64>::min());
    requireHistory(from > 0 && to > 0 && from <= std::numeric_limits<int>::max() && to <= std::numeric_limits<int>::max(), "Invalid Link endpoints");
    requireSemantics(type == 0 || type == 1, "Unsupported historical Link type");
    links.push_back({int(from), int(to), int(type), pose(s, 3)});
  }
  done(db_, rc);
  return links;
}

void MapDatabase::validateHistoricalRecords(float grid_resolution) const {
  try {
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      requireReadable();
      database_detail::Statement integrity(db_, "PRAGMA quick_check;");
      requireHistory(
          sqlite3_step(integrity.get()) == SQLITE_ROW && std::string(reinterpret_cast<const char *>(sqlite3_column_text(integrity.get(), 0))) == "ok",
          "SQLite integrity check failed");
      database_detail::Statement foreign(db_, "PRAGMA foreign_key_check;");
      requireHistory(sqlite3_step(foreign.get()) == SQLITE_DONE, "Orphan historical payload");
      if (retainsScenes()) {
        database_detail::Statement missing(db_, "SELECT n.id FROM Node n LEFT JOIN SceneState s ON s.node_id=n.id WHERE s.node_id IS NULL LIMIT 1;");
        requireHistory(sqlite3_step(missing.get()) == SQLITE_DONE, "Missing scene retention state");
        database_detail::Statement states(db_, "SELECT node_id,retired_by,retired_revision FROM SceneState ORDER BY node_id;");
        int rc;
        while ((rc=sqlite3_step(states.get())) == SQLITE_ROW) {
          const int id=database_detail::readInteger(states.get(),0,1,std::numeric_limits<int>::max());
          if (sqlite3_column_type(states.get(),1)==SQLITE_NULL) {
            requireHistory(sqlite3_column_type(states.get(),2)==SQLITE_NULL,"Partial scene retirement state");
            continue;
          }
          const int next=database_detail::readInteger(states.get(),1,sqlite3_int64(id)+1,std::numeric_limits<int>::max());
          (void)database_detail::readInteger(states.get(),2,1,expected_revision_);
          database_detail::Statement link(db_, "SELECT 1 FROM Link WHERE from_id=? AND to_id=? AND type=1;");
          sqlite3_bind_int(link.get(),1,next); sqlite3_bind_int(link.get(),2,id);
          requireHistory(sqlite3_step(link.get())==SQLITE_ROW,"Retired scene lacks its committed association");
          for (const char *table : {"LaserRecord","PyramidVoxel","ImageRecord","VisualScene"}) {
            database_detail::Statement payload(db_,("SELECT 1 FROM "+std::string(table)+" WHERE node_id=? LIMIT 1;").c_str());
            sqlite3_bind_int(payload.get(),1,id);
            requireHistory(sqlite3_step(payload.get())==SQLITE_DONE,"Retired scene still has heavy payload");
          }
        }
        done(db_,rc);
      }
      for (const char *table : {"SpatialRecord", "NaviTrajectory", "LaserRecord", "PyramidVoxel", "FlatGrid"}) {
        if (!archivesNavigation() && (std::strcmp(table, "FlatGrid") == 0 || std::strcmp(table, "NaviTrajectory") == 0)) continue;
        const std::string sql = "SELECT n.id FROM Node n LEFT JOIN " + std::string(table) + " p ON p.node_id=n.id WHERE p.node_id IS NULL" +
            (retainsScenes() && (std::strcmp(table,"LaserRecord")==0 || std::strcmp(table,"PyramidVoxel")==0) ?
             " AND NOT EXISTS(SELECT 1 FROM SceneState s WHERE s.node_id=n.id AND s.retired_by IS NOT NULL)" : "") + " LIMIT 1;";
        database_detail::Statement missing(db_, sql.c_str());
        requireHistory(sqlite3_step(missing.get()) == SQLITE_DONE, "Missing required " + std::string(table) + " record");
      }
      for (const char *table :
           {"SpatialRecord", "NaviTrajectory", "LaserRecord", "PyramidVoxel", "FlatGrid", "ImageRecord", "VisualScene", "MetaTag"}) {
        if (!archivesNavigation() && (std::strcmp(table, "FlatGrid") == 0 || std::strcmp(table, "NaviTrajectory") == 0)) continue;
        const std::string sql = "SELECT node_id FROM " + std::string(table) + ";";
        database_detail::Statement ids(db_, sql.c_str());
        int rc;
        while ((rc = sqlite3_step(ids.get())) == SQLITE_ROW) (void)database_detail::readInteger(ids.get(), 0, 1, std::numeric_limits<int>::max());
        done(db_, rc);
      }
    }
    std::vector<Eigen::Isometry3d> anchors;
    std::vector<Eigen::Isometry3f> committed;
    std::vector<AABB> bounds;
    visitHistoricalNodes([&](int, const auto &anchor, const auto &map_pose, const auto &box) {
      anchors.push_back(anchor);
      committed.push_back(map_pose);
      bounds.push_back(box);
    });
    const auto links = loadGraphLinks();
    const auto revision = graphRevision();
    requireHistory(anchors.empty() ? (links.empty() && revision == 0) : revision > 0, "Uncommitted historical graph");
    (void)validateTopology(anchors, links);
    if (archivesNavigation()) requireHistory(std::isfinite(grid_resolution) && grid_resolution > 0, "Invalid occupancy resolution");
    const double inf = std::numeric_limits<double>::infinity();
    Eigen::Vector4d occupancy_bounds(inf, inf, -inf, -inf);
    // Validate every payload, even when its runtime consumer is disabled.
    for (std::size_t index = 0; index < anchors.size(); ++index) {
      const int id = int(index + 1);
      if (!sceneActive(id)) continue; // Graph/anchor/bounds/topology checks still include this key.
      const auto cloud = loadCloud(id);
      requireHistory(cloud && !cloud->empty(), "Empty required historical Gaussian cloud");
      const AABB actual(*cloud);
      requireHistory(floatEquivalent(actual.minimum, bounds[index].minimum) && floatEquivalent(actual.maximum, bounds[index].maximum),
                     "Spatial bounds disagree with persisted cloud");
      (void)loadPyramidVoxel(id);
      const auto path = loadNavigation(id);
      double previous = -std::numeric_limits<double>::infinity();
      for (const auto &sample : path.samples) {
        requireHistory(std::isfinite(sample.timestamp) && sample.timestamp >= previous && std::isfinite(sample.x) && std::isfinite(sample.y) &&
                           std::isfinite(sample.z) && std::isfinite(sample.distance) && std::isfinite(sample.qx) && std::isfinite(sample.qy) &&
                           std::isfinite(sample.qz) && std::isfinite(sample.qw),
                       "Nonfinite/unordered navigation samples");
        previous = sample.timestamp;
      }
      for (const auto &p : path.spline.control_points) requireHistory(p.allFinite(), "Nonfinite navigation control point");
      for (float knot : path.spline.knots) requireHistory(std::isfinite(knot), "Nonfinite navigation knot");
      if (archivesNavigation()) {
        LocalGrid grid;
        requireHistory(loadLocalGrid(id, grid) && std::isfinite(grid.cellSize) && grid.cellSize > 0 && grid.viewPoint.allFinite(),
                       "Invalid historical LocalGrid metadata");
        includeOccupancyEvidence(grid, committed[index], grid_resolution, occupancy_bounds);
      }
      const auto frames = loadVisualFrames(id);
      std::set<std::array<std::uint8_t, 32>> unique;
      std::vector<std::array<std::uint8_t, 32>> descriptors;
      for (const auto &frame : frames) {
        const auto &matrix = frame.descriptors();
        requireSemantics(matrix.empty() ? frame.points().empty() : matrix.type() == CV_8UC1 && matrix.cols == 32,
                         "Unsupported historical appearance descriptor");
        for (const auto &point : frame.points())
          requireHistory(point.pixel().allFinite() && std::isfinite(point.response()) && point.level() >= 0, "Invalid historical visual point");
        for (int row = 0; row < matrix.rows && descriptors.size() < mapping::scene::point_capacity; ++row) {
          std::array<std::uint8_t, 32> descriptor;
          std::memcpy(descriptor.data(), matrix.ptr(row), 32);
          if (unique.insert(descriptor).second) descriptors.push_back(descriptor);
        }
      }
      const auto bytes = loadVisualScene(id);
      if (!bytes.empty()) {
        const auto scene = mapping::scene::FeatureMap::decode(bytes.data(), bytes.size());
        requireSemantics(scene->vocabulary_hash() == 0 && scene->max_points() == mapping::scene::point_capacity,
                         "Unsupported historical scene identity");
        requireHistory(scene->points().size() == descriptors.size(), "Scene/ImageRecord descriptor membership mismatch");
        validateSceneGeometry(*scene, *buildVisualScene(frames, anchors[index]));
        for (std::size_t i = 0; i < descriptors.size(); ++i) {
          const auto &point = scene->points()[i];
          requireSemantics(point.appearance_count == 1 && point.appearances[0].word_id == -1 &&
                               point.appearances[0].tree_node == features::FeatureBlock::invalid_node_id,
                           "Unsupported historical scene semantics");
          requireHistory(point.id == i + 1 && point.appearances[0].descriptor == descriptors[i], "Scene/ImageRecord descriptor identity mismatch");
        }
      }
    }
    validateOccupancyExtent(occupancy_bounds);
    for (const auto &[id, tag] : loadMetaTags()) {
      requireHistory(id < anchors.size(), "Orphan MetaTag");
      requireHistory(cv::checkRange(tag.descriptor()), "Nonfinite MetaTag");
    }
    validateReadIdentity();
  } catch (const MapError &) {
    throw;
  } catch (const std::bad_alloc &) {
    throw MapError(MapErrorCode::Storage, "Insufficient memory for historical records");
  } catch (const std::exception &e) {
    throw MapError(MapErrorCode::HistoricalData, e.what());
  }
}
namespace {
// Exactly one gate/hook domain in Sapphire core. Supported startup has no foreign
// Unix-VFS users; admission excludes every other MapDatabase during installation.
std::mutex admission_mutex;
std::size_t raw_owner_slots = 0;
bool writer_exclusive = false;
std::once_flag observer_once;
using FstatCall = int (*)(int, struct stat *);
FstatCall original_fstat = nullptr;
sqlite3_vfs *observed_vfs = nullptr;
bool observer_installed = false;
struct FstatObservation {
  unsigned calls = 0;
  int rc = -1;
  struct stat identity {};
};
thread_local FstatObservation *active_observation = nullptr;
int observeFstat(int fd, struct stat *info) noexcept {
  const int rc = original_fstat(fd, info);
  const int error = errno;
  if (active_observation) {
    if (active_observation->calls < 2) ++active_observation->calls;
    active_observation->rc = rc;
    if (rc == 0) active_observation->identity = *info;
  }
  errno = error;
  return rc;
}
void verifyObserver() {
  auto *vfs = sqlite3_vfs_find("unix");
  if (!observer_installed || vfs != observed_vfs || !vfs || vfs->iVersion < 3 || !vfs->xGetSystemCall || !vfs->xSetSystemCall ||
      vfs->xGetSystemCall(vfs, "fstat") != reinterpret_cast<sqlite3_syscall_ptr>(observeFstat))
    throw MapError(MapErrorCode::UnsupportedMode, "Writable maps require the installed stock-unix fstat observer");
}
void installObserver() {
  std::call_once(observer_once, [] {
#if defined(__linux__) && defined(__x86_64__)
    static_assert(sizeof(off_t) == 8, "W requires the reviewed 64-bit Linux syscall ABI");
    auto *vfs = sqlite3_vfs_find("unix");
    if (!vfs || !vfs->zName || std::strcmp(vfs->zName, "unix") != 0 || vfs->iVersion < 3 || !vfs->xGetSystemCall || !vfs->xSetSystemCall ||
        !sqlite3_threadsafe())
      return;
    const auto callback = vfs->xGetSystemCall(vfs, "fstat");
    if (!callback || callback == reinterpret_cast<sqlite3_syscall_ptr>(observeFstat)) return;
    original_fstat = reinterpret_cast<FstatCall>(callback);
    observed_vfs = vfs;
    observer_installed = vfs->xSetSystemCall(vfs, "fstat", reinterpret_cast<sqlite3_syscall_ptr>(observeFstat)) == SQLITE_OK &&
                         vfs->xGetSystemCall(vfs, "fstat") == reinterpret_cast<sqlite3_syscall_ptr>(observeFstat);
#endif
  });
  verifyObserver();
}
bool sameInode(const struct stat &a, const struct stat &b) {
  return S_ISREG(a.st_mode) && S_ISREG(b.st_mode) && a.st_dev == b.st_dev && a.st_ino == b.st_ino;
}
}  // namespace

__attribute__((weak)) void database_detail::writableStorageTestPoint(const char *, void *) {}

void MapDatabase::reserveAdmission(bool writer) {
  std::lock_guard<std::mutex> lock(admission_mutex);
  if (writer_exclusive || (writer && raw_owner_slots != 0))
    throw MapError(MapErrorCode::WriterConflict, "MapDatabase descriptor admission conflicts with an existing owner");
  if (raw_owner_slots == std::numeric_limits<std::size_t>::max()) throw MapError(MapErrorCode::Storage, "Map owner slots exhausted");
  ++raw_owner_slots;
  writer_exclusive = writer;
  admission_ = writer ? Admission::ExclusiveSlot : Admission::SharedSlot;
}
void MapDatabase::releaseAdmission() noexcept {
  std::lock_guard<std::mutex> lock(admission_mutex);
  if (admission_ == Admission::None) return;
  if (!raw_owner_slots || db_ || file_fd_ >= 0 || mapped_data_ != MAP_FAILED) std::terminate();
  if (admission_ == Admission::ExclusiveSlot) {
    if (!writer_exclusive || raw_owner_slots != 1) std::terminate();
    writer_exclusive = false;
  } else if (writer_exclusive)
    std::terminate();
  --raw_owner_slots;
  admission_ = Admission::None;
}
struct stat MapDatabase::observeMainFile(sqlite3 *db) {
  verifyObserver();
  if (active_observation) throw MapError(MapErrorCode::Lifecycle, "Nested writable-file observation is unsupported");
  auto *mutex = sqlite3_db_mutex(db);
  if (!mutex) throw MapError(MapErrorCode::UnsupportedMode, "Writable maps require an effective SQLite connection mutex");
  sqlite3_mutex_enter(mutex);
  struct Unlock {
    sqlite3_mutex *mutex;
    ~Unlock() { sqlite3_mutex_leave(mutex); }
  } unlock{mutex};
  sqlite3_vfs *vfs = nullptr;
  sqlite3_file *file = nullptr;
  if (sqlite3_file_control(db, "main", SQLITE_FCNTL_VFS_POINTER, &vfs) != SQLITE_OK || vfs != observed_vfs ||
      sqlite3_file_control(db, "main", SQLITE_FCNTL_FILE_POINTER, &file) != SQLITE_OK || !file || !file->pMethods || file->pMethods->iVersion < 1 ||
      !file->pMethods->xFileSize)
    throw MapError(MapErrorCode::UnsupportedMode, "Cannot inspect actual SQLite main file");
  FstatObservation observation;
  sqlite3_int64 size = -1;
  int rc;
  {
    active_observation = &observation;
    struct Clear {
      ~Clear() { active_observation = nullptr; }
    } clear;
    rc = file->pMethods->xFileSize(file, &size);
  }
  verifyObserver();
  if (rc != SQLITE_OK || size < 0 || observation.calls != 1 || observation.rc != 0 || !S_ISREG(observation.identity.st_mode))
    throw MapError(MapErrorCode::Storage, "No unique successful main-file fstat observation");
  return observation.identity;
}
void MapDatabase::proveWritableIdentity() const {
  const auto actual = observeMainFile(db_);
  struct stat held {};
  if (::fstat(file_fd_, &held) != 0 || !sameInode(actual, held) || !sameInode(actual, file_identity_) || sqlite3_db_readonly(db_, "main") != 0)
    throw MapError(MapErrorCode::StaleMap, "SQLite is not a writable connection to the validated map inode");
  checkFileIdentity();
}
void MapDatabase::openWritableConnection() {
  installObserver();
  database_detail::writableStorageTestPoint("before-sqlite-open");
  if (sqlite3_open_v2(path_.c_str(), &db_, SQLITE_OPEN_READWRITE | SQLITE_OPEN_FULLMUTEX | SQLITE_OPEN_NOFOLLOW, "unix") != SQLITE_OK)
    throw MapError(MapErrorCode::Storage, db_ ? sqlite3_errmsg(db_) : "Cannot allocate SQLite writer");
  if (sqlite3_db_config(db_, SQLITE_DBCONFIG_NO_CKPT_ON_CLOSE, 1, nullptr) != SQLITE_OK)
    throw MapError(MapErrorCode::UnsupportedMode, "Cannot disable implicit checkpoint on writer close");
  database_detail::writableStorageTestPoint("before-file-proof");
  proveWritableIdentity();  // Before any application SQL or journal recovery.
}
void MapDatabase::setJournalMode(const char *mode) {
  const std::string sql = "PRAGMA journal_mode=" + std::string(mode) + ";";
  database_detail::Statement statement(db_, sql.c_str());
  if (sqlite3_step(statement.get()) != SQLITE_ROW || sqlite3_column_type(statement.get(), 0) != SQLITE_TEXT ||
      std::strcmp(reinterpret_cast<const char *>(sqlite3_column_text(statement.get(), 0)), mode) != 0 || sqlite3_step(statement.get()) != SQLITE_DONE)
    throw MapError(MapErrorCode::Storage, "SQLite journal-mode transition failed or remained busy");
}
void MapDatabase::closeHandlesChecked() {
  if (admission_ == Admission::None && !db_ && file_fd_ < 0 && mapped_data_ == MAP_FAILED) {
    state_ = State::Closed;
    return;
  }
  database_detail::writableStorageTestPoint("before-sqlite-close");
  if (db_) {
    const int rc = sqlite3_close(db_);  // No zombie close: BUSY keeps ownership/admission.
    if (rc != SQLITE_OK) {
      state_ = State::Failed;
      throw MapError(MapErrorCode::Lifecycle, "SQLite close incomplete; retain handle, owner fd and descriptor admission");
    }
    db_ = nullptr;
  }
  database_detail::writableStorageTestPoint("after-sqlite-close");
  if (mapped_data_ != MAP_FAILED) {
    if (::munmap(mapped_data_, mapped_size_) != 0) {
      state_ = State::Failed;
      throw MapError(MapErrorCode::Lifecycle, "Cannot unmap historical map");
    }
    mapped_data_ = MAP_FAILED;
    mapped_size_ = 0;
  }
  int close_error = 0;
  if (file_fd_ >= 0) {
    const int fd = file_fd_;
    file_fd_ = -1;  // Linux close must never be retried using a possibly reused fd number.
    if (::close(fd) != 0) close_error = errno;
  }
  database_detail::writableStorageTestPoint("after-owner-close");
  releaseAdmission();
  state_ = State::Closed;
  if (close_error) throw MapError(MapErrorCode::Lifecycle, "Raw map close failed: " + std::string(std::strerror(close_error)));
}
void MapDatabase::closeHandles() noexcept {
  try {
    closeHandlesChecked();
  } catch (...) {
    std::terminate();
  }
}
void MapDatabase::finish() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (state_ == State::Closed) return;
  try {
    if (state_ == State::Writable) {
      validateWriter();
      if (!sqlite3_get_autocommit(db_)) throw MapError(MapErrorCode::Lifecycle, "Cannot finish a writer with an active transaction");
      database_detail::writableStorageTestPoint("before-checkpoint");
      int frames = -1, checkpointed = -1;
      if (sqlite3_wal_checkpoint_v2(db_, "main", SQLITE_CHECKPOINT_TRUNCATE, &frames, &checkpointed) != SQLITE_OK ||
          (frames >= 0 && checkpointed != frames))
        throw MapError(MapErrorCode::Storage, "WAL checkpoint failed or remained busy");
      database_detail::writableStorageTestPoint("before-finish-journal");
      setJournalMode("delete");
    }
    closeHandlesChecked();
  } catch (...) {
    state_ = State::Failed;
    throw;
  }
}
MapDatabase::~MapDatabase() noexcept {
  try {
    finish();
  } catch (...) {
    closeHandles();
  }
}
void MapDatabase::promoteToWritable(float grid_resolution) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  requireReadable();
  if (state_ != State::HistoricalReadOnly) throw MapError(MapErrorCode::Lifecycle, "Only healthy historical storage can be promoted");
  // Validate the prepared clean view before taking away its read service.
  {
    std::lock_guard<std::mutex> admission_lock(admission_mutex);
    if (admission_ != Admission::SharedSlot || raw_owner_slots != 1 || writer_exclusive)
      throw MapError(MapErrorCode::WriterConflict, "Promotion requires the sole reserved/active map owner slot");
    writer_exclusive = true;
    admission_ = Admission::ExclusiveSlot;
  }
  state_ = State::Transitioning;
  try {
    database_detail::writableStorageTestPoint("exclusive-promotion");
    validateHistoricalRecords(grid_resolution);
    checkNoJournal();
    checkFileIdentity();
    if (sqlite3_close(db_) != SQLITE_OK) throw MapError(MapErrorCode::Lifecycle, "Historical SQLite close incomplete");
    db_ = nullptr;
    if (::munmap(mapped_data_, mapped_size_) != 0) throw MapError(MapErrorCode::Lifecycle, "Historical unmap failed");
    mapped_data_ = MAP_FAILED;
    mapped_size_ = 0;
    database_detail::writableStorageTestPoint("before-flock-conversion");
    if (::flock(file_fd_, LOCK_EX | LOCK_NB) != 0)
      throw MapError(MapErrorCode::WriterConflict, "Exclusive flock conversion failed; history is unusable");
    checkNoJournal();
    checkFileIdentity();
    openWritableConnection();
    checkNoJournal();  // Never recover a sidecar that appeared during clean handoff.
    writable_ = true;
    validateSchema();
    readIdentity(false);
    execute("PRAGMA foreign_keys=ON;");
    execute("PRAGMA synchronous=NORMAL;");
    execute("PRAGMA temp_store=MEMORY;");
    validateHistoricalRecords(grid_resolution);
    occupancy_resolution_ = grid_resolution;  // Retain the resolution used by the accepted handoff validation.
    first_finalized_commit_ = true;
    route_s_only_ = true;
    state_ = State::Writable;
  } catch (...) {
    state_ = State::Failed;
    // A BUSY close retains the live resources; only cleanup may be retried.
    const auto failure = std::current_exception();
    try {
      closeHandlesChecked();
    } catch (...) {
      throw;
    }
    state_ = State::Failed;
    std::rethrow_exception(failure);
  }
}

MapDatabase::MapDatabase(const std::string &path, const std::string &mode, std::string config_identity, float grid_resolution)
    : config_identity_(std::move(config_identity)), occupancy_resolution_(grid_resolution) {
  if (mode != "new" && mode != "resume" && mode != "recover") throw MapError(MapErrorCode::UnsupportedMode, "Unsupported map mode: " + mode);
  writable_ = mode != "resume";
  const bool recovery = mode == "recover";
  try {
    reserveAdmission(writable_);
    database_detail::writableStorageTestPoint("admission-reserved");
    if (path.empty()) throw MapError(MapErrorCode::Storage, "Empty map path");
    // Resolve the existing parent through the filesystem: lexical removal of
    // alias/.. changes symlink traversal and missing/.. must not become valid.
    // Keep the final component unresolved for exclusive creation / O_NOFOLLOW.
    const auto requested = std::filesystem::absolute(path);
    std::error_code resolution_error;
    const auto parent = std::filesystem::canonical(requested.parent_path(), resolution_error);
    if (resolution_error)
      throw MapError(!writable_ && resolution_error == std::errc::no_such_file_or_directory ? MapErrorCode::MissingMap : MapErrorCode::Storage,
                     "Cannot resolve map parent " + requested.parent_path().string() + ": " + resolution_error.message());
    path_ = (parent / requested.filename()).string();
    // Nonblocking open followed by fstat validates the opened object, avoiding
    // both a FIFO wait and a stat-then-blocking-open type replacement race.
    const int flags = (mode == "new" ? O_RDWR | O_CREAT | O_EXCL : (recovery ? O_RDWR | O_NONBLOCK : O_RDONLY | O_NONBLOCK)) | O_CLOEXEC | O_NOFOLLOW;
    file_fd_ = ::open(path_.c_str(), flags, 0600);
    if (file_fd_ < 0) {
      const auto code = mode == "new" && errno == EEXIST   ? MapErrorCode::DestinationExists
                        : mode != "new" && errno == ENOENT ? MapErrorCode::MissingMap
                                                           : MapErrorCode::Storage;
      throw MapError(code, "Cannot open map " + path_ + ": " + std::strerror(errno));
    }
    database_detail::writableStorageTestPoint("owner-opened");
    if (::fstat(file_fd_, &file_identity_) != 0 || !S_ISREG(file_identity_.st_mode))
      throw MapError(MapErrorCode::Storage, "Map destination must be a regular file");
    if (::flock(file_fd_, (writable_ ? LOCK_EX : LOCK_SH) | LOCK_NB) != 0)
      throw MapError(MapErrorCode::WriterConflict, "Map is already owned by an incompatible opener");
    checkFileIdentity();
    // Eligibility inspects only a quiescent, checkpointed file; never recover sidecars.
    if (!recovery) checkNoJournal();
    if (!writable_) {
      if (file_identity_.st_size < 100 || static_cast<std::uintmax_t>(file_identity_.st_size) > std::numeric_limits<std::size_t>::max() ||
          static_cast<std::uintmax_t>(file_identity_.st_size) > static_cast<std::uintmax_t>(std::numeric_limits<sqlite3_int64>::max()))
        throw MapError(MapErrorCode::UnsupportedFormat, "Invalid or unsupported map file size");
      mapped_size_ = static_cast<std::size_t>(file_identity_.st_size);
      mapped_data_ = ::mmap(nullptr, mapped_size_, PROT_READ | PROT_WRITE, MAP_PRIVATE, file_fd_, 0);
      if (mapped_data_ == MAP_FAILED) throw MapError(MapErrorCode::Storage, "Cannot map validated map inode: " + std::string(std::strerror(errno)));
      auto *header = static_cast<unsigned char *>(mapped_data_);
      if (std::memcmp(header, "SQLite format 3\0", 16) != 0 || !((header[18] == 1 && header[19] == 1) || (header[18] == 2 && header[19] == 2)))
        throw MapError(MapErrorCode::UnsupportedFormat, "Invalid or unsupported SQLite map header");
      // SQLite deserialize requires rollback-format bytes. Adapt only this
      // private view of a clean WAL-header file, never its canonical bytes.
      if (header[18] == 2) header[18] = header[19] = 1;
      if (::mprotect(mapped_data_, mapped_size_, PROT_READ) != 0)
        throw MapError(MapErrorCode::Storage, "Cannot protect map eligibility view: " + std::string(std::strerror(errno)));
    }
    // Resume must not reopen any filesystem pathname, including /proc/self/fd:
    // SQLite's VFS can canonicalize such a reference back to a mutable path.
    if (writable_)
      openWritableConnection();
    else if (sqlite3_open_v2(":memory:", &db_, SQLITE_OPEN_FULLMUTEX | SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, nullptr) != SQLITE_OK)
      throw MapError(MapErrorCode::Storage, db_ ? sqlite3_errmsg(db_) : "Cannot allocate historical SQLite handle");
    database_detail::writableStorageTestPoint("validating");
    if (recovery) {
      validateSchema();
      readIdentity(true);
      execute("PRAGMA foreign_keys=ON;");
      execute("PRAGMA synchronous=NORMAL;");
      execute("PRAGMA temp_store=MEMORY;");
      validateHistoricalRecords(grid_resolution);
      first_finalized_commit_ = true;
      route_s_only_ = true;
    } else if (writable_) {
      if (!validConfigIdentity(config_identity_)) throw MapError(MapErrorCode::IncompatibleConfig, "Invalid configuration identity");
      map_uuid_ = makeUuid();
      execute("PRAGMA foreign_keys=ON;");
      execute("BEGIN IMMEDIATE;");
      try {
        createSchema();
        execute(archivesNavigation() ? "PRAGMA user_version=1;" : (retainsScenes() ? "PRAGMA user_version=3;" : "PRAGMA user_version=2;"));
        execute("COMMIT;");
      } catch (...) {
        sqlite3_exec(db_, "ROLLBACK;", nullptr, nullptr, nullptr);
        throw;
      }
      setJournalMode("wal");
      execute("PRAGMA synchronous=NORMAL;");
      execute("PRAGMA temp_store=MEMORY;");
    } else {
      const auto size = static_cast<sqlite3_int64>(mapped_size_);
      if (sqlite3_deserialize(db_, "main", static_cast<unsigned char *>(mapped_data_), size, size, SQLITE_DESERIALIZE_READONLY) != SQLITE_OK)
        throw MapError(MapErrorCode::Storage, sqlite3_errmsg(db_));
      validateSchema();
      readIdentity(true);
      checkNoJournal();
    }
    checkFileIdentity();
    state_ = writable_ ? State::Writable : State::HistoricalReadOnly;
  } catch (const MapError &) {
    state_ = State::Failed;
    closeHandles();
    throw;
  } catch (const std::exception &error) {
    state_ = State::Failed;
    closeHandles();
    throw MapError(MapErrorCode::Storage, error.what());
  }
}

std::uint64_t MapDatabase::commitFinalizedSubmap(const SubmapFrame &submap, const LocalGrid &grid, const std::vector<std::uint8_t> &scene,
                                                 const std::vector<std::pair<int, Eigen::Isometry3f>> &poses, const GraphLink &base,
                                                 const std::optional<GraphLink> &loop, const std::string &expected_uuid,
                                                 std::uint64_t expected_revision, int expected_next_id, const std::string &expected_config,
                                                 int expected_chain_root) {
  requireWritable();
  // Encoding temporaries only, not another runtime/persistence owner. Every copy,
  // matrix layout conversion and input payload validation precedes mutex/BEGIN.
  // Persisted occupancy evidence is traversed under the mutex, still before BEGIN.
  struct MatrixBytes {
    int rows, cols, type;
    std::vector<std::uint8_t> bytes;
  };
  struct ImageBytes {
    double stamp;
    int camera;
    std::size_t point_count;
    std::vector<std::uint8_t> points;
    MatrixBytes descriptors;
  };
  struct PoseBytes {
    int id;
    Eigen::Matrix4f pose;
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> tag;
  };
  std::vector<std::uint8_t> gaussian, scene_bytes;
  std::vector<double> odom_times, odom, nav_times;
  std::vector<float> ground, obstacles, empty, nav_samples, controls, knots;
  std::vector<ImageBytes> images;
  std::vector<std::pair<int, MatrixBytes>> tags;
  std::vector<PoseBytes> final_poses;
  std::vector<std::pair<int, Eigen::Isometry3f>> occupancy_poses;
  std::string pyramid;
  Eigen::Matrix4f final_pose;
  Eigen::Isometry3d original_anchor;
  int id = 0;
  const auto bounded = [](std::size_t count, std::size_t stride) {
    if (count > std::size_t(std::numeric_limits<int>::max()) / stride)
      throw MapError(MapErrorCode::HistoricalData, "Finalized payload exceeds SQLite byte limit");
  };
  const auto rigid = [](const Eigen::Matrix4f &p) {
    const Eigen::Matrix3d r = p.topLeftCorner<3, 3>().cast<double>();
    requireHistory(p.allFinite() && p.row(3) == Eigen::RowVector4f(0, 0, 0, 1) && (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() <= 1e-6 &&
                       std::abs(r.determinant() - 1) <= 1e-6,
                   "Invalid finalized rigid transform");
  };
  const auto pack_matrix = [&](const cv::Mat &m) -> MatrixBytes {
    if (m.empty()) return {0, 0, -1, {}};
    requireHistory((m.type() == CV_8UC1 || m.type() == CV_32FC1) && cv::checkRange(m), "Invalid finalized matrix");
    bounded(m.total(), m.elemSize());
    const auto contiguous = m.isContinuous() ? m : m.clone();
    return {m.rows, m.cols, m.type(), {contiguous.data, contiguous.data + m.total() * m.elemSize()}};
  };
  try {
    id = databaseId(submap.id());
    requireHistory(id >= 2 && id == expected_next_id, "Finalized append requires canonical next SQL ID after an existing Node");
    requireHistory(submap.lio().pcd && !submap.lio().pcd->empty() && !submap.odom_poses().empty(), "Finalized submap lacks required cloud/odometry");
    requireHistory(submap.lio().pcd->size() <= (std::size_t(std::numeric_limits<int>::max()) - database_detail::kGaussianPayloadHeaderBytes) /
                                                   database_detail::kGaussianPayloadRecordBytes,
                   "Finalized cloud exceeds SQLite byte limit");
    gaussian = database_detail::packGaussianCloud(*submap.lio().pcd);
    bounded(gaussian.size(), 1);
    bounded(submap.odom_poses().size(), 7 * sizeof(double));
    double previous = -std::numeric_limits<double>::infinity();
    for (const auto &sample : submap.odom_poses()) {
      const auto &p = sample.T_odom_base;
      requireHistory(std::isfinite(sample.timestamp) && sample.timestamp >= previous && p.matrix().allFinite() &&
                         p.matrix().row(3) == Eigen::RowVector4d(0, 0, 0, 1) &&
                         (p.rotation().transpose() * p.rotation() - Eigen::Matrix3d::Identity()).norm() <= 1e-9 &&
                         std::abs(p.rotation().determinant() - 1) <= 1e-9,
                     "Invalid finalized original odometry");
      previous = sample.timestamp;
    }
    original_anchor = submap.odom_poses().front().T_odom_base;
    requireHistory(std::isfinite(submap.lio().timestamp) && std::abs(submap.lio().timestamp - submap.odom_poses().front().timestamp) <= 1e-9,
                   "Finalized stamp differs from original anchor");
    odom_times = database_detail::packOdomTimestamps(submap.odom_poses());
    odom = database_detail::packOdomPoses(submap.odom_poses());
    std::set<int> unique;
    bool has_new_pose = false;
    for (const auto &[node, pose] : poses) {
      requireHistory(node > 0 && node <= id && unique.insert(node).second, "Invalid/duplicate finalized pose identity");
      rigid(pose.matrix());
      final_poses.push_back({node, pose.matrix(), pose.matrix()});
      if (node == id) {
        final_pose = pose.matrix();
        has_new_pose = true;
      }
    }
    requireHistory(has_new_pose, "Finalized new Node requires its committed pose");
    rigid(base.transform.matrix());
    if (loop) rigid(loop->transform.matrix());
    requireHistory(
        (base.type == 0 && base.from_id == id - 1 && base.to_id == id) || (base.type == 1 && base.from_id == id && base.to_id > 0 && base.to_id < id),
        "Invalid finalized base factor");
    if (loop)
      requireHistory(base.type == 0 && loop->type == 1 && loop->from_id == id && loop->to_id > 0 && loop->to_id < id &&
                         expected_chain_root > 0 && expected_chain_root < id &&
                         loop_policy::eligible(id - 1, loop->to_id - 1, expected_chain_root - 1,
                                               loop->to_id < expected_chain_root ? loop->to_id - 1 : expected_chain_root - 1, id - 1),
                     "Invalid finalized ordinary loop or duplicate root type-1 slot");
    const auto &voxels = submap.pyramid_voxels();
    requireHistory(voxels.max_level >= 0 && voxels.max_level <= 32 && voxels.valid(), "Invalid finalized pyramid");
    std::size_t pyramid_bytes = 8 + sizeof(std::uint32_t) + sizeof(float) + 2 * sizeof(int);
    for (const auto &buckets : voxels.level_buckets) {
      requireHistory(buckets.size() <= submap.lio().pcd->size() * 128 && buckets.size() - 1 <= std::numeric_limits<std::uint32_t>::max(),
                     "Unrepresentable finalized pyramid buckets");
      const auto occupied = std::count_if(buckets.begin(), buckets.end(), [](const auto &bucket) { return bucket.w() != 0; });
      requireHistory(pyramid_bytes <= std::size_t(std::numeric_limits<int>::max()) - 2 * sizeof(std::size_t) &&
                         std::size_t(occupied) <= (std::size_t(std::numeric_limits<int>::max()) - pyramid_bytes - 2 * sizeof(std::size_t)) / 16,
                     "Finalized pyramid exceeds SQLite byte limit");
      pyramid_bytes += 2 * sizeof(std::size_t) + std::size_t(occupied) * 16;
    }
    std::ostringstream encoded(std::ios::binary);
    cpu::save_voxelmaps(encoded, voxels);
    pyramid = encoded.str();
    database_detail::validatePyramidPayload(pyramid.data(), pyramid.size(), submap.lio().pcd->size());
    if (archivesNavigation()) {
      requireHistory(std::isfinite(grid.cellSize) && grid.cellSize > 0 && grid.viewPoint.allFinite(), "Invalid finalized grid metadata");
      for (const auto *points : {&grid.groundCells, &grid.obstacleCells, &grid.emptyCells}) {
        bounded(points->size(), 2 * sizeof(float));
        for (const auto &point : *points) requireHistory(point.allFinite(), "Invalid finalized grid evidence");
      }
      ground = database_detail::packGrid(grid.groundCells);
      obstacles = database_detail::packGrid(grid.obstacleCells);
      empty = database_detail::packGrid(grid.emptyCells);
    }
    if (archivesNavigation()) {
      const auto &navigation = submap.navigation();
      bounded(navigation.samples.size(), 8 * sizeof(float));
      bounded(navigation.spline.control_points.size(), 3 * sizeof(float));
      bounded(navigation.spline.knots.size(), sizeof(float));
      previous = -std::numeric_limits<double>::infinity();
      for (const auto &s : navigation.samples) {
        requireHistory(std::isfinite(s.timestamp) && s.timestamp >= previous && std::isfinite(s.x) && std::isfinite(s.y) && std::isfinite(s.z) &&
                           std::isfinite(s.qx) && std::isfinite(s.qy) && std::isfinite(s.qz) && std::isfinite(s.qw) && std::isfinite(s.distance),
                       "Invalid finalized navigation sample");
        previous = s.timestamp;
      }
      requireHistory(navigation.spline.degree >= 0 && navigation.spline.degree <= 3 &&
                         ((navigation.spline.control_points.empty() && navigation.spline.knots.empty()) || navigation.spline.valid()),
                     "Invalid finalized navigation spline");
      for (float knot : navigation.spline.knots) requireHistory(std::isfinite(knot), "Nonfinite finalized navigation knot");
      nav_times = database_detail::packTrajectoryTimestamps(navigation.samples);
      nav_samples = database_detail::packTrajectorySamples(navigation.samples);
      controls = database_detail::packCloud(navigation.spline.control_points);
      knots = navigation.spline.knots;
    }
    bounded(submap.visual_frames().size(), 1);
    std::set<std::array<std::uint8_t, 32>> unique_descriptors;
    std::vector<std::array<std::uint8_t, 32>> descriptors;
    for (const auto &image : submap.visual_frames()) {
      requireHistory(std::isfinite(image.timestamp()) && image.camera_id() <= 1, "Invalid finalized image identity");
      bounded(image.points().size(), 4 * sizeof(float));
      const auto &m = image.descriptors();
      requireHistory(m.empty() ? image.points().empty() : m.type() == CV_8UC1 && m.cols == 32 && std::size_t(m.rows) == image.points().size(),
                     "Invalid finalized appearance descriptors");
      for (const auto &point : image.points())
        requireHistory(point.pixel().allFinite() && std::isfinite(point.response()) && point.level() >= 0 && point.level() <= 255,
                       "Invalid finalized image point");
      for (int row = 0; row < m.rows && descriptors.size() < mapping::scene::point_capacity; ++row) {
        std::array<std::uint8_t, 32> d;
        std::memcpy(d.data(), m.ptr(row), d.size());
        if (unique_descriptors.insert(d).second) descriptors.push_back(d);
      }
      images.push_back({image.timestamp(), static_cast<int>(image.camera_id()), image.points().size(), database_detail::packVisualObservation(image), pack_matrix(m)});
    }
    requireHistory(scene.size() <= 4 * 1024 * 1024, "Finalized scene exceeds current payload limit");
    scene_bytes = scene;
    if (!scene_bytes.empty()) {
      const auto decoded = mapping::scene::FeatureMap::decode(scene_bytes.data(), scene_bytes.size());
      requireHistory(decoded->vocabulary_hash() == 0 && decoded->max_points() == mapping::scene::point_capacity &&
                         decoded->points().size() == descriptors.size(),
                     "Invalid finalized scene identity/membership");
      validateSceneGeometry(*decoded, *buildVisualScene(submap.visual_frames(), original_anchor));
      for (std::size_t i = 0; i < descriptors.size(); ++i) {
        const auto &p = decoded->points()[i];
        requireHistory(p.id == i + 1 && p.appearance_count == 1 && p.appearances[0].descriptor == descriptors[i] &&
                           p.appearances[0].word_id == -1 && p.appearances[0].tree_node == features::FeatureBlock::invalid_node_id,
                       "Invalid finalized scene appearance");
      }
    }
    for (const auto &tag : submap.tags())
      if (tag && tag->type() != MetaTagType::kPose) tags.emplace_back(static_cast<int>(tag->type()), pack_matrix(tag->descriptor()));
    tags.emplace_back(static_cast<int>(MetaTagType::kPose), pack_matrix(MetaTag(Eigen::Isometry3f(final_pose)).descriptor()));
    occupancy_poses = poses;
    std::sort(occupancy_poses.begin(), occupancy_poses.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
    database_detail::writableStorageTestPoint("prepared-before-transaction");
  } catch (const MapError &) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    state_ = State::Failed;
    throw;
  } catch (const std::bad_alloc &) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    state_ = State::Failed;
    throw MapError(MapErrorCode::Storage, "Insufficient memory for finalized payloads");
  } catch (const std::exception &e) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    state_ = State::Failed;
    throw MapError(MapErrorCode::HistoricalData, e.what());
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  requireWritable();
  bool begun = false, commit_attempted = false, committed = false;
  // Persisted-evidence preflight precedes BEGIN; the transaction remains fixed SQL/checked bindings.
  const auto blob = [](sqlite3_stmt *s, int column, const void *data, std::size_t bytes) {
    const int rc = bytes ? sqlite3_bind_blob(s, column, data, static_cast<int>(bytes), SQLITE_TRANSIENT) : sqlite3_bind_zeroblob(s, column, 0);
    if (rc != SQLITE_OK) throw MapError(MapErrorCode::Storage, "Cannot bind finalized payload");
  };
  const auto run = [&](const char *sql, const auto &bind) {
    database_detail::Statement statement(db_, sql);
    bind(statement.get());
    database_detail::step(db_, statement.get());
  };
  try {
    validateWriter();
    if (archivesNavigation()) {
      // Read one persisted grid at a time under the existing owner mutex. Combine
      // unchanged poses with proposed historical poses and the new Node, before
      // BEGIN. The authoritative revision/topology recheck below remains unchanged.
      const double inf = std::numeric_limits<double>::infinity();
      Eigen::Vector4d occupancy_bounds(inf, inf, -inf, -inf);
      {
        auto proposed = occupancy_poses.begin();
        database_detail::Statement nodes(db_, "SELECT id,submap_pose FROM Node ORDER BY id;");
        int rc;
        while ((rc = sqlite3_step(nodes.get())) == SQLITE_ROW) {
          const int historical_id = static_cast<int>(database_detail::readInteger(nodes.get(), 0, 1, std::numeric_limits<int>::max()));
          while (proposed != occupancy_poses.end() && proposed->first < historical_id) ++proposed;
          const auto final_historical_pose =
              proposed != occupancy_poses.end() && proposed->first == historical_id ? proposed->second : pose(nodes.get(), 1);
          LocalGrid historical_grid;
          requireHistory(loadLocalGrid(historical_id, historical_grid), "Missing historical LocalGrid for finalized occupancy");
          includeOccupancyEvidence(historical_grid, final_historical_pose, occupancy_resolution_, occupancy_bounds);
        }
        done(db_, rc);
      }
      includeOccupancyEvidence(grid, Eigen::Isometry3f(final_pose), occupancy_resolution_, occupancy_bounds);
      validateOccupancyExtent(occupancy_bounds);
    }
    execute("BEGIN IMMEDIATE;");
    begun = true;
    validateWriter();
    if (expected_uuid != map_uuid_ || expected_config != config_identity_ || expected_revision != std::uint64_t(expected_revision_))
      throw MapError(MapErrorCode::StaleMap, "Finalized preparation identity/config/revision is stale");
    if (expected_revision_ == std::numeric_limits<sqlite3_int64>::max()) throw MapError(MapErrorCode::Storage, "Map revision exhausted");
    std::vector<Eigen::Isometry3d> anchors;
    visitHistoricalNodes([&](int, const auto &a, const auto &, const auto &) { anchors.push_back(a); });
    if (anchors.size() + 1 != std::size_t(expected_next_id)) throw MapError(MapErrorCode::StaleMap, "Finalized next Node identity is stale");
    auto links = loadGraphLinks();
    (void)validateTopology(anchors, links);
    anchors.push_back(original_anchor);
    links.push_back(base);
    if (loop) links.push_back(*loop);
    const auto roots = validateTopology(anchors, links);
    if (roots.back() != expected_chain_root) throw MapError(MapErrorCode::StaleMap, "Finalized chain topology differs from preparation");
    // Header/identity/topology checked before first application write.
    run("INSERT INTO Node(id,stamp,odom_pose_count,odom_timestamps,odom_poses,submap_pose) VALUES(?,?,?,?,?,?);", [&](auto *s) {
      sqlite3_bind_int(s, 1, id);
      sqlite3_bind_double(s, 2, submap.lio().timestamp);
      sqlite3_bind_int64(s, 3, odom_times.size());
      blob(s, 4, odom_times.data(), odom_times.size() * sizeof(double));
      blob(s, 5, odom.data(), odom.size() * sizeof(double));
      blob(s, 6, final_pose.data(), 64);
    });
    database_detail::writableStorageTestPoint("after-node");
    if (retainsScenes()) run("INSERT INTO SceneState(node_id) VALUES(?);", [&](auto *s) { sqlite3_bind_int(s,1,id); });
    run("INSERT INTO SpatialRecord VALUES(?,?,?,?,?,?,?);", [&](auto *s) {
      sqlite3_bind_int(s, 1, id);
      for (int axis = 0; axis < 3; ++axis) {
        sqlite3_bind_double(s, axis + 2, submap.bounds().minimum[axis]);
        sqlite3_bind_double(s, axis + 5, submap.bounds().maximum[axis]);
      }
    });
    database_detail::writableStorageTestPoint("after-spatial");
    run("INSERT INTO LaserRecord VALUES(?,?,?);", [&](auto *s) {
      sqlite3_bind_int(s, 1, id);
      sqlite3_bind_int64(s, 2, submap.lio().pcd->size());
      blob(s, 3, gaussian.data(), gaussian.size());
    });
    database_detail::writableStorageTestPoint("after-cloud");
    run("INSERT INTO PyramidVoxel VALUES(?,?);", [&](auto *s) {
      sqlite3_bind_int(s, 1, id);
      blob(s, 2, pyramid.data(), pyramid.size());
    });
    database_detail::writableStorageTestPoint("after-pyramid");
    if (archivesNavigation()) run("INSERT INTO NaviTrajectory VALUES(?,?,?,?,?,?,?,?,?);", [&](auto *s) {
      sqlite3_bind_int(s, 1, id);
      sqlite3_bind_int64(s, 2, nav_times.size());
      blob(s, 3, nav_times.data(), nav_times.size() * sizeof(double));
      blob(s, 4, nav_samples.data(), nav_samples.size() * sizeof(float));
      sqlite3_bind_int(s, 5, submap.navigation().spline.degree);
      sqlite3_bind_int64(s, 6, controls.size() / 3);
      blob(s, 7, controls.data(), controls.size() * sizeof(float));
      sqlite3_bind_int64(s, 8, knots.size());
      blob(s, 9, knots.data(), knots.size() * sizeof(float));
    });
    database_detail::writableStorageTestPoint("after-navigation");
    if (archivesNavigation()) run("INSERT INTO FlatGrid VALUES(?,?,?,?,?,?,?,?,?,?,?);", [&](auto *s) {
      sqlite3_bind_int(s, 1, id);
      sqlite3_bind_int64(s, 2, ground.size() / 2);
      blob(s, 3, ground.data(), ground.size() * sizeof(float));
      sqlite3_bind_int64(s, 4, obstacles.size() / 2);
      blob(s, 5, obstacles.data(), obstacles.size() * sizeof(float));
      sqlite3_bind_int64(s, 6, empty.size() / 2);
      blob(s, 7, empty.data(), empty.size() * sizeof(float));
      sqlite3_bind_double(s, 8, grid.cellSize);
      for (int axis = 0; axis < 3; ++axis) sqlite3_bind_double(s, axis + 9, grid.viewPoint[axis]);
    });
    database_detail::writableStorageTestPoint("after-grid");
    for (std::size_t i = 0; i < images.size(); ++i) {
      const auto &v = images[i];
      run("INSERT INTO "
          "ImageRecord(node_id,frame_index,stamp,point_count,points,descriptors_rows,descriptors_cols,descriptors_type,descriptors,camera_id) "
          "VALUES(?,?,?,?,?,?,?,?,?,?);",
          [&](auto *s) {
            sqlite3_bind_int(s, 1, id);
            sqlite3_bind_int64(s, 2, i);
            sqlite3_bind_double(s, 3, v.stamp);
            sqlite3_bind_int64(s, 4, v.point_count);
            blob(s, 5, v.points.data(), v.points.size());
            sqlite3_bind_int(s, 6, v.descriptors.rows);
            sqlite3_bind_int(s, 7, v.descriptors.cols);
            sqlite3_bind_int(s, 8, v.descriptors.type);
            blob(s, 9, v.descriptors.bytes.data(), v.descriptors.bytes.size());
            sqlite3_bind_int(s, 10, v.camera);
          });
    }
    database_detail::writableStorageTestPoint("after-images");
    if (!scene_bytes.empty())
      run("INSERT INTO VisualScene VALUES(?,?);", [&](auto *s) {
        sqlite3_bind_int(s, 1, id);
        blob(s, 2, scene_bytes.data(), scene_bytes.size());
      });
    database_detail::writableStorageTestPoint("after-scene");
    for (const auto &[type, m] : tags)
      run("INSERT INTO MetaTag VALUES(?,?,?,?,?,?);", [&](auto *s) {
        sqlite3_bind_int(s, 1, id);
        sqlite3_bind_int(s, 2, type);
        sqlite3_bind_int(s, 3, m.rows);
        sqlite3_bind_int(s, 4, m.cols);
        sqlite3_bind_int(s, 5, m.type);
        blob(s, 6, m.bytes.data(), m.bytes.size());
      });
    database_detail::writableStorageTestPoint("after-tags");
    const auto insert_link = [&](const GraphLink &link) {
      run("INSERT INTO Link VALUES(?,?,?,?);", [&](auto *s) {
        sqlite3_bind_int(s, 1, link.from_id);
        sqlite3_bind_int(s, 2, link.to_id);
        sqlite3_bind_int(s, 3, link.type);
        blob(s, 4, link.transform.data(), 64);
      });
    };
    insert_link(base);
    if (loop) insert_link(*loop);
    database_detail::writableStorageTestPoint("after-factors");
    for (const auto &p : final_poses) {
      run("UPDATE Node SET submap_pose=? WHERE id=?;", [&](auto *s) {
        blob(s, 1, p.pose.data(), 64);
        sqlite3_bind_int(s, 2, p.id);
      });
      if (sqlite3_changes(db_) != 1) throw MapError(MapErrorCode::StaleMap, "Finalized pose references missing Node");
      database_detail::writableStorageTestPoint("during-pose-updates");
      run("UPDATE MetaTag SET tag_rows=4,tag_cols=4,tag_cv_type=5,tag=? WHERE node_id=? AND tag_type=2;", [&](auto *s) {
        blob(s, 1, p.tag.data(), 64);
        sqlite3_bind_int(s, 2, p.id);
      });
      if (sqlite3_changes(db_) != 1) throw MapError(MapErrorCode::HistoricalData, "Finalized pose witness missing");
      database_detail::writableStorageTestPoint("during-tag-updates");
    }
    database_detail::writableStorageTestPoint("before-revision");
    run("UPDATE MapState SET graph_revision=graph_revision+1 WHERE id=1 AND graph_revision=? AND map_uuid=? AND config_identity=?;", [&](auto *s) {
      sqlite3_bind_int64(s, 1, expected_revision_);
      sqlite3_bind_text(s, 2, expected_uuid.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(s, 3, expected_config.c_str(), -1, SQLITE_TRANSIENT);
    });
    if (sqlite3_changes(db_) != 1) throw MapError(MapErrorCode::StaleMap, "Conditional finalized revision increment did not match exactly one row");
    database_detail::writableStorageTestPoint("before-commit");
    commit_attempted = true;
    const int rc = sqlite3_exec(db_, "COMMIT;", nullptr, nullptr, nullptr);
    if (rc != SQLITE_OK) throw MapError(MapErrorCode::CommitAmbiguous, "COMMIT outcome requires authoritative reopen", CommitOutcome::Unknown);
    committed = true;
    begun = false;
    ++expected_revision_;  // No fallible operation before recording the known commit.
    database_detail::writableStorageTestPoint("after-commit");
    if (first_finalized_commit_) {
      database_detail::writableStorageTestPoint("before-wal-activation");
      setJournalMode("wal");
      first_finalized_commit_ = false;
    }
    return static_cast<std::uint64_t>(expected_revision_);
  } catch (...) {
    const auto error = std::current_exception();
    state_ = State::Failed;
    bool rollback_failed = false;
    if (begun && !sqlite3_get_autocommit(db_)) rollback_failed = sqlite3_exec(db_, "ROLLBACK;", nullptr, nullptr, nullptr) != SQLITE_OK;
    if (committed)
      throw MapError(MapErrorCode::CommittedFailure, "Finalized Node committed; subsequent storage/materialization failed", CommitOutcome::Committed);
    if (commit_attempted || rollback_failed)
      throw MapError(MapErrorCode::CommitAmbiguous, "Storage outcome/cleanup uncertain; close and reconstruct before retry", CommitOutcome::Unknown);
    try {
      std::rethrow_exception(error);
    } catch (const MapError &) {
      throw;
    } catch (const std::exception &e) {
      throw MapError(MapErrorCode::Storage, e.what());
    }
  }
}

}  // namespace sapphire

namespace sapphire {
bool MapDatabase::sceneActive(int id) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  requireReadable();
  if (!retainsScenes()) return true;
  database_detail::Statement row(db_, "SELECT retired_by FROM SceneState WHERE node_id=?;");
  sqlite3_bind_int(row.get(),1,id);
  requireHistory(sqlite3_step(row.get())==SQLITE_ROW,"Missing scene membership");
  return sqlite3_column_type(row.get(),0)==SQLITE_NULL;
}
namespace {
// Completeness over the bounded indexed visual scene, not a claim of free-space
// visibility or equivalent camera FOV. Missing metric support preserves old views.
bool preservesVisualScene(const mapping::scene::FeatureMap &old_scene,
                          const mapping::scene::FeatureMap &new_scene,
                          const Eigen::Isometry3d &T_new_old) {
  using Cell=std::array<std::int64_t,3>;
  const auto cell=[](const Eigen::Vector3d &p) {
    Cell c;
    for(int a=0;a<3;++a) {
      const double v=std::floor(p[a]/.2);
      if(!std::isfinite(v)||std::abs(v)>0x1p50) throw MapError(MapErrorCode::HistoricalData,"Invalid visual refresh coordinates");
      c[a]=static_cast<std::int64_t>(v);
    }
    return c;
  };
  const auto xyz=[](const mapping::scene::Landmark &p) { return Eigen::Vector3d(p.position.x,p.position.y,p.position.z); };
  const auto &old=old_scene.points(), &fresh=new_scene.points();
  if(old.empty()) return true;
  if(old.size()>fresh.size()) return false;
  std::map<Cell,std::vector<std::size_t>> index;
  for(std::size_t j=0;j<fresh.size();++j) if(fresh[j].has_position) index[cell(xyz(fresh[j]))].push_back(j);
  struct Pair { int distance; double error; std::size_t old, fresh; };
  std::vector<Pair> pairs;
  std::size_t comparisons=0;
  for(std::size_t i=0;i<old.size();++i) {
    if(!old[i].has_position || old[i].appearance_count!=1) return false;
    const Eigen::Vector3d position=T_new_old*xyz(old[i]);
    const auto center=cell(position);
    for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z) {
      const auto found=index.find({center[0]+x,center[1]+y,center[2]+z});
      if(found==index.end())continue;
      for(auto j:found->second) {
        if(++comparisons>200000)return false;
        const double error=(position-xyz(fresh[j])).squaredNorm();
        if(error>.04 || fresh[j].appearance_count!=1)continue;
        int distance=0;
        for(std::size_t b=0;b<32;++b)distance+=__builtin_popcount(unsigned(old[i].appearances[0].descriptor[b]^fresh[j].appearances[0].descriptor[b]));
        if(distance<=32)pairs.push_back({distance,error,i,j});
      }
    }
  }
  std::sort(pairs.begin(),pairs.end(),[](const auto &a,const auto &b) {
    return std::tie(a.distance,a.error,a.old,a.fresh)<std::tie(b.distance,b.error,b.old,b.fresh);
  });
  std::vector<bool> used_old(old.size()),used_new(fresh.size());std::size_t matched=0;
  for(const auto &pair:pairs)if(!used_old[pair.old]&&!used_new[pair.fresh]) {
    used_old[pair.old]=used_new[pair.fresh]=true;++matched;
  }
  return matched==old.size();
}
}
std::optional<std::uint64_t> MapDatabase::refreshCoveredScene(int old_id,int new_id) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  requireWritable();
  if(!retainsScenes())return std::nullopt;
  validateWriter();
  if(old_id<=0 || new_id<=old_id)throw MapError(MapErrorCode::Lifecycle,"Invalid scene refresh identity order");
  if(!sceneActive(old_id)||!sceneActive(new_id))return std::nullopt;
  Eigen::Isometry3f T_new_old;
  {
    database_detail::Statement link(db_,"SELECT transform FROM Link WHERE from_id=? AND to_id=? AND type=1;");
    sqlite3_bind_int(link.get(),1,new_id);sqlite3_bind_int(link.get(),2,old_id);
    const auto rc=sqlite3_step(link.get());
    if(rc==SQLITE_DONE)return std::nullopt;
    if(rc!=SQLITE_ROW)throw MapError(MapErrorCode::Storage,sqlite3_errmsg(db_));
    T_new_old=pose(link.get(),0);
  }
  // Reject oversized evidence before allocating another pair of decoded clouds.
  // Ordinary registration already owns its own bounded working set.
  for(int id:{old_id,new_id}) {
    database_detail::Statement count(db_,"SELECT gaussian_count FROM LaserRecord WHERE node_id=?;");
    sqlite3_bind_int(count.get(),1,id);
    requireHistory(sqlite3_step(count.get())==SQLITE_ROW,"Missing active refresh geometry");
    const auto size=database_detail::readInteger(count.get(),0,1,std::numeric_limits<int>::max());
    if(size>static_cast<sqlite3_int64>(RevisitCoverageOptions{}.max_points))return std::nullopt;
  }
  const auto old_cloud=loadCloud(old_id), new_cloud=loadCloud(new_id);
  const auto coverage=measureRevisitCoverage(*new_cloud,*old_cloud,T_new_old.cast<double>().inverse());
  if(!coverage.support || coverage.support->pairs!=old_cloud->size())return std::nullopt;
  const auto old_scene=buildVisualScene(loadVisualFrames(old_id),loadOriginalAnchor(old_id));
  const auto new_scene=buildVisualScene(loadVisualFrames(new_id),loadOriginalAnchor(new_id));
  // An accepted measurement need not reproduce the committed layout exactly.
  // Deleting old support needs agreement in that authoritative layout too; a
  // finite residual only defers retirement and never changes loop acceptance.
  const auto committedPose=[&](int id) -> Eigen::Isometry3d {
    database_detail::Statement row(db_,"SELECT submap_pose FROM Node WHERE id=?;");
    sqlite3_bind_int(row.get(),1,id);
    requireHistory(sqlite3_step(row.get())==SQLITE_ROW,"Missing refresh committed pose");
    return pose(row.get(),0).cast<double>();
  };
  const auto T_map_old=committedPose(old_id), T_map_new=committedPose(new_id);
  const Eigen::Isometry3d measured=T_new_old.cast<double>();
  const Eigen::Isometry3d inverse_measured=measured.inverse();
  for(const auto &p:*new_cloud)
    if((T_map_new*p.mean.cast<double>()-T_map_old*(inverse_measured*p.mean.cast<double>())).norm()>.02)
      return std::nullopt;
  for(const auto &p:old_scene->points()) if(p.has_position) {
    const Eigen::Vector3d position(p.position.x,p.position.y,p.position.z);
    if((T_map_old*position-T_map_new*(measured*position)).norm()>.02)return std::nullopt;
  }
  for(const auto &[id,scene]:{std::make_pair(old_id,old_scene),std::make_pair(new_id,new_scene)}) {
    const auto payload=loadVisualScene(id);
    if(!payload.empty()) {
      const auto decoded=mapping::scene::FeatureMap::decode(payload.data(),payload.size());
      requireHistory(decoded->vocabulary_hash()==0 && decoded->max_points()==mapping::scene::point_capacity,
                     "Invalid refresh scene identity");
      validateSceneGeometry(*decoded,*scene); // Same original-anchor float tolerance as A2/W.
      for(std::size_t i=0;i<scene->points().size();++i) {
        const auto &p=decoded->points()[i], &expected=scene->points()[i];
        requireHistory(p.id==expected.id && p.appearance_count==1 &&
                       p.appearances[0].descriptor==expected.appearances[0].descriptor &&
                       p.appearances[0].word_id==-1 && p.appearances[0].tree_node==features::FeatureBlock::invalid_node_id,
                       "Scene refresh appearance disagrees with frozen images");
      }
    }
  }
  if(!preservesVisualScene(*old_scene,*new_scene,T_new_old.cast<double>()))return std::nullopt;
  if(expected_revision_==std::numeric_limits<sqlite3_int64>::max())throw MapError(MapErrorCode::Storage,"Scene refresh revision overflow");
  bool begun=false,commit_attempted=false,committed=false;
  try {
    database_detail::writableStorageTestPoint("refresh-before-transaction");
    execute("BEGIN IMMEDIATE;");begun=true;validateWriter();
    for(const char *table:{"LaserRecord","PyramidVoxel","ImageRecord","VisualScene"}) {
      database_detail::Statement remove(db_,("DELETE FROM "+std::string(table)+" WHERE node_id=?;").c_str());
      sqlite3_bind_int(remove.get(),1,old_id);database_detail::step(db_,remove.get());
      database_detail::writableStorageTestPoint("refresh-after-delete");
    }
    {
      database_detail::Statement tags(db_,"DELETE FROM MetaTag WHERE node_id=? AND tag_type<>2;");
      sqlite3_bind_int(tags.get(),1,old_id);database_detail::step(db_,tags.get());
      database_detail::Statement state(db_,"UPDATE SceneState SET retired_by=?,retired_revision=? WHERE node_id=? AND retired_by IS NULL;");
      sqlite3_bind_int(state.get(),1,new_id);sqlite3_bind_int64(state.get(),2,expected_revision_+1);sqlite3_bind_int(state.get(),3,old_id);
      database_detail::step(db_,state.get());
      requireHistory(sqlite3_changes(db_)==1,"Scene membership changed during refresh");
      database_detail::Statement revision(db_,"UPDATE MapState SET graph_revision=graph_revision+1 WHERE id=1 AND graph_revision=?;");
      sqlite3_bind_int64(revision.get(),1,expected_revision_);database_detail::step(db_,revision.get());
      requireHistory(sqlite3_changes(db_)==1,"Scene refresh revision changed");
    }
    database_detail::writableStorageTestPoint("refresh-before-commit");
    commit_attempted=true;
    if(sqlite3_exec(db_,"COMMIT;",nullptr,nullptr,nullptr)!=SQLITE_OK)
      throw MapError(MapErrorCode::CommitAmbiguous,"Scene refresh commit outcome unknown",CommitOutcome::Unknown);
    committed=true;begun=false;++expected_revision_;
    database_detail::writableStorageTestPoint("refresh-after-commit");
    return static_cast<std::uint64_t>(expected_revision_);
  } catch(...) {
    const auto error=std::current_exception();state_=State::Failed;
    const bool rollback_failed=begun&&!sqlite3_get_autocommit(db_)&&sqlite3_exec(db_,"ROLLBACK;",nullptr,nullptr,nullptr)!=SQLITE_OK;
    if(committed)throw MapError(MapErrorCode::CommittedFailure,"Scene refresh committed before failure",CommitOutcome::Committed);
    if(commit_attempted||rollback_failed)throw MapError(MapErrorCode::CommitAmbiguous,"Scene refresh outcome uncertain",CommitOutcome::Unknown);
    std::rethrow_exception(error);
  }
}
} // namespace sapphire
