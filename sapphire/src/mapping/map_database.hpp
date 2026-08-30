#pragma once
#include <sqlite3.h>

#include <Eigen/Geometry>
#include <voxelmaps.hpp>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "aabb.hpp"
#include "key_frame.hpp"
#include "local_gridmap.hpp"

namespace sapphire {

namespace database_detail {

class Statement {
 public:
  Statement(sqlite3 *database, const char *sql) : database_(database) {
    const int result = sqlite3_prepare_v2(database_, sql, -1, &statement_, nullptr);
    if (result != SQLITE_OK) {
      throw std::runtime_error(sqlite3_errmsg(database_));
    }
  }

  ~Statement() { sqlite3_finalize(statement_); }
  Statement(const Statement &) = delete;
  Statement &operator=(const Statement &) = delete;
  sqlite3_stmt *get() const { return statement_; }

 private:
  sqlite3 *database_;
  sqlite3_stmt *statement_ = nullptr;
};

inline void step(sqlite3 *database, sqlite3_stmt *statement) {
  if (sqlite3_step(statement) != SQLITE_DONE) {
    throw std::runtime_error(sqlite3_errmsg(database));
  }
}

inline std::vector<float> packCloud(const vvec<float, 3> &cloud) {
  std::vector<float> data;
  data.reserve(cloud.size() * 3);
  for (const Eigen::Vector3f &point : cloud) {
    data.push_back(point.x());
    data.push_back(point.y());
    data.push_back(point.z());
  }
  return data;
}

inline std::vector<float> packGrid(const GridPoints &points) {
  std::vector<float> data;
  data.reserve(points.size() * 2);
  for (const GridPoint &point : points) {
    data.push_back(point.x());
    data.push_back(point.y());
  }
  return data;
}

inline void bindFloats(sqlite3_stmt *statement, int index, const std::vector<float> &data) {
  if (data.empty()) {
    sqlite3_bind_zeroblob(statement, index, 0);
  } else {
    sqlite3_bind_blob(statement, index, data.data(), static_cast<int>(data.size() * sizeof(float)), SQLITE_TRANSIENT);
  }
}

inline void bindDoubles(sqlite3_stmt *statement, int index, const std::vector<double> &data) {
  if (data.empty()) {
    sqlite3_bind_zeroblob(statement, index, 0);
  } else {
    sqlite3_bind_blob(statement, index, data.data(), static_cast<int>(data.size() * sizeof(double)), SQLITE_TRANSIENT);
  }
}

inline std::vector<double> packTrajectoryTimestamps(const std::vector<TrajectorySample> &samples) {
  std::vector<double> timestamps;
  timestamps.reserve(samples.size());
  for (const TrajectorySample &sample : samples) {
    timestamps.push_back(sample.timestamp);
  }
  return timestamps;
}

inline std::vector<float> packTrajectorySamples(const std::vector<TrajectorySample> &samples) {
  std::vector<float> data;
  data.reserve(samples.size() * 8);
  for (const TrajectorySample &sample : samples) {
    data.push_back(sample.x);
    data.push_back(sample.y);
    data.push_back(sample.z);
    data.push_back(sample.qx);
    data.push_back(sample.qy);
    data.push_back(sample.qz);
    data.push_back(sample.qw);
    data.push_back(sample.distance);
  }
  return data;
}

inline GridPoints readGrid(sqlite3_stmt *statement, int countColumn, int blobColumn) {
  const size_t count = static_cast<size_t>(sqlite3_column_int64(statement, countColumn));
  const int bytes = sqlite3_column_bytes(statement, blobColumn);
  if (bytes != static_cast<int>(count * 2 * sizeof(float))) {
    throw std::runtime_error("Invalid local-grid blob size");
  }
  const float *data = static_cast<const float *>(sqlite3_column_blob(statement, blobColumn));
  GridPoints points;
  points.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    points.emplace_back(data[i * 2], data[i * 2 + 1]);
  }
  return points;
}

inline std::vector<float> packVisualPoints(const std::vector<VisualPoint> &points) {
  std::vector<float> data;
  data.reserve(points.size() * 4);
  for (const VisualPoint &point : points) {
    data.push_back(point.pixel().x());
    data.push_back(point.pixel().y());
    data.push_back(static_cast<float>(point.level()));
    data.push_back(point.response());
  }
  return data;
}

inline std::vector<VisualPoint> readVisualPoints(sqlite3_stmt *statement, int countColumn, int blobColumn) {
  const sqlite3_int64 storedCount = sqlite3_column_int64(statement, countColumn);
  const int bytes = sqlite3_column_bytes(statement, blobColumn);
  if (storedCount < 0 || storedCount > std::numeric_limits<int>::max() / static_cast<sqlite3_int64>(4 * sizeof(float))) {
    throw std::runtime_error("Invalid visual-point blob size");
  }
  const size_t count = static_cast<size_t>(storedCount);
  const int expectedBytes = static_cast<int>(count * 4 * sizeof(float));
  if (bytes != expectedBytes) {
    throw std::runtime_error("Invalid visual-point blob size");
  }
  const float *data = static_cast<const float *>(sqlite3_column_blob(statement, blobColumn));
  std::vector<VisualPoint> points;
  points.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    points.emplace_back(Eigen::Vector2f(data[i * 4], data[i * 4 + 1]), static_cast<int>(data[i * 4 + 2]), data[i * 4 + 3]);
  }
  return points;
}

inline void bindMatrix(sqlite3_stmt *statement, int index, const cv::Mat &matrix) {
  if (matrix.empty()) {
    sqlite3_bind_zeroblob(statement, index, 0);
    return;
  }
  const size_t bytes = matrix.total() * matrix.elemSize();
  if (bytes > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::runtime_error("Visual descriptor matrix is too large");
  }
  const cv::Mat continuous = matrix.isContinuous() ? matrix : matrix.clone();
  sqlite3_bind_blob(statement, index, continuous.data, static_cast<int>(bytes), SQLITE_TRANSIENT);
}

inline cv::Mat readMatrix(sqlite3_stmt *statement, int rowsColumn, int colsColumn, int typeColumn, int blobColumn) {
  const int rows = sqlite3_column_int(statement, rowsColumn);
  const int cols = sqlite3_column_int(statement, colsColumn);
  const int type = sqlite3_column_int(statement, typeColumn);
  const int bytes = sqlite3_column_bytes(statement, blobColumn);
  if (rows == 0 && cols == 0 && bytes == 0) {
    return {};
  }
  if (rows <= 0 || cols <= 0 || CV_MAT_CN(type) != 1 || (CV_MAT_DEPTH(type) != CV_8U && CV_MAT_DEPTH(type) != CV_32F)) {
    throw std::runtime_error("Invalid visual descriptor matrix metadata");
  }
  const size_t expectedBytes = static_cast<size_t>(rows) * static_cast<size_t>(cols) * CV_ELEM_SIZE(type);
  if (expectedBytes > static_cast<size_t>(std::numeric_limits<int>::max()) || bytes != static_cast<int>(expectedBytes)) {
    throw std::runtime_error("Invalid visual descriptor matrix blob size");
  }
  cv::Mat matrix(rows, cols, type);
  std::memcpy(matrix.data, sqlite3_column_blob(statement, blobColumn), static_cast<size_t>(bytes));
  return matrix;
}

}  // namespace database_detail

class MapDatabase {
 public:
  explicit MapDatabase(const std::string &path) : path_(path) {
    const int result = sqlite3_open_v2(path.c_str(), &db_, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX, nullptr);
    if (result != SQLITE_OK) {
      const std::string message = db_ ? sqlite3_errmsg(db_) : "Cannot allocate SQLite handle";
      if (db_) {
        sqlite3_close_v2(db_);
      }
      db_ = nullptr;
      throw std::runtime_error(message);
    }
    execute("PRAGMA journal_mode=WAL;");
    execute("PRAGMA synchronous=NORMAL;");
    execute("PRAGMA temp_store=MEMORY;");
    execute("PRAGMA foreign_keys=ON;");
    initializeSchema();
  }

  ~MapDatabase() {
    if (db_) {
      sqlite3_close_v2(db_);
    }
  }

  MapDatabase(const MapDatabase &) = delete;
  MapDatabase &operator=(const MapDatabase &) = delete;

  bool isOpen() const { return db_ != nullptr; }
  const std::string &path() const { return path_; }

  void saveSubmap(const SubmapFrame &submap, const LocalGrid &grid) {
    const int id = databaseId(submap.id());
    const LioFrame &lio = submap.lio();
    if (!lio.pcd) {
      throw std::invalid_argument("Cannot save a submap without a LiDAR cloud");
    }

    const std::vector<float> xyz = database_detail::packCloud(*lio.pcd);
    const std::vector<float> ground = database_detail::packGrid(grid.groundCells);
    const std::vector<float> obstacles = database_detail::packGrid(grid.obstacleCells);
    const std::vector<float> empty = database_detail::packGrid(grid.emptyCells);
    const Eigen::Matrix4f pose = lio.T_odom_base.matrix().cast<float>();
    std::ostringstream voxel_stream(std::ios::binary);
    cpu::save_voxelmaps(voxel_stream, submap.pyramid_voxels());
    const std::string voxel_data = voxel_stream.str();

    std::lock_guard<std::mutex> lock(mutex_);
    execute("BEGIN IMMEDIATE;");
    try {
      saveNode(id, lio.timestamp, pose);
      saveSpatialRecord(id, submap.bounds());
      saveNavigationRecord(id, submap.navigation());
      saveLaserRecord(id, lio.pcd->size(), xyz);
      savePyramidVoxel(id, voxel_data);
      saveFlatGrid(id, grid, ground, obstacles, empty);
      deleteVisualFrames(id);
      const std::vector<VisualFrame> &visual_frames = submap.visual_frames();
      for (std::size_t index = 0; index < visual_frames.size(); ++index) {
        saveVisualFrame(id, index, visual_frames[index]);
      }
      deleteMetaTags(id);
      for (const std::optional<MetaTag> &tag : submap.tags()) {
        if (tag) {
          saveMetaTag(id, tag.value());
        }
      }
      execute("COMMIT;");
    } catch (...) {
      try {
        execute("ROLLBACK;");
      } catch (...) {
      }
      throw;
    }
  }

  std::vector<VisualFrame> loadVisualFrames(int id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    database_detail::Statement statement(db_,
                                         "SELECT stamp,point_count,points,descriptors_rows,descriptors_cols,descriptors_type,descriptors"
                                         " FROM ImageRecord WHERE node_id=? ORDER BY frame_index;");
    sqlite3_bind_int(statement.get(), 1, id);
    std::vector<VisualFrame> frames;
    while (true) {
      const int result = sqlite3_step(statement.get());
      if (result == SQLITE_DONE) {
        break;
      }
      if (result != SQLITE_ROW) {
        throw std::runtime_error(sqlite3_errmsg(db_));
      }
      VisualFrame frame(sqlite3_column_double(statement.get(), 0));
      frame.update_features(database_detail::readVisualPoints(statement.get(), 1, 2), database_detail::readMatrix(statement.get(), 3, 4, 5, 6));
      frames.emplace_back(std::move(frame));
    }
    return frames;
  }

  NavigationPath loadNavigation(int id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    database_detail::Statement statement(db_,
                                         "SELECT sample_count,timestamps,samples,spline_degree,control_point_count,control_points,knot_count,knots"
                                         " FROM NavigationRecord WHERE node_id=?;");
    sqlite3_bind_int(statement.get(), 1, id);
    NavigationPath path;
    if (sqlite3_step(statement.get()) != SQLITE_ROW) {
      return path;
    }

    const sqlite3_int64 stored_sample_count = sqlite3_column_int64(statement.get(), 0);
    const sqlite3_int64 stored_control_count = sqlite3_column_int64(statement.get(), 4);
    const sqlite3_int64 stored_knot_count = sqlite3_column_int64(statement.get(), 6);
    if (stored_sample_count < 0 || stored_control_count < 0 || stored_knot_count < 0 ||
        stored_sample_count > std::numeric_limits<int>::max() / static_cast<sqlite3_int64>(8 * sizeof(float)) ||
        stored_control_count > std::numeric_limits<int>::max() / static_cast<sqlite3_int64>(3 * sizeof(float)) ||
        stored_knot_count > std::numeric_limits<int>::max() / static_cast<sqlite3_int64>(sizeof(float))) {
      throw std::runtime_error("Invalid navigation record");
    }

    const std::size_t sample_count = static_cast<std::size_t>(stored_sample_count);
    const std::size_t control_count = static_cast<std::size_t>(stored_control_count);
    const std::size_t knot_count = static_cast<std::size_t>(stored_knot_count);
    if (sqlite3_column_bytes(statement.get(), 1) != static_cast<int>(sample_count * sizeof(double)) ||
        sqlite3_column_bytes(statement.get(), 2) != static_cast<int>(sample_count * 8 * sizeof(float)) ||
        sqlite3_column_bytes(statement.get(), 5) != static_cast<int>(control_count * 3 * sizeof(float)) ||
        sqlite3_column_bytes(statement.get(), 7) != static_cast<int>(knot_count * sizeof(float))) {
      throw std::runtime_error("Invalid navigation record");
    }

    const auto *timestamps = static_cast<const double *>(sqlite3_column_blob(statement.get(), 1));
    const auto *samples = static_cast<const float *>(sqlite3_column_blob(statement.get(), 2));
    path.samples.reserve(sample_count);
    for (std::size_t index = 0; index < sample_count; ++index) {
      path.samples.push_back({timestamps[index], samples[index * 8], samples[index * 8 + 1], samples[index * 8 + 2], samples[index * 8 + 3],
                              samples[index * 8 + 4], samples[index * 8 + 5], samples[index * 8 + 6], samples[index * 8 + 7]});
    }

    path.spline.degree = sqlite3_column_int(statement.get(), 3);
    const auto *controls = static_cast<const float *>(sqlite3_column_blob(statement.get(), 5));
    path.spline.control_points.reserve(control_count);
    for (std::size_t index = 0; index < control_count; ++index) {
      path.spline.control_points.emplace_back(controls[index * 3], controls[index * 3 + 1], controls[index * 3 + 2]);
    }
    const auto *knots = static_cast<const float *>(sqlite3_column_blob(statement.get(), 7));
    if (knot_count != 0) {
      path.spline.knots.assign(knots, knots + knot_count);
    }
    if ((control_count != 0 || knot_count != 0) && !path.spline.valid()) {
      throw std::runtime_error("Invalid navigation spline");
    }
    return path;
  }

  template <typename Visitor>
  void visitSpatialRecords(Visitor &&visitor) const {
    std::lock_guard<std::mutex> lock(mutex_);
    database_detail::Statement statement(
        db_,
        "SELECT n.id,s.min_x,s.min_y,s.min_z,s.max_x,s.max_y,s.max_z,COALESCE(n.optimized_pose,n.odom_pose)"
        " FROM SpatialRecord s JOIN Node n ON n.id=s.node_id ORDER BY n.id;");
    while (true) {
      const int result = sqlite3_step(statement.get());
      if (result == SQLITE_DONE) {
        break;
      }
      if (result != SQLITE_ROW) {
        throw std::runtime_error(sqlite3_errmsg(db_));
      }
      const sqlite3_int64 node_id = sqlite3_column_int64(statement.get(), 0);
      if (node_id <= 0 || sqlite3_column_bytes(statement.get(), 7) != static_cast<int>(16 * sizeof(float))) {
        throw std::runtime_error("Invalid spatial record");
      }
      const std::uint64_t submap_id = static_cast<std::uint64_t>(node_id - 1);
      const AABB local_bounds{
          Eigen::Vector3f(static_cast<float>(sqlite3_column_double(statement.get(), 1)),
                          static_cast<float>(sqlite3_column_double(statement.get(), 2)),
                          static_cast<float>(sqlite3_column_double(statement.get(), 3))),
          Eigen::Vector3f(static_cast<float>(sqlite3_column_double(statement.get(), 4)),
                          static_cast<float>(sqlite3_column_double(statement.get(), 5)),
                          static_cast<float>(sqlite3_column_double(statement.get(), 6)))};
      Eigen::Matrix4f pose_matrix;
      std::memcpy(pose_matrix.data(), sqlite3_column_blob(statement.get(), 7), 16 * sizeof(float));
      Eigen::Isometry3f map_T_submap = Eigen::Isometry3f::Identity();
      map_T_submap.matrix() = pose_matrix;
      if (!local_bounds.valid() || !map_T_submap.matrix().allFinite()) {
        throw std::runtime_error("Invalid spatial record");
      }
      visitor(submap_id, local_bounds, map_T_submap);
    }
  }

  std::vector<std::pair<std::uint64_t, MetaTag>> loadMetaTags() const {
    std::lock_guard<std::mutex> lock(mutex_);
    database_detail::Statement statement(db_, "SELECT node_id,tag_type,tag_rows,tag_cols,tag_cv_type,tag FROM MetaTag;");
    std::vector<std::pair<std::uint64_t, MetaTag>> tags;
    while (true) {
      const int result = sqlite3_step(statement.get());
      if (result == SQLITE_DONE) {
        break;
      }
      if (result != SQLITE_ROW) {
        throw std::runtime_error(sqlite3_errmsg(db_));
      }
      const sqlite3_int64 nodeId = sqlite3_column_int64(statement.get(), 0);
      if (nodeId <= 0) {
        throw std::runtime_error("Invalid descriptor tag");
      }
      tags.emplace_back(static_cast<std::uint64_t>(nodeId - 1), MetaTag(static_cast<MetaTagType>(sqlite3_column_int(statement.get(), 1)),
                                                                        database_detail::readMatrix(statement.get(), 2, 3, 4, 5)));
    }
    return tags;
  }

  std::shared_ptr<vvec<float, 3>> loadCloud(int id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    database_detail::Statement statement(db_, "SELECT point_count,xyz FROM LaserRecord WHERE node_id=?;");
    sqlite3_bind_int(statement.get(), 1, id);
    auto cloud = std::make_shared<vvec<float, 3>>();
    if (sqlite3_step(statement.get()) != SQLITE_ROW) {
      return cloud;
    }
    const size_t count = static_cast<size_t>(sqlite3_column_int64(statement.get(), 0));
    if (sqlite3_column_bytes(statement.get(), 1) != static_cast<int>(count * 3 * sizeof(float))) {
      throw std::runtime_error("Invalid point-cloud blob size");
    }
    const float *xyz = static_cast<const float *>(sqlite3_column_blob(statement.get(), 1));
    cloud->resize(count);
    for (size_t i = 0; i < count; ++i) {
      (*cloud)[i] = Eigen::Vector3f(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]);
    }
    return cloud;
  }

  cpu::VoxelMapsData loadPyramidVoxel(int id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    database_detail::Statement statement(db_, "SELECT data FROM PyramidVoxel WHERE node_id=?;");
    sqlite3_bind_int(statement.get(), 1, id);
    if (sqlite3_step(statement.get()) != SQLITE_ROW) {
      throw std::runtime_error("Missing pyramid voxel");
    }
    const auto *data = static_cast<const char *>(sqlite3_column_blob(statement.get(), 0));
    const int size = sqlite3_column_bytes(statement.get(), 0);
    std::istringstream stream(std::string(data, size), std::ios::binary);
    return cpu::load_voxelmaps(stream);
  }

  bool loadLocalGrid(int id, LocalGrid &grid) const {
    std::lock_guard<std::mutex> lock(mutex_);
    database_detail::Statement statement(
        db_, "SELECT ground_count,ground,obstacle_count,obstacles,empty_count,empty,cell_size,view_x,view_y,view_z FROM FlatGrid WHERE node_id=?;");
    sqlite3_bind_int(statement.get(), 1, id);
    if (sqlite3_step(statement.get()) != SQLITE_ROW) {
      return false;
    }
    grid.groundCells = database_detail::readGrid(statement.get(), 0, 1);
    grid.obstacleCells = database_detail::readGrid(statement.get(), 2, 3);
    grid.emptyCells = database_detail::readGrid(statement.get(), 4, 5);
    grid.cellSize = static_cast<float>(sqlite3_column_double(statement.get(), 6));
    grid.viewPoint =
        Eigen::Vector3f(static_cast<float>(sqlite3_column_double(statement.get(), 7)), static_cast<float>(sqlite3_column_double(statement.get(), 8)),
                        static_cast<float>(sqlite3_column_double(statement.get(), 9)));
    return true;
  }

  void saveOptimizedPose(int id, const Eigen::Isometry3f &pose) { saveOptimizedPoses({{id, pose}}); }

  void saveOptimizedPoses(const std::vector<std::pair<int, Eigen::Isometry3f>> &poses) {
    if (poses.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    execute("BEGIN IMMEDIATE;");
    try {
      database_detail::Statement statement(db_, "UPDATE Node SET optimized_pose=? WHERE id=?;");
      for (const auto &entry : poses) {
        const Eigen::Matrix4f matrix = entry.second.matrix();
        sqlite3_reset(statement.get());
        sqlite3_clear_bindings(statement.get());
        sqlite3_bind_blob(statement.get(), 1, matrix.data(), sizeof(float) * 16, SQLITE_TRANSIENT);
        sqlite3_bind_int(statement.get(), 2, entry.first);
        database_detail::step(db_, statement.get());
        saveMetaTag(entry.first, MetaTag(entry.second));
      }
      execute("COMMIT;");
    } catch (...) {
      try {
        execute("ROLLBACK;");
      } catch (...) {
      }
      throw;
    }
  }

  void saveLink(int fromId, int toId, int type, const Eigen::Isometry3f &transform) {
    const Eigen::Matrix4f matrix = transform.matrix();
    std::lock_guard<std::mutex> lock(mutex_);
    database_detail::Statement statement(db_, "INSERT OR REPLACE INTO Link VALUES(?,?,?,?);");
    sqlite3_bind_int(statement.get(), 1, fromId);
    sqlite3_bind_int(statement.get(), 2, toId);
    sqlite3_bind_int(statement.get(), 3, type);
    sqlite3_bind_blob(statement.get(), 4, matrix.data(), sizeof(float) * 16, SQLITE_TRANSIENT);
    database_detail::step(db_, statement.get());
  }

 private:
  static int databaseId(std::uint64_t submap_id) {
    if (submap_id >= static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
      throw std::overflow_error("Submap ID exceeds SQLite integer range");
    }
    return static_cast<int>(submap_id) + 1;
  }

  void execute(const char *sql) const {
    char *error = nullptr;
    if (sqlite3_exec(db_, sql, nullptr, nullptr, &error) != SQLITE_OK) {
      const std::string message = error ? error : sqlite3_errmsg(db_);
      sqlite3_free(error);
      throw std::runtime_error(message);
    }
  }

  void initializeSchema() {
    execute(
        // 创建 Node 表：存储子图节点（位姿、时间戳等）
        "CREATE TABLE IF NOT EXISTS Node("
        "id INTEGER PRIMARY KEY, stamp REAL NOT NULL,"    // id 主键（由子图ID+1得到），stamp 时间戳（秒）
        "odom_pose BLOB NOT NULL, optimized_pose BLOB);"  // odom_pose 原始里程计位姿(4x4矩阵)，optimized_pose 优化后位姿（可空）

        // 创建 SpatialRecord 表：存储子图坐标系内不可变的点云边界
        "CREATE TABLE IF NOT EXISTS SpatialRecord("
        "node_id INTEGER PRIMARY KEY,min_x REAL NOT NULL,min_y REAL NOT NULL,min_z REAL NOT NULL,"
        "max_x REAL NOT NULL,max_y REAL NOT NULL,max_z REAL NOT NULL,"
        "FOREIGN KEY(node_id) REFERENCES Node(id));"

        // 创建 MetaTag 表：独立存储常驻热路径的召回标签
        "CREATE TABLE IF NOT EXISTS MetaTag("
        "node_id INTEGER NOT NULL, tag_type INTEGER NOT NULL,"  // 0:浮点描述子，1:二进制描述子，2:位姿描述子
        "tag_rows INTEGER NOT NULL, tag_cols INTEGER NOT NULL,"
        "tag_cv_type INTEGER NOT NULL, tag BLOB NOT NULL,"
        "PRIMARY KEY(node_id,tag_type),"
        "FOREIGN KEY(node_id) REFERENCES Node(id));"

        // 创建 NavigationRecord 表：存储一次子图构建期间的原始轨迹和局部三次B样条
        "CREATE TABLE IF NOT EXISTS NavigationRecord("
        "node_id INTEGER PRIMARY KEY,sample_count INTEGER NOT NULL,timestamps BLOB NOT NULL,samples BLOB NOT NULL,"
        "spline_degree INTEGER NOT NULL,control_point_count INTEGER NOT NULL,control_points BLOB NOT NULL,"
        "knot_count INTEGER NOT NULL,knots BLOB NOT NULL,"
        "FOREIGN KEY(node_id) REFERENCES Node(id));"

        // 创建 LaserRecord 表：存储点云数据
        "CREATE TABLE IF NOT EXISTS LaserRecord("
        "node_id INTEGER PRIMARY KEY, point_count INTEGER NOT NULL,"  // node_id 主键且外键关联 Node.id，point_count 点数
        "xyz BLOB NOT NULL,"                                          // xyz 点坐标数组（每个点3个float）
        "FOREIGN KEY(node_id) REFERENCES Node(id));"                  // 外键约束

        // 创建 ImageRecord 表：存储视觉数据
        "CREATE TABLE IF NOT EXISTS ImageRecord("
        "node_id INTEGER NOT NULL, frame_index INTEGER NOT NULL, stamp REAL NOT NULL,"
        "point_count INTEGER NOT NULL, points BLOB NOT NULL,"  // 特征点数量与数据（每个点4个float：x,y,level,response）
        "descriptors_rows INTEGER NOT NULL, descriptors_cols INTEGER NOT NULL,"  // 局部描述子矩阵行、列
        "descriptors_type INTEGER NOT NULL, descriptors BLOB NOT NULL,"          // 局部描述子类型（OpenCV类型）及数据
        "PRIMARY KEY(node_id,frame_index),"
        "FOREIGN KEY(node_id) REFERENCES Node(id));"  // 外键约束

        // 创建 PyramidVoxel 表：存储3D-BBS结构化地图数据
        "CREATE TABLE IF NOT EXISTS PyramidVoxel("         // node_id 主键且外键关联 Node.id
        "node_id INTEGER PRIMARY KEY,data BLOB NOT NULL,"  // data 3D-BBS结构化地图数据
        "FOREIGN KEY(node_id) REFERENCES Node(id));"       // 外键约束

        // 创建 FlatGrid 表：存储2D栅格地图数据
        "CREATE TABLE IF NOT EXISTS FlatGrid("                       // node_id 主键且外键关联 Node.id
        "node_id INTEGER PRIMARY KEY,"                               // node_id 主键且外键关联 Node.id
        "ground_count INTEGER NOT NULL, ground BLOB NOT NULL,"       // 地面栅格点数量与数据（每个点2个float）
        "obstacle_count INTEGER NOT NULL, obstacles BLOB NOT NULL,"  // 障碍物栅格点数量与数据
        "empty_count INTEGER NOT NULL, empty BLOB NOT NULL,"         // 空栅格点数量与数据
        "cell_size REAL NOT NULL, view_x REAL NOT NULL,"             // cell_size 栅格尺寸，view_x 观察点X坐标
        "view_y REAL NOT NULL, view_z REAL NOT NULL,"                // 观察点Y、Z坐标
        "FOREIGN KEY(node_id) REFERENCES Node(id));"                 // 外键约束

        // 创建 Link 表：存储子图之间的连接（位姿图边）
        "CREATE TABLE IF NOT EXISTS Link("
        "from_id INTEGER NOT NULL, to_id INTEGER NOT NULL,"  // 连接的两个节点ID
        "type INTEGER NOT NULL, transform BLOB NOT NULL,"    // 连接类型，相对变换矩阵(4x4 float)
        "PRIMARY KEY(from_id,to_id,type));"                  // 复合主键，确保唯一性
    );
  }

  void saveNode(int id, double stamp, const Eigen::Matrix4f &pose) {
    database_detail::Statement statement(
        db_, "INSERT INTO Node(id,stamp,odom_pose) VALUES(?,?,?) ON CONFLICT(id) DO UPDATE SET stamp=excluded.stamp,odom_pose=excluded.odom_pose;");
    sqlite3_bind_int(statement.get(), 1, id);
    sqlite3_bind_double(statement.get(), 2, stamp);
    sqlite3_bind_blob(statement.get(), 3, pose.data(), sizeof(float) * 16, SQLITE_TRANSIENT);
    database_detail::step(db_, statement.get());
  }

  void saveSpatialRecord(int id, const AABB &bounds) {
    if (!bounds.valid()) {
      throw std::invalid_argument("Cannot save invalid submap bounds");
    }
    database_detail::Statement statement(
        db_, "INSERT OR REPLACE INTO SpatialRecord(node_id,min_x,min_y,min_z,max_x,max_y,max_z) VALUES(?,?,?,?,?,?,?);");
    sqlite3_bind_int(statement.get(), 1, id);
    for (int axis = 0; axis < 3; ++axis) {
      sqlite3_bind_double(statement.get(), axis + 2, bounds.minimum[axis]);
      sqlite3_bind_double(statement.get(), axis + 5, bounds.maximum[axis]);
    }
    database_detail::step(db_, statement.get());
  }

  void saveNavigationRecord(int id, const NavigationPath &path) {
    const std::vector<double> timestamps = database_detail::packTrajectoryTimestamps(path.samples);
    const std::vector<float> samples = database_detail::packTrajectorySamples(path.samples);
    const std::vector<float> control_points = database_detail::packCloud(path.spline.control_points);
    database_detail::Statement statement(
        db_,
        "INSERT OR REPLACE INTO NavigationRecord(node_id,sample_count,timestamps,samples,spline_degree,control_point_count,"
        "control_points,knot_count,knots) VALUES(?,?,?,?,?,?,?,?,?);");
    sqlite3_bind_int(statement.get(), 1, id);
    sqlite3_bind_int64(statement.get(), 2, static_cast<sqlite3_int64>(path.samples.size()));
    database_detail::bindDoubles(statement.get(), 3, timestamps);
    database_detail::bindFloats(statement.get(), 4, samples);
    sqlite3_bind_int(statement.get(), 5, path.spline.degree);
    sqlite3_bind_int64(statement.get(), 6, static_cast<sqlite3_int64>(path.spline.control_points.size()));
    database_detail::bindFloats(statement.get(), 7, control_points);
    sqlite3_bind_int64(statement.get(), 8, static_cast<sqlite3_int64>(path.spline.knots.size()));
    database_detail::bindFloats(statement.get(), 9, path.spline.knots);
    database_detail::step(db_, statement.get());
  }

  void saveLaserRecord(int id, size_t pointCount, const std::vector<float> &xyz) {
    database_detail::Statement statement(db_, "INSERT OR REPLACE INTO LaserRecord(node_id,point_count,xyz) VALUES(?,?,?);");
    sqlite3_bind_int(statement.get(), 1, id);
    sqlite3_bind_int64(statement.get(), 2, static_cast<sqlite3_int64>(pointCount));
    database_detail::bindFloats(statement.get(), 3, xyz);
    database_detail::step(db_, statement.get());
  }

  void saveFlatGrid(int id, const LocalGrid &grid, const std::vector<float> &ground, const std::vector<float> &obstacles,
                    const std::vector<float> &empty) {
    database_detail::Statement statement(db_, "INSERT OR REPLACE INTO FlatGrid VALUES(?,?,?,?,?,?,?,?,?,?,?);");
    sqlite3_bind_int(statement.get(), 1, id);
    sqlite3_bind_int64(statement.get(), 2, static_cast<sqlite3_int64>(grid.groundCells.size()));
    database_detail::bindFloats(statement.get(), 3, ground);
    sqlite3_bind_int64(statement.get(), 4, static_cast<sqlite3_int64>(grid.obstacleCells.size()));
    database_detail::bindFloats(statement.get(), 5, obstacles);
    sqlite3_bind_int64(statement.get(), 6, static_cast<sqlite3_int64>(grid.emptyCells.size()));
    database_detail::bindFloats(statement.get(), 7, empty);
    sqlite3_bind_double(statement.get(), 8, grid.cellSize);
    sqlite3_bind_double(statement.get(), 9, grid.viewPoint.x());
    sqlite3_bind_double(statement.get(), 10, grid.viewPoint.y());
    sqlite3_bind_double(statement.get(), 11, grid.viewPoint.z());
    database_detail::step(db_, statement.get());
  }

  void savePyramidVoxel(int id, const std::string &data) {
    database_detail::Statement statement(db_, "INSERT OR REPLACE INTO PyramidVoxel(node_id,data) VALUES(?,?);");
    sqlite3_bind_int(statement.get(), 1, id);
    sqlite3_bind_blob(statement.get(), 2, data.data(), static_cast<int>(data.size()), SQLITE_STATIC);
    database_detail::step(db_, statement.get());
  }

  void saveVisualFrame(int id, std::size_t frame_index, const VisualFrame &frame) {
    const std::vector<float> points = database_detail::packVisualPoints(frame.points());
    database_detail::Statement statement(
        db_,
        "INSERT OR REPLACE INTO ImageRecord(node_id,frame_index,stamp,point_count,points,descriptors_rows,descriptors_cols,"
        "descriptors_type,descriptors) VALUES(?,?,?,?,?,?,?,?,?);");
    sqlite3_bind_int(statement.get(), 1, id);
    sqlite3_bind_int64(statement.get(), 2, static_cast<sqlite3_int64>(frame_index));
    sqlite3_bind_double(statement.get(), 3, frame.timestamp());
    sqlite3_bind_int64(statement.get(), 4, static_cast<sqlite3_int64>(frame.points().size()));
    database_detail::bindFloats(statement.get(), 5, points);
    sqlite3_bind_int(statement.get(), 6, frame.descriptors().rows);
    sqlite3_bind_int(statement.get(), 7, frame.descriptors().cols);
    sqlite3_bind_int(statement.get(), 8, frame.descriptors().empty() ? -1 : frame.descriptors().type());
    database_detail::bindMatrix(statement.get(), 9, frame.descriptors());
    database_detail::step(db_, statement.get());
  }

  void deleteVisualFrames(int id) {
    database_detail::Statement statement(db_, "DELETE FROM ImageRecord WHERE node_id=?;");
    sqlite3_bind_int(statement.get(), 1, id);
    database_detail::step(db_, statement.get());
  }

  void saveMetaTag(int id, const MetaTag &tag) {
    database_detail::Statement statement(db_,
                                         "INSERT OR REPLACE INTO MetaTag(node_id,tag_type,tag_rows,tag_cols,tag_cv_type,tag)"
                                         " VALUES(?,?,?,?,?,?);");
    sqlite3_bind_int(statement.get(), 1, id);
    sqlite3_bind_int(statement.get(), 2, static_cast<int>(tag.type()));
    sqlite3_bind_int(statement.get(), 3, tag.descriptor().rows);
    sqlite3_bind_int(statement.get(), 4, tag.descriptor().cols);
    sqlite3_bind_int(statement.get(), 5, tag.descriptor().type());
    database_detail::bindMatrix(statement.get(), 6, tag.descriptor());
    database_detail::step(db_, statement.get());
  }

  void deleteMetaTags(int id) {
    database_detail::Statement statement(db_, "DELETE FROM MetaTag WHERE node_id=?;");
    sqlite3_bind_int(statement.get(), 1, id);
    database_detail::step(db_, statement.get());
  }

  std::string path_;
  sqlite3 *db_ = nullptr;
  mutable std::mutex mutex_;
};

}  // namespace sapphire
