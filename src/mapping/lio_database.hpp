#pragma once
#include "local_gridmap.hpp"

#include <Eigen/Geometry>
#include <sqlite3.h>

#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <memory>

namespace database_detail {

class Statement
{
public:
  Statement(sqlite3 *database, const char *sql) : database_(database)
  {
    const int result = sqlite3_prepare_v2(database_, sql, -1, &statement_, nullptr);
    if(result != SQLITE_OK)
    {
      throw std::runtime_error(sqlite3_errmsg(database_));
    }
  }

  ~Statement() {sqlite3_finalize(statement_);}
  Statement(const Statement &) = delete;
  Statement &operator=(const Statement &) = delete;
  sqlite3_stmt *get() const {return statement_;}

private:
  sqlite3 *database_;
  sqlite3_stmt *statement_ = nullptr;
};

inline void step(sqlite3 *database, sqlite3_stmt *statement)
{
  if(sqlite3_step(statement) != SQLITE_DONE)
  {
    throw std::runtime_error(sqlite3_errmsg(database));
  }
}

inline std::vector<float> packCloud(const vvec<float, 3> &cloud)
{
  std::vector<float> data;
  data.reserve(cloud.size()*3);
  for(const Eigen::Vector3f &point : cloud)
  {
    data.push_back(point.x());
    data.push_back(point.y());
    data.push_back(point.z());
  }
  return data;
}

inline std::vector<float> packGrid(const GridPoints &points)
{
  std::vector<float> data;
  data.reserve(points.size()*2);
  for(const GridPoint &point : points)
  {
    data.push_back(point.x());
    data.push_back(point.y());
  }
  return data;
}

inline void bindFloats(sqlite3_stmt *statement, int index, const std::vector<float> &data)
{
  if(data.empty())
  {
    sqlite3_bind_zeroblob(statement, index, 0);
  }
  else
  {
    sqlite3_bind_blob(statement, index, data.data(), static_cast<int>(data.size()*sizeof(float)), SQLITE_TRANSIENT);
  }
}

inline GridPoints readGrid(sqlite3_stmt *statement, int countColumn, int blobColumn)
{
  const size_t count = static_cast<size_t>(sqlite3_column_int64(statement, countColumn));
  const int bytes = sqlite3_column_bytes(statement, blobColumn);
  if(bytes != static_cast<int>(count*2*sizeof(float)))
  {
    throw std::runtime_error("Invalid local-grid blob size");
  }
  const float *data = static_cast<const float *>(sqlite3_column_blob(statement, blobColumn));
  GridPoints points; points.reserve(count);
  for(size_t i = 0; i < count; ++i)
  {
    points.emplace_back(data[i*2], data[i*2+1]);
  }
  return points;
}

} // namespace database_detail

class LioDatabase
{
public:
  explicit LioDatabase(const std::string &path) : path_(path)
  {
    const int result = sqlite3_open_v2(path.c_str(), &db_, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX, nullptr);
    if(result != SQLITE_OK)
    {
      const std::string message = db_
        ? sqlite3_errmsg(db_)
        : "Cannot allocate SQLite handle";
      if(db_)
      {
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

  ~LioDatabase()
  {
    if(db_)
    {
      sqlite3_close_v2(db_);
    }
  }

  LioDatabase(const LioDatabase &) = delete;
  LioDatabase &operator=(const LioDatabase &) = delete;

  bool isOpen() const { return db_ != nullptr; }
  const std::string &path() const { return path_; }

  void saveKeyframe(int id, double stamp, const Eigen::Isometry3f &odomPose, const vvec<float, 3> &cloud, const LocalGrid &grid)
  {
    const std::vector<float> xyz = database_detail::packCloud(cloud);
    const std::vector<float> ground = database_detail::packGrid(grid.groundCells);
    const std::vector<float> obstacles = database_detail::packGrid(grid.obstacleCells);
    const std::vector<float> empty = database_detail::packGrid(grid.emptyCells);
    const Eigen::Matrix4f pose = odomPose.matrix();

    std::lock_guard<std::mutex> lock(mutex_);
    execute("BEGIN IMMEDIATE;");
    try
    {
      saveNode(id, stamp, pose);
      saveScan(id, cloud.size(), xyz);
      saveGrid(id, grid, ground, obstacles, empty);
      execute("COMMIT;");
    }
    catch(...)
    {
      try {execute("ROLLBACK;");} catch(...) {}
      throw;
    }
  }

  std::shared_ptr<vvec<float, 3>> loadCloud(int id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    database_detail::Statement statement(db_, "SELECT point_count,xyz FROM Scan WHERE node_id=?;");
    sqlite3_bind_int(statement.get(), 1, id);
    auto cloud = std::make_shared<vvec<float, 3>>();
    if(sqlite3_step(statement.get()) != SQLITE_ROW)
    {
      return cloud;
    }
    const size_t count = static_cast<size_t>(sqlite3_column_int64(statement.get(), 0));
    if(sqlite3_column_bytes(statement.get(), 1) != static_cast<int>(count*3*sizeof(float)))
    {
      throw std::runtime_error("Invalid point-cloud blob size");
    }
    const float *xyz = static_cast<const float *>(sqlite3_column_blob(statement.get(), 1));
    cloud->resize(count);
    for(size_t i = 0; i < count; ++i)
    {
      (*cloud)[i] = Eigen::Vector3f(xyz[i*3], xyz[i*3+1], xyz[i*3+2]);
    }
    return cloud;
  }

  bool loadLocalGrid(int id, LocalGrid &grid) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    database_detail::Statement statement(db_, "SELECT ground_count,ground,obstacle_count,obstacles,empty_count,empty,cell_size,view_x,view_y,view_z FROM LocalGrid WHERE node_id=?;");
    sqlite3_bind_int(statement.get(), 1, id);
    if(sqlite3_step(statement.get()) != SQLITE_ROW)
    {
      return false;
    }
    grid.groundCells = database_detail::readGrid(statement.get(), 0, 1);
    grid.obstacleCells = database_detail::readGrid(statement.get(), 2, 3);
    grid.emptyCells = database_detail::readGrid(statement.get(), 4, 5);
    grid.cellSize = static_cast<float>(sqlite3_column_double(statement.get(), 6));
    grid.viewPoint = Eigen::Vector3f(
      static_cast<float>(sqlite3_column_double(statement.get(), 7)),
      static_cast<float>(sqlite3_column_double(statement.get(), 8)),
      static_cast<float>(sqlite3_column_double(statement.get(), 9)));
    return true;
  }

  void saveOptimizedPose(int id, const Eigen::Isometry3f &pose)
  {
    saveOptimizedPoses({{id, pose}});
  }

  void saveOptimizedPoses(const std::vector<std::pair<int, Eigen::Isometry3f>> &poses)
  {
    if(poses.empty()) return;

    std::lock_guard<std::mutex> lock(mutex_);
    execute("BEGIN IMMEDIATE;");
    try
    {
      database_detail::Statement statement(db_, "UPDATE Node SET optimized_pose=? WHERE id=?;");
      for(const auto &entry : poses)
      {
        const Eigen::Matrix4f matrix = entry.second.matrix();
        sqlite3_reset(statement.get());
        sqlite3_clear_bindings(statement.get());
        sqlite3_bind_blob(statement.get(), 1, matrix.data(), sizeof(float)*16, SQLITE_TRANSIENT);
        sqlite3_bind_int(statement.get(), 2, entry.first);
        database_detail::step(db_, statement.get());
      }
      execute("COMMIT;");
    }
    catch(...)
    {
      try {execute("ROLLBACK;");} catch(...) {}
      throw;
    }
  }

  void saveLink(int fromId, int toId, int type, const Eigen::Isometry3f &transform)
  {
    const Eigen::Matrix4f matrix = transform.matrix();
    std::lock_guard<std::mutex> lock(mutex_);
    database_detail::Statement statement(db_, "INSERT OR REPLACE INTO Link VALUES(?,?,?,?);");
    sqlite3_bind_int(statement.get(), 1, fromId);
    sqlite3_bind_int(statement.get(), 2, toId);
    sqlite3_bind_int(statement.get(), 3, type);
    sqlite3_bind_blob(statement.get(), 4, matrix.data(), sizeof(float)*16, SQLITE_TRANSIENT);
    database_detail::step(db_, statement.get());
  }

private:
  void execute(const char *sql) const
  {
    char *error = nullptr;
    if(sqlite3_exec(db_, sql, nullptr, nullptr, &error) != SQLITE_OK)
    {
      const std::string message = error?error:sqlite3_errmsg(db_);
      sqlite3_free(error);
      throw std::runtime_error(message);
    }
  }

  void initializeSchema()
  {
    execute(
      "CREATE TABLE IF NOT EXISTS Node("
      "id INTEGER PRIMARY KEY, stamp REAL NOT NULL,"
      "odom_pose BLOB NOT NULL, optimized_pose BLOB);"
      "CREATE TABLE IF NOT EXISTS Scan("
      "node_id INTEGER PRIMARY KEY, point_count INTEGER NOT NULL,"
      "xyz BLOB NOT NULL,"
      "FOREIGN KEY(node_id) REFERENCES Node(id));"
      "CREATE TABLE IF NOT EXISTS LocalGrid("
      "node_id INTEGER PRIMARY KEY,"
      "ground_count INTEGER NOT NULL, ground BLOB NOT NULL,"
      "obstacle_count INTEGER NOT NULL, obstacles BLOB NOT NULL,"
      "empty_count INTEGER NOT NULL, empty BLOB NOT NULL,"
      "cell_size REAL NOT NULL, view_x REAL NOT NULL,"
      "view_y REAL NOT NULL, view_z REAL NOT NULL,"
      "FOREIGN KEY(node_id) REFERENCES Node(id));"
      "CREATE TABLE IF NOT EXISTS Link("
      "from_id INTEGER NOT NULL, to_id INTEGER NOT NULL,"
      "type INTEGER NOT NULL, transform BLOB NOT NULL,"
      "PRIMARY KEY(from_id,to_id,type));");
  }

  void saveNode(int id, double stamp, const Eigen::Matrix4f &pose)
  {
    database_detail::Statement statement(db_, "INSERT INTO Node(id,stamp,odom_pose) VALUES(?,?,?) ON CONFLICT(id) DO UPDATE SET stamp=excluded.stamp,odom_pose=excluded.odom_pose;");
    sqlite3_bind_int(statement.get(), 1, id);
    sqlite3_bind_double(statement.get(), 2, stamp);
    sqlite3_bind_blob(statement.get(), 3, pose.data(), sizeof(float)*16, SQLITE_TRANSIENT);
    database_detail::step(db_, statement.get());
  }

  void saveScan(int id, size_t pointCount, const std::vector<float> &xyz)
  {
    database_detail::Statement statement(db_, "INSERT OR REPLACE INTO Scan(node_id,point_count,xyz) VALUES(?,?,?);");
    sqlite3_bind_int(statement.get(), 1, id);
    sqlite3_bind_int64(statement.get(), 2, static_cast<sqlite3_int64>(pointCount));
    database_detail::bindFloats(statement.get(), 3, xyz);
    database_detail::step(db_, statement.get());
  }

  void saveGrid(int id, const LocalGrid &grid, const std::vector<float> &ground, const std::vector<float> &obstacles, const std::vector<float> &empty)
  {
    database_detail::Statement statement(db_, "INSERT OR REPLACE INTO LocalGrid VALUES(?,?,?,?,?,?,?,?,?,?,?);");
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

  std::string path_;
  sqlite3 *db_ = nullptr;
  mutable std::mutex mutex_;
};
