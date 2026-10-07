#pragma once
#include <fcntl.h>
#include <sqlite3.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#include <voxelmaps.hpp>

#include "common/aabb.hpp"
#include "common/key_frame.hpp"
#include "backend/storage/visual_observation_archive.hpp"
#include "backend/grid/local_gridmap.hpp"

namespace sapphire {

namespace database_detail {

// Weak no-op boundaries for focused W production-path fault/barrier tests; no runtime registration.
void writableStorageTestPoint(const char *point, void *context = nullptr);

// Bounds-check the current native pyramid encoding before its trusted-stream decoder allocates.
void validatePyramidPayload(const void *data, std::size_t size, std::size_t source_point_count);

// Local SQLite readers: check the stored class before SQLite can coerce it.
inline sqlite3_int64 readInteger(sqlite3_stmt *s, int column, sqlite3_int64 minimum = 0,
                                 sqlite3_int64 maximum = std::numeric_limits<sqlite3_int64>::max()) {
  if (sqlite3_column_type(s, column) != SQLITE_INTEGER) throw std::runtime_error("Historical field requires INTEGER storage");
  const auto value = sqlite3_column_int64(s, column);
  if (value < minimum || value > maximum) throw std::runtime_error("Historical INTEGER field out of range");
  return value;
}

inline double readReal(sqlite3_stmt *s, int column) {
  if (sqlite3_column_type(s, column) != SQLITE_FLOAT) throw std::runtime_error("Historical field requires REAL storage");
  const double value = sqlite3_column_double(s, column);
  if (!std::isfinite(value)) throw std::runtime_error("Nonfinite historical REAL field");
  return value;
}

inline float readFloat(sqlite3_stmt *s, int column) {
  const double value = readReal(s, column);
  if (std::abs(value) > std::numeric_limits<float>::max()) throw std::runtime_error("Historical REAL field exceeds float range");
  return static_cast<float>(value);
}

inline int blobBytes(sqlite3_stmt *s, int column) {
  if (sqlite3_column_type(s, column) != SQLITE_BLOB) throw std::runtime_error("Historical payload requires BLOB storage");
  const int bytes = sqlite3_column_bytes(s, column);
  if (bytes < 0 || (bytes > 0 && !sqlite3_column_blob(s, column))) throw std::runtime_error("Missing historical BLOB data");
  return bytes;
}

inline sqlite3_int64 readCount(sqlite3_stmt *s, int count_column, int blob_column, sqlite3_int64 element_bytes) {
  const auto count = readInteger(s, count_column, 0, std::numeric_limits<int>::max() / element_bytes);
  if (blobBytes(s, blob_column) != count * element_bytes) throw std::runtime_error("Historical count/BLOB length mismatch");
  return count;
}

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

inline std::vector<double> packOdomTimestamps(const OdomPoses &poses) {
  std::vector<double> timestamps;
  timestamps.reserve(poses.size());
  for (const OdomPose &pose : poses) {
    timestamps.push_back(pose.timestamp);
  }
  return timestamps;
}

inline std::vector<double> packOdomPoses(const OdomPoses &poses) {
  std::vector<double> data;
  data.reserve(poses.size() * 7);
  for (const OdomPose &pose : poses) {
    const Eigen::Vector3d &translation = pose.T_odom_base.translation();
    Eigen::Quaterniond orientation(pose.T_odom_base.rotation());
    orientation.normalize();
    data.push_back(translation.x());
    data.push_back(translation.y());
    data.push_back(translation.z());
    data.push_back(orientation.x());
    data.push_back(orientation.y());
    data.push_back(orientation.z());
    data.push_back(orientation.w());
  }
  return data;
}

inline GridPoints readGrid(sqlite3_stmt *statement, int countColumn, int blobColumn) {
  const sqlite3_int64 stored_count = readCount(statement, countColumn, blobColumn, 2 * sizeof(float));
  const size_t count = static_cast<size_t>(stored_count);
  const float *data = static_cast<const float *>(sqlite3_column_blob(statement, blobColumn));
  GridPoints points;
  points.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    points.emplace_back(data[i * 2], data[i * 2 + 1]);
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
  const auto rows = readInteger(statement, rowsColumn, 0, std::numeric_limits<int>::max());
  const auto cols = readInteger(statement, colsColumn, 0, std::numeric_limits<int>::max());
  const auto type = readInteger(statement, typeColumn, -1, CV_32FC1);
  const int bytes = blobBytes(statement, blobColumn);
  // saveVisualFrame writes -1 for its empty descriptor matrix; it is not a cv::Mat type.
  if (rows == 0 && cols == 0 && bytes == 0) {
    if (type != -1) throw std::runtime_error("Invalid empty descriptor matrix type");
    return {};
  }
  if (type != CV_8UC1 && type != CV_32FC1) throw std::runtime_error("Invalid matrix element type");
  const sqlite3_int64 element_bytes = type == CV_8UC1 ? 1 : sizeof(float);
  if (rows <= 0 || cols <= 0 || cols > std::numeric_limits<int>::max() / element_bytes ||
      rows > std::numeric_limits<int>::max() / (cols * element_bytes)) {
    throw std::runtime_error("Invalid visual descriptor matrix metadata");
  }
  const sqlite3_int64 expectedBytes = rows * cols * element_bytes;
  if (bytes != expectedBytes) {
    throw std::runtime_error("Invalid visual descriptor matrix blob size");
  }
  cv::Mat matrix(static_cast<int>(rows), static_cast<int>(cols), static_cast<int>(type));
  std::memcpy(matrix.data, sqlite3_column_blob(statement, blobColumn), static_cast<size_t>(bytes));
  return matrix;
}

constexpr std::array<std::uint8_t, 8> kGaussianPayloadMagic{{'S', 'A', 'P', 'H', 'G', 'A', 'U', 'S'}};
constexpr std::size_t kGaussianPayloadHeaderBytes = 8 + sizeof(std::uint64_t);
constexpr std::size_t kGaussianPayloadRecordBytes =
    3 * sizeof(std::int64_t) + 2 * sizeof(std::int32_t) + 3 * sizeof(float) + 6 * sizeof(float) + sizeof(float) + sizeof(std::uint8_t);
constexpr std::int32_t kMaximumGaussianLevel = 30;
static_assert(kGaussianPayloadRecordBytes == 73, "Gaussian payload record must be 73 bytes");

template <typename Unsigned>
inline void appendUnsignedLittleEndian(std::vector<std::uint8_t> &output, Unsigned value) {
  static_assert(std::is_unsigned<Unsigned>::value, "Unsigned integer required");
  for (std::size_t byte = 0; byte < sizeof(Unsigned); ++byte) {
    output.push_back(static_cast<std::uint8_t>((value >> (byte * 8U)) & static_cast<Unsigned>(0xffU)));
  }
}

template <typename Signed>
inline void appendSignedLittleEndian(std::vector<std::uint8_t> &output, Signed value) {
  static_assert(std::is_signed<Signed>::value, "Signed integer required");
  using Unsigned = typename std::make_unsigned<Signed>::type;
  appendUnsignedLittleEndian(output, static_cast<Unsigned>(value));
}

inline void appendFloatLittleEndian(std::vector<std::uint8_t> &output, float value) {
  static_assert(sizeof(float) == sizeof(std::uint32_t), "Gaussian payload requires 32-bit floats");
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  appendUnsignedLittleEndian(output, bits);
}

class PayloadReader {
 public:
  PayloadReader(const void *data, std::size_t size) : data_(static_cast<const std::uint8_t *>(data)), size_(size) {}

  template <typename Unsigned>
  Unsigned readUnsigned() {
    static_assert(std::is_unsigned<Unsigned>::value, "Unsigned integer required");
    require(sizeof(Unsigned));
    Unsigned value = 0;
    for (std::size_t byte = 0; byte < sizeof(Unsigned); ++byte) {
      value |= static_cast<Unsigned>(data_[offset_ + byte]) << (byte * 8U);
    }
    offset_ += sizeof(Unsigned);
    return value;
  }

  template <typename Signed>
  Signed readSigned() {
    static_assert(std::is_signed<Signed>::value, "Signed integer required");
    using Unsigned = typename std::make_unsigned<Signed>::type;
    const Unsigned bits = readUnsigned<Unsigned>();
    Signed value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  float readFloat() {
    const std::uint32_t bits = readUnsigned<std::uint32_t>();
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  std::uint8_t readByte() {
    require(1);
    return data_[offset_++];
  }

  std::size_t remaining() const { return size_ - offset_; }

 private:
  void require(std::size_t bytes) const {
    if (bytes > size_ - offset_) {
      throw std::runtime_error("Truncated Gaussian payload");
    }
  }

  const std::uint8_t *data_;
  std::size_t size_;
  std::size_t offset_ = 0;
};

inline bool isRegularizedGaussian(const GaussianPoint &point) {
  if (!point.valid() || point.voxel_key.level < 0 || point.voxel_key.level > kMaximumGaussianLevel) {
    return false;
  }
  const Eigen::Matrix3d covariance = point.covariance.cast<double>();
  const double covariance_scale = std::max(1.0, covariance.norm());
  if ((covariance - covariance.transpose()).norm() > 1e-5 * covariance_scale) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(0.5 * (covariance + covariance.transpose()));
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    return false;
  }
  const double largest = solver.eigenvalues().maxCoeff();
  const double minimum_regularized = std::max(1e-6, largest / 1e3);
  const double tolerance = 1e-5 * std::max(1.0, largest);
  if (largest < 1e-6 - tolerance || solver.eigenvalues().minCoeff() < minimum_regularized - tolerance) {
    return false;
  }
  const double expected_radius = 3.0 * std::sqrt(largest);
  return std::abs(static_cast<double>(point.radius) - expected_radius) <= 1e-5 * std::max(1.0, expected_radius);
}

inline std::vector<std::uint8_t> packGaussianCloud(const GaussianCloud &cloud) {
  if (cloud.size() > (std::numeric_limits<std::size_t>::max() - kGaussianPayloadHeaderBytes) / kGaussianPayloadRecordBytes) {
    throw std::overflow_error("Gaussian cloud is too large");
  }
  std::vector<std::uint8_t> payload;
  payload.reserve(kGaussianPayloadHeaderBytes + cloud.size() * kGaussianPayloadRecordBytes);
  payload.insert(payload.end(), kGaussianPayloadMagic.begin(), kGaussianPayloadMagic.end());
  appendUnsignedLittleEndian(payload, static_cast<std::uint64_t>(cloud.size()));
  for (const GaussianPoint &point : cloud) {
    if (!isRegularizedGaussian(point)) {
      throw std::invalid_argument("Cannot serialize an invalid or unregularized frozen Gaussian");
    }
    appendSignedLittleEndian(payload, static_cast<std::int64_t>(point.voxel_key.x));
    appendSignedLittleEndian(payload, static_cast<std::int64_t>(point.voxel_key.y));
    appendSignedLittleEndian(payload, static_cast<std::int64_t>(point.voxel_key.z));
    appendSignedLittleEndian(payload, static_cast<std::int32_t>(point.voxel_key.level));
    appendSignedLittleEndian(payload, static_cast<std::int32_t>(point.N));
    for (int axis = 0; axis < 3; ++axis) {
      appendFloatLittleEndian(payload, point.mean[axis]);
    }
    appendFloatLittleEndian(payload, point.covariance(0, 0));
    appendFloatLittleEndian(payload, point.covariance(0, 1));
    appendFloatLittleEndian(payload, point.covariance(0, 2));
    appendFloatLittleEndian(payload, point.covariance(1, 1));
    appendFloatLittleEndian(payload, point.covariance(1, 2));
    appendFloatLittleEndian(payload, point.covariance(2, 2));
    appendFloatLittleEndian(payload, point.radius);
    payload.push_back(point.is_plane ? 1U : 0U);
  }
  return payload;
}

inline GaussianCloud readGaussianCloud(sqlite3_stmt *statement, int countColumn, int blobColumn) {
  const sqlite3_int64 stored_count = readInteger(statement, countColumn);
  const int stored_bytes = blobBytes(statement, blobColumn);
  if (stored_count < 0 || stored_bytes < 0) {
    throw std::runtime_error("Invalid Gaussian record metadata");
  }
  const std::uint64_t database_count = static_cast<std::uint64_t>(stored_count);
  const std::size_t bytes = static_cast<std::size_t>(stored_bytes);
  const void *blob = sqlite3_column_blob(statement, blobColumn);
  if (bytes != 0 && blob == nullptr) {
    throw std::runtime_error("Missing Gaussian payload");
  }
  PayloadReader reader(blob, bytes);
  for (std::uint8_t expected : kGaussianPayloadMagic) {
    if (reader.readByte() != expected) {
      throw std::runtime_error("Invalid Gaussian payload magic");
    }
  }
  const std::uint64_t payload_count = reader.readUnsigned<std::uint64_t>();
  if (payload_count != database_count) {
    throw std::runtime_error("Gaussian database count does not match payload count");
  }
  if (payload_count > (std::numeric_limits<std::size_t>::max() - kGaussianPayloadHeaderBytes) / kGaussianPayloadRecordBytes ||
      bytes != kGaussianPayloadHeaderBytes + static_cast<std::size_t>(payload_count) * kGaussianPayloadRecordBytes) {
    throw std::runtime_error("Invalid Gaussian payload size");
  }

  GaussianCloud cloud;
  cloud.reserve(static_cast<std::size_t>(payload_count));
  for (std::uint64_t index = 0; index < payload_count; ++index) {
    GaussianPoint point;
    point.voxel_key.x = reader.readSigned<std::int64_t>();
    point.voxel_key.y = reader.readSigned<std::int64_t>();
    point.voxel_key.z = reader.readSigned<std::int64_t>();
    point.voxel_key.level = reader.readSigned<std::int32_t>();
    point.N = reader.readSigned<std::int32_t>();
    for (int axis = 0; axis < 3; ++axis) {
      point.mean[axis] = reader.readFloat();
    }
    point.covariance(0, 0) = reader.readFloat();
    point.covariance(0, 1) = point.covariance(1, 0) = reader.readFloat();
    point.covariance(0, 2) = point.covariance(2, 0) = reader.readFloat();
    point.covariance(1, 1) = reader.readFloat();
    point.covariance(1, 2) = point.covariance(2, 1) = reader.readFloat();
    point.covariance(2, 2) = reader.readFloat();
    point.radius = reader.readFloat();
    const std::uint8_t is_plane = reader.readByte();
    if (is_plane > 1U) {
      throw std::runtime_error("Invalid Gaussian is_plane value in payload");
    }
    point.is_plane = is_plane != 0U;
    if (!isRegularizedGaussian(point)) {
      throw std::runtime_error("Invalid or unregularized Gaussian in payload");
    }
    cloud.emplace_back(std::move(point));
  }
  if (reader.remaining() != 0) {
    throw std::runtime_error("Trailing bytes in Gaussian payload");
  }
  return cloud;
}

}  // namespace database_detail

struct GraphLink {
  int from_id = 0, to_id = 0, type = 0;
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
};

enum class MapErrorCode {
  DestinationExists,
  MissingMap,
  UnsupportedFormat,
  IncompatibleConfig,
  MalformedMetadata,
  UnsupportedMode,
  DuplicateNode,
  WriterConflict,
  StaleMap,
  ReadOnly,
  ResumeUnavailable,
  HistoricalData,
  UnsupportedSemantics,
  ReconstructionDiscrepancy,
  CorrectionUnavailable,
  Lifecycle,
  CommitAmbiguous,
  CommittedFailure,
  Storage
};

enum class CommitOutcome { NotCommitted, Committed, Unknown };

class MapError : public std::runtime_error {
 public:
  MapError(MapErrorCode code, const std::string &message, CommitOutcome outcome = CommitOutcome::NotCommitted)
      : std::runtime_error(message), code_(code), outcome_(outcome) {}
  CommitOutcome outcome() const noexcept { return outcome_; }
  MapErrorCode code() const noexcept { return code_; }

 private:
  MapErrorCode code_;
  CommitOutcome outcome_;
};

class MapDatabase {
 public:
  static constexpr int kSchemaVersion = 1;  // A1 storage eligibility; not a claim of graph restoration.
  static constexpr int kGeometrySchemaVersion = 2;
  static constexpr int kRetainedSceneSchemaVersion = 3;

  explicit MapDatabase(const std::string &path, const std::string &mode = "new",
                       std::string config_identity = map_config_identity(PoseGraphParameters{}, NaviMapParameters{}), float grid_resolution = .1f);
  ~MapDatabase() noexcept;

  enum class State { HistoricalReadOnly, Transitioning, Writable, Failed, Closed };
  State state() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return state_;
  }
  void promoteToWritable(float grid_resolution = .1f);
  void finish();  // Checked storage finish/close, not a publication/drain guarantee.

  std::uint64_t commitFinalizedSubmap(const SubmapFrame &submap, const LocalGrid &grid, const std::vector<std::uint8_t> &scene,
                                      const std::vector<std::pair<int, Eigen::Isometry3f>> &poses, const GraphLink &base,
                                      const std::optional<GraphLink> &loop, const std::string &expected_uuid, std::uint64_t expected_revision,
                                      int expected_next_id, const std::string &expected_config, int expected_chain_root);

  MapDatabase(const MapDatabase &) = delete;
  MapDatabase &operator=(const MapDatabase &) = delete;

  bool isOpen() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return db_ != nullptr;
  }
  std::uint64_t committedRevision() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return expected_revision_;
  }
  const std::string &path() const { return path_; }
  const std::string &mapUuid() const { return map_uuid_; }
  bool retainsScenes() const { return config_identity_.compare(0, 3, "c4:") == 0; }
  bool archivesNavigation() const { return config_identity_.compare(0, 3, "c3:") != 0 && !retainsScenes(); }
  bool sceneActive(int id) const;
  // Both observations must already be committed, with a verified stored loop.
  // nullopt keeps both; successful refresh returns its scene-only revision.
  std::optional<std::uint64_t> refreshCoveredScene(int old_id, int new_id);
  bool writable() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return state_ == State::Writable;
  }

  // A2 reads through this same A1-owned SQLite handle; never reopen the path.
  using NodeVisitor = std::function<void(int, const Eigen::Isometry3d &, const Eigen::Isometry3f &, const AABB &)>;
  void visitHistoricalNodes(const NodeVisitor &visitor) const;
  std::vector<GraphLink> loadGraphLinks() const;
  void validateHistoricalRecords(float grid_resolution) const;
  void validateReadIdentity() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    requireReadable();
    if (!writable_) checkNoJournal();
    checkFileIdentity();
    if (writable_) proveWritableIdentity();
  }

  void saveSubmap(const SubmapFrame &submap, const LocalGrid &grid, const std::vector<std::uint8_t> &visual_scene = {}) {
    requireWritable();
    if (route_s_only_) throw MapError(MapErrorCode::ReadOnly, "Resumed storage requires finalized-submap atomic commits");
    const int id = databaseId(submap.id());
    const LioFrame &lio = submap.lio();
    if (!lio.pcd) {
      throw std::invalid_argument("Cannot save a submap without a LiDAR cloud");
    }

    const std::vector<std::uint8_t> gaussian_payload = database_detail::packGaussianCloud(*lio.pcd);
    const std::vector<float> ground = database_detail::packGrid(grid.groundCells);
    const std::vector<float> obstacles = database_detail::packGrid(grid.obstacleCells);
    const std::vector<float> empty = database_detail::packGrid(grid.emptyCells);
    const std::vector<double> odom_timestamps = database_detail::packOdomTimestamps(submap.odom_poses());
    const std::vector<double> odom_poses = database_detail::packOdomPoses(submap.odom_poses());
    const Eigen::Matrix4f submap_pose = lio.T_odom_base.matrix().cast<float>();
    std::ostringstream voxel_stream(std::ios::binary);
    cpu::save_voxelmaps(voxel_stream, submap.pyramid_voxels());
    const std::string voxel_data = voxel_stream.str();

    std::lock_guard<std::recursive_mutex> lock(mutex_);
    requireReadable();
    validateWriter();
    execute("BEGIN IMMEDIATE;");
    try {
      validateWriter();
      saveNode(id, lio.timestamp, submap.odom_poses().size(), odom_timestamps, odom_poses, submap_pose);
      if (retainsScenes()) {
        database_detail::Statement state(db_, "INSERT INTO SceneState(node_id) VALUES(?);");
        sqlite3_bind_int(state.get(), 1, id); database_detail::step(db_, state.get());
      }
      saveSpatialRecord(id, submap.bounds());
      if (archivesNavigation()) saveNaviTrajectory(id, submap.navigation());
      saveLaserRecord(id, lio.pcd->size(), gaussian_payload);
      savePyramidVoxel(id, voxel_data);
      if (archivesNavigation()) saveFlatGrid(id, grid, ground, obstacles, empty);
      deleteVisualFrames(id);
      const std::vector<VisualFrame> &visual_frames = submap.visual_frames();
      for (std::size_t index = 0; index < visual_frames.size(); ++index) {
        saveVisualFrame(id, index, visual_frames[index]);
      }
      {
        database_detail::Statement remove(db_, "DELETE FROM VisualScene WHERE node_id=?;");
        sqlite3_bind_int(remove.get(), 1, id);
        database_detail::step(db_, remove.get());
        if (!visual_scene.empty()) saveVisualSceneUnlocked(id, visual_scene);
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

  std::vector<std::uint8_t> loadVisualScene(int id) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    requireReadable();
    database_detail::Statement statement(db_, "SELECT payload FROM VisualScene WHERE node_id=?;");
    sqlite3_bind_int(statement.get(), 1, id);
    const int status = sqlite3_step(statement.get());
    if (status == SQLITE_DONE) return {};
    if (status != SQLITE_ROW) throw std::runtime_error(sqlite3_errmsg(db_));
    const int size = database_detail::blobBytes(statement.get(), 0);
    const auto *data = static_cast<const std::uint8_t *>(sqlite3_column_blob(statement.get(), 0));
    if (!data || size <= 0 || size > 4 * 1024 * 1024) throw std::runtime_error("Invalid visual scene payload");
    return {data, data + size};
  }

  Eigen::Isometry3d loadOriginalAnchor(int id) const;  // keyed original odometry, never optimized pose

  std::vector<VisualFrame> loadVisualFrames(int id) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    requireReadable();
    database_detail::Statement statement(db_,
                                         "SELECT stamp,point_count,points,descriptors_rows,descriptors_cols,descriptors_type,descriptors,camera_id,"
                                         "node_id,frame_index"
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
      if (database_detail::readInteger(statement.get(), 8, 1, std::numeric_limits<int>::max()) != id ||
          database_detail::readInteger(statement.get(), 9, 0, std::numeric_limits<int>::max()) != static_cast<sqlite3_int64>(frames.size()))
        throw std::runtime_error("Invalid historical image identity/order");
      auto frame = database_detail::readVisualObservation(database_detail::readReal(statement.get(), 0),
          database_detail::readInteger(statement.get(), 7, 0, 1), database_detail::readInteger(statement.get(), 1),
          sqlite3_column_blob(statement.get(), 2), database_detail::blobBytes(statement.get(), 2),
          database_detail::readMatrix(statement.get(), 3, 4, 5, 6));
      frames.emplace_back(std::move(frame));
    }
    return frames;
  }

  NavigationPath loadNavigation(int id) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    requireReadable();
    if (!archivesNavigation()) return {};
    database_detail::Statement statement(db_,
                                         "SELECT sample_count,timestamps,samples,spline_degree,control_point_count,control_points,knot_count,knots"
                                         " FROM NaviTrajectory WHERE node_id=?;");
    sqlite3_bind_int(statement.get(), 1, id);
    NavigationPath path;
    if (sqlite3_step(statement.get()) != SQLITE_ROW) {
      return path;
    }

    const sqlite3_int64 stored_sample_count = database_detail::readCount(statement.get(), 0, 2, 8 * sizeof(float));
    (void)database_detail::readCount(statement.get(), 0, 1, sizeof(double));
    const sqlite3_int64 stored_control_count = database_detail::readCount(statement.get(), 4, 5, 3 * sizeof(float));
    const sqlite3_int64 stored_knot_count = database_detail::readCount(statement.get(), 6, 7, sizeof(float));
    const auto degree = database_detail::readInteger(statement.get(), 3, 0, 3);

    const std::size_t sample_count = static_cast<std::size_t>(stored_sample_count);
    const std::size_t control_count = static_cast<std::size_t>(stored_control_count);
    const std::size_t knot_count = static_cast<std::size_t>(stored_knot_count);
    const auto *timestamps = static_cast<const double *>(sqlite3_column_blob(statement.get(), 1));
    const auto *samples = static_cast<const float *>(sqlite3_column_blob(statement.get(), 2));
    path.samples.reserve(sample_count);
    for (std::size_t index = 0; index < sample_count; ++index) {
      path.samples.push_back({timestamps[index], samples[index * 8], samples[index * 8 + 1], samples[index * 8 + 2], samples[index * 8 + 3],
                              samples[index * 8 + 4], samples[index * 8 + 5], samples[index * 8 + 6], samples[index * 8 + 7]});
    }

    path.spline.degree = static_cast<int>(degree);
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
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    requireReadable();
    database_detail::Statement statement(db_,
                                         "SELECT n.id,s.min_x,s.min_y,s.min_z,s.max_x,s.max_y,s.max_z,n.submap_pose"
                                         " FROM SpatialRecord s JOIN Node n ON n.id=s.node_id ORDER BY n.id;");
    while (true) {
      const int result = sqlite3_step(statement.get());
      if (result == SQLITE_DONE) {
        break;
      }
      if (result != SQLITE_ROW) {
        throw std::runtime_error(sqlite3_errmsg(db_));
      }
      const sqlite3_int64 node_id = database_detail::readInteger(statement.get(), 0, 1, std::numeric_limits<int>::max());
      if (database_detail::blobBytes(statement.get(), 7) != static_cast<int>(16 * sizeof(float))) {
        throw std::runtime_error("Invalid spatial record");
      }
      const std::uint64_t submap_id = static_cast<std::uint64_t>(node_id - 1);
      const AABB local_bounds{Eigen::Vector3f(database_detail::readFloat(statement.get(), 1), database_detail::readFloat(statement.get(), 2),
                                              database_detail::readFloat(statement.get(), 3)),
                              Eigen::Vector3f(database_detail::readFloat(statement.get(), 4), database_detail::readFloat(statement.get(), 5),
                                              database_detail::readFloat(statement.get(), 6))};
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
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    requireReadable();
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
      const sqlite3_int64 nodeId = database_detail::readInteger(statement.get(), 0, 1, std::numeric_limits<int>::max());
      if (nodeId <= 0) {
        throw std::runtime_error("Invalid descriptor tag");
      }
      tags.emplace_back(static_cast<std::uint64_t>(nodeId - 1),
                        MetaTag(static_cast<MetaTagType>(database_detail::readInteger(statement.get(), 1, 0, 2)),
                                database_detail::readMatrix(statement.get(), 2, 3, 4, 5)));
    }
    return tags;
  }

  GaussianCloudPtr loadCloud(int id) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    requireReadable();
    database_detail::Statement statement(db_, "SELECT gaussian_count,payload FROM LaserRecord WHERE node_id=?;");
    sqlite3_bind_int(statement.get(), 1, id);
    if (sqlite3_step(statement.get()) != SQLITE_ROW) {
      throw std::runtime_error("Missing Gaussian cloud record");
    }
    return std::make_shared<const GaussianCloud>(database_detail::readGaussianCloud(statement.get(), 0, 1));
  }

  cpu::VoxelMapsData loadPyramidVoxel(int id) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    requireReadable();
    database_detail::Statement statement(db_,
                                         "SELECT p.data,l.gaussian_count,l.payload FROM PyramidVoxel p "
                                         "JOIN LaserRecord l ON l.node_id=p.node_id WHERE p.node_id=?;");
    sqlite3_bind_int(statement.get(), 1, id);
    if (sqlite3_step(statement.get()) != SQLITE_ROW) {
      throw std::runtime_error("Missing pyramid voxel");
    }
    const int size = database_detail::blobBytes(statement.get(), 0);
    // SubmapFrameBuffer builds every pyramid level from this persisted Gaussian cloud.
    // Bound the original voxel population using its checked, representable payload length.
    const auto points = database_detail::readInteger(
        statement.get(), 1, 0,
        (std::numeric_limits<int>::max() - database_detail::kGaussianPayloadHeaderBytes) / database_detail::kGaussianPayloadRecordBytes);
    if (database_detail::blobBytes(statement.get(), 2) !=
        database_detail::kGaussianPayloadHeaderBytes + points * database_detail::kGaussianPayloadRecordBytes)
      throw std::runtime_error("Invalid pyramid source-cloud length");
    const auto *data = static_cast<const char *>(sqlite3_column_blob(statement.get(), 0));
    database_detail::validatePyramidPayload(data, static_cast<std::size_t>(size), static_cast<std::size_t>(points));
    std::istringstream stream(std::string(data, size), std::ios::binary);
    auto result = cpu::load_voxelmaps(stream);
    if (stream.peek() != std::char_traits<char>::eof()) throw std::runtime_error("Trailing pyramid voxel payload");
    return result;
  }

  bool loadLocalGrid(int id, LocalGrid &grid) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    requireReadable();
    if (!archivesNavigation()) return false;
    database_detail::Statement statement(
        db_, "SELECT ground_count,ground,obstacle_count,obstacles,empty_count,empty,cell_size,view_x,view_y,view_z FROM FlatGrid WHERE node_id=?;");
    sqlite3_bind_int(statement.get(), 1, id);
    if (sqlite3_step(statement.get()) != SQLITE_ROW) {
      return false;
    }
    grid.groundCells = database_detail::readGrid(statement.get(), 0, 1);
    grid.obstacleCells = database_detail::readGrid(statement.get(), 2, 3);
    grid.emptyCells = database_detail::readGrid(statement.get(), 4, 5);
    grid.cellSize = database_detail::readFloat(statement.get(), 6);
    grid.viewPoint = Eigen::Vector3f(database_detail::readFloat(statement.get(), 7), database_detail::readFloat(statement.get(), 8),
                                     database_detail::readFloat(statement.get(), 9));
    return true;
  }

  void saveSubmapPose(int id, const Eigen::Isometry3f &pose) { saveSubmapPoses({{id, pose}}); }

  void saveSubmapPoses(const std::vector<std::pair<int, Eigen::Isometry3f>> &poses, const std::vector<GraphLink> &links = {}) {
    requireWritable();
    if (route_s_only_) throw MapError(MapErrorCode::ReadOnly, "Resumed storage requires finalized-submap atomic commits");
    if (poses.empty() && links.empty()) {
      return;
    }

    std::lock_guard<std::recursive_mutex> lock(mutex_);
    requireReadable();
    validateWriter();
    execute("BEGIN IMMEDIATE;");
    try {
      validateWriter();
      if (expected_revision_ == std::numeric_limits<sqlite3_int64>::max()) throw MapError(MapErrorCode::Storage, "Map revision exhausted");
      database_detail::Statement statement(db_, "UPDATE Node SET submap_pose=? WHERE id=?;");
      for (const auto &entry : poses) {
        if (entry.first <= 0 || !entry.second.matrix().allFinite()) throw std::invalid_argument("Invalid graph pose");
        const Eigen::Matrix4f matrix = entry.second.matrix();
        sqlite3_reset(statement.get());
        sqlite3_clear_bindings(statement.get());
        sqlite3_bind_blob(statement.get(), 1, matrix.data(), sizeof(float) * 16, SQLITE_TRANSIENT);
        sqlite3_bind_int(statement.get(), 2, entry.first);
        database_detail::step(db_, statement.get());
        if (sqlite3_changes(db_) != 1) throw std::runtime_error("Graph pose references missing node");
        saveMetaTag(entry.first, MetaTag(entry.second));
      }
      for (const auto &link : links) saveLinkUnlocked(link.from_id, link.to_id, link.type, link.transform);
      execute("UPDATE MapState SET graph_revision=graph_revision+1 WHERE id=1;");
      execute("COMMIT;");
      ++expected_revision_;
    } catch (...) {
      try {
        execute("ROLLBACK;");
      } catch (...) {
      }
      throw;
    }
  }

  void saveLink(int fromId, int toId, int type, const Eigen::Isometry3f &transform) { saveSubmapPoses({}, {{fromId, toId, type, transform}}); }
  std::uint64_t graphRevision() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    requireReadable();
    database_detail::Statement state(db_, "SELECT graph_revision FROM MapState WHERE id=1;");
    if (sqlite3_step(state.get()) != SQLITE_ROW) throw std::runtime_error("Missing map state");
    return static_cast<std::uint64_t>(database_detail::readInteger(state.get(), 0));
  }

 private:
  void requireWritable() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (state_ == State::Failed || state_ == State::Closed) throw MapError(MapErrorCode::Lifecycle, "Map storage is failed or closed");
    if (state_ != State::Writable || !writable_) throw MapError(MapErrorCode::ReadOnly, "Historical map writes are prohibited");
  }

  enum class Admission { None, SharedSlot, ExclusiveSlot };
  void reserveAdmission(bool writer);
  void releaseAdmission() noexcept;
  void closeHandlesChecked();
  void closeHandles() noexcept;
  void openWritableConnection();
  void proveWritableIdentity() const;
  static struct stat observeMainFile(sqlite3 *db);
  void setJournalMode(const char *mode);
  void requireReadable() const {
    if (!db_ || state_ == State::Failed || state_ == State::Closed)
      throw MapError(MapErrorCode::Lifecycle, "Map storage is failed or closed; reconstruct from DB before retry");
  }
  friend struct MapDatabaseTestAccess;

  void checkFileIdentity() const {
    struct stat current {};
    if (::lstat(path_.c_str(), &current) != 0 || !S_ISREG(current.st_mode) || current.st_dev != file_identity_.st_dev ||
        current.st_ino != file_identity_.st_ino)
      throw MapError(MapErrorCode::StaleMap, "Map path no longer names the validated file");
  }

  void checkNoJournal() const {
    for (const char *suffix : {"-wal", "-shm", "-journal"}) {
      struct stat info {};
      const std::string sidecar = path_ + suffix;
      if (::lstat(sidecar.c_str(), &info) == 0)
        throw MapError(MapErrorCode::WriterConflict, "A1 preflight requires a closed, checkpointed map: " + sidecar);
      if (errno != ENOENT) throw MapError(MapErrorCode::Storage, "Cannot inspect map journal: " + sidecar);
    }
  }

  static std::string makeUuid() {
    std::array<unsigned char, 16> bytes{};
    sqlite3_randomness(static_cast<int>(bytes.size()), bytes.data());
    bytes[6] = (bytes[6] & 15) | 64;
    bytes[8] = (bytes[8] & 63) | 128;
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < bytes.size(); ++i) {
      if (i == 4 || i == 6 || i == 8 || i == 10) out << '-';
      out << std::setw(2) << static_cast<unsigned>(bytes[i]);
    }
    return out.str();
  }

  static bool isHex(char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); }
  static bool validUuid(const std::string &uuid) {
    if (uuid.size() != 36 || uuid[14] != '4' || std::string("89ab").find(uuid[19]) == std::string::npos) return false;
    for (std::size_t i = 0; i < uuid.size(); ++i) {
      if (i == 8 || i == 13 || i == 18 || i == 23) {
        if (uuid[i] != '-') return false;
      } else if (!isHex(uuid[i]))
        return false;
    }
    return true;
  }
  static bool validConfigIdentity(const std::string &identity) {
    // Recognize the old encoding structurally so normal c2 callers reject it as
    // incompatible rather than silently assigning it the new field semantics.
    return identity.size() == 19 && (identity.substr(0, 3) == "c2:" || identity.substr(0, 3) == "c3:" || identity.substr(0, 3) == "c4:" || identity.substr(0, 3) == "a1:") &&
           std::all_of(identity.begin() + 3, identity.end(), isHex);
  }

  void validateSchema() const {
    try {
      database_detail::Statement version(db_, "PRAGMA user_version;");
      if (sqlite3_step(version.get()) != SQLITE_ROW ||
          sqlite3_column_int(version.get(), 0) != (archivesNavigation() ? kSchemaVersion : (retainsScenes() ? kRetainedSceneSchemaVersion : kGeometrySchemaVersion)))
        throw MapError(MapErrorCode::UnsupportedFormat, "Unsupported or legacy map schema version");
      database_detail::Statement tables(db_,
                                        "SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name IN "
                                        "('Node','SpatialRecord','MetaTag','NaviTrajectory','LaserRecord','ImageRecord','VisualScene','PyramidVoxel',"
                                        "'FlatGrid','Link','MapState','SceneState');");
      if (sqlite3_step(tables.get()) != SQLITE_ROW || sqlite3_column_int(tables.get(), 0) != (archivesNavigation() ? 11 : (retainsScenes() ? 10 : 9)))
        throw MapError(MapErrorCode::UnsupportedFormat, "Missing required map tables");
      // Inspect columns only. This does not decode records or reconstruct mapping state.
      for (const char *query :
           {"SELECT id,stamp,odom_pose_count,odom_timestamps,odom_poses,submap_pose FROM Node LIMIT 0;",
            "SELECT node_id,min_x,min_y,min_z,max_x,max_y,max_z FROM SpatialRecord LIMIT 0;",
            "SELECT node_id,tag_type,tag_rows,tag_cols,tag_cv_type,tag FROM MetaTag LIMIT 0;",
            "SELECT node_id,sample_count,timestamps,samples,spline_degree,control_point_count,control_points,knot_count,knots FROM NaviTrajectory "
            "LIMIT 0;",
            "SELECT node_id,gaussian_count,payload FROM LaserRecord LIMIT 0;",
            "SELECT node_id,frame_index,stamp,point_count,points,descriptors_rows,descriptors_cols,descriptors_type,descriptors,camera_id FROM "
            "ImageRecord LIMIT 0;",
            "SELECT node_id,payload FROM VisualScene LIMIT 0;", "SELECT node_id,data FROM PyramidVoxel LIMIT 0;",
            "SELECT node_id,ground_count,ground,obstacle_count,obstacles,empty_count,empty,cell_size,view_x,view_y,view_z FROM FlatGrid LIMIT 0;",
            "SELECT from_id,to_id,type,transform FROM Link LIMIT 0;", "SELECT id,graph_revision,map_uuid,config_identity FROM MapState LIMIT 0;"}) {
        if (!archivesNavigation() && (std::strstr(query, "FROM FlatGrid") || std::strstr(query, "FROM NaviTrajectory"))) continue;
        database_detail::Statement columns(db_, query);
      }
      if (retainsScenes()) {
        database_detail::Statement columns(db_, "SELECT node_id,retired_by,retired_revision FROM SceneState LIMIT 0;");
      }
    } catch (const MapError &) {
      throw;
    } catch (const std::exception &error) {
      throw MapError(MapErrorCode::UnsupportedFormat, error.what());
    }
  }

  void readIdentity(bool initial) {
    database_detail::Statement state(db_, "SELECT id,graph_revision,map_uuid,config_identity FROM MapState;");
    if (sqlite3_step(state.get()) != SQLITE_ROW || sqlite3_column_type(state.get(), 0) != SQLITE_INTEGER ||
        sqlite3_column_int64(state.get(), 0) != 1 || sqlite3_column_type(state.get(), 1) != SQLITE_INTEGER ||
        sqlite3_column_int64(state.get(), 1) < 0 || sqlite3_column_type(state.get(), 2) != SQLITE_TEXT ||
        sqlite3_column_type(state.get(), 3) != SQLITE_TEXT || sqlite3_column_bytes(state.get(), 2) != 36 ||
        sqlite3_column_bytes(state.get(), 3) != 19)
      throw MapError(MapErrorCode::MalformedMetadata, "Missing or malformed MapState identity/revision");
    const auto revision = sqlite3_column_int64(state.get(), 1);
    const std::string uuid(reinterpret_cast<const char *>(sqlite3_column_text(state.get(), 2)), sqlite3_column_bytes(state.get(), 2));
    const std::string config(reinterpret_cast<const char *>(sqlite3_column_text(state.get(), 3)), sqlite3_column_bytes(state.get(), 3));
    if (!validUuid(uuid) || !validConfigIdentity(config) || sqlite3_step(state.get()) != SQLITE_DONE)
      throw MapError(MapErrorCode::MalformedMetadata, "Malformed or non-unique map identity");
    if (config != config_identity_)
      throw MapError(initial ? MapErrorCode::IncompatibleConfig : MapErrorCode::StaleMap, "Map configuration identity differs");
    if (initial) {
      map_uuid_ = uuid;
      expected_revision_ = revision;
    } else if (uuid != map_uuid_ || revision != expected_revision_)
      throw MapError(MapErrorCode::StaleMap, "Map identity or revision changed outside this writer");
  }

  void validateWriter() {
    requireWritable();
    proveWritableIdentity();
    checkFileIdentity();
    validateSchema();
    readIdentity(false);
  }

  void saveLinkUnlocked(int fromId, int toId, int type, const Eigen::Isometry3f &transform) {
    if (fromId <= 0 || toId <= 0 || !transform.matrix().allFinite()) throw std::invalid_argument("Invalid graph link");
    database_detail::Statement nodes(db_, "SELECT COUNT(*) FROM Node WHERE id IN (?,?);");
    sqlite3_bind_int(nodes.get(), 1, fromId);
    sqlite3_bind_int(nodes.get(), 2, toId);
    if (sqlite3_step(nodes.get()) != SQLITE_ROW || sqlite3_column_int(nodes.get(), 0) != (fromId == toId ? 1 : 2))
      throw std::runtime_error("Graph link references missing node");
    const Eigen::Matrix4f matrix = transform.matrix();
    database_detail::Statement statement(db_, "INSERT OR REPLACE INTO Link VALUES(?,?,?,?);");
    sqlite3_bind_int(statement.get(), 1, fromId);
    sqlite3_bind_int(statement.get(), 2, toId);
    sqlite3_bind_int(statement.get(), 3, type);
    sqlite3_bind_blob(statement.get(), 4, matrix.data(), sizeof(float) * 16, SQLITE_TRANSIENT);
    database_detail::step(db_, statement.get());
  }

  void saveVisualSceneUnlocked(int id, const std::vector<std::uint8_t> &payload) {
    if (payload.empty() || payload.size() > 4 * 1024 * 1024) throw std::invalid_argument("Invalid visual scene size");
    database_detail::Statement statement(db_, "INSERT OR REPLACE INTO VisualScene(node_id,payload) VALUES(?,?);");
    sqlite3_bind_int(statement.get(), 1, id);
    if (sqlite3_bind_blob(statement.get(), 2, payload.data(), static_cast<int>(payload.size()), SQLITE_TRANSIENT) != SQLITE_OK)
      throw std::runtime_error(sqlite3_errmsg(db_));
    database_detail::step(db_, statement.get());
  }

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
      const int code = sqlite3_errcode(db_);
      throw MapError(code == SQLITE_BUSY || code == SQLITE_LOCKED ? MapErrorCode::WriterConflict : MapErrorCode::Storage, message);
    }
  }

  void createSchema() {
    execute(
        // 创建 Node 表：存储子图节点（位姿、时间戳等）
        "CREATE TABLE IF NOT EXISTS Node("
        "id INTEGER PRIMARY KEY,stamp REAL NOT NULL,"
        "odom_pose_count INTEGER NOT NULL,odom_timestamps BLOB NOT NULL,odom_poses BLOB NOT NULL,"
        "submap_pose BLOB NOT NULL);"

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

        // 创建 NaviTrajectory 表：存储独立的导航 samples 和局部三次 B 样条
        "CREATE TABLE IF NOT EXISTS NaviTrajectory("
        "node_id INTEGER PRIMARY KEY,sample_count INTEGER NOT NULL,timestamps BLOB NOT NULL,samples BLOB NOT NULL,"
        "spline_degree INTEGER NOT NULL,control_point_count INTEGER NOT NULL,control_points BLOB NOT NULL,"
        "knot_count INTEGER NOT NULL,knots BLOB NOT NULL,"
        "FOREIGN KEY(node_id) REFERENCES Node(id));"

        // 创建 LaserRecord 表：存储固定字段编码的高斯云
        "CREATE TABLE IF NOT EXISTS LaserRecord("
        "node_id INTEGER PRIMARY KEY, gaussian_count INTEGER NOT NULL,"
        "payload BLOB NOT NULL,"
        "FOREIGN KEY(node_id) REFERENCES Node(id));"

        // 创建 ImageRecord 表：存储视觉数据
        "CREATE TABLE IF NOT EXISTS ImageRecord("
        "node_id INTEGER NOT NULL, frame_index INTEGER NOT NULL, stamp REAL NOT NULL,"
        "point_count INTEGER NOT NULL, points BLOB NOT NULL,"  // 特征点数量与数据（每个点4个float：x,y,level,response）
        "descriptors_rows INTEGER NOT NULL, descriptors_cols INTEGER NOT NULL,"  // 局部描述子矩阵行、列
        "descriptors_type INTEGER NOT NULL, descriptors BLOB NOT NULL,"          // 局部描述子类型（OpenCV类型）及数据
        "PRIMARY KEY(node_id,frame_index),"
        "FOREIGN KEY(node_id) REFERENCES Node(id));"  // 外键约束

        "CREATE TABLE IF NOT EXISTS VisualScene("
        "node_id INTEGER PRIMARY KEY,payload BLOB NOT NULL,"
        "FOREIGN KEY(node_id) REFERENCES Node(id));"

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
    execute(
        "CREATE TABLE MapState(id INTEGER PRIMARY KEY CHECK(id=1),graph_revision INTEGER NOT NULL,"
        "map_uuid TEXT NOT NULL,config_identity TEXT NOT NULL);");
    // Only reached for exclusive creation of a new database, never migration.
    if (!archivesNavigation()) execute("DROP TABLE FlatGrid; DROP TABLE NaviTrajectory;");
    if (retainsScenes()) execute("CREATE TABLE SceneState(node_id INTEGER PRIMARY KEY REFERENCES Node(id),"
        "retired_by INTEGER REFERENCES Node(id),retired_revision INTEGER,"
        "CHECK((retired_by IS NULL AND retired_revision IS NULL) OR "
        "(retired_by IS NOT NULL AND retired_revision IS NOT NULL AND retired_by>node_id AND retired_revision>0)));");
    {
      database_detail::Statement state(db_, "INSERT INTO MapState VALUES(1,0,?,?);");
      sqlite3_bind_text(state.get(), 1, map_uuid_.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(state.get(), 2, config_identity_.c_str(), -1, SQLITE_TRANSIENT);
      database_detail::step(db_, state.get());
    }
    bool has_camera_id = false;
    {
      database_detail::Statement columns(db_, "PRAGMA table_info(ImageRecord);");
      while (sqlite3_step(columns.get()) == SQLITE_ROW)
        if (std::string(reinterpret_cast<const char *>(sqlite3_column_text(columns.get(), 1))) == "camera_id") has_camera_id = true;
    }
    if (!has_camera_id) execute("ALTER TABLE ImageRecord ADD COLUMN camera_id INTEGER NOT NULL DEFAULT 0;");
  }

  void saveNode(int id, double stamp, std::size_t odom_pose_count, const std::vector<double> &odom_timestamps, const std::vector<double> &odom_poses,
                const Eigen::Matrix4f &submap_pose) {
    database_detail::Statement existing(db_, "SELECT 1 FROM Node WHERE id=?;");
    sqlite3_bind_int(existing.get(), 1, id);
    if (sqlite3_step(existing.get()) == SQLITE_ROW)
      throw MapError(MapErrorCode::DuplicateNode, "Node identity already exists: " + std::to_string(id));
    database_detail::Statement statement(db_,
                                         "INSERT INTO Node(id,stamp,odom_pose_count,odom_timestamps,odom_poses,submap_pose) VALUES(?,?,?,?,?,?);");
    sqlite3_bind_int(statement.get(), 1, id);
    sqlite3_bind_double(statement.get(), 2, stamp);
    sqlite3_bind_int64(statement.get(), 3, static_cast<sqlite3_int64>(odom_pose_count));
    database_detail::bindDoubles(statement.get(), 4, odom_timestamps);
    database_detail::bindDoubles(statement.get(), 5, odom_poses);
    sqlite3_bind_blob(statement.get(), 6, submap_pose.data(), sizeof(float) * 16, SQLITE_TRANSIENT);
    database_detail::step(db_, statement.get());
  }

  void saveSpatialRecord(int id, const AABB &bounds) {
    if (!bounds.valid()) {
      throw std::invalid_argument("Cannot save invalid submap bounds");
    }
    database_detail::Statement statement(db_,
                                         "INSERT OR REPLACE INTO SpatialRecord(node_id,min_x,min_y,min_z,max_x,max_y,max_z) VALUES(?,?,?,?,?,?,?);");
    sqlite3_bind_int(statement.get(), 1, id);
    for (int axis = 0; axis < 3; ++axis) {
      sqlite3_bind_double(statement.get(), axis + 2, bounds.minimum[axis]);
      sqlite3_bind_double(statement.get(), axis + 5, bounds.maximum[axis]);
    }
    database_detail::step(db_, statement.get());
  }

  void saveNaviTrajectory(int id, const NavigationPath &path) {
    const std::vector<double> timestamps = database_detail::packTrajectoryTimestamps(path.samples);
    const std::vector<float> samples = database_detail::packTrajectorySamples(path.samples);
    const std::vector<float> control_points = database_detail::packCloud(path.spline.control_points);
    database_detail::Statement statement(
        db_,
        "INSERT OR REPLACE INTO NaviTrajectory(node_id,sample_count,timestamps,samples,spline_degree,control_point_count,"
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

  void saveLaserRecord(int id, size_t gaussian_count, const std::vector<std::uint8_t> &payload) {
    if (payload.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
      throw std::overflow_error("Gaussian payload exceeds SQLite blob limit");
    }
    database_detail::Statement statement(db_, "INSERT OR REPLACE INTO LaserRecord(node_id,gaussian_count,payload) VALUES(?,?,?);");
    sqlite3_bind_int(statement.get(), 1, id);
    sqlite3_bind_int64(statement.get(), 2, static_cast<sqlite3_int64>(gaussian_count));
    sqlite3_bind_blob(statement.get(), 3, payload.data(), static_cast<int>(payload.size()), SQLITE_TRANSIENT);
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
    const auto points = database_detail::packVisualObservation(frame);
    database_detail::Statement statement(
        db_,
        "INSERT OR REPLACE INTO ImageRecord(node_id,frame_index,stamp,point_count,points,descriptors_rows,descriptors_cols,"
        "descriptors_type,descriptors,camera_id) VALUES(?,?,?,?,?,?,?,?,?,?);");
    sqlite3_bind_int(statement.get(), 1, id);
    sqlite3_bind_int64(statement.get(), 2, static_cast<sqlite3_int64>(frame_index));
    sqlite3_bind_double(statement.get(), 3, frame.timestamp());
    sqlite3_bind_int64(statement.get(), 4, static_cast<sqlite3_int64>(frame.points().size()));
    if (points.empty()) sqlite3_bind_zeroblob(statement.get(), 5, 0);
    else sqlite3_bind_blob(statement.get(), 5, points.data(), static_cast<int>(points.size()), SQLITE_TRANSIENT);
    sqlite3_bind_int(statement.get(), 6, frame.descriptors().rows);
    sqlite3_bind_int(statement.get(), 7, frame.descriptors().cols);
    sqlite3_bind_int(statement.get(), 8, frame.descriptors().empty() ? -1 : frame.descriptors().type());
    database_detail::bindMatrix(statement.get(), 9, frame.descriptors());
    sqlite3_bind_int(statement.get(), 10, static_cast<int>(frame.camera_id()));
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
  std::string config_identity_, map_uuid_;
  float occupancy_resolution_ = .1f;  // Runtime configuration, never a new persisted fact.
  bool writable_ = false;
  int file_fd_ = -1;
  void *mapped_data_ = MAP_FAILED;
  std::size_t mapped_size_ = 0;
  struct stat file_identity_ {};
  sqlite3_int64 expected_revision_ = 0;
  sqlite3 *db_ = nullptr;
  // Recursive only to compose validation from the existing locked record readers.
  mutable std::recursive_mutex mutex_;
  Admission admission_ = Admission::None;
  State state_ = State::Transitioning;
  bool first_finalized_commit_ = false;
  bool route_s_only_ = false;
};

}  // namespace sapphire
