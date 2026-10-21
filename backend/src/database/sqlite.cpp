#include "backend/database/sqlite.hpp"

#include <sqlite3.h>
#include <zlib.h>

#include <algorithm>
#include <backend/common/common_type.hpp>
#include <backend/database/localmap_archive.hpp>
#include <backend/maplayer/navimap.hpp>
#include <backend/parameters.hpp>
#include <backend/params/database.hpp>
#include <backend/tools/filesystem.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/strings.hpp>
#include <backend/tools/timer.hpp>
#include <climits>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <regex>
#include <set>

#include "DatabaseSchema_sql.h"
#include "backend/database/compression.hpp"
#include "backend/database/sensor_archive.hpp"
#include "backend/maplayer/node.hpp"
#include "backend/sensor/lidar/geometry.hpp"

namespace sapphire::backend {
namespace {

using Statement = std::unique_ptr<sqlite3_stmt, decltype(&sqlite3_finalize)>;
Statement prepare_statement(sqlite3 *database, const char *sql) {
  sqlite3_stmt *statement = nullptr;
  if (sqlite3_prepare_v2(database, sql, -1, &statement, nullptr) != SQLITE_OK) throw std::runtime_error(sqlite3_errmsg(database));
  return Statement(statement, sqlite3_finalize);
}
void check_sqlite(sqlite3 *database, int result) {
  if (result != SQLITE_OK && result != SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(database));
}
// SUM(NULL) and a closed database both retain the public zero-result contract.
int64_t read_scalar(sqlite3 *database, const char *sql) {
  if (!database) return 0;
  const auto statement = prepare_statement(database, sql);
  const int result = sqlite3_step(statement.get());
  if (result == SQLITE_DONE) return 0;
  if (result != SQLITE_ROW) throw std::runtime_error(sqlite3_errmsg(database));
  const auto value = sqlite3_column_int64(statement.get(), 0);
  check_sqlite(database, sqlite3_step(statement.get()));
  return value;
}
std::vector<uint8_t> encode_words(const std::vector<bow::WordCount> &words) {
  std::vector<uint8_t> bytes;
  bytes.reserve(words.size() * 8);
  for (const auto &word : words)
    for (uint32_t value : {static_cast<uint32_t>(word.word_id), word.count})
      for (int j = 0; j < 4; ++j) bytes.push_back(value >> (8 * j));
  return bytes;
}
std::vector<bow::WordCount> decode_words(const void *data, int size) {
  if (size < 0 || size % 8 || size > int(featmap::point_capacity * 8) || (size && !data))
    throw std::runtime_error("Invalid BoW retrieval summary size");
  const auto *bytes = static_cast<const uint8_t *>(data);
  std::vector<bow::WordCount> words;
  for (int i = 0; i < size; i += 8) {
    uint32_t word = 0, count = 0;
    for (int j = 0; j < 4; ++j) {
      word |= uint32_t(bytes[i + j]) << (8 * j);
      count |= uint32_t(bytes[i + j + 4]) << (8 * j);
    }
    words.push_back({static_cast<int32_t>(word), count});
  }
  return words;
}
uint64_t read_vocabulary_hash(sqlite3_stmt *statement, int column) {
  const auto *text = sqlite3_column_text(statement, column);
  const int size = sqlite3_column_bytes(statement, column);
  if (!text || size <= 0 || size > 20) throw std::runtime_error("Invalid vocabulary hash");
  const std::string value(reinterpret_cast<const char *>(text), size);
  if (value.find_first_not_of("0123456789") != std::string::npos) throw std::runtime_error("Invalid vocabulary hash encoding");
  return std::stoull(value);
}

bool stateGroupFromBlob(const void *data, int dataSize, StateGroupf &state) {
  if (data == nullptr || dataSize != static_cast<int>(StateGroupf::kCoefficientCount * sizeof(float))) {
    return false;
  }
  StateGroupf::Coefficients coefficients{};
  std::memcpy(coefficients.data(), data, sizeof(coefficients));
  state = StateGroupf(coefficients);
  return true;
}

StateGroupf::Coefficients copyStateGroupCoefficients(const StateGroupf &state) {
  StateGroupf::Coefficients coefficients{};
  std::copy(state.coefficients().begin(), state.coefficients().end(), coefficients.begin());
  return coefficients;
}

constexpr std::size_t kPerspectiveCalibrationSize = sizeof(std::uint32_t) * 2U + sizeof(double) * 5U + sizeof(float) * StateGroupf::kCoefficientCount;
static_assert(kPerspectiveCalibrationSize == 96U);

template <typename T>
void appendCalibrationValue(std::vector<unsigned char> &buffer, const T &value) {
  const std::size_t offset = buffer.size();
  buffer.resize(offset + sizeof(T));
  std::memcpy(buffer.data() + offset, &value, sizeof(T));
}

void appendPerspectiveCalibration(std::vector<unsigned char> &buffer, const camera::Perspective &model) {
  EXT_ASSERT(model.isValidForProjection());
  EXT_ASSERT(model.imageSize().width > 0 && model.imageSize().height > 0);
  const std::uint32_t width = static_cast<std::uint32_t>(model.imageSize().width);
  const std::uint32_t height = static_cast<std::uint32_t>(model.imageSize().height);
  appendCalibrationValue(buffer, width);
  appendCalibrationValue(buffer, height);
  appendCalibrationValue(buffer, model.fx());
  appendCalibrationValue(buffer, model.fy());
  appendCalibrationValue(buffer, model.cx());
  appendCalibrationValue(buffer, model.cy());
  appendCalibrationValue(buffer, model.focalXBaseline());
  const StateGroupf::Coefficients coefficients = copyStateGroupCoefficients(model.cameraToBase());
  const std::size_t offset = buffer.size();
  buffer.resize(offset + sizeof(coefficients));
  std::memcpy(buffer.data() + offset, coefficients.data(), sizeof(coefficients));
}

bool decodePerspectiveCalibrations(const void *data, int dataSize, std::vector<camera::Perspective> &models) {
  if (data == nullptr || dataSize <= 0 || static_cast<std::size_t>(dataSize) % kPerspectiveCalibrationSize != 0U) return false;
  const auto *bytes = static_cast<const unsigned char *>(data);
  models.reserve(models.size() + static_cast<std::size_t>(dataSize) / kPerspectiveCalibrationSize);
  for (std::size_t block = 0; block < static_cast<std::size_t>(dataSize); block += kPerspectiveCalibrationSize) {
    std::size_t offset = block;
    const auto read = [&](auto &value) {
      std::memcpy(&value, bytes + offset, sizeof(value));
      offset += sizeof(value);
    };
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double focalXBaseline = 0.0;
    StateGroupf::Coefficients coefficients{};
    read(width);
    read(height);
    read(fx);
    read(fy);
    read(cx);
    read(cy);
    read(focalXBaseline);
    std::memcpy(coefficients.data(), bytes + offset, sizeof(coefficients));
    if (width == 0U || height == 0U || width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) || !std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) ||
        !std::isfinite(cy) || !std::isfinite(focalXBaseline) || fx <= 0.0 || fy <= 0.0 || cx < 0.0 || cy < 0.0) {
      return false;
    }
    const StateGroupf cameraToBase(coefficients);
    if (cameraToBase.isNull()) return false;
    models.emplace_back(fx, fy, cx, cy, cameraToBase, cv::Size(static_cast<int>(width), static_cast<int>(height)), focalXBaseline);
  }
  return true;
}

}  // namespace

// All link queries use the same five-column layout. Link owns its information matrix.
std::optional<Link> DBDriverSqlite3::read_link(sqlite3_stmt *statement, bool ignoreNullLinks) {
  const int from = sqlite3_column_int(statement, 0), to = sqlite3_column_int(statement, 1);
  if (from <= 0 || to <= 0) throw std::runtime_error("Stored Link requires positive node IDs");
  const auto type = static_cast<Link::Type>(sqlite3_column_int(statement, 2));
  if (!Link::isConstraintType(type))
    throw std::runtime_error(fmt::format("Unsupported stored Link type {} ({} -> {}); this database requires rebuilding", int(type), from, to));

  const void *data = sqlite3_column_blob(statement, 3);
  const int size = sqlite3_column_bytes(statement, 3);
  StateGroupf transform;
  if (!stateGroupFromBlob(data, size, transform) && size) throw std::runtime_error(fmt::format("Invalid stored Link transform ({} -> {})", from, to));
  if (ignoreNullLinks && transform.isNull()) return std::nullopt;

  data = sqlite3_column_blob(statement, 4);
  if (!data || sqlite3_column_bytes(statement, 4) != 36 * sizeof(double))
    throw std::runtime_error(fmt::format("Invalid stored Link information matrix ({} -> {})", from, to));
  MatRC_t<6, 6, double, Eigen::RowMajor> information;
  std::memcpy(information.data(), data, sizeof(double) * 36);
  Link result(from, to, type, transform, information);
  return result;
}

DBDriverSqlite3::DBDriverSqlite3(const ParametersMap &parameters) : DBDriverSqlite3(DatabaseParams{}.withOverrides(parameters)) {}

DBDriverSqlite3::DBDriverSqlite3(const DatabaseParams &configuration) : DBDriver(configuration), _ppDb(0), _version("0"), _memoryUsedEstimate(0) {
  _dbInMemory = configuration.sqlite_in_memory;
  _cacheSize = configuration.sqlite_cache_size_pages;
  _journalMode = configuration.sqlite_journal_mode;
  _synchronous = configuration.sqlite_synchronous_mode;
  _tempStore = configuration.sqlite_temp_store;
}

DBDriverSqlite3::~DBDriverSqlite3() {
  try {
    closeConnection(isWorkerHealthy());
  } catch (const std::exception &error) {
    // Explicit Save reports failures to its caller; destruction must still release resources.
    EXT_LOG_ERROR("Database destruction: {}", error.what());
    try {
      closeConnection(false);
    } catch (const std::exception &cleanup) {
      EXT_LOG_ERROR("Database cleanup: {}", cleanup.what());
    }
  }
}

/*
** This function is used to load the contents of a database file on disk
** into the "main" database of open database connection pInMemory, or
** to save the current contents of the database opened by pInMemory into
** a database file on disk. pInMemory is probably an in-memory database,
** but this function will also work fine if it is not.
**
** Parameter zFilename points to a nul-terminated string containing the
** name of the database file on disk to load from or save to. If parameter
** isSave is non-zero, then the contents of the file zFilename are
** overwritten with the contents of the database opened by pInMemory. If
** parameter isSave is zero, then the contents of the database opened by
** pInMemory are replaced by data loaded from the file zFilename.
**
** If the operation is successful, SQLITE_OK is returned. Otherwise, if
** an error occurs, an SQLite error code is returned.
*/
int DBDriverSqlite3::loadOrSaveDb(sqlite3 *pInMemory, const std::string &fileName, int isSave) const {
  int rc;                      /* Function return code */
  sqlite3 *pFile = 0;          /* Database connection opened on zFilename */
  sqlite3_backup *pBackup = 0; /* Backup object used to copy data */
  sqlite3 *pTo = 0;            /* Database to copy to (pFile or pInMemory) */
  sqlite3 *pFrom = 0;          /* Database to copy from (pFile or pInMemory) */

  /* Open the database file identified by zFilename. Exit early if this fails
  ** for any reason. */
  rc = sqlite3_open(fileName.c_str(), &pFile);
  if (rc == SQLITE_OK) {
    /* If this is a 'load' operation (isSave==0), then data is copied
    ** from the database file just opened to database pInMemory.
    ** Otherwise, if this is a 'save' operation (isSave==1), then data
    ** is copied from pInMemory to pFile.  Set the variables pFrom and
    ** pTo accordingly. */
    pFrom = (isSave ? pInMemory : pFile);
    pTo = (isSave ? pFile : pInMemory);

    /* Set up the backup procedure to copy from the "main" database of
    ** connection pFile to the main database of connection pInMemory.
    ** If something goes wrong, pBackup will be set to NULL and an error
    ** code and  message left in connection pTo.
    **
    ** If the backup object is successfully created, call backup_step()
    ** to copy data from pFile to pInMemory. Then call backup_finish()
    ** to release resources associated with the pBackup object.  If an
    ** error occurred, then  an error code and message will be left in
    ** connection pTo. If no error occurred, then the error code belonging
    ** to pTo is set to SQLITE_OK.
    */
    pBackup = sqlite3_backup_init(pTo, "main", pFrom, "main");
    if (pBackup) {
      (void)sqlite3_backup_step(pBackup, -1);
      (void)sqlite3_backup_finish(pBackup);
    }
    rc = sqlite3_errcode(pTo);
  }

  /* Close the database connection opened on database file zFilename
  ** and return the result of this function. */
  (void)sqlite3_close(pFile);
  return rc;
}

bool DBDriverSqlite3::connectDatabaseQuery(const std::string &url, bool overwritten, bool readOnly) {
  this->disconnectDatabaseQuery();
  // Open a database connection
  _ppDb = 0;
  _memoryUsedEstimate = 0;

  int rc = SQLITE_OK;
  bool dbFileExist = false;
  if (!url.empty()) {
    dbFileExist = tools::path_exists(url);
    if (dbFileExist && overwritten && !readOnly) {
      EXT_LOG_INFO("Deleting database {}...", url.c_str());
      std::error_code error;
      const bool removed = tools::remove_file(url, error);
      EXT_ASSERT_MSG(removed && !error, "{}", error.message());
      dbFileExist = false;
    } else if (dbFileExist) {
      std::error_code error;
      const std::uintmax_t size = tools::file_size(this->getUrl(), error);
      if (error) {
        EXT_LOG_ERROR("Failed to get database size for {}: {}", this->getUrl().c_str(), error.message().c_str());
      } else {
        _memoryUsedEstimate = size > ULONG_MAX ? ULONG_MAX : static_cast<unsigned long>(size);
      }
    }
  }

  if (_dbInMemory || url.empty()) {
    if (!url.empty()) {
      EXT_LOG_INFO("Using database \"{}\" in the memory.", url.c_str());
    } else {
      EXT_LOG_INFO("Using empty database in the memory.");
    }
    rc = sqlite3_open_v2(":memory:", &_ppDb, readOnly ? SQLITE_OPEN_READONLY : SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, 0);
  } else {
    EXT_LOG_INFO("Using database \"{}\" from the hard drive.", url.c_str());
    rc = sqlite3_open_v2(url.c_str(), &_ppDb, readOnly ? SQLITE_OPEN_READONLY : SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, 0);
  }
  if (rc != SQLITE_OK) {
    EXT_LOG_FATAL(
        "DB error : {} (path=\"{}\"). Make sure that your user has write "
        "permission on the target directory (you may have to change the working directory). ",
        sqlite3_errmsg(_ppDb), url.c_str());
    _ppDb = 0;
    return false;
  }

  // A concurrent inspection/backup may briefly hold a read lock. SQLite's
  // default zero timeout turns that into an immediate write failure.
  check_sqlite(_ppDb, sqlite3_busy_timeout(_ppDb, 1000));

  if (_dbInMemory && dbFileExist) {
    tools::Timer timer;
    timer.reset();
    EXT_LOG_DEBUG("Loading DB ...");
    rc = loadOrSaveDb(_ppDb, url, 0);  // Load memory from file
    EXT_LOG_INFO("Loading DB time = {:.6f}s, ({})", timer.lap_seconds(), url.c_str());
    if (rc != SQLITE_OK) {
      EXT_LOG_FATAL("DB error 2 : {}", sqlite3_errmsg(_ppDb));
      sqlite3_close(_ppDb);
      _ppDb = 0;
      return false;
    }
  }

  if (!dbFileExist) {
    if (!url.empty()) {
      EXT_LOG_INFO("Database \"{}\" doesn't exist, creating a new one...", url.c_str());
    }
    // Create the database
    std::string schema = DATABASESCHEMA_SQL;
    schema = tools::hex_to_bytes(schema);
    this->executeNoResultQuery(schema.c_str());
  }
  {
    auto statement = prepare_statement(_ppDb, "PRAGMA user_version");
    const int version = sqlite3_step(statement.get()) == SQLITE_ROW ? sqlite3_column_int(statement.get(), 0) : -1;
    statement.reset();
    if (version != 12) {
      EXT_LOG_ERROR("Unsupported database schema: expected 12 (node graph and LiDAR geometry); automatic migration is not supported");
      sqlite3_close(_ppDb);
      _ppDb = nullptr;
      return false;
    }
  }
  // The caller holds DBDriver's database mutex while connecting. Apply these
  // directly to avoid recursively entering the public locking wrappers.
  this->executeNoResultQuery(("PRAGMA cache_size = " + std::to_string(_cacheSize) + ";").c_str());
  static const char *journalModes[] = {"DELETE", "TRUNCATE", "PERSIST", "MEMORY", "OFF", "WAL"};
  if (!readOnly) this->executeNoResultQuery((std::string("PRAGMA journal_mode = ") + journalModes[_journalMode] + ";").c_str());
  static const char *synchronousModes[] = {"OFF", "NORMAL", "FULL"};
  this->executeNoResultQuery((std::string("PRAGMA synchronous = ") + synchronousModes[_synchronous] + ";").c_str());
  static const char *tempStores[] = {"DEFAULT", "FILE", "MEMORY"};
  this->executeNoResultQuery((std::string("PRAGMA temp_store = ") + tempStores[_tempStore] + ";").c_str());

  return true;
}
void DBDriverSqlite3::disconnectDatabaseQuery(bool save, const std::string &outputUrl) {
  EXT_LOG_DEBUG("");
  if (_ppDb) {
    int rc = SQLITE_OK;
    // make sure that all statements are finalized
    sqlite3_stmt *pStmt;
    while ((pStmt = sqlite3_next_stmt(_ppDb, 0)) != 0) {
      rc = sqlite3_finalize(pStmt);
      if (rc != SQLITE_OK) {
        EXT_LOG_ERROR("");
      }
    }

    if (save && (_dbInMemory || this->getUrl().empty())) {
      tools::Timer timer;
      timer.reset();
      std::string outputFile = this->getUrl();
      if (!outputUrl.empty()) {
        outputFile = outputUrl;
      }
      if (outputFile.empty()) {
        EXT_LOG_WARN(
            "Database was initialized with an empty url (in memory). To save it, "
            "the output url should not be empty. The database is thus closed without being saved!");
      } else {
        EXT_LOG_INFO("Saving database to {} ...", outputFile.c_str());
        rc = loadOrSaveDb(_ppDb, outputFile, 1);  // Save memory to file
        EXT_ASSERT_MSG(rc == SQLITE_OK,
                       "DB error ({}), could not save \"{}\": {}. Make sure that your user has write "
                       "permission on the target directory (you may have to change the working directory). ",
                       _version.c_str(), outputFile.c_str(), sqlite3_errmsg(_ppDb));
        EXT_LOG_DEBUG("Saving DB time = {:.6f}s", timer.lap_seconds());
      }
    }

    // Then close (delete) the database connection
    EXT_LOG_INFO("Disconnecting database {}...", this->getUrl().c_str());
    sqlite3_close(_ppDb);
    _ppDb = 0;

    if (save && !_dbInMemory && !outputUrl.empty() && !this->getUrl().empty() && outputUrl.compare(this->getUrl()) != 0) {
      EXT_LOG_WARN(
          "Output database path ({}) is different than the opened database "
          "path ({}). Opened database path is overwritten then renamed to output path.",
          outputUrl.c_str(), this->getUrl().c_str());
      std::error_code error;
      tools::rename_file(this->getUrl(), outputUrl, error);
      if (error) {
        EXT_LOG_ERROR("Failed to rename just closed db {} to {}", this->getUrl().c_str(), outputUrl.c_str());
      }
    }
    EXT_LOG_INFO("Disconnected database {}!", this->getUrl().c_str());
  }
}

bool DBDriverSqlite3::isConnectedQuery() const { return _ppDb != 0; }

// In bytes
void DBDriverSqlite3::executeNoResultQuery(const std::string &sql) const {
  if (_ppDb) {
    tools::Timer timer;
    timer.reset();
    int rc;
    rc = sqlite3_exec(_ppDb, sql.c_str(), 0, 0, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error: {}, the query is {}", sqlite3_errmsg(_ppDb), sql.c_str());
    EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());
  }
}

unsigned long DBDriverSqlite3::getMemoryUsedQuery() const {
  if (_dbInMemory) {
    return sqlite3_memory_used();
  } else {
    return _memoryUsedEstimate;
  }
}

long DBDriverSqlite3::getNodesMemoryUsedQuery() const {
  return read_scalar(_ppDb,
                     "SELECT sum(length(id) + length(map_id) + length(weight) + length(pose) + length(stamp) + ifnull(length(label),0) + "
                     "ifnull(length(velocity),0) + "
                     "ifnull(length(env_sensors),0) + length(time_enter)) from Node;");
}

long DBDriverSqlite3::getLinksMemoryUsedQuery() const {
  return read_scalar(_ppDb,
                     "SELECT sum(length(type) + length(information_matrix) + length(transform) + length(from_id) + length(to_id)) from "
                     "Link;");
}

long DBDriverSqlite3::getImagesMemoryUsedQuery() const {
  return read_scalar(_ppDb, "SELECT sum(ifnull(length(image),0) + ifnull(length(time_enter),0)) from Data;");
}

long DBDriverSqlite3::getDepthImagesMemoryUsedQuery() const {
  return read_scalar(_ppDb, "SELECT sum(ifnull(length(depth),0) + ifnull(length(time_enter),0)) from Data;");
}

long DBDriverSqlite3::getCalibrationsMemoryUsedQuery() const { return read_scalar(_ppDb, "SELECT sum(length(calibration)) from Data;"); }

long DBDriverSqlite3::getGridsMemoryUsedQuery() const {
  return read_scalar(
      _ppDb,
      "SELECT sum(ifnull(length(ground_cells),0) + ifnull(length(obstacle_cells),0) + ifnull(length(empty_cells),0) + length(cell_size) + "
      "length(view_point_x) + length(view_point_y) + "
      "length(view_point_z)) from Data;");
}

long DBDriverSqlite3::getUserDataMemoryUsedQuery() const { return read_scalar(_ppDb, "SELECT sum(length(user_data)) from Data;"); }

long DBDriverSqlite3::getFeaturesMemoryUsedQuery() const {
  return read_scalar(_ppDb,
                     "SELECT sum(length(node_id) + length(feature_count) + length(vocabulary_hash) + length(words) + ifnull(length(block),0)) "
                     "FROM Feature");
}

long DBDriverSqlite3::getStatisticsMemoryUsedQuery() const {
  return read_scalar(_ppDb, "SELECT sum(length(id) + length(stamp) + ifnull(length(data),0) + ifnull(length(wm_state),0)) FROM Statistics;");
}

void DBDriverSqlite3::addInfoAfterRunQuery(int size, const RunInfo &info) const {
  auto row = prepare_statement(
      _ppDb, "INSERT INTO Info(STM_size,last_sign_added,process_mem_used,database_mem_used,dictionary_size,parameters) VALUES(?1,?2,?3,?4,?5,?6)");
  check_sqlite(_ppDb, sqlite3_bind_int(row.get(), 1, size));
  check_sqlite(_ppDb, sqlite3_bind_int(row.get(), 2, info.lastNode));
  check_sqlite(_ppDb, sqlite3_bind_int(row.get(), 3, info.processMemoryUsed));
  check_sqlite(_ppDb, sqlite3_bind_int(row.get(), 4, info.databaseMemoryUsed));
  check_sqlite(_ppDb, sqlite3_bind_int(row.get(), 5, info.dictionarySize));
  const auto parameters = Parameters::serialize(info.parameters);
  check_sqlite(_ppDb, sqlite3_bind_text(row.get(), 6, parameters.c_str(), -1, SQLITE_TRANSIENT));
  check_sqlite(_ppDb, sqlite3_step(row.get()));
}

void DBDriverSqlite3::saveWorkingMemoryQuery(const std::vector<Node *> &nodes, const RunInfo &info) const {
  executeNoResultQuery("DELETE FROM WorkingMemory;");
  auto row = prepare_statement(_ppDb, "INSERT INTO WorkingMemory(node_id) VALUES(?1)");
  for (const auto *node : nodes) {
    check_sqlite(_ppDb, sqlite3_bind_int(row.get(), 1, node->id()));
    check_sqlite(_ppDb, sqlite3_step(row.get()));
    check_sqlite(_ppDb, sqlite3_reset(row.get()));
  }
  addInfoAfterRunQuery(static_cast<int>(nodes.size()), info);
}

int DBDriverSqlite3::getLastNodesSizeQuery() const { return read_scalar(_ppDb, "SELECT count(node_id) FROM WorkingMemory;"); }

int DBDriverSqlite3::getTotalNodesSizeQuery() const { return read_scalar(_ppDb, "SELECT count(id) from Node;"); }

ParametersMap DBDriverSqlite3::getLastParametersQuery() const {
  if (!_ppDb) return {};
  auto statement = prepare_statement(_ppDb, "SELECT parameters FROM Info ORDER BY rowid DESC LIMIT 1");
  const int rc = sqlite3_step(statement.get());
  if (rc == SQLITE_DONE) return {};
  if (rc != SQLITE_ROW) throw std::runtime_error(sqlite3_errmsg(_ppDb));
  const auto *text = sqlite3_column_text(statement.get(), 0);
  return text ? Parameters::deserialize(reinterpret_cast<const char *>(text)) : ParametersMap{};
}

std::map<std::string, float> DBDriverSqlite3::getStatisticsQuery(int nodeId, double &stamp, std::vector<int> *wmState) const {
  EXT_LOG_DEBUG("nodeId={:d}", nodeId);
  std::map<std::string, float> data;
  if (_ppDb) {
    {
      std::stringstream query;

      if (wmState) {
        query << "SELECT stamp, data, wm_state "
              << "FROM Statistics "
              << "WHERE id=" << nodeId << ";";
      } else {
        query << "SELECT stamp, data "
              << "FROM Statistics "
              << "WHERE id=" << nodeId << ";";
      }

      int rc = SQLITE_OK;
      sqlite3_stmt *ppStmt = 0;
      rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      rc = sqlite3_step(ppStmt);
      if (rc == SQLITE_ROW) {
        int index = 0;
        stamp = sqlite3_column_double(ppStmt, index++);

        std::string text;
        {
          const void *dataPtr = sqlite3_column_blob(ppStmt, index);
          int dataSize = sqlite3_column_bytes(ppStmt, index++);
          if (dataSize > 0 && dataPtr) {
            text = uncompressString(cv::Mat(1, dataSize, CV_8UC1, (void *)dataPtr));
          }
        }

        if (text.size()) {
          data = Statistics::deserializeData(text);
        }

        if (wmState) {
          const void *dataPtr = sqlite3_column_blob(ppStmt, index);
          int dataSize = sqlite3_column_bytes(ppStmt, index++);
          if (dataSize > 0 && dataPtr) {
            cv::Mat wmStateMat = uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)dataPtr));
            EXT_ASSERT(wmStateMat.type() == CV_32SC1 && wmStateMat.rows == 1);
            wmState->resize(wmStateMat.cols);
            memcpy(wmState->data(), wmStateMat.data, wmState->size() * sizeof(int));
          }
        }

        rc = sqlite3_step(ppStmt);
      }
      EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      rc = sqlite3_finalize(ppStmt);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    }
  }
  return data;
}

std::map<int, std::pair<std::map<std::string, float>, double>> DBDriverSqlite3::getAllStatisticsQuery() const {
  EXT_LOG_DEBUG("");
  std::map<int, std::pair<std::map<std::string, float>, double>> data;
  if (_ppDb) {
    {
      std::stringstream query;

      query << "SELECT id, stamp, data "
            << "FROM Statistics;";

      int rc = SQLITE_OK;
      sqlite3_stmt *ppStmt = 0;
      rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      rc = sqlite3_step(ppStmt);
      while (rc == SQLITE_ROW) {
        int index = 0;
        int id = sqlite3_column_int(ppStmt, index++);
        double stamp = sqlite3_column_double(ppStmt, index++);

        std::string text;
        {
          const void *dataPtr = 0;
          int dataSize = 0;
          dataPtr = sqlite3_column_blob(ppStmt, index);
          dataSize = sqlite3_column_bytes(ppStmt, index++);
          if (dataSize > 0 && dataPtr) {
            text = uncompressString(cv::Mat(1, dataSize, CV_8UC1, (void *)dataPtr));
          }
        }

        if (text.size()) {
          data.insert(std::make_pair(id, std::make_pair(Statistics::deserializeData(text), stamp)));
        }

        rc = sqlite3_step(ppStmt);
      }
      EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      rc = sqlite3_finalize(ppStmt);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    }
  }
  EXT_LOG_DEBUG("");
  return data;
}

std::map<int, std::vector<int>> DBDriverSqlite3::getAllStatisticsWmStatesQuery() const {
  EXT_LOG_DEBUG("");
  std::map<int, std::vector<int>> data;
  if (_ppDb) {
    {
      std::stringstream query;

      query << "SELECT id, wm_state "
            << "FROM Statistics;";

      int rc = SQLITE_OK;
      sqlite3_stmt *ppStmt = 0;
      rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      rc = sqlite3_step(ppStmt);
      while (rc == SQLITE_ROW) {
        int index = 0;
        int id = sqlite3_column_int(ppStmt, index++);

        std::vector<int> wmState;
        const void *dataPtr = sqlite3_column_blob(ppStmt, index);
        int dataSize = sqlite3_column_bytes(ppStmt, index++);
        if (dataSize > 0 && dataPtr) {
          cv::Mat wmStateMat = uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)dataPtr));
          EXT_ASSERT(wmStateMat.type() == CV_32SC1 && wmStateMat.rows == 1);
          wmState.resize(wmStateMat.cols);
          memcpy(wmState.data(), wmStateMat.data, wmState.size() * sizeof(int));
        }

        if (!wmState.empty()) {
          data.insert(std::make_pair(id, wmState));
        }

        rc = sqlite3_step(ppStmt);
      }
      EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      rc = sqlite3_finalize(ppStmt);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    }
  }
  EXT_LOG_DEBUG("");
  return data;
}

void DBDriverSqlite3::loadNodeDataQuery(std::list<SensorRecord *> &records, bool images, bool userData, bool occupancyGrid) const {
  if (!images && !userData && !occupancyGrid) {
    EXT_LOG_WARN("All requested data fields are false! Nothing loaded...");
    return;
  }

  if (_ppDb) {
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::stringstream query;

    {
      std::stringstream fields;

      if (images) {
        { fields << "image, depth, calibration"; }

        if (userData || occupancyGrid) {
          fields << ", ";
        }
      }
      if (userData) {
        fields << "user_data";
        if (occupancyGrid) {
          fields << ", ";
        }
      }
      if (occupancyGrid) {
        { fields << "ground_cells, obstacle_cells, empty_cells, cell_size, view_point_x, view_point_y, view_point_z"; }
      }

      query << "SELECT " << fields.str().c_str() << " "
            << "FROM Data "
            << "WHERE id = ?"
            << ";";
    }

    rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    const void *data = 0;
    int dataSize = 0;
    int index = 0;

    for (auto iter = records.begin(); iter != records.end(); ++iter) {
      EXT_ASSERT(*iter != 0);

      // EXT_LOG_DEBUG("Loading data for %d...", (*iter)->observation.id());
      //  bind id
      rc = sqlite3_bind_int(ppStmt, 1, (*iter)->observation.id());
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

      // Process the result if one
      rc = sqlite3_step(ppStmt);
      if (rc == SQLITE_ROW) {
        index = 0;

        cv::Mat imageCompressed;
        cv::Mat depthCompressed;
        std::vector<camera::Perspective> models;
        cv::Mat userDataCompressed;

        if (images) {
          // Create the image
          data = sqlite3_column_blob(ppStmt, index);
          dataSize = sqlite3_column_bytes(ppStmt, index++);
          if (dataSize > 4 && data) {
            imageCompressed = cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone();
          }

          // Create the depth image
          data = sqlite3_column_blob(ppStmt, index);
          dataSize = sqlite3_column_bytes(ppStmt, index++);
          if (dataSize > 4 && data) {
            depthCompressed = cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone();
          }

          // calibration
          {
            data = sqlite3_column_blob(ppStmt, index);
            dataSize = sqlite3_column_bytes(ppStmt, index++);
            if (dataSize > 0 && data) {
              EXT_ASSERT_MSG(decodePerspectiveCalibrations(data, dataSize, models), "Invalid Perspective calibration field (size={:d} bytes)",
                             dataSize);
            }
          }
        }

        if (userData) {
          {
            data = sqlite3_column_blob(ppStmt, index);
            dataSize = sqlite3_column_bytes(ppStmt, index++);
            // Create the userData
            if (dataSize > 4 && data) {
              {
                userDataCompressed = cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone();  // userData
              }
            }
          }
        }

        // Occupancy grid
        cv::Mat groundCellsCompressed;
        cv::Mat obstacleCellsCompressed;
        cv::Mat emptyCellsCompressed;
        float cellSize = 0.0f;
        cv::Point3f viewPoint;
        if (occupancyGrid) {
          // ground
          data = sqlite3_column_blob(ppStmt, index);
          dataSize = sqlite3_column_bytes(ppStmt, index++);
          if (dataSize > 0 && data) {
            groundCellsCompressed = cv::Mat(1, dataSize, CV_8UC1);
            memcpy((void *)groundCellsCompressed.data, data, dataSize);
          }

          // obstacle
          data = sqlite3_column_blob(ppStmt, index);
          dataSize = sqlite3_column_bytes(ppStmt, index++);
          if (dataSize > 0 && data) {
            obstacleCellsCompressed = cv::Mat(1, dataSize, CV_8UC1);
            memcpy((void *)obstacleCellsCompressed.data, data, dataSize);
          }

          {
            // empty
            data = sqlite3_column_blob(ppStmt, index);
            dataSize = sqlite3_column_bytes(ppStmt, index++);
            if (dataSize > 0 && data) {
              emptyCellsCompressed = cv::Mat(1, dataSize, CV_8UC1);
              memcpy((void *)emptyCellsCompressed.data, data, dataSize);
            }
          }

          cellSize = sqlite3_column_double(ppStmt, index++);
          viewPoint.x = sqlite3_column_double(ppStmt, index++);
          viewPoint.y = sqlite3_column_double(ppStmt, index++);
          viewPoint.z = sqlite3_column_double(ppStmt, index++);
        }

        if (images) {
          if (models.empty() && (!imageCompressed.empty() || !depthCompressed.empty())) {
            models.push_back(camera::Perspective());
          }
          (*iter)->observation.setImageData({}, {}, models);
          (**iter).archive.image = imageCompressed;
          (**iter).archive.depth = depthCompressed;
        }
        if (userData) {
          (*iter)->observation.setUserData({});
          (**iter).archive.user = userDataCompressed;
        }

        if (occupancyGrid) {
          restoreGrid((**iter), {groundCellsCompressed, obstacleCellsCompressed, emptyCellsCompressed, cellSize, viewPoint});
        }

        rc = sqlite3_step(ppStmt);  // next result...
      }
      EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

      // reset
      rc = sqlite3_reset(ppStmt);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    }

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    // EXT_LOG_DEBUG("Time=%fs", timer.lap_seconds());
  }
}

bool DBDriverSqlite3::getCalibrationQuery(int nodeId, std::vector<camera::Perspective> &models) const {
  bool found = false;
  if (_ppDb && nodeId) {
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::stringstream query;

    {
      query << "SELECT calibration "
            << "FROM Data "
            << "WHERE id = " << nodeId << ";";
    }

    rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    const void *data = 0;
    int dataSize = 0;

    // Process the result if one
    rc = sqlite3_step(ppStmt);
    if (rc == SQLITE_ROW) {
      found = true;
      int index = 0;

      // calibration

      {
        data = sqlite3_column_blob(ppStmt, index);
        dataSize = sqlite3_column_bytes(ppStmt, index++);
        if (dataSize > 0 && data) {
          EXT_ASSERT_MSG(decodePerspectiveCalibrations(data, dataSize, models), "Invalid Perspective calibration field (size={:d} bytes)", dataSize);
        }
      }

      rc = sqlite3_step(ppStmt);  // next result...
    }
    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  }
  return found;
}

bool DBDriverSqlite3::getNodeInfoQuery(int nodeId, StateGroupf &pose, int &mapId, int &weight, std::string &label, double &stamp,
                                       std::vector<float> &velocity, EnvSensors &sensors) const {
  bool found = false;
  if (_ppDb && nodeId) {
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::stringstream query;

    {
      query << "SELECT pose, map_id, weight, label, stamp, velocity, env_sensors "
               "FROM Node "
               "WHERE id = "
            << nodeId << ";";
    }

    rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    const void *data = 0;
    int dataSize = 0;

    // Process the result if one
    rc = sqlite3_step(ppStmt);
    if (rc == SQLITE_ROW) {
      found = true;
      int index = 0;
      data = sqlite3_column_blob(ppStmt, index);  // pose
      dataSize = sqlite3_column_bytes(ppStmt, index++);
      stateGroupFromBlob(data, dataSize, pose);

      mapId = sqlite3_column_int(ppStmt, index++);   // map id
      weight = sqlite3_column_int(ppStmt, index++);  // weight

      {
        const unsigned char *p = sqlite3_column_text(ppStmt, index++);
        if (p) {
          label = reinterpret_cast<const char *>(p);  // label
        }
        stamp = sqlite3_column_double(ppStmt, index++);  // stamp

        velocity.resize(6, 0);
        data = sqlite3_column_blob(ppStmt, index);  // velocity
        dataSize = sqlite3_column_bytes(ppStmt, index++);
        if ((unsigned int)dataSize == velocity.size() * sizeof(float) && data) {
          memcpy(velocity.data(), data, dataSize);
        }

        data = sqlite3_column_blob(ppStmt, index);
        dataSize = sqlite3_column_bytes(ppStmt, index++);
        if (data) {
          EXT_ASSERT(dataSize % sizeof(double) == 0 && (dataSize / sizeof(double)) % 3 == 0);
          const double *dataDouble = (const double *)data;
          int sensorsNum = (dataSize / sizeof(double)) / 3;
          for (int i = 0; i < sensorsNum; ++i) {
            EnvSensor::Type type = (EnvSensor::Type)(int)dataDouble[i * 3];
            sensors.insert(std::make_pair(type, EnvSensor(type, dataDouble[i * 3 + 1], dataDouble[i * 3 + 2])));
          }
        }
      }

      rc = sqlite3_step(ppStmt);  // next result...
    }
    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  }
  return found;
}

void DBDriverSqlite3::getLocalFeaturesQuery(int nodeId, std::multimap<int, int> &words, std::vector<cv::KeyPoint> &keypoints,
                                            std::vector<cv::Point3f> &points, cv::Mat &descriptors) const {
  Node s(nodeId);
  std::list<Node *> ids;
  ids.push_back(&s);
  this->loadWordsQuery(ids);
  words = ids.front()->getWords();
  keypoints = ids.front()->getWordsKpts();
  points = ids.front()->getWords3();
  descriptors = ids.front()->getWordsDescriptors().clone();
}

void DBDriverSqlite3::getLastNodeIdsQuery(std::set<int> &ids) const {
  if (_ppDb) {
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::string query;

    {
      query =
          "SELECT n.id "
          "FROM Node AS n JOIN WorkingMemory AS w ON w.node_id=n.id "
          "ORDER BY n.id;";
    }

    rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Process the result if one
    rc = sqlite3_step(ppStmt);
    while (rc == SQLITE_ROW) {
      ids.insert(ids.end(), sqlite3_column_int(ppStmt, 0));  // Node Id
      rc = sqlite3_step(ppStmt);
    }
    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    EXT_LOG_DEBUG("Time={:.6f} ids={:d}", timer.lap_seconds(), (int)ids.size());
  }
}

void DBDriverSqlite3::getAllNodeIdsQuery(std::set<int> &ids, bool ignoreChildren, bool ignoreBadNodes, bool ignoreIntermediateNodes) const {
  if (_ppDb) {
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::stringstream query;

    query << "SELECT DISTINCT id "
          << "FROM Node ";
    if (ignoreChildren) {
      // use to_id to ignore all children (which don't have link pointing on them)
      // ignore self referring links
      // keep nodes without link to other nodes (map has only a single node)
      query << "WHERE ";
      query << "(EXISTS (select 1 from Link where Node.id=to_id and from_id != to_id) OR ";
      query << " NOT EXISTS (select 1 from Link where id=to_id and from_id != to_id)) ";

      query << "AND weight>-9 ";  // ignore invalid nodes
      if (ignoreIntermediateNodes) {
        query << "AND weight!=-1 ";  // ignore intermediate nodes
      }
    } else if (ignoreIntermediateNodes) {
      query << "WHERE weight!=-1 ";  // ignore intermediate nodes
    }

    if (ignoreBadNodes) {
      if (ignoreChildren || ignoreIntermediateNodes) {
        query << "AND ";
      } else {
        query << "WHERE ";
      }
      { query << " id in (select node_id from Feature) "; }
    }

    query << "ORDER BY id";

    rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Process the result if one
    rc = sqlite3_step(ppStmt);
    while (rc == SQLITE_ROW) {
      ids.insert(ids.end(), sqlite3_column_int(ppStmt, 0));  // Node Id
      rc = sqlite3_step(ppStmt);
    }
    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    EXT_LOG_DEBUG("Time={:.6f} ids={:d}", timer.lap_seconds(), (int)ids.size());
  }
}

void DBDriverSqlite3::getAllOdomPosesQuery(std::map<int, StateGroupf> &poses, bool ignoreChildren, bool ignoreIntermediateNodes) const {
  if (_ppDb) {
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::stringstream query;

    query << "SELECT DISTINCT id, pose "
          << "FROM Node ";
    if (ignoreChildren) {
      query << "INNER JOIN Link ";
      query << "ON id = to_id ";           // use to_id to ignore all children (which don't have link pointing on them)
      query << "WHERE from_id != to_id ";  // ignore self referring links
      query << "AND weight>-9 ";           // ignore invalid nodes
      if (ignoreIntermediateNodes) {
        query << "AND weight!=-1 ";  // ignore intermediate nodes
      }
    } else if (ignoreIntermediateNodes) {
      query << "WHERE weight!=-1 ";  // ignore intermediate nodes
    }

    query << "ORDER BY id";

    rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    const void *data = 0;
    int dataSize = 0;

    // Process the result if one
    rc = sqlite3_step(ppStmt);
    while (rc == SQLITE_ROW) {
      int id = sqlite3_column_int(ppStmt, 0);  // Node Id
      data = sqlite3_column_blob(ppStmt, 1);   // Pose
      dataSize = sqlite3_column_bytes(ppStmt, 1);

      StateGroupf pose;
      if (!stateGroupFromBlob(data, dataSize, pose) && dataSize) {
        EXT_LOG_ERROR("Error while loading pose for node {:d}! Setting to null...", id);
      }
      poses.insert(std::make_pair(id, pose));
      rc = sqlite3_step(ppStmt);
    }
    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    EXT_LOG_DEBUG("Time={:.6f} ids={:d}", timer.lap_seconds(), (int)poses.size());
  }
}

void DBDriverSqlite3::getAllLinksQuery(std::multimap<int, Link> &links, bool ignoreNullLinks) const {
  links.clear();
  if (!_ppDb) return;
  auto statement = prepare_statement(_ppDb, "SELECT from_id,to_id,type,transform,information_matrix FROM Link ORDER BY from_id,to_id");
  int result;
  while ((result = sqlite3_step(statement.get())) == SQLITE_ROW) {
    if (auto link = read_link(statement.get(), ignoreNullLinks)) links.emplace(link->from(), std::move(*link));
  }
  check_sqlite(_ppDb, result);
}

void DBDriverSqlite3::getLastIdQuery(const std::string &tableName, int &id, const std::string &fieldName) const {
  if (_ppDb) {
    EXT_LOG_DEBUG("get last {} from table \"{}\"", fieldName.c_str(), tableName.c_str());
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::stringstream query;

    if (tableName == "Node" && fieldName == "id")
      query << "SELECT high_water FROM NodeSequence WHERE singleton=1";
    else
      query << "SELECT COALESCE(MAX(" << fieldName << "), " << id << ") "  // In case the table is empty, return back input id
            << "FROM " << tableName << ";";

    rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Process the result if one
    rc = sqlite3_step(ppStmt);
    if (rc == SQLITE_ROW) {
      id = sqlite3_column_int(ppStmt, 0);  // Node Id
      rc = sqlite3_step(ppStmt);
      EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    } else {
      EXT_LOG_DEBUG("No result from the DB for table {} with field {}", tableName.c_str(), fieldName.c_str());
    }

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());
  }
}

void DBDriverSqlite3::getInvertedIndexNiQuery(int nodeId, int &ni) const {
  ni = 0;
  if (!_ppDb) return;
  auto statement = prepare_statement(_ppDb, "SELECT retrieval_mass FROM Feature WHERE node_id=?");
  check_sqlite(_ppDb, sqlite3_bind_int(statement.get(), 1, nodeId));
  const int result = sqlite3_step(statement.get());
  if (result == SQLITE_ROW)
    ni = sqlite3_column_int(statement.get(), 0);
  else
    check_sqlite(_ppDb, result);
}

void DBDriverSqlite3::getNodeIdByLabelQuery(const std::string &label, int &id) const {
  if (_ppDb && !label.empty()) {
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::stringstream query;
    query << "SELECT id FROM Node WHERE label='" << label << "'";

    rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Process the result if one
    rc = sqlite3_step(ppStmt);
    if (rc == SQLITE_ROW) {
      id = sqlite3_column_int(ppStmt, 0);
      rc = sqlite3_step(ppStmt);
    }
    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    EXT_LOG_DEBUG("Time={:.6f}", timer.lap_seconds());
  }
}

void DBDriverSqlite3::getAllLabelsQuery(std::map<int, std::string> &labels) const {
  if (_ppDb) {
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::stringstream query;
    query << "SELECT id,label FROM Node WHERE label IS NOT NULL";

    rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Process the result if one
    rc = sqlite3_step(ppStmt);
    while (rc == SQLITE_ROW) {
      int index = 0;
      int id = sqlite3_column_int(ppStmt, index++);
      const unsigned char *p = sqlite3_column_text(ppStmt, index++);
      if (p) {
        std::string label = reinterpret_cast<const char *>(p);
        if (!label.empty()) {
          labels.insert(std::make_pair(id, label));
        }
      }
      rc = sqlite3_step(ppStmt);
    }
    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    EXT_LOG_DEBUG("Time={:.6f}", timer.lap_seconds());
  }
}

void DBDriverSqlite3::getWeightQuery(int nodeId, int &weight) const {
  weight = 0;
  if (_ppDb) {
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::stringstream query;

    query << "SELECT weight FROM node WHERE id =  " << nodeId << ";";

    rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Process the result if one
    rc = sqlite3_step(ppStmt);
    if (rc == SQLITE_ROW) {
      weight = sqlite3_column_int(ppStmt, 0);  // weight
      rc = sqlite3_step(ppStmt);
    }
    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  }
}

// may be slower than the previous version but don't have a limit of words that can be loaded at the same time
void DBDriverSqlite3::loadNodesQuery(const std::list<int> &ids, std::list<Node *> &output, bool loadWordIdsOnly) const {
  // Publish raw-pointer API results only after every archive has decoded.
  std::vector<std::unique_ptr<Node>> owned;
  std::list<Node *> nodes;
  EXT_LOG_DEBUG("count={:d}", (int)ids.size());
  if (_ppDb && ids.size()) {
    std::string type;
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    Statement statement(nullptr, sqlite3_finalize);
    sqlite3_stmt *ppStmt = nullptr;
    std::stringstream query;
    unsigned int loaded = 0;

    // Load nodes information
    {
      query << "SELECT id, map_id, weight, pose, stamp, label, velocity, lidar_geometry "
            << "FROM Node "
            << "WHERE id=?;";
    }

    statement = prepare_statement(_ppDb, query.str().c_str());
    ppStmt = statement.get();

    for (std::list<int>::const_iterator iter = ids.begin(); iter != ids.end(); ++iter) {
      // EXT_LOG_DEBUG("Loading node %d...", *iter);
      //  bind id
      rc = sqlite3_bind_int(ppStmt, 1, *iter);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

      int id = 0;
      int mapId = 0;
      double stamp = 0.0;
      int weight = 0;
      StateGroupf pose;
      std::vector<float> velocity;
      const void *data = 0;
      int dataSize = 0;
      std::string label;

      GaussianCloudPtr lidar_cloud;
      // Process the result if one
      rc = sqlite3_step(ppStmt);
      if (rc == SQLITE_ROW) {
        int index = 0;
        id = sqlite3_column_int(ppStmt, index++);      // Node Id
        mapId = sqlite3_column_int(ppStmt, index++);   // Map Id
        weight = sqlite3_column_int(ppStmt, index++);  // weight

        data = sqlite3_column_blob(ppStmt, index);  // pose
        dataSize = sqlite3_column_bytes(ppStmt, index++);
        stateGroupFromBlob(data, dataSize, pose);

        {
          stamp = sqlite3_column_double(ppStmt, index++);                 // stamp
          const unsigned char *p = sqlite3_column_text(ppStmt, index++);  // label
          if (p) {
            label = reinterpret_cast<const char *>(p);
          }

          {
            velocity.resize(6, 0);
            data = sqlite3_column_blob(ppStmt, index);  // velocity
            dataSize = sqlite3_column_bytes(ppStmt, index++);
            if ((unsigned int)dataSize == velocity.size() * sizeof(float) && data) {
              memcpy(velocity.data(), data, dataSize);
            }
          }
        }

        if (sqlite3_column_type(ppStmt, 7) != SQLITE_NULL) {
          const auto bytes = sqlite3_column_bytes(ppStmt, 7);
          if (bytes <= 0) throw std::runtime_error("Empty LiDAR geometry record");
          lidar_cloud = lidar::decode(cv::Mat(1, bytes, CV_8UC1, const_cast<void *>(sqlite3_column_blob(ppStmt, 7))));
        }
        rc = sqlite3_step(ppStmt);
      }
      EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

      // create the node
      if (id) {
        // EXT_LOG_DEBUG("Creating %d (map=%d, pose=%s)", *iter, mapId, pose.debugString().c_str());
        auto owner = std::make_unique<Node>(id, mapId, weight, stamp, label, pose);
        owner->set_lidar_cloud(std::move(lidar_cloud));
        Node *s = owner.get();
        owned.push_back(std::move(owner));
        if (velocity.size() == 6) {
          s->setVelocity(velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]);
        }
        s->setSaved(true);
        nodes.push_back(s);
        ++loaded;
      } else {
        EXT_LOG_ERROR("Node {:d} not found in database!", *iter);
      }

      // reset
      rc = sqlite3_reset(ppStmt);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    }

    // Finalize (delete) the statement
    rc = sqlite3_finalize(statement.release());
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());

    // Prepare the query... Get the map from node and visual words
    EXT_LOG_DEBUG("Loading local features (ids only={})....", loadWordIdsOnly ? "true" : "false");
    if (loadWordIdsOnly) {
      this->loadWordIdsQuery(nodes);
    } else {
      this->loadWordsQuery(nodes);
    }
    EXT_LOG_DEBUG("Loading local features.... done! (in {:.6f} s)", timer.lap_seconds());

    this->loadLinksQuery(nodes);
    EXT_LOG_DEBUG("Time loading links={:.6f}s", timer.lap_seconds());

    for (std::list<Node *>::iterator iter = nodes.begin(); iter != nodes.end(); ++iter) {
      (*iter)->setModified(false);
    }

    // load calibrations
    if (nodes.size()) {
      std::stringstream query3;
      query3 << "SELECT calibration "
                "FROM Data "
                "WHERE id = ? ";

      rc = sqlite3_prepare_v2(_ppDb, query3.str().c_str(), -1, &ppStmt, 0);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

      for (std::list<Node *>::const_iterator iter = nodes.begin(); iter != nodes.end(); ++iter) {
        // bind id
        rc = sqlite3_bind_int(ppStmt, 1, (*iter)->id());
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

        rc = sqlite3_step(ppStmt);
        if (rc == SQLITE_ROW) {
          int index = 0;
          const void *data = 0;
          int dataSize = 0;
          StateGroupf localTransform;
          std::vector<camera::Perspective> models;

          // calibration
          data = sqlite3_column_blob(ppStmt, index);
          dataSize = sqlite3_column_bytes(ppStmt, index++);
          if (dataSize > 0 && data) {
            EXT_ASSERT_MSG(decodePerspectiveCalibrations(data, dataSize, models), "Invalid Perspective calibration field (size={:d} bytes)",
                           dataSize);

            auto frame = (*iter)->visualFrame();
            frame.setPerspectives(std::move(models));
            (*iter)->set_reference_frame(std::move(frame));
          }
          rc = sqlite3_step(ppStmt);
        }
        EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

        // reset
        rc = sqlite3_reset(ppStmt);
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      }
      // Finalize (delete) the statement
      rc = sqlite3_finalize(statement.release());
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

      EXT_LOG_DEBUG("Time load {:d} calibrations={:.6f}s", (int)nodes.size(), timer.lap_seconds());
    }

    // load global descriptors
    if (nodes.size()) {
      std::stringstream query3;
      query3 << "SELECT type, info, data "
                "FROM GlobalDescriptor "
                "WHERE node_id = ? ";

      rc = sqlite3_prepare_v2(_ppDb, query3.str().c_str(), -1, &ppStmt, 0);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

      for (std::list<Node *>::const_iterator iter = nodes.begin(); iter != nodes.end(); ++iter) {
        // bind id
        rc = sqlite3_bind_int(ppStmt, 1, (*iter)->id());
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

        std::vector<GlobalDescriptor> globalDescriptors;

        rc = sqlite3_step(ppStmt);
        while (rc == SQLITE_ROW) {
          int index = 0;
          const void *data = 0;
          int dataSize = 0;
          int type = -1;
          cv::Mat info;
          cv::Mat dataMat;

          type = sqlite3_column_int(ppStmt, index++);
          data = sqlite3_column_blob(ppStmt, index);
          dataSize = sqlite3_column_bytes(ppStmt, index++);
          if (dataSize && data) {
            info = sapphire::backend::uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone());
          }
          data = sqlite3_column_blob(ppStmt, index);
          dataSize = sqlite3_column_bytes(ppStmt, index++);
          if (dataSize && data) {
            dataMat = sapphire::backend::uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)data).clone());
          }

          EXT_ASSERT(!dataMat.empty());
          globalDescriptors.push_back(GlobalDescriptor(type, dataMat, info));

          rc = sqlite3_step(ppStmt);
        }
        EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

        if (!globalDescriptors.empty()) {
          (*iter)->set_global_descriptors(std::move(globalDescriptors));
          EXT_LOG_DEBUG("Add {:d} global descriptors to node {:d}", (int)globalDescriptors.size(), (*iter)->id());
        }

        // reset
        rc = sqlite3_reset(ppStmt);
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      }
      // Finalize (delete) the statement
      rc = sqlite3_finalize(statement.release());
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

      EXT_LOG_DEBUG("Time load {:d} global descriptors={:.6f}s", (int)nodes.size(), timer.lap_seconds());
    }

    if (ids.size() != loaded) {
      EXT_LOG_ERROR("Some nodes not found in database");
    }
  }
  output.splice(output.end(), nodes);
  for (auto &node : owned) node.release();
}

void DBDriverSqlite3::loadLastNodesQuery(std::list<Node *> &nodes, bool loadWordIdsOnly) const {
  EXT_LOG_DEBUG("");
  if (_ppDb) {
    std::string type;
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::string query;
    std::list<int> ids;

    {
      query =
          "SELECT n.id "
          "FROM Node AS n JOIN WorkingMemory AS w ON w.node_id=n.id "
          "ORDER BY n.id;";
    }

    rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Process the result if one
    rc = sqlite3_step(ppStmt);
    while (rc == SQLITE_ROW) {
      ids.push_back(sqlite3_column_int(ppStmt, 0));  // Node id
      rc = sqlite3_step(ppStmt);                     // next result...
    }

    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    EXT_LOG_DEBUG("Loading {:d} nodes...", (int)ids.size());
    this->loadNodesQuery(ids, nodes, loadWordIdsOnly);
    EXT_LOG_DEBUG("loaded={:d}, Time={:.6f}s", (int)nodes.size(), timer.lap_seconds());
  }
}

void DBDriverSqlite3::loadWordIdsQuery(std::list<Node *> &nodes) const {
  if (!_ppDb) return;
  auto statement = prepare_statement(_ppDb, "SELECT words, feature_count, vocabulary_hash, kind, retrieval_mass FROM Feature WHERE node_id=?");
  for (auto *node : nodes) {
    check_sqlite(_ppDb, sqlite3_bind_int(statement.get(), 1, node->id()));
    const int result = sqlite3_step(statement.get());
    if (result == SQLITE_ROW) {
      auto words = decode_words(sqlite3_column_blob(statement.get(), 0), sqlite3_column_bytes(statement.get(), 0));
      const int kind = sqlite3_column_int(statement.get(), 3);
      if (kind == 1)
        node->set_feature_summary(std::move(words), read_vocabulary_hash(statement.get(), 2), sqlite3_column_int(statement.get(), 1));
      else if (kind == 0) {
        const auto hash = read_vocabulary_hash(statement.get(), 2);
        if (!hash && words.empty())
          node->set_raw_summary(sqlite3_column_int(statement.get(), 1));
        else
          node->set_retrieval_words(std::move(words), hash);
      } else
        throw std::runtime_error("Unknown feature record kind");
      if (node->retrieval_mass() != static_cast<uint32_t>(sqlite3_column_int(statement.get(), 4)))
        throw std::runtime_error("Inconsistent retrieval mass");
      if (node->feature_count() != static_cast<size_t>(sqlite3_column_int(statement.get(), 1)))
        throw std::runtime_error("Inconsistent BoW feature count");

    } else
      check_sqlite(_ppDb, result);
    check_sqlite(_ppDb, sqlite3_reset(statement.get()));
  }
}

void DBDriverSqlite3::visit_retrieval_summaries_query(const RetrievalVisitor &visitor) const {
  if (!_ppDb) return;
  auto query = prepare_statement(_ppDb,
                                 "SELECT f.node_id,f.words,f.feature_count,f.vocabulary_hash,f.kind,f.retrieval_mass "
                                 "FROM Feature f JOIN Node n ON n.id=f.node_id WHERE n.weight>=0 ORDER BY f.node_id");
  int status;
  while ((status = sqlite3_step(query.get())) == SQLITE_ROW) {
    Node summary(sqlite3_column_int(query.get(), 0));
    auto words = decode_words(sqlite3_column_blob(query.get(), 1), sqlite3_column_bytes(query.get(), 1));
    const auto hash = read_vocabulary_hash(query.get(), 3);
    const int kind = sqlite3_column_int(query.get(), 4);
    if (kind == 1)
      summary.set_feature_summary(std::move(words), hash, sqlite3_column_int(query.get(), 2));
    else if (kind == 0) {
      if (!hash && words.empty())
        summary.set_raw_summary(sqlite3_column_int(query.get(), 2));
      else
        summary.set_retrieval_words(std::move(words), hash);
    } else
      throw std::runtime_error("Unknown retrieval summary kind");
    if (summary.retrieval_mass() != uint32_t(sqlite3_column_int(query.get(), 5)) ||
        summary.feature_count() != size_t(sqlite3_column_int(query.get(), 2)))
      throw std::runtime_error("Inconsistent global retrieval summary");
    visitor(summary);
  }
  check_sqlite(_ppDb, status);
}

void DBDriverSqlite3::visit_global_descriptors_query(const RetrievalVisitor &visitor) const {
  if (!_ppDb) return;
  auto query = prepare_statement(_ppDb,
                                 "SELECT g.node_id,g.info,g.data FROM GlobalDescriptor g JOIN Node n ON n.id=g.node_id "
                                 "WHERE n.weight>=0 AND g.type=2 ORDER BY g.node_id");
  int status;
  while ((status = sqlite3_step(query.get())) == SQLITE_ROW) {
    const auto decode = [&](int column) {
      const auto *bytes = sqlite3_column_blob(query.get(), column);
      const int size = sqlite3_column_bytes(query.get(), column);
      return size && bytes ? sapphire::backend::uncompressData(cv::Mat(1, size, CV_8U, const_cast<void *>(bytes)).clone()) : cv::Mat{};
    };
    Node summary(sqlite3_column_int(query.get(), 0));
    summary.set_global_descriptors({GlobalDescriptor(2, decode(2), decode(1))});
    visitor(summary);
  }
  check_sqlite(_ppDb, status);
}

std::shared_ptr<const bow::FeatureBlock> DBDriverSqlite3::load_loop_features(int node_id) const {
  if (!_ppDb) return {};
  auto statement = prepare_statement(_ppDb, "SELECT block, feature_count, vocabulary_hash, kind FROM Feature WHERE node_id=?");
  check_sqlite(_ppDb, sqlite3_bind_int(statement.get(), 1, node_id));
  const int result = sqlite3_step(statement.get());
  if (result != SQLITE_ROW) {
    check_sqlite(_ppDb, result);
    return {};
  }
  if (sqlite3_column_int(statement.get(), 3) == 1) return {};
  if (sqlite3_column_int(statement.get(), 3) != 0) throw std::runtime_error("Unknown feature record kind");
  if (sqlite3_column_type(statement.get(), 0) == SQLITE_NULL) return {};
  auto block = bow::FeatureBlock::decode(sqlite3_column_blob(statement.get(), 0), sqlite3_column_bytes(statement.get(), 0));
  if (block->keypoints.size() != static_cast<size_t>(sqlite3_column_int(statement.get(), 1)))
    throw std::runtime_error("Inconsistent BoW feature block count");
  if (block->vocabulary_hash != read_vocabulary_hash(statement.get(), 2)) throw std::runtime_error("Inconsistent BoW vocabulary hash");
  return block;
}

std::shared_ptr<const featmap::LandmarkMap> DBDriverSqlite3::load_map_features(int node_id) const {
  if (!_ppDb) return {};
  auto statement = prepare_statement(_ppDb, "SELECT block, kind, words, feature_count, vocabulary_hash, retrieval_mass FROM Feature WHERE node_id=?");
  check_sqlite(_ppDb, sqlite3_bind_int(statement.get(), 1, node_id));
  const int result = sqlite3_step(statement.get());
  if (result != SQLITE_ROW) {
    check_sqlite(_ppDb, result);
    return {};
  }
  const int kind = sqlite3_column_int(statement.get(), 1);
  if (kind == 0) return {};
  if (kind != 1) throw std::runtime_error("Unknown feature record kind");
  auto map = featmap::LandmarkMap::decode(sqlite3_column_blob(statement.get(), 0), sqlite3_column_bytes(statement.get(), 0));
  Node metadata(node_id);
  metadata.set_feature_summary(decode_words(sqlite3_column_blob(statement.get(), 2), sqlite3_column_bytes(statement.get(), 2)),
                               read_vocabulary_hash(statement.get(), 4), sqlite3_column_int(statement.get(), 3));
  if (metadata.retrieval_mass() != static_cast<uint32_t>(sqlite3_column_int(statement.get(), 5)))
    throw std::runtime_error("Inconsistent retrieval mass");
  if (metadata.feature_count() != static_cast<size_t>(sqlite3_column_int(statement.get(), 3)))
    throw std::runtime_error("Inconsistent BoW feature count");
  metadata.cache_map_features(map);  // Verify count, vocabulary and every posting against the payload.
  return map;
}

void DBDriverSqlite3::loadWordsQuery(std::list<Node *> &nodes) const {
  for (auto *node : nodes) {
    if (auto submap = load_map_features(node->id())) {
      node->set_map_features(std::move(submap));
      continue;
    }
    auto block = load_loop_features(node->id());
    if (block)
      node->set_loop_features(std::move(block));
    else {
      std::list<Node *> one{node};
      loadWordIdsQuery(one);
    }
  }
}

void DBDriverSqlite3::loadLinksQuery(int nodeId, std::multimap<int, Link> &links, std::optional<Link::Type> type) const {
  links.clear();
  if (!_ppDb) return;
  auto statement = prepare_statement(_ppDb,
                                     "SELECT from_id,to_id,type,transform,information_matrix FROM Link "
                                     "WHERE from_id=?1 AND (?2 IS NULL OR type=?2) ORDER BY to_id");
  check_sqlite(_ppDb, sqlite3_bind_int(statement.get(), 1, nodeId));
  check_sqlite(_ppDb, type ? sqlite3_bind_int(statement.get(), 2, *type) : sqlite3_bind_null(statement.get(), 2));
  int result;
  while ((result = sqlite3_step(statement.get())) == SQLITE_ROW) {
    auto link = read_link(statement.get());
    links.emplace(link->to(), std::move(*link));
  }
  check_sqlite(_ppDb, result);
}

void DBDriverSqlite3::loadLinksQuery(std::list<Node *> &nodes) const {
  if (!_ppDb) return;
  auto statement = prepare_statement(_ppDb, "SELECT from_id,to_id,type,transform,information_matrix FROM Link WHERE from_id=?1 ORDER BY to_id");
  for (auto *node : nodes) {
    check_sqlite(_ppDb, sqlite3_bind_int(statement.get(), 1, node->id()));
    std::list<Link> links;
    int result;
    while ((result = sqlite3_step(statement.get())) == SQLITE_ROW) {
      auto link = read_link(statement.get());
      links.push_back(std::move(*link));
    }
    check_sqlite(_ppDb, result);
    node->addLinks(links);
    check_sqlite(_ppDb, sqlite3_reset(statement.get()));
  }
}

void DBDriverSqlite3::updateQuery(const std::list<Node *> &nodes) const {
  EXT_LOG_DEBUG("nodes = {:d}", (int)nodes.size());
  if (_ppDb && nodes.size()) {
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    Node *s = 0;

    std::string query;
    { query = "UPDATE Node SET weight=?, label=? WHERE id=?;"; }
    rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    for (std::list<Node *>::const_iterator i = nodes.begin(); i != nodes.end(); ++i) {
      s = *i;
      int index = 1;
      if (s && s->isModified()) {
        rc = sqlite3_bind_int(ppStmt, index++, s->getWeight());
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

        {
          if (s->getLabel().empty()) {
            rc = sqlite3_bind_null(ppStmt, index++);
            EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
          } else {
            rc = sqlite3_bind_text(ppStmt, index++, s->getLabel().c_str(), -1, SQLITE_STATIC);
            EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
          }
        }

        rc = sqlite3_bind_int(ppStmt, index++, s->id());
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

        // step
        rc = sqlite3_step(ppStmt);
        EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {} (node id = {:d} map={:d})", _version.c_str(), sqlite3_errmsg(_ppDb), s->id(),
                       s->mapId());

        rc = sqlite3_reset(ppStmt);
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      }
    }
    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    EXT_LOG_DEBUG("Update Node table, Time={:.6f}s", timer.lap_seconds());

    // Update links part1
    query = fmt::format("DELETE FROM Link WHERE from_id=?;");
    rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    for (std::list<Node *>::const_iterator j = nodes.begin(); j != nodes.end(); ++j) {
      if ((*j)->isLinksModified()) {
        rc = sqlite3_bind_int(ppStmt, 1, (*j)->id());
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

        rc = sqlite3_step(ppStmt);
        EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

        rc = sqlite3_reset(ppStmt);
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      }
    }
    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Update links part2
    query = queryStepLink();
    rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    for (std::list<Node *>::const_iterator j = nodes.begin(); j != nodes.end(); ++j) {
      if ((*j)->isLinksModified()) {
        // Save links
        const std::multimap<int, Link> &links = (*j)->getLinks();
        for (std::multimap<int, Link>::const_iterator i = links.begin(); i != links.end(); ++i) {
          stepLink(ppStmt, i->second);
        }
      }
    }
    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    EXT_LOG_DEBUG("Update Neighbors Time={:.6f}s", timer.lap_seconds());

    write_features(nodes, true);
    EXT_LOG_DEBUG("nodes update={:.6f}s", timer.lap_seconds());
  }
}

void DBDriverSqlite3::write_features(const std::list<Node *> &nodes, bool update, bool store_history) const {
  auto statement =
      prepare_statement(_ppDb,
                        "INSERT INTO Feature(node_id,feature_count,words,block,vocabulary_hash,kind,retrieval_mass) VALUES(?,?,?,?,?,?,?) "
                        "ON CONFLICT(node_id) DO UPDATE SET feature_count=excluded.feature_count,words=excluded.words,block=excluded.block,"
                        "vocabulary_hash=excluded.vocabulary_hash,kind=excluded.kind,retrieval_mass=excluded.retrieval_mass");
  for (const auto *node : nodes) {
    if (update && node->features_erased()) {
      auto erase = prepare_statement(_ppDb, "DELETE FROM Feature WHERE node_id=?");
      check_sqlite(_ppDb, sqlite3_bind_int(erase.get(), 1, node->id()));
      check_sqlite(_ppDb, sqlite3_step(erase.get()));
      continue;
    }
    if (update && node->has_map_features() && !node->feature_modified()) continue;
    if (node->has_map_features() && !node->map_features()) {
      throw std::runtime_error("Cannot write a new or modified submap without its feature bank");
    }
    if (!node->has_map_features() && (!node->feature_count() || (update && !node->loop_features()))) continue;
    // Encode before writing either half. A resident observation history and its points belong
    // to the same node transaction; cold metadata updates preserve both rows.
    const auto observations = store_history && node->has_observation_history() ? node->observation_history().encode() : cv::Mat();
    const auto words = encode_words(node->retrieval_words());
    const auto block = node->has_map_features() ? node->map_features()->encode()
                       : node->loop_features()  ? node->loop_features()->encode()
                                                : std::vector<uint8_t>();
    check_sqlite(_ppDb, sqlite3_bind_int(statement.get(), 1, node->id()));
    check_sqlite(_ppDb, sqlite3_bind_int(statement.get(), 2, node->feature_count()));
    check_sqlite(_ppDb, words.empty() ? sqlite3_bind_zeroblob(statement.get(), 3, 0)
                                      : sqlite3_bind_blob(statement.get(), 3, words.data(), words.size(), SQLITE_STATIC));
    check_sqlite(_ppDb, block.empty() ? sqlite3_bind_null(statement.get(), 4)
                                      : sqlite3_bind_blob(statement.get(), 4, block.data(), block.size(), SQLITE_STATIC));
    const auto hash = std::to_string(node->vocabulary_hash());
    check_sqlite(_ppDb, sqlite3_bind_text(statement.get(), 5, hash.c_str(), hash.size(), SQLITE_STATIC));
    check_sqlite(_ppDb, sqlite3_bind_int(statement.get(), 6, node->has_map_features() ? 1 : 0));
    check_sqlite(_ppDb, sqlite3_bind_int(statement.get(), 7, node->retrieval_mass()));
    check_sqlite(_ppDb, sqlite3_step(statement.get()));
    check_sqlite(_ppDb, sqlite3_reset(statement.get()));
    check_sqlite(_ppDb, sqlite3_clear_bindings(statement.get()));
    if (store_history && node->has_map_features()) {
      auto create = prepare_statement(_ppDb, "CREATE TABLE IF NOT EXISTS SubmapBundle(anchor_id INTEGER PRIMARY KEY,archive BLOB NOT NULL)");
      check_sqlite(_ppDb, sqlite3_step(create.get()));
      auto history = prepare_statement(_ppDb, observations.empty() ? "DELETE FROM SubmapBundle WHERE anchor_id=?1"
                                                                   : "INSERT OR REPLACE INTO SubmapBundle(anchor_id,archive) VALUES(?1,?2)");
      check_sqlite(_ppDb, sqlite3_bind_int(history.get(), 1, node->id()));
      if (!observations.empty()) check_sqlite(_ppDb, sqlite3_bind_blob(history.get(), 2, observations.ptr(), observations.total(), SQLITE_STATIC));
      check_sqlite(_ppDb, sqlite3_step(history.get()));
    }
  }
}

void DBDriverSqlite3::saveQuery(const std::list<Node *> &nodes, const std::map<int, SensorRecord> &records) {
  EXT_LOG_DEBUG("");
  if (_ppDb && nodes.size()) {
    std::string type;
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;

    // Node table
    std::string query = queryStepNode();
    rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    for (std::list<Node *>::const_iterator i = nodes.begin(); i != nodes.end(); ++i) {
      _memoryUsedEstimate += (*i)->getMemoryUsed();
      const auto record = records.find((*i)->id());
      if (record != records.end()) _memoryUsedEstimate += record->second.archive.memoryUsed();
      static const EnvSensors noSensors;
      stepNode(ppStmt, *i, record != records.end() ? record->second.observation.envSensors() : noSensors);
    }
    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());

    // Create new entries in table Link
    query = queryStepLink();
    rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    for (std::list<Node *>::const_iterator jter = nodes.begin(); jter != nodes.end(); ++jter) {
      // Save links
      const std::multimap<int, Link> &links = (*jter)->getLinks();
      for (std::multimap<int, Link>::const_iterator i = links.begin(); i != links.end(); ++i) {
        stepLink(ppStmt, i->second);
      }
    }
    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());

    write_features(nodes, false);

    {
      // Global descriptor table
      std::string query = queryStepGlobalDescriptor();
      rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

      for (std::list<Node *>::const_iterator i = nodes.begin(); i != nodes.end(); ++i) {
        for (size_t d = 0; d < (*i)->featmap().global_descriptors().size(); ++d) {
          stepGlobalDescriptor(ppStmt, (*i)->id(), (*i)->featmap().global_descriptors()[d]);
        }
      }
      // Finalize (delete) the statement
      rc = sqlite3_finalize(ppStmt);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

      EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());
    }

    {
      // Add SensorData
      query = queryStepSensorData();
      rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      EXT_LOG_DEBUG("Saving {:d} images", (int)nodes.size());

      for (std::list<Node *>::const_iterator i = nodes.begin(); i != nodes.end(); ++i) {
        auto found = records.find((*i)->id());
        SensorRecord payload;
        if (found != records.end()) payload = found->second;
        payload.observation.setId((*i)->id());
        if (!(*i)->visualFrame().perspectives().empty()) payload.observation.setPerspectives((*i)->visualFrame().perspectives());
        if ((*i)->has_map_features()) payload.clearCompressedData(true, false, false);
        stepSensorData(ppStmt, payload);
      }

      // Finalize (delete) the statement
      rc = sqlite3_finalize(ppStmt);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());
    }

    EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());
  }
}

void DBDriverSqlite3::addLinkQuery(const Link &link) const {
  if (!_ppDb) return;
  auto statement = prepare_statement(_ppDb, queryStepLink().c_str());
  stepLink(statement.get(), link);
}

void DBDriverSqlite3::updateLinkQuery(const Link &link) const {
  if (!_ppDb) return;
  auto statement = prepare_statement(_ppDb, queryStepLinkUpdate().c_str());
  stepLink(statement.get(), link);
}

void DBDriverSqlite3::updateCalibrationQuery(int nodeId, const std::vector<camera::Perspective> &models) const {
  EXT_LOG_DEBUG("");
  if (_ppDb) {
    std::string type;
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;

    // Create query
    std::string query = queryStepCalibrationUpdate();
    rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // step calibration
    stepCalibrationUpdate(ppStmt, nodeId, models);

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());
  }
}

void DBDriverSqlite3::updateDepthImageQuery(int nodeId, const cv::Mat &image) const {
  EXT_LOG_DEBUG("");
  if (_ppDb) {
    std::string type;
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;

    // Create query
    std::string query = queryStepDepthUpdate();
    rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Save depth
    stepDepthUpdate(ppStmt, nodeId, image);

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());
  }
}

void DBDriverSqlite3::addStatisticsQuery(const Statistics &statistics, bool saveWmState) const {
  EXT_LOG_DEBUG("Ref ID = {:d}", statistics.refImageId());
  if (_ppDb) {
    std::string type;
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;

    // Create query
    {
      std::string param = Statistics::serializeData(statistics.data());
      if (param.size() && statistics.refImageId() > 0) {
        std::string query;
        {
          query =
              "INSERT INTO Statistics(id, stamp, data, wm_state) values(?,?,?,?) ON CONFLICT(id) DO UPDATE SET "
              "stamp=excluded.stamp,data=excluded.data,wm_state=excluded.wm_state;";
        }
        rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

        int index = 1;
        rc = sqlite3_bind_int(ppStmt, index++, statistics.refImageId());
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
        rc = sqlite3_bind_double(ppStmt, index++, statistics.stamp());
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

        cv::Mat compressedParam;
        {
          compressedParam = compressString(param);
          rc = sqlite3_bind_blob(ppStmt, index++, compressedParam.data, compressedParam.cols, SQLITE_STATIC);
          EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
        }

        cv::Mat compressedWmState;
        {
          if (saveWmState && !statistics.wmState().empty()) {
            compressedWmState = compressData(cv::Mat(1, statistics.wmState().size(), CV_32SC1, (void *)statistics.wmState().data()));
            rc = sqlite3_bind_blob(ppStmt, index++, compressedWmState.data, compressedWmState.cols, SQLITE_STATIC);
            EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
          } else {
            rc = sqlite3_bind_null(ppStmt, index++);
            EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
          }
        }

        // step
        rc = sqlite3_step(ppStmt);
        EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

        rc = sqlite3_reset(ppStmt);
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

        // Finalize (delete) the statement
        rc = sqlite3_finalize(ppStmt);
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

        EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());
      }
    }
  }
}

void DBDriverSqlite3::savePreviewImageQuery(const cv::Mat &image) const {
  EXT_LOG_DEBUG("");
  if (_ppDb) {
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::string query;

    // Update table Admin
    query = fmt::format("UPDATE Admin SET preview_image=? WHERE version='{}';", _version.c_str());
    rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    int index = 1;
    cv::Mat compressedImage;
    if (image.empty()) {
      rc = sqlite3_bind_null(ppStmt, index);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    } else {
      // compress
      if (image.rows == 1 && image.type() == CV_8UC1) {
        // already compressed
        compressedImage = image;
      } else {
        compressedImage = compressImage(image);
      }
      rc = sqlite3_bind_blob(ppStmt, index++, compressedImage.data, compressedImage.cols, SQLITE_STATIC);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    }

    // execute query
    rc = sqlite3_step(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());
  }
}
cv::Mat DBDriverSqlite3::loadPreviewImageQuery() const {
  EXT_LOG_DEBUG("");
  cv::Mat image;
  if (_ppDb) {
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::stringstream query;

    query << "SELECT preview_image "
          << "FROM Admin "
          << "WHERE version='" << _version.c_str() << "';";

    rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Process the result if one
    rc = sqlite3_step(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_ROW, "DB error ({}): Not found first Admin row: query=\"{}\"", _version.c_str(), query.str().c_str());
    if (rc == SQLITE_ROW) {
      const void *data = 0;
      int dataSize = 0;
      int index = 0;

      // preview_image
      data = sqlite3_column_blob(ppStmt, index);
      dataSize = sqlite3_column_bytes(ppStmt, index++);
      if (dataSize > 0 && data) {
        image = uncompressImage(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
      }
      EXT_LOG_DEBUG("Image={:d}x{:d}", image.cols, image.rows);

      rc = sqlite3_step(ppStmt);  // next result...
    }
    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());
  }
  return image;
}

std::map<int, RootFrame> DBDriverSqlite3::load_root_frames_query() const {
  std::map<int, RootFrame> frames;
  auto exists = prepare_statement(_ppDb, "SELECT 1 FROM sqlite_master WHERE name='MapFrame' AND type='table'");
  if (sqlite3_step(exists.get()) != SQLITE_ROW) return frames;
  auto rows = prepare_statement(_ppDb,
                                "SELECT f.frame_id,f.anchor_id,f.pose,n.id,n.weight,f.creation_order FROM MapFrame f LEFT JOIN Node n ON "
                                "n.id=f.anchor_id ORDER BY f.anchor_id");
  int rc;
  while ((rc = sqlite3_step(rows.get())) == SQLITE_ROW) {
    RootFrame frame;
    const auto *id = sqlite3_column_text(rows.get(), 0);
    if (!id || sqlite3_column_type(rows.get(), 3) == SQLITE_NULL || sqlite3_column_int(rows.get(), 4) < 0)
      throw std::runtime_error("Component datum missing or retired; refusing to choose a replacement");
    frame.id = reinterpret_cast<const char *>(id);
    frame.anchor = sqlite3_column_int(rows.get(), 1);
    frame.creation_order = sqlite3_column_int64(rows.get(), 5);
    if (!stateGroupFromBlob(sqlite3_column_blob(rows.get(), 2), sqlite3_column_bytes(rows.get(), 2), frame.frame_from_anchor))
      throw std::runtime_error("Corrupt component datum pose");
    frame.validate();
    if (!frames.emplace(frame.anchor, frame).second) throw std::runtime_error("Duplicate component datum");
  }
  check_sqlite(_ppDb, rc);
  if (frames.empty()) throw std::runtime_error("Empty component frame ledger");
  return frames;
}

RootFrame DBDriverSqlite3::create_root_frame_query(int anchor, const StateGroupf &pose) {
  const auto existing = load_root_frames_query();
  if (existing.count(anchor)) {
    if ((existing.at(anchor).frame_from_anchor.matrix() - pose.matrix()).norm() > 1e-6f)
      throw std::runtime_error("Cannot redefine a component datum");
    return existing.at(anchor);
  }
  auto node = prepare_statement(_ppDb, "SELECT weight FROM Node WHERE id=?1");
  check_sqlite(_ppDb, sqlite3_bind_int(node.get(), 1, anchor));
  if (sqlite3_step(node.get()) != SQLITE_ROW || sqlite3_column_int(node.get(), 0) < 0)
    throw std::runtime_error("Cannot establish component frame without a saved keyframe");
  node.reset();
  RootFrame frame;
  frame.anchor = anchor;
  frame.frame_from_anchor = pose;
  frame.creation_order = 1;
  for (const auto &entry : existing) frame.creation_order = std::max(frame.creation_order, entry.second.creation_order + 1);
  auto uuid = prepare_statement(_ppDb, "SELECT 'mf_' || lower(hex(randomblob(16)))");
  if (sqlite3_step(uuid.get()) != SQLITE_ROW) throw std::runtime_error("Cannot allocate frame identity");
  frame.id = reinterpret_cast<const char *>(sqlite3_column_text(uuid.get(), 0));
  frame.validate();
  uuid.reset();
  auto run = [&](const char *sql) { check_sqlite(_ppDb, sqlite3_exec(_ppDb, sql, nullptr, nullptr, nullptr)); };
  run("SAVEPOINT component_frame");
  try {
    run("CREATE TABLE IF NOT EXISTS MapFrame(frame_id TEXT PRIMARY KEY,anchor_id INTEGER NOT NULL UNIQUE,pose BLOB NOT NULL,creation_order INTEGER "
        "NOT NULL UNIQUE CHECK(creation_order>0))");
    auto row = prepare_statement(_ppDb, "INSERT INTO MapFrame(frame_id,anchor_id,pose,creation_order) VALUES(?1,?2,?3,?4)");
    check_sqlite(_ppDb, sqlite3_bind_text(row.get(), 1, frame.id.c_str(), -1, SQLITE_TRANSIENT));
    check_sqlite(_ppDb, sqlite3_bind_int(row.get(), 2, anchor));
    check_sqlite(_ppDb, sqlite3_bind_int64(row.get(), 4, frame.creation_order));
    const auto coefficients = copyStateGroupCoefficients(pose);
    check_sqlite(_ppDb, sqlite3_bind_blob(row.get(), 3, coefficients.data(), sizeof(coefficients), SQLITE_TRANSIENT));
    check_sqlite(_ppDb, sqlite3_step(row.get()));
    row.reset();
    run("CREATE TRIGGER IF NOT EXISTS MapFrame_no_delete BEFORE DELETE ON MapFrame BEGIN SELECT RAISE(ABORT,'Immutable component frame'); END");
    run("CREATE TRIGGER IF NOT EXISTS MapFrame_no_update BEFORE UPDATE ON MapFrame BEGIN SELECT RAISE(ABORT,'Immutable component frame'); END");
    run("CREATE TRIGGER IF NOT EXISTS MapFrame_keep_anchor BEFORE DELETE ON Node WHEN EXISTS(SELECT 1 FROM MapFrame WHERE anchor_id=OLD.id) BEGIN "
        "SELECT RAISE(ABORT,'Protected component datum'); END");
    run("CREATE TRIGGER IF NOT EXISTS MapFrame_keep_anchor_active BEFORE UPDATE OF id,weight ON Node WHEN EXISTS(SELECT 1 FROM MapFrame WHERE "
        "anchor_id=OLD.id) AND (NEW.id!=OLD.id OR NEW.weight<0) BEGIN SELECT RAISE(ABORT,'Protected component datum'); END");
    run("RELEASE component_frame");
  } catch (...) {
    sqlite3_exec(_ppDb, "ROLLBACK TO component_frame; RELEASE component_frame", nullptr, nullptr, nullptr);
    throw;
  }
  return frame;
}

void DBDriverSqlite3::saveOptimizedPosesQuery(const std::map<int, StateGroupf> &poses, const StateGroupf &lastPose) const {
  EXT_LOG_DEBUG("poses={:d} lastPose={}", (int)poses.size(), lastPose.debugString().c_str());
  if (_ppDb) {
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::string query;

    // Update table Admin
    query = fmt::format("UPDATE Admin SET opt_ids=?, opt_poses=?, opt_last_localization=?, time_enter = DATETIME('NOW') WHERE version='{}';",
                        _version.c_str());
    rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    int index = 1;

    // opt ids and poses
    cv::Mat compressedIds;
    cv::Mat compressedPoses;
    if (poses.empty()) {
      rc = sqlite3_bind_null(ppStmt, index++);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      rc = sqlite3_bind_null(ppStmt, index++);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    } else {
      std::vector<int> serializedIds(poses.size());
      std::vector<float> serializedPoses(poses.size() * StateGroupf::kCoefficientCount);
      int i = 0;
      for (std::map<int, StateGroupf>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter) {
        serializedIds[i] = iter->first;
        const StateGroupf::Coefficients coefficients = copyStateGroupCoefficients(iter->second);
        std::copy(coefficients.begin(), coefficients.end(), serializedPoses.begin() + StateGroupf::kCoefficientCount * i);
        ++i;
      }

      compressedIds = compressData(cv::Mat(1, serializedIds.size(), CV_32SC1, serializedIds.data()));
      compressedPoses = compressData(cv::Mat(1, serializedPoses.size(), CV_32FC1, serializedPoses.data()));

      rc = sqlite3_bind_blob(ppStmt, index++, compressedIds.data, compressedIds.cols, SQLITE_STATIC);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      rc = sqlite3_bind_blob(ppStmt, index++, compressedPoses.data, compressedPoses.cols, SQLITE_STATIC);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    }

    StateGroupf::Coefficients lastPoseCoefficients{};
    if (lastPose.isNull()) {
      rc = sqlite3_bind_null(ppStmt, index++);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    } else {
      EXT_LOG_DEBUG("lastPose={}", lastPose.debugString().c_str());
      lastPoseCoefficients = copyStateGroupCoefficients(lastPose);
      rc = sqlite3_bind_blob(ppStmt, index++, lastPoseCoefficients.data(), sizeof(lastPoseCoefficients), SQLITE_STATIC);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    }

    // execute query
    rc = sqlite3_step(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());
  }
}

std::map<int, StateGroupf> DBDriverSqlite3::loadOptimizedPosesQuery(StateGroupf *lastPose) const {
  EXT_LOG_DEBUG("");
  std::map<int, StateGroupf> poses;
  if (_ppDb) {
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::stringstream query;

    query << "SELECT opt_ids, opt_poses, opt_last_localization "
          << "FROM Admin "
          << "WHERE version='" << _version.c_str() << "';";

    rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Process the result if one
    rc = sqlite3_step(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_ROW, "DB error ({}): Not found first Admin row: query=\"{}\"", _version.c_str(), query.str().c_str());
    if (rc == SQLITE_ROW) {
      const void *data = 0;
      int dataSize = 0;
      int index = 0;

      // opt_poses
      cv::Mat serializedIds;
      data = sqlite3_column_blob(ppStmt, index);
      dataSize = sqlite3_column_bytes(ppStmt, index++);
      if (dataSize > 0 && data) {
        serializedIds = uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
        EXT_LOG_DEBUG("serializedIds={:d}", serializedIds.cols);
      }

      data = sqlite3_column_blob(ppStmt, index);
      dataSize = sqlite3_column_bytes(ppStmt, index++);
      if (dataSize > 0 && data) {
        cv::Mat serializedPoses = uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
        EXT_LOG_DEBUG("serializedPoses={:d}", serializedPoses.cols);

        EXT_ASSERT(serializedIds.cols == serializedPoses.cols / static_cast<int>(StateGroupf::kCoefficientCount));
        EXT_ASSERT(serializedPoses.type() == CV_32FC1);
        EXT_ASSERT(serializedIds.type() == CV_32SC1);
        for (int i = 0; i < serializedIds.cols; ++i) {
          StateGroupf::Coefficients coefficients{};
          const float *serializedPose = serializedPoses.ptr<float>() + i * StateGroupf::kCoefficientCount;
          std::copy_n(serializedPose, StateGroupf::kCoefficientCount, coefficients.begin());
          StateGroupf t(coefficients);
          poses.insert(poses.end(), std::make_pair(serializedIds.at<int>(i), t));
          // EXT_LOG_DEBUG("Optimized pose %d: %s", serializedIds.at<int>(i), t.debugString().c_str());
        }
      }

      data = sqlite3_column_blob(ppStmt, index);  // opt_last_localization
      dataSize = sqlite3_column_bytes(ppStmt, index++);
      if (lastPose) {
        stateGroupFromBlob(data, dataSize, *lastPose);
        EXT_LOG_DEBUG("lastPose={}", lastPose->debugString().c_str());
      }

      rc = sqlite3_step(ppStmt);  // next result...
    }
    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());
  }
  return poses;
}

std::set<int> DBDriverSqlite3::submap_neighbors_query(int id) const {
  std::set<int> result;
  auto statement = prepare_statement(_ppDb, "SELECT from_id FROM Link WHERE to_id=?1 AND from_id>0 AND from_id!=?1");
  check_sqlite(_ppDb, sqlite3_bind_int(statement.get(), 1, id));
  int status;
  while ((status = sqlite3_step(statement.get())) == SQLITE_ROW) result.insert(sqlite3_column_int(statement.get(), 0));
  check_sqlite(_ppDb, status);
  return result;
}

cv::Mat DBDriverSqlite3::load_submap_bundle_query(int anchor) const {
  auto exists = prepare_statement(_ppDb, "SELECT 1 FROM sqlite_master WHERE name='SubmapBundle' AND type='table'");
  if (sqlite3_step(exists.get()) != SQLITE_ROW) return {};
  auto row = prepare_statement(_ppDb, "SELECT archive FROM SubmapBundle WHERE anchor_id=?1");
  sqlite3_bind_int(row.get(), 1, anchor);
  if (sqlite3_step(row.get()) != SQLITE_ROW) return {};
  return cv::Mat(1, sqlite3_column_bytes(row.get(), 0), CV_8UC1, const_cast<void *>(sqlite3_column_blob(row.get(), 0))).clone();
}
std::map<int, int> DBDriverSqlite3::submap_members_query() const {
  std::map<int, int> out;
  auto exists = prepare_statement(_ppDb, "SELECT 1 FROM sqlite_master WHERE name='SubmapMember' AND type='table'");
  if (sqlite3_step(exists.get()) != SQLITE_ROW) return out;
  auto rows = prepare_statement(_ppDb, "SELECT node_id,anchor_id FROM SubmapMember");
  int rc;
  while ((rc = sqlite3_step(rows.get())) == SQLITE_ROW) out.emplace(sqlite3_column_int(rows.get(), 0), sqlite3_column_int(rows.get(), 1));
  check_sqlite(_ppDb, rc);
  return out;
}
void DBDriverSqlite3::refresh_submap_query(Node &target, int source, const cv::Mat &bundle) {
  auto run = [&](const char *sql) {
    char *message = nullptr;
    int code = sqlite3_exec(_ppDb, sql, nullptr, nullptr, &message);
    std::string error = message ? message : "";
    sqlite3_free(message);
    if (code != SQLITE_OK) throw std::runtime_error(error);
  };
  run("CREATE TABLE IF NOT EXISTS SubmapBundle(anchor_id INTEGER PRIMARY KEY,archive BLOB NOT NULL)");
  run("CREATE TABLE IF NOT EXISTS SubmapMember(node_id INTEGER PRIMARY KEY,anchor_id INTEGER NOT NULL)");
  auto count = prepare_statement(_ppDb, "SELECT COUNT(*) FROM Node WHERE id=?1 OR id=?2");
  sqlite3_bind_int(count.get(), 1, target.id());
  sqlite3_bind_int(count.get(), 2, source);
  if (sqlite3_step(count.get()) != SQLITE_ROW || sqlite3_column_int(count.get(), 0) != 2)
    throw std::runtime_error("Missing refresh boundary keyframe");
  auto members = submap_members_query();
  if (members.count(target.id()) || (members.count(source) && members.at(source) != target.id()))
    throw std::runtime_error("Conflicting submap ownership");
  if (!load_submap_bundle_query(source).empty() ||
      std::any_of(members.begin(), members.end(), [&](const auto &entry) { return entry.second == source; }))
    throw std::runtime_error("Mature source history requires region consolidation, not single-view refresh");
  // Preserve Node, Link, LocalMap, Trajectory and every original query feature.
  // Only the canonical feature bank, bounded pixels and retrieval role change.
  write_features(std::list<Node *>{&target}, true, false);
  auto row = prepare_statement(_ppDb, "INSERT OR REPLACE INTO SubmapBundle(anchor_id,archive) VALUES(?1,?2)");
  sqlite3_bind_int(row.get(), 1, target.id());
  sqlite3_bind_blob(row.get(), 2, bundle.ptr(), bundle.total(), SQLITE_TRANSIENT);
  check_sqlite(_ppDb, sqlite3_step(row.get()));
  auto member = prepare_statement(_ppDb, "INSERT OR REPLACE INTO SubmapMember(node_id,anchor_id) VALUES(?1,?2)");
  sqlite3_bind_int(member.get(), 1, source);
  sqlite3_bind_int(member.get(), 2, target.id());
  check_sqlite(_ppDb, sqlite3_step(member.get()));
}

void DBDriverSqlite3::consolidate_submap_query(Node &target, int source, const cv::Mat &archive, const cv::Mat &expected_target,
                                               const cv::Mat &expected_source) {
  const auto equal = [](const cv::Mat &a, const cv::Mat &b) {
    return !a.empty() && a.type() == b.type() && a.total() == b.total() && a.isContinuous() && b.isContinuous() &&
           std::memcmp(a.ptr(), b.ptr(), a.total() * a.elemSize()) == 0;
  };
  if (!equal(load_submap_bundle_query(target.id()), expected_target) || !equal(load_submap_bundle_query(source), expected_source))
    throw std::runtime_error("Mature submap changed while merge was being prepared");
  const auto members = submap_members_query();
  if (members.count(target.id()) || members.count(source)) throw std::runtime_error("Mature region owner is already retired");
  auto count = prepare_statement(_ppDb, "SELECT COUNT(*) FROM Node WHERE id=?1 OR id=?2");
  sqlite3_bind_int(count.get(), 1, target.id());
  sqlite3_bind_int(count.get(), 2, source);
  if (sqlite3_step(count.get()) != SQLITE_ROW || sqlite3_column_int(count.get(), 0) != 2) throw std::runtime_error("Missing mature region anchor");
  // Move region ownership only. Original Node/Link/LocalMap/Trajectory records
  // remain attached to their physical capture poses; no rigid graph contraction.
  write_features(std::list<Node *>{&target}, true, false);
  auto row = prepare_statement(_ppDb, "UPDATE SubmapBundle SET archive=?1 WHERE anchor_id=?2");
  sqlite3_bind_blob(row.get(), 1, archive.ptr(), archive.total(), SQLITE_TRANSIENT);
  sqlite3_bind_int(row.get(), 2, target.id());
  check_sqlite(_ppDb, sqlite3_step(row.get()));
  if (sqlite3_changes(_ppDb) != 1) throw std::runtime_error("Missing retained mature bundle");
  auto descendants = prepare_statement(_ppDb, "UPDATE SubmapMember SET anchor_id=?1 WHERE anchor_id=?2");
  sqlite3_bind_int(descendants.get(), 1, target.id());
  sqlite3_bind_int(descendants.get(), 2, source);
  check_sqlite(_ppDb, sqlite3_step(descendants.get()));
  auto member = prepare_statement(_ppDb, "INSERT INTO SubmapMember(node_id,anchor_id) VALUES(?1,?2)");
  sqlite3_bind_int(member.get(), 1, source);
  sqlite3_bind_int(member.get(), 2, target.id());
  check_sqlite(_ppDb, sqlite3_step(member.get()));
  auto erase = prepare_statement(_ppDb, "DELETE FROM SubmapBundle WHERE anchor_id=?1");
  sqlite3_bind_int(erase.get(), 1, source);
  check_sqlite(_ppDb, sqlite3_step(erase.get()));
  if (sqlite3_changes(_ppDb) != 1) throw std::runtime_error("Missing source mature bundle");
}

void DBDriverSqlite3::merge_submap_query(int retained, int removed, const std::vector<Node *> &nodes, const cv::Mat &archive,
                                         const std::map<int, StateGroupf> &poses, const StateGroupf &last_pose,
                                         const StateGroupf &retained_from_removed) {
  const auto run_id = [&](const char *sql, int id) {
    auto statement = prepare_statement(_ppDb, sql);
    check_sqlite(_ppDb, sqlite3_bind_int(statement.get(), 1, id));
    check_sqlite(_ppDb, sqlite3_step(statement.get()));
  };
  // Release a transferred label inside this same transaction before assigning it.
  run_id("UPDATE Node SET label=NULL WHERE id=?1", removed);
  // All rows are already durable after the writer barrier. Rewrite only changed
  // owners; incoming links are supplied by the complete incident-neighbor draft.
  for (auto *node : nodes) {
    auto statement = prepare_statement(_ppDb, "UPDATE Node SET weight=?1,label=?2 WHERE id=?3");
    sqlite3_bind_int(statement.get(), 1, node->getWeight());
    if (!node->getLabel().empty()) sqlite3_bind_text(statement.get(), 2, node->getLabel().c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(statement.get(), 3, node->id());
    check_sqlite(_ppDb, sqlite3_step(statement.get()));
    if (sqlite3_changes(_ppDb) != 1) throw std::runtime_error("Missing submap transaction node");
    run_id("DELETE FROM Link WHERE from_id=?1", node->id());
    const auto write_link = [&](const Link &link) {
      if (link.from() == removed || link.to() == removed) throw std::runtime_error("Retired endpoint in submap transaction");
      auto row = prepare_statement(_ppDb, "INSERT INTO Link(from_id,to_id,type,information_matrix,transform) VALUES(?1,?2,?3,?4,?5)");
      const auto transform = copyStateGroupCoefficients(link.transform());
      sqlite3_bind_int(row.get(), 1, link.from());
      sqlite3_bind_int(row.get(), 2, link.to());
      sqlite3_bind_int(row.get(), 3, link.type());
      const MatRC_t<6, 6, double, Eigen::RowMajor> information = link.infMatrix();
      sqlite3_bind_blob(row.get(), 4, information.data(), 36 * sizeof(double), SQLITE_TRANSIENT);
      sqlite3_bind_blob(row.get(), 5, transform.data(), sizeof(transform), SQLITE_TRANSIENT);
      check_sqlite(_ppDb, sqlite3_step(row.get()));
    };
    for (const auto &entry : node->getLinks()) write_link(entry.second);
  }
  // Only the retained submap changes visual content. Incident neighbors are
  // graph edits; rewriting their feature blocks wastes encoding and journal I/O
  // and would incorrectly persist a summary-only cold-neighbor draft.
  std::list<Node *> changed;
  for (auto *node : nodes)
    if (node->id() == retained) changed.push_back(node);
  if (changed.size() != 1) throw std::runtime_error("Missing or duplicate retained submap");
  write_features(changed, true);
  if (!archive.empty()) save_localmap_query(retained, archive);
  run_id("DELETE FROM Link WHERE from_id=?1 OR to_id=?1", removed);
  run_id("DELETE FROM Feature WHERE node_id=?1", removed);
  run_id("DELETE FROM GlobalDescriptor WHERE node_id=?1", removed);
  run_id("DELETE FROM Statistics WHERE id=?1", removed);
  run_id("DELETE FROM Data WHERE id=?1", removed);
  run_id("DELETE FROM LocalMap WHERE anchor_id=?1", removed);
  // Reanchor the independent trajectory in the same transaction as graph contraction.
  auto samples =
      prepare_statement(_ppDb, "SELECT sequence,anchor_from_body FROM Trajectory WHERE anchor_id=?1 AND sequence>?2 ORDER BY sequence LIMIT 128");
  auto update = prepare_statement(_ppDb, "UPDATE Trajectory SET anchor_id=?1,anchor_from_body=?2 WHERE sequence=?3");
  int last_sequence = 0;
  while (true) {
    std::vector<std::pair<int, StateGroupf::Coefficients>> batch;
    sqlite3_bind_int(samples.get(), 1, removed);
    sqlite3_bind_int(samples.get(), 2, last_sequence);
    int rc;
    while ((rc = sqlite3_step(samples.get())) == SQLITE_ROW) {
      StateGroupf relative;
      if (!stateGroupFromBlob(sqlite3_column_blob(samples.get(), 1), sqlite3_column_bytes(samples.get(), 1), relative) || !relative.isValid())
        throw std::runtime_error("Invalid trajectory in submap contraction");
      batch.emplace_back(sqlite3_column_int(samples.get(), 0), copyStateGroupCoefficients(retained_from_removed.compose(relative)));
    }
    if (rc != SQLITE_DONE) check_sqlite(_ppDb, rc);
    sqlite3_reset(samples.get());  // Finish the read before changing its index entries.
    if (batch.empty()) break;
    for (const auto &[sequence, transformed] : batch) {
      sqlite3_reset(update.get());
      sqlite3_bind_int(update.get(), 1, retained);
      sqlite3_bind_blob(update.get(), 2, transformed.data(), sizeof(transformed), SQLITE_TRANSIENT);
      sqlite3_bind_int(update.get(), 3, sequence);
      check_sqlite(_ppDb, sqlite3_step(update.get()));
    }
    last_sequence = batch.back().first;
  }
  // Canonical identity survives physical contraction and restart. Persist the
  // same mapping applied by Topograph, inside this existing SQL transaction.
  auto create_members = prepare_statement(_ppDb, "CREATE TABLE IF NOT EXISTS SubmapMember(node_id INTEGER PRIMARY KEY,anchor_id INTEGER NOT NULL)");
  check_sqlite(_ppDb, sqlite3_step(create_members.get()));
  auto redirect_members = prepare_statement(_ppDb, "UPDATE SubmapMember SET anchor_id=?1 WHERE anchor_id=?2");
  sqlite3_bind_int(redirect_members.get(), 1, retained);
  sqlite3_bind_int(redirect_members.get(), 2, removed);
  check_sqlite(_ppDb, sqlite3_step(redirect_members.get()));
  auto member = prepare_statement(_ppDb, "INSERT OR REPLACE INTO SubmapMember(node_id,anchor_id) VALUES(?1,?2)");
  sqlite3_bind_int(member.get(), 1, removed);
  sqlite3_bind_int(member.get(), 2, retained);
  check_sqlite(_ppDb, sqlite3_step(member.get()));
  run_id("DELETE FROM Node WHERE id=?1", removed);
  // Same Admin representation as saveOptimizedPosesQuery, with recoverable errors.
  std::vector<int> ids;
  std::vector<float> transforms;
  for (const auto &entry : poses) {
    ids.push_back(entry.first);
    const auto pose = copyStateGroupCoefficients(entry.second);
    transforms.insert(transforms.end(), pose.begin(), pose.end());
  }
  auto packed_ids = compressData(cv::Mat(1, ids.size(), CV_32SC1, ids.data()));
  auto packed_poses = compressData(cv::Mat(1, transforms.size(), CV_32FC1, transforms.data()));
  auto admin = prepare_statement(_ppDb, "UPDATE Admin SET opt_ids=?1,opt_poses=?2,opt_last_localization=?3");
  sqlite3_bind_blob(admin.get(), 1, packed_ids.ptr(), packed_ids.total(), SQLITE_TRANSIENT);
  sqlite3_bind_blob(admin.get(), 2, packed_poses.ptr(), packed_poses.total(), SQLITE_TRANSIENT);
  const auto last = copyStateGroupCoefficients(last_pose);
  if (!last_pose.isNull()) sqlite3_bind_blob(admin.get(), 3, last.data(), sizeof(last), SQLITE_TRANSIENT);
  check_sqlite(_ppDb, sqlite3_step(admin.get()));
}

void DBDriverSqlite3::save_trajectory_query(const std::vector<OdomObservation> &rows) const {
  check_sqlite(_ppDb, sqlite3_exec(_ppDb, "SAVEPOINT trajectory_batch", nullptr, nullptr, nullptr));
  try {
    auto insert = prepare_statement(_ppDb,
                                    "INSERT INTO Trajectory(sequence,anchor_id,stamp,anchor_from_body,odom_from_body) "
                                    "SELECT ?1,?2,?3,?4,?5 WHERE EXISTS(SELECT 1 FROM Node WHERE id=?2)");
    for (const auto &row : rows) {
      if (row.anchor_id <= 0 || row.sequence <= 0 || !std::isfinite(row.stamp) || row.anchor_from_body.isNull() || !row.anchor_from_body.isValid() ||
          row.odom_from_body.isNull() || !row.odom_from_body.isValid())
        throw std::invalid_argument("Invalid trajectory observation");
      const auto relative = copyStateGroupCoefficients(row.anchor_from_body);
      const auto odom = copyStateGroupCoefficients(row.odom_from_body);
      sqlite3_reset(insert.get());
      sqlite3_clear_bindings(insert.get());
      sqlite3_bind_int(insert.get(), 1, row.sequence);
      sqlite3_bind_int(insert.get(), 2, row.anchor_id);
      sqlite3_bind_double(insert.get(), 3, row.stamp);
      sqlite3_bind_blob(insert.get(), 4, relative.data(), sizeof(relative), SQLITE_TRANSIENT);
      sqlite3_bind_blob(insert.get(), 5, odom.data(), sizeof(odom), SQLITE_TRANSIENT);
      check_sqlite(_ppDb, sqlite3_step(insert.get()));
      if (sqlite3_changes(_ppDb) != 1) throw std::runtime_error("Missing trajectory anchor");
    }
    check_sqlite(_ppDb, sqlite3_exec(_ppDb, "RELEASE trajectory_batch", nullptr, nullptr, nullptr));
  } catch (...) {
    check_sqlite(_ppDb, sqlite3_exec(_ppDb, "ROLLBACK TO trajectory_batch", nullptr, nullptr, nullptr));
    check_sqlite(_ppDb, sqlite3_exec(_ppDb, "RELEASE trajectory_batch", nullptr, nullptr, nullptr));
    throw;
  }
}

int DBDriverSqlite3::trajectory_high_water_query() const {
  auto query = prepare_statement(_ppDb, "SELECT COALESCE(MAX(sequence),0) FROM Trajectory");
  if (sqlite3_step(query.get()) != SQLITE_ROW) throw std::runtime_error("Cannot read trajectory sequence");
  return sqlite3_column_int(query.get(), 0);
}

std::vector<OdomObservation> DBDriverSqlite3::load_recent_trajectory_query(int limit) const {
  std::vector<OdomObservation> result;
  auto query =
      prepare_statement(_ppDb, "SELECT sequence,anchor_id,stamp,anchor_from_body,odom_from_body FROM Trajectory ORDER BY sequence DESC LIMIT ?1");
  sqlite3_bind_int(query.get(), 1, limit);
  int rc;
  while ((rc = sqlite3_step(query.get())) == SQLITE_ROW) {
    OdomObservation row;
    row.anchor_id = sqlite3_column_int(query.get(), 1);
    row.sequence = sqlite3_column_int(query.get(), 0);
    row.stamp = sqlite3_column_double(query.get(), 2);
    if (!stateGroupFromBlob(sqlite3_column_blob(query.get(), 3), sqlite3_column_bytes(query.get(), 3), row.anchor_from_body) ||
        !stateGroupFromBlob(sqlite3_column_blob(query.get(), 4), sqlite3_column_bytes(query.get(), 4), row.odom_from_body))
      throw std::runtime_error("Corrupt trajectory pose");
    result.push_back(std::move(row));
  }
  if (rc != SQLITE_DONE) check_sqlite(_ppDb, rc);
  std::reverse(result.begin(), result.end());
  return result;
}

void DBDriverSqlite3::save_visit_history_query(const std::string &archive) const {
  if (!_ppDb || archive.empty() || archive.size() > 16 * 1024 * 1024) throw std::invalid_argument("Invalid visit archive size");
  check_sqlite(_ppDb, sqlite3_exec(_ppDb, "CREATE TABLE IF NOT EXISTS VisitHistory(id INTEGER PRIMARY KEY CHECK(id=1), archive TEXT NOT NULL)",
                                   nullptr, nullptr, nullptr));
  auto row = prepare_statement(_ppDb, "INSERT INTO VisitHistory(id,archive) VALUES(1,?1) ON CONFLICT(id) DO UPDATE SET archive=excluded.archive");
  check_sqlite(_ppDb, sqlite3_bind_text(row.get(), 1, archive.data(), static_cast<int>(archive.size()), SQLITE_TRANSIENT));
  check_sqlite(_ppDb, sqlite3_step(row.get()));
}

std::string DBDriverSqlite3::load_visit_history_query() const {
  if (!_ppDb) return {};
  auto exists = prepare_statement(_ppDb, "SELECT 1 FROM sqlite_master WHERE name='VisitHistory' AND type='table'");
  const int status = sqlite3_step(exists.get());
  if (status != SQLITE_ROW) {
    check_sqlite(_ppDb, status);
    return {};
  }
  auto row = prepare_statement(_ppDb, "SELECT archive FROM VisitHistory WHERE id=1");
  const int result = sqlite3_step(row.get());
  if (result != SQLITE_ROW) {
    check_sqlite(_ppDb, result);
    return {};
  }
  const int size = sqlite3_column_bytes(row.get(), 0);
  const auto *data = reinterpret_cast<const char *>(sqlite3_column_text(row.get(), 0));
  if (!data || size <= 0 || size > 16 * 1024 * 1024) throw std::runtime_error("Invalid visit archive");
  return {data, static_cast<size_t>(size)};
}

void DBDriverSqlite3::save_localmap_query(int anchor_id, const cv::Mat &archive) const {
  if (!_ppDb || anchor_id <= 0 || archive.type() != CV_8UC1 || archive.rows != 1 || archive.empty())
    throw std::invalid_argument("Invalid localmap database write");
  auto statement = prepare_statement(_ppDb,
                                     "INSERT INTO LocalMap(anchor_id, archive) SELECT ?1, ?2 WHERE EXISTS (SELECT 1 FROM Node WHERE id=?1) ON "
                                     "CONFLICT(anchor_id) DO UPDATE SET archive=excluded.archive");
  check_sqlite(_ppDb, sqlite3_bind_int(statement.get(), 1, anchor_id));
  check_sqlite(_ppDb, sqlite3_bind_blob(statement.get(), 2, archive.ptr(), archive.cols, SQLITE_STATIC));
  check_sqlite(_ppDb, sqlite3_step(statement.get()));
  if (sqlite3_changes(_ppDb) != 1) throw std::runtime_error("Localmap anchor is not committed");
}
std::map<int, cv::Mat> DBDriverSqlite3::load_localmaps_query() const {
  std::map<int, cv::Mat> result;
  if (!_ppDb) return result;
  auto statement = prepare_statement(_ppDb, "SELECT anchor_id, archive FROM LocalMap ORDER BY anchor_id");
  int status;
  while ((status = sqlite3_step(statement.get())) == SQLITE_ROW) {
    const int id = sqlite3_column_int(statement.get(), 0), size = sqlite3_column_bytes(statement.get(), 1);
    const void *bytes = sqlite3_column_blob(statement.get(), 1);
    if (id <= 0 || size <= 0 || size > 258 * 1024 * 1024 || !bytes) throw std::runtime_error("Invalid localmap database archive");
    result.emplace(id, cv::Mat(1, size, CV_8UC1, const_cast<void *>(bytes)).clone());
  }
  check_sqlite(_ppDb, status);
  return result;
}
std::vector<int> DBDriverSqlite3::localmap_ids_query() const {
  std::vector<int> result;
  if (!_ppDb) return result;
  auto query = prepare_statement(_ppDb, "SELECT anchor_id FROM LocalMap ORDER BY anchor_id");
  int status;
  while ((status = sqlite3_step(query.get())) == SQLITE_ROW) result.push_back(sqlite3_column_int(query.get(), 0));
  check_sqlite(_ppDb, status);
  return result;
}
cv::Mat DBDriverSqlite3::load_localmap_query(int anchor_id) const {
  if (!_ppDb || anchor_id <= 0) throw std::runtime_error("Localmap database is unavailable");
  auto query = prepare_statement(_ppDb, "SELECT archive FROM LocalMap WHERE anchor_id=?1");
  check_sqlite(_ppDb, sqlite3_bind_int(query.get(), 1, anchor_id));
  const int status = sqlite3_step(query.get());
  if (status == SQLITE_DONE) throw std::runtime_error("Localmap snapshot no longer exists");
  if (status != SQLITE_ROW) check_sqlite(_ppDb, status);
  const int size = sqlite3_column_bytes(query.get(), 0);
  const void *bytes = sqlite3_column_blob(query.get(), 0);
  if (!bytes || size <= 0 || size > 258 * 1024 * 1024) throw std::runtime_error("Invalid localmap database archive");
  return cv::Mat(1, size, CV_8UC1, const_cast<void *>(bytes)).clone();
}

void DBDriverSqlite3::save2DMapQuery(const cv::Mat &map, float xMin, float yMin, float cellSize) const {
  EXT_LOG_DEBUG("");
  if (_ppDb) {
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::string query;

    // Update table Admin
    query = fmt::format(
        "UPDATE Admin SET opt_map=?, opt_map_x_min=?, opt_map_y_min=?, opt_map_resolution=?, time_enter = DATETIME('NOW') WHERE version='{}';",
        _version.c_str());
    rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    int index = 1;

    // opt ids and poses
    cv::Mat compressedMap;
    if (map.empty()) {
      rc = sqlite3_bind_null(ppStmt, index++);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    } else {
      compressedMap = compressData(map);

      rc = sqlite3_bind_blob(ppStmt, index++, compressedMap.data, compressedMap.cols, SQLITE_STATIC);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    }

    rc = sqlite3_bind_double(ppStmt, index++, xMin);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    rc = sqlite3_bind_double(ppStmt, index++, yMin);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    rc = sqlite3_bind_double(ppStmt, index++, cellSize);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // execute query
    rc = sqlite3_step(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());
  }
}

cv::Mat DBDriverSqlite3::load2DMapQuery(float &xMin, float &yMin, float &cellSize) const {
  EXT_LOG_DEBUG("");
  cv::Mat map;
  if (_ppDb) {
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::stringstream query;

    query << "SELECT  opt_map, opt_map_x_min, opt_map_y_min, opt_map_resolution "
          << "FROM Admin "
          << "WHERE version='" << _version.c_str() << "';";

    rc = sqlite3_prepare_v2(_ppDb, query.str().c_str(), -1, &ppStmt, 0);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Process the result if one
    rc = sqlite3_step(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_ROW, "DB error ({}): Not found first Admin row: query=\"{}\"", _version.c_str(), query.str().c_str());
    if (rc == SQLITE_ROW) {
      const void *data = 0;
      int dataSize = 0;
      int index = 0;

      // opt_map
      data = sqlite3_column_blob(ppStmt, index);
      dataSize = sqlite3_column_bytes(ppStmt, index++);
      if (dataSize > 0 && data) {
        map = uncompressData(cv::Mat(1, dataSize, CV_8UC1, (void *)data));
        EXT_LOG_DEBUG("map={:d}/{:d}", map.cols, map.rows);
      }

      xMin = sqlite3_column_double(ppStmt, index++);
      EXT_LOG_DEBUG("xMin={:.6f}", xMin);
      yMin = sqlite3_column_double(ppStmt, index++);
      EXT_LOG_DEBUG("yMin={:.6f}", yMin);
      cellSize = sqlite3_column_double(ppStmt, index++);
      EXT_LOG_DEBUG("cellSize={:.6f}", cellSize);

      rc = sqlite3_step(ppStmt);  // next result...
    }
    EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // Finalize (delete) the statement
    rc = sqlite3_finalize(ppStmt);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    EXT_LOG_DEBUG("Time={:.6f}s", timer.lap_seconds());
  }
  return map;
}

std::string DBDriverSqlite3::queryStepNode() const {
  return "INSERT INTO Node(id, map_id, weight, pose, stamp, label, velocity, env_sensors, lidar_geometry) VALUES(?,?,?,?,?,?,?,?,?);";
}
void DBDriverSqlite3::stepNode(sqlite3_stmt *ppStmt, const Node *s, const EnvSensors &sensors) const {
  EXT_LOG_DEBUG("Save node {:d}", s->id());
  if (!ppStmt || !s) {
    EXT_LOG_FATAL("");
  }
  int rc = SQLITE_OK;

  int index = 1;
  rc = sqlite3_bind_int(ppStmt, index++, s->id());
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  rc = sqlite3_bind_int(ppStmt, index++, s->mapId());
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  rc = sqlite3_bind_int(ppStmt, index++, s->getWeight());
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  const StateGroupf::Coefficients poseCoefficients = copyStateGroupCoefficients(s->getPose());
  rc = sqlite3_bind_blob(ppStmt, index++, poseCoefficients.data(), sizeof(poseCoefficients), SQLITE_STATIC);
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  {
    rc = sqlite3_bind_double(ppStmt, index++, s->getStamp());
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    if (s->getLabel().empty()) {
      rc = sqlite3_bind_null(ppStmt, index++);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    } else {
      rc = sqlite3_bind_text(ppStmt, index++, s->getLabel().c_str(), -1, SQLITE_STATIC);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    }
  }

  std::vector<double> envSensors;
  if (s->getVelocity().empty()) {
    rc = sqlite3_bind_null(ppStmt, index++);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  } else {
    rc = sqlite3_bind_blob(ppStmt, index++, s->getVelocity().data(), s->getVelocity().size() * sizeof(float), SQLITE_STATIC);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  }

  if (sensors.size() == 0) {
    rc = sqlite3_bind_null(ppStmt, index++);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  } else {
    envSensors.resize(sensors.size() * 3, 0.0);
    int j = 0;
    for (std::map<EnvSensor::Type, EnvSensor>::const_iterator iter = sensors.begin(); iter != sensors.end(); ++iter, j += 3) {
      envSensors[j] = (double)iter->second.type();
      envSensors[j + 1] = iter->second.value();
      envSensors[j + 2] = iter->second.stamp();
    }
    rc = sqlite3_bind_blob(ppStmt, index++, envSensors.data(), envSensors.size() * sizeof(double), SQLITE_STATIC);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  }

  const cv::Mat geometry = s->has_lidar() ? lidar::encode(*s->lidar_cloud()) : cv::Mat();
  rc =
      geometry.empty() ? sqlite3_bind_null(ppStmt, index++) : sqlite3_bind_blob(ppStmt, index++, geometry.data, int(geometry.total()), SQLITE_STATIC);
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error: {}", sqlite3_errmsg(_ppDb));

  // step
  rc = sqlite3_step(ppStmt);
  EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  rc = sqlite3_reset(ppStmt);
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
}

std::string DBDriverSqlite3::queryStepCalibrationUpdate() const { return "UPDATE Data SET calibration=? WHERE id=?;"; }
void DBDriverSqlite3::stepCalibrationUpdate(sqlite3_stmt *ppStmt, int nodeId, const std::vector<camera::Perspective> &models) const {
  if (!ppStmt) {
    EXT_LOG_FATAL("");
  }

  int rc = SQLITE_OK;
  int index = 1;

  // calibration
  std::vector<unsigned char> calibrationData;
  if (models.size() && models[0].isValidForProjection()) {
    calibrationData.reserve(models.size() * kPerspectiveCalibrationSize);
    for (const camera::Perspective &model : models) appendPerspectiveCalibration(calibrationData, model);
  }
  if (calibrationData.size()) {
    rc = sqlite3_bind_blob(ppStmt, index++, calibrationData.data(), calibrationData.size(), SQLITE_STATIC);
  } else {
    rc = sqlite3_bind_null(ppStmt, index++);
  }
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  // id
  rc = sqlite3_bind_int(ppStmt, index++, nodeId);
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  // step
  rc = sqlite3_step(ppStmt);
  EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  rc = sqlite3_reset(ppStmt);
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
}

std::string DBDriverSqlite3::queryStepDepthUpdate() const { return "UPDATE Data SET depth=? WHERE id=?;"; }
void DBDriverSqlite3::stepDepthUpdate(sqlite3_stmt *ppStmt, int nodeId, const cv::Mat &image) const {
  if (!ppStmt) {
    EXT_LOG_FATAL("");
  }

  int rc = SQLITE_OK;
  int index = 1;

  cv::Mat imageCompressed;
  EXT_ASSERT_MSG(image.empty() || image.type() == CV_16UC1, "Depth database update accepts only raw CV_16UC1 millimeter depth.");
  imageCompressed = compressDepth(image);
  if (!imageCompressed.empty()) {
    rc = sqlite3_bind_blob(ppStmt, index++, imageCompressed.data, (int)imageCompressed.cols, SQLITE_STATIC);
  } else {
    rc = sqlite3_bind_zeroblob(ppStmt, index++, 4);
  }
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  // id
  rc = sqlite3_bind_int(ppStmt, index++, nodeId);
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  // step
  rc = sqlite3_step(ppStmt);
  EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  rc = sqlite3_reset(ppStmt);
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
}

std::string DBDriverSqlite3::queryStepSensorData() const {
  return "INSERT INTO Data(id, image, depth, calibration, user_data, ground_cells, obstacle_cells, empty_cells, cell_size, view_point_x, "
         "view_point_y, view_point_z) "
         "VALUES(?,?,?,?,?,?,?,?,?,?,?,?);";
}
void DBDriverSqlite3::stepSensorData(sqlite3_stmt *ppStmt, const SensorRecord &record) const {
  const auto &sensorData = record.observation;
  EXT_LOG_DEBUG("Save sensor data {:d} (image={:d} depth={:d})", sensorData.id(), (int)record.archive.image.cols, (int)record.archive.depth.cols);
  if (!ppStmt) {
    EXT_LOG_FATAL("");
  }

  const cv::Mat userData = record.archive.user.empty() ? compressData(sensorData.userDataRaw()) : record.archive.user;
  int rc = SQLITE_OK;
  int index = 1;

  // id
  rc = sqlite3_bind_int(ppStmt, index++, sensorData.id());
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  // image
  if (!record.archive.image.empty()) {
    rc = sqlite3_bind_blob(ppStmt, index++, record.archive.image.data, (int)record.archive.image.cols, SQLITE_STATIC);
  } else {
    rc = sqlite3_bind_null(ppStmt, index++);
  }
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  // depth image
  if (!record.archive.depth.empty()) {
    rc = sqlite3_bind_blob(ppStmt, index++, record.archive.depth.data, (int)record.archive.depth.cols, SQLITE_STATIC);
  } else {
    rc = sqlite3_bind_null(ppStmt, index++);
  }
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  // calibration
  std::vector<unsigned char> calibrationData;
  if (sensorData.perspectives().size() && sensorData.perspectives()[0].isValidForProjection()) {
    calibrationData.reserve(sensorData.perspectives().size() * kPerspectiveCalibrationSize);
    for (const camera::Perspective &model : sensorData.perspectives()) appendPerspectiveCalibration(calibrationData, model);
  }
  if (calibrationData.size()) {
    rc = sqlite3_bind_blob(ppStmt, index++, calibrationData.data(), calibrationData.size(), SQLITE_STATIC);
  } else {
    rc = sqlite3_bind_null(ppStmt, index++);
  }
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  {
    // user_data
    if (!userData.empty()) {
      rc = sqlite3_bind_blob(ppStmt, index++, userData.data, (int)userData.cols, SQLITE_STATIC);
    } else {
      rc = sqlite3_bind_null(ppStmt, index++);
    }
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  }

  {
    // ground_cells
    if (record.archive.grid.ground.empty()) {
      rc = sqlite3_bind_null(ppStmt, index++);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    } else {
      // compress
      rc = sqlite3_bind_blob(ppStmt, index++, record.archive.grid.ground.data, (int)record.archive.grid.ground.cols, SQLITE_STATIC);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    }

    // obstacle_cells
    if (record.archive.grid.obstacles.empty()) {
      rc = sqlite3_bind_null(ppStmt, index++);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    } else {
      rc = sqlite3_bind_blob(ppStmt, index++, record.archive.grid.obstacles.data, (int)record.archive.grid.obstacles.cols, SQLITE_STATIC);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    }

    {
      // empty_cells
      if (record.archive.grid.empty.empty()) {
        rc = sqlite3_bind_null(ppStmt, index++);
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      } else {
        rc = sqlite3_bind_blob(ppStmt, index++, record.archive.grid.empty.data, (int)record.archive.grid.empty.cols, SQLITE_STATIC);
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      }
    }

    // cell_size
    rc = sqlite3_bind_double(ppStmt, index++, record.archive.grid.cell_size);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

    // view_point
    rc = sqlite3_bind_double(ppStmt, index++, record.archive.grid.view_point.x);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    rc = sqlite3_bind_double(ppStmt, index++, record.archive.grid.view_point.y);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    rc = sqlite3_bind_double(ppStmt, index++, record.archive.grid.view_point.z);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  }

  // step
  rc = sqlite3_step(ppStmt);
  EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  rc = sqlite3_reset(ppStmt);
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
}

std::string DBDriverSqlite3::queryStepLinkUpdate() const {
  return "UPDATE Link SET type=?, information_matrix=?, transform=? WHERE from_id=? AND to_id = ?;";
}
std::string DBDriverSqlite3::queryStepLink() const {
  // from_id, to_id are at the end to match the update query above
  return "INSERT INTO Link(type, information_matrix, transform, from_id, to_id) VALUES(?,?,?,?,?);";
}
void DBDriverSqlite3::stepLink(sqlite3_stmt *ppStmt, const Link &link) const {
  if (!ppStmt) {
    EXT_LOG_FATAL("");
  }
  // EXT_LOG_DEBUG("Save link from %d to %d, type=%d", link.from(), link.to(), link.type());

  if (!Link::isConstraintType(link.type())) throw std::invalid_argument("Cannot store an invalid Link type");

  int rc = SQLITE_OK;
  int index = 1;
  rc = sqlite3_bind_int(ppStmt, index++, link.type());
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  {
    // information_matrix
    const MatRC_t<6, 6, double, Eigen::RowMajor> information = link.infMatrix();
    rc = sqlite3_bind_blob(ppStmt, index++, information.data(), 36 * sizeof(double), SQLITE_TRANSIENT);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  }

  const StateGroupf::Coefficients transformCoefficients = copyStateGroupCoefficients(link.transform());
  rc = sqlite3_bind_blob(ppStmt, index++, transformCoefficients.data(), sizeof(transformCoefficients), SQLITE_STATIC);
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  rc = sqlite3_bind_int(ppStmt, index++, link.from());
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  rc = sqlite3_bind_int(ppStmt, index++, link.to());
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  rc = sqlite3_step(ppStmt);
  EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  rc = sqlite3_reset(ppStmt);
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
}

std::string DBDriverSqlite3::queryStepGlobalDescriptor() const { return "INSERT INTO GlobalDescriptor(node_id, type, info, data) VALUES(?,?,?,?);"; }
void DBDriverSqlite3::stepGlobalDescriptor(sqlite3_stmt *ppStmt, int nodeId, const GlobalDescriptor &descriptor) const {
  if (!ppStmt) {
    EXT_LOG_FATAL("");
  }
  int rc = SQLITE_OK;
  int index = 1;

  // node_id
  rc = sqlite3_bind_int(ppStmt, index++, nodeId);
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  // type
  rc = sqlite3_bind_int(ppStmt, index++, descriptor.type());
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  // info
  cv::Mat infoBytes = sapphire::backend::compressData(descriptor.info());
  if (infoBytes.empty()) {
    rc = sqlite3_bind_null(ppStmt, index++);
  } else {
    rc = sqlite3_bind_blob(ppStmt, index++, infoBytes.data, infoBytes.total(), SQLITE_STATIC);
  }
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  // data
  cv::Mat dataBytes = sapphire::backend::compressData(descriptor.data());
  if (dataBytes.empty()) {
    rc = sqlite3_bind_null(ppStmt, index++);
  } else {
    rc = sqlite3_bind_blob(ppStmt, index++, dataBytes.data, dataBytes.total(), SQLITE_STATIC);
  }
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  rc = sqlite3_step(ppStmt);
  EXT_ASSERT_MSG(rc == SQLITE_DONE, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

  rc = sqlite3_reset(ppStmt);
  EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
}

}  // namespace sapphire::backend
