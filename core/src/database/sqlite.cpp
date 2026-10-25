#include "database/sqlite.hpp"

#include <sqlite3.h>
#include <zlib.h>

#include <algorithm>
#include <common/common_type.hpp>
#include <parameters.hpp>
#include <tools/filesystem.hpp>
#include <tools/logging.hpp>
#include <tools/strings.hpp>
#include <tools/timer.hpp>
#include <climits>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <regex>
#include <set>

#include "DatabaseSchema_sql.h"
#include "database/compression.hpp"
#include "maplayer/topograph/node.hpp"

namespace sapphire {
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

bool stateGroupFromBlob(const void *data, int dataSize, SE3f &state) {
  if (data == nullptr || dataSize != static_cast<int>(SE3f::kCoefficientCount * sizeof(float))) {
    return false;
  }
  SE3f::Coefficients coefficients{};
  std::memcpy(coefficients.data(), data, sizeof(coefficients));
  state = SE3f(coefficients);
  return true;
}

SE3f::Coefficients copyStateGroupCoefficients(const SE3f &state) {
  SE3f::Coefficients coefficients{};
  std::copy(state.coefficients().begin(), state.coefficients().end(), coefficients.begin());
  return coefficients;
}

}  // namespace

std::optional<Link> DBDriverSqlite3::read_link(sqlite3_stmt *statement, bool ignoreNullLinks) {
  const int from = sqlite3_column_int(statement, 0), to = sqlite3_column_int(statement, 1);
  if (from <= 0 || to <= 0) throw std::runtime_error("Stored Link requires positive node IDs");
  const auto type = static_cast<Link::Type>(sqlite3_column_int(statement, 2));
  if (!Link::isConstraintType(type))
    throw std::runtime_error(fmt::format("Unsupported stored Link type {} ({} -> {}); this database requires rebuilding", int(type), from, to));

  const void *data = sqlite3_column_blob(statement, 3);
  const int size = sqlite3_column_bytes(statement, 3);
  SE3f transform;
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
    if (version != 14) {
      EXT_LOG_ERROR("Unsupported database schema: expected 14 (LiDAR graph without auxiliary user data); automatic migration is not supported");
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
                     "length(time_enter)) from Node;");
}

long DBDriverSqlite3::getLinksMemoryUsedQuery() const {
  return read_scalar(_ppDb,
                     "SELECT sum(length(type) + length(information_matrix) + length(transform) + length(from_id) + length(to_id)) from "
                     "Link;");
}

long DBDriverSqlite3::getStatisticsMemoryUsedQuery() const {
  return read_scalar(_ppDb, "SELECT sum(length(id) + length(stamp) + ifnull(length(data),0) + ifnull(length(wm_state),0)) FROM Statistics;");
}

void DBDriverSqlite3::addInfoAfterRunQuery(int size, const RunInfo &info) const {
  auto row =
      prepare_statement(_ppDb, "INSERT INTO Info(STM_size,last_sign_added,process_mem_used,database_mem_used,parameters) VALUES(?1,?2,?3,?4,?5)");
  check_sqlite(_ppDb, sqlite3_bind_int(row.get(), 1, size));
  check_sqlite(_ppDb, sqlite3_bind_int(row.get(), 2, info.lastNode));
  check_sqlite(_ppDb, sqlite3_bind_int(row.get(), 3, info.processMemoryUsed));
  check_sqlite(_ppDb, sqlite3_bind_int(row.get(), 4, info.databaseMemoryUsed));
  const auto parameters = Parameters::serialize(info.parameters);
  check_sqlite(_ppDb, sqlite3_bind_text(row.get(), 5, parameters.c_str(), -1, SQLITE_TRANSIENT));
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

bool DBDriverSqlite3::getNodeInfoQuery(int nodeId, SE3f &pose, int &mapId, int &weight, std::string &label, double &stamp,
                                       std::vector<float> &velocity) const {
  bool found = false;
  if (_ppDb && nodeId) {
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;
    std::stringstream query;

    {
      query << "SELECT pose, map_id, weight, label, stamp, velocity "
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
      { query << " lidar_geometry IS NOT NULL "; }
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

void DBDriverSqlite3::getAllOdomPosesQuery(std::map<int, SE3f> &poses, bool ignoreChildren, bool ignoreIntermediateNodes) const {
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

      SE3f pose;
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

void DBDriverSqlite3::loadNodesQuery(const std::list<int> &ids, std::list<Node *> &output) const {
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
      SE3f pose;
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

    this->loadLinksQuery(nodes);
    EXT_LOG_DEBUG("Time loading links={:.6f}s", timer.lap_seconds());

    for (std::list<Node *>::iterator iter = nodes.begin(); iter != nodes.end(); ++iter) {
      (*iter)->setModified(false);
    }

    if (ids.size() != loaded) {
      EXT_LOG_ERROR("Some nodes not found in database");
    }
  }
  output.splice(output.end(), nodes);
  for (auto &node : owned) node.release();
}

void DBDriverSqlite3::loadLastNodesQuery(std::list<Node *> &nodes) const {
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
    this->loadNodesQuery(ids, nodes);
    EXT_LOG_DEBUG("loaded={:d}, Time={:.6f}s", (int)nodes.size(), timer.lap_seconds());
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

    EXT_LOG_DEBUG("nodes update={:.6f}s", timer.lap_seconds());
  }
}

void DBDriverSqlite3::saveQuery(const std::list<Node *> &nodes) {
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
      stepNode(ppStmt, *i);
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

void DBDriverSqlite3::addStatisticsQuery(const Statistics &statistics, bool saveWmState) const {
  EXT_LOG_DEBUG("Ref ID = {:d}", statistics.refNodeId());
  if (_ppDb) {
    std::string type;
    tools::Timer timer;
    timer.reset();
    int rc = SQLITE_OK;
    sqlite3_stmt *ppStmt = 0;

    // Create query
    {
      std::string param = Statistics::serializeData(statistics.data());
      if (param.size() && statistics.refNodeId() > 0) {
        std::string query;
        {
          query =
              "INSERT INTO Statistics(id, stamp, data, wm_state) values(?,?,?,?) ON CONFLICT(id) DO UPDATE SET "
              "stamp=excluded.stamp,data=excluded.data,wm_state=excluded.wm_state;";
        }
        rc = sqlite3_prepare_v2(_ppDb, query.c_str(), -1, &ppStmt, 0);
        EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));

        int index = 1;
        rc = sqlite3_bind_int(ppStmt, index++, statistics.refNodeId());
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

RootFrame DBDriverSqlite3::create_root_frame_query(int anchor, const SE3f &pose) {
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

void DBDriverSqlite3::saveOptimizedPosesQuery(const std::map<int, SE3f> &poses, const SE3f &lastPose) const {
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
      std::vector<float> serializedPoses(poses.size() * SE3f::kCoefficientCount);
      int i = 0;
      for (std::map<int, SE3f>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter) {
        serializedIds[i] = iter->first;
        const SE3f::Coefficients coefficients = copyStateGroupCoefficients(iter->second);
        std::copy(coefficients.begin(), coefficients.end(), serializedPoses.begin() + SE3f::kCoefficientCount * i);
        ++i;
      }

      compressedIds = compressData(cv::Mat(1, serializedIds.size(), CV_32SC1, serializedIds.data()));
      compressedPoses = compressData(cv::Mat(1, serializedPoses.size(), CV_32FC1, serializedPoses.data()));

      rc = sqlite3_bind_blob(ppStmt, index++, compressedIds.data, compressedIds.cols, SQLITE_STATIC);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
      rc = sqlite3_bind_blob(ppStmt, index++, compressedPoses.data, compressedPoses.cols, SQLITE_STATIC);
      EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
    }

    SE3f::Coefficients lastPoseCoefficients{};
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

std::map<int, SE3f> DBDriverSqlite3::loadOptimizedPosesQuery(SE3f *lastPose) const {
  EXT_LOG_DEBUG("");
  std::map<int, SE3f> poses;
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

        EXT_ASSERT(serializedIds.cols == serializedPoses.cols / static_cast<int>(SE3f::kCoefficientCount));
        EXT_ASSERT(serializedPoses.type() == CV_32FC1);
        EXT_ASSERT(serializedIds.type() == CV_32SC1);
        for (int i = 0; i < serializedIds.cols; ++i) {
          SE3f::Coefficients coefficients{};
          const float *serializedPose = serializedPoses.ptr<float>() + i * SE3f::kCoefficientCount;
          std::copy_n(serializedPose, SE3f::kCoefficientCount, coefficients.begin());
          SE3f t(coefficients);
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

std::string DBDriverSqlite3::queryStepNode() const {
  return "INSERT INTO Node(id, map_id, weight, pose, stamp, label, velocity, lidar_geometry) VALUES(?,?,?,?,?,?,?,?);";
}
void DBDriverSqlite3::stepNode(sqlite3_stmt *ppStmt, const Node *s) const {
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
  const SE3f::Coefficients poseCoefficients = copyStateGroupCoefficients(s->getPose());
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

  if (s->getVelocity().empty()) {
    rc = sqlite3_bind_null(ppStmt, index++);
    EXT_ASSERT_MSG(rc == SQLITE_OK, "DB error ({}): {}", _version.c_str(), sqlite3_errmsg(_ppDb));
  } else {
    rc = sqlite3_bind_blob(ppStmt, index++, s->getVelocity().data(), s->getVelocity().size() * sizeof(float), SQLITE_STATIC);
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

  const SE3f::Coefficients transformCoefficients = copyStateGroupCoefficients(link.transform());
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

}  // namespace sapphire
