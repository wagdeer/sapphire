#include <backend/common/common_type.hpp>
#include <backend/params/database.hpp>
#include <backend/params/topograph.hpp>
#include <backend/parameters.hpp>
#include <backend/system/action/mapping.hpp>
#include <backend/database/driver.hpp>
#include <backend/database/reader.hpp>
#include <backend/database/recovery.hpp>
#include <backend/maplayer/topograph.hpp>
#include <backend/tools/filesystem.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/strings.hpp>

namespace sapphire::backend {

bool databaseRecovery(const std::string& corruptedDatabase, bool keepCorruptedDatabase, std::string* errorMsg,
                      const std::function<bool(std::string_view)>& progress, const ParametersMap& parameters) {
  const auto startup = LoadParameters::from_map(parameters);
  if (DatabaseParams{}.withOverrides(startup).sqlite_in_memory)
    throw std::invalid_argument("Recovery requires a disk-backed database");
  bool canceled = false;
  const auto report = [&](std::string_view message) {
    if (!canceled && progress) {
      canceled = !progress(message);
    }
    return !canceled;
  };

  EXT_LOG_DEBUG("Recovering \"{}\"", corruptedDatabase);

  const tools::fs::path databasePath = tools::replace_char(corruptedDatabase, '~', tools::home_directory().string());
  if (!tools::path_exists(databasePath)) {
    if (errorMsg) *errorMsg = fmt::format("File \"{}\" doesn't exist!", databasePath.string());
    return false;
  }

  if (tools::extension(databasePath).compare("db") != 0) {
    if (errorMsg) *errorMsg = fmt::format("File \"{}\" is not a database (*.db)!", databasePath.string());
    return false;
  }

  tools::fs::path recoveryPath = databasePath;
  recoveryPath.replace_extension(".recovery.db");
  if (tools::path_exists(recoveryPath)) {
    std::error_code error;
    const bool removed = tools::remove_file(recoveryPath, error);
    if (!removed || error) {
      if (errorMsg)
        *errorMsg =
            fmt::format("Failed to remove temporary recovery database \"{}\", is it opened by another app? ({})", recoveryPath.string(), error.message());
      return false;
    }
  }

  DBDriver* dbDriver = DBDriver::create();
  if (!dbDriver->openConnection(databasePath.string(), false)) {
    if (errorMsg) *errorMsg = fmt::format("Failed opening database!");
    delete dbDriver;
    return false;
  }

  std::set<int> ids;
  dbDriver->getAllNodeIds(ids);
  if (ids.empty()) {
    if (errorMsg) *errorMsg = fmt::format("Input database doesn't have any nodes saved in it.");
    dbDriver->closeConnection(false);
    delete dbDriver;
    return false;
  }
  report(fmt::format("Found {} nodes to recover.", (int)ids.size()));

  // Detect if the database is corrupted
  std::multimap<int, Link> links;
  dbDriver->getAllLinks(links, true);
  bool corrupted = false;
  for (std::multimap<int, Link>::iterator iter = links.begin(); iter != links.end(); ++iter) {
    if (iter->second.type() == Link::kOdometry && graph::findLink(links, iter->second.to(), iter->second.from(), false) == links.end()) {
      corrupted = true;
      break;
    }
  }

  report(corrupted ? "Database is indeed corrupted, found one or more neighbor links missing." : "Database doesn't seem to be corrupted, still recovering it.");

  dbDriver->closeConnection(false);
  delete dbDriver;

  Mapping rtabmap;
  rtabmap.init(startup, recoveryPath.string());
  const bool odometryIgnored = !TopographParams{}.withOverrides(startup).metric_enabled;
  {
    DBReader dbReader(databasePath.string(), 0, odometryIgnored);
    dbReader.init();

    DBReaderInfo info;
    SensorData data = dbReader.readNext(&info);
    int processed = 0;
    report(fmt::format("Recovering data of \"{}\"...", databasePath.string()));
    while (data.isValid() && report({})) {
      std::string status;
      if (!odometryIgnored && info.odomPose.isNull()) {
        status = fmt::format("Skipping node {} as it doesn't have odometry pose set.", data.id());
      } else {
        if (!odometryIgnored && info.odomCovariance(0, 0) >= 9999) {
          status = fmt::format("High variance detected, triggering a new map...");
          rtabmap.triggerNewMap();
        }
        if (!rtabmap.process(data, info.odomPose, info.odomCovariance, info.odomVelocity)) {
          status = fmt::format("Failed processing node {}.", data.id());
        }
      }
      if (!status.empty()) {
        report(status);
      }

      data = dbReader.readNext(&info);

      report(fmt::format("Processed {}/{} nodes...", ++processed, (int)ids.size()));
    }
  }

  if (canceled) {
    rtabmap.close(false);
    if (errorMsg) *errorMsg = fmt::format("Recovery canceled, removing temporary recovery database \"{}\".", recoveryPath.string());

    std::error_code ignored;
    tools::remove_file(recoveryPath, ignored);
    return false;
  }

  report(fmt::format("Closing database \"{}\"...", recoveryPath.string()));
  rtabmap.close(true);
  report(fmt::format("Closing database \"{}\"... done!", recoveryPath.string()));

  if (keepCorruptedDatabase) {
    tools::fs::path backupPath = databasePath;
    backupPath.replace_extension(".backup.db");

    if (!tools::path_exists(backupPath)) {
      report(fmt::format("Renaming \"{}\" to \"{}\"... (keep corrupted database backup option is enabled).", tools::filename(databasePath),
                         tools::filename(backupPath)));
      std::error_code error;
      tools::rename_file(databasePath, backupPath, error);
      if (error) {
        if (errorMsg)
          *errorMsg = fmt::format("Failed renaming database file from \"{}\" to \"{}\". Is it opened by another app? ({})", tools::filename(databasePath),
                                  tools::filename(backupPath), error.message());
        return false;
      }
      report(fmt::format("Renaming \"{}\" to \"{}\"... done!", tools::filename(databasePath), tools::filename(backupPath)));
    } else {
      report(fmt::format("Backup \"{}\" already exists, won't copy again.", tools::filename(backupPath)));
    }
  } else {
    std::error_code error;
    const bool removed = tools::remove_file(databasePath, error);
    if (!removed || error) {
      if (errorMsg)
        *errorMsg = fmt::format(
            "Failed remove original database file \"{}\". Is it opened by another app? The recovered database \"{}\" cannot be copied back to original name. "
            "({})",
            tools::filename(databasePath), tools::filename(recoveryPath), error.message());
      return false;
    }
  }

  report(fmt::format("Renaming \"{}\" to \"{}\"...", tools::filename(recoveryPath), tools::filename(databasePath)));
  std::error_code error;
  tools::rename_file(recoveryPath, databasePath, error);
  if (error) {
    if (errorMsg)
      *errorMsg = fmt::format("Failed renaming database file from \"{}\" to \"{}\". Is it opened by another app? ({})", tools::filename(recoveryPath),
                              tools::filename(databasePath), error.message());
    return false;
  }
  report(fmt::format("Renaming \"{}\" to \"{}\"... done!", tools::filename(recoveryPath), tools::filename(databasePath)));

  return true;
}

}  // namespace sapphire::backend
