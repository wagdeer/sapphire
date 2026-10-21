#pragma once
#include <backend/common/common_type.hpp>

#include <backend/parameters.hpp>
#include <functional>
#include <string>
#include <string_view>

namespace sapphire::backend {

/**
 * Return true on success. The database is
 * renamed to "*.backup.db" before recovering.
 * @param corruptedDatabase database to recover
 * @param keepCorruptedDatabase if false and on recovery success, the backup database is removed
 * @param errorMsg error message if the function returns false
 * @param progress Optional progress callback. Returning false cancels recovery.
 * @param parameters Startup configuration from LoadParameters, never historical database tuning.
 */
bool databaseRecovery(const std::string& corruptedDatabase, bool keepCorruptedDatabase = true, std::string* errorMsg = 0,
                      const std::function<bool(std::string_view)>& progress = {}, const ParametersMap& parameters = {});

}  // namespace sapphire::backend
