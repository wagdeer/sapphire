#include <sys/wait.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include "backend/storage/map_database.hpp"

using namespace sapphire;
namespace fs = std::filesystem;

extern "C" {
void *__real_mmap(void *, size_t, int, int, int, off_t);
int __real_mprotect(void *, size_t, int);
int __real_munmap(void *, size_t);
int __real_close(int);
int __real_sqlite3_open_v2(const char *, sqlite3 **, int, const char *);
int __real_sqlite3_deserialize(sqlite3 *, const char *, unsigned char *, sqlite3_int64, sqlite3_int64, unsigned);
int __real_sqlite3_close(sqlite3 *);
}

namespace {
void check(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}
std::string bytes(const fs::path &path) {
  std::ifstream input(path, std::ios::binary);
  check(bool(input), "read fixture bytes");
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}
std::string bytes(int fd) {
  struct stat info {};
  check(fstat(fd, &info) == 0, "stat verification descriptor");
  std::string data(static_cast<std::size_t>(info.st_size), '\0');
  check(pread(fd, data.data(), data.size(), 0) == static_cast<ssize_t>(data.size()), "read verification descriptor");
  return data;
}
std::map<std::string, std::string> files(const fs::path &root) {
  std::map<std::string, std::string> result;
  for (const auto &entry : fs::directory_iterator(root)) {
    const auto type = entry.symlink_status().type();
    result[entry.path().filename().string()] = type == fs::file_type::regular   ? bytes(entry.path())
                                               : type == fs::file_type::symlink ? "symlink:" + fs::read_symlink(entry.path()).string()
                                                                                : "type:" + std::to_string(static_cast<int>(type));
  }
  return result;
}
void sql(const fs::path &path, const char *query) {
  sqlite3 *db = nullptr;
  check(sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READWRITE, nullptr) == SQLITE_OK, "open fixture");
  const int rc = sqlite3_exec(db, query, nullptr, nullptr, nullptr);
  check(sqlite3_close_v2(db) == SQLITE_OK && rc == SQLITE_OK, "mutate fixture");
}
std::string scalar(sqlite3 *db, const char *query) {
  database_detail::Statement statement(db, query);
  check(sqlite3_step(statement.get()) == SQLITE_ROW, "read eligibility connection");
  return reinterpret_cast<const char *>(sqlite3_column_text(statement.get(), 0));
}
std::size_t fdCount() { return std::distance(fs::directory_iterator("/proc/self/fd"), fs::directory_iterator{}); }

enum class Replacement { None, Rename, Unlink, Fifo, UnlinkFifo, Regular, Symlink };
enum class Failure { None, Mmap, Protect, Open, Deserialize, AfterDeserialize };

// Linker wrappers below observe the actual production constructor/cleanup.
// They add no hook or access to the production MapDatabase interface.
struct Probe {
  bool active = false, probe_identity = true, replaced = false, pathname_open = false, before_mapping = false;
  Replacement replacement = Replacement::None;
  Failure failure = Failure::None;
  fs::path path, alternate;
  struct stat original {};
  std::string uuid;
  int fd = -1, maps = 0, opens = 0, deserializes = 0, sql_closes = 0, unmaps = 0, fd_closes = 0;
  sqlite3 *db = nullptr;
  void *mapped = MAP_FAILED;
  std::size_t size = 0;
  bool attached = false, identity_read = false, writes_rejected = false;
  std::map<std::string, std::string> expected_files;
  std::vector<std::string> errors;
} probe;

void observe(bool condition, const char *message) {
  if (!condition) probe.errors.emplace_back(message);
}
bool lockHeld() {
  const int other = open(("/proc/self/fd/" + std::to_string(probe.fd)).c_str(), O_RDONLY | O_CLOEXEC);
  if (other < 0) return false;
  const int rc = flock(other, LOCK_EX | LOCK_NB);
  const int error = errno;
  __real_close(other);
  return rc == -1 && (error == EAGAIN || error == EWOULDBLOCK);
}
void replacePath() {
  if (probe.replacement == Replacement::None || probe.replaced) return;
  if (probe.replacement == Replacement::Unlink || probe.replacement == Replacement::UnlinkFifo)
    fs::remove(probe.path);
  else
    fs::rename(probe.path, probe.path.string() + ".original");
  if (probe.replacement == Replacement::Fifo || probe.replacement == Replacement::UnlinkFifo)
    check(mkfifo(probe.path.c_str(), 0600) == 0, "replace path with FIFO");
  else if (probe.replacement == Replacement::Regular)
    fs::copy_file(probe.alternate, probe.path);
  else if (probe.replacement == Replacement::Symlink)
    fs::create_symlink(probe.alternate, probe.path);
  probe.replaced = true;
  probe.expected_files = files(probe.path.parent_path());
}
}  // namespace

extern "C" void *__wrap_mmap(void *address, size_t length, int protection, int flags, int fd, off_t offset) {
  if (!probe.active) return __real_mmap(address, length, protection, flags, fd, offset);
  probe.fd = fd;
  struct stat actual {};
  observe(fstat(fd, &actual) == 0 && actual.st_ino == probe.original.st_ino && actual.st_dev == probe.original.st_dev,
          "mapping did not use validated inode");
  observe(length == static_cast<std::size_t>(probe.original.st_size) && offset == 0, "mapping size/offset");
  observe(flags == MAP_PRIVATE && (protection & PROT_READ), "mapping must be private and readable");
  observe(lockHeld(), "validated inode must remain locked before mapping");
  if (probe.before_mapping) replacePath();
  if (probe.failure == Failure::Mmap) {
    errno = ENOMEM;
    return MAP_FAILED;
  }
  probe.mapped = __real_mmap(address, length, protection, flags, fd, offset);
  if (probe.mapped != MAP_FAILED) {
    probe.size = length;
    ++probe.maps;
  }
  return probe.mapped;
}
extern "C" int __wrap_mprotect(void *address, size_t size, int protection) {
  if (probe.active && address == probe.mapped) {
    observe(size == probe.size && protection == PROT_READ, "eligibility view must become read-only");
    if (probe.failure == Failure::Protect) {
      errno = EACCES;
      return -1;
    }
  }
  return __real_mprotect(address, size, protection);
}
extern "C" int __wrap_sqlite3_open_v2(const char *name, sqlite3 **db, int flags, const char *vfs) {
  if (!probe.active) return __real_sqlite3_open_v2(name, db, flags, vfs);
  // Same deterministic injection boundary as the independent review's reproducer.
  replacePath();
  probe.pathname_open = std::string(name) != ":memory:";
  ++probe.opens;
  const int rc = __real_sqlite3_open_v2(name, db, flags, vfs);
  probe.db = *db;
  return probe.failure == Failure::Open ? SQLITE_NOMEM : rc;
}
extern "C" int __wrap_sqlite3_deserialize(sqlite3 *db, const char *schema, unsigned char *data, sqlite3_int64 size, sqlite3_int64 capacity,
                                          unsigned flags) {
  if (!probe.active) return __real_sqlite3_deserialize(db, schema, data, size, capacity, flags);
  ++probe.deserializes;
  observe(db == probe.db && data == probe.mapped && size == static_cast<sqlite3_int64>(probe.size) && capacity == size,
          "deserialize must consume MapDatabase's mapping");
  observe(flags == SQLITE_DESERIALIZE_READONLY, "deserialize flags must not transfer/resize mmap ownership");
  auto expected = bytes(probe.fd);
  if (expected[18] == 2 && expected[19] == 2) expected[18] = expected[19] = 1;
  observe(std::memcmp(data, expected.data(), expected.size()) == 0, "private view differs beyond the two permitted WAL-header bytes");
  if (probe.failure == Failure::Deserialize) return SQLITE_NOMEM;
  const int rc = __real_sqlite3_deserialize(db, schema, data, size, capacity, flags);
  probe.attached = rc == SQLITE_OK;
  if (probe.attached && probe.probe_identity) {
    probe.identity_read = scalar(db, "SELECT map_uuid FROM MapState") == probe.uuid && scalar(db, "PRAGMA user_version") == "1" &&
                          scalar(db, "SELECT graph_revision FROM MapState") == "7" &&
                          scalar(db, "SELECT config_identity FROM MapState") == map_config_identity({}, {});
    probe.writes_rejected = sqlite3_exec(db, "UPDATE MapState SET graph_revision=99", nullptr, nullptr, nullptr) == SQLITE_READONLY &&
                            sqlite3_exec(db, "PRAGMA user_version=99", nullptr, nullptr, nullptr) == SQLITE_READONLY;
    observe(probe.identity_read && probe.writes_rejected, "original identity must be read and actual SQL mutations rejected");
  }
  return probe.failure == Failure::AfterDeserialize ? SQLITE_ERROR : rc;
}
extern "C" int __wrap_sqlite3_close(sqlite3 *db) {
  if (probe.active && db == probe.db) {
    observe(probe.unmaps == 0 && probe.fd_closes == 0, "SQLite close must precede unmap and descriptor close");
    observe(sqlite3_next_stmt(db, nullptr) == nullptr, "no statement may defer SQLite close");
    if (probe.attached) observe(!scalar(db, "PRAGMA user_version").empty(), "mapped buffer must still be readable at SQLite close");
    ++probe.sql_closes;
  }
  const int rc = __real_sqlite3_close(db);
  if (probe.active && db == probe.db) observe(rc == SQLITE_OK, "SQLite close failed");
  return rc;
}
extern "C" int __wrap_munmap(void *address, size_t size) {
  if (probe.active && address == probe.mapped) {
    observe(probe.db == nullptr || probe.sql_closes == 1, "unmap before SQLite closed");
    observe(probe.unmaps == 0 && probe.fd_closes == 0 && size == probe.size && lockHeld(), "mapping cleanup/ownership order");
    ++probe.unmaps;
  }
  return __real_munmap(address, size);
}
extern "C" int __wrap_close(int fd) {
  if (probe.active && fd == probe.fd) {
    observe(probe.maps == probe.unmaps && (probe.db == nullptr || probe.sql_closes == 1) && probe.fd_closes == 0,
            "descriptor must close once, after SQLite and mapping");
    ++probe.fd_closes;
  }
  return __real_close(fd);
}

namespace {
void exercise(const fs::path &path, const fs::path &alternate, const std::string &uuid, Replacement replacement = Replacement::None,
              Failure failure = Failure::None, std::optional<MapErrorCode> expected_error = {}, bool probe_identity = true,
              bool before_mapping = false) {
  const int verifier = open(path.c_str(), O_RDONLY | O_CLOEXEC);
  check(verifier >= 0, "open verification descriptor");
  const auto before = bytes(verifier);
  const auto descriptors = fdCount();
  probe = {};
  probe.path = path;
  probe.alternate = alternate;
  probe.uuid = uuid;
  probe.replacement = replacement;
  probe.failure = failure;
  probe.probe_identity = probe_identity;
  probe.before_mapping = before_mapping;
  probe.expected_files = files(path.parent_path());
  check(fstat(verifier, &probe.original) == 0, "record original inode");
  probe.active = true;
  std::optional<MapErrorCode> result;
  try {
    MapDatabase reader(path.string(), "resume");
    check(!reader.writable() && reader.mapUuid() == uuid && reader.graphRevision() == 7, "valid eligibility result");
  } catch (const MapError &error) {
    result = error.code();
  }
  probe.active = false;
  check(result == expected_error, "unexpected eligibility outcome");
  check(probe.errors.empty(), probe.errors.empty() ? "" : probe.errors.front());
  check(!probe.pathname_open, "SQLite reopened a filesystem pathname");
  check(probe.maps == probe.unmaps && (probe.db == nullptr || probe.sql_closes == 1), "resource cleanup counts");
  check(probe.fd < 0 || probe.fd_closes == 1, "validated descriptor not closed exactly once");
  check(fdCount() == descriptors, "constructor leaked a descriptor");
  check(flock(verifier, LOCK_EX | LOCK_NB) == 0, "ownership leaked after reader cleanup");
  check(bytes(verifier) == before, "canonical bytes/header/identity/revision changed");
  close(verifier);
  check(files(path.parent_path()) == probe.expected_files, "eligibility changed replacement, original file or sidecars");
  if (replacement != Replacement::None)
    check(probe.replaced && probe.identity_read && probe.writes_rejected, "replacement probe did not inspect original");
  if (!expected_error)
    check(probe.identity_read && probe.writes_rejected && probe.maps == 1 && probe.deserializes == 1, "valid map bypassed mapped SQL inspection");
}

template <class F>
void bounded(F run) {
  const pid_t child = fork();
  check(child >= 0, "fork watchdog");
  if (child == 0) {
    alarm(3);  // Test-only hang detection; production never relies on a timeout.
    try {
      run();
      _exit(0);
    } catch (const std::exception &error) {
      std::cerr << "child failure: " << error.what() << std::endl;
      _exit(1);
    }
  }
  int status = 0;
  check(waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
        "eligibility child failed or was killed by watchdog/memory fault");
}
}  // namespace

int main() {
  char directory[] = "/tmp/sapphire-a12-XXXXXX";
  if (!mkdtemp(directory)) return 1;
  const fs::path root(directory), seed = root / "seed.db", alternate = root / "alternate.db";
  try {
    std::string uuid;
    {
      MapDatabase writer(seed.string());
      uuid = writer.mapUuid();
    }
    {
      MapDatabase writer(alternate.string());
      check(writer.mapUuid() != uuid, "distinct replacement identity");
    }
    sql(seed, "UPDATE MapState SET graph_revision=7;");
    exercise(seed, alternate, uuid);
    sql(seed, "PRAGMA journal_mode=WAL;");
    check(bytes(seed)[18] == 2 && bytes(seed)[19] == 2, "clean WAL-header fixture");
    exercise(seed, alternate, uuid);
    std::cout << "PASS DELETE/WAL-header eligibility, actual SQL write rejection, byte identity and normal resource lifetime\n";

    int id = 0;
    for (const bool before_mapping : {false, true})
      for (auto replacement :
           {Replacement::Rename, Replacement::Unlink, Replacement::Fifo, Replacement::UnlinkFifo, Replacement::Regular, Replacement::Symlink}) {
        const auto path = root / ("race-" + std::to_string(id++) + ".db");
        fs::copy_file(seed, path);
        bounded([&] { exercise(path, alternate, uuid, replacement, Failure::None, MapErrorCode::StaleMap, true, before_mapping); });
      }
    std::cout << "PASS six namespace replacements before mmap and before SQLite open: original inode inspected, no pathname reopen or hang\n";

    for (auto failure : {Failure::Mmap, Failure::Protect, Failure::Open, Failure::Deserialize, Failure::AfterDeserialize})
      bounded([&] { exercise(seed, alternate, uuid, Replacement::None, failure, MapErrorCode::Storage); });
    const auto bad_schema = root / "bad-schema.db";
    fs::copy_file(seed, bad_schema);
    sql(bad_schema, "PRAGMA user_version=999;");
    bounded([&] { exercise(bad_schema, alternate, uuid, Replacement::None, Failure::None, MapErrorCode::UnsupportedFormat, false); });
    std::cout << "PASS mmap/protection/open/deserialize/post-attach/validation failures with ordered cleanup and no leaks\n";

    for (const auto size : {0, 16, 99}) {
      const auto path = root / ("short-" + std::to_string(size));
      std::ofstream(path, std::ios::binary) << std::string(size, 'x');
      exercise(path, alternate, uuid, Replacement::None, Failure::None, MapErrorCode::UnsupportedFormat, false);
    }
    for (const auto versions : {std::pair{0, 0}, std::pair{1, 2}, std::pair{2, 1}, std::pair{3, 3}}) {
      const auto path = root / ("header-" + std::to_string(versions.first) + std::to_string(versions.second));
      auto data = bytes(seed);
      data[18] = versions.first;
      data[19] = versions.second;
      std::ofstream(path, std::ios::binary).write(data.data(), data.size());
      exercise(path, alternate, uuid, Replacement::None, Failure::None, MapErrorCode::UnsupportedFormat, false);
    }
    const auto bad_magic = root / "bad-magic.db";
    auto data = bytes(seed);
    data[0] = 'X';
    std::ofstream(bad_magic, std::ios::binary).write(data.data(), data.size());
    exercise(bad_magic, alternate, uuid, Replacement::None, Failure::None, MapErrorCode::UnsupportedFormat, false);
    std::cout << "PASS invalid sizes, SQLite magic and malformed/unsupported header versions rejected without normalization\n";
    fs::remove_all(root);
    std::cout << "A1.2 eligibility tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << " (fixtures retained at " << root << ")\n";
    return 1;
  }
}
