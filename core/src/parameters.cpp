#include <toml++/toml.h>
#include <common/common_type.hpp>
#include <parameters.hpp>
#include <tools/logging.hpp>
#include <tools/strings.hpp>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iterator>
#include <limits>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <tuple>
#include "sensor/lidar_type.hpp"
#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace sapphire {

void Parameters::validateRecord(const ParametersMap& parameters) {
  for (const auto& [key, value] : parameters) {
    if (key.empty() || key.find_first_of(":;") != std::string::npos || key.find('\0') != std::string::npos || value.find(';') != std::string::npos ||
        value.find('\0') != std::string::npos)
      throw std::invalid_argument("Parameter record cannot encode entry: " + key);
  }
}

std::string Parameters::serialize(const ParametersMap& parameters) {
  validateRecord(parameters);
  std::stringstream output;
  for (ParametersMap::const_iterator iter = parameters.begin(); iter != parameters.end(); ++iter) {
    if (iter != parameters.begin()) {
      output << ";";
    }
    output << iter->first << ":" << iter->second;
  }
  EXT_LOG_DEBUG("output={}", output.str().c_str());
  return output.str();
}

ParametersMap Parameters::deserialize(const std::string& parameters) {
  EXT_LOG_DEBUG("parameters={}", parameters.c_str());
  ParametersMap output;
  for (const auto& entry : tools::split(parameters, ';')) {
    const auto colon = entry.find(':');
    if (colon == std::string::npos || colon == 0) throw std::invalid_argument("Invalid parameter record: " + entry);
    // Empty string values are valid; a parameter name may occur only once.
    const auto key = entry.substr(0, colon);
    if (!output.emplace(key, entry.substr(colon + 1)).second) throw std::invalid_argument("Duplicate parameter: " + key);
  }
  validateRecord(output);
  return output;
}

std::string_view scalar_text(const std::string& text) {
  const auto first = text.find_first_not_of(" \t\n\r\f\v");
  if (first == std::string::npos) return {};
  const auto last = text.find_last_not_of(" \t\n\r\f\v");
  return std::string_view(text).substr(first, last - first + 1);
}
bool equals_ascii(std::string_view value, std::string_view word) {
  if (value.size() != word.size()) return false;
  for (size_t i = 0; i < word.size(); ++i) {
    const char c = value[i] >= 'A' && value[i] <= 'Z' ? value[i] + ('a' - 'A') : value[i];
    if (c != word[i]) return false;
  }
  return true;
}
[[noreturn]] void invalid_scalar(std::string_view key, const std::string& text) {
  throw std::invalid_argument("Invalid parameter " + std::string(key) + "='" + text + "': expected a complete, finite value in range");
}
template <class T>
T scalar(std::string_view key, const std::string& text) {
  auto input = scalar_text(text);
  if constexpr (std::is_same_v<T, bool>) {
    if (equals_ascii(input, "true") || input == "1") return true;
    if (equals_ascii(input, "false") || input == "0") return false;
    invalid_scalar(key, text);
  } else {
    if (!input.empty() && input.front() == '+') {
      input.remove_prefix(1);
      if (!input.empty() && (input.front() == '+' || input.front() == '-')) invalid_scalar(key, text);
    }
    if (input.empty()) invalid_scalar(key, text);
    T result{};
    const auto parsed = std::from_chars(input.data(), input.data() + input.size(), result);
    if (parsed.ec != std::errc() || parsed.ptr != input.data() + input.size()) invalid_scalar(key, text);
    if constexpr (std::is_floating_point_v<T>) {
      if (!std::isfinite(result)) invalid_scalar(key, text);
    }
    return result;
  }
}
template <class T>
bool parse_scalar(const ParametersMap& parameters, std::string_view key, T& value) {
  const auto found = parameters.find(key);
  if (found == parameters.end()) return false;
  const T draft = scalar<T>(key, found->second);
  value = draft;
  return true;
}

template <class T>
bool Parameters::parse(const ParametersMap& parameters, std::string_view key, T& value) {
  if constexpr (std::is_same_v<T, std::string>) {
    const auto found = parameters.find(key);
    if (found == parameters.end()) return false;
    value = found->second;
    return true;
  } else
    return parse_scalar(parameters, key, value);
}
template bool Parameters::parse<bool>(const ParametersMap&, std::string_view, bool&);
template bool Parameters::parse<int>(const ParametersMap&, std::string_view, int&);
template bool Parameters::parse<unsigned int>(const ParametersMap&, std::string_view, unsigned int&);
template bool Parameters::parse<float>(const ParametersMap&, std::string_view, float&);
template bool Parameters::parse<double>(const ParametersMap&, std::string_view, double&);
template bool Parameters::parse<std::string>(const ParametersMap&, std::string_view, std::string&);

void Parameters::validate(const ParametersMap& parameters) {
  for (const auto& [key, value] : parameters) {
    const auto type = getType(key);
    if (!type) throw std::invalid_argument("Unknown algorithm parameter: " + key);
    switch (*type) {
      case scalar_type_t::Bool:
        scalar<bool>(key, value);
        break;
      case scalar_type_t::Int:
        scalar<int>(key, value);
        break;
      case scalar_type_t::UnsignedInt:
        scalar<unsigned int>(key, value);
        break;
      case scalar_type_t::Float:
        scalar<float>(key, value);
        break;
      case scalar_type_t::Double:
        scalar<double>(key, value);
        break;
      case scalar_type_t::String:
        break;
    }
  }
}

namespace {
void read_table(const toml::table& table, const std::string& prefix, ParametersMap& output) {
  using scalar_type_t = Parameters::scalar_type_t;
  for (const auto& [name, node] : table) {
    const auto key = prefix + std::string(name.str());
    const auto invalid = [&](const std::string& reason) {
      throw std::invalid_argument("line " + std::to_string(node.source().begin.line) + ": " + key + ": " + reason);
    };
    if (const auto* child = node.as_table()) {
      const auto& defaults = Parameters::getDefaultParameters();
      const auto first = defaults.lower_bound(key + '.');
      if (first == defaults.end() || first->first.compare(0, key.size() + 1, key + '.') != 0) invalid("unknown parameter table");
      read_table(*child, key + '.', output);
      continue;
    }
    const auto type = Parameters::getType(key);
    if (!type) invalid("unknown algorithm parameter");
    std::string value;
    switch (*type) {
      case scalar_type_t::Bool:
        if (!node.is_boolean()) invalid("requires a TOML boolean");
        value = node.as_boolean()->get() ? "true" : "false";
        break;
      case scalar_type_t::String:
        if (!node.is_string()) invalid("requires a TOML string");
        value = node.as_string()->get();
        break;
      case scalar_type_t::Int:
      case scalar_type_t::UnsignedInt:
        if (!node.is_integer()) invalid("requires a TOML integer");
        value = std::to_string(node.as_integer()->get());
        break;
      case scalar_type_t::Float:
      case scalar_type_t::Double:
        if (const auto* integer = node.as_integer())
          value = std::to_string(integer->get());
        else if (const auto* number = node.as_floating_point()) {
          std::ostringstream text;
          text.imbue(std::locale::classic());
          text << std::setprecision(std::numeric_limits<double>::max_digits10) << number->get();
          value = text.str();
        } else
          invalid("requires a TOML number");
        break;
    }
    if (!output.emplace(key, std::move(value)).second) invalid("duplicate parameter key");
  }
}
template <class C, class T>
struct Field {
  const char* key;
  T C::*member;
  const char* description;
};
template <class C, class T>
Field<C, T> field(const char* key, T C::*member, const char* description) {
  return {key, member, description};
}

auto bindings(const ArchiveParams&) {
  using C = ArchiveParams;
  return std::tuple{field("archive.keep_unlinked_nodes", &C::keep_unlinked_nodes, "Keep not linked nodes in db (rehearsed nodes and deleted nodes).")};
}


auto bindings(const DatabaseParams&) {
  using C = DatabaseParams;
  return std::tuple{
      field("database.sqlite.cache_size_pages", &C::sqlite_cache_size_pages,
            "PRAGMA cache_size: number of database pages kept in SQLite's page cache (approx. cacheSize * page_size bytes, often ~4 KiB per page). "
            "Larger "
            "values reduce disk I/O when the working set fits in RAM. SQLite built-in default is typically 2000 pages."),
      field("database.sqlite.in_memory", &C::sqlite_in_memory, "Using database in the memory instead of a file on the hard disk."),
      field(
          "database.sqlite.journal_mode", &C::sqlite_journal_mode,
          "PRAGMA journal_mode: rollback journal storage. See sqlite.org/pragma.html#pragma_journal_mode for more details. 0=DELETE (SQLite "
          "default): "
          "journal file deleted after each commit. 1=TRUNCATE: journal truncated to zero length. 2=PERSIST: journal file kept, header zeroed after "
          "commit. "
          "3=MEMORY: journal in RAM only; faster, weaker crash safety. 4=OFF: no journal; fastest, risk of corruption on crash. 5=WAL: transactional "
          "incremental writes."),
      field("database.sqlite.synchronous_mode", &C::sqlite_synchronous_mode,
            "PRAGMA synchronous: how aggressively SQLite syncs the database to disk. See sqlite.org/pragma.html#pragma_synchronous for more details. "
            "0=OFF: no "
            "wait for persistent storage; fastest, corruption possible on power loss. 1=NORMAL: sync at critical moments (common SQLite default with "
            "WAL). "
            "2=FULL (SQLite safest default): sync after every commit; slowest."),
      field("database.sqlite.temp_store", &C::sqlite_temp_store,
            "PRAGMA temp_store: where SQLite stores temporary tables and indices. See sqlite.org/pragma.html#pragma_temp_store for more details. "
            "0=DEFAULT: "
            "SQLite compile-time default (often on-disk temp files). 1=FILE: temporary files in the system temp directory. 2=MEMORY: temporary data "
            "in RAM "
            "when possible."),
      field("database.writer.batch_max_bytes", &C::writer_batch_max_bytes, "Maximum estimated bytes written in one database transaction."),
      field("database.writer.batch_max_items", &C::writer_batch_max_items, "Maximum complete batches written in one database transaction."),
      field("database.writer.batch_max_latency_ms", &C::writer_batch_max_latency_ms,
            "Maximum time the oldest pending database item waits for batching."),
      field("database.writer.commit_timeout_ms", &C::writer_commit_timeout_ms,
            "Acceptance-to-commit deadline in milliseconds, including pending, in-flight and commit time. Must be positive."),
      field("database.writer.max_bytes", &C::writer_max_bytes,
            "Maximum estimated bytes retained by the asynchronous database worker. Zero disables the byte limit."),
      field("database.writer.max_items", &C::writer_max_items,
            "Maximum number of pending and in-flight complete batches. Zero disables the item-count limit."),
      field("database.writer.producer_wait_ms", &C::writer_producer_wait_ms,
            "Capacity wait deadline in milliseconds; zero rejects full queues immediately. No retries.")};
}

auto bindings(const MemoryParams&) {
  using C = MemoryParams;
  return std::tuple{
      field("memory.eviction_order_by_id", &C::eviction_order_by_id,
            "On transfer, nodes are sorted by weight->ID only (i.e. the oldest of the lowest weighted nodes are transferred first). If false, the "
            "nodes are "
            "sorted by weight->Age->ID (i.e. the oldest inserted in WM of the lowest weighted nodes are transferred first). Note that retrieval "
            "updates the "
            "age, not the ID."),
      field("memory.load_all_nodes_on_start", &C::load_all_nodes_on_start,
            "Initialize the Working Memory with all nodes in Long-Term Memory. When false, it is initialized with nodes of the previous session."),
      field("memory.protected_local_ratio", &C::protected_local_ratio, "Ratio of working memory for which local nodes are immunized from transfer."),
      field("memory.protected_recent_ratio", &C::protected_recent_ratio,
            "Ratio of locations after the last loop closure in WM that cannot be transferred."),
      field("memory.recent_nodes", &C::recent_nodes, "Short-term memory size."),
      field("memory.resident_budget_bytes", &C::resident_budget_bytes,
            "WM node payload budget in bytes (0 disables). Evict least recently accessed eligible nodes to 90% after exceeding it. Excludes STM, "
            "global "
            "graph metadata and allocator overhead; not a process RSS limit."),
      field("memory.resident_budget_nodes", &C::resident_budget_nodes,
            "Maximum nodes in the Working Memory (0 means infinity). Similar to \"memory.update_budget_ms\", when the number of nodes in Working "
            "Memory (WM) "
            "exceeds this treshold, some nodes are transferred to Long-Term Memory to keep WM size fixed."),
      field("memory.update_budget_ms", &C::update_budget_ms,
            "Maximum time allowed for map update (ms) (0 means infinity). When map update time exceeds this fixed time threshold, some nodes in "
            "Working Memory "
            "(WM) are transferred to Long-Term Memory to limit the size of the WM and decrease the update time.")};
}

auto bindings(const PGOParams&) {
  using C = PGOParams;
  return std::tuple{
      field(
          "pose_graph.anchor_at_last_node", &C::anchor_at_last_node,
          "Optimize graph from the newest node. If false, the graph is optimized from the oldest node of the current graph (this adds an overhead "
          "computation to detect to oldest node of the current graph, but it can be useful to preserve the map referential from the oldest node). "
          "Warning "
          "when set to false: when some nodes are transferred, the first referential of the local map may change, resulting in momentary changes in "
          "robot/map position (which are annoying in teleoperation)."),
      field("pose_graph.gnc_enabled", &C::gnc_enabled,
            "Batch GNC-TLS pose-graph verification using LM; overrides pose_graph.incremental_enabled. Reject proposals containing suppressed or "
            "unresolved "
            "non-odometry constraints."),
      field("pose_graph.gravity_sigma", &C::gravity_sigma,
            "Gravity sigma value (>=0, typically between 0.1 and 0.3). Optimization is done while preserving gravity orientation of poses from "
            "LiDAR-inertial "
            "odometry. Set to 0 to disable gravity constraints."),
      field("pose_graph.ignore_constraint_variance", &C::ignore_constraint_variance,
            "Ignore constraints' variance. If checked, identity information matrix is used for each constraint. Otherwise, an information matrix is "
            "generated "
            "from the variance saved in the links."),
      field("pose_graph.ignore_priors", &C::ignore_priors, "Ignore global pose prior constraints while optimizing."),
      field("pose_graph.incremental_enabled", &C::incremental_enabled,
            "Do graph optimization incrementally with iSAM2. Only GaussNewton and Dogleg are supported in this mode (see pose_graph.optimizer)."),
      field("pose_graph.isam2.adaptive_reorder_enabled", &C::isam2_adaptive_reorder_enabled,
            "Reorder the Bayes tree when long-running incremental fill-in grows excessively."),
      field("pose_graph.isam2.relinearize_rotation_rad", &C::isam2_relinearize_rotation_rad,
            "Pose3 rotational relinearization threshold in radians."),
      field("pose_graph.isam2.relinearize_skip", &C::isam2_relinearize_skip, "Only relinearize variables every N calls to ISAM2::update()."),
      field("pose_graph.isam2.relinearize_translation_m", &C::isam2_relinearize_translation_m,
            "Pose3 and Point3 translational relinearization threshold in meters."),
      field("pose_graph.isam2.reorder_growth_ratio", &C::isam2_reorder_growth_ratio,
            "Trigger an adaptive batch reorder when Bayes-tree nonzeros exceed this multiple of the last fresh ordering."),
      field("pose_graph.isam2.wildfire_threshold", &C::isam2_wildfire_threshold, "Stop wildfire back-substitution below this change threshold."),
      field("pose_graph.max_constraint_error_sigma", &C::max_constraint_error_sigma,
            "Reject loop closures if optimization error ratio is greater than this value (0=disabled). Ratio is computed as absolute error over "
            "standard "
            "deviation of each link. This will help to detect when a wrong loop closure is added to the graph. With \"pose_graph.gnc_enabled\", GNC "
            "also "
            "rejects proposals containing unsupported constraints."),
      field("pose_graph.max_iterations", &C::max_iterations, "Optimization iterations."),
      field("pose_graph.min_error_improvement", &C::min_error_improvement, "Stop optimizing when the error improvement is less than this value."),
      field("pose_graph.optimizer", &C::optimizer, "0=Levenberg 1=GaussNewton 2=Dogleg"),
      field("pose_graph.repair_radius_m", &C::repair_radius_m,
            "If two consecutive loop closures are rejected by pose_graph.max_constraint_error_sigma on the same old loop closure link, we will "
            "remove that old "
            "link, and other old links under that radius if necessary, until optimization is accepted. When optimization is accepted, the old loop "
            "closure links "
            "are removed from the graph. This feature is useful to reject bad loop closures that were accepted previously. Set to 0 to disable this "
            "feature.")};
}

auto bindings(const RegistrationParams&) {
  using C = RegistrationParams;
  return std::tuple{field("registration.lidar.odometry_translation_sigma_m", &C::lidar_odometry_translation_sigma_m,
                          "Fixed submap odometry factor model, translation standard deviation in metres."),
                    field("registration.lidar.odometry_rotation_sigma_rad", &C::lidar_odometry_rotation_sigma_rad,
                          "Fixed submap odometry factor model, rotation standard deviation in radians."),
                    field("registration.lidar.bbs_enabled", &C::lidar_bbs_enabled, "Run bounded BBS before Gaussian GICP."),
                    field("registration.lidar.min_inliers", &C::lidar_min_inliers, "Minimum Gaussian correspondences."),
                    field("registration.lidar.max_iterations", &C::lidar_max_iterations, "Maximum serial GICP iterations."),
                    field("registration.lidar.max_correspondence_m", &C::lidar_max_correspondence_m, "Correspondence radius in metres."),
                    field("registration.lidar.min_overlap", &C::lidar_min_overlap, "Minimum fraction of query Gaussians supported by target."),
                    field("registration.lidar.max_fitness_m2", &C::lidar_max_fitness_m2, "Maximum mean squared Euclidean residual."),
                    field("registration.lidar.translation_sigma_m", &C::lidar_translation_sigma_m, "Graph translation uncertainty floor in metres."),
                    field("registration.lidar.rotation_sigma_rad", &C::lidar_rotation_sigma_rad, "Graph rotation uncertainty floor in radians.")};
}

auto bindings(const RelocalizerParams&) {
  using C = RelocalizerParams;
  return std::tuple{
      field("relocalizer.global.enabled", &C::global_enabled, "Automatic geometry retrieval and initial poses for all relocalization queries."),
      field("relocalizer.global.resolution_m", &C::global_resolution_m, "Fixed physical feature voxel size; final registration uses full Gaussians."),
      field("relocalizer.global.feature_radius_m", &C::global_feature_radius_m, "FPFH feature radius using frozen covariance normals."),
      field("relocalizer.global.clique_time_sec", &C::global_clique_time_sec, "Serial maximum-clique search time per pyramid level."),
      field("relocalizer.global.max_associations", &C::global_max_associations, "Maximum association graph vertices; oversized searches reject without adaptive sampling."),
      field("relocalizer.global.index_max_nodes", &C::global_index_max_nodes, "Maximum cached derived retrieval signatures; evict and rebuild on demand."),
      field("relocalizer.spatial.enabled", &C::spatial_enabled, "Enable bounded spatial candidate search and LiDAR registration."),
      field("relocalizer.spatial.max_candidates", &C::spatial_max_candidates,
            "One positive maximum of effective-path geometric trials per query, shared by all retrieval evidence."),
      field("relocalizer.spatial.radius_m", &C::spatial_radius_m,
            "Candidate search radius in metres within a committed metric component; never an acceptance radius."),
      field("relocalizer.spatial.max_graph_hops", &C::spatial_max_graph_hops,
            "Maximum graph edge distance from the query to a spatial candidate (0 disables this filter); not a cap on visited nodes."),
      field("relocalizer.path.max_nodes", &C::path_max_nodes, "Maximum nodes in a committed historical graph path; 1 is the single-node baseline."),
      field("relocalizer.path.max_bytes", &C::path_max_bytes, "Target Gaussian payload budget, not total working memory. Shorten context by whole Nodes; an oversized anchor is reported as resource-limited."),
      field("relocalizer.limit_covariance_by_odometry", &C::limit_covariance_by_odometry,
            "Limit covariance of non-neighbor links to minimum covariance of neighbor links. In other words, if covariance of a loop closure link is "
            "smaller "
            "than the minimum covariance of odometry links, its covariance is set to minimum covariance of odometry links."),
      field("relocalizer.max_loop_distance_m", &C::max_loop_distance_m,
            "Optional additional spatial retrieval radius in metres (0=disabled); not a post-registration rejection gate.")};
}

auto bindings(const StatisticsParams&) {
  using C = StatisticsParams;
  return std::tuple{
      field("statistics.buffer_logs", &C::buffer_logs, "Use bounded stream buffering; false flushes the statistics log after each observation."),
      field("statistics.log_enabled", &C::log_enabled, "Logging enabled."),
      field("statistics.log_headers", &C::log_headers, "Add column header description to log files."),
      field("statistics.save_resident_nodes", &C::save_resident_nodes, "Save working memory state after each update in statistics.")};
}

auto bindings(const NodeParams&) {
  using C = NodeParams;
  return std::tuple{field("node.generate_node_ids", &C::generate_node_ids, "Generate node IDs."),
                    field("node.ignore_bad_frames", &C::ignore_bad_frames, "Reject empty geometry.")};
}

auto bindings(const TopographParams&) {
  using C = TopographParams;
  return std::tuple{
      field("topograph.new_session_odometry_jump_m", &C::new_session_odometry_jump_m,
            "A new map is created if a change of odometry translation greater than X m is detected (0 m = disabled)."),
      field("topograph.start_session_on_good_frame", &C::start_session_on_good_frame, "Start a new map only if the first node has LiDAR geometry."),
      field("topograph.use_previous_map", &C::use_previous_map,
            "Use stored maps as localization/loop candidates. False starts a disconnected map in the same database."),
      field("topograph.automatic_map_labels", &C::automatic_map_labels,
            "Create map labels. The first node of a map will be labeled as \"map#\" where # is the map ID."),
      field("topograph.ignore_odometry_cross_covariance", &C::ignore_odometry_cross_covariance,
            "Ignore off diagonal values of the covariance matrix."),
      field("topograph.persistent_root_frames", &C::persistent_root_frames,
            "Persist one immutable coordinate anchor per component and publish distinct frame identities. Requires OptimizeFromGraphEnd=false. "
            "Existing "
            "components are migrated without recentering."),
      field("topograph.reduce_graph", &C::reduce_graph,
            "Reduce graph. Merge nodes when loop closures are added (ignoring those with user data). Note that this approach assumes that 100% of "
            "the loop "
            "closures accepted are good, so it is highly recommended to enable \"pose_graph.max_constraint_error_sigma\" at the same time."),
      field("topograph.use_odometry_gravity", &C::use_odometry_gravity,
            "Use gravity-aligned external odometry orientation to add gravity links to new nodes. Gravity constraints are used by graph optimization "
            "only if "
            "\"pose_graph.gravity_sigma\" is not zero.")};
}

template <class C, class F>
void visit_fields(C& config, F&& visitor) {
  std::apply([&](const auto&... item) { (visitor(item.key, config.*(item.member), item.description), ...); }, bindings(config));
}

// Shared traversal checks only representability. Each module owns its semantic constraints.
template <class C>
void finite_fields(const C& params) {
  visit_fields(params, [](const char* key, const auto& value, const char*) {
    if constexpr (std::is_floating_point_v<std::decay_t<decltype(value)>>)
      if (!std::isfinite(value)) throw std::invalid_argument(std::string(key) + " must be finite");
  });
}
template <class F>
void visit_defaults(F&& visitor) {
  visitor(ArchiveParams{});
  visitor(DatabaseParams{});
  visitor(MemoryParams{});
  visitor(PGOParams{});
  visitor(RegistrationParams{});
  visitor(RelocalizerParams{});
  visitor(StatisticsParams{});
  visitor(NodeParams{});
  visitor(TopographParams{});
}
template <class T>
std::string text(const T& value) {
  if constexpr (std::is_same_v<T, std::string>)
    return value;
  else if constexpr (std::is_same_v<T, bool>)
    return value ? "true" : "false";
  else {
    std::ostringstream output;
    output.imbue(std::locale::classic());
    if constexpr (std::is_floating_point_v<T>) output << std::setprecision(std::numeric_limits<T>::max_digits10);
    output << value;
    return output.str();
  }
}
template <class T>
constexpr Parameters::scalar_type_t scalar_type() {
  using scalar_type_t = Parameters::scalar_type_t;
  if constexpr (std::is_same_v<T, bool>)
    return scalar_type_t::Bool;
  else if constexpr (std::is_same_v<T, int>)
    return scalar_type_t::Int;
  else if constexpr (std::is_same_v<T, unsigned int>)
    return scalar_type_t::UnsignedInt;
  else if constexpr (std::is_same_v<T, float>)
    return scalar_type_t::Float;
  else if constexpr (std::is_same_v<T, double>)
    return scalar_type_t::Double;
  else {
    static_assert(std::is_same_v<T, std::string>, "Unsupported parameter wire type");
    return scalar_type_t::String;
  }
}
struct Descriptor {
  std::string default_value;
  Parameters::scalar_type_t type;
  const char* description;
};
const auto& catalog() {
  static const auto entries = [] {
    std::map<std::string, Descriptor, std::less<>> result;
    visit_defaults([&](const auto& module) {
      visit_fields(module, [&](const char* key, const auto& value, const char* description) {
        using T = std::decay_t<decltype(value)>;
        if (!result.emplace(key, Descriptor{text(value), scalar_type<T>(), description}).second)
          throw std::logic_error(std::string("Duplicate parameter binding: ") + key);
      });
    });
    return result;
  }();
  return entries;
}
template <class C>
C read_fields(C config, const ParametersMap& input) {
  visit_fields(config, [&](const char* key, auto& value, const char*) {
    Parameters::parse(input, key, value);
    if constexpr (std::is_floating_point_v<std::decay_t<decltype(value)>>)
      if (!std::isfinite(value)) throw std::invalid_argument(std::string(key) + " must be finite");
  });
  config.validate();
  return config;
}
template <class C>
C load(const C& config, const ParametersMap& input) {
  for (const auto& [key, value] : input)
    if (!catalog().count(key)) throw std::invalid_argument("Unknown algorithm parameter: " + key);
  return read_fields(config, input);
}
template <class C>
ParametersMap dump(const C& config) {
  config.validate();
  ParametersMap result;
  visit_fields(config, [&](const char* key, const auto& value, const char*) { result.emplace(key, text(value)); });
  return result;
}
}  // namespace

const ParametersMap& Parameters::getDefaultParameters() {
  static const auto defaults = [] {
    ParametersMap result;
    for (const auto& [key, item] : catalog()) result.emplace(key, item.default_value);
    return result;
  }();
  return defaults;
}
std::optional<Parameters::scalar_type_t> Parameters::getType(const std::string& key) {
  const auto item = catalog().find(key);
  if (item == catalog().end()) return std::nullopt;
  return item->second.type;
}
std::string Parameters::getDescription(const std::string& key) {
  const auto item = catalog().find(key);
  return item == catalog().end() ? "" : item->second.description;
}

ArchiveParams ArchiveParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap ArchiveParams::parameters() const { return dump(*this); }
void ArchiveParams::validate() const {}

DatabaseParams DatabaseParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap DatabaseParams::parameters() const { return dump(*this); }
void DatabaseParams::validate() const {
  finite_fields(*this);
  if (!sqlite_cache_size_pages || sqlite_journal_mode < 0 || sqlite_journal_mode > 5 || sqlite_synchronous_mode < 0 || sqlite_synchronous_mode > 2 ||
      sqlite_temp_store < 0 || sqlite_temp_store > 2)
    throw std::invalid_argument("Invalid SQLite cache or PRAGMA configuration");
  if (!writer_batch_max_items) throw std::invalid_argument("database.writer.batch_max_items must be positive");
  if (!writer_commit_timeout_ms) throw std::invalid_argument("database.writer.commit_timeout_ms must be positive");
}

MemoryParams MemoryParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap MemoryParams::parameters() const { return dump(*this); }
void MemoryParams::validate() const {
  finite_fields(*this);
  if (update_budget_ms < 0) throw std::invalid_argument("Memory update budget must be nonnegative");
  if (resident_budget_nodes < 0) throw std::invalid_argument("memory.resident_budget_nodes must be nonnegative");
  if (!(protected_recent_ratio >= 0 && protected_recent_ratio <= 1) || !(protected_local_ratio >= 0 && protected_local_ratio <= 1))
    throw std::invalid_argument("Memory protection ratios must be in [0,1]");
}

PGOParams PGOParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap PGOParams::parameters() const { return dump(*this); }
void PGOParams::validate() const {
  finite_fields(*this);
  if (optimizer < 0 || optimizer > 2 || max_iterations < 0 || min_error_improvement < 0 || gravity_sigma < 0 || max_constraint_error_sigma < 0 ||
      repair_radius_m < 0)
    throw std::invalid_argument("Invalid PGO optimizer, iteration budget or error thresholds");
  if (isam2_relinearize_rotation_rad <= 0 || isam2_relinearize_translation_m <= 0 || isam2_wildfire_threshold < 0 || isam2_relinearize_skip <= 0 ||
      isam2_reorder_growth_ratio <= 1)
    throw std::invalid_argument("Invalid iSAM2 relinearization or reorder parameters");
}

RegistrationParams RegistrationParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap RegistrationParams::parameters() const { return dump(*this); }
void RegistrationParams::validate() const {
  finite_fields(*this);
  if (lidar_min_inliers < 6 || lidar_max_iterations < 1 || lidar_max_correspondence_m <= 0 || lidar_min_overlap <= 0 || lidar_min_overlap > 1 ||
      lidar_max_fitness_m2 <= 0 || lidar_translation_sigma_m <= 0 || lidar_rotation_sigma_rad <= 0 || lidar_odometry_translation_sigma_m <= 0 ||
      lidar_odometry_rotation_sigma_rad <= 0)
    throw std::invalid_argument("Invalid LiDAR registration policy");
}

RelocalizerParams RelocalizerParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap RelocalizerParams::parameters() const { return dump(*this); }
void RelocalizerParams::validate() const {
  finite_fields(*this);
  if (global_resolution_m <= 0 || global_feature_radius_m <= global_resolution_m || global_clique_time_sec<=0 || !global_max_associations || global_max_associations>4096 || !global_index_max_nodes)
    throw std::invalid_argument("Invalid automatic relocalization resource/search bounds");
  if (!path_max_nodes || path_max_nodes > 9 || !path_max_bytes || !spatial_max_candidates || spatial_radius_m <= 0 || max_loop_distance_m < 0)
    throw std::invalid_argument("Invalid spatial candidate budget or radius");
}

StatisticsParams StatisticsParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap StatisticsParams::parameters() const { return dump(*this); }
void StatisticsParams::validate() const {}

NodeParams NodeParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap NodeParams::parameters() const { return dump(*this); }
void NodeParams::validate() const {}

TopographParams TopographParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap TopographParams::parameters() const { return dump(*this); }

void TopographParams::validate() const {
  finite_fields(*this);
  if (new_session_odometry_jump_m < 0) throw std::invalid_argument("Invalid odometry jump threshold");
}

MappingParameters MappingParameters::from_record(const ParametersMap& values) {
  return MappingParameters{}.withOverrides(values);
}
MappingParameters MappingParameters::withOverrides(const ParametersMap& values) const {
  Parameters::validate(values);
  MappingParameters result = *this;
  result.archive = archive.withOverrides(values);
  result.database = database.withOverrides(values);
  result.memory = memory.withOverrides(values);
  result.node = node.withOverrides(values);
  result.pose_graph = pose_graph.withOverrides(values);
  result.registration = registration.withOverrides(values);
  result.relocalizer = relocalizer.withOverrides(values);
  result.statistics = statistics.withOverrides(values);
  result.topograph = topograph.withOverrides(values);
  result.validate();
  return result;
}
ParametersMap MappingParameters::parameters() const {
  ParametersMap result;
  { const auto values = archive.parameters(); result.insert(values.begin(), values.end()); }
  { const auto values = database.parameters(); result.insert(values.begin(), values.end()); }
  { const auto values = memory.parameters(); result.insert(values.begin(), values.end()); }
  { const auto values = node.parameters(); result.insert(values.begin(), values.end()); }
  { const auto values = pose_graph.parameters(); result.insert(values.begin(), values.end()); }
  { const auto values = registration.parameters(); result.insert(values.begin(), values.end()); }
  { const auto values = relocalizer.parameters(); result.insert(values.begin(), values.end()); }
  { const auto values = statistics.parameters(); result.insert(values.begin(), values.end()); }
  { const auto values = topograph.parameters(); result.insert(values.begin(), values.end()); }
  return result;
}
void MappingParameters::validate() const {
  archive.validate();
  database.validate();
  memory.validate();
  node.validate();
  pose_graph.validate();
  registration.validate();
  relocalizer.validate();
  statistics.validate();
  topograph.validate();
  if (topograph.persistent_root_frames && (pose_graph.anchor_at_last_node || pose_graph.max_iterations <= 0))
    throw std::invalid_argument("Persistent component frames require oldest datum and an enabled optimizer");
}

namespace {

void require_positive(double value, const char *name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
}

void require_nonnegative(double value, const char *name) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and nonnegative");
  }
}

void require_positive(int value, const char *name) {
  if (value <= 0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
}

template <typename T>
void read_value(const toml::table &config, const char *path, T &value) {
  const auto node = config.at_path(path);
  if (!node) {
    return;
  }
  const std::optional<T> parsed = node.value<T>();
  if (!parsed) {
    throw std::invalid_argument(std::string(path) + " has an invalid type");
  }
  value = *parsed;
}

void read_array(const toml::table &config, const char *path, std::vector<double> &values) {
  const auto node = config.at_path(path);
  if (!node) {
    return;
  }
  const toml::array *array = node.as_array();
  if (array == nullptr) {
    throw std::invalid_argument(std::string(path) + " must be an array");
  }

  std::vector<double> parsed;
  parsed.reserve(array->size());
  for (const toml::node &node : *array) {
    const std::optional<double> value = node.value<double>();
    if (!value) {
      throw std::invalid_argument(std::string(path) + " must contain only numbers");
    }
    parsed.push_back(*value);
  }
  values = std::move(parsed);
}

void validate_sensor(SensorParameters &sensor) {
  if (sensor.lidar_type < 0 || sensor.lidar_type > 3) {
    throw std::invalid_argument("Sensor.lidar_type must be 0, 1, 2 or 3");
  }
  require_positive(sensor.point_filter_num, "Sensor.point_filter_num");
  require_positive(sensor.max_scan_duration, "Sensor.max_scan_duration");
  require_nonnegative(sensor.blind, "Sensor.blind");
  if (sensor.extrinsic_tran.size() != 3) {
    throw std::invalid_argument("Sensor.extrinsic_tran must contain 3 values");
  }
  if (sensor.extrinsic_rota.size() != 9) {
    throw std::invalid_argument("Sensor.extrinsic_rota must contain 9 values, got " + std::to_string(sensor.extrinsic_rota.size()));
  }
  for (double value : sensor.extrinsic_tran) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Sensor.extrinsic_tran must be finite");
    }
  }
  for (double value : sensor.extrinsic_rota) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Sensor.extrinsic_rota must be finite");
    }
  }
  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotation(sensor.extrinsic_rota.data());
  if (!rotation.isUnitary(1e-6) || std::abs(rotation.determinant() - 1.0) > 1e-6) {
    throw std::invalid_argument("Sensor.extrinsic_rota must be a rotation matrix");
  }

  sensor.blind_squared = sensor.blind * sensor.blind;
  if (!std::isfinite(sensor.blind_squared) || !std::isfinite(sensor.time_offset))
    throw std::invalid_argument("Invalid sensor blind range or time offset");
}

}  // namespace

SensorConfig load_sensor_parameters(const std::filesystem::path &path) {
  try {
    const auto config = toml::parse_file(path.string());
    auto keys = [](const toml::table &table, std::initializer_list<std::string_view> allowed) {
      for (const auto &[key, value] : table)
        if (std::find(allowed.begin(), allowed.end(), key.str()) == allowed.end())
          throw std::invalid_argument("Unknown sensor parameter: " + std::string(key.str()));
    };
    auto table = [&](const char *name) -> const toml::table & {
      const auto *result = config[name].as_table();
      if (!result) throw std::invalid_argument(std::string(name) + " must be a table");
      return *result;
    };
    auto topic = [](const toml::table &table) {
      auto value = table["topic"].value<std::string>();
      if (!value || value->empty() || value->find_first_of(" \t\r\n") != std::string::npos || value->find('\0') != std::string::npos)
        throw std::invalid_argument("Each sensor requires a nonempty topic without whitespace");
      return *value;
    };
    keys(config, {"imu", "lidar0", "lidar1", "preprocess", "sync"});
    SensorConfig result;
    const auto &imu = table("imu");
    keys(imu, {"topic", "rate_hz"});
    result.imu_topic = topic(imu);
    read_value(imu, "rate_hz", result.imu_rate_hz);
    require_positive(result.imu_rate_hz, "imu.rate_hz");
    SensorParameters defaults;
    if (config.contains("preprocess")) {
      const auto &preprocess = table("preprocess");
      keys(preprocess, {"max_scan_duration", "point_filter_num", "blind"});
      read_value(preprocess, "max_scan_duration", defaults.max_scan_duration);
      read_value(preprocess, "point_filter_num", defaults.point_filter_num);
      read_value(preprocess, "blind", defaults.blind);
    }
    std::set<std::string> topics{result.imu_topic};
    for (const char *name : {"lidar0", "lidar1"}) {
      if (std::string_view(name) == "lidar1" && !config.contains(name)) break;
      const auto &lidar = table(name);
      keys(lidar, {"type", "topic", "time_offset", "extrinsic_tran", "extrinsic_rota"});
      auto sensor = defaults;
      sensor.topic = topic(lidar);
      if (!topics.insert(sensor.topic).second) throw std::invalid_argument("Sensor topics must be distinct");
      if (const auto type = lidar["type"].value<std::string>()) {
        sensor.lidar_type = static_cast<int>(parse_lidar_type(*type));
      } else if (lidar["type"].is_integer()) {
        const auto type = lidar["type"].value<int>();
        if (!type || *type < 0 || *type > 3)
          throw std::invalid_argument(std::string(name) + ".type must be an integer in [0, 3]");
        sensor.lidar_type = *type;
      } else {
        throw std::invalid_argument(std::string(name) + ".type requires a model name or integer in [0, 3]");
      }
      read_value(lidar, "time_offset", sensor.time_offset);
      // Never silently use identity when the installation calibration is missing.
      sensor.extrinsic_tran.clear(); sensor.extrinsic_rota.clear();
      read_array(lidar, "extrinsic_tran", sensor.extrinsic_tran);
      read_array(lidar, "extrinsic_rota", sensor.extrinsic_rota);
      validate_sensor(sensor);
      result.lidars.push_back(std::move(sensor));
    }
    if (config.contains("sync")) {
      const auto &sync = table("sync");
      keys(sync, {"lidar_merge_hz", "max_time_gap", "max_wait", "min_points", "queue_size"});
      read_value(sync, "lidar_merge_hz", result.lidar_merge_hz);
      read_value(sync, "max_time_gap", result.max_time_gap);
      read_value(sync, "max_wait", result.max_wait);
      read_value(sync, "min_points", result.min_points);
      read_value(sync, "queue_size", result.queue_size);
    }
    require_positive(result.max_time_gap, "sync.max_time_gap");
    require_positive(result.max_wait, "sync.max_wait");
    require_positive(result.min_points, "sync.min_points");
    require_nonnegative(result.lidar_merge_hz, "sync.lidar_merge_hz");
    if (result.queue_size < 2 || result.lidar_merge_hz > 100.)
      throw std::invalid_argument("sync.queue_size must be >=2 and lidar_merge_hz <=100");
    if (result.lidars.size() == 2) {
      for (const auto &lidar : result.lidars)
        if (lidar.lidar_type != static_cast<int>(LidarType::Robosense))
          throw std::invalid_argument("Dual input currently requires two RoboSense/Airy LiDARs");
      if (result.lidar_merge_hz > 0. &&
          (result.lidar_merge_hz * 5. > result.imu_rate_hz || 1. / result.lidar_merge_hz > defaults.max_scan_duration))
        throw std::invalid_argument("Fusion rate must leave >=5 IMU samples/window and fit preprocess.max_scan_duration");
    }
    return result;
  } catch (const std::exception &error) {
    throw std::invalid_argument(path.string() + ": " + error.what());
  }
}

SapphireParameters load_parameters(const std::filesystem::path &path, const SensorParameters &sensor) {
  const toml::table config = toml::parse_file(path.string());
  SapphireParameters parameters;
  parameters.sensor = sensor;

  read_value(config, "general.save_path", parameters.general.save_path);
  read_value(config, "map.mode", parameters.pose_graph.map_mode);
  read_value(config, "map.database_path", parameters.pose_graph.database_path);

  if (config.contains("sensor"))
    throw std::invalid_argument("Move [sensor] to sensors.toml [lidar0] / [preprocess]");

  read_value(config, "initializer.imu_init_samples", parameters.initializer.imu_init_samples);
  read_value(config, "initializer.down_size", parameters.initializer.down_size);
  read_value(config, "initializer.max_iterations", parameters.initializer.max_iterations);
  read_value(config, "initializer.min_plane_factors", parameters.initializer.min_plane_factors);
  read_value(config, "initializer.ba_iterations", parameters.initializer.ba_iterations);
  read_value(config, "initializer.convergence_threshold", parameters.initializer.convergence_threshold);
  read_value(config, "initializer.refined_convergence_threshold", parameters.initializer.refined_convergence_threshold);
  read_value(config, "initializer.degeneracy_threshold", parameters.initializer.degeneracy_threshold);
  read_value(config, "initializer.min_eigen_value", parameters.initializer.min_eigen_value);
  read_value(config, "initializer.plane_eigen_value_thre", parameters.initializer.plane_eigen_value_thre);

  read_value(config, "odometry.frontend", parameters.odometry.frontend);
  if (config.at_path("odometry.gaussian_fallback"))
    throw std::invalid_argument("odometry.gaussian_fallback was renamed to odometry.gicp_fallback; update the configuration");
  read_value(config, "odometry.gicp_fallback.enabled", parameters.odometry.gicp_fallback.enabled);
  read_value(config, "odometry.gicp_fallback.source_resolution", parameters.odometry.gicp_fallback.source_resolution);
  read_value(config, "odometry.gicp_fallback.max_distance", parameters.odometry.gicp_fallback.max_distance);
  read_value(config, "odometry.gicp_fallback.variance_floor", parameters.odometry.gicp_fallback.variance_floor);
  read_value(config, "odometry.gicp_fallback.max_iterations", parameters.odometry.gicp_fallback.max_iterations);
  if (config.at_path("odometry.ricp"))
    throw std::invalid_argument("The standalone RICP configuration is retired; use frontend=eskf and odometry.gicp_fallback.enabled=true");
  read_value(config, "odometry.cov_gyr", parameters.odometry.cov_gyr);
  read_value(config, "odometry.cov_acc", parameters.odometry.cov_acc);
  read_value(config, "odometry.rdw_gyr", parameters.odometry.rdw_gyr);
  read_value(config, "odometry.rdw_acc", parameters.odometry.rdw_acc);
  read_value(config, "odometry.down_size", parameters.odometry.down_size);
  read_value(config, "odometry.dept_err", parameters.odometry.dept_err);
  read_value(config, "odometry.beam_err", parameters.odometry.beam_err);
  read_value(config, "odometry.voxel_size", parameters.odometry.voxel_size);
  read_value(config, "odometry.fixed_map_budget_bytes", parameters.odometry.fixed_map_budget_bytes);
  read_value(config, "odometry.min_eigen_value", parameters.odometry.min_eigen_value);
  read_value(config, "odometry.degrade_bound", parameters.odometry.degrade_bound);
  read_value(config, "recovery.input_timeout_sec", parameters.recovery.input_timeout_sec);
  read_value(config, "recovery.max_imu_gap_sec", parameters.recovery.max_imu_gap_sec);
  read_value(config, "recovery.max_lidar_gap_sec", parameters.recovery.max_lidar_gap_sec);

  read_value(config, "local_submap.win_size", parameters.local_submap.win_size);
  read_value(config, "local_submap.max_layer", parameters.local_submap.max_layer);
  read_value(config, "local_submap.cov_gyr", parameters.local_submap.cov_gyr);
  read_value(config, "local_submap.cov_acc", parameters.local_submap.cov_acc);
  read_value(config, "local_submap.rdw_gyr", parameters.local_submap.rdw_gyr);
  read_value(config, "local_submap.rdw_acc", parameters.local_submap.rdw_acc);
  read_array(config, "local_submap.plane_eigen_value_thre", parameters.local_submap.plane_eigen_value_thre);
  read_value(config, "local_submap.imu_coef", parameters.local_submap.imu_coef);
  read_value(config, "local_submap.thread_num", parameters.local_submap.thread_num);

  read_value(config, "pose_graph.enabled", parameters.pose_graph.enabled);
  read_value(config, "pose_graph.submap_travel_distance", parameters.pose_graph.submap_travel_distance);
  read_value(config, "pose_graph.update_period_sec", parameters.pose_graph.update_period_sec);

  int cloud_cache_mb = 128;
  read_value(config, "storage.cloud_cache_mb", cloud_cache_mb);
  if (cloud_cache_mb < 1 || cloud_cache_mb > 4095)
    throw std::invalid_argument("Storage cache budget must be 1..4095 MiB");
  parameters.pose_graph.mapping.memory.resident_budget_bytes = static_cast<unsigned int>(cloud_cache_mb) * 1024U * 1024U;
  parameters.pose_graph.mapping.relocalizer.spatial_radius_m = parameters.pose_graph.submap_travel_distance * 2;
  if (const auto *mapping = config["mapping"].as_table()) {
    ParametersMap values;
    read_table(*mapping, "", values);
    parameters.pose_graph.mapping = parameters.pose_graph.mapping.withOverrides(values);
  }
  if (config.contains("backend"))
    throw std::invalid_argument("Rename [backend.*] to [mapping.*]");
  if (config.contains("visual_loop")) throw std::invalid_argument("visual_loop was removed; use LiDAR/IMU inputs");

  validate_parameters(parameters);
  return parameters;
}

void validate_parameters(SapphireParameters &parameters) {
  const auto &mode = parameters.pose_graph.map_mode;
  if (mode != "new" && mode != "resume" && mode != "localize") throw std::invalid_argument("map.mode must be new, resume, or localize");
  if (mode == "resume" && parameters.pose_graph.database_path.empty())
    throw std::invalid_argument("map.database_path is required for resume eligibility validation");
  if (mode == "new" && !parameters.pose_graph.enabled && !parameters.pose_graph.database_path.empty())
    throw std::invalid_argument("an explicit map.database_path requires pose_graph.enabled");
  validate_sensor(parameters.sensor);

  require_positive(parameters.initializer.imu_init_samples, "Initializer.imu_init_samples");
  require_positive(parameters.initializer.down_size, "Initializer.down_size");
  require_positive(parameters.initializer.max_iterations, "Initializer.max_iterations");
  require_positive(parameters.initializer.min_plane_factors, "Initializer.min_plane_factors");
  require_positive(parameters.initializer.ba_iterations, "Initializer.ba_iterations");
  require_positive(parameters.initializer.convergence_threshold, "Initializer.convergence_threshold");
  require_positive(parameters.initializer.refined_convergence_threshold, "Initializer.refined_convergence_threshold");
  require_positive(parameters.initializer.degeneracy_threshold, "Initializer.degeneracy_threshold");
  require_positive(parameters.initializer.min_eigen_value, "Initializer.min_eigen_value");
  require_positive(parameters.initializer.plane_eigen_value_thre, "Initializer.plane_eigen_value_thre");

  if (parameters.odometry.frontend != "eskf")
    throw std::invalid_argument("odometry.frontend must be eskf; RICP is now an optional component enabled by odometry.gicp_fallback.enabled=true");
  require_positive(parameters.odometry.gicp_fallback.source_resolution, "Odometry.gicp_fallback.source_resolution");
  require_positive(parameters.odometry.gicp_fallback.max_distance, "Odometry.gicp_fallback.max_distance");
  require_positive(parameters.odometry.gicp_fallback.variance_floor, "Odometry.gicp_fallback.variance_floor");
  if (parameters.odometry.gicp_fallback.max_iterations < 1 || parameters.odometry.gicp_fallback.max_iterations > 20 ||
      (parameters.odometry.gicp_fallback.enabled && parameters.odometry.gicp_fallback.max_distance > parameters.odometry.voxel_size))
    throw std::invalid_argument("GICP fallback requires 1..20 iterations and distance <= root voxel size");
  require_positive(parameters.odometry.cov_gyr, "Odometry.cov_gyr");
  require_positive(parameters.odometry.cov_acc, "Odometry.cov_acc");
  require_positive(parameters.odometry.rdw_gyr, "Odometry.rdw_gyr");
  require_positive(parameters.odometry.rdw_acc, "Odometry.rdw_acc");
  require_positive(parameters.odometry.down_size, "Odometry.down_size");
  require_positive(parameters.odometry.dept_err, "Odometry.dept_err");
  require_positive(parameters.odometry.beam_err, "Odometry.beam_err");
  require_positive(parameters.odometry.voxel_size, "Odometry.voxel_size");
  if (parameters.odometry.fixed_map_budget_bytes == 0)
    throw std::invalid_argument("Odometry.fixed_map_budget_bytes must be positive");
  require_positive(parameters.odometry.min_eigen_value, "Odometry.min_eigen_value");
  require_positive(parameters.odometry.degrade_bound, "Odometry.degrade_bound");
  if (!std::isfinite(parameters.recovery.input_timeout_sec) || parameters.recovery.input_timeout_sec < 0)
    throw std::invalid_argument("Recovery.input_timeout_sec must be finite and nonnegative");
  require_positive(parameters.recovery.max_imu_gap_sec, "Recovery.max_imu_gap_sec");
  require_positive(parameters.recovery.max_lidar_gap_sec, "Recovery.max_lidar_gap_sec");

  if (parameters.local_submap.win_size < 2) throw std::invalid_argument("LocalSubmap.win_size must be at least two");
  require_positive(parameters.local_submap.max_layer, "LocalSubmap.max_layer");
  if (parameters.local_submap.max_layer > 30) {
    throw std::invalid_argument("LocalSubmap.max_layer must not exceed 30");
  }
  require_positive(parameters.local_submap.cov_gyr, "LocalSubmap.cov_gyr");
  require_positive(parameters.local_submap.cov_acc, "LocalSubmap.cov_acc");
  require_positive(parameters.local_submap.rdw_gyr, "LocalSubmap.rdw_gyr");
  require_positive(parameters.local_submap.rdw_acc, "LocalSubmap.rdw_acc");
  require_positive(parameters.local_submap.imu_coef, "LocalSubmap.imu_coef");
  require_positive(parameters.local_submap.thread_num, "LocalSubmap.thread_num");
  if (parameters.local_submap.plane_eigen_value_thre.size() <= static_cast<size_t>(parameters.local_submap.max_layer)) {
    throw std::invalid_argument("LocalSubmap.plane_eigen_value_thre must cover max_layer");
  }
  parameters.local_submap.plane_eigen_value_thre_inv.clear();
  for (double threshold : parameters.local_submap.plane_eigen_value_thre) {
    require_positive(threshold, "LocalSubmap.plane_eigen_value_thre");
    parameters.local_submap.plane_eigen_value_thre_inv.push_back(1.0 / threshold);
  }

  require_positive(parameters.pose_graph.submap_travel_distance, "PoseGraph.submap_travel_distance");
  require_positive(parameters.pose_graph.update_period_sec, "PoseGraph.update_period_sec");

  parameters.pose_graph.mapping.validate();

  parameters.initializer.down_size_inv = 1.0 / parameters.initializer.down_size;
  parameters.initializer.plane_eigen_value_thre_inv = 1.0 / parameters.initializer.plane_eigen_value_thre;
  parameters.odometry.down_size_inv = 1.0 / parameters.odometry.down_size;
  parameters.odometry.voxel_size_inv = 1.0 / parameters.odometry.voxel_size;
}

std::string map_config_identity(const PoseGraphParameters &graph) {
  std::ostringstream encoded;
  encoded.imbue(std::locale::classic());
  encoded << std::setprecision(std::numeric_limits<double>::max_digits10);
  const auto add = [&](const auto &value) { encoded << value << ';'; };
  add("sapphire-lidar-map-config-5");
  // FNV-1a: stable across processes/platforms; a compatibility check, not authentication.
  std::uint64_t hash = 14695981039346656037ULL;
  for (unsigned char byte : encoded.str()) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  std::ostringstream result;
  result.imbue(std::locale::classic());
  result << "c5:" << std::hex << std::setfill('0') << std::setw(16) << hash;
  return result.str();
}

}  // namespace sapphire
