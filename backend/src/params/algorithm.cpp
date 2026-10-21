#include <backend/common/common_type.hpp>
#include <backend/parameters.hpp>
#include <backend/params/archive.hpp>
#include <backend/params/compression.hpp>
#include <backend/params/database.hpp>
#include <backend/params/feature.hpp>
#include <backend/params/memory.hpp>
#include <backend/params/navimap.hpp>
#include <backend/params/node.hpp>
#include <backend/params/pgo.hpp>
#include <backend/params/preprocess.hpp>
#include <backend/params/registration.hpp>
#include <backend/params/relocalizer.hpp>
#include <backend/params/statistics.hpp>
#include <backend/params/topograph.hpp>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <type_traits>

namespace sapphire::backend {
namespace {
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
  return std::tuple{field("archive.keep_images_in_memory", &C::keep_images_in_memory, "Keep raw images in RAM."),
                    field("archive.keep_intermediate_data", &C::keep_intermediate_data, "Keep intermediate node data in db."),
                    field("archive.keep_sensor_data", &C::keep_sensor_data, "Keep binary data in db."),
                    field("archive.keep_unlinked_nodes", &C::keep_unlinked_nodes, "Keep not linked nodes in db (rehearsed nodes and deleted nodes)."),
                    field("archive.parallel_compression", &C::parallel_compression, "Compression of sensor data is multi-threaded.")};
}

auto bindings(const LocalBAParams&) {
  using C = LocalBAParams;
  return std::tuple{field("node.local_ba.max_optimized_points", &C::max_optimized_points,
                          "Maximum optimized points in one local submap BA. Does not limit the persistent submap feature bank."),
                    field("node.local_ba.max_iterations", &C::max_iterations, "Maximum solver iterations for local node BA.")};
}

auto bindings(const CompressionParams&) {
  using C = CompressionParams;
  return std::tuple{
      field("compression.detect.angle_deg", &C::detect_angle_deg, "Maximum orientation difference in degrees."),
      field("compression.detect.iterations", &C::detect_iterations, "Number of loop detection rounds."),
      field("compression.detect.max_radius_m", &C::detect_max_radius_m, "Maximum spatial search radius in meters."),
      field("compression.detect.min_radius_m", &C::detect_min_radius_m, "Minimum spatial search radius in meters."),
      field("compression.detect.scope", &C::detect_scope, "Session scope: both, intra or inter."),
      field("compression.detect.session_id", &C::detect_session_id, "Session ID: -2 all sessions, -1 latest session, otherwise a nonnegative ID."),
      field("compression.merge.keep_latest", &C::merge_keep_latest, "Merge older nodes into newer nodes."),
      field("compression.merge.keep_linked", &C::merge_keep_linked, "Retain links from reduced nodes."),
      field("compression.merge.radius_m", &C::merge_radius_m, "Maximum loop-edge distance for merging in meters.")};
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

auto bindings(const FeatureParams&) {
  using C = FeatureParams;
  return std::tuple{
      field("registration.feature.enabled", &C::enabled, "Enable visual features and appearance retrieval; LiDAR remains independent."),
      field("registration.feature.method", &C::method,
            "0: FAST + ORB + FBoW; 1: FAST + XFeat + SigLIP2 cosine FAISS. Stereo depth always uses ZNCC."),
      field("registration.feature.fast.secondary_detection_enabled", &C::fast_secondary_detection_enabled,
            "Run a secondary FAST pass with feature.fast.secondary_threshold in cells where the initial pass did not find enough features."),
      field("registration.feature.fast.secondary_threshold", &C::fast_secondary_threshold,
            "Minimum FAST threshold used by the secondary detection pass of the SIMD ORB extractor."),
      field("registration.feature.fast.threshold", &C::fast_threshold,
            "Threshold on difference between intensity of the central pixel and pixels of a circle around this pixel."),
      field("registration.feature.max_features", &C::max_features,
            "Maximum features extracted from the images (0 means not bounded, <0 means no extraction)."),
      field("registration.feature.fast.harris_k", &C::orb_harris_k, "Harris response coefficient used by the SIMD ORB extractor."),
      field("registration.feature.pyramid.levels", &C::orb_pyramid_levels,
            "Number of ORB pyramid levels, including the input image. The last level is scaled by pow(scaleFactor, nlevels-1)."),
      field("registration.feature.pyramid.scale", &C::orb_pyramid_scale,
            "Scale ratio between adjacent ORB pyramid levels; must be greater than one.")};
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
      field("memory.feature_cache_nodes", &C::feature_cache_nodes,
            "Recent feature blocks; also bounds the additional demand/predictive submap cache. Range 12-1024."),
      field("memory.load_all_nodes_on_start", &C::load_all_nodes_on_start,
            "Initialize the Working Memory with all nodes in Long-Term Memory. When false, it is initialized with nodes of the previous session."),
      field("memory.predictive_prefetch_enabled", &C::predictive_prefetch_enabled,
            "Prefetch next-place submap features into the version-checked feature cache. Demand reads reuse this cache independently."),
      field("memory.preload_visual_features", &C::preload_visual_features,
            "Preload recent BoW feature blocks up to memory.feature_cache_nodes. Otherwise initialize from retrieval summaries and read candidate "
            "blocks on "
            "demand."),
      field("memory.protected_local_ratio", &C::protected_local_ratio, "Ratio of working memory for which local nodes are immunized from transfer."),
      field("memory.protected_recent_ratio", &C::protected_recent_ratio,
            "Ratio of locations after the last loop closure in WM that cannot be transferred."),
      field("memory.recent_nodes", &C::recent_nodes, "Short-term memory size."),
      field("memory.resident_budget_bytes", &C::resident_budget_bytes,
            "WM node payload budget in bytes (0 disables). Evict least recently accessed eligible nodes to 90% after exceeding it. Excludes STM, "
            "global "
            "retrieval index, immutable vocabulary, map layers and allocator overhead; not a process RSS limit."),
      field("memory.resident_budget_nodes", &C::resident_budget_nodes,
            "Maximum nodes in the Working Memory (0 means infinity). Similar to \"memory.update_budget_ms\", when the number of nodes in Working "
            "Memory (WM) "
            "exceeds this treshold, some nodes are transferred to Long-Term Memory to keep WM size fixed."),
      field("memory.update_budget_ms", &C::update_budget_ms,
            "Maximum time allowed for map update (ms) (0 means infinity). When map update time exceeds this fixed time threshold, some nodes in "
            "Working Memory "
            "(WM) are transferred to Long-Term Memory to limit the size of the WM and decrease the update time.")};
}

auto bindings(const NavimapParams&) {
  using C = NavimapParams;
  return std::tuple{
      field("node.navimap.cell_size_m", &C::cell_size_m, "Resolution of the occupancy grid."),
      field("node.navimap.depth_decimation", &C::depth_decimation,
            "[node.navimap.depth_decimation=true] Decimation of the depth image before creating cloud."),
      field("node.navimap.depth_roi_ratios", &C::depth_roi_ratios,
            "Region of interest ratios [left, right, top, bottom] for depth cloud generation."),
      field("node.navimap.footprint.height_m", &C::footprint_height_m,
            "Footprint height used to filter points over the footprint of the robot. Footprint length and width should be set."),
      field("node.navimap.footprint.length_m", &C::footprint_length_m, "Footprint length used to filter points over the footprint of the robot."),
      field("node.navimap.footprint.width_m", &C::footprint_width_m,
            "Footprint width used to filter points over the footprint of the robot. Footprint length should be set."),
      field("node.navimap.frame_grid_enabled", &C::frame_grid_enabled, "Create local occupancy grid maps. See \"Grid\" group for parameters."),
      field("node.navimap.ground_seg.max_height_correction_m", &C::ground_max_height_correction_m,
            "Maximum change from the last reliable floor height in meters; unreliable floor skips geometry fusion."),
      field("node.navimap.ground_seg.method", &C::ground_method,
            "Ground fitting: 0=image blocks, 1=CAPE planes. Both use gravity alignment and horizontal floor-height consensus; no KNN fallback."),
      field("node.navimap.ground_seg.plane_tolerance_m", &C::ground_plane_tolerance_m, "Horizontal floor consensus residual in meters."),
      field("node.navimap.max_depth_m", &C::max_depth_m,
            "Maximum optical-axis depth (m). 0=inf. With node.navimap.ray_tracing_enabled, valid depths at or beyond this limit contribute free rays "
            "only."),
      field("node.navimap.max_obstacle_height_m", &C::max_obstacle_height_m,
            "Navigation upper obstacle height above the observed floor (inclusive)."),
      field("node.navimap.method", &C::method, "Mapping method: 0=2D depth-cloud projection, 1=3D depth cloud with elevation."),
      field("node.navimap.min_depth_m", &C::min_depth_m, "Minimum range from sensor."),
      field("node.navimap.min_obstacle_height_m", &C::min_obstacle_height_m,
            "Navigation obstacle band starts at this height above the observed floor (inclusive); lower returns are traversable."),
      field("node.navimap.project_in_map_frame", &C::project_in_map_frame,
            "Projection in map frame. On a 3D terrain and a fixed local camera transform (the cloud is created relative to ground), you may want to "
            "disable "
            "this to do the projection in robot frame instead."),
      field("node.navimap.raster.cache_budget_bytes", &C::raster_cache_budget_bytes,
            "Retained immutable navigation source raster budget in bytes (0 disables retention)."),
      field("node.navimap.tsdf.truncation_cells", &C::tsdf_truncation_cells, "Horizontal signed-distance truncation in grid cells."),
      field("node.navimap.tsdf.surface_weight", &C::tsdf_surface_weight, "Minimum support for contour extraction."),
      field("node.navimap.tsdf.maximum_weight", &C::tsdf_maximum_weight, "Maximum accumulated distance confidence."),
      field("node.navimap.erode_obstacles", &C::raster_erode_obstacles, "Erode occupied and unknown output cells bordering free space."),
      field("node.navimap.raster.footprint_radius_m", &C::raster_footprint_radius_m,
            "Footprint radius (m) used to clear all obstacles under the graph."),
      field("node.navimap.raster.hit_probability", &C::raster_hit_probability, "Hit probability in [0.5,1); 0.5 contributes no evidence."),
      field("node.navimap.raster.max_probability", &C::raster_max_probability, "Probability clamping maximum (value between 0 and 1)."),
      field("node.navimap.raster.min_probability", &C::raster_min_probability, "Probability clamping minimum (value between 0 and 1)."),
      field("node.navimap.raster.miss_probability", &C::raster_miss_probability, "Miss probability in (0,0.5]; 0.5 contributes no evidence."),
      field("node.navimap.raster.occupied_probability", &C::raster_occupied_probability,
            "Occupancy threshold in [0,1); 0 uses the latest observation classification."),
      field("node.navimap.ray_tracing_enabled", &C::ray_tracing_enabled,
            "[node.navimap.method=0/1] Clip valid depth rays at node.navimap.max_depth_m and mark the limit free. Method 0 also traces in-range "
            "surface rays. Invalid "
            "depth is skipped."),
      field("node.navimap.voxel_filter_enabled", &C::voxel_filter_enabled,
            "Input cloud is downsampled by voxel filter (voxel size is \"node.navimap.cell_size_m\") before doing segmentation of obstacles and "
            "ground.")};
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
            "visual-inertial "
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

auto bindings(const PreprocessParams&) {
  using C = PreprocessParams;
  return std::tuple{
      field("preprocess.feature_image_decimation", &C::feature_image_decimation,
            "Decimation of the grayscale image before visual feature detection. Camera calibration and stored depth are scaled consistently."),
      field("preprocess.stereo_image_width", &C::stereo_image_width,
            "Fixed fresh stereo node canvas width; 0 keeps input size. Requires both decimations=1."),
      field("preprocess.stereo_image_height", &C::stereo_image_height,
            "Fixed fresh stereo node canvas height; 0 keeps input size. Images, masks, registered depth and camera geometry are resized together."),
      field("preprocess.storage_image_decimation", &C::storage_image_decimation,
            "Decimation of the grayscale image before saving it to database. If depth size is larger than the decimated image size, depth is "
            "decimated to be always at most equal to the image size. Decimation is done from the original image. If set to the same value as "
            "preprocess.feature_image_decimation, already decimated data is saved (no need to re-decimate the image).")};
}

auto bindings(const RegistrationParams&) {
  using C = RegistrationParams;
  return std::tuple{
      field("registration.lidar.odometry_translation_sigma_m", &C::lidar_odometry_translation_sigma_m,
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
      field("registration.lidar.rotation_sigma_rad", &C::lidar_rotation_sigma_rad, "Graph rotation uncertainty floor in radians."),
      field("registration.matching.distance_ratio", &C::matching_distance_ratio,
            "Ratio between the best and second eligible Hamming distances in one BoW group. Equality passes."),
      field("registration.matching.max_comparisons", &C::matching_max_comparisons,
            "Maximum descriptor comparisons per candidate. Reject before matching if the shared-group bound exceeds this value."),
      field("registration.matching.max_hamming_distance", &C::matching_max_hamming_distance,
            "Maximum ORB Hamming distance within a shared vocabulary node."),
      field("registration.solving.max_reprojection_error_px", &C::max_reprojection_error_px, "PoseLibPlus PnP reprojection error in pixels."),
      field("registration.solving.min_inlier_distribution", &C::min_inlier_distribution,
            "Minimum distribution value of the inliers in the image to accept the transformation. The distribution is the second eigen value of the "
            "PCA "
            "(Principal Component Analysis) on the keypoints of the normalized image [-0.5, 0.5]. The value would be between 0 and 0.5. 0 means "
            "disabled."),
      field("registration.solving.min_inliers", &C::min_inliers, "Minimum feature correspondences to compute/accept the transformation."),
      field("registration.solving.ransac_iterations", &C::ransac_iterations, "Maximum iterations to compute the transform."),
      field("registration.solving.refine_iterations", &C::refine_iterations, "PoseLibPlus local and bundle refinement iterations after PnP RANSAC."),
      field("registration.solving.stereo_refinement_enabled", &C::stereo_refinement_enabled,
            "Refine loop pose with original stereo pixel observations and marginalize landmark uncertainty.")};
}

auto bindings(const RelocalizerParams&) {
  using C = RelocalizerParams;
  return std::tuple{
      field("relocalizer.visit_history.enabled", &C::visits_enabled,
            "Use bounded visit sequences to order cold loads and prefetch; geometry gates and Bayes formulas are unchanged."),
      field("relocalizer.visit_history.max_entries", &C::visits_max_entries,
            "Maximum persisted place visits, in [4,100000]. Consecutive visits to the same place collapse."),
      field("relocalizer.visit_history.max_gap_s", &C::visits_max_gap_s, "Maximum sensor-time gap within a continuous visit sequence."),
      field("relocalizer.spatial.enabled", &C::spatial_enabled,
            "Enable bounded spatial fallback after appearance candidates fail; disabling skips spatial search and its geometric trials."),
      field("relocalizer.spatial.max_candidates", &C::spatial_max_candidates,
            "Positive maximum of additional geometric trials after appearance candidates fail. Already tried nodes do not consume slots."),
      field("relocalizer.spatial.radius_m", &C::spatial_radius_m,
            "Spatial fallback search and accepted relative-pose radius in meters; candidates require poses in the current graph."),
      field("relocalizer.spatial.max_view_angle_deg", &C::spatial_max_view_angle_deg,
            "Maximum viewing direction difference for spatial fallback, in degrees (0 disables the direction filter)."),
      field("relocalizer.spatial.max_graph_hops", &C::spatial_max_graph_hops,
            "Maximum graph edge distance from the query to a spatial candidate (0 disables this filter); not a cap on visited nodes."),
      field("relocalizer.identity_initial_guess", &C::identity_initial_guess,
            "Use the identity matrix as the initial guess when computing a visual loop-closure transform; otherwise no guess is used."),
      field("relocalizer.invert_registration", &C::invert_registration,
            "On loop closure, do registration from the target to reference instead of reference to target."),
      field("relocalizer.limit_covariance_by_odometry", &C::limit_covariance_by_odometry,
            "Limit covariance of non-neighbor links to minimum covariance of neighbor links. In other words, if covariance of a loop closure link is "
            "smaller "
            "than the minimum covariance of odometry links, its covariance is set to minimum covariance of odometry links."),
      field("relocalizer.local_radius_m", &C::local_radius_m,
            "Local radius (m) for nodes selection in the local map. This parameter is used in some approaches about the local map management."),
      field("relocalizer.max_local_retrieved", &C::max_local_retrieved,
            "Maximum local locations retrieved (0=disabled) near the current pose in the local map."),
      field("relocalizer.max_loop_distance_m", &C::max_loop_distance_m,
            "Reject loop closures if the distance from the map is over this distance (0=disabled)."),
      field("relocalizer.max_retrieved", &C::max_retrieved, "Maximum nodes retrieved at the same time from LTM."),
      field("relocalizer.min_known_probability", &C::min_known_probability, "Loop closing threshold; values above 1 disable acceptance."),
      field("relocalizer.min_known_probability_ratio", &C::min_known_probability_ratio,
            "The loop closure hypothesis must be over LoopRatio x lastHypothesisValue."),
      field("relocalizer.min_unlinked_known_probability", &C::min_unlinked_known_probability,
            "Loop closure threshold used (overriding relocalizer.min_known_probability) when a new mapping session is not yet linked to a map of the "
            "highest "
            "loop closure hypothesis. The goal is to aggressively loop on a previous map in the database. Only used when topograph.metric_enabled is "
            "enabled. "
            "Set 1 to disable."),
      field("relocalizer.virtual_place_likelihood_method", &C::virtual_place_likelihood_method,
            "Likelihood ratio for virtual place (for no loop closure hypothesis): 0=Mean / StdDev, 1=StdDev / (Max-Mean)")};
}

auto bindings(const StatisticsParams&) {
  using C = StatisticsParams;
  return std::tuple{
      field("statistics.buffer_logs", &C::buffer_logs, "Use bounded stream buffering; false flushes the statistics log after each observation."),
      field("statistics.collect_memory_usage", &C::collect_memory_usage,
            "Publishing RAM usage in statistics (may add a small overhead to get info from the system)."),
      field("statistics.log_enabled", &C::log_enabled, "Logging enabled."),
      field("statistics.log_headers", &C::log_headers, "Add column header description to log files."),
      field("statistics.publish_enabled", &C::publish_enabled, "Publishing statistics."),
      field("statistics.save_resident_nodes", &C::save_resident_nodes, "Save working memory state after each update in statistics.")};
}

auto bindings(const NodeParams&) {
  using C = NodeParams;
  return std::tuple{
      field("node.profile_enabled", &C::profile_enabled, "Log node maintenance stage timings."),
      field("node.generate_node_ids", &C::generate_node_ids, "True=Generate location IDs, False=use input image IDs."),
      field("node.ignore_bad_frames", &C::ignore_bad_frames, "Bad nodes are ignored."),
      field("node.min_rotation_rad", &C::min_rotation_rad, "Angular tolerance (rad) for legacy rehearsal merging; not a keyframe admission gate."),
      field("node.min_translation_m", &C::min_translation_m,
            "Translation tolerance (m) for legacy rehearsal merging; not a keyframe admission gate."),
      field("node.merge_max_comparisons", &C::merge_max_comparisons,
            "Maximum exact Hamming comparisons for a submap pair, counting both directions. Does not cap persisted landmark or observer counts."),
      field("node.merge_previous", &C::merge_previous,
            "After a submap update, check only the preceding accepted keyframe region in this session for consolidation."),
      field("node.refresh_enabled", &C::refresh_enabled,
            "Maintain canonical submap content with bounded boundary-conditioned local BA, preserving all graph keyframes and original constraints."),
      field("node.rehearsal_ignore_moving", &C::rehearsal_ignore_moving, "When the robot is moving, weights are not updated on rehearsal."),
      field("node.rehearsal_similarity", &C::rehearsal_similarity, "Rehearsal similarity."),
      field("node.rehearsal_use_new_id", &C::rehearsal_use_new_id, "On merge, update to new id. When false, no copy.")};
}

auto bindings(const TopographParams&) {
  using C = TopographParams;
  return std::tuple{
      field("topograph.metric_enabled", &C::metric_enabled,
            "Activate metric SLAM. If set to false, classic RTAB-Map loop closure detection is done using only images and without any metric "
            "information."),
      field("topograph.new_session_odometry_jump_m", &C::new_session_odometry_jump_m,
            "A new map is created if a change of odometry translation greater than X m is detected (0 m = disabled)."),
      field("topograph.start_session_on_good_frame", &C::start_session_on_good_frame,
            "Start a new map only if the first node is not bad (i.e., has enough features, visual retrieval evidence is present)."),
      field("topograph.use_previous_map", &C::use_previous_map,
            "Use stored maps as localization/loop candidates. False starts a disconnected map in the same database."),

      field("topograph.automatic_map_labels", &C::automatic_map_labels,
            "Create map labels. The first node of a map will be labeled as \"map#\" where # is the map ID."),
      field("topograph.belief.rebuild_prediction_each_update", &C::belief_rebuild_prediction_each_update,
            "Regenerate all the prediction matrix on each iteration (otherwise only removed/added ids are updated)."),
      field("topograph.belief.transition_weights", &C::belief_transition_weights,
            "Prediction of loop closures (Gaussian-like, here with sigma=1.6) - Format: {VirtualPlaceProb, LoopClosureProb, NeighborLvl1, "
            "NeighborLvl2, ...}. "
            "Considering we are at a previously visited location, the first value is the probability to move to a new place (unvisited location), "
            "the second "
            "value is the probability to stay at the same location, the third value is the probability to move to a neighbor or loop closure at the "
            "first "
            "depth level, the fourth value is the probability to move to a neighbor or loop closure at the second depth level, etc. If the sum of "
            "the values "
            "is not 1, the difference is normalized against all remaining visited locations. Normally, the sum of these values should be 1."),
      field("topograph.belief.virtual_place_prior_threshold", &C::belief_virtual_place_prior_threshold,
            "Virtual place prior. Considering that we are at a new place, this is the prior probability to move again to a new place (unvisited "
            "location). The "
            "prior probability to move to a previously visited location is 1 - VirtualPlacePriorThr (split equally against all previously visited "
            "locations)."),
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
template <class F>
void visit_fields(NodeParams& params, F&& visitor) {
  std::apply([&](const auto&... item) { (visitor(item.key, params.*(item.member), item.description), ...); }, bindings(params));
  visit_fields(params.local_ba, visitor);
}
template <class F>
void visit_fields(const NodeParams& params, F&& visitor) {
  std::apply([&](const auto&... item) { (visitor(item.key, params.*(item.member), item.description), ...); }, bindings(params));
  visit_fields(params.local_ba, visitor);
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
  visitor(CompressionParams{});
  visitor(DatabaseParams{});
  visitor(FeatureParams{});
  visitor(MemoryParams{});
  visitor(NavimapParams{});
  visitor(PGOParams{});
  visitor(PreprocessParams{});
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

LocalBAParams LocalBAParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap LocalBAParams::parameters() const { return dump(*this); }
void LocalBAParams::validate() const {
  finite_fields(*this);
  if (max_optimized_points < 6 || max_optimized_points > 8192 || max_iterations <= 0) throw std::invalid_argument("Invalid local BA budgets");
}

CompressionParams CompressionParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap CompressionParams::parameters() const { return dump(*this); }
void CompressionParams::validate() const {
  finite_fields(*this);
  if (!std::isfinite(detect_min_radius_m) || !std::isfinite(detect_max_radius_m) || detect_min_radius_m < 0 ||
      detect_max_radius_m <= detect_min_radius_m || !std::isfinite(detect_angle_deg) || detect_angle_deg < 0 || detect_angle_deg > 180 ||
      detect_iterations < 1 || detect_session_id < -2 || !std::isfinite(merge_radius_m) || merge_radius_m <= 0)
    throw std::invalid_argument("Invalid compression radii, angle, iteration count or session ID");
  if (detect_scope != "both" && detect_scope != "intra" && detect_scope != "inter")
    throw std::invalid_argument("compression.detect.scope must be both, intra or inter");
}

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

FeatureParams FeatureParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap FeatureParams::parameters() const { return dump(*this); }
void FeatureParams::validate() const {
  finite_fields(*this);
  if (method != 0 && method != 1) throw std::invalid_argument("registration.feature.method must be 0 or 1");
  if (uses_xfeat() && (max_features < 1 || max_features > 512)) throw std::invalid_argument("Feature method 1 requires 1..512 points");
  if (fast_threshold < 0 || fast_threshold > 255 || fast_secondary_threshold < 0 || fast_secondary_threshold > 255 || orb_harris_k <= 0)
    throw std::invalid_argument("FAST thresholds must be in [0,255] and Harris k must be positive");
  if (orb_pyramid_levels < 1 || orb_pyramid_levels > 64 || !std::isfinite(orb_pyramid_scale) || orb_pyramid_scale <= 0 ||
      (orb_pyramid_levels > 1 && orb_pyramid_scale <= 1))
    throw std::invalid_argument("Invalid feature pyramid configuration");
}

MemoryParams MemoryParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap MemoryParams::parameters() const { return dump(*this); }
void MemoryParams::validate() const {
  finite_fields(*this);
  if (update_budget_ms < 0) throw std::invalid_argument("Memory update budget must be nonnegative");
  if (resident_budget_nodes < 0) throw std::invalid_argument("memory.resident_budget_nodes must be nonnegative");
  if (feature_cache_nodes < 12 || feature_cache_nodes > 1024) throw std::invalid_argument("memory.feature_cache_nodes must be in [12,1024]");
  if (!(protected_recent_ratio >= 0 && protected_recent_ratio <= 1) || !(protected_local_ratio >= 0 && protected_local_ratio <= 1))
    throw std::invalid_argument("Memory protection ratios must be in [0,1]");
}

NavimapParams NavimapParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap NavimapParams::parameters() const { return dump(*this); }
std::array<float, 4> NavimapParams::roiRatios() const {
  std::array<float, 4> roi{};
  std::istringstream input(depth_roi_ratios);
  input.imbue(std::locale::classic());
  std::string token;
  for (auto& value : roi) {
    if (!(input >> token)) throw std::invalid_argument("node.navimap.depth_roi_ratios requires four numbers");
    Parameters::parse({{"roi", token}}, "roi", value);
    if (value < 0 || value >= 1) throw std::invalid_argument("node.navimap.depth_roi_ratios must be in [0,1)");
  }
  if (input >> token) throw std::invalid_argument("node.navimap.depth_roi_ratios requires exactly four numbers");
  if (roi[0] + roi[1] >= 1 || roi[2] + roi[3] >= 1) throw std::invalid_argument("node.navimap.depth_roi_ratios must leave a nonempty image");
  return roi;
}
void NavimapParams::validate() const {
  finite_fields(*this);
  if (!(cell_size_m > 0) || !depth_decimation || !(ground_plane_tolerance_m > 0) || !(ground_max_height_correction_m > 0))
    throw std::invalid_argument("Navigation cell size, depth stride and ground tolerances must be positive");
  if (min_depth_m < 0 || max_depth_m < 0 || (method != 0 && method != 1) || (ground_method != 0 && ground_method != 1))
    throw std::invalid_argument("Invalid navigation depth bounds or method");
  if (min_obstacle_height_m <= 0 || max_obstacle_height_m <= min_obstacle_height_m)
    throw std::invalid_argument("navimap heights require 0 < min_obstacle_height_m < max_obstacle_height_m");
  if (max_depth_m > 0 && max_depth_m <= min_depth_m)
    throw std::invalid_argument("node.navimap.max_depth_m must exceed min_depth_m, or be 0 (unlimited)");
  if (footprint_height_m < 0 || footprint_length_m < 0 || footprint_width_m < 0 || raster_footprint_radius_m < 0)
    throw std::invalid_argument("Navigation footprint and raster distances must be nonnegative");
  if (!(raster_hit_probability >= .5f && raster_hit_probability < 1) || !(raster_miss_probability > 0 && raster_miss_probability <= .5f) ||
      !(raster_min_probability > 0 && raster_min_probability < raster_max_probability && raster_max_probability < 1) ||
      !(raster_occupied_probability >= 0 && raster_occupied_probability < 1))
    throw std::invalid_argument("Invalid navigation hit, miss, clamping or occupancy probabilities");
  if (tsdf_truncation_cells < 1 || tsdf_truncation_cells > 64 || tsdf_surface_weight <= 0 || tsdf_maximum_weight < tsdf_surface_weight)
    throw std::invalid_argument("Invalid navimap TSDF truncation or confidence bounds");
  roiRatios();
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

PreprocessParams PreprocessParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap PreprocessParams::parameters() const { return dump(*this); }
void PreprocessParams::validate() const {
  finite_fields(*this);
  if (!feature_image_decimation || !storage_image_decimation) throw std::invalid_argument("Image decimation must be positive");
  if (!((stereo_image_width == 0 && stereo_image_height == 0) ||
        (stereo_image_width >= 64 && stereo_image_width <= 4096 && stereo_image_height >= 64 && stereo_image_height <= 4096)))
    throw std::invalid_argument("Stereo target size must be both zero or dimensions in [64,4096]");
  if (stereo_image_width && (feature_image_decimation != 1 || storage_image_decimation != 1))
    throw std::invalid_argument("Fixed stereo canvas requires feature/storage decimation=1");
}

RegistrationParams RegistrationParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap RegistrationParams::parameters() const { return dump(*this); }
void RegistrationParams::validate() const {
  finite_fields(*this);
  if (lidar_odometry_translation_sigma_m <= 0 || lidar_odometry_rotation_sigma_rad <= 0 || lidar_min_inliers < 6 || lidar_max_iterations < 1 ||
      lidar_max_iterations > 1000 || lidar_max_correspondence_m <= 0 || lidar_min_overlap <= 0 || lidar_min_overlap > 1 ||
      lidar_max_fitness_m2 <= 0 || lidar_translation_sigma_m <= 0 || lidar_rotation_sigma_rad <= 0)
    throw std::invalid_argument("Invalid LiDAR registration parameters");
  if (min_inliers < 6 || ransac_iterations <= 0 || ransac_iterations > 1000 || refine_iterations < 0 || refine_iterations > 20 ||
      !(max_reprojection_error_px > 0) || !std::isfinite(max_reprojection_error_px) ||
      !(matching_distance_ratio > 0 && matching_distance_ratio <= 1) || matching_max_hamming_distance < 0 || matching_max_hamming_distance > 256 ||
      matching_max_comparisons <= 0 || !std::isfinite(min_inlier_distribution) || min_inlier_distribution < 0)
    throw std::invalid_argument("Invalid bounded BoW/PnP registration parameters");
}

RelocalizerParams RelocalizerParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap RelocalizerParams::parameters() const { return dump(*this); }
void RelocalizerParams::validate() const {
  finite_fields(*this);
  if (spatial_max_candidates == 0)
    throw std::invalid_argument("relocalizer.spatial.max_candidates must be positive; use enabled=false to disable spatial fallback");
  if (spatial_radius_m <= 0 || spatial_max_view_angle_deg < 0 || spatial_max_view_angle_deg > 180)
    throw std::invalid_argument("Invalid bounded spatial fallback radius or view angle");
  if (local_radius_m < 0 || max_loop_distance_m < 0 || min_known_probability < 0 || min_unlinked_known_probability < 0 ||
      min_known_probability_ratio < 0 || (virtual_place_likelihood_method != 0 && virtual_place_likelihood_method != 1))
    throw std::invalid_argument("Invalid relocalizer distance, probability or search policy");
  if (visits_max_entries < 4 || visits_max_entries > 100000 || visits_max_gap_s <= 0)
    throw std::invalid_argument("Invalid relocalizer visit-history bounds");
}

StatisticsParams StatisticsParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap StatisticsParams::parameters() const { return dump(*this); }
void StatisticsParams::validate() const {}

NodeParams NodeParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap NodeParams::parameters() const { return dump(*this); }
void NodeParams::validate() const {
  finite_fields(*this);
  if (min_translation_m < 0 || min_rotation_rad < 0) throw std::invalid_argument("Node rehearsal motion tolerances must be nonnegative");
  if (!(rehearsal_similarity >= 0 && rehearsal_similarity <= 1)) throw std::invalid_argument("node.rehearsal_similarity must be in [0,1]");
  local_ba.validate();
}

TopographParams TopographParams::withOverrides(const ParametersMap& input) const { return load(*this, input); }
ParametersMap TopographParams::parameters() const { return dump(*this); }
std::vector<double> TopographParams::transitionWeights() const {
  std::istringstream input(belief_transition_weights);
  std::vector<double> result;
  std::string token;
  double total = 0;
  while (input >> token) {
    double value = 0;
    Parameters::parse({{"topograph.belief.transition_weights", token}}, "topograph.belief.transition_weights", value);
    if (value < 0 || value > 1) throw std::invalid_argument("Topograph transition weights must be in [0,1]");
    result.push_back(value);
    total += value;
  }
  if (result.size() < 2 || !(total > 0)) throw std::invalid_argument("Topograph transition model needs at least two weights and positive mass");
  return result;
}
void TopographParams::validate() const {
  finite_fields(*this);
  if (new_session_odometry_jump_m < 0 || belief_virtual_place_prior_threshold < 0 || belief_virtual_place_prior_threshold > 1)
    throw std::invalid_argument("Invalid topograph session threshold or virtual-place prior");
  transitionWeights();
}

void FeatureParams::validate(const PreprocessParams& preprocess) const {
  validate();
  preprocess.validate();
  if (uses_xfeat() && (preprocess.stereo_image_width != 320 || preprocess.stereo_image_height != 240))
    throw std::invalid_argument("XFeat/SigLIP2 require preprocess.stereo_image_width=320 and stereo_image_height=240");
  if (uses_xfeat() && (preprocess.feature_image_decimation != 1 || preprocess.storage_image_decimation != 1))
    throw std::invalid_argument("XFeat requires feature_image_decimation=1 and storage_image_decimation=1");
}

void LoadParameters::validate_modules(const ParametersMap& input) {
  Parameters::validate(input);
  visit_defaults([&](const auto& defaults) { read_fields(defaults, input); });
  const auto feature = FeatureParams{}.withOverrides(input);
  feature.validate(PreprocessParams{}.withOverrides(input));
}
}  // namespace sapphire::backend
