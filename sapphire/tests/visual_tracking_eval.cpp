#include "backend/visual/feature/visual_features.hpp"
// Offline evaluation of the production tracker against recorded images and
// interpolated, read-only LIO poses. Input/export recipe lives in the audit packet.
#include "backend/visual/feature/visual_tracker.hpp"
#include "backend/visual/visual_loop.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core.hpp>
#include <chrono>

using namespace sapphire;
int main(int argc, char **argv) {
  try {
    if (argc != 5) throw std::invalid_argument("usage: eval config.toml frames.bin output.csv tracker|orb");
    cv::setNumThreads(1);
    auto config = load_parameters(argv[1]).pose_graph.visual;
    const bool baseline = std::string(argv[4]) == "orb";
    if (!baseline && std::string(argv[4]) != "tracker") throw std::invalid_argument("Unknown evaluation mode");
    VisualTrackingParameters params;
    VisualTracker tracker(config.left, 0, params);
    std::ifstream input(argv[2], std::ios::binary);
    std::ofstream output(argv[3]);
    if (!input || !output) throw std::runtime_error("Cannot open input/output");
    output << "stamp,tracks,mature,new,flow_rejected,coverage,all_coverage,retention,renewal,spatial_retention,displacement,triangulated,keyframe,descriptor_rows,metric_rows,submap_candidate,gap,reference_lost,pose,tracking_ms,descriptor_ms,total_ms,raw_candidate,anchors,anchor_points,projection_tests,projected,match_visits,match_budget_exhausted,associated,reacquired,affiliation_coverage,unexplained_fraction,reference_anchor,best_anchor_matches,affiliated,internal_keyframe,recovering,affiliation_ms\n";
    output << std::setprecision(17);
    std::array<char, 8> magic{}; input.read(magic.data(), magic.size());
    if (std::string(magic.data(), 8) != "SVTRACK1") throw std::runtime_error("Invalid input format");
    std::uint32_t dimensions[2]; input.read(reinterpret_cast<char *>(dimensions), sizeof(dimensions));
    if (int(dimensions[0]) != config.left.width || int(dimensions[1]) != config.left.height) throw std::runtime_error("Image dimensions mismatch");
    std::array<double, 18> record{};
    cv::Mat gray(config.left.height, config.left.width, CV_8UC1);
    double last_baseline_stamp = -1;
    bool has_reference = false;
    std::size_t count = 0, keyframes = 0, boundaries = 0;
    while (input.read(reinterpret_cast<char *>(record.data()), sizeof(record))) {
      input.read(reinterpret_cast<char *>(gray.data), std::streamsize(gray.total()));
      if (!input) throw std::runtime_error("Truncated image");
      std::optional<Eigen::Isometry3d> pose;
      if (record[1] == 1) {
        Eigen::Isometry3d value;
        for (int row = 0; row < 4; ++row) for (int col = 0; col < 4; ++col) value.matrix()(row, col) = record[2 + 4 * row + col];
        pose = value;
      }
      auto start = std::chrono::steady_clock::now();
      VisualTracker::Result result;
      if (baseline) {
        if (last_baseline_stamp < 0 || record[0] - last_baseline_stamp >= config.image_interval) {
          last_baseline_stamp = record[0];
          result.keyframe = extractLoopFeatures(ImageMeas(record[0], gray), config);
          result.stats.descriptor_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        }
      } else result = tracker.process(ImageMeas(record[0], gray), pose);
      const double total = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
      const auto &s = result.stats;
      std::size_t rows = 0, metric_rows = 0;
      if (result.keyframe) {
        ++keyframes; rows = result.keyframe->points().size();
        for (const auto &p : result.keyframe->points()) metric_rows += bool(p.geometry());
      }
      if (!s.anchors) has_reference = false;
      // This simulates only selector acknowledgement. No actual submap is built.
      if (!baseline && ((result.keyframe && !has_reference) || s.submap_candidate)) {
        tracker.acceptSubmapReference(); has_reference = true; ++boundaries;
      }
      output << record[0] << ',' << s.tracks << ',' << s.mature_tracks << ',' << s.new_tracks << ',' << s.flow_rejected << ','
             << s.coverage << ',' << s.all_point_coverage << ',' << s.reference_retention << ',' << s.renewal << ',' << s.spatial_retention << ','
             << s.reference_displacement_px << ',' << s.triangulated << ',' << bool(result.keyframe) << ',' << rows << ',' << metric_rows << ','
             << s.submap_candidate << ',' << s.reset_by_gap << ',' << s.reference_lost << ',' << bool(pose) << ','
             << s.tracking_ms << ',' << s.descriptor_ms << ',' << total << ',' << s.raw_submap_candidate << ',' << s.anchors << ','
             << s.anchor_points << ',' << s.projection_tests << ',' << s.projected << ',' << s.match_visits << ',' << s.match_budget_exhausted << ','
             << s.associated << ',' << s.reacquired << ',' << s.affiliation_coverage << ',' << s.unexplained_fraction << ',' << s.reference_anchor << ','
             << s.best_anchor_matches << ',' << s.affiliated << ',' << s.internal_keyframe << ',' << s.recovering << ',' << s.affiliation_ms << '\n';
      ++count;
    }
    if (!input.eof() || input.gcount()) throw std::runtime_error("Truncated record header");
    std::cout << "images=" << count << " keyframes=" << keyframes << " simulated_reference_accepts=" << boundaries << '\n';
    return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
