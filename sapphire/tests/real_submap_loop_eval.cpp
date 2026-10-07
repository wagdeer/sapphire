#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "backend/registration/loop_closure.hpp"
#include "parameters.h"
#include "backend/storage/retrieval_index.hpp"

namespace {

using Clock = std::chrono::steady_clock;

struct EvalFrame {
  std::uint64_t id = 0;
  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  std::shared_ptr<const sapphire::vvec<float, 3>> cloud;
  sapphire::GaussianCloudPtr gaussian_cloud;
  sapphire::AABB bounds;
  cpu::VoxelMapsData pyramid;
};

struct EvalCandidate {
  std::size_t target_id = 0;
  Eigen::Isometry3d initial_target_query = Eigen::Isometry3d::Identity();
};

double elapsedMilliseconds(Clock::time_point start) { return std::chrono::duration<double, std::milli>(Clock::now() - start).count(); }

std::shared_ptr<const sapphire::vvec<float, 3>> loadAsciiPly(const std::filesystem::path &path) {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("Cannot open PLY: " + path.string());
  }

  std::size_t vertex_count = 0;
  std::string line;
  bool found_header_end = false;
  while (std::getline(stream, line)) {
    if (line.rfind("element vertex ", 0) == 0) {
      std::istringstream parser(line.substr(15));
      parser >> vertex_count;
    } else if (line == "end_header") {
      found_header_end = true;
      break;
    }
  }
  if (!found_header_end || vertex_count == 0) {
    throw std::runtime_error("Invalid or empty ASCII PLY: " + path.string());
  }

  auto cloud = std::make_shared<sapphire::vvec<float, 3>>();
  cloud->reserve(vertex_count);
  for (std::size_t index = 0; index < vertex_count; ++index) {
    Eigen::Vector3f point;
    if (!(stream >> point.x() >> point.y() >> point.z())) {
      throw std::runtime_error("Truncated PLY: " + path.string());
    }
    if (point.allFinite()) {
      cloud->push_back(point);
    }
  }
  return cloud;
}

/// Offline adapter for legacy XYZ evaluation data. It assigns a fixed regularized
/// covariance and intentionally performs no neighborhood covariance estimation.
sapphire::GaussianCloudPtr makeGaussianCloud(const sapphire::vvec<float, 3> &points) {
  auto cloud = std::make_shared<sapphire::GaussianCloud>();
  cloud->reserve(points.size());
  for (const Eigen::Vector3f &point : points) {
    sapphire::GaussianPoint gaussian;
    gaussian.mean = point;
    gaussian.covariance = Eigen::Matrix3f::Identity() / 9.0F;
    gaussian.N = 1;
    gaussian.regularize();
    cloud->push_back(gaussian);
  }
  return cloud;
}

std::vector<EvalFrame> loadFrames(const std::filesystem::path &directory) {
  std::ifstream pose_stream(directory / "poses.txt");
  if (!pose_stream) {
    throw std::runtime_error("Cannot open poses.txt");
  }

  std::vector<EvalFrame> frames;
  while (pose_stream) {
    EvalFrame frame;
    Eigen::Vector3f translation;
    Eigen::Quaternionf orientation;
    float qx = 0.0F;
    float qy = 0.0F;
    float qz = 0.0F;
    float qw = 1.0F;
    double timestamp = 0.0;
    if (!(pose_stream >> frame.id >> timestamp >> translation.x() >> translation.y() >> translation.z() >> qx >> qy >> qz >> qw)) {
      break;
    }
    if (frame.id != frames.size()) {
      throw std::runtime_error("Submap IDs must be contiguous");
    }
    orientation = Eigen::Quaternionf(qw, qx, qy, qz);
    orientation.normalize();
    frame.pose.linear() = orientation.toRotationMatrix();
    frame.pose.translation() = translation;
    frame.cloud = loadAsciiPly(directory / (std::to_string(frame.id) + ".ply"));
    frame.gaussian_cloud = makeGaussianCloud(*frame.cloud);
    frame.bounds = sapphire::AABB(*frame.cloud);

    cpu::VoxelMaps voxelmaps;
    voxelmaps.create_voxelmaps(frame.cloud->data(), frame.cloud->size());
    frame.pyramid = voxelmaps.release_data();
    frames.push_back(std::move(frame));
  }
  return frames;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc != 3) {
      std::cerr << "usage: sapphire_real_submap_loop_eval <config.toml> <submap_directory>\n";
      return 2;
    }
    sapphire::load_parameters(argv[1]);
    const std::vector<EvalFrame> frames = loadFrames(argv[2]);
    sapphire::SubmapSpatialIndex spatial_index;
    for (const EvalFrame &frame : frames) {
      spatial_index.upsert(frame.id, frame.bounds, frame.pose);
    }

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "loaded_submaps=" << frames.size() << '\n';
    std::size_t total_spatial = 0;
    std::size_t total_eligible = 0;
    std::size_t total_bbs_accepted = 0;
    std::size_t total_gicp_attempted = 0;
    std::size_t total_gicp_accepted = 0;
    for (std::size_t query_id = 0; query_id < frames.size(); ++query_id) {
      const EvalFrame &query = frames[query_id];
      const std::vector<sapphire::SpatialMatch> spatial_matches = spatial_index.query(query_id, query.pose);
      total_spatial += spatial_matches.size();
      sapphire::LoopCandidateStats stats;
      std::vector<EvalCandidate> candidates;
      for (const sapphire::SpatialMatch &spatial : spatial_matches) {
        if (!sapphire::isEligibleLoopTarget(spatial.submap_id, query_id, stats)) {
          continue;
        }
        ++total_eligible;
        const EvalFrame &target = frames[static_cast<std::size_t>(spatial.submap_id)];
        Eigen::Isometry3d initial = Eigen::Isometry3d::Identity();
        initial.matrix() = target.pose.inverse().matrix().cast<double>() * query.pose.matrix().cast<double>();
        candidates.push_back({static_cast<std::size_t>(spatial.submap_id), initial});
      }

      std::cout << "query=" << query_id << " spatial=" << spatial_matches.size() << " eligible=" << (stats.spatial_matches - stats.rejected())
                << " candidates=" << candidates.size() << '\n';
      const EvalCandidate *top_candidate = nullptr;
      std::vector<gpu::LocalSearchTarget> bbs_targets;
      bbs_targets.reserve(candidates.size());
      for (const EvalCandidate &candidate : candidates) {
        const EvalFrame &target = frames[candidate.target_id];
        bbs_targets.push_back(
            {static_cast<std::uint64_t>(candidate.target_id), &target.pyramid, candidate.initial_target_query.cast<float>()});
      }
      const sapphire::BbsResult bbs_result = sapphire::alignLoopBbs(*query.cloud, bbs_targets);
      total_bbs_accepted += bbs_result.accepted;
      if (bbs_result.accepted) {
        const auto found = std::find_if(candidates.begin(), candidates.end(), [&](const EvalCandidate &candidate) {
          return candidate.target_id == bbs_result.target_id;
        });
        if (found != candidates.end()) {
          top_candidate = &*found;
        }
      }
      std::cout << "  roots=" << bbs_result.root_nodes << " initial_overlap=" << bbs_result.initial_overlap
                << " tolerant_overlap=" << bbs_result.overlap << " bbs_ms=" << bbs_result.elapsed_ms << " bbs_accept=" << bbs_result.accepted
                << " expanded=" << bbs_result.expanded_nodes[3] << '/' << bbs_result.expanded_nodes[2] << '/'
                << bbs_result.expanded_nodes[1] << '/' << bbs_result.expanded_nodes[0] << " pruned=" << bbs_result.pruned_nodes[3] << '/'
                << bbs_result.pruned_nodes[2] << '/' << bbs_result.pruned_nodes[1] << '/' << bbs_result.pruned_nodes[0] << '\n';

      if (top_candidate) {
        const EvalFrame &target = frames[top_candidate->target_id];
        const auto gicp_start = Clock::now();
        ++total_gicp_attempted;
        const sapphire::GicpResult refined =
            sapphire::refineLoopGicp(query.gaussian_cloud, target.gaussian_cloud, bbs_result.T_target_query);
        total_gicp_accepted += refined.accepted;
        std::cout << "  top1=" << top_candidate->target_id << " tolerant_overlap=" << bbs_result.overlap << " gicp_count=1"
                  << " gicp_fitness=" << refined.fitness << " gicp_inliers=" << refined.inliers << " gicp_accept=" << refined.accepted
                  << " gicp_ms=" << elapsedMilliseconds(gicp_start) << '\n';
      } else {
        std::cout << "  top1=-1 tolerant_overlap=0.000 gicp_count=0\n";
      }
    }
    std::cout << "summary spatial=" << total_spatial << " eligible=" << total_eligible << " bbs_accept=" << total_bbs_accepted
              << " gicp_attempted=" << total_gicp_attempted
              << " gicp_accept=" << total_gicp_accepted << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "evaluation failed: " << error.what() << '\n';
    return 1;
  }
}
