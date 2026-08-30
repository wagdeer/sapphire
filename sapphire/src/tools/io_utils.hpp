#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace sapphire {

inline double monotonic_time_seconds() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

inline std::string current_time_filename() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time;
  localtime_r(&now_time, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y-%m-%d_%H-%M-%S");
  return stream.str();
}

inline void prepare_output_directory(const std::string &savepath, const std::string &filename) {
  const std::filesystem::path output_path = std::filesystem::path(savepath) / filename;
  if (std::filesystem::exists(output_path)) {
    spdlog::warn("Output directory already exists, clearing: {}", output_path.string());
    std::filesystem::remove_all(output_path);
  }
  std::filesystem::create_directories(output_path);
}

class FileReaderWriter {
 public:
  static FileReaderWriter &instance() {
    static FileReaderWriter instance;
    return instance;
  }

  void open_session(const std::string &savepath, const std::string &session_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    close_session_unlocked();
    const std::filesystem::path session_dir = std::filesystem::path(savepath) / session_name;
    submap_dir_ = (session_dir / "submaps").string();
    std::filesystem::create_directories(submap_dir_);
    odom_poses_file_.open(session_dir / "odom_poses.txt");
    odom_pose_tum_file_.open(session_dir / "odom_pose.tum");
    submap_pose_file_.open(std::filesystem::path(submap_dir_) / "poses.txt");
  }

  void save_pose(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &position, const Eigen::Vector3d &velocity,
                 const Eigen::Vector3d &gyro_bias, const Eigen::Vector3d &accel_bias, const Eigen::Vector3d &gravity,
                 const Eigen::Matrix<double, 6, 1> &variance, double timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    const Eigen::Quaterniond q(rotation);
    odom_poses_file_ << std::fixed << std::setprecision(6) << timestamp << " " << std::setprecision(7) << position.x() << " " << position.y() << " "
                     << position.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << velocity.x() << " " << velocity.y()
                     << " " << velocity.z() << " " << gyro_bias.x() << " " << gyro_bias.y() << " " << gyro_bias.z() << " " << accel_bias.x() << " "
                     << accel_bias.y() << " " << accel_bias.z() << " " << gravity.x() << " " << gravity.y() << " " << gravity.z();
    for (int i = 0; i < 6; ++i) {
      odom_poses_file_ << " " << variance[i];
    }
    odom_poses_file_ << '\n';

    odom_pose_tum_file_ << std::fixed << std::setprecision(9) << timestamp << " " << position.x() << " " << position.y() << " " << position.z() << " "
                        << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << '\n';
  }

  void save_submap(const std::shared_ptr<const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>> &frame_pcd,
                   const Eigen::Isometry3d &T_odom_base, double timestamp, int id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::filesystem::path path = std::filesystem::path(submap_dir_) / (std::to_string(id) + ".ply");
    std::ofstream cloud_file(path);
    cloud_file << "ply\nformat ascii 1.0\n"
               << "element vertex " << frame_pcd->size() << "\n"
               << "property float x\nproperty float y\nproperty float z\n"
               << "end_header\n";
    cloud_file << std::setprecision(9);
    for (const Eigen::Vector3f &point : *frame_pcd) {
      cloud_file << point.x() << " " << point.y() << " " << point.z() << '\n';
    }

    const Eigen::Quaterniond q(T_odom_base.rotation());
    const Eigen::Vector3d &position = T_odom_base.translation();
    submap_pose_file_ << std::fixed << std::setprecision(6) << id << " " << timestamp << " " << std::setprecision(7) << position.x() << " "
                      << position.y() << " " << position.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << '\n';
  }

  void close_session() {
    std::lock_guard<std::mutex> lock(mutex_);
    close_session_unlocked();
  }

 private:
  void close_session_unlocked() {
    if (odom_poses_file_.is_open()) {
      odom_poses_file_.close();
    }
    if (odom_pose_tum_file_.is_open()) {
      odom_pose_tum_file_.close();
    }
    if (submap_pose_file_.is_open()) {
      submap_pose_file_.close();
    }
    submap_dir_.clear();
  }

  std::mutex mutex_;
  std::ofstream odom_poses_file_;
  std::ofstream odom_pose_tum_file_;
  std::ofstream submap_pose_file_;
  std::string submap_dir_;
};

}  // namespace sapphire
