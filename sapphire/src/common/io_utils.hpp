#pragma once

#include <spdlog/spdlog.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

#include "lio_frame.hpp"

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
    keyframe_dir_ = (session_dir / "keyframes").string();
    std::filesystem::create_directories(keyframe_dir_);
    odom_poses_file_.open(session_dir / "odom_poses.txt");
    odom_pose_tum_file_.open(session_dir / "odom_pose.tum");
    keyframe_pose_file_.open(std::filesystem::path(keyframe_dir_) / "poses.txt");
  }

  void save_pose(const MargiFrame &frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    const StateGroup &state = frame.x;
    const Eigen::Quaterniond q = Eigen::Quaterniond(state.R).normalized();
    odom_poses_file_ << std::fixed << std::setprecision(6) << state.t << " " << std::setprecision(7) << state.p.x() << " " << state.p.y() << " "
                     << state.p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << state.v.x() << " " << state.v.y() << " "
                     << state.v.z() << " " << state.bg.x() << " " << state.bg.y() << " " << state.bg.z() << " " << state.ba.x() << " " << state.ba.y()
                     << " " << state.ba.z() << " " << state.g.x() << " " << state.g.y() << " " << state.g.z();
    for (int i = 0; i < 6; ++i) {
      odom_poses_file_ << " " << frame.v6[i];
    }
    odom_poses_file_ << '\n';

    odom_pose_tum_file_ << std::fixed << std::setprecision(9) << frame.timestamp << " " << state.p.x() << " " << state.p.y() << " " << state.p.z()
                        << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << '\n';
  }

  void save_keyframe(const LioFrame &frame, int id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::filesystem::path path = std::filesystem::path(keyframe_dir_) / (std::to_string(id) + ".pcd");
    std::ofstream cloud_file(path);
    cloud_file << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
               << "TYPE F F F\nCOUNT 1 1 1\nWIDTH " << frame.pcd->size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << frame.pcd->size()
               << "\nDATA ascii\n";
    cloud_file << std::setprecision(9);
    for (const Eigen::Vector3f &point : *frame.pcd) {
      cloud_file << point.x() << " " << point.y() << " " << point.z() << '\n';
    }

    const Eigen::Quaterniond q(frame.T_odom_base.rotation());
    const Eigen::Vector3d &position = frame.T_odom_base.translation();
    keyframe_pose_file_ << std::fixed << std::setprecision(6) << id << " " << frame.timestamp << " " << std::setprecision(7) << position.x() << " "
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
    if (keyframe_pose_file_.is_open()) {
      keyframe_pose_file_.close();
    }
    keyframe_dir_.clear();
  }

  std::mutex mutex_;
  std::ofstream odom_poses_file_;
  std::ofstream odom_pose_tum_file_;
  std::ofstream keyframe_pose_file_;
  std::string keyframe_dir_;
};

}  // namespace sapphire
