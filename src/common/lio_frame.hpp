#pragma once

#include "common.hpp"

#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

struct MargiFrame
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StateGroup x;
  PointCloudPtr pvec;
  Eigen::Matrix<double, 6, 1> v6;
  double timestamp = -1.0;

  MargiFrame(const StateGroup &state, PointCloudPtr points, double stamp, const Eigen::Matrix<double, 6, 1> &variance)
    : x(state), pvec(std::move(points)), v6(variance), timestamp(stamp) {}
};

struct LioFrame
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
  std::shared_ptr<const vvec<float, 3>> pcd;
  double timestamp = -1.0;
};

class KeyframeBuffer
{
public:
  static constexpr size_t FUSE_MARGI_FRAME_NUM = 10;

  explicit KeyframeBuffer(double inv_ds_voxel_size) : inv_ds_voxel_size_(inv_ds_voxel_size)
  {
    margi_frames_.reserve(FUSE_MARGI_FRAME_NUM);
  }

  void clear()
  {
    clear_frames();
    has_keyframe_ = false;
  }

  bool push(MargiFrame &&margi_frame, LioFrame &keyframe)
  {
    const size_t tail = (frame_begin_ + frame_count_) % FUSE_MARGI_FRAME_NUM;
    if(margi_frames_.size() < FUSE_MARGI_FRAME_NUM)
    {
      margi_frames_.emplace_back(std::move(margi_frame));
    }
    else
    {
      margi_frames_[tail] = std::move(margi_frame);
    }
    ++frame_count_;

    if(frame_count_ < FUSE_MARGI_FRAME_NUM) return false;

    const MargiFrame &latest = frame(FUSE_MARGI_FRAME_NUM - 1);
    if(has_keyframe_)
    {
      const double angle = Eigen::Quaterniond(last_keyframe_rotation_).angularDistance(Eigen::Quaterniond(latest.x.R));
      const double distance = (latest.x.p - last_keyframe_position_).norm();
      if(angle < MIN_KEYFRAME_ROTATION && distance < MIN_KEYFRAME_TRANSLATION)
      {
        margi_frames_[frame_begin_].pvec.reset();
        frame_begin_ = (frame_begin_ + 1) % FUSE_MARGI_FRAME_NUM;
        --frame_count_;
        return false;
      }
    }

    fuse(latest, keyframe);
    last_keyframe_rotation_ = latest.x.R;
    last_keyframe_position_ = latest.x.p;
    has_keyframe_ = true;
    clear_frames();
    return true;
  }

private:
  static constexpr double MIN_KEYFRAME_ROTATION = 5.0 * M_PI / 180.0;
  static constexpr double MIN_KEYFRAME_TRANSLATION = 0.1;

  MargiFrame &frame(size_t index)
  {
    return margi_frames_[(frame_begin_ + index) % FUSE_MARGI_FRAME_NUM];
  }

  const MargiFrame &frame(size_t index) const
  {
    return margi_frames_[(frame_begin_ + index) % FUSE_MARGI_FRAME_NUM];
  }

  static int64_t voxelIndex(double coordinate)
  {
    const int64_t index = static_cast<int64_t>(coordinate);
    return index - static_cast<int64_t>(coordinate < static_cast<double>(index));
  }

  void fuse(const MargiFrame &latest, LioFrame &keyframe)
  {
    size_t point_count = 0;
    for(size_t i = 0; i < frame_count_; ++i)
    {
      point_count += frame(i).pvec->size();
    }

    voxel_sums_.clear();
    voxel_sums_.reserve(point_count);

    const Eigen::Matrix3d R_base_odom = latest.x.R.transpose();
    for(size_t i = 0; i < frame_count_; ++i)
    {
      const MargiFrame &source = frame(i);
      const Eigen::Matrix3d R_base_source = R_base_odom * source.x.R;
      const Eigen::Vector3d p_base_source = R_base_odom * (source.x.p - latest.x.p);
      for(const pointVar &point : *source.pvec)
      {
        const Eigen::Vector3d transformed = R_base_source * point.pnt + p_base_source;
        const VOXEL_LOC voxel(
          voxelIndex(transformed.x() * inv_ds_voxel_size_),
          voxelIndex(transformed.y() * inv_ds_voxel_size_),
          voxelIndex(transformed.z() * inv_ds_voxel_size_));
        auto [iter, inserted] = voxel_sums_.try_emplace(voxel, transformed.x(), transformed.y(), transformed.z(), 1.0);
        if(!inserted)
        {
          iter->second.head<3>() += transformed;
          iter->second.w() += 1.0;
        }
      }
    }

    auto cloud = std::make_shared<vvec<float, 3>>();
    cloud->reserve(voxel_sums_.size());
    for(const auto &entry : voxel_sums_)
    {
      const Eigen::Vector4d &sum = entry.second;
      const double count_inv = 1.0 / sum.w();
      cloud->emplace_back(static_cast<float>(sum.x() * count_inv), static_cast<float>(sum.y() * count_inv), static_cast<float>(sum.z() * count_inv));
    }

    keyframe.T_odom_base.setIdentity();
    keyframe.T_odom_base.linear() = latest.x.R;
    keyframe.T_odom_base.translation() = latest.x.p;
    keyframe.pcd = std::move(cloud);
    keyframe.timestamp = latest.timestamp;
  }

  void clear_frames()
  {
    margi_frames_.clear();
    frame_begin_ = 0;
    frame_count_ = 0;
  }

  double inv_ds_voxel_size_;
  std::vector<MargiFrame, Eigen::aligned_allocator<MargiFrame>> margi_frames_;
  size_t frame_begin_ = 0;
  size_t frame_count_ = 0;
  bool has_keyframe_ = false;
  Eigen::Matrix3d last_keyframe_rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d last_keyframe_position_ = Eigen::Vector3d::Zero();
  unordered_map<VOXEL_LOC, Eigen::Vector4d> voxel_sums_;
};
