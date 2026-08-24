#pragma once
#include "lio_database.hpp"

#include <algorithm>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class Memory
{
public:
  Memory(const std::string &databasePath, size_t stmCapacity, size_t wmCapacity)
    : database_(databasePath), stmCapacity_(std::max<size_t>(1, stmCapacity)), wmCapacity_(wmCapacity)
  {
    residentFrames_.reserve(stmCapacity_+wmCapacity_);
  }

  Memory(const Memory &) = delete;
  Memory &operator=(const Memory &) = delete;

  void saveKeyframe(int id, double stamp, const Eigen::Isometry3f &odomPose, const std::shared_ptr<const vvec<float, 3>> &cloud, const LocalGrid &grid)
  {
    database_.saveKeyframe(id, stamp, odomPose, *cloud, grid);
    std::lock_guard<std::mutex> lock(mutex_);
    ResidentFrame frame;
    frame.cloud = cloud;
    frame.grid = std::make_shared<LocalGrid>(grid);
    frame.tier = Tier::ShortTerm;
    residentFrames_[id] = std::move(frame);
    shortTermIds_.push_back(id);
    promoteShortTerm();
  }

  std::shared_ptr<const vvec<float, 3>> loadCloud(int id)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto iter = residentFrames_.find(id);
      if(iter != residentFrames_.end() && iter->second.cloud)
      {
        return iter->second.cloud;
      }
    }
    const std::shared_ptr<const vvec<float, 3>> cloud = database_.loadCloud(id);
    addLoadedFrame(id, cloud, nullptr);
    return cloud;
  }

  bool loadLocalGrid(int id, LocalGrid &grid)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto iter = residentFrames_.find(id);
      if(iter != residentFrames_.end() && iter->second.grid)
      {
        grid = *iter->second.grid;
        return true;
      }
    }
    LocalGrid loaded(grid.cellSize);
    if(!database_.loadLocalGrid(id, loaded))
    {
      return false;
    }
    grid = loaded;
    addLoadedFrame(id, nullptr, std::make_shared<LocalGrid>(std::move(loaded)));
    return true;
  }

  void saveOptimizedPose(int id, const Eigen::Isometry3f &pose)
  {
    database_.saveOptimizedPose(id, pose);
  }

  void saveOptimizedPoses(const std::vector<std::pair<int, Eigen::Isometry3f>> &poses)
  {
    database_.saveOptimizedPoses(poses);
  }

  void saveLink(int fromId, int toId, int type, const Eigen::Isometry3f &transform)
  {
    database_.saveLink(fromId, toId, type, transform);
  }

  size_t shortTermSize() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return shortTermIds_.size();
  }

  size_t workingSize() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return workingIds_.size();
  }

  size_t residentSize() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return residentFrames_.size();
  }

private:
  enum class Tier {ShortTerm, Working};

  struct ResidentFrame
  {
    std::shared_ptr<const vvec<float, 3>> cloud;
    std::shared_ptr<const LocalGrid> grid;
    Tier tier = Tier::Working;
  };

  void addLoadedFrame(int id, const std::shared_ptr<const vvec<float, 3>> &cloud, const std::shared_ptr<const LocalGrid> &grid)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto [iter, inserted] = residentFrames_.try_emplace(id);
    if(cloud)
    {
      iter->second.cloud = cloud;
    }
    if(grid)
    {
      iter->second.grid = grid;
    }
    if(inserted)
    {
      iter->second.tier = Tier::Working;
      workingIds_.push_back(id);
      trimWorkingMemory();
    }
  }

  void promoteShortTerm()
  {
    while(shortTermIds_.size()>stmCapacity_)
    {
      const int id = shortTermIds_.front();
      shortTermIds_.pop_front();
      const auto iter = residentFrames_.find(id);
      if(iter == residentFrames_.end()) continue;
      iter->second.tier = Tier::Working;
      workingIds_.push_back(id);
    }
    trimWorkingMemory();
  }

  void trimWorkingMemory()
  {
    while(workingIds_.size()>wmCapacity_)
    {
      const int id = workingIds_.front();
      workingIds_.pop_front();
      const auto iter = residentFrames_.find(id);
      if(iter != residentFrames_.end() && iter->second.tier==Tier::Working)
      {
        residentFrames_.erase(iter);
      }
    }
  }

  LioDatabase database_;
  const size_t stmCapacity_;
  const size_t wmCapacity_;
  mutable std::mutex mutex_;
  std::unordered_map<int, ResidentFrame> residentFrames_;
  std::deque<int> shortTermIds_;
  std::deque<int> workingIds_;
};
