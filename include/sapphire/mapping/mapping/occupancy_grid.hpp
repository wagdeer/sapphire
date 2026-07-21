#pragma once

#include <sapphire/types.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace sapphire {

/// ROS-free occupancy grid export (nav_msgs/OccupancyGrid compatible).
struct OccupancyGridMsg {
    double resolution = 0.1;
    double origin_x = 0.0;
    double origin_y = 0.0;
    int width = 0;
    int height = 0;
    /// -1 unknown, 0 free, 100 occupied
    std::vector<int8_t> data;
    size_t revision = 0;
};

/// Attitude-aware 2.5D occupancy grid for global navigation.
///
/// Filtering plane: origin = keyframe translation t, normal = R.col(2)
/// (body z in map frame). Points with signed distance d outside
/// [-h_clearance, d_max] are not marked occupied, but still cast free rays.
class OccupancyGrid {
public:
    using Options = Config::Pgo::Occupancy;

    explicit OccupancyGrid(const Options& options);
    ~OccupancyGrid();

    OccupancyGrid(const OccupancyGrid&) = delete;
    OccupancyGrid& operator=(const OccupancyGrid&) = delete;

    void clear();

    /// Expand map AABB (with margin) so the given world bounds fit.
    void ensureBounds(double min_x, double min_y, double max_x, double max_y);

    /// Insert one scan. cloud_lidar is in the lidar/body frame; T_map_lidar
    /// is the optimized pose used for projection and plane definition.
    void insertScan(
        const PointCloudConstPtr& cloud_lidar,
        const Isometry3d& T_map_lidar);

    OccupancyGridMsg toMsg() const;

    size_t revision() const;

    bool empty() const;

private:
    static constexpr int kSubGridBits = 4;
    static constexpr int kSubGridWidth = 1 << kSubGridBits;  // 16
    static constexpr int kSubGridCells = kSubGridWidth * kSubGridWidth;

    struct CellData {
        uint32_t hit_cnt = 0;
        uint32_t visit_cnt = 0;
        float d_min = 1e4f;
    };

    class SubGrid {
    public:
        SubGrid() = default;
        SubGrid(const SubGrid& other);
        SubGrid& operator=(const SubGrid& other);
        SubGrid(SubGrid&&) noexcept = default;
        SubGrid& operator=(SubGrid&&) noexcept = default;

        void clear();
        CellData* cell(int sub_x, int sub_y);
        const CellData* cell(int sub_x, int sub_y) const;
        bool allocated() const { return data_ != nullptr; }

    private:
        void mallocIfNeeded();
        std::unique_ptr<CellData[]> data_;
    };

    bool worldToGlobalIndex(double x, double y, int& gx, int& gy) const;
    CellData* mutableCell(int gx, int gy);
    void updateHit(int gx, int gy, float d);
    void updateMiss(int gx, int gy, float d_ray);
    void castRay(
        double origin_x,
        double origin_y,
        double end_x,
        double end_y,
        float d_sensor,
        float d_end,
        bool mark_hit);
    void resizeTo(
        double min_x,
        double min_y,
        double max_x,
        double max_y);

    bool isOccupied(const CellData& cell) const;
    bool isFree(const CellData& cell) const;

    Options options_;
    float min_x_ = 0.0f;
    float min_y_ = 0.0f;
    float max_x_ = 0.0f;
    float max_y_ = 0.0f;
    int grid_size_x_ = 0;  // number of SubGrids in x
    int grid_size_y_ = 0;
    float subgrid_reso_ = 0.0f;  // resolution * 16
    std::vector<SubGrid> grids_;
    size_t revision_ = 0;
    mutable std::mutex mutex_;
};

}  // namespace sapphire
