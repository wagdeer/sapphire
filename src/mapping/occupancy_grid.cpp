#include <sapphire/mapping/occupancy_grid.hpp>
#include <sapphire/odometry/voxel_filter.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace sapphire {

OccupancyGrid::SubGrid::SubGrid(const SubGrid& other) {
    if (other.data_) {
        data_ = std::make_unique<CellData[]>(kSubGridCells);
        std::copy(other.data_.get(), other.data_.get() + kSubGridCells, data_.get());
    }
}

OccupancyGrid::SubGrid& OccupancyGrid::SubGrid::operator=(const SubGrid& other) {
    if (this == &other) {
        return *this;
    }
    if (!other.data_) {
        data_.reset();
        return *this;
    }
    if (!data_) {
        data_ = std::make_unique<CellData[]>(kSubGridCells);
    }
    std::copy(other.data_.get(), other.data_.get() + kSubGridCells, data_.get());
    return *this;
}

void OccupancyGrid::SubGrid::clear() {
    data_.reset();
}

void OccupancyGrid::SubGrid::mallocIfNeeded() {
    if (!data_) {
        data_ = std::make_unique<CellData[]>(kSubGridCells);
    }
}

OccupancyGrid::CellData* OccupancyGrid::SubGrid::cell(int sub_x, int sub_y) {
    if (sub_x < 0 || sub_x >= kSubGridWidth || sub_y < 0 || sub_y >= kSubGridWidth) {
        return nullptr;
    }
    mallocIfNeeded();
    return &data_[sub_y * kSubGridWidth + sub_x];
}

const OccupancyGrid::CellData* OccupancyGrid::SubGrid::cell(
    int sub_x,
    int sub_y) const
{
    if (!data_
        || sub_x < 0
        || sub_x >= kSubGridWidth
        || sub_y < 0
        || sub_y >= kSubGridWidth) {
        return nullptr;
    }
    return &data_[sub_y * kSubGridWidth + sub_x];
}

OccupancyGrid::OccupancyGrid(const Options& options)
    : options_(options)
    , subgrid_reso_(static_cast<float>(options.resolution * kSubGridWidth))
{
}

OccupancyGrid::~OccupancyGrid() = default;

void OccupancyGrid::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    grids_.clear();
    allocated_subgrids_.clear();
    grid_size_x_ = 0;
    grid_size_y_ = 0;
    min_x_ = min_y_ = 0.0f;
    max_x_ = max_y_ = 0.0f;
    visited_max_wx_ = kUnvisitedSentinel;
    visited_max_wy_ = kUnvisitedSentinel;
    occupied_cells_ = 0;
    free_cells_ = 0;
    ++revision_;
}

size_t OccupancyGrid::revision() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return revision_;
}

bool OccupancyGrid::empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return grids_.empty();
}

void OccupancyGrid::ensureBounds(
    double min_x,
    double min_y,
    double max_x,
    double max_y)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const double margin = options_.margin;
    resizeTo(
        min_x - margin,
        min_y - margin,
        max_x + margin,
        max_y + margin);
}

void OccupancyGrid::resizeTo(
    double min_x,
    double min_y,
    double max_x,
    double max_y)
{
    if (!(max_x > min_x) || !(max_y > min_y)) {
        return;
    }

    if (grids_.empty()) {
        // Snap min to SubGrid boundary so shift_x/y are exact integers on
        // future expansions. Without snapping, the SubGrid-level data shift
        // (1.6 m granularity) and cell-level origin change (0.1 m granularity)
        // diverge, causing existing grid data to misalign by up to 8 cells.
        min_x_ = std::floor(static_cast<float>(min_x / subgrid_reso_))
            * subgrid_reso_;
        min_y_ = std::floor(static_cast<float>(min_y / subgrid_reso_))
            * subgrid_reso_;
        grid_size_x_ = static_cast<int>(
            std::ceil((static_cast<float>(max_x) - min_x_) / subgrid_reso_));
        grid_size_y_ = static_cast<int>(
            std::ceil((static_cast<float>(max_y) - min_y_) / subgrid_reso_));
        grid_size_x_ = std::max(grid_size_x_, 1);
        grid_size_y_ = std::max(grid_size_y_, 1);
        max_x_ = min_x_ + grid_size_x_ * subgrid_reso_;
        max_y_ = min_y_ + grid_size_y_ * subgrid_reso_;
        grids_.assign(
            static_cast<size_t>(grid_size_x_ * grid_size_y_), SubGrid{});
        allocated_subgrids_.clear();
        return;
    }

    if (min_x >= min_x_ && min_y >= min_y_ && max_x <= max_x_ && max_y <= max_y_) {
        return;
    }

    // Expand to the nearest SubGrid-aligned boundary that covers the request.
    const float new_min_x = std::min(min_x_, static_cast<float>(min_x));
    const float new_min_y = std::min(min_y_, static_cast<float>(min_y));
    const float new_max_x = std::max(max_x_, static_cast<float>(max_x));
    const float new_max_y = std::max(max_y_, static_cast<float>(max_y));

    const float snapped_min_x =
        std::floor(new_min_x / subgrid_reso_) * subgrid_reso_;
    const float snapped_min_y =
        std::floor(new_min_y / subgrid_reso_) * subgrid_reso_;

    int new_size_x = static_cast<int>(
        std::ceil((new_max_x - snapped_min_x) / subgrid_reso_));
    int new_size_y = static_cast<int>(
        std::ceil((new_max_y - snapped_min_y) / subgrid_reso_));
    new_size_x = std::max(new_size_x, 1);
    new_size_y = std::max(new_size_y, 1);

    const float snapped_max_x = snapped_min_x + new_size_x * subgrid_reso_;
    const float snapped_max_y = snapped_min_y + new_size_y * subgrid_reso_;

    // Both old and new corner are SubGrid multiples → division is exact.
    const int shift_x = static_cast<int>(
        std::llround((min_x_ - snapped_min_x) / subgrid_reso_));
    const int shift_y = static_cast<int>(
        std::llround((min_y_ - snapped_min_y) / subgrid_reso_));

    std::vector<SubGrid> new_grids(
        static_cast<size_t>(new_size_x * new_size_y));
    for (int x = 0; x < grid_size_x_; ++x) {
        for (int y = 0; y < grid_size_y_; ++y) {
            const int nx = x + shift_x;
            const int ny = y + shift_y;
            if (nx < 0 || ny < 0 || nx >= new_size_x || ny >= new_size_y) {
                continue;
            }
            new_grids[static_cast<size_t>(ny * new_size_x + nx)] =
                std::move(grids_[static_cast<size_t>(y * grid_size_x_ + x)]);
        }
    }

    grids_ = std::move(new_grids);
    min_x_ = snapped_min_x;
    min_y_ = snapped_min_y;
    max_x_ = snapped_max_x;
    max_y_ = snapped_max_y;
    grid_size_x_ = new_size_x;
    grid_size_y_ = new_size_y;

    // Rebuild allocated SubGrid index after grid remapping.
    allocated_subgrids_.clear();
    const size_t total = static_cast<size_t>(grid_size_x_ * grid_size_y_);
    allocated_subgrids_.reserve(total);
    for (size_t i = 0; i < total; ++i) {
        if (grids_[i].allocated()) {
            allocated_subgrids_.push_back(i);
        }
    }
}

bool OccupancyGrid::worldToGlobalIndex(
    double x,
    double y,
    int& gx,
    int& gy) const
{
    if (grids_.empty()
        || x < min_x_
        || y < min_y_
        || x >= max_x_
        || y >= max_y_) {
        return false;
    }
    gx = static_cast<int>(std::floor((x - min_x_) / options_.resolution));
    gy = static_cast<int>(std::floor((y - min_y_) / options_.resolution));
    return gx >= 0
        && gy >= 0
        && gx < grid_size_x_ * kSubGridWidth
        && gy < grid_size_y_ * kSubGridWidth;
}

OccupancyGrid::CellData* OccupancyGrid::mutableCell(int gx, int gy) {
    const int sx = gx >> kSubGridBits;
    const int sy = gy >> kSubGridBits;
    if (sx < 0 || sy < 0 || sx >= grid_size_x_ || sy >= grid_size_y_) {
        return nullptr;
    }
    const int sub_x = gx & (kSubGridWidth - 1);
    const int sub_y = gy & (kSubGridWidth - 1);
    const size_t idx =
        static_cast<size_t>(sy * grid_size_x_ + sx);
    SubGrid& sub = grids_[idx];
    const bool was_allocated = sub.allocated();
    CellData* cell = sub.cell(sub_x, sub_y);
    if (!was_allocated && sub.allocated()) {
        allocated_subgrids_.push_back(idx);
    }
    return cell;
}

bool OccupancyGrid::isOccupied(const CellData& cell) const {
    if (cell.hit_cnt < 2 || cell.visit_cnt == 0) {
        return false;
    }
    return static_cast<float>(cell.hit_cnt)
        / static_cast<float>(cell.visit_cnt)
        > static_cast<float>(options_.occ_threshold);
}

bool OccupancyGrid::isFree(const CellData& cell) const {
    return cell.visit_cnt > 0 && !isOccupied(cell);
}

void OccupancyGrid::updateHit(int gx, int gy, float d) {
    CellData* cell = mutableCell(gx, gy);
    if (!cell) {
        return;
    }

    // Track visited bounds in world coordinates (invariant under grid expansion).
    const float wx = min_x_ + (static_cast<float>(gx) + 0.5f)
        * static_cast<float>(options_.resolution);
    const float wy = min_y_ + (static_cast<float>(gy) + 0.5f)
        * static_cast<float>(options_.resolution);
    visited_max_wx_ = std::max(visited_max_wx_, wx);
    visited_max_wy_ = std::max(visited_max_wy_, wy);

    // Snapshot occupancy state before mutation for incremental counters.
    const bool was_occupied = isOccupied(*cell);
    const bool was_free = isFree(*cell);

    // First hit resets d_min to the current obstacle height. Subsequent
    // hits track the lowest observed height so free rays cannot punch
    // through from above. Without the reset, a near-ground in-band point
    // that lands first can permanently depress d_min, blocking all later
    // free-ray clearing (ghost obstacles / multi-layer walls).
    if (cell->hit_cnt == 0) {
        cell->d_min = d;
    } else {
        cell->d_min = std::min(cell->d_min, d);
    }
    cell->hit_cnt += 1;
    cell->visit_cnt += 1;

    // Apply counter deltas.
    const bool now_occupied = isOccupied(*cell);
    const bool now_free = isFree(*cell);
    if (!was_occupied && now_occupied) { ++occupied_cells_; }
    if (was_occupied && !now_occupied) { --occupied_cells_; }
    if (!was_free && now_free) { ++free_cells_; }
    if (was_free && !now_free) { --free_cells_; }
}

void OccupancyGrid::updateMiss(int gx, int gy, float d_ray) {
    CellData* cell = mutableCell(gx, gy);
    if (!cell) {
        return;
    }

    // Track visited bounds in world coordinates (invariant under grid expansion).
    const float wx = min_x_ + (static_cast<float>(gx) + 0.5f)
        * static_cast<float>(options_.resolution);
    const float wy = min_y_ + (static_cast<float>(gy) + 0.5f)
        * static_cast<float>(options_.resolution);
    visited_max_wx_ = std::max(visited_max_wx_, wx);
    visited_max_wy_ = std::max(visited_max_wy_, wy);

    // Snapshot occupancy state before mutation for incremental counters.
    const bool was_occupied = isOccupied(*cell);
    const bool was_free = isFree(*cell);

    // Free rays never define obstacle height (d_min). Their only role is
    // visit counting for hit/visit occupancy, and as a dynamic-obstacle
    // signal: a free ray at/below the recorded underside suggests the
    // obstacle has been cleared or the hit endpoint jittered along the ray.
    //
    // Use <= d_min + eps (not strict < d_min): same-height pass-through is
    // exactly what happens when a wall endpoint moves one cell farther —
    // strict < left the old cell permanently occupied (multi-layer walls).
    if (cell->hit_cnt > 0) {
        const float clear_ceiling = cell->d_min
            + static_cast<float>(options_.clear_height_eps);
        if (d_ray <= clear_ceiling) {
            cell->visit_cnt += 1;
        }
    } else {
        cell->visit_cnt += 1;
    }

    // Apply counter deltas.
    const bool now_occupied = isOccupied(*cell);
    const bool now_free = isFree(*cell);
    if (!was_occupied && now_occupied) { ++occupied_cells_; }
    if (was_occupied && !now_occupied) { --occupied_cells_; }
    if (!was_free && now_free) { ++free_cells_; }
    if (was_free && !now_free) { --free_cells_; }
}

void OccupancyGrid::castRay(
    double origin_x,
    double origin_y,
    double end_x,
    double end_y,
    float d_sensor,
    float d_end,
    bool mark_hit)
{
    int x0 = 0;
    int y0 = 0;
    int x1 = 0;
    int y1 = 0;
    if (!worldToGlobalIndex(origin_x, origin_y, x0, y0)) {
        return;
    }
    if (!worldToGlobalIndex(end_x, end_y, x1, y1)) {
        // Clamp end to map for partial free-space clearing.
        const double cx = std::clamp(end_x, static_cast<double>(min_x_) + 1e-3,
            static_cast<double>(max_x_) - 1e-3);
        const double cy = std::clamp(end_y, static_cast<double>(min_y_) + 1e-3,
            static_cast<double>(max_y_) - 1e-3);
        if (!worldToGlobalIndex(cx, cy, x1, y1)) {
            return;
        }
        // Recompute d_end for the clamped endpoint so that d_step
        // matches the actual Bresenham path length. Without this,
        // d_end corresponds to the original (out-of-bounds) endpoint
        // and each cell's interpolated d_ray is biased.
        const double orig_len = std::hypot(end_x - origin_x, end_y - origin_y);
        const double clamped_len = std::hypot(cx - origin_x, cy - origin_y);
        if (orig_len > 1e-9) {
            const double t = clamped_len / orig_len;
            d_end = d_sensor + static_cast<float>(t) * (d_end - d_sensor);
        }
        mark_hit = false;
    }

    const int dx = x1 - x0;
    const int dy = y1 - y0;
    if (dx == 0 && dy == 0) {
        if (mark_hit) {
            updateHit(x1, y1, d_end);
        }
        return;
    }

    // Horizontal line: simple scan, no Bresenham overhead.
    if (dy == 0) {
        const float d_step =
            (d_end - d_sensor) / static_cast<float>(dx);
        const int sign_x = (dx > 0) - (dx < 0);
        float d_ray = d_sensor;
        for (int i = sign_x; i != dx; i += sign_x) {
            d_ray += d_step;
            updateMiss(x0 + i, y0, d_ray);
        }
    } else if (std::abs(dy) > std::abs(dx)) {
        const float d_step =
            (d_end - d_sensor) / static_cast<float>(dy);
        const int sign_y = (dy > 0) - (dy < 0);
        const int sign_x = (dx > 0) - (dx < 0);
        const int adx = std::abs(dx);
        const int ady = std::abs(dy);
        int error = 2 * adx - ady;
        int i = 0;
        float d_ray = d_sensor;
        for (int j = sign_y; j != dy; j += sign_y) {
            d_ray += d_step;
            if (error > 0) {
                i += sign_x;
                error -= 2 * ady;
            }
            error += 2 * adx;
            updateMiss(x0 + i, y0 + j, d_ray);
        }
    } else {
        const float d_step =
            (d_end - d_sensor) / static_cast<float>(dx);
        const int sign_x = (dx > 0) - (dx < 0);
        const int sign_y = (dy > 0) - (dy < 0);
        const int adx = std::abs(dx);
        const int ady = std::abs(dy);
        int error = 2 * ady - adx;
        int j = 0;
        float d_ray = d_sensor;
        for (int i = sign_x; i != dx; i += sign_x) {
            d_ray += d_step;
            if (error > 0) {
                j += sign_y;
                error -= 2 * adx;
            }
            error += 2 * ady;
            updateMiss(x0 + i, y0 + j, d_ray);
        }
    }

    if (mark_hit) {
        updateHit(x1, y1, d_end);
    } else {
        updateMiss(x1, y1, d_end);
    }
}

void OccupancyGrid::insertScan(
    const PointCloudConstPtr& cloud_lidar,
    const Isometry3d& T_map_lidar)
{
    if (!cloud_lidar || cloud_lidar->empty()
        || !T_map_lidar.matrix().allFinite()) {
        return;
    }

    // Keep occupancy downsample single-threaded: nested OpenMP on the PGO
    // worker steals cores from frontend VGICP/ESKF for almost no gain here.
    PointCloudConstPtr cloud = cloud_lidar;
    if (options_.cloud_voxel_size > 0.0) {
        cloud = deterministicVoxelDownsample(
            *cloud_lidar, options_.cloud_voxel_size, /*max_threads=*/1);
    }

    const Eigen::Vector3d t = T_map_lidar.translation();
    // 2.5D grid uses world Z for height filtering — invariant across keyframes.
    // Body-z filtering (R.col(2)) couples obstacle selection to sensor attitude:
    // tilted mounts or pitch/roll inject out-of-band points or miss in-band ones.
    // World Z gives consistent height bands, and d values are cross-keyframe comparable.
    const Eigen::Vector3d sensor = t;  // lidar origin in map
    const float d_sensor = 0.0f;       // sensor defines reference height
    const float min_range = static_cast<float>(options_.min_range);
    const float max_range = static_cast<float>(options_.usable_range);
    const float h_clearance = static_cast<float>(options_.h_clearance);
    const float ground_margin = static_cast<float>(options_.ground_margin);
    // Raise the lower HIT edge so the near-ground slice (≈ -h_clearance) does
    // not become occupied or lock cell d_min for free-ray clearing.
    const float d_hit_min = -h_clearance + ground_margin;
    const float d_max = static_cast<float>(options_.d_max);

    // Single-pass: collect map-frame XY + filtered height per in-range point.
    // Bounds are accumulated in the same loop. The cached (x,y,d,hit) tuple
    // feeds ray casting after resizeTo so T_map_lidar * p_body is computed once.
    struct MappedPoint {
        float x;
        float y;
        float d;
        bool hit;
    };
    std::vector<MappedPoint> mapped;
    mapped.reserve(cloud->points.size());

    double scan_min_x = t.x();
    double scan_min_y = t.y();
    double scan_max_x = t.x();
    double scan_max_y = t.y();
    float d_min_seen = std::numeric_limits<float>::max();
    float d_max_seen = std::numeric_limits<float>::lowest();

    for (const Point& point : cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)
            || !std::isfinite(point.z)) {
            continue;
        }
        const Eigen::Vector3d p_body(point.x, point.y, point.z);
        const float range = static_cast<float>(p_body.norm());
        if (range < min_range || range > max_range) {
            continue;
        }

        const Eigen::Vector3d p_map = T_map_lidar * p_body;
        const double px = p_map.x();
        const double py = p_map.y();
        const float d = static_cast<float>(p_map.z() - t.z());
        const bool hit = (d >= d_hit_min)
            && (d_max <= 0.0f || d <= d_max);

        scan_min_x = std::min(scan_min_x, px);
        scan_min_y = std::min(scan_min_y, py);
        scan_max_x = std::max(scan_max_x, px);
        scan_max_y = std::max(scan_max_y, py);
        d_min_seen = std::min(d_min_seen, d);
        d_max_seen = std::max(d_max_seen, d);

        mapped.push_back(
            {static_cast<float>(px), static_cast<float>(py), d, hit});
    }

    std::lock_guard<std::mutex> lock(mutex_);
    resizeTo(
        scan_min_x - options_.margin,
        scan_min_y - options_.margin,
        scan_max_x + options_.margin,
        scan_max_y + options_.margin);

    size_t used = mapped.size();
    size_t in_band = 0;
    size_t casted = 0;

    for (const MappedPoint& mp : mapped) {
        if (!mp.hit) {
            // Out-of-band returns (ground / high canopy) must NOT cast long
            // free rays. Mid-360 ground density otherwise floods every cell
            // between the sensor and the return, diluting real obstacles to
            // free before they can be confirmed. Free space is still carved
            // by the miss segment of in-band obstacle rays.
            continue;
        }

        ++in_band;
        ++casted;
        castRay(
            sensor.x(),
            sensor.y(),
            static_cast<double>(mp.x),
            static_cast<double>(mp.y),
            d_sensor,
            mp.d,
            true);
    }
    ++revision_;

    if (used > 0 && (revision_ <= 3 || revision_ % 20 == 0)) {
        spdlog::info(
            "[occupancy] scan rev={} points={} band_hits={} "
            "d=[{:.2f},{:.2f}] band=[{:.2f},{:.2f}] "
            "cells(occ/free)={}/{} world_Z=({:.2f},{:.2f},{:.2f})",
            revision_,
            used,
            casted,
            d_min_seen,
            d_max_seen,
            d_hit_min,
            d_max <= 0.0f ? std::numeric_limits<float>::infinity() : d_max,
            occupied_cells_,
            free_cells_,
            0.0,  // world Z
            0.0,
            1.0);
    }
}


OccupancyGridMsg OccupancyGrid::toMsg() const {
    std::lock_guard<std::mutex> lock(mutex_);
    OccupancyGridMsg msg;
    msg.resolution = options_.resolution;
    msg.revision = revision_;
    if (grids_.empty() || visited_max_wx_ <= kUnvisitedSentinel + 1.0f) {
        return msg;
    }

    msg.origin_x = static_cast<double>(min_x_);
    msg.origin_y = static_cast<double>(min_y_);
    // Cell-center tracking: ceil maps 0.5 → 1 cell, 99.5 → 100 cells.
    msg.width = static_cast<int>(
        std::ceil((static_cast<double>(visited_max_wx_) - msg.origin_x)
            / options_.resolution));
    msg.height = static_cast<int>(
        std::ceil((static_cast<double>(visited_max_wy_) - msg.origin_y)
            / options_.resolution));
    msg.data.assign(
        static_cast<size_t>(msg.width * msg.height),
        static_cast<int8_t>(-1));

    for (size_t idx : allocated_subgrids_) {
        const SubGrid& sub = grids_[idx];
        const int sy = static_cast<int>(idx / static_cast<size_t>(grid_size_x_));
        const int sx = static_cast<int>(idx % static_cast<size_t>(grid_size_x_));
        for (int j = 0; j < kSubGridWidth; ++j) {
            for (int i = 0; i < kSubGridWidth; ++i) {
                const CellData* cell = sub.cell(i, j);
                if (!cell || cell->visit_cnt == 0) {
                    continue;
                }
                const int gx = (sx << kSubGridBits) + i;
                const int gy = (sy << kSubGridBits) + j;
                if (gx >= msg.width || gy >= msg.height) {
                    continue;
                }
                const size_t index =
                    static_cast<size_t>(gy * msg.width + gx);
                if (isOccupied(*cell)) {
                    msg.data[index] = 100;
                } else if (isFree(*cell)) {
                    msg.data[index] = 0;
                }
            }
        }
    }
    return msg;
}

}  // namespace sapphire
