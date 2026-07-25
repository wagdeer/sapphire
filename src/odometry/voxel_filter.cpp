#include <sapphire/odometry/voxel_filter.hpp>

#include <small_gicp/util/sort_omp.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <omp.h>
#include <stdexcept>
#include <vector>

namespace sapphire {
namespace {

struct IndexedVoxel {
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;
    size_t point_index = 0;
    bool valid = false;
};

bool sameVoxel(const IndexedVoxel& lhs, const IndexedVoxel& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

}  // namespace

PointCloudPtr deterministicVoxelDownsample(
    const PointCloud& points,
    double leaf_size,
    int max_threads)
{
    if (!std::isfinite(leaf_size) || leaf_size <= 0.0) {
        throw std::invalid_argument(
            "voxel leaf size must be finite and positive");
    }

    auto output = std::make_shared<PointCloud>();
    output->header = points.header;
    output->sensor_origin_ = points.sensor_origin_;
    output->sensor_orientation_ = points.sensor_orientation_;
    output->is_dense = points.is_dense;
    if (points.empty()) {
        return output;
    }

    constexpr size_t kParallelPointThreshold = 4096;
    const int thread_count = std::max(
        1, std::min(max_threads, omp_get_max_threads()));
    const bool use_parallel =
        points.size() >= kParallelPointThreshold && thread_count > 1;
    const bool in_parallel = omp_in_parallel();
    const double inverse_leaf_size = 1.0 / leaf_size;
    const double coordinate_limit = std::ldexp(1.0, 63);

    std::vector<IndexedVoxel> voxels(points.size());

    auto compute_voxel = [&](std::int64_t i) {
        const Point& point = points.points[static_cast<size_t>(i)];
        IndexedVoxel voxel;
        voxel.point_index = static_cast<size_t>(i);
        const double x = std::floor(
            static_cast<double>(point.x) * inverse_leaf_size);
        const double y = std::floor(
            static_cast<double>(point.y) * inverse_leaf_size);
        const double z = std::floor(
            static_cast<double>(point.z) * inverse_leaf_size);
        voxel.valid = std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
            && x >= -coordinate_limit && x < coordinate_limit
            && y >= -coordinate_limit && y < coordinate_limit
            && z >= -coordinate_limit && z < coordinate_limit;
        if (voxel.valid) {
            voxel.x = static_cast<std::int64_t>(x);
            voxel.y = static_cast<std::int64_t>(y);
            voxel.z = static_cast<std::int64_t>(z);
        }
        voxels[static_cast<size_t>(i)] = voxel;
    };

    // Dispatch: use outer parallel team when already inside a parallel
    // region, otherwise create one. This avoids nested fork/join overhead.
    const std::int64_t n = static_cast<std::int64_t>(points.size());
    if (in_parallel) {
#pragma omp for schedule(static)
        for (std::int64_t i = 0; i < n; ++i) {
            compute_voxel(i);
        }
    } else if (use_parallel) {
#pragma omp parallel for schedule(static) num_threads(thread_count)
        for (std::int64_t i = 0; i < n; ++i) {
            compute_voxel(i);
        }
    } else {
        for (std::int64_t i = 0; i < n; ++i) {
            compute_voxel(i);
        }
    }

    const auto compare = [](const IndexedVoxel& lhs, const IndexedVoxel& rhs) {
        if (lhs.valid != rhs.valid) {
            return lhs.valid > rhs.valid;
        }
        if (lhs.z != rhs.z) {
            return lhs.z < rhs.z;
        }
        if (lhs.y != rhs.y) {
            return lhs.y < rhs.y;
        }
        if (lhs.x != rhs.x) {
            return lhs.x < rhs.x;
        }
        return lhs.point_index < rhs.point_index;
    };
    if (use_parallel) {
        small_gicp::quick_sort_omp(
            voxels.begin(), voxels.end(), compare, thread_count);
    } else {
        std::sort(voxels.begin(), voxels.end(), compare);
    }

    output->points.reserve(points.size());
    size_t begin = 0;
    while (begin < voxels.size() && voxels[begin].valid) {
        size_t end = begin + 1;
        while (end < voxels.size()
               && voxels[end].valid
               && sameVoxel(voxels[begin], voxels[end])) {
            ++end;
        }

        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        double sum_intensity = 0.0;
        double sum_timestamp = 0.0;
        for (size_t i = begin; i < end; ++i) {
            const Point& point = points.points[voxels[i].point_index];
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
            sum_intensity += point.intensity;
            sum_timestamp += point.timestamp;
        }

        const double inverse_count =
            1.0 / static_cast<double>(end - begin);
        Point centroid{};
        centroid.x = static_cast<float>(sum_x * inverse_count);
        centroid.y = static_cast<float>(sum_y * inverse_count);
        centroid.z = static_cast<float>(sum_z * inverse_count);
        centroid.data[3] = 1.0f;
        centroid.intensity =
            static_cast<float>(sum_intensity * inverse_count);
        centroid.timestamp = sum_timestamp * inverse_count;
        output->points.push_back(centroid);
        begin = end;
    }

    output->width = output->points.size();
    output->height = 1;
    return output;
}

}  // namespace sapphire
