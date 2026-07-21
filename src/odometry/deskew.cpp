#include <sapphire/odometry/deskew.hpp>
#include <sapphire/odometry/detail/mean_only_gal3_integrator.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <omp.h>

namespace sapphire {

namespace {

using Gal3 = detail::MeanOnlyGal3Integrator::Gal3;

struct TimedState {
    Gal3 state;
    double stamp = 0.0;
};

struct ScanTimeline {
    PointCloud sorted_scan;
    std::vector<double> stamps;
    std::vector<size_t> group_offsets;
};

Point transformPoint(const Point& source, const Isometry3f& transform) {
    Point output = source;
    output.getVector3fMap() = transform * source.getVector3fMap();
    output.data[3] = 1.0f;
    return output;
}

DeskewResult makeFallback(
    const PointCloudConstPtr& scan,
    const Isometry3d& T_world_imu,
    const Eigen::Vector3d& v_world,
    const Isometry3d& T_imu_lidar,
    double reference_stamp,
    DeskewStatus status)
{
    const Isometry3d T_world_lidar = T_world_imu * T_imu_lidar;
    const Isometry3f T_world_lidar_f = T_world_lidar.cast<float>();
    auto out = std::make_shared<PointCloud>();
    out->points.reserve(scan ? scan->points.size() : 0);
    if (scan) {
        for (const Point& point : scan->points) {
            out->points.push_back(transformPoint(point, T_world_lidar_f));
        }
    }
    out->width = out->points.size();
    out->height = 1;
    out->is_dense = false;

    DeskewResult result;
    result.cloud = out;
    result.T_world_lidar_ref = T_world_lidar;
    result.v_world_ref = v_world;
    result.reference_stamp = reference_stamp;
    result.status = status;
    return result;
}

Isometry3d poseFromState(const Gal3& state) {
    Isometry3d pose = Isometry3d::Identity();
    pose.linear() = state.R();
    pose.translation() = state.p();
    return pose;
}

DeskewStatus buildScanTimeline(
    const PointCloud& scan,
    double scan_stamp,
    bool time_offset,
    ScanTimeline& timeline)
{
    timeline.sorted_scan.points = scan.points;
    std::sort(
        timeline.sorted_scan.points.begin(),
        timeline.sorted_scan.points.end(),
        [](const Point& a, const Point& b) {
            return a.timestamp < b.timestamp;
        });

    const double min_point_time =
        timeline.sorted_scan.points.front().timestamp;
    const double max_point_time =
        timeline.sorted_scan.points.back().timestamp;
    if (!std::isfinite(min_point_time) || !std::isfinite(max_point_time)
        || max_point_time - min_point_time < 1e-12) {
        return DeskewStatus::NoPointTimestamps;
    }

    const double offset =
        scan_stamp - (time_offset ? min_point_time : 0.0);
    for (size_t i = 0; i < timeline.sorted_scan.points.size(); ++i) {
        if (i == 0
            || timeline.sorted_scan.points[i].timestamp
                != timeline.sorted_scan.points[i - 1].timestamp) {
            timeline.stamps.push_back(
                timeline.sorted_scan.points[i].timestamp + offset);
            timeline.group_offsets.push_back(i);
        }
    }
    timeline.group_offsets.push_back(timeline.sorted_scan.points.size());
    return DeskewStatus::Success;
}

DeskewStatus findImuStart(
    const ImuBuffer& imu_buf,
    double prev_stamp,
    double scan_start,
    double scan_end,
    size_t& imu_start)
{
    if (imu_buf.empty()) {
        return DeskewStatus::EmptyImuBuffer;
    }
    if (scan_start < prev_stamp) {
        return DeskewStatus::InvalidTimeRange;
    }

    for (size_t i = 1; i < imu_buf.size(); ++i) {
        if (!(imu_buf[i].stamp > imu_buf[i - 1].stamp)) {
            return DeskewStatus::InvalidImuOrder;
        }
    }

    imu_start = imu_buf.size();
    for (size_t i = 0; i < imu_buf.size(); ++i) {
        if (imu_buf[i].stamp <= prev_stamp) {
            imu_start = i;
        } else {
            break;
        }
    }

    if (imu_start == imu_buf.size()
        || imu_start + 1 >= imu_buf.size()
        || imu_buf.back().stamp < scan_end) {
        return DeskewStatus::InsufficientImuCoverage;
    }
    return DeskewStatus::Success;
}

Gal3 interpolateState(
    const Gal3& start,
    const Gal3& end,
    double alpha,
    double interval_dt)
{
    alpha = std::clamp(alpha, 0.0, 1.0);
    const Eigen::Quaterniond rotation =
        start.q().slerp(alpha, end.q()).normalized();
    const Eigen::Vector3d velocity =
        (1.0 - alpha) * start.v() + alpha * end.v();

    // Cubic Hermite interpolation preserves constant velocity and constant
    // world acceleration exactly while remaining cheap for every point stamp.
    const double a2 = alpha * alpha;
    const double a3 = a2 * alpha;
    const double h00 = 2.0 * a3 - 3.0 * a2 + 1.0;
    const double h10 = a3 - 2.0 * a2 + alpha;
    const double h01 = -2.0 * a3 + 3.0 * a2;
    const double h11 = a3 - a2;
    const Eigen::Vector3d position =
        h00 * start.p()
        + h10 * interval_dt * start.v()
        + h01 * end.p()
        + h11 * interval_dt * end.v();

    Gal3::IsometriesType isometries{velocity, position};
    return Gal3(rotation, isometries, 0.0);
}

std::vector<TimedState> integrateTimeline(
    const std::vector<double>& target_stamps,
    const ImuBuffer& imu_buf,
    size_t imu_start,
    double prev_stamp,
    const Isometry3d& T_world_imu_prev,
    const Eigen::Vector3d& v_world_prev,
    const Eigen::Vector3d& gravity_world)
{
    detail::MeanOnlyGal3Integrator integrator;

    Gal3::IsometriesType initial_isometries{
        v_world_prev,
        T_world_imu_prev.translation(),
    };
    const Gal3 initial_state(
        T_world_imu_prev.rotation(), initial_isometries, 0.0);

    std::vector<TimedState> states;
    states.reserve(target_stamps.size());
    size_t target_idx = 0;
    double integrated_until = prev_stamp;
    // World state recovered at the end of the previous IMU interval; the first
    // interval uses prev_stamp as its start.
    Gal3 prev_interval_end_state = initial_state;

    for (size_t imu_idx = imu_start + 1;
         imu_idx < imu_buf.size() && target_idx < target_stamps.size();
         ++imu_idx) {
        const ImuData& measurement = imu_buf[imu_idx];
        const double interval_end = measurement.stamp;
        const double interval_dt = interval_end - integrated_until;
        if (interval_dt <= 0.0) {
            continue;
        }

        // Upsilon accumulates from prev_stamp; recoverWorldState always uses
        // initial_state at prev_stamp and Gamma built from the total elapsed s.
        integrator.integrate(measurement, interval_dt);
        const Gal3 interval_end_state = detail::recoverWorldState(
            integrator.Upsilon(),
            initial_state,
            gravity_world);

        if (target_idx < target_stamps.size()
            && target_stamps[target_idx] <= interval_end) {
            while (target_idx < target_stamps.size()
                   && target_stamps[target_idx] <= interval_end) {
                const double target_stamp = target_stamps[target_idx];
                const double alpha =
                    (target_stamp - integrated_until) / interval_dt;
                states.push_back({
                    interpolateState(
                        prev_interval_end_state,
                        interval_end_state,
                        alpha,
                        interval_dt),
                    target_stamp,
                });
                ++target_idx;
            }
        }

        integrated_until = interval_end;
        prev_interval_end_state = interval_end_state;
    }
    return states;
}

PointCloudPtr transformScan(
    const ScanTimeline& timeline,
    const std::vector<TimedState>& states,
    const Isometry3d& T_imu_lidar)
{
    auto output = std::make_shared<PointCloud>();
    output->points.resize(timeline.sorted_scan.points.size());

    constexpr size_t kParallelPointThreshold = 4096;
    constexpr int kMaxDeskewThreads = 4;
    const int thread_count =
        std::min(kMaxDeskewThreads, omp_get_max_threads());
    const auto group_count =
        static_cast<std::ptrdiff_t>(timeline.stamps.size());
    const bool use_parallel =
        output->points.size() >= kParallelPointThreshold && group_count > 1;
    const bool in_parallel = omp_in_parallel();

    if (in_parallel) {
#pragma omp for schedule(static)
        for (std::ptrdiff_t group = 0; group < group_count; ++group) {
            const size_t group_index = static_cast<size_t>(group);
            const Isometry3f T_world_lidar = (
                poseFromState(states[group_index].state) * T_imu_lidar
            ).cast<float>();
            const size_t begin = timeline.group_offsets[group_index];
            const size_t end = timeline.group_offsets[group_index + 1];
            for (size_t point_idx = begin; point_idx < end; ++point_idx) {
                output->points[point_idx] = transformPoint(
                    timeline.sorted_scan.points[point_idx], T_world_lidar);
            }
        }
    } else if (use_parallel) {
#pragma omp parallel for schedule(static) num_threads(thread_count)
        for (std::ptrdiff_t group = 0; group < group_count; ++group) {
            const size_t group_index = static_cast<size_t>(group);
            const Isometry3f T_world_lidar = (
                poseFromState(states[group_index].state) * T_imu_lidar
            ).cast<float>();
            const size_t begin = timeline.group_offsets[group_index];
            const size_t end = timeline.group_offsets[group_index + 1];
            for (size_t point_idx = begin; point_idx < end; ++point_idx) {
                output->points[point_idx] = transformPoint(
                    timeline.sorted_scan.points[point_idx], T_world_lidar);
            }
        }
    } else {
        for (std::ptrdiff_t group = 0; group < group_count; ++group) {
            const size_t group_index = static_cast<size_t>(group);
            const Isometry3f T_world_lidar = (
                poseFromState(states[group_index].state) * T_imu_lidar
            ).cast<float>();
            const size_t begin = timeline.group_offsets[group_index];
            const size_t end = timeline.group_offsets[group_index + 1];
            for (size_t point_idx = begin; point_idx < end; ++point_idx) {
                output->points[point_idx] = transformPoint(
                    timeline.sorted_scan.points[point_idx], T_world_lidar);
            }
        }
    }

    output->width = output->points.size();
    output->height = 1;
    output->is_dense = false;
    return output;
}

}  // namespace

DeskewResult deskew(
    const PointCloudConstPtr& scan,
    double scan_stamp,
    const ImuBuffer& imu_buf,
    double prev_stamp,
    const Isometry3d& T_world_imu_prev,
    const Eigen::Vector3d& v_world_prev,
    const Isometry3d& T_imu_lidar,
    const Eigen::Vector3d& gravity_world,
    const ImuNoiseConfig& noise,
    bool time_offset)
{
    (void)noise;

    if (!scan || scan->empty()) {
        spdlog::warn("[deskew] empty scan");
        return makeFallback(
            scan, T_world_imu_prev, v_world_prev, T_imu_lidar, prev_stamp,
            DeskewStatus::EmptyScan);
    }

    ScanTimeline timeline;
    DeskewStatus status =
        buildScanTimeline(*scan, scan_stamp, time_offset, timeline);
    if (status != DeskewStatus::Success) {
        return makeFallback(
            scan, T_world_imu_prev, v_world_prev, T_imu_lidar, prev_stamp,
            status);
    }
    size_t imu_start = imu_buf.size();
    status = findImuStart(
        imu_buf,
        prev_stamp,
        timeline.stamps.front(),
        timeline.stamps.back(),
        imu_start);
    if (status != DeskewStatus::Success) {
        return makeFallback(
            scan, T_world_imu_prev, v_world_prev, T_imu_lidar, prev_stamp,
            status);
    }

    const std::vector<TimedState> states = integrateTimeline(
        timeline.stamps,
        imu_buf,
        imu_start,
        prev_stamp,
        T_world_imu_prev,
        v_world_prev,
        gravity_world);
    if (states.size() != timeline.stamps.size()) {
        return makeFallback(
            scan, T_world_imu_prev, v_world_prev, T_imu_lidar, prev_stamp,
            DeskewStatus::InsufficientImuCoverage);
    }

    const size_t reference_idx = timeline.stamps.size() / 2;
    const TimedState& reference = states[reference_idx];
    DeskewResult result;
    result.cloud = transformScan(timeline, states, T_imu_lidar);
    result.T_world_lidar_ref =
        poseFromState(reference.state) * T_imu_lidar;
    result.v_world_ref = reference.state.v();
    result.reference_stamp = reference.stamp;
    result.status = DeskewStatus::Success;
    return result;
}

}  // namespace sapphire
