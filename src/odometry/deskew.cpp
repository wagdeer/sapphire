#include <sapphire/odometry/deskew.hpp>

#include <preintegration.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <omp.h>

namespace sapphire {

namespace {

using Pim = preintegration::EquivariantPreintegration<double>;
using Gal3 = Pim::Gal3;
constexpr double kMaxIntegrationStepSec = 0.02;

void integrateMeasurement(
    Pim& pim,
    const ImuData& measurement,
    double dt)
{
    if (dt <= 0.0) {
        return;
    }
    const int steps = std::max(
        1, static_cast<int>(std::ceil(dt / kMaxIntegrationStepSec)));
    const double step_dt = dt / static_cast<double>(steps);
    for (int step = 0; step < steps; ++step) {
        pim.integrateMeasurementMeanOnly(
            measurement.accel, measurement.gyro, step_dt);
    }
}

struct TimedState {
    Gal3 state;
    double stamp = 0.0;
};

struct ScanTimeline {
    PointCloud sorted_scan;
    std::vector<double> stamps;
    std::vector<size_t> group_offsets;
};

Point transformPoint(const Point& source, const Isometry3d& transform) {
    Point output = source;
    Eigen::Vector4d homogeneous(
        static_cast<double>(source.x),
        static_cast<double>(source.y),
        static_cast<double>(source.z),
        1.0);
    homogeneous = transform.matrix() * homogeneous;
    output.x = static_cast<float>(homogeneous.x());
    output.y = static_cast<float>(homogeneous.y());
    output.z = static_cast<float>(homogeneous.z());
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
    auto out = std::make_shared<PointCloud>();
    out->points.reserve(scan ? scan->points.size() : 0);
    if (scan) {
        for (const Point& point : scan->points) {
            out->points.push_back(transformPoint(point, T_world_lidar));
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

std::shared_ptr<preintegration::PreintegrationParams<double>>
makePreintegrationParams(
    const Eigen::Vector3d& gravity_world,
    const ImuNoiseConfig& noise)
{
    return std::make_shared<preintegration::PreintegrationParams<double>>(
        gravity_world,
        noise.gyro_noise_density,
        noise.accel_noise_density,
        0.0,
        0.0,
        noise.gyro_random_walk,
        noise.accel_random_walk,
        0.0,
        0.0);
}

std::vector<TimedState> integrateTimeline(
    const std::vector<double>& target_stamps,
    const ImuBuffer& imu_buf,
    size_t imu_start,
    double prev_stamp,
    const Isometry3d& T_world_imu_prev,
    const Eigen::Vector3d& v_world_prev,
    const Eigen::Vector3d& gravity_world,
    const ImuNoiseConfig& noise)
{
    Pim pim(makePreintegrationParams(gravity_world, noise));
    pim.resetIntegrationAndSetBias(Pim::Vec10::Zero());

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

    for (size_t imu_idx = imu_start + 1;
         imu_idx < imu_buf.size() && target_idx < target_stamps.size();
         ++imu_idx) {
        const ImuData& measurement = imu_buf[imu_idx];
        const double interval_end = measurement.stamp;

        if (target_idx < target_stamps.size()
            && target_stamps[target_idx] <= interval_end) {
            Pim partial = pim;
            double partial_until = integrated_until;
            while (target_idx < target_stamps.size()
                   && target_stamps[target_idx] <= interval_end) {
                const double target_stamp = target_stamps[target_idx];
                const double partial_dt = target_stamp - partial_until;
                integrateMeasurement(partial, measurement, partial_dt);
                states.push_back({
                    partial.Gamma_ij() * initial_state
                        * partial.Upsilon(),
                    target_stamp,
                });
                partial_until = target_stamp;
                ++target_idx;
            }
        }

        const double interval_dt = interval_end - integrated_until;
        integrateMeasurement(pim, measurement, interval_dt);
        integrated_until = interval_end;
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

#pragma omp parallel for schedule(static) num_threads(thread_count) if(use_parallel)
    for (std::ptrdiff_t group = 0; group < group_count; ++group) {
        const size_t group_index = static_cast<size_t>(group);
        const Isometry3d T_world_lidar =
            poseFromState(states[group_index].state) * T_imu_lidar;
        const size_t begin = timeline.group_offsets[group_index];
        const size_t end = timeline.group_offsets[group_index + 1];
        for (size_t point_idx = begin; point_idx < end; ++point_idx) {
            output->points[point_idx] = transformPoint(
                timeline.sorted_scan.points[point_idx], T_world_lidar);
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
        gravity_world,
        noise);
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
