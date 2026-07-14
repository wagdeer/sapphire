#include <sapphire/imu_init.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <spdlog/spdlog.h>

namespace sapphire {

ImuInitializer::ImuInitializer(const ImuInitConfig& config)
    : cfg_(config)
{
    // Reserve for 5s at 200Hz = 1000 samples — single allocation
    constexpr int kExpectedMaxSamples = 1200;
    samples_.reserve(kExpectedMaxSamples);
}

void ImuInitializer::reset() {
    state_ = State::WAITING;
    samples_.clear();
    start_stamp_ = 0.0;
    last_check_stamp_ = 0.0;
}

std::optional<ImuInitializer::Result>
ImuInitializer::feedImu(double stamp,
                         const Eigen::Vector3d& accel,
                         const Eigen::Vector3d& gyro) {
    if (state_ == State::CONVERGED || state_ == State::TIMEOUT) {
        return std::nullopt;
    }
    if (!std::isfinite(stamp)
        || !accel.allFinite()
        || !gyro.allFinite()) {
        spdlog::warn("[imu_init] dropping sample with non-finite values");
        return std::nullopt;
    }
    if (!samples_.empty() && stamp <= samples_.back().stamp) {
        spdlog::warn(
            "[imu_init] dropping non-monotonic timestamp: "
            "current={:.9f}, previous={:.9f}",
            stamp,
            samples_.back().stamp);
        return std::nullopt;
    }

    if (samples_.empty()) {
        start_stamp_ = stamp;
        last_check_stamp_ = stamp;
        state_ = State::CALIBRATING;
        std::fprintf(stderr, "[imu_init] calibrating...\n");
    }

    samples_.push_back({stamp, accel, gyro});

    if (static_cast<int>(samples_.size()) < cfg_.min_samples) {
        return std::nullopt;
    }

    double elapsed = stamp - start_stamp_;
    double since_last_check = stamp - last_check_stamp_;

    if (since_last_check >= cfg_.check_interval_sec) {
        last_check_stamp_ = stamp;

        if (checkConvergence()) {
            state_ = State::CONVERGED;
            std::fprintf(stderr, "\n");
            return computeResult();
        }
    }

    if (elapsed >= cfg_.timeout_sec) {
        state_ = State::TIMEOUT;
        std::fprintf(stderr, "\n");
        spdlog::warn("[imu_init] timed out after {:.1f}s, using approximate result", elapsed);
        return computeResult();
    }

    return std::nullopt;
}

double ImuInitializer::computeMedian(std::vector<double>& v) {
    if (v.empty()) return 0.0;
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + static_cast<long>(n), v.end());
    return v[n];
}

void ImuInitializer::rejectOutliers() {
    if (samples_.size() < 10) return;

    constexpr double kMADThreshold = 5.0;
    constexpr double kMADScale    = 0.6745;  // σ = MAD / Φ^(-1)(3/4) for normal distribution

    std::vector<double> gyro_x, gyro_y, gyro_z;
    std::vector<double> accel_x, accel_y, accel_z;
    gyro_x.reserve(samples_.size());
    gyro_y.reserve(samples_.size());
    gyro_z.reserve(samples_.size());
    accel_x.reserve(samples_.size());
    accel_y.reserve(samples_.size());
    accel_z.reserve(samples_.size());

    for (const auto& s : samples_) {
        gyro_x.push_back(s.gyro.x());
        gyro_y.push_back(s.gyro.y());
        gyro_z.push_back(s.gyro.z());
        accel_x.push_back(s.accel.x());
        accel_y.push_back(s.accel.y());
        accel_z.push_back(s.accel.z());
    }

    double med_gx = computeMedian(gyro_x);
    double med_gy = computeMedian(gyro_y);
    double med_gz = computeMedian(gyro_z);
    double med_ax = computeMedian(accel_x);
    double med_ay = computeMedian(accel_y);
    double med_az = computeMedian(accel_z);

    auto mad = [](std::vector<double>& v, double median) {
        for (auto& x : v) x = std::abs(x - median);
        return computeMedian(v) / kMADScale;
    };

    double mad_gx = mad(gyro_x, med_gx);
    double mad_gy = mad(gyro_y, med_gy);
    double mad_gz = mad(gyro_z, med_gz);
    double mad_ax = mad(accel_x, med_ax);
    double mad_ay = mad(accel_y, med_ay);
    double mad_az = mad(accel_z, med_az);

    std::vector<Sample> filtered;
    filtered.reserve(samples_.size());
    int rejected = 0;
    for (const auto& s : samples_) {
        bool outlier = false;
        outlier |= (std::abs(s.gyro.x() - med_gx) > kMADThreshold * mad_gx);
        outlier |= (std::abs(s.gyro.y() - med_gy) > kMADThreshold * mad_gy);
        outlier |= (std::abs(s.gyro.z() - med_gz) > kMADThreshold * mad_gz);
        outlier |= (std::abs(s.accel.x() - med_ax) > kMADThreshold * mad_ax);
        outlier |= (std::abs(s.accel.y() - med_ay) > kMADThreshold * mad_ay);
        outlier |= (std::abs(s.accel.z() - med_az) > kMADThreshold * mad_az);
        if (!outlier) {
            filtered.push_back(s);
        } else {
            ++rejected;
        }
    }

    samples_.swap(filtered);
}

bool ImuInitializer::checkConvergence() {
    rejectOutliers();

    if (samples_.size() < static_cast<size_t>(cfg_.min_samples)) {
        return false;
    }

    Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
    for (const auto& s : samples_) {
        gyro_sum += s.gyro;
        accel_sum += s.accel;
    }
    Eigen::Vector3d gyro_mean = gyro_sum / static_cast<double>(samples_.size());
    Eigen::Vector3d accel_mean = accel_sum / static_cast<double>(samples_.size());

    Eigen::Vector3d gyro_var = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_var = Eigen::Vector3d::Zero();
    for (const auto& s : samples_) {
        Eigen::Vector3d dg = s.gyro - gyro_mean;
        Eigen::Vector3d da = s.accel - accel_mean;
        gyro_var += dg.cwiseProduct(dg);
        accel_var += da.cwiseProduct(da);
    }
    double n = static_cast<double>(samples_.size());
    double gyro_std  = std::sqrt((gyro_var.x() + gyro_var.y() + gyro_var.z()) / n);
    double accel_std = std::sqrt((accel_var.x() + accel_var.y() + accel_var.z()) / n);

    double elapsed = samples_.back().stamp - start_stamp_;

    std::fprintf(stderr,
        "\r[imu_init] %.1fs | %zu samples | "
        "gyro_σ=%.4f/%.4f  accel_σ=%.3f/%.3f  ",
        elapsed, samples_.size(),
        gyro_std, cfg_.convergence_gyro_std,
        accel_std, cfg_.convergence_accel_std);
    std::fflush(stderr);

    return gyro_std < cfg_.convergence_gyro_std
        && accel_std < cfg_.convergence_accel_std;
}

ImuInitializer::Result ImuInitializer::computeResult() {
    rejectOutliers();

    Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
    for (const auto& s : samples_) {
        gyro_sum += s.gyro;
        accel_sum += s.accel;
    }
    double n = static_cast<double>(samples_.size());
    Eigen::Vector3d gyro_mean = gyro_sum / n;
    Eigen::Vector3d accel_mean = accel_sum / n;

    Eigen::Vector3d gyro_var = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_var = Eigen::Vector3d::Zero();
    for (const auto& s : samples_) {
        Eigen::Vector3d dg = s.gyro - gyro_mean;
        Eigen::Vector3d da = s.accel - accel_mean;
        gyro_var += dg.cwiseProduct(dg);
        accel_var += da.cwiseProduct(da);
    }
    double gyro_std  = std::sqrt((gyro_var.x() + gyro_var.y() + gyro_var.z()) / n);
    double accel_std = std::sqrt((accel_var.x() + accel_var.y() + accel_var.z()) / n);

    // ── Gravity alignment: IMU accel direction → world Z-up ────
    //    At rest, accel measures support force = −g.
    //    FromTwoVectors maps measured-up to world-up, giving R_imu_world.
    //    q_gravity = R_imu_world expressed as quaternion.
    Eigen::Vector3d grav_imu = accel_mean.normalized();
    Eigen::Vector3d grav_world(0.0, 0.0, 1.0);  // world Z = up
    Eigen::Quaterniond q_gravity =
        Eigen::Quaterniond::FromTwoVectors(grav_imu, grav_world);
    q_gravity.normalize();

    // Bias = mean(meas) − gravity_vector_in_imu_frame
    // Gravity in IMU frame = q_gravity.inverse() * (0,0,g_mag)
    //   = points downward in IMU coordinates
    Eigen::Vector3d grav_in_imu =
        q_gravity.inverse() * Eigen::Vector3d(0, 0, cfg_.gravity_mag);
    Eigen::Vector3d accel_bias = accel_mean - grav_in_imu;
    Eigen::Vector3d gyro_bias = gyro_mean;

    double elapsed = samples_.back().stamp - start_stamp_;

    spdlog::info(
        "[imu_init] calibration complete: {:.2f}s, {} samples, "
        "gyro_bias=[{:.6f},{:.6f},{:.6f}], "
        "accel_bias=[{:.6f},{:.6f},{:.6f}], "
        "gyro_std={:.6f}, accel_std={:.6f}",
        elapsed,
        static_cast<int>(samples_.size()),
        gyro_bias.x(), gyro_bias.y(), gyro_bias.z(),
        accel_bias.x(), accel_bias.y(), accel_bias.z(),
        gyro_std, accel_std);

    return Result{
        .gyro_bias   = gyro_bias,
        .accel_bias  = accel_bias,
        .q_gravity   = q_gravity,
        .gyro_std    = gyro_std,
        .accel_std   = accel_std,
        .duration_sec = elapsed,
        .sample_count = static_cast<int>(samples_.size()),
        .final_state  = state_,
    };
}

}  // namespace sapphire
