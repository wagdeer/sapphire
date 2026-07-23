# Borrowing from RKO-LIO: Top-3 Features for Sapphire

> **Source:** [PRBonn/rko_lio](https://github.com/PRBonn/rko_lio) — Meher Malladi, Stachniss Lab, IEEE RA-L 2026.
> RKO-LIO is a minimal LIO frontend (~500 lines C++ core) that achieves robust odometry
> across platforms **without an ESKF**, using three decoupled IMU contributions.
>
> This document selects the three features that yield the highest ROI for Sapphire,
> with full derivations and justification of why each is compatible with Sapphire's
> existing Gal(3) ESKF architecture.

---

## Top-1: ICP Orientation Regularization

### 1.1 Motivation

In degenerate geometries (long corridors, tunnels, open plains), LiDAR scan matching
loses observability on roll and pitch. The GICP Hessian becomes rank-deficient along
those rotational axes, and the ESKF's Kalman gain approaches zero — the filter trusts
the IMU prior exclusively. If the IMU bias estimate drifts, the orientation slowly tilts
with no LiDAR correction to pull it back.

RKO-LIO solves this **not** with a full filter, but by adding a soft gravity-alignment
constraint directly into the ICP linear system. The constraint strength adapts to the
platform's motion: strong when stationary (gravity is measurable), weak during aggressive
maneuvers (gravity is confounded with body acceleration).

### 1.2 Mechanism Overview

```
┌──────────────┐     ┌──────────────────┐     ┌────────────────────┐
│ IMU samples   │────▶│ Body Accel KF    │────▶│ local_gravity      │
│ (between      │     │ (3-state,        │     │ estimate (body     │
│  LiDAR scans) │     │  jerk-driven)    │     │  frame, points up) │
└──────────────┘     └──────────────────┘     └────────┬───────────┘
                                                       │
                              ┌────────────────────────┘
                              │
                              ▼
                    ┌────────────────────┐
                    │ beta = min_beta ×  │
                    │ (1 + accel_var)    │
                    └────────┬───────────┘
                             │
                             ▼
              ┌──────────────────────────────┐
              │  H_total = H_gicp + H_ori/β  │
              │  b_total = b_gicp + b_ori/β  │
              └──────────────────────────────┘
```

### 1.3 Body Acceleration Kalman Filter

#### State and Assumptions

The filter estimates a single 3-dimensional quantity: the platform's body-frame
acceleration not due to gravity.

$$
\mathbf{x} = \mathbf{a}_{\text{body}} \in \mathbb{R}^3
$$

We model the acceleration as driven by jerk (derivative of acceleration), which is
physically bounded by the platform's actuator limits. The jerk is assumed uniformly
distributed within $[-j_{\text{max}}, +j_{\text{max}}]$.

#### Prediction Step

The process noise covariance follows from the uniform jerk assumption:

$$
Q = \frac{(j_{\text{max}} \cdot \Delta t)^2}{3} \cdot \mathbf{I}_3
$$

Derivation: for a uniform distribution on $[-a, a]$, the variance is $(2a)^2 / 12 = a^2/3$.
Here $a = j_{\text{max}} \cdot \Delta t$ (jerk integrated over the interval).

The predicted covariance is:

$$
P^- = P + Q
$$

#### Observation

At the end of each LiDAR interval, we have:

- $\bar{\mathbf{a}}_{\text{imu}}$: mean of bias-corrected IMU accelerometer readings
- $\mathbf{R}_{\text{wb}}$: current rotation estimate (body → world)

The body acceleration observation is obtained by removing the expected gravity component:

$$
\mathbf{z} = \bar{\mathbf{a}}_{\text{imu}} + \mathbf{R}_{\text{wb}}^T \cdot \mathbf{g}
$$

where $\mathbf{g} = (0, 0, -9.8107)$ in world frame. Note: $\mathbf{R}^T\mathbf{g}$ gives
gravity expressed in body frame; adding it to the IMU reading cancels the static gravity
component, leaving the true kinematic acceleration.

#### Measurement Noise

The measurement noise is derived from the acceleration magnitude variance computed
via Welford's online algorithm across the IMU samples in the interval:

$$
R = \frac{\sigma^2_{\lVert\mathbf{a}\rVert}}{3} \cdot \mathbf{I}_3
$$

We use an isotropic approximation (uniformly distributing the scalar magnitude variance
across three axes). This is conservative — the true acceleration covariance may be
anisotropic, but the isotropic assumption is simple and works well in practice.

#### Update Step

Standard KF update:

$$
\begin{aligned}
\mathbf{S} &= P^- + R \\
\mathbf{K} &= P^- \mathbf{S}^{-1} \\
\hat{\mathbf{x}} &= \mathbf{x}^- + \mathbf{K}(\mathbf{z} - \mathbf{x}^-) \\
P &= P^- - \mathbf{K} P^-
\end{aligned}
$$

#### Output

The key output is the **local gravity estimate** — our best guess of which way is "up"
in body frame:

$$
\hat{\mathbf{g}}_{\text{local}} = \bar{\mathbf{a}}_{\text{imu}} - \hat{\mathbf{a}}_{\text{body}}
$$

Intuition: total acceleration measured by IMU, minus estimated body motion, equals gravity.
This vector points **upward** in body frame (opposite to world-frame gravity direction).

### 1.4 Adaptive Regularization Weight β

The acceleration magnitude variance $\sigma^2_{\lVert\mathbf{a}\rVert}$ (from Welford)
directly measures how much the platform is maneuvering:

| Condition | $\sigma^2$ | β | Constraint |
|-----------|-----------|-----|------------|
| Stationary / constant velocity | ~0 | $\approx \beta_{\text{min}}$ | Strong |
| Moderate acceleration | medium | medium | Moderate |
| Aggressive maneuver | large | large | Weak |

$$
\beta = \beta_{\text{min}} \cdot \left(1 + \sigma^2_{\lVert\mathbf{a}\rVert}\right)
$$

where $\beta_{\text{min}}$ is a configurable parameter (RKO-LIO default: 200).
Set $\beta_{\text{min}} = -1$ to disable the constraint entirely.

**Why this works:** when the platform is maneuvering, the IMU reading is a mixture of
gravity and body acceleration — you cannot disentangle them without additional information.
The filter relaxes the gravity constraint exactly when the gravity estimate is unreliable.

### 1.5 Orientation Residual in ICP

#### Predicted Gravity Direction

Given the current ICP rotation estimate $\mathbf{R}$, the predicted gravity direction
in body frame is:

$$
\mathbf{g}_{\text{pred}} = \mathbf{R}^T \cdot (-\mathbf{g})
$$

Both $\mathbf{g}_{\text{pred}}$ and $\hat{\mathbf{g}}_{\text{local}}$ point **upward**
in body frame.

#### Residual

$$
\mathbf{r}_{\text{ori}} = \mathbf{g}_{\text{pred}} - \hat{\mathbf{g}}_{\text{local}} \in \mathbb{R}^3
$$

#### Jacobian (with respect to SE(3) perturbation $\delta\xi = [\delta\mathbf{t}, \delta\boldsymbol{\theta}]$)

Only the orientation component is non-zero (gravity is translation-invariant):

$$
\mathbf{J}_{\text{ori}} = \begin{bmatrix} \mathbf{0}_{3\times3} & \mathbf{R}^T \cdot [-\mathbf{g}]_\times \end{bmatrix} \in \mathbb{R}^{3\times6}
$$

where $[-\mathbf{g}]_\times$ is the skew-symmetric matrix:

$$
[-\mathbf{g}]_\times = \begin{bmatrix}
0 & g & 0 \\
-g & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
\quad\text{for}\quad \mathbf{g} = (0, 0, -g)
$$

**Crucially**, the third row and column are zero — the yaw axis is unconstrained.
Only roll (rotation around x) and pitch (rotation around y) are regularized.

#### Augmented Linear System

The standard ICP linear system is:

$$
\mathbf{H}_{\text{icp}} \cdot \Delta\xi = -\mathbf{b}_{\text{icp}}
$$

We augment it with the orientation constraint:

$$
\begin{aligned}
\mathbf{H}_{\text{total}} &= \mathbf{H}_{\text{icp}} + \frac{1}{\beta}\mathbf{J}_{\text{ori}}^T\mathbf{J}_{\text{ori}} \\
\mathbf{b}_{\text{total}} &= \mathbf{b}_{\text{icp}} + \frac{1}{\beta}\mathbf{J}_{\text{ori}}^T\mathbf{r}_{\text{ori}}
\end{aligned}
$$

Then solve $\mathbf{H}_{\text{total}} \cdot \Delta\xi = -\mathbf{b}_{\text{total}}$ as usual.

### 1.6 Interaction with Sapphire's ESKF

**This constraint lives entirely inside the ICP optimization loop.** It does not touch
the ESKF prediction, the ESKF correction, or the Gal(3) integrator.

```
                 ┌──────────┐
  IMU ──────────▶│  ESKF    │────▶ prior state + covariance
                 │ predict  │
                 └──────────┘
                       │
                       ▼
                 ┌──────────┐     ┌───────────────────┐
  LiDAR ────────▶│ GICP     │────▶│ H_gicp + H_ori/β │
                 │ register │     │ b_gicp + b_ori/β │
                 └──────────┘     └────────┬──────────┘
                                           │
                                           ▼
                 ┌──────────┐     ┌───────────────────┐
                 │  ESKF    │◀────│ fused pose +      │
                 │ update   │     │ Hessian           │
                 └──────────┘     └───────────────────┘
```

Why they coexist without conflict:

- **Normal geometry:** GICP Hessian is well-conditioned → H_gicp dominates → constraint has negligible effect
- **Degenerate geometry:** GICP Hessian is rank-deficient → H_ori/β provides the missing roll/pitch information
- **High acceleration:** β is large → constraint is weak → ICP geometry still carries the day (or both fail gracefully)
- **Static:** β is small → constraint is strong → orientation is locked (redundant with ESKF gravity alignment, but harmless)

### 1.7 Implementation Notes for Sapphire

The main challenge is that Sapphire's ESKF `predict()` consumes IMU samples directly
without accumulating interval statistics. To feed the body acceleration KF, we need:

1. **Accumulate per-interval statistics during ESKF prediction:**
   Add an `IntervalStats` member (see top-3 below) that tracks within the `predict()`
   loop: sum of unbiased accel, sum of unbiased gyro, and Welford accumulators for
   acceleration magnitude.

2. **Reset after each LiDAR correction:**
   After ESKF `correct()`, read the accumulated statistics, run the body accel KF,
   store the resulting $\hat{\mathbf{g}}_{\text{local}}$ and $\sigma^2$, then reset
   the accumulator.

3. **Inject into GICP:**
   In the registration wrapper, after constructing the GICP linear system, augment
   with the orientation term using the stored values.

4. **Configuration:**
   ```toml
   [odometry.orientation_regularization]
   enabled = true
   min_beta = 200.0          # -1 to disable
   max_expected_jerk = 3.0   # m/s³, platform-specific
   ```

### 1.8 Why This Over Pure ESKF

| Approach | Degenerate geometry | Bias drift | Complexity |
|----------|-------------------|------------|------------|
| ESKF alone | Kalman gain → 0, trusts IMU prior | If bias drifts, orientation drifts | High (15-state) |
| ESKF + ICP constraint | Gravity prior directly in optimization | Bias compensated by body accel KF | High + ~80 loc |

The ICP constraint is a **defense-in-depth** measure. The ESKF handles the nominal case;
the constraint prevents catastrophic failure in the edge case. It costs little code and
zero additional runtime overhead beyond a 3×3 KF update per LiDAR scan.

---

## Top-2: IMU-Rate Odometry Publishing

### 2.1 Motivation

LiDAR odometry publishes at scan rate (typically 10 Hz). Downstream consumers
(controllers, localization, visualization) benefit from higher-rate pose estimates.
IMU-rate odometry (typically 200 Hz) provides smooth, low-latency poses between
LiDAR corrections.

This is the feature the Sapphire author [requested](https://github.com/PRBonn/rko_lio/issues)
RKO-LIO to add, which was subsequently implemented as `OnlineImuRateNode`.

### 2.2 Mechanism

Sapphire's ESKF already maintains the necessary state. After each LiDAR correction,
`replayToLatest()` propagates the nominal state through all buffered IMU samples to
produce `tip_state_` — the best estimate at the most recent IMU timestamp.

Between LiDAR frames, each IMU callback calls `predict()` which advances the tip state.
This tip state is exactly what the IMU-rate odometry message should contain.

### 2.3 Implementation

In the ROS2 wrapper's IMU callback (pseudocode):

```cpp
void imu_callback(const ImuMsg& msg) {
    ImuData data = convert(msg);
    eskf_.predict(data);          // advances tip_state_

    if (!eskf_.initialized()) return;  // no valid pose yet

    auto odom_msg = make_odom_msg(
        eskf_.tipState(),          // pose + velocity + angular_velocity
        msg.header.stamp,
        odom_frame, base_frame
    );
    imu_rate_odom_pub_->publish(odom_msg);
}
```

### 2.4 Topic Convention

| Topic | Rate | Content | Use Case |
|-------|------|---------|----------|
| `/odom` | LiDAR rate (~10 Hz) | ESKF corrected pose | High-accuracy localization |
| `/odom_imu_rate` | IMU rate (~200 Hz) | ESKF propagated pose | Low-latency control, smooth viz |

Both share the same `odom → base_link` TF tree convention.

### 2.5 Cost

- **Code:** ~20 lines in the ROS2 wrapper
- **Compute:** zero — `predict()` already runs, `make_odom_msg()` is a struct fill
- **Bandwidth:** ~50 KB/s additional (nav_msgs/Odometry at 200 Hz)
- **Risk:** none — the `tip_state_` already exists and is maintained

---

## Top-3: IntervalStats with Welford Variance

### 3.1 Motivation

Between two LiDAR scans (typically 100 ms), an IMU at 200 Hz produces ~20 samples.
Computing statistics over this window (mean angular velocity, mean acceleration,
acceleration magnitude variance) is useful for:

1. **Body acceleration KF** (top-1) — needs mean accel and magnitude variance
2. **Motion classification** — detect stationary vs moving vs aggressive maneuver
3. **ESKF noise adaptation** — inflate process noise during high dynamics
4. **Degeneracy detection** — low angular velocity means no rotational excitation

A naive approach stores all samples and computes stats at the end of the interval.
Welford's online algorithm computes mean and variance in a single pass with O(1)
memory.

### 3.2 Welford's Online Algorithm

For a sequence of scalar values $x_1, x_2, \ldots, x_n$:

$$
\begin{aligned}
\bar{x}_k &= \bar{x}_{k-1} + \frac{x_k - \bar{x}_{k-1}}{k} \\
M_{2,k} &= M_{2,k-1} + (x_k - \bar{x}_{k-1})(x_k - \bar{x}_k)
\end{aligned}
$$

where $\bar{x}_k$ is the running mean after $k$ samples and $M_{2,k}$ is the running
sum of squared differences from the mean.

The population variance is:

$$
\sigma^2 = \frac{M_{2,n}}{n-1} \quad (n \ge 2)
$$

### 3.3 Sapphire-Specific Struct

```cpp
struct IntervalStats {
    int imu_count = 0;

    // Running sums (for means)
    Eigen::Vector3d angular_velocity_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_acceleration_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu_acceleration_sum = Eigen::Vector3d::Zero();

    // Welford accumulators for acceleration magnitude
    double accel_mag_mean = 0.0;
    double welford_m2 = 0.0;

    void update(const Eigen::Vector3d& unbiased_ang_vel,
                const Eigen::Vector3d& unbiased_accel,
                const Eigen::Vector3d& compensated_accel) {
        ++imu_count;
        angular_velocity_sum += unbiased_ang_vel;
        imu_acceleration_sum += unbiased_accel;
        body_acceleration_sum += compensated_accel;

        // Welford update for ||accel||
        const double norm = unbiased_accel.norm();
        const double prev_mean = accel_mag_mean;
        accel_mag_mean += (norm - prev_mean) / imu_count;
        welford_m2 += (norm - prev_mean) * (norm - accel_mag_mean);
    }

    [[nodiscard]] double accel_magnitude_variance() const {
        return (imu_count >= 2) ? welford_m2 / (imu_count - 1) : 0.0;
    }

    void reset() { *this = IntervalStats{}; }
};
```

### 3.4 Integration into ESKF predict()

The `predict()` loop already iterates over IMU samples. Adding the accumulator is a
one-liner per sample:

```cpp
void ESKF::predict(const ImuData& imu) {
    // ... existing Gal(3) integration + covariance propagation ...

    const Eigen::Vector3d unbiased_ang = imu.gyro - bg_;
    const Eigen::Vector3d unbiased_acc = imu.accel - ba_;
    const Eigen::Vector3d local_gravity = R_.inverse() * gravity;
    const Eigen::Vector3d compensated = unbiased_acc + local_gravity;

    interval_stats_.update(unbiased_ang, unbiased_acc, compensated);
}
```

### 3.5 Downstream Consumers

| Consumer | Uses | Field |
|----------|------|-------|
| Body Accel KF | mean accel + magnitude variance | `imu_acceleration_sum / imu_count`, `accel_magnitude_variance()` |
| Motion classifier | angular velocity magnitude | `angular_velocity_sum / imu_count` |
| ESKF noise scaling | acceleration variance | `accel_magnitude_variance()` |
| Deskew validation | IMU count sanity check | `imu_count` |

### 3.6 Why Welford Over Naive

| Method | Memory | Passes | Numerical stability |
|--------|--------|--------|-------------------|
| Store all samples, compute at end | O(N) | 2 | Good |
| Naive sum-of-squares | O(1) | 1 | Catastrophic cancellation |
| Welford | O(1) | 1 | Excellent |

At 200 Hz IMU and 10 Hz LiDAR, N=20 per interval — small enough that naive would work.
But Welford costs nothing extra and generalizes to arbitrary interval lengths.

---

## Summary

| # | Feature | Lines | Touches | Value |
|---|---------|-------|---------|-------|
| 1 | ICP orientation regularization | ~80 | registration.cpp, eskf.hpp, config | Robustness in degenerate geometry |
| 2 | IMU-rate odometry | ~20 | ros2 wrapper | Low-latency pose for consumers |
| 3 | IntervalStats + Welford | ~30 | eskf.hpp/cpp | Enables top-1, reusable utility |

All three are independent, can be implemented incrementally, and do not alter Sapphire's
core ESKF/GICP architecture. Features #2 and #3 are prerequisites or enablers for #1.

---

*Analysis by 米西, 2026-07-22. Based on deep reading of RKO-LIO v0.3.1 source.*
*Source repo: /home/bindeer/git/rko_lio*
