# ESKF on Gal(3) for Sapphire Frontend LIO

> 完整的 Error-State Kalman Filter 推导，基于 Gal(3) 等变预积分框架。
> 目标：替换当前的 Geometric Observer，用协方差感知的滤波融合 IMU 预积分与 ICP 位姿测量。
>
> **Document status (v1.1.1):** Approach A 为推荐实现路径。相对 v1.0 已修正：
> Gal(3) 预积分增量分量顺序、离散过程噪声 $Q_d$ 的 $\Delta t$ 方向、
> ICP 观测残差与测量雅可比的坐标系一致性。Approach B（完整 TG 协方差映射）
> 标为 *future / unverified*，暂不作为实现依据。启用 ESKF 时须在配置里打开非零 bias random walk。

---

## Notation

| Symbol | Meaning |
|--------|---------|
| $\chi = (R, v, p, s) \in \text{Gal}(3)$ | Group element: rotation $R \in SO(3)$, velocity $v \in \mathbb{R}^3$, position $p \in \mathbb{R}^3$, time $s$ |
| $\mathfrak{gal}(3) \simeq \mathbb{R}^{10}$ | Lie algebra, element $u = (\omega, \alpha, a, \tau)$ |
| $\bar{x}$ | Nominal state (on manifold) |
| $\delta x$ | Error state (in tangent space, Euclidean) |
| $\hat{x}$ | Estimated / corrected state |
| $(\cdot)^\wedge$ | Wedge: $\mathbb{R}^{10} \to \mathfrak{gal}(3)$ (vector to matrix) |
| $(\cdot)^\vee$ | Vee: $\mathfrak{gal}(3) \to \mathbb{R}^{10}$ (matrix to vector) |
| $\exp(\cdot)$ | Exponential map $\mathfrak{g} \to G$ |
| $\log(\cdot)$ | Logarithmic map $G \to \mathfrak{g}$ |
| $\text{Ad}_\chi$ | Adjoint representation of $\chi$ on $\mathfrak{g}$ |
| $\text{J}_l(u)$ | Left Jacobian of the group at $u \in \mathfrak{g}$ |
| $(\cdot)_\times$ | Skew-symmetric matrix |
| $\times$ | Cross product |
| $\Delta t$ | Time interval |
| $g_w$ | Gravity vector in world frame: $(0, 0, -9.80665)$ |

Frame conventions:
- World frame `w`: gravity-aligned, z-up
- Body/IMU frame `b`: sensor frame
- $R_{wb}$ maps vectors from body to world: $v_w = R_{wb} v_b$

Lie-algebra vector ordering (matches `lie::Gal3` / `MeanOnlyGal3Integrator`):

$$
u = \begin{bmatrix}\omega \\ \alpha \\ a \\ \tau\end{bmatrix}
\in \mathbb{R}^{10}
\quad\text{with}\quad
u^\wedge =
\begin{bmatrix}
\omega_\times & \alpha & a \\
0 & 0 & \tau \\
0 & 0 & 0
\end{bmatrix}
$$

For IMU mean preintegration the increment uses $\alpha = a_{\text{body}}$, $a = 0$, $\tau = 1$.

---

## 1. Gal(3) Group Structure

### 1.1 Group Definition

Gal(3) is the Galilean group, represented as $5 \times 5$ matrices:

$$\chi = \begin{bmatrix} R & v & p \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} \in \text{Gal}(3) \subset \mathbb{R}^{5\times5}$$

where $R \in SO(3)$, $v \in \mathbb{R}^3$ (velocity), $p \in \mathbb{R}^3$ (position). The full group also stores a time scalar $s$ (matrix entry $(3,4)$), making it a 10-parameter group.

Group composition $\chi_1 \cdot \chi_2$ (matches `Gal3::operator*`):

$$R = R_1 R_2$$
$$v = R_1 v_2 + v_1$$
$$p = R_1 p_2 + v_1\, s_2 + p_1$$
$$s = s_1 + s_2$$

### 1.2 Lie Algebra $\mathfrak{gal}(3)$

An element $u \in \mathfrak{gal}(3) \simeq \mathbb{R}^{10}$ has the matrix form:

$$u^\wedge = \begin{bmatrix} \omega_\times & \alpha & a \\ 0 & 0 & \tau \\ 0 & 0 & 0 \end{bmatrix}$$

with components:
- $\omega \in \mathbb{R}^3$: angular velocity (SO(3) part)
- $\alpha \in \mathbb{R}^3$: velocity / specific-force channel (matrix column 3)
- $a \in \mathbb{R}^3$: position channel (matrix column 4)
- $\tau \in \mathbb{R}$: time increment

The wedge operator creates the $5\times5$ matrix:

```cpp
U.block<3,3>(0,0) = SO3::wedge(ω);   // [ω]×
U.block<3,1>(0,3) = α;
U.block<3,1>(0,4) = a;
U(3,4) = τ;
```

### 1.3 Exponential Map

$$\chi = \exp(u^\wedge) = \sum_{k=0}^\infty \frac{(u^\wedge)^k}{k!}$$

In closed form (from `Gal3::exp`):

$$R = \exp(\omega_\times) \in SO(3)$$
$$v = \text{J}_l^{SO(3)}(\omega) \cdot \alpha$$
$$p = \text{J}_l^{SO(3)}(\omega) \cdot a + \tau \cdot \Gamma_2^{SO(3)}(\omega) \cdot \alpha$$

where $\text{J}_l^{SO(3)}$ is the SO(3) left Jacobian and $\Gamma_2$ is a helper matrix.

### 1.4 Adjoint Representation

The Adjoint of $\chi = (R, v, p, s)$ acting on $\xi = (\omega, \alpha, a, \tau) \in \mathfrak{gal}(3)$ (matches `Gal3::Adjoint()`):

$$\text{Ad}_\chi \cdot \xi = \begin{bmatrix}
R & 0 & 0 & 0 \\
[v]_\times R & R & 0 & 0 \\
[p - s v]_\times R & -s R & R & v \\
0 & 0 & 0 & 1
\end{bmatrix}
\begin{bmatrix} \omega \\ \alpha \\ a \\ \tau \end{bmatrix}$$

The first nine components are the navigation algebra $(\omega, \alpha, a)$; the last component is time.

---

## 2. State Definition

### 2.1 Nominal State (15D)

$$\bar{x} = \begin{bmatrix} R_{wb} \\ v_w \\ p_w \\ b_a \\ b_g \end{bmatrix} \in SO(3) \times \mathbb{R}^{12}$$

- $R_{wb} \in SO(3)$: body-to-world rotation
- $v_w \in \mathbb{R}^3$: velocity in world frame
- $p_w \in \mathbb{R}^3$: position in world frame
- $b_a \in \mathbb{R}^3$: accelerometer bias
- $b_g \in \mathbb{R}^3$: gyroscope bias

### 2.2 Error State (15D, Euclidean)

$$\delta x = \begin{bmatrix} \delta\theta \\ \delta v \\ \delta p \\ \delta b_a \\ \delta b_g \end{bmatrix} \in \mathbb{R}^{15}$$

The true state relates to the nominal state via:

$$R_{wb} = \bar{R}_{wb} \cdot \exp(\delta\theta_\times) \quad \text{(right-multiplied error)}$$
$$v_w = \bar{v}_w + \delta v$$
$$p_w = \bar{p}_w + \delta p$$
$$b_a = \bar{b}_a + \delta b_a$$
$$b_g = \bar{b}_g + \delta b_g$$

We use the right-multiplied orientation error (local / body tangent), which is the common robotics ESKF convention (Solà). Position and velocity errors are additive in the world frame.

The $\boxplus$ operator maps the error state to the manifold:

$$\bar{x} \boxplus \delta x = \begin{bmatrix}
\bar{R} \cdot \exp(\delta\theta_\times) \\
\bar{v} + \delta v \\
\bar{p} + \delta p \\
\bar{b}_a + \delta b_a \\
\bar{b}_g + \delta b_g
\end{bmatrix}$$

The $\boxminus$ operator extracts the error:

$$\delta x = \hat{x} \boxminus \bar{x} = \begin{bmatrix}
\log(\bar{R}^T \hat{R})^\vee \\
\hat{v} - \bar{v} \\
\hat{p} - \bar{p} \\
\hat{b}_a - \bar{b}_a \\
\hat{b}_g - \bar{b}_g
\end{bmatrix}$$

### 2.3 Error-State Covariance

$$P = \mathbb{E}[\delta x \cdot \delta x^T] \in \mathbb{R}^{15 \times 15}$$

Initialized based on IMU stationary calibration uncertainty:
- $\sigma_\theta \approx 1^\circ$ (gravity alignment residual)
- $\sigma_v = 0.01$ m/s (assumed stationary)
- $\sigma_p = 0.01$ m (assumed at origin)
- $\sigma_{b_a}, \sigma_{b_g}$ from IMU initialization variance estimates

---

## 3. IMU Kinematic Model

### 3.1 Measurement Model

$$\omega_m(t) = \omega_b(t) + b_g(t) + \eta_g(t)$$
$$a_m(t) = R_{wb}^T(t) \left( a_w(t) - g_w \right) + b_a(t) + \eta_a(t)$$

where:
- $\omega_m, a_m$: measured angular velocity and specific force (in body frame)
- $\omega_b$: true angular velocity (body frame)
- $a_w$: true kinematic acceleration in world ($\dot{v}_w$)
- $g_w = (0, 0, -g)$: gravity vector (world frame)
- Noise: $\eta_g \sim \mathcal{N}(0, \sigma_g^2 I)$, $\eta_a \sim \mathcal{N}(0, \sigma_a^2 I)$

With this convention, specific force satisfies $f_b = R_{wb}^T(a_w - g_w)$, and the navigation equations become:

$$\dot{v}_w = R_{wb}(a_m - b_a) + g_w$$

### 3.2 Bias Model

Biases are modeled as random walks:

$$\dot{b}_a(t) = \eta_{ba}(t), \quad \eta_{ba} \sim \mathcal{N}(0, \sigma_{ba}^2 I)$$
$$\dot{b}_g(t) = \eta_{bg}(t), \quad \eta_{bg} \sim \mathcal{N}(0, \sigma_{bg}^2 I)$$

**Bias random walk (implementation note):**
current `ImuNoiseConfig` defaults are

```cpp
accel_bias_rw_sigma = 0.0;
gyro_bias_rw_sigma  = 0.0;
```

If ESKF builds $Q_c$ from these zeros, the bias covariance blocks never grow, so the filter effectively **does not estimate bias online** and degenerates toward a fixed-bias propagator plus pose-only corrections. That is worse than the geometric observer, which at least applies heuristic bias gains.

For ESKF bring-up, set these in the TOML / runtime config to the DLIO defaults in §9.1:

- `accel_bias_rw_sigma = 1.65e-4`  (m/s²·√s)
- `gyro_bias_rw_sigma  = 6.55e-5`  (rad/s·√s)

No change to the `ImuNoiseConfig` struct itself is required — only non-zero config values when `odometry.fusion = "eskf"`.

### 3.3 Nominal State Kinematics

The nominal state evolves continuously as:

$$\dot{\bar{R}}_{wb} = \bar{R}_{wb} \cdot (\omega_m - \bar{b}_g)_\times$$
$$\dot{\bar{v}}_w = \bar{R}_{wb} \cdot (a_m - \bar{b}_a) + g_w$$
$$\dot{\bar{p}}_w = \bar{v}_w$$
$$\dot{\bar{b}}_a = 0$$
$$\dot{\bar{b}}_g = 0$$

### 3.4 Error State Kinematics (Linearized)

For the right-multiplied orientation error above:

$$\dot{\delta\theta} = -(\omega_m - \bar{b}_g)_\times \cdot \delta\theta - \delta b_g - \eta_g$$
$$\dot{\delta v} = -\bar{R}_{wb} \cdot (a_m - \bar{b}_a)_\times \cdot \delta\theta - \bar{R}_{wb} \cdot \delta b_a - \bar{R}_{wb} \cdot \eta_a$$
$$\dot{\delta p} = \delta v$$
$$\dot{\delta b_a} = \eta_{ba}$$
$$\dot{\delta b_g} = \eta_{bg}$$

In matrix form $\dot{\delta x} = F \cdot \delta x + G \cdot w$:

$$F = \begin{bmatrix}
-[\omega_m - \bar{b}_g]_\times & 0 & 0 & 0 & -I_3 \\
-\bar{R}_{wb} [a_m - \bar{b}_a]_\times & 0 & 0 & -\bar{R}_{wb} & 0 \\
0 & I_3 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0
\end{bmatrix}_{15\times15}$$

$$G = \begin{bmatrix}
-I_3 & 0 & 0 & 0 \\
0 & -\bar{R}_{wb} & 0 & 0 \\
0 & 0 & 0 & 0 \\
0 & 0 & I_3 & 0 \\
0 & 0 & 0 & I_3
\end{bmatrix}_{15\times12}$$

$$w = \begin{bmatrix} \eta_g \\ \eta_a \\ \eta_{ba} \\ \eta_{bg} \end{bmatrix} \sim \mathcal{N}(0, Q_c), \quad Q_c = \begin{bmatrix}
\sigma_g^2 I_3 & 0 & 0 & 0 \\
0 & \sigma_a^2 I_3 & 0 & 0 \\
0 & 0 & \sigma_{ba}^2 I_3 & 0 \\
0 & 0 & 0 & \sigma_{bg}^2 I_3
\end{bmatrix}$$

Here $Q_c$ is a continuous-time power spectral density (PSD). Units match the noise densities in §9.1 (`/√Hz` and `/√s`).

---

## 4. ESKF Prediction Step

There are **two approaches** for the prediction step. **Only Approach A is approved for the first implementation.**

### 4.1 Approach A: Hybrid (Gal(3) Nominal + Linearized Covariance) — recommended

Use Gal(3) preintegration for the nominal state (as Sapphire already does), and use the standard linearized error dynamics for covariance propagation.

#### Nominal State Propagation (same as current `propagateStateLocked`)

For each IMU sample at ~200 Hz between LiDAR corrections:

1. Gal(3) preintegration (mean only), matching `MeanOnlyGal3Integrator`:

$$
u_k =
\begin{bmatrix}
\omega_m - \bar{b}_g \\
a_m - \bar{b}_a \\
0_{3\times1} \\
1
\end{bmatrix}
=
\begin{bmatrix}\omega \\ \alpha \\ a \\ \tau\end{bmatrix},
\quad
\Delta\Upsilon_k = \exp\!\big(u_k^\wedge \cdot \Delta t_k\big)
$$

$$\Upsilon \leftarrow \Upsilon \cdot \Delta\Upsilon_k$$

> Critical: $\alpha$ receives the bias-corrected specific force; the position algebra channel $a$ is zero. Swapping $\alpha$ and $a$ breaks mean propagation.
>
> Note on v1.0: the earlier draft wrote $[\omega;\;0;\;a_{\text{body}};\;1]$. That was a **documentation** error. Sapphire's current `MeanOnlyGal3Integrator` already uses the correct ordering; do not "fix" the mean path. The swap would mainly poison an ESKF / preintegration-covariance derivation that copied the wrong $u$.

2. Recover world-frame nominal state (from `recoverWorldState`):

$$\bar{\chi}_k = \text{Gamma}(\Upsilon.s) \cdot \bar{\chi}_i \cdot \Upsilon$$

where $\bar{\chi}_i$ is the nominal state at the last correction time $i$, and

$$\text{Gamma}(dt) = \begin{bmatrix} I & g_w \cdot dt & -\frac{1}{2}g_w \cdot dt^2 \\ 0 & 1 & -dt \\ 0 & 0 & 1 \end{bmatrix}$$

#### Covariance Propagation

Discretize the continuous linearized error dynamics at each IMU step $k$ with interval $\Delta t_k$:

$$\Phi_k = I_{15} + F_k \cdot \Delta t_k$$

Because $Q_c$ is a PSD, the first-order discrete process noise is:

$$Q_{d,k} = \Phi_k \, G_k \, Q_c \, G_k^T \, \Phi_k^T \, \Delta t_k$$

Then:

$$P \leftarrow \Phi_k \, P \, \Phi_k^T + Q_{d,k}$$

Repeat for every IMU sample between LiDAR corrections. Use the current nominal $\bar{R}_{wb}$ (and bias-corrected $\omega, a$) when building $F_k, G_k$.

> Do **not** divide by $\Delta t_k$. With `/Δt`, smaller IMU periods would incorrectly inflate process noise.

Optional higher-order forms (van Loan, $Q_d \approx G Q_c G^T \Delta t$ without the $\Phi$ sandwich) are acceptable later; the formula above is the intended v1 default.

*Advantages:* Simple, proven (FAST-LIO-style), minimal code changes, nominal path identical to current Sapphire.
*Disadvantages:* Does not use Gal(3) preintegration covariance; Euler linearization per IMU step.

### 4.2 Approach B: Full Gal(3) Preintegration Covariance — future / unverified

> Status: conceptual only. The 20D tangent-group covariance $\to$ 15D ESKF mapping below has **not** been verified against `EquivariantPreintegration` error conventions. Do not implement until separately derived and unit-tested.

The library provides:

- `Upsilon()`: preintegrated Gal(3) motion
- `Jb()`: $\partial \text{Upsilon}/\partial b$ ($10\times10$)
- `Cov()`: $20\times20$ tangent-group error covariance
- `CovNav()`: top-left $9\times9$ navigation block

A correct Approach B must:

1. State the exact TG error definition used by the library (`log_TG`, left/right convention).
2. Derive how $\chi_j = \Gamma_{ij}\chi_i\Upsilon_{ij}$ transports that error into the ESKF $\delta x$ convention of §2.2.
3. Couple bias errors through `Jb()` without ad-hoc $J_b^{-1}$ inversions.
4. Prove that the resulting $P$ stays consistent with Approach A on short horizons.

Until then, keep Approach A.

### 4.3 Recommendation

Start with **Approach A**:
1. FAST-LIO-style linearized covariance works well at ~200 Hz IMU.
2. Nominal Gal(3) path stays identical to current code.
3. Approach B can replace only the covariance block later, after a dedicated derivation.

---

## 5. ESKF Correction Step

### 5.1 ICP Measurement Model

At each LiDAR scan, GICP registration against the submap provides:

- $\hat{T}_{wl} \in SE(3)$: estimated LiDAR pose in world frame
- $H_{\text{gicp}} \in \mathbb{R}^{6\times6}$: final information / Hessian from `small_gicp` (`getFinalHessian()`, order `[rx, ry, rz, tx, ty, tz]`)

Convert to IMU frame with the known extrinsic:

$$\hat{T}_{wi} = \hat{T}_{wl} \cdot T_{imu\_lidar}^{-1}$$

Let the nominal IMU pose at the scan reference time be

$$\bar{T}_{wi} = (\bar{R}_{wb},\; \bar{p}_w).$$

**Use an observation residual consistent with the §2.2 error-state definition** (right-multiplied $SO(3)$ + world-frame additive position):

$$
z =
\begin{bmatrix}
\log\!\big(\bar{R}_{wb}^{T}\,\hat{R}_{wb}\big)^{\vee} \\
\hat{p}_w - \bar{p}_w
\end{bmatrix}
\in \mathbb{R}^{6}
$$

> Do **not** feed a raw $SE(3)$ logarithm $\log(\bar{T}^{-1}\hat{T})^\vee$ into the Jacobian below. That SE(3) log puts the translational component in the body frame, which is inconsistent with world-frame $\delta p$.

If a body-frame translational residual is preferred later, redefine both $z$ and $H$ together (e.g. $z_p = \bar{R}^{T}(\hat{p}-\bar{p})$ and put $\bar{R}^{T}$ into the position block of $H$).

### 5.2 Measurement Covariance from ICP Hessian

`small_gicp::RegistrationResult::H` / `getFinalHessian()` is the final information matrix of the GICP cost. A practical measurement covariance is:

$$R = \sigma_{icp}^{2}\, H_{\text{gicp}}^{-1} \in \mathbb{R}^{6\times6}$$

with safeguards:

1. Symmetrize $H_{\text{gicp}} \leftarrow \tfrac12(H+H^{T})$.
2. Eigenvalue floor / pseudo-inverse so degenerate directions do not produce near-zero variance.
3. Inflate by $\sigma_{icp}^{2}$ (typical $5\sim25$) because scan-to-submap ICP covariances are optimistic and correlated with the map.
4. Optional: detect near-null eigenvalues and further inflate those axes.

If the Hessian is unavailable or ill-conditioned, fall back to a diagonal covariance:

$$R = \text{diag}(\sigma_r^2 I_3,\; \sigma_t^2 I_3)$$

with defaults $\sigma_r \approx 0.02$ rad and $\sigma_t \approx 0.05$ m.

For the first bring-up, prefer the diagonal fallback; enable Hessian-based $R$ only after the filter is stable.

### 5.3 Measurement Jacobian

With the residual of §5.1 and $\delta x=0$ before update (ESKF reset convention):

$$
h(\delta x)=
\begin{bmatrix}
\delta\theta \\
\delta p
\end{bmatrix},
\quad
H =
\begin{bmatrix}
I_3 & 0 & 0 & 0 & 0 \\
0 & 0 & I_3 & 0 & 0
\end{bmatrix}_{6\times15}
$$

State order is $[\delta\theta,\;\delta v,\;\delta p,\;\delta b_a,\;\delta b_g]$. This matches `small_gicp` Hessian axis order `[rx,ry,rz,tx,ty,tz]` when $R$ is expressed in the same residual coordinates.

### 5.4 Kalman Update

Innovation (error state is zero after the previous reset):

$$\nu = z$$

Innovation covariance:

$$S = H\, P\, H^{T} + R \in \mathbb{R}^{6\times6}$$

Kalman gain:

$$K = P\, H^{T}\, S^{-1} \in \mathbb{R}^{15\times6}$$

Error-state update:

$$\delta\hat{x} = K\, \nu$$

Prefer the Joseph form for covariance update (§10.2).

### 5.5 Outlier Rejection

Use **layered** rejection; do not replace the existing hard gates immediately.

1. **Hard gates (keep, DLIO-compatible):** reject if translation / rotation correction exceeds `max_correction_trans` / `max_correction_rot_deg`, or inliers are too few.
2. **Mahalanobis gate (add):**
   $$d = \nu^{T} S^{-1} \nu$$
   Reject if $d > \chi^{2}_{6,0.95} \approx 12.59$.

Until $R$ is well calibrated, Mahalanobis alone can mis-accept / mis-reject. Keep both.

---

## 6. ESKF Reset Step

After correction, inject the error state into the nominal state and reset the error state to zero.

### 6.1 Nominal State Injection

$$\bar{R}_{wb} \leftarrow \bar{R}_{wb} \cdot \exp(\delta\hat{\theta}_\times)$$
$$\bar{v}_w \leftarrow \bar{v}_w + \delta\hat{v}$$
$$\bar{p}_w \leftarrow \bar{p}_w + \delta\hat{p}$$
$$\bar{b}_a \leftarrow \bar{b}_a + \delta\hat{b}_a$$
$$\bar{b}_g \leftarrow \bar{b}_g + \delta\hat{b}_g$$

### 6.2 Covariance Reset

After injection, the error state is redefined w.r.t. the new nominal orientation. Transform covariance:

$$P \leftarrow J_r\, P\, J_r^{T}$$

$$J_r = \begin{bmatrix}
I_3 - \tfrac12[\delta\hat{\theta}]_\times & 0 & 0 & 0 & 0 \\
0 & I_3 & 0 & 0 & 0 \\
0 & 0 & I_3 & 0 & 0 \\
0 & 0 & 0 & I_3 & 0 \\
0 & 0 & 0 & 0 & I_3
\end{bmatrix}$$

For small single-step corrections, $J_r\approx I$ is acceptable and slightly overestimates uncertainty. Prefer keeping the first-order $J_r$ once the filter is online.

---

## 7. Complete ESKF Cycle

```
Algorithm: ESKF LIO Frontend Cycle (Approach A)

Input: IMU samples at ~200 Hz, LiDAR scans at ~10 Hz
State: x̄ (nominal 15D), P (15×15), Gal(3) integrator from last correction

On each IMU sample (ω_m, a_m, dt):
    1. Bias-correct: ω = ω_m - b̄_g, a = a_m - b̄_a
    2. Gal(3) mean: Υ ← Υ · exp([ω; a; 0; 1]^∧ · dt)
    3. Build F_k, G_k from ω, a, R̄_wb
    4. Φ_k = I + F_k · dt
    5. Q_{d,k} = Φ_k · G_k · Q_c · G_k^T · Φ_k^T · dt
    6. P ← Φ_k · P · Φ_k^T + Q_{d,k}
    7. Recover nominal: χ̄ ← Gamma(Υ.s) · χ̄_ref · Υ
    8. Publish 200 Hz odometry: (R̄_wb, p̄_w, v̄_w)

On each LiDAR scan:
    1. Wait for IMU coverage through scan end
    2. Deskew via Gal(3) (unchanged)
    3. Ensure P / nominal are at deskew reference_stamp
    4. GICP → T̂_wl, optional H_gicp
    5. T̂_wi = T̂_wl · T_imu_lidar^{-1}
    6. Residual z = [ log(R̄^T R̂)^∨ ; p̂ - p̄ ]
    7. Build R from diagonal fallback or σ² H_gicp^{-1} (with floors)
    8. Hard gates (trans/rot/inliers) — keep
    9. S = H_meas P H_meas^T + R
   10. Mahalanobis gate: reject if ν^T S^{-1} ν > χ²_6,0.95
   11. K = P H_meas^T S^{-1}
   12. δx̂ = K ν
   13. Inject: x̄ ← x̄ ⊞ δx̂
   14. Joseph update + J_r reset on P
   15. Reset Gal(3) integrator; replay IMU newer than reference
       (same role as current rebasePropagation)
   16. Update submap / PGO (unchanged)

Note: preintegrate only from correction i to correction i+1.
```

Timing constraint (important for Sapphire):
- Deskew / GICP prior are evaluated at `reference_stamp`.
- Covariance $P$ must be propagated to that same stamp before `correct()`.
- After correction, rebase and replay buffered IMU beyond the reference stamp (existing pipeline pattern).

---

## 8. Comparison: Geometric Observer vs ESKF

| Aspect | Geometric Observer (current) | ESKF (proposed) |
|--------|------------------------------|-----------------|
| **State uncertainty** | Not modeled | Full $15\times15$ covariance |
| **IMU prediction** | Gal(3) mean only | Gal(3) mean + linearized $P$ propagation |
| **ICP measurement** | Fixed gains (Kp, Kv, Kq) | Kalman gain from $P$ and $R$ |
| **ICP quality** | Not used (fixed gains) | Optional Hessian → $R$; start with diagonal |
| **Outlier rejection** | Hard threshold (1 m, 20°) | Hard threshold **plus** Mahalanobis |
| **Bias estimation** | Heuristic gains (Kab, Kgb) | Coupled via cross-covariance |
| **Degenerate scenarios** | Blindly trusts ICP if accepted | Inflate / gate uncertain directions |
| **PGO integration** | Heuristic information | Can later use marginal pose covariance |
| **Code complexity** | ~70 lines (`observer.cpp`) | ~300 lines + Hessian plumbing |
| **Tuning parameters** | 5 gains | IMU densities + ICP $R$ scale |
| **Numerical issues** | None | Keep $P$ SPD (Joseph + symmetrize) |

---

## 9. Noise Parameter Tuning

### 9.1 IMU Noise (from BMI088 datasheet + DLIO defaults)

| Parameter | Symbol | Value | Unit | Source |
|-----------|--------|-------|------|--------|
| Gyro noise density | $\sigma_g$ | 0.000152 | rad/s/√Hz | BMI088 ≈ 0.014 °/s/√Hz, DLIO default |
| Accel noise density | $\sigma_a$ | 0.000196 | m/s²/√Hz | BMI088 ≈ 175 μg/√Hz |
| Gyro bias random walk | $\sigma_{bg}$ | $6.55 \times 10^{-5}$ | rad/s/√s | DLIO default |
| Accel bias random walk | $\sigma_{ba}$ | $1.65 \times 10^{-4}$ | m/s²/√s | DLIO default |

These enter $Q_c$ as PSD values. With the §4.1 discretization they are multiplied by $\Delta t$, not divided.

### 9.2 ICP Measurement Noise

| Parameter | Symbol | Typical Range | Notes |
|-----------|--------|---------------|-------|
| ICP covariance scale | $\sigma_{icp}^2$ | $5 \sim 25$ | Higher = trust ICP less |
| Fallback rotation std | $\sigma_r$ | $0.02$ rad | First bring-up default |
| Fallback translation std | $\sigma_t$ | $0.05$ m | First bring-up default |

### 9.3 Initial Covariance

After IMU initialization:

$$P_0 = \text{diag}\left(
\sigma_{\theta_0}^2 I_3,\;
\sigma_{v_0}^2 I_3,\;
\sigma_{p_0}^2 I_3,\;
\sigma_{ba_0}^2 I_3,\;
\sigma_{bg_0}^2 I_3
\right)$$

Typical values:
- $\sigma_{\theta_0} = 0.017$ rad (1°, from gravity alignment)
- $\sigma_{v_0} = 0.01$ m/s (assumed stationary)
- $\sigma_{p_0} = 0.01$ m (assumed at origin)
- $\sigma_{ba_0}, \sigma_{bg_0}$ from `ImuInitializer` variance output when available

---

## 10. Implementation Notes

### 10.1 ICP Hessian Extraction from small_gicp

`RegistrationPCL` already exposes:

```cpp
const Eigen::Matrix<double, 6, 6>& H =
    gicp_.getFinalHessian();  // [rx, ry, rz, tx, ty, tz]
```

Also available via `getRegistrationResult().H` (documented as final information matrix).

Extend Sapphire `RegistrationResult` to carry this matrix (and optionally `error`) so the fusion layer can build $R$.

### 10.2 Covariance Numerical Stability

1. **Joseph form** (preferred):
   $$P \leftarrow (I - KH)P(I - KH)^{T} + KRK^{T}$$
2. **Enforce symmetry** after predict/update:
   $$P \leftarrow 0.5\,(P + P^{T})$$
3. Optional eigenvalue floor if $P$ loses SPD due to large corrections.

### 10.3 Code Structure

Suggested class layout:

```cpp
class Eskf {
public:
    struct State {
        Eigen::Isometry3d T_world_imu;
        Eigen::Vector3d v_world;
        Eigen::Vector3d accel_bias;
        Eigen::Vector3d gyro_bias;
    };

    void predict(const ImuData& imu, double dt);
    bool correct(const Isometry3d& T_world_imu_measured,
                 const Eigen::Matrix<double, 6, 6>& R_icp);

    const State& state() const;
    const Eigen::Matrix<double, 15, 15>& covariance() const;

private:
    State nominal_;
    Eigen::Matrix<double, 15, 15> P_;
    detail::MeanOnlyGal3Integrator integrator_;
    // F, G, Q_c builders for Approach A
};
```

Keep `applyGeometricObserver` as the default fusion path. Add a config switch:

```text
odometry.fusion = "observer" | "eskf"
```

### 10.4 Integration with Current Pipeline

Replace / branch the fusion components in `processLidarScan()`:

```
Current:                        ESKF branch:
  deskew                         deskew (same)
  GICP registration              GICP registration (+ expose Hessian)
  geometricObserver(...)    →    eskf.correct(T_measured, R_icp)
  rebasePropagation(...)    →    integrator reset + IMU replay (same role)
```

IMU callback (`pushImu`):

```
propagateStateLocked → eskf.predict(imu, dt) + recover nominal
```

Bring-up order:
1. Diagonal $R_icp`, Approach A, observer still default.
2. Unit tests: static, pure translation, bias step, Mahalanobis reject.
3. Enable Hessian-based $R$ with inflation / eigenvalue floors.
4. Only then consider Approach B.

---

## 11. References

- Fornasier, A., Delama, G., et al. "Equivariant IMU Preintegration with Biases: a Galilean Group Approach." arXiv:2411.05548, 2024.
- Fornasier, A., et al. "Equivariant Symmetries for Inertial Navigation Systems." arXiv:2309.03765, 2023.
- Solà, J. "Quaternion kinematics for the error-state Kalman filter." arXiv:1711.02508, 2017.
- Xu, W., Zhang, F. "FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter." RA-L, 2021.
- Chen, K., et al. "Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction." ICRA, 2023.
- Koide, K. "small_gicp: Efficient and parallel algorithms for point cloud registration." JOSS, 2024.

---

## 12. Changelog

### v1.1.1 — 2026-07-16

- Clarified that the v1.0 $\alpha$/$a$ swap was a documentation bug only; current mean-only integrator is already correct.
- Expanded §3.2: with zero bias RW in $Q_c$, ESKF does not estimate bias; set DLIO defaults in TOML when enabling ESKF (no struct change).

### v1.1 — 2026-07-16

- Fixed Gal(3) IMU increment ordering to $[\omega,\;a_{\text{body}},\;0,\;1]$ (was incorrectly swapped in the draft).
- Fixed discrete process noise to $Q_d = \Phi G Q_c G^{T}\Phi^{T}\Delta t$ (was $/Δt$).
- Aligned ICP residual with right-multiplied $SO(3)$ + world-frame $\delta p$; removed inconsistent raw $SE(3)$ log usage with that $H$.
- Marked Approach B as future / unverified.
- Kept hard ICP gates; Mahalanobis is additive, not a full replacement.
- Documented `small_gicp` Hessian order and bring-up safeguards.
- Noted bias random-walk must be enabled for ESKF (current config defaults are zero).

### v1.0 — 2026-07-16

- Initial DeepSeek draft.

---

*Document version: v1.1.1*
*Date: 2026-07-16*
*Author: Sapphire team*
