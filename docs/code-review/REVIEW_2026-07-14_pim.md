# Sapphire 代码审查 — PIM / IMU 预积分性能分析

审查日期: 2026-07-14 | 范围: `external/preintegration/`, `deskew.cpp`, `pipeline.cpp`, `integrate_measurement.hpp`

---

## Executive Summary

项目采用 **Equivariant IMU Preintegration on Gal(3)**（[Delama et al., IEEE RA-L 2025](https://doi.org/10.1109/LRA.2024.3511424)），理论选型正确，热路径已使用 `integrateMeasurementMeanOnly` 跳过协方差传播。

**公司环境「超级慢」的首要根因是 Debug 构建，而非算法错误。** 本机微基准显示 Debug 比 Release 慢约 **200×**；20,000 唯一时间戳的 deskew 积分在 Release 下约 7 ms，Debug 下约 1.5 s。

Release 下当前实现 10 Hz LiDAR 可接受；Mid-360 等高密度时间戳场景仍有算法级优化空间（时间分片 / 区间内插值）。

**Verdict: 算法选型 OK；优先确保 Release 构建，再考虑 deskew 积分策略优化。**

---

## Top Issues Summary

| # | 文件 | 问题 | 严重度 |
|---|------|------|--------|
| 1 | `CMakeLists.txt` | 未默认 `CMAKE_BUILD_TYPE=Release`，colcon 易落到 Debug | **HIGH** |
| 2 | `deskew.cpp:174-228` | 对每个唯一时间戳做精确 Gal(3) 积分，复杂度 O(N_ts) | **HIGH** |
| 3 | `preintegration.hpp:290-295` | 每步调用 `phi(X_, xi0_)`，含 inv + 10×10 Adjoint | MEDIUM |
| 4 | `deskew.cpp:207` | `Pim partial = pim` 全量拷贝 Cov/Jxi（mean-only 不需要） | MEDIUM |
| 5 | `preintegration.hpp:96-112` | `Gamma_ij()` + `Upsilon()` 重复计算 `phi(X_, xi0_)` | MEDIUM |
| 6 | `integrate_measurement.hpp:11` | `kMaxIntegrationStepSec=0.02` 对大 dt 拆步，调用次数 ×N | LOW |

---

## 1. 架构概览

### 1.1 库来源与理论

| 组件 | 路径 | 说明 |
|------|------|------|
| Equivariant PIM 核心 | `external/preintegration/preintegration.hpp` | Gal(3) 等变预积分，BSD-2 许可 |
| Lie 群运算 | `external/lie/Gal3.hpp`, `TG.hpp`, `SO3.hpp` | exp / log / Adjoint / leftJacobian |
| 积分封装 | `include/sapphire/odometry/detail/integrate_measurement.hpp` | 步长切分 + mean-only 调用 |
| 使用方 | `deskew.cpp`, `pipeline.cpp` | deskew 时间线 + 实时状态传播 |

参考文献:
- [Equivariant IMU Preintegration with Biases (arXiv:2411.05548)](https://arxiv.org/abs/2411.05548)
- [开源实现 aau-cns/equivariant-preintegration](https://github.com/aau-cns/equivariant-preintegration)

相比传统 Forster 式 9 维 PIM，Gal(3) 方法将导航状态与 bias 几何耦合，协方差一致性更好。Sapphire 选型合理。

### 1.2 两条热路径

| 路径 | 入口 | 积分方式 | 频率 |
|------|------|----------|------|
| 实时传播 | `pipeline.cpp::propagateStateLocked` | `integrateMeasurementMeanOnly` | ~200 Hz（每 IMU 样本） |
| 去畸变 | `deskew.cpp::integrateTimeline` | `integrateMeasurementMeanOnly` + 状态恢复 | 每帧 LiDAR，次数 = 唯一时间戳数 |

已做对的一点：热路径使用 `integrateMeasurementMeanOnly`，跳过 20×20 协方差传播。`tests/preintegration_test.cpp` 验证 Upsilon 与完整积分一致（1e-12），Cov/Jxi 不变。

```cpp
// preintegration.hpp:290-295
void integrateMeasurementMeanOnly(const Vec3& accMeas, const Vec3& gyroMeas, FPType dt) {
    Input u(gyroMeas, accMeas);
    X_ = X_ * Lambda(phi(X_, xi0_), u, dt);  // 仅均值，不更新 Cov/Jxi
}
```

---

## 2. 性能根因：Debug vs Release（~200× 差距）

### 2.1 微基准结果

在本机复现与 deskew 相同调用模式的微基准（模拟 `integrateTimeline` 逻辑）：

| 场景 | Release (-O3) | Debug (-O0) | 倍数 |
|------|---------------|-------------|------|
| 单次 `integrateMeasurement` (dt=5 ms) | **0.2 µs** | **45 µs** | ~225× |
| deskew 模式 (200 时间戳) | **0.12 ms** | **16 ms** | ~133× |
| deskew 模式 (**20,000** 时间戳) | **7.0 ms** | **1484 ms** | ~212× |

结论:
- **Release**: 20K 时间戳积分 ~7 ms，10 Hz LiDAR 可接受
- **Debug**: 同场景 ~1.5 s/帧，体感「卡死」

### 2.2 为什么 Debug 这么慢

PIM 是 header-only 模板库，每次 `integrateMeasurementMeanOnly` 触发大量内联 Eigen 运算:
- `Gal3::exp` → SO(3) exp + Gal(3) leftJacobian（10×10）
- `phi` → `inv()` + 10×10 `Adjoint()`
- `Gal3TG::operator*` → 群乘法 + Adjoint 作用

在 `-O0` 下无法内联/向量化/SIMD，每次调用成本从亚微秒级膨胀到数十微秒。

### 2.3 构建配置问题

`sapphire/CMakeLists.txt` **未**设置默认构建类型。对比 DLIO:

```cmake
# direct_lidar_inertial_odometry/CMakeLists.txt:32
set(CMAKE_BUILD_TYPE "Release")
```

若 `colcon build` 未指定 `-DCMAKE_BUILD_TYPE`，容易落到 Debug 或未优化构建。

**建议 (P0)**:

```bash
# 开发调试（保留符号 + 基本优化）
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 上车 / 跑 bag
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

或在 `sapphire/CMakeLists.txt` 增加:

```cmake
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
```

验证命令:

```bash
grep CMAKE_BUILD_TYPE build/sapphire/CMakeCache.txt
```

---

## 3. Deskew 算法：精度换算力的主要瓶颈

### 3.1 当前实现

`integrateTimeline` 对每个**唯一时间戳**做精确 Gal(3) 积分:

```cpp
// deskew.cpp:205-226
Pim partial = pim;                              // 区间起点快照
while (target_stamps[target_idx] <= interval_end) {
    detail::integrateMeasurement(partial, measurement, partial_dt);
    states.push_back({
        partial.Gamma_ij() * initial_state * partial.Upsilon(),
        target_stamp,
    });
}
detail::integrateMeasurement(pim, measurement, interval_dt);  // 主 pim 推进
```

复杂度: **O(N_unique_timestamps)** 次 Lie 群运算。

Mid-360 一帧可达数千～两万唯一时间戳（`DESIGN.md` 注释: "~20K unique timestamps"）。微基准中 20K 时间戳、20 个 IMU 区间 → 19 次 pim 拷贝 + 19,020 次积分调用。

### 3.2 与 DLIO 对比

DLIO（`direct_lidar_inertial_odometry/src/dlio/odom.cc:1494-1580`）策略:

1. 每个 IMU 区间只做 **1 次** `integrateMeasurement`（含协方差）
2. 区间内各时间戳用 **slerp（旋转）+ 线性插值（位置）**

```cpp
// DLIO integrateImu — 区间内插值，非逐时间戳积分
const Eigen::Matrix3d dR0 = pim_->deltaRij();
pim_->integrateMeasurement(accel, gyro, dt);
const Eigen::Matrix3d dR1 = pim_->deltaRij();
while (*stamp_it <= m.stamp) {
    const double alpha = (*stamp_it - t0) / dt;
    T.block<3,3>(0,0) = R_init * q0.slerp(a, q1).toRotationMatrix();
    T.block<3,1>(0,3) = P_init + R_init * (dP0 + a * (dP1 - dP0));
}
```

| 维度 | Sapphire（当前） | DLIO |
|------|------------------|------|
| 积分次数 | O(N_timestamps) | O(N_imu_intervals) |
| 区间内策略 | 精确 Gal(3) 恢复 | slerp + lerp |
| 速度状态 | ✅ 完整保留 | ❌ deskew 忽略速度 |
| 精度 | 最高 | 5 ms 内近似足够 |
| Release 20K ts | ~7 ms | 预估 <1 ms |

Sapphire 理论更完整（Gal(3) 恢复保留速度，利于传播/观测器），但算力随时间戳数线性增长。

### 3.3 优化方案

| 方案 | 复杂度 | 精度损失 | 实现难度 |
|------|--------|----------|----------|
| **A. 自适应时间分片** | O(N_slices), N_slices≪N_ts | 可控（如 64/128 片） | 低 |
| **B. IMU 区间内 Gal(3) 插值** | O(N_imu + N_ts) | 5 ms 内近似 | 中 |
| **C. 仅保存 Gal3TG 快照** | 同当前，减少拷贝 | 无 | 低 |

`types.hpp:100-102` 注释已提到 "Desired number of deskew time slices"，但 `DeskewConfig` 目前只有 `time_offset`，分片尚未实现。

**建议 (P1)**: 在 `DeskewConfig` 增加 `max_time_slices`（默认如 128），对相近时间戳分桶，只在桶边界做精确积分。对 5 ms IMU 区间，插值误差对去畸变通常可忽略。

---

## 4. 热路径微优化

### 4.1 每步调用 `phi(X_, xi0_)`

`integrateMeasurementMeanOnly` 每次积分都通过 `Lambda(phi(X_, xi0_), u, dt)` 重建当前状态。`phi` 实现:

```cpp
// preintegration.hpp:161-164
const State phi(const Gal3TG& X, const State& xi) const {
    return State(xi.Upsilon() * X.G(), X.G().inv().Adjoint() * (xi.bias() - X.g()));
}
```

含 `inv()` + 10×10 `Adjoint()`，是单次积分中最重的操作之一。

当前 deskew 和 propagation 均使用 `biasHat_ = Zero`。可为此场景添加专用快速路径，避免通用 `phi` 调用。

### 4.2 `Gamma_ij()` 与 `Upsilon()` 重复计算

```cpp
// preintegration.hpp:96-112
const State xi() const { return phi(X_, xi0_); }
const Gal3 Upsilon() const { return xi().Upsilon(); }
const FPType deltaTij() const { return xi().Upsilon().s(); }  // Gamma_ij 间接调用
```

deskew 每个时间戳执行 `partial.Gamma_ij() * initial_state * partial.Upsilon()`，`phi` 被计算两次。

**建议 (P2)**: 增加融合恢复方法:

```cpp
Gal3 recoverWorldState(const Gal3& initial) const {
    const State s = xi();  // 只算一次
    return buildGamma(s) * initial * s.Upsilon();
}
```

### 4.3 `Pim partial = pim` 全量拷贝

即使 mean-only 不更新协方差，`EquivariantPreintegration` 拷贝仍包含:
- `Mat20 Cov_` (400 doubles)
- `Mat20 Jxi_` (400 doubles)
- `Gal3TG X_`, `Vec10 biasHat_`, `State xi0_`, `shared_ptr<Params>`

每 IMU 区间拷贝一次（20 区间 ≈ 19 次）。不是主瓶颈（积分次数才是），但是纯浪费。

**建议 (P2)**: 只保存 `Gal3TG X_snapshot`，或实现轻量 `MeanOnlyIntegrator { Gal3TG X_; }`（约 110 doubles）。

### 4.4 积分步长切分

```cpp
// integrate_measurement.hpp:11-28
constexpr double kMaxIntegrationStepSec = 0.02;
// dt > 20 ms 拆成 ceil(dt/0.02) 步
```

| 场景 | dt | 步数 | 影响 |
|------|-----|------|------|
| 200 Hz IMU（正常） | 5 ms | 1 | 无 |
| IMU 丢包 | 50 ms | 3 | 调用 ×3 |
| IMU 丢包 | 100 ms | 5 | 调用 ×5 |

对 propagation 可考虑: dt < 100 ms 不拆分。deskew 保持拆分以保证大 dt 数值稳定。

### 4.5 `Params::Qc()` 动态构造

完整 `integrateMeasurement` 每次调用 `p_->Qc()` 构造 20×20 矩阵。热路径已用 mean-only 避开。若未来后端因子图需要完整积分，应缓存 `Qc()`。

---

## 5. Pipeline 传播路径（非瓶颈）

```cpp
// pipeline.cpp:235-244
const double dt = imu.stamp - propagated_state_.stamp;
detail::integrateMeasurement(*propagation_pim_, imu, dt);
recoverPropagatedStateLocked(imu.stamp);
```

200 Hz × 0.2 µs/call ≈ **0.04 ms/s**，Release 下可忽略。

性能问题集中在 **deskew 的 per-timestamp 积分**，不在实时传播。

---

## 6. 完整积分 vs Mean-Only 使用边界

| 方法 | 更新内容 | 单次成本 | 当前使用 |
|------|----------|----------|----------|
| `integrateMeasurement` | 均值 + 20×20 Cov + 20×20 Jxi | 高（~10-50× mean-only） | 未用于热路径 |
| `integrateMeasurementMeanOnly` | 仅 Gal3TG X_ | 低 | deskew + propagation |

完整积分中的协方差传播:

```cpp
Cov_ = A * Cov_ * A.transpose() + B * (p_->Qc() / dt) * B.transpose();  // 20×20 × 20×20
Jxi_ = Phi_b * Jxi_;
```

若未来 PGO / 因子图需要 IMU 因子，应评估是否:
- 在因子构建时离线积分（非热路径），或
- 使用更轻量的 9 维 Forster PIM 仅用于优化

当前前端不需要完整积分，mean-only 选择正确。

---

## 7. 优化优先级

```
┌──────────┬──────────────────────────────────────────────────────────────┐
│ P0 立刻  │ 确保 Release/RelWithDebInfo 构建；CMake 默认 Release         │
├──────────┼──────────────────────────────────────────────────────────────┤
│ P1 高收益│ deskew 时间分片 或 IMU 区间内插值                            │
│          │ 20K ts: Release 7ms → 预估 <1ms                              │
├──────────┼──────────────────────────────────────────────────────────────┤
│ P2 中收益│ 轻量积分器（只存 Gal3TG）；合并 Gamma+Upsilon 恢复          │
│          │ bias=0 快速路径                                              │
├──────────┼──────────────────────────────────────────────────────────────┤
│ P3 低收益│ 按需调整步长切分阈值；缓存 Qc()（仅完整积分场景）            │
└──────────┴──────────────────────────────────────────────────────────────┘
```

---

## 8. 验证步骤

1. 确认构建类型: `grep CMAKE_BUILD_TYPE build/sapphire/CMakeCache.txt`
2. Release 重编后观察 deskew 耗时是否从秒级降到毫秒级
3. 若 Release 仍不够（如 Jetson Orin NX），优先实现时间分片（P1-A）
4. 可选: 恢复 `DeskewMetrics` 计时（`integration_ms`, `pim_copies`）用于板上 profiling

---

## 9. 结论

| 维度 | 评价 |
|------|------|
| 理论选型 | ✅ Gal(3) equivariant PIM 正确且先进 |
| Mean-only 分流 | ✅ 热路径正确跳过协方差 |
| 测试覆盖 | ✅ `preintegration_test.cpp` 验证等价性 |
| 公司环境慢 | ⚠️ **Debug 构建是主因**（~200×） |
| Deskew 扩展性 | ⚠️ O(N_ts) 精确积分，高密度时间戳有优化空间 |
| 算法 vs 工程 | 性能问题是**工程实现路径**，不是理论选型错误 |

---

## 附录 A: 关键文件索引

| 文件 | 行号 | 内容 |
|------|------|------|
| `external/preintegration/preintegration.hpp` | 252-278 | 完整积分（含 Cov） |
| `external/preintegration/preintegration.hpp` | 290-295 | Mean-only 积分 |
| `include/sapphire/odometry/detail/integrate_measurement.hpp` | 11-28 | 步长切分封装 |
| `src/odometry/deskew.cpp` | 174-228 | `integrateTimeline` |
| `src/odometry/pipeline.cpp` | 235-244 | `propagateStateLocked` |
| `tests/preintegration_test.cpp` | 19-66 | Mean-only 等价性测试 |

## 附录 B: 相关审查文档

- `REVIEW_2026-07-11.md` §3.4 — PIM 命名空间、deskew partial copy 首次记录
- `REVIEW_2026-07-11_v2.md` §4.9, §5 — Mean-only 验证、DeskewMetrics profiling
- `REVIEW_2026-07-14.md` — 同日其他审查（日志/诊断清理）
