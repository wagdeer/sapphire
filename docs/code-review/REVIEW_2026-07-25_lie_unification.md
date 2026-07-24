┌──────────────────────────────────────────────────────────────────────┐
│  ◆ SAPPHIRE  ::  李群工具统一审查                                    │
│  DATE       ::  2026-07-25                                           │
│  SCOPE      ::  external/lie/SO3.hpp 在全库中的使用机会               │
│  REF        ::  REVIEW_2026-07-24_docker_fresh.md §2.1               │
└──────────────────────────────────────────────────────────────────────┘

═══════════════════════════════════════════════════════════════════════
一、背景
═══════════════════════════════════════════════════════════════════════

  SAPPHIRE 已集成了 `external/lie/` 李群库（SO3, SEn3, Gal3, TG），
  GAL3 预积分器已经全线使用 `Gal3::exp()` / `multiplyRight()` 做精确
  流形传播。但下游代码仍散布着大量手动四元数运算、AngleAxis 构造和
  一阶雅可比近似，与 GAL3 的严谨性不匹配。

  本次审查追踪从上轮 §2.1（几何观测器四元数修正）出发，对全库做
  系统扫描，识别所有可迁移到 `lie::SO3d` 的位置。

═══════════════════════════════════════════════════════════════════════
二、已完成修复：几何观测器旋转修正 (P1 → RESOLVED)
═══════════════════════════════════════════════════════════════════════

  文件: src/odometry/observer.cpp:57-62

  旧代码（DLIO 原版近似）:
    q_correction(1.0 - std::abs(q_error.w()), q_error.vec())
    q_correction = q_prior * q_correction
    线性加法: q_observer = q_prior + dt * gain * q_correction
    q_observer.normalize()

  新代码（SO(3) Exponential Map）:
    lie::SO3d so3_error(q_error);
    Eigen::Vector3d omega = lie::SO3d::log(so3_error);
    omega *= dt * config.orientation_gain;
    lie::SO3d so3_inc = lie::SO3d::exp(omega);
    update.state.T_world_imu.linear() =
        (q_prior * so3_inc.q()).toRotationMatrix();

  效果:
    - 全程在 SO(3) 流形上操作，无需 normalize 拉回
    - 与 GAL3 积分器 `Gal3::exp(input*dt) + multiplyRight` 风格一致
    - 大角度修正 (>30°) 无偏置累积
    - CMakeLists.txt 新增 external/lie include path

  关联: observer.cpp 同时移除了不再需要的 #include <cmath>

═══════════════════════════════════════════════════════════════════════
三、已完成修复：手动 atan2 角度提取 (P1 → RESOLVED)
═══════════════════════════════════════════════════════════════════════

  3 处重复的手动 atan2 角度提取已统一为 lie::SO3d::log()。
  lie::SO3d::log() 内部处理了小角度展开和符号翻转。

  位置 1: src/odometry/submap.cpp:28-29
    修改前:
      Eigen::Quaterniond rotation(delta.rotation());
      rotation.normalize();
      const double angle =
          2.0 * std::atan2(rotation.vec().norm(), std::abs(rotation.w()));

    修改后:
      const double angle =
          lie::SO3d::log(lie::SO3d(delta.rotation())).norm();

  位置 2: src/backend/pose_graph.cpp:33-34 (rotationAngle 辅助函数)
    修改前:
      Eigen::Quaterniond rotation(transform.rotation());
      rotation.normalize();
      return 2.0 * std::atan2(rotation.vec().norm(), std::abs(rotation.w()));

    修改后:
      return lie::SO3d::log(lie::SO3d(transform.rotation())).norm();

  位置 3: src/odometry/registration.cpp:78-80 (GICP 修正拒绝阈值)
    修改前:
      Eigen::Quaterniond q_corr(T_correction.rotation());
      q_corr.normalize();
      corr_rot_deg =
          2.0 * std::atan2(q_corr.vec().norm(), std::abs(q_corr.w()))
          * (180.0 / M_PI);

    修改后:
      corr_rot_deg =
          lie::SO3d::log(lie::SO3d(T_correction.rotation())).norm()
          * (180.0 / M_PI);

  效果:
    - 共减少 8 行手动四元数 / atan2 实现，增加 3 行 lie::SO3d 调用
    - 各文件新增 #include <SO3.hpp>

═══════════════════════════════════════════════════════════════════════
四、已完成修复：eskf.cpp 手动 invRightJacobian 和 skew → wedge 统一 (P1 → RESOLVED)
═══════════════════════════════════════════════════════════════════════

  文件: src/odometry/eskf.cpp

  修复内容:

  1) 手动 `skew()` 函数（7 行）删除，4 处调用点替换为 `SO3::wedge()`:
     - propagateCovariance(): F 矩阵陀螺项 `-SO3::wedge(omega)`
     - propagateCovariance(): F 矩阵加速度项 `-R_world_imu * SO3::wedge(accel)`
     - registrationToInnovationJacobian(): 平移雅可比项
     - correctAt(): Joseph 重置雅可比 `I - 0.5 * SO3::wedge(dx_rot)`

  2) `so3RightJacobianInverse()` 从 16 行手动闭式精简为单行委托:
     修改前:
       const double theta = phi.norm();
       const Eigen::Matrix3d Phi = skew(phi);
       if (theta < 1e-6) { /* 二阶泰勒展开 */ }
       /* 闭式: I + 0.5Φ + (1/θ² - (1+cosθ)/(2θ·sinθ))·Φ² */

     修改后:
       return SO3::invRightJacobian(phi);

     SO3::invRightJacobian 内部使用 cot 半角公式，大角度数值更稳定；
     底层 epsilon = 1e-9（原实现为 1e-6），小角度阈值也更严格。

  效果:
    - 减少约 20 行手动李代数实现
    - invRightJacobian 数值稳定性提升（cot 半角 vs 1/θ² 闭式）
    - 与 GAL3 预积分器共用同一 SO3 工具链，全库风格统一
    - 调用方 `so3RightJacobianInverse` 接口不变，上层逻辑无影响

═══════════════════════════════════════════════════════════════════════
五、已完成修复：observer.cpp 陀螺偏置更新 — 小角度近似 (P1 → RESOLVED)
═══════════════════════════════════════════════════════════════════════

  文件: src/odometry/observer.cpp:44

  修改前:
    dt * config.gyro_bias_gain * q_error.w() * q_error.vec();

  q_error.w() * q_error.vec() ≈ ½·sin(θ)·axis 是小角度近似。

  修改后:
    dt * config.gyro_bias_gain * lie::SO3d::log(lie::SO3d(q_error));

  lie::SO3d::log 返回精确的旋转向量 θ·axis，不依赖小角度假设。
  与同文件姿态修正部分（lie::SO3d::log + lie::SO3d::exp）风格一致。

═══════════════════════════════════════════════════════════════════════
六、已完成修复：eskf.cpp 重置雅可比 — 一阶截断 → 精确闭式 (P1 → RESOLVED)
═══════════════════════════════════════════════════════════════════════

  文件: src/odometry/eskf.cpp:589-590

  修改前:
    Jr.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity() - 0.5 * SO3::wedge(dx.segment<3>(0));

  这是一阶近似 Jr ≈ I - ½Φ，丢掉了 (Φ^)² 及更高阶项。

  修改后:
    Jr.block<3, 3>(0, 0) =
        lie::SO3d::invLeftJacobian(-dx.segment<3>(0));

  invLeftJacobian 计算逆左雅可比精确闭式，大角度修正下协方差传播更准确。

═══════════════════════════════════════════════════════════════════════
七、待统一清单 — 按影响优先级
═══════════════════════════════════════════════════════════════════════

7.1 imu_init.cpp 重力对齐 — FromTwoVectors → SO3 构造 (P2)

  文件: src/imu_init.cpp:222-226

  当前:
    Eigen::Quaterniond q_gravity =
        Eigen::Quaterniond::FromTwoVectors(grav_imu, grav_world);
    q_gravity.normalize();

  建议:
    lie::SO3d so3_gravity(grav_imu, grav_world);
    Eigen::Quaterniond q_gravity = so3_gravity.q();

  lie::SO3d(u, v) 构造函数（SO3.hpp:79-105）已内置对平行/反平行
  等边界情况的处理，且不需要后续 normalize。

───────────────────────────────────────────────────────────────────────

7.2 deskew.cpp SLERP → Lie 代数插值 (P3)

  文件: src/odometry/deskew.cpp:155-156

  当前:
    start.q().slerp(alpha, end.q()).normalized();

  Lie 代数插值在正确性上等效于 SLERP:
    Eigen::Vector3d omega = lie::SO3d::log(start_so3 * end_so3.inv());
    lie::SO3d::exp(alpha * omega) * start_so3;

  收益较小（功能等价），留待后续统一。

───────────────────────────────────────────────────────────────────────

7.3 测试文件中的 AngleAxis (P3)

  文件: tests/eskf_test.cpp, observer_test.cpp, submap_test.cpp,
        deskew_test.cpp, mean_only_gal3_integrator_test.cpp

  测试中的 Eigen::AngleAxisd 构造可统一为 lie::SO3d::exp()。
  不影响生产行为，但可以当 lie::SO3d 的正确使用示例。

═══════════════════════════════════════════════════════════════════════
八、汇总
═══════════════════════════════════════════════════════════════════════

  ┌──────────────────────────────────────────┬─────────┬──────────┐
  │ 位置                                     │ 数量    │ 优先级   │
  ├──────────────────────────────────────────┼─────────┼──────────┤
  │ observer.cpp 旋转修正                     │ 1 ✅    │ P1 已修 │
  │ atan2 角度提取 (3 文件)                   │ 3 ✅    │ P1 已修 │
  │ eskf.cpp invRightJacobian + skew→wedge   │ 1 ✅    │ P1 已修 │
  │ observer.cpp 陀螺偏置                     │ 1 ✅    │ P1 已修 │
  │ eskf.cpp 重置雅可比                       │ 1 ✅    │ P1 已修 │
  │ imu_init.cpp 重力对齐                    │ 1       │ P2      │
  │ deskew.cpp SLERP                         │ 1       │ P3      │
  │ 各测试 AngleAxis                          │ ~10     │ P3      │
  │ config.cpp quaternion normalize          │ 1       │ 跳过    │
  │ pose_graph.cpp angularDistance           │ 1       │ P3      │
  └──────────────────────────────────────────┴─────────┴──────────┘

  已完成 5 项 P1 统一，剩余 3 项 P2/P3 待处理。
  净效果: 全库旋转运算统一对标 GAL3 精度等级
