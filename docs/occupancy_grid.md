# Occupancy Grid Map for Navigation

## 问题：为什么不用地面分割

传统 2D 栅格图从 3D 点云生成的常见做法是：先做地面分割，把点分为"地面"和"障碍物"，地面标 free，障碍物标 occupied。

这在平坦场景下没问题，但在坡道、台阶、不平整路面上，地面分割会遇到根本性困难：

- **全局地面模型**（如 RANSAC 平面拟合）在斜坡上会把坡面误判为非地面，或把远处低处地面误判为障碍物。
- **局部地面拟合**（如 cloth simulation、patch-wise RANSAC）增加了大量计算和参数调优，且在台阶边缘、路沿等几何突变处仍然不稳定。
- **法向量分类**依赖局部法向量估计，对稀疏点云和噪声敏感。

对于导航来说，我们关心的语义很简单：**机器人能不能走过去**。不需要精确区分"这是地面"还是"这是坡面"，只需要知道"这里有没有挡住我的东西"。

## 方案：姿态感知的局部水平面过滤

核心思想：**用 IMU 给出的机器人姿态定义一个局部水平面，只保留平面上方的点作为障碍物候选。**

这个平面不是世界坐标系的固定水平面，而是**跟着机器人姿态转**的。上坡时平面也倾斜，坡面上的点不会因为世界坐标 z 变高而被误保留。

### 几何定义

设机器人当前位姿为 $(t, R)$，其中 $t \in \mathbb{R}^3$ 是位置，$R \in SO(3)$ 是旋转矩阵（世界系←本体系）。

机器人的本体 z 轴（朝上）在世界坐标系下的方向为：

$$
\hat{z}_w = R \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}
$$

即旋转矩阵的第三列。

以机器人位置 $t$ 为平面上一点，$\hat{z}_w$ 为法向量，定义局部水平面：

$$
\Pi: \quad \hat{z}_w \cdot (p - t) = 0
$$

对于空间中任意一点 $p$，计算其到有符号距离：

$$
d(p) = \hat{z}_w \cdot (p - t)
$$

过滤规则：

$$
\begin{cases}
d(p) < -h_c & \Rightarrow \text{丢弃（平面下方，地面或坡面下部）} \\
d(p) \geq -h_c & \Rightarrow \text{保留（障碍物候选）}
\end{cases}
$$

其中 $h_c$ 是 clearance 高度（机器人底盘离地间隙或传感器安装高度相关的阈值）。

### 为什么等价于本体坐标系判断

在本体坐标系下，判断条件是：

$$
p_{\text{body},z} < -h_c
$$

展开本体坐标变换 $p_{\text{body}} = R^T (p_w - t)$：

$$
\begin{bmatrix} 0 & 0 & 1 \end{bmatrix} R^T (p_w - t) < -h_c
$$

$$
\left( R \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \right)^T (p_w - t) < -h_c
$$

$$
\hat{z}_w \cdot (p_w - t) < -h_c
$$

这正好就是世界坐标系下的点积公式。**两种表述数学上完全等价，不需要做任何坐标变换。**

## 栅格图数据结构

栅格图的存储设计参考 lightning-lm g2p5 的 SubGrid 方案，采用两级索引 + 懒分配，在大场景下内存友好。

### CellData

每个栅格单元存储三个字段：

```cpp
struct CellData {
    uint32_t hit_cnt   = 0;     // 障碍物命中计数
    uint32_t visit_cnt = 0;     // 总访问计数（含 hit + miss）
    float    d_min     = 1e4f;  // 该栅格处障碍物的最小 d 值
};
```

$d_{\min}$ 的物理意义：该栅格位置观测到的障碍物底面到姿态平面的有符号距离。

- 初始值 $10^4$（无穷远），表示从未被障碍物命中
- 被 hit 更新时记录最小的 $d$（最接近姿态平面的障碍物底面）
- 用于 d-aware 射线更新：只有低于 $d_{\min}$ 的射线才能刷白已确认的障碍物

### SubGrid 懒分配

采用 16×16 子网格（`SUB_GRID_SIZE = 4`，`width = 1 << 4 = 16`）的两级索引：

```cpp
class SubGrid {
    CellData* data_ = nullptr;   // 懒分配：首次访问时才 new

    void MallocIfNeeded() {
        if (data_ == nullptr) {
            data_ = new CellData[16 * 16];  // 256 cells
        }
    }
};
```

顶层栅格图是一个 `SubGrid**` 二维数组：

```cpp
class OccupancyGrid {
    float min_x_, min_y_, max_x_, max_y_;   // 世界坐标范围
    int   grid_size_x_, grid_size_y_;       // SubGrid 数组维度
    float resolution_;                       // 单元格分辨率（如 0.1m）
    SubGrid** grids_ = nullptr;

    // 索引计算
    // 单元格全局索引 = floor((p - min) / resolution)
    // SubGrid 索引   = global_idx >> 4
    // SubGrid 内偏移 = global_idx & 0xF
};
```

大场景（200m × 200m，分辨率 0.1m）共 $2000 \times 2000 = 4 \times 10^6$ 个单元格，分为 $125 \times 125 = 15625$ 个 SubGrid。未访问区域 `data_ = nullptr`，不占内存。

### 占据判定

不使用 log-odds 概率模型，使用 hit/visit 比值：

```cpp
bool IsOccupied(const CellData& cell) {
    return cell.visit_cnt > 3 &&
           static_cast<float>(cell.hit_cnt) / cell.visit_cnt > occ_threshold_;
}

bool IsFree(const CellData& cell) {
    return cell.visit_cnt > 0 && !IsOccupied(cell);
}

// visit_cnt == 0 → unknown
```

比值模型的优点：
- 参数少（一个 `occ_threshold` vs log-odds 的 $l_{\text{free}}, l_{\text{occ}}, l_{\min}, l_{\max}$）
- 收敛快（几次 visit 就能判定，不需要累积到阈值）
- 直觉（hit 占 visit 的比例就是"有多大把握是障碍物"）

## 栅格图更新

### 输入

| 数据 | 来源 | 坐标系 |
|------|------|--------|
| 位姿 $t, R$ | propagation | 世界系 |
| 点云 $\{p_i\}$ | deskew + registration 后投影 | 世界系 |
| 传感器原点 $t_s$ | $t + R \cdot t_{\text{ext}}$（外参补偿） | 世界系 |

### 过滤

对每个点 $p_i$：

```
z_hat = R.col(2)                      // 机器人 z 轴在世界系方向
d = z_hat.dot(p_i - t)                // 有符号距离
if d >= -h_clearance:
    keep(p_i)                          // 障碍物候选（hit）
else:
    keep(p_i)                          // 地面点（仅 ray cast 标 free）
```

**所有点都保留**用于 ray casting，只是 hit/miss 标记不同。这是消除幽灵障碍物的关键（详见 PGO 章节）。

### 射线遍历：斜率插值

从传感器原点 $t_s$ 到每个点 $p_i$，在 2D 栅格索引空间做整数直线填充：

```
// 起终点栅格索引
(x0, y0) = grid_index(sensor_origin.xy)
(x1, y1) = grid_index(p_i.xy)

dx = x1 - x0
dy = y1 - y0

// 沿主轴步进，副轴用斜率插值
if |dy| > |dx|:
    k = dx / dy                        // 唯一一次浮点除法
    sign = dy > 0 ? 1 : -1
    for j = sign; j != dy; j += sign:
        i = j * k
        cell_idx = (x0 + i, y0 + j)
        process_cell(cell_idx, ...)
else:
    k = dy / dx
    sign = dx > 0 ? 1 : -1
    for i = sign; i != dx; i += sign:
        j = i * k
        cell_idx = (x0 + i, y0 + j)
        process_cell(cell_idx, ...)
```

循环内部无浮点除法，无三角函数，只有加法和乘法。

### d 值插值

射线沿途每个栅格的 $d$ 值（到姿态平面的有符号距离）沿射线线性插值：

$$
d_{\text{sensor}} = \hat{z}_w \cdot (t_s - t)
$$

$$
d_{\text{end}} = \hat{z}_w \cdot (p_i - t)
$$

沿射线参数化（$N$ 为总步数，$j$ 为当前步）：

$$
d(j) = d_{\text{sensor}} + \frac{j}{N} (d_{\text{end}} - d_{\text{sensor}})
$$

增量计算避免重复点积：

```
d_step = (d_end - d_sensor) / N
d_ray = d_sensor
for each cell along ray:
    d_ray += d_step
    process_cell(cell, d_ray, ...)
```

### d-aware 更新规则

这是 2.5D 栅格的核心创新（参考 lightning-lm g2p5）。解决的问题：**防止高处射线穿透低处障碍物**。

场景示例：

```
    sensor (d=1.0)
        *
       / \
      /   \   ray A (d=0.8 at wall)
     /     \    → d_ray > d_min → 不刷白 ✓
    /   ┌──┐\
   /    │墙│ \   wall cell: d_min=0.3, hit_cnt=5
  /     │d=│  \
 /      │0.3┘  \
*───────┴───────*
       ray B (d=0.1 at wall)
         → d_ray < d_min → 刷白 visit_cnt++ ✓
```

ray A 从墙上方穿过，不能清除墙体。ray B 从墙下方穿过，确认墙下空间是 free。

更新规则：

```
// 终点 hit（障碍物点，d >= -h_clearance）
if d < cell.d_min:
    cell.d_min   = d
    cell.hit_cnt++
    cell.visit_cnt++

// 沿途 miss（射线穿过的中间栅格）
if cell.hit_cnt > 3:                          // 已确认的强障碍物
    if d_ray < cell.d_min:                    // 射线低于障碍物底面
        cell.visit_cnt++                      // → 可以刷白
    // else: 射线高于障碍物 → 不更新（不能穿透）
else:                                          // 弱/无障碍物
    cell.visit_cnt++
    cell.d_min = std::min(cell.d_min, d_ray)  // 取最小值，不覆盖
```

**关键逻辑**：`cell.hit_cnt > 3` 区分"已确认障碍物"和"可能是噪声"。强障碍物只有低于它的射线才能刷白，防止多条高射线把矮墙错误清除。

### 栅格参数

| 参数 | 含义 | 典型值 |
|------|------|--------|
| resolution | 单元格分辨率 | 0.05 ~ 0.1 m |
| h_clearance | 过滤阈值（底盘间隙） | 0.1 ~ 0.3 m |
| occ_threshold | 占据判定比值阈值 | 0.3 |
| usable_range | 最大有效射线距离 | 30 ~ 50 m |
| min_range | 最小有效射线距离 | 0.5 m |
| d_max | d 值上限（过滤悬挂物，可选） | 2.0 m |
| sub_grid_size | SubGrid 边长（2 的幂次） | 16 (1 << 4) |

### 额外过滤

除了姿态平面过滤外，叠加以下过滤以去除噪声：

- **最小距离过滤**：丢弃距传感器 < `min_range` 的点（近距离自反射和噪声）
- **d 值上限过滤**（可选）：丢弃 $d(p) > d_{\max}$ 的点（悬挂物、天花板，对地面导航无用）

```
for p in cloud:
    dist = (p - sensor_origin).norm()
    if dist < min_range:
        discard(p)
    if d_max > 0 && d(p) > d_max:
        discard(p)
```

## 坡道场景分析

### 上坡

```
          机器人 z 轴（倾斜）
              ↑
              |  ← 局部平面跟着倾斜
              |
    ──────────●──────────    ← 局部水平面（与坡面平行）
            / |
          /   |
        /     |
      /       |  ← 坡面在平面下方 → 被滤除 ✓
    /         |
```

上坡时，机器人前倾（pitch > 0），局部平面也前倾。坡面上的点位于倾斜平面下方，被正确滤除。前方真正高于坡面的障碍物（如路沿、箱子）位于平面上方，被正确保留。

### 下坡

```
              ↑
              |  机器人 z 轴（后仰）
              |
    ──────────●──────────    ← 局部水平面（与坡面平行）
              | \
              |   \
              |     \        ← 坡面在平面下方 → 被滤除 ✓
              |       \
```

下坡时同理，局部平面跟着后仰，坡面点被滤除。

### 平坦路面

```
              ↑
              |  机器人 z 轴（竖直）
              |
    ──────────●──────────    ← 局部水平面 = 世界水平面
              |
    ══════════╪══════════    ← 地面在平面下方 → 被滤除 ✓
```

平坦路面上退化为简单的 z 轴过滤。

## 与业界方案的对比

### vs 地面分割

| 维度 | 地面分割 | 姿态平面过滤 |
|------|----------|-------------|
| 坡道适应性 | 差（全局模型）或复杂（局部模型） | 天然适应（平面跟姿态转） |
| 计算量 | 高（RANSAC/cloth/法向量） | 极低（每点一次点积） |
| 参数 | 多（平面阈值、patch 大小、法向量阈值...） | 少（仅 h_clearance） |
| 导航语义 | 区分地面/非地面 | 区分"比底盘低" / "比底盘高" |
| 误判处理 | 坡面误判为非地面 → 假障碍物 | 坡面在平面下 → 正确滤除 |

### vs lightning-lm g2p5

Sapphire 的 occupancy grid 在 ray casting 层面参考了 lightning-lm g2p5 的设计，但在过滤层面做了根本性改造。

**采纳的设计**：

| 模块 | 来源 | 说明 |
|------|------|------|
| 斜率插值射线遍历 | g2p5 `SetMissPoint()` | 循环内无浮点除法，仅加法+乘法 |
| hit_cnt / visit_cnt 比值模型 | g2p5 `SubGrid` | 比 log-odds 参数更少、收敛更快 |
| 高度感知更新（d-aware） | g2p5 `SetGridHitPoint()` | 防止高射线穿透低障碍物 |
| SubGrid 懒分配 | g2p5 `SubGrid` | 16×16 分块，nullptr 懒分配 |
| Keyframe 全量重建 | g2p5 `RenderBack()` | PGO 后从 keyframe 重建整张地图 |
| 点云存本体系 | g2p5 `Keyframe::GetCloud()` | 位姿可替换，观测不变 |

**摒弃的设计**：

| g2p5 的做法 | 问题 | Sapphire 的替代 |
|-------------|------|-----------------|
| 固定地面法向量 `[0,0,1]` | 不支持斜坡 | $\hat{z}_w = R \cdot [0,0,1]^T$，跟姿态转 |
| RANSAC 地面检测 | 坡面法向量 $z < 0.99$ 就失败回退 | 不需要地面检测，纯几何过滤 |
| 360° 角度分桶（1°精度） | 精度损失，每角度只保留一个最近障碍 | 全量逐点 ray casting |
| `default_floor_height` 硬编码 | 换场景需重新调参 | 姿态平面自适应 |
| 射线方向限于本体 xy 平面 | 未考虑俯仰角 | 世界系 3D 射线 |
| 地面点不做 ray cast | 幽灵障碍物不衰减 | 地面点 ray cast 标 free |

**g2p5 的 RANSAC 失败分析**：

g2p5 的 `DetectPlaneCoeffs()` 有两个硬性限制导致斜坡适应失败：

```cpp
// 1. 候选点过滤：只保留本体系 z 较低的点
if (pt.z < lidar_height_ + default_floor_height_)  // 斜坡上方点被排除

// 2. 水平度检查：法向量 z 分量 < 0.99 就判失败
if (coefficients->values[2] < 0.99) return false;  // ~8° 以上就放弃
```

失败后回退到 `floor_coeffs_ = Vec4d(0, 0, 1, -default_floor_height_)`，即全局水平面。

### vs 标准 log-odds

| 维度 | log-odds | hit/visit 比值 |
|------|----------|----------------|
| 参数 | 4 个（$l_{\text{free}}, l_{\text{occ}}, l_{\min}, l_{\max}$） | 1 个（`occ_threshold`） |
| 收敛速度 | 慢（需多次累积到阈值） | 快（几次 visit 即可判定） |
| 高度感知 | 无标准扩展 | 天然支持（d_min 字段） |
| 时间衰减 | 需额外实现（clamp 或 decay） | 无需（比值自带稀释） |
| 概率语义 | 有（贝叶斯后验） | 无（纯频率统计） |

log-odds 的概率语义更严谨，但对导航场景来说 hit/visit 比值够用且实现更简单。

## 局限性

1. **极端姿态**：当机器人俯仰/横滚角很大时（> 30°），z 轴方向偏离竖直方向较多，过滤平面可能切到不该切的点。但对地面导航机器人来说，这通常意味着地形本身就不可通行。

2. **悬挂障碍物**：低于传感器但高于底盘的障碍物（如低矮树枝）会被保留为 occupied。如果不需要，可以加 $d_{\max}$ 上限过滤。

3. **动态障碍物**：此方案不区分静态/动态障碍物。对导航来说这不是问题（都该避开），但如果需要语义信息则需要额外处理。

## PGO 回环与栅格图一致性

### 问题

栅格图是增量构建的——每帧用当前位姿做 ray casting + hit/miss 更新。但 PGO（后端回环优化）会修正历史位姿，导致之前投进栅格的点位置全部失效，地图出现撕裂或重影。

核心矛盾：**增量式栅格图 vs 位姿事后修正**。

### 业界方案对比

| 方案 | 代表项目 | 思路 | 优点 | 缺点 |
|------|----------|------|------|------|
| Keyframe 重建 | lightning-lm g2p5, RTAB-Map | 回环时从 keyframe 全量重建栅格 | 实现简单，结果精确 | keyframe 多时重建耗时 |
| Submap 拼接 | Cartographer | 局部 submap 冻结，PGO 只动 submap 锚点位姿 | PGO 不碰内部结构 | 边界拼接复杂，实现量大 |
| 双层地图（Live + Global） | — | 实时层用前端位姿增量更新，全局层回环时重建 | 实时性不受影响 | 两套地图，一致性维护复杂 |

### 选择：Keyframe 重建

Sapphire 选 Keyframe 重建方案，理由：

1. Sapphire 已有 keyframe/submap 概念，可直接复用
2. Keyframe 密度低（移动一定距离才触发），数量可控
3. PGO 触发频率低，重建次数有限
4. 实现最干净，不引入新数据结构

### Keyframe 触发条件

Keyframe 在以下任一条件满足时创建：

- **平移阈值**：距离上一 keyframe 超过 0.5m
- **旋转阈值**：相对上一 keyframe 旋转超过 5°

典型城市场景下约每 2~3 秒产生一个 keyframe，室内走廊约每 5~10 秒一个。

### 关键设计：点云存本体系

Keyframe 中的点云**必须存本体系坐标**，不能存世界系。

**存世界系的问题**：

```
创建时：pt_body → pose_front → pt_world（存下来）
PGO 后：pose 变了，但原始 pt_body 已丢失
       → 无法用新位姿重新投影，该 keyframe 作废
```

**存本体系**：

```
创建时：pt_body（直接存）
重建时：pt_body → pose_pgo → pt_world_new → 投到栅格
```

位姿可以随时换，观测数据不变。

### Keyframe 数据结构

```cpp
struct OccupancyKeyframe {
    SE3 pose;                    // PGO 优化位姿（重建时从 ISAM2 读取）
    Vec3 sensor_origin_body;     // 传感器原点（本体系）
    vector<Vec3> cloud_body;     // 完整点云（本体系），含地面点和障碍点
};
```

重建时用 `pose` 和 `z_hat = pose.R.col(2)` 重新计算每个点的 $d$ 值，重新分类 hit/miss。不需要在创建时预分类——姿态变了分类也会变。

### 幽灵障碍物问题与修复

**问题**：某栅格被标记为 occupied（如坡顶姿态跳变导致），后续帧中该区域的点 $d$ 值都低于 $-h_c$，被归类为 miss。但如果这些 miss 射线不参与更新，occupied 状态永远不衰减——形成幽灵障碍物。

**修复**：所有点（包括 $d < -h_c$ 的地面点）都做 ray casting。地面点的射线标 free 不标 occupied：

```
对所有点：
    ray cast 从传感器原点到该点 → 沿途栅格按 d-aware 规则更新（miss）
    if d >= -h_clearance:
        终点栅格按 d-aware 规则更新（hit）
    // d < -h_clearance 的点：射线仍然清扫路径，但不标 hit
```

在 hit/visit 比值模型中，地面点的 miss 射线会增大 `visit_cnt`，稀释 `hit_cnt / visit_cnt` 比值，幽灵障碍物在几次更新后自动清除。

### 重建流程

```
PGO 触发（回环检测成功）
    ↓
清空 occupancy grid（释放所有 SubGrid data_，重置为 nullptr）
    ↓
for each keyframe:
    pose = isam2.getOptimizedPose(kf.id)
    R = pose.rotation_matrix()
    t = pose.translation()
    z_hat = R.col(2)
    sensor_origin = pose * kf.sensor_origin_body
    d_sensor = z_hat.dot(sensor_origin - t)

    for p_body in kf.cloud_body:
        p_world = pose * p_body
        d = z_hat.dot(p_world - t)
        is_hit = (d >= -h_clearance)

        // Bresenham 射线遍历 + d 值插值
        N = max(|dx|, |dy|)
        d_step = (d - d_sensor) / N
        d_ray = d_sensor

        for each cell along ray:
            d_ray += d_step
            // d-aware miss 更新
            if cell.hit_cnt > 3:
                if d_ray < cell.d_min:
                    cell.visit_cnt++
            else:
                cell.visit_cnt++
                cell.d_min = std::min(cell.d_min, d_ray)

        // 终点 hit 更新（仅障碍物点）
        if is_hit:
            if d < cell.d_min:
                cell.d_min = d
                cell.hit_cnt++
                cell.visit_cnt++
    ↓
新地图就绪，替换旧地图
```

### 实时导航的衔接

重建是异步的（后台线程），不影响前端。如果担心重建期间导航地图过时：

```
前端（每帧）：用 propagation 位姿做增量更新
              只维护机器人周围局部区域（如 ±30m）
              供局部避障使用

后端（回环时）：从 keyframe 全量重建 global map
              重建完成后替换 global map
              前端 local patch 在下一帧与 global map 同步
```

**前后端同步机制**：

```
重建完成后：
    1. 后端写 global_map 到新 buffer（double-buffering）
    2. 设置 atomic flag：global_updated = true
    3. 前端下一帧检测到 flag，执行：
       - 清空 local_map
       - 从 global_map 拷贝机器人周围 ±local_range 区域
       - 重置 global_updated = false
    4. 后续帧继续增量更新 local_map
```

这样前端始终保持低延迟（只维护局部），后端重建不影响实时性。

### 性能估算

假设：
- 1000 个 keyframe
- 每个 keyframe ~5000 个点
- 栅格分辨率 0.1m，范围 200m × 200m

重建计算量：$1000 \times 5000 = 5 \times 10^6$ 次 ray casting + grid update，modern CPU 上约 200 ~ 500ms。

如果 keyframe 过多导致重建变慢，可对存储的点云做 voxel downsample（0.2m），点数降一个数量级。

## 改进项优先级

| 优先级 | 改进 | 说明 |
|--------|------|------|
| P0 | 所有点都做 ray casting（地面点标 free） | 消除幽灵障碍物 |
| P0 | 点云存本体系 | PGO 重建的前提 |
| P0 | d-aware 更新规则 | 防止高射线穿透低障碍物 |
| P1 | 对已 occupied 栅格做反向验证 | 如果当前帧该位置所有点 d < -h_c，主动增大 visit_cnt 加速清除 |
| P2 | 帧间姿态平滑（z_hat 指数加权） | 减少过滤平面跳变噪声 |
| P2 | 高度分层投影 | 区分低矮台阶和墙壁，用于 costmap |
| P3 | SlidingMap 数据结构 | 大场景（>100m）时的内存优化，参考 ROG-Map |
