# MINS LiDAR Updater 算法流程分析

## 文件: `src/MINS/mins/src/update/lidar/UpdaterLidar.cpp`
## 对应源码版本: MINS (2023 Woosik Lee, Guoquan Huang)

---

## 1. 整体架构概述

`UpdaterLidar` 是 MINS 中处理 LiDAR 点云的 EKF 更新模块。其核心思想是 **point-to-plane 观测模型**：将当前扫描点投影到局部地图中，通过点到平面的距离构造残差，利用 MSCKF 框架将观测信息融合到 IMU 状态和 clone 位姿的估计中。

### 三个队列（管道设计）

| 队列 | 含义 |
|------|------|
| `stack_lidar_raw` | 原始接收的点云，等待预处理（运动畸变去除、降采样、ikd-tree 初始化） |
| `stack_lidar_new` | 预处理完毕，等待进行 EKF 更新的点云 |
| `stack_lidar_used` | 已完成 EKF 更新，等待注册到 ikd-tree 地图中的点云 |

---

## 2. 算法主流程

### 2.1 `feed_measurement()` — 数据接收与预处理 (Line 50-104)

```
点云输入 → stack_lidar_raw
    │
    ├── 第1步: 时间窗口过滤
    │     - 若点云时间+时间偏移 < oldest_clone_time → 丢弃（太老）
    │     - 若点云时间+时间偏移+0.05s > newest_clone_time → 等待（太新，IMU 还未来得及传播）
    │     - 0.05s 延迟是为了等 IMU 传播稳定后再融合
    │
    ├── 第2步: 运动畸变去除 (remove_motion_blur)
    │     - 利用 IMU 的 CPI (Continuous Preintegration) 对每个点逐点补偿运动
    │     - 失败则跳过这帧
    │
    ├── 第3步: 降采样 (downsample)
    │     - 若 raw_do_downsample=true，按 raw_downsample_size 体素降采样
    │
    ├── 第4步: ikd-tree 地图初始化
    │     - 若 ikd-tree 未初始化 OR 上次更新距今 > 1s
    │     → init_map_local() 用当前扫描构建局部地图
    │     - 初始化后直接丢弃当前帧（不参与 EKF 更新）
    │
    └── 第5步: 移入 stack_lidar_new 等待 EKF 更新
```

### 2.2 `try_update()` — EKF 更新调度 (Line 106-125)

```
for each lidar in stack_lidar_new:
    │
    ├── update(lidar, ikd_data, Chi)  // 核心 EKF 更新
    │     ├── 成功 → 移入 stack_lidar_used
    │     └── 失败 → 保留在 stack_lidar_new 等待下次尝试
    │
for each lidar in stack_lidar_used:
    └── register_scan()  // 将已更新后的扫描注册到 ikd-tree 地图
```

### 2.3 `propagate_map_frame()` — 地图帧推进 (Line 147-160)

当滑动窗口滑动导致最老 clone 被边缘化时，若 ikd-tree 地图的锚定时间已经比 window 内第3老的 clone 还老，则需要将地图整体变换到最新 clone 坐标系下，防止地图被边缘化丢弃。

---

## 3. `update()` 核心算法详解 (Line 163-408)

这是 LiDAR EKF 更新的核心。流程分为以下步骤：

### 3.1 坐标系定义

代码注释中给出了清晰的帧关系图 (Line 164-167):

```
  L(Map at pMinM)                     L (LiDAR 帧)
  |                                   |
  A (Anchor, IMU clone) -- C -- C -- I (当前 IMU clone)
```

- **G**: 全局/世界坐标系 (Global)
- **I**: 当前 IMU 位姿对应帧（通过插值获得）
- **A**: 地图锚定的 Anchor IMU clone 位姿
- **L**: LiDAR 传感器帧（通过外参 `I→L` 与 IMU 关联）
- **M**: 地图帧 Map Frame（与 Anchor 帧固定绑定，等价于在 A 帧处定义的 L 帧）

关键关系：
- `{M}` 实际上是 `{L}` 在 Anchor clone 时间处的表示
- `RLtoM` = LiDAR相对于Map的旋转，即 LiDAR 新扫描相对于 Map 的旋转

### 3.2 状态向量提取

```
H_order = [calibration, timeoffset, orderI, orderA]
```

其中：
- `calibration`: LiDAR-IMU 外参 (PoseJPL, 6维: 旋转3 + 平移3)
- `timeoffset`: LiDAR-IMU 时间偏移 (1维)
- `orderI`: 当前 LiDAR 时间对应的 IMU 插值位姿相关的状态变量
- `orderA`: Map Anchor 时间对应的 IMU 插值位姿相关的状态变量

通过 `state->get_interpolated_jacobian()` 获取插值后的位姿及其相对于各状态量的雅可比 `dTdxI`、`dTdxA`。

### 3.3 坐标系变换预处理 (Line 222-249)

构建以下变换矩阵（同时维护当前估计值 `_est` 和 FEJ 值 `_fej`）：

```
RItoL = calibration->Rot()           // IMU→LiDAR 旋转
pIinL = calibration->pos()           // IMU→LiDAR 平移
RLtoI = RItoL^T
pLinI = -RLtoI * pIinL              // LiDAR 在 IMU 下的位置

// 当前 LiDAR 帧
RGtoL = RItoL * RGtoI               // Global→LiDAR
pLinG = pIinG + RGtoI^T * pLinI     // LiDAR 在 Global 下的位置

// Map 帧（= Anchor 帧的 LiDAR）
RGtoM = RItoL * RGtoA               // Global→Map
pMinG = pAinG + RGtoA^T * pLinI     // Map 在 Global 下的位置

// LiDAR→Map 变换（关键！）
RLtoM = RGtoM * RLtoG               // LiDAR→Map 旋转
pLinM = RGtoM * (pLinG - pMinG)     // LiDAR 在 Map 下的位置
```

### 3.4 观测模型：Point-to-Plane 残差

#### 3.4.1 平面拟合 (Line 286-293)

对每个 LiDAR 点 `pfinL`：
1. `get_neighbors()` — 在 ikd-tree 地图中搜索该点最近邻（默认 `map_ngbr_num` 个）
2. `compute_plane()` — 对邻近点做 PCA/平面拟合，得到平面参数 `plane_abcd = [a, b, c, d]^T`

平面方程：**a·x + b·y + c·z + d = 0**，其中 `[a,b,c]^T` 是平面单位法向量（归一化后），`d` 是平面到原点的有符号距离。

#### 3.4.2 残差定义 (Line 305-314)

残差向量 `res` 维度 = `map_ngbr_num + 1`：

```
对每个地图邻居点 pninM (j = 0...N-1):
    res(j) = 0 - (a·pninM_x + b·pninM_y + c·pninM_z + d)
           = -plane_abcd.head(3)^T * pninM - plane_abcd(3)
           = -(ax + by + cz + d)

对当前扫描点 pfinM:
    res(N) = -(a·pfinM_x_est + b·pfinM_y_est + c·pfinM_z_est + d)
           = -plane_abcd.head(3)^T * pfinM_est - plane_abcd(3)
```

**核心思想**：地图邻居点理论上应落在拟合平面上（残差→0），当前扫描点也应落在地图的同一个平面上。残差度量的是各点到拟合平面的有符号距离。

#### 3.4.3 当前扫描点的地图坐标（核心方程）

```
pfinM = pLinM + RLtoM * pfinL                                    (1)
      = pLinM_fej + RLtoM_fej * pfinL   (FEJ版本用于雅可比)       (2)
```

这就是 **LiDAR 观测方程的核心**：将 LiDAR 坐标系下的点 `pfinL` 通过 `{L}→{M}` 变换投影到 Map 坐标系，得到 `pfinM`。

**{L}→{M} 的物理含义**：

```
RLtoM = RGtoM * RLtoG = (RItoL * RGtoA) * (RItoL * RGtoI)^T
```

即：LiDAR扫描帧 → Global → Map。等价于通过两帧IMU位姿（当前I和Anchor A）及LiDAR外参共同确定。

### 3.5 雅可比推导 (Line 319-330)

#### 3.5.1 链式法则

```
∂res/∂state = ∂res/∂pfinM · ∂pfinM/∂state

其中 ∂res/∂pfinM = plane_abcd.head(3)^T  (1×3)  // dz_dpfinM_bottom_row
```

#### 3.5.2 对当前 IMU 位姿 (I) 的雅可比 (Line 321-323)

```
dpfinM_dI = [∂pfinM/∂θ_I, ∂pfinM/∂p_I]  (3×6)

∂pfinM/∂θ_I = -RGtoM * RItoG * skew(RLtoI * (pfinL - pIinL))   (3×3)
∂pfinM/∂p_I = RGtoM                                              (3×3)
```

**物理解释**：
- 旋转部分：IMU 旋转误差通过 `RLtoI * (pfinL - pIinL)`（点在IMU系下的杆臂向量）被杠杆放大
- 平移部分：IMU 平移直接影响 Map 系下的点位置，需通过 `RGtoM` 旋转到 Map 系

#### 3.5.3 对 Anchor IMU 位姿 (A) 的雅可比 (Line 324-326)

```
dpfinM_dA = [∂pfinM/∂θ_A, ∂pfinM/∂p_A]  (3×6)

∂pfinM/∂θ_A = RItoL * skew(RGtoA * (pIinG + RItoG * pLinI - pAinG + RLtoG * pfinL))  (3×3)
∂pfinM/∂p_A = -RGtoM                                                                    (3×3)
```

**物理含义**：
- Anchor 平移增加 → Map 原点移动 → pfinM 反向移动（负号）
- Anchor 旋转误差通过长杠杆臂（Global 下 Anchor 到 LiDAR 点的矢量）被放大

#### 3.5.4 对外参 (calibration) 的雅可比 (Line 327-329)

```
dpfinM_dcalib = [∂pfinM/∂θ_calib, ∂pfinM/∂p_calib]  (3×6)

∂pfinM/∂θ_calib = skew(RGtoM*(pIinG-pAinG)) - skew(RLtoM*(pIinL-pfinL)) + RLtoM·skew(pIinL-pfinL)
∂pfinM/∂p_calib = I₃ - RLtoM
```

#### 3.5.5 链式法则汇总 (Line 343-350)

```
H_row = dz_dpfinM_bottom_row * dpfinM_dI * dTdxI     ← 当前帧IMU位姿相关
      + dz_dpfinM_bottom_row * dpfinM_dA * dTdxA     ← Anchor帧IMU位姿相关
      + dz_dpfinM_bottom_row * dpfinM_dcalib          ← 外参（可选）
```

其中 `dTdxI`、`dTdxA` 是插值位姿对各自 IMU clone 状态的雅可比（MSCKF 的标准插值链式法则）。

#### 3.5.6 插值不确定度处理 (Line 332-337)

若当前 LiDAR 时间不是精确落在某个 IMU clone 上，则 IMU 插值引入额外不确定性 `intr_std`：

```
intr_std = sqrt(ie_o * ||HI_ori||² + ie_p * ||HI_pos||²)
```

其中 `ie_o`、`ie_p` 是插值旋转/平移协方差，通过 `state->intr_ori_cov()` / `state->intr_pos_cov()` 获取。

#### 3.5.7 平面参数的雅可比 (Line 305-314)

对地图邻居点，其残差对平面参数 abc 的雅可比：
```
dz_dplane_abc(j, :) = pninM^T  (1×3)
```

对当前扫描点（FEJ版本）：
```
dz_dplane_abc(N, :) = pfinM_fej^T  (1×3)
```

### 3.6 噪声白化 (Line 353-359)

```
raw_noise = ||plane_abc|| * raw_noise_std + intr_std     // 扫描点噪声
map_noise = ||plane_abc|| * map_noise_std + intr_std     // 地图点噪声
```

**重要设计**：噪声通过平面法向量模长 `||plane_abc||` 缩放——法向量越接近单位向量，噪声越接近原始噪声参数。再加上插值不确定性 `intr_std`。

对 H、dz_dplane_abc、res 各行除以对应噪声标准差实现白化。

### 3.7 零空间投影 (Line 362)

```cpp
StateHelper::nullspace_project_inplace(dz_dplane_abc, H, res);
```

**这是解决平面拟合参数引入额外自由度的关键步骤。**

`dz_dplane_abc` 是残差对平面参数 abc 的雅可比（`(N+1)×3`），其列空间张成了平面参数的观测方向。通过零空间投影，将 `H` 和 `res` 投影到 `dz_dplane_abc` 的左零空间中，消除对平面参数的依赖，仅保留对状态的约束。

**数学表达**：
```
设 A = dz_dplane_abc, 找到左零空间 N s.t. N^T * A = 0
则 H_proj = N^T * H, res_proj = N^T * res
```

这样处理后，每个点对的残差从 `N+1` 维降为 `N-2` 维（因为 3 个平面参数自由度已被消去），保留了纯粹的位姿/外参约束。

### 3.8 Chi² 检验 (Line 365)

```
chi->Chi2Check(P, H, res, R)
```

对该点对的残差进行卡方检验，通过则保留，不通过则丢弃（可能是离群点或动态物体）。

### 3.9 多线程并行处理 (Line 269-371)

将所有 LiDAR 点按 `num_threads`（至少4个）均匀分块，每线程独立执行：邻居搜索 → 平面拟合 → 雅可比计算 → 零空间投影 → Chi² 检验。结果存入 `vec_lin`。

### 3.10 组装与测量压缩 (Line 384-402)

```
1. 收集所有通过检验的点对的 res 和 H → 组装成大矩阵
2. StateHelper::measurement_compress_inplace(H_big, res_big)
   - 对 H_big 做 QR 分解压缩，降维到 min(total_res, total_hx)
3. StateHelper::EKFUpdate(state, H_order, H_big, res_big, R_big, "LIDAR")
   - 标准 EKF 更新
```

---

## 4. LiDAR 观测方程总结

### 完整观测模型（数学形式）

设 LiDAR 点在自身坐标系下为 $\mathbf{p}^L$，地图系下的对应点为 $\mathbf{p}^M$：

$$\mathbf{p}^M = \mathbf{p}_L^M + \mathbf{R}_L^M \cdot \mathbf{p}^L$$

其中：

$$\mathbf{R}_L^M = \mathbf{R}_G^M \cdot (\mathbf{R}_G^L)^T = (\mathbf{R}_I^L \mathbf{R}_G^I|_A) \cdot (\mathbf{R}_I^L \mathbf{R}_G^I|_{\text{curr}})^T$$

$$\mathbf{p}_L^M = \mathbf{R}_G^M \cdot (\mathbf{p}_L^G - \mathbf{p}_M^G)$$

$$\mathbf{p}_L^G = \mathbf{p}_I^G + (\mathbf{R}_G^I)^T \cdot \mathbf{p}_L^I = \mathbf{p}_I^G + (\mathbf{R}_G^I)^T \cdot (-\mathbf{R}_L^I \cdot \mathbf{p}_I^L)$$

$$\mathbf{p}_M^G = \mathbf{p}_A^G + (\mathbf{R}_G^A)^T \cdot \mathbf{p}_L^I$$

### Point-to-Plane 残差

对拟合平面 $ax+by+cz+d=0$，残差定义为点到平面的有符号距离：

$$r = a \cdot p_x^M + b \cdot p_y^M + c \cdot p_z^M + d = \mathbf{n}^T \cdot \mathbf{p}^M + d$$

其中 $\mathbf{n} = [a,b,c]^T$ 为平面法向量。

### 雅可比链式法则

$$\frac{\partial r}{\partial \mathbf{x}} = \frac{\partial r}{\partial \mathbf{p}^M} \cdot \frac{\partial \mathbf{p}^M}{\partial \mathbf{T}_I} \cdot \frac{\partial \mathbf{T}_I}{\partial \mathbf{x}} + \frac{\partial r}{\partial \mathbf{p}^M} \cdot \frac{\partial \mathbf{p}^M}{\partial \mathbf{T}_A} \cdot \frac{\partial \mathbf{T}_A}{\partial \mathbf{x}} + \frac{\partial r}{\partial \mathbf{p}^M} \cdot \frac{\partial \mathbf{p}^M}{\partial \mathbf{x}_{\text{calib}}}$$

其中 $\mathbf{T}_I = (\mathbf{R}_G^I, \mathbf{p}_I^G)$ 是当前帧 IMU 位姿，$\mathbf{T}_A = (\mathbf{R}_G^A, \mathbf{p}_A^G)$ 是 Anchor 帧位姿，$\mathbf{x}_{\text{calib}} = (\boldsymbol{\theta}_I^L, \mathbf{p}_I^L)$ 是 LiDAR-IMU 外参。

$$\frac{\partial r}{\partial \mathbf{p}^M} = \mathbf{n}^T$$

---

## 5. 代码-模块对应表

| 模块 | 源文件 | 核心功能 |
|------|--------|----------|
| `UpdaterLidar` | `UpdaterLidar.cpp/h` | 主调度器：feed_measurement / try_update / update / propagate_map_frame |
| `LidarHelper` | `LidarHelper.cpp/h` | 工具函数：运动畸变去除、降采样、地图初始化、点云变换、邻居搜索、平面拟合、扫描注册、地图传播 |
| `LiDARData` | `LidarTypes.h` | LiDAR 数据结构：时间戳、ID、原始点云、去畸变点云、地图系点云 |
| `iKDDATA` | `LidarTypes.h` | ikd-tree 地图数据：KD_TREE 指针、锚定时间戳、最后更新时间 |
| `StateHelper` | `state/StateHelper.h` | EKF 工具：insert_map、nullspace_project_inplace、measurement_compress_inplace、EKFUpdate、get_marginal_covariance |
| `UpdaterStatistics` | `update/UpdaterStatistics.h` | Chi² 检验统计量管理 |
| `State` | `state/State.h` | 系统状态管理：clones、外参、时间偏移、插值查询 |

---

## 6. 数据流总图

```
LiDAR 点云 (ROS msg)
    │
    ▼
feed_measurement()
    │
    ├── 时间窗口检查 (vs. sliding window clones)
    ├── remove_motion_blur()   ─── IMU CPI 畸变补偿
    ├── downsample()           ─── 体素降采样
    ├── init_map_local()       ─── ikd-tree 初始化（仅首帧/重定位时）
    │
    ▼ stack_lidar_new
    │
try_update()
    │
    ▼
update() ─── 对每个点：
    │
    ├── get_interpolated_jacobian(I)     ─── 获取当前帧插值位姿+雅可比
    ├── get_interpolated_jacobian(A)     ─── 获取 Anchor 帧插值位姿+雅可比
    ├── 预计算变换矩阵 RLtoM, pLinM
    ├── get_neighbors()                  ─── ikd-tree KNN 搜索
    ├── compute_plane()                  ─── PCA 平面拟合 → plane_abcd
    ├── 残差计算: res = -(n^T·p + d)
    ├── 雅可比计算: ∂res/∂state (链式法则)
    ├── 噪声白化 (raw_noise / map_noise)
    ├── nullspace_project_inplace()      ─── 消去平面参数自由度
    ├── Chi2Check()                      ─── 离群点剔除
    │
    ▼ 多线程汇总
    │
    ├── 组装 H_big, res_big
    ├── measurement_compress_inplace()   ─── QR 压缩
    ├── EKFUpdate()                      ─── 标准 EKF 状态更新
    │
    ▼ stack_lidar_used
    │
register_scan()                          ─── 更新后点云注册到 ikd-tree

propagate_map_frame()                    ─── 地图锚定帧随滑动窗口推进
```

这就是 MINS 中 LiDAR 更新的完整算法流程。核心创新点在于：
1. **point-to-plane** 观测模型，避免了点到点的强假设
2. **零空间投影**消除平面参数自由度，将约束精确投射到位姿/外参空间
3. **MSCKF 滑动窗口 + FEJ**保证了一致性
4. **插值不确定性**补偿了非精确 clone 时间的位姿误差
5. **地图邻居点同时参与残差和零空间投影**，天然处理了地图点不确定性