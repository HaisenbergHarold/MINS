## `run_bag.cpp` 算法流程与模块对应分析

MINS (*Multisensor-aided Inertial Navigation System*) 是一个基于 MSCKF（多状态约束卡尔曼滤波）的多传感器融合定位系统。`run_bag.cpp` 是其 **离线 rosbag 回放模式** 的主入口。下面是完整的算法流程与代码模块的对应关系。

---

### 一、系统总体架构

```
run_bag.cpp (主流程调度)
    │
    ├── system_setup()          → 参数加载 + 各模块初始化
    ├── main() 消息循环          → 按时间顺序分派传感器测量
    │      ├── IMU              → SystemManager::feed_measurement_imu()
    │      ├── CAM              → SystemManager::feed_measurement_camera()
    │      ├── WHEEL            → SystemManager::feed_measurement_wheel()
    │      ├── GPS              → SystemManager::feed_measurement_gps()
    │      ├── LiDAR            → SystemManager::feed_measurement_lidar()
    │      └── VICON            → SystemManager::feed_measurement_vicon()
    └── visualize_final()       → 最终结果打印与日志保存
```

核心类 `SystemManager` 持有 6 个子系统组件：

| 成员 | 类型 | 功能 |
|------|------|------|
| `state` | `State` | 滤波器状态（clones、协方差、标定参数） |
| `prop` | `Propagator` | IMU 正向传播与 CPI（连续预积分） |
| `initializer` | `Initializer` | 系统初始化（静态/IMU-轮速/IMU-GPS-Vision） |
| `up_cam` | `UpdaterCamera` | 视觉 MSCKF 更新 + SLAM 特征 |
| `up_gps` | `UpdaterGPS` | GNSS 位置更新（4-DOF 转换估计） |
| `up_ldr` | `UpdaterLidar` | LiDAR 点云 ICP 更新 |
| `up_vcn` | `UpdaterVicon` | Vicon 位姿更新 |
| `up_whl` | `UpdaterWheel` | 轮速里程计更新 |

---

### 二、算法流程详细分析

#### **阶段1: 系统初始化 `system_setup()`** (Line 314–470)

对应代码：`run_bag.cpp:314-470`

**步骤1.1 参数加载**
```
config_path → YamlParser → Options (所有传感器配置、滤波器参数)
```
- `OptionsEstimator`: 顶层估计器配置
- `OptionsIMU / OptionsCamera / OptionsGPS / OptionsLidar / OptionsWheel / OptionsVicon`: 各传感器配置（话题名、噪声、外参、标定使能等）
- `OptionsSystem`: 系统级配置（bag路径、起止时间、窗口大小、克隆频率等）

**步骤1.2 核心模块实例化** (`run_bag.cpp:332-334`)
```cpp
sys = make_shared<SystemManager>(op->est);      // 系统管理器
pub = make_shared<ROSPublisher>(nh, sys, op);    // ROS 可视化发布器
sim_viz = make_shared<SimVisualizer>(nh, sys);   // 仿真可视化（真值发布）
```

`SystemManager` 构造函数 (`SystemManager.cpp:60-80`) 依次创建：
```
State → Propagator → UpdaterCamera/UpdaterGPS/UpdaterLidar/UpdaterVicon/UpdaterWheel → Initializer
```

**步骤1.3 Rosbag 预扫描与建图** (`run_bag.cpp:342-461`)

遍历 bag 中所有消息，建立 **按时间排序的 `msgs` 向量**，同时为每个相机建立 **`cam_map`**（`map<double, int>`，用于双目匹配快速查找）。

关键数据结构：
- `msgs`: `vector<rosbag::MessageInstance>` — 所有有效传感器消息的时序列表
- `cam_map`: `vector<map<double, int>>` — `{cam_id: {timestamp → msgs索引}}`，用于后续双目配对查找
- `used_index`: 记录已被使用的（双目右图）消息索引，避免重复处理

**步骤1.4 时间窗口控制** (`run_bag.cpp:463-466`)
```cpp
time_init  = bag起始时间 + bag_start偏移
time_finish = bag_durr < 0 ? bag结束时间 : time_init + bag_durr
```

---

#### **阶段2: 主消息处理循环** (`main()`, Line 91–194)

对应代码：`run_bag.cpp:91-194`

按时间顺序遍历 `msgs`，根据消息的 topic 分派到对应处理模块。

##### **2.1 IMU 消息处理** (Line 106–118)

```
msgs[i] 属于 IMU topic
    │
    ▼
ROSHelper::Imu2Data()  →  ov_core::ImuData
    │
    ▼
SystemManager::feed_measurement_imu(imu)       ← SystemManager.cpp:82-134
    │
    ├── [1] prop->feed_imu(imu)                ← 存入 IMU 缓冲区
    │
    ├── [2] initializer->try_initializtion()   ← 尝试初始化（若尚未初始化，返回 false 则跳过后续）
    │
    ├── [3] 检查是否需要创建新 clone:
    │    │  get_next_clone_time()              ← SystemManager.cpp:265-394
    │    │    ├── compute_accelerations()       ← 计算线/角加速度（用于动态克隆决策）
    │    │    ├── dynamic_cloning()             ← 基于加速度自适应调整克隆频率和插值阶数
    │    │    └── 在传感器测量时间附近确定 clone 时刻
    │    │
    │    ├── prop->propagate(clone_time)        ← IMU 积分传播到 clone 时刻
    │    ├── StateHelper::augment_clone(state)  ← 随机克隆：将当前 IMU 位姿复制为新的 clone（MSCKF 核心操作）
    │    ├── up_ldr->propagate_map_frame()      ← LiDAR 局部地图坐标系传播
    │    ├── StateHelper::marginalize_old_clone() ← 滑动窗口边缘化：删除最旧的 clone
    │    ├── prop->reset_cpi(state->time)       ← 重置连续预积分（CPI）基准
    │    ├── state->flush_old_data()            ← 清理旧的插值/多项式数据
    │    └── print_status()                     ← 打印当前估计状态、传感器频率、标定参数
    │
    ├── [4] prop->propagate(imu.timestamp)      ← 传播到最新 IMU 时间戳
    ├── [5] state->add_polynomial()             ← 构建多项式插值数据（用于传感器时间对齐）
    └── [6] 返回 clone_time > 0 触发可视化
```

**核心算法模块：**
- **CPI（连续预积分）**: `Propagator` 使用 RK4 数值积分实现 IMU 运动学传播，同时计算状态转移矩阵 Φ 和离散噪声协方差 Qd
- **MSCKF 随机克隆**: `StateHelper::augment_clone()` 将当前 IMU 位姿作为新 clone 添加到滑动窗口，并扩展协方差矩阵（考虑时间偏移的雅可比）
- **滑动窗口边缘化**: `StateHelper::marginalize_old_clone()` 删除超出窗口大小的旧 clone，将其信息压缩到剩余状态中
- **动态克隆**: `dynamic_cloning()` 根据加速度大小自适应选择克隆频率（hz）和插值阶数（固定为 3 阶），以平衡计算量与精度
- **多项式插值**: `State::add_polynomial()` 为后续传感器更新提供任意时刻的位姿插值（用于多传感器时间对齐）

##### **2.2 相机消息处理** (Line 121–129)

```
msgs[i] 属于 CAM topic
    │
    ▼
feed_camera(cam_id, idx)                        ← run_bag.cpp:207-275
    │
    ├── 单目模式:
    │    ROSHelper::Image2Data() → CameraData
    │    sys->feed_measurement_camera(cam)      ← SystemManager.cpp:136-145
    │       ├── up_cam->feed_measurement(cam)   ← 存储测量，提取特征
    │       └── up_cam->try_update(cam_id)      ← 触发 MSCKF 视觉更新
    │
    └── 双目模式:
         find_stereo_pair()                      ← run_bag.cpp:277-312
            └── 在 cam_map 中查找时间最接近的立体配对图像（阈值 0.01s）
         sys->feed_measurement_camera(cam_stereo)
```

**核心算法模块：**
- **UpdaterCamera** (`update/cam/UpdaterCamera.cpp`):
  - 特征提取与跟踪（光流/描述子匹配）
  - MSCKF 视觉更新：通过 nullspace 投影消除特征位置的依赖，构建关于 clone 位姿的观测约束
  - SLAM 特征初始化与更新：对于长期跟踪的特征，使用 `StateHelper::initialize()` 将其加入状态向量
  - 外参/内参/时间偏移在线标定

##### **2.3 轮速里程计** (Line 132–146)

```
msgs[i] 属于 Wheel topic
    │
    ▼
ROSHelper::JointState2Data() / Odometry2Data() → WheelData
    │
    ▼
sys->feed_measurement_wheel(data)               ← SystemManager.cpp:167-175
    ├── up_whl->feed_measurement(wheel)         ← 2D 轮速测量预处理
    └── up_whl->try_update()                   ← EKF 更新（预积分约束）
```

**核心算法模块：**
- **UpdaterWheel** (`update/wheel/UpdaterWheel.cpp`): 基于双轮差速模型的 2D 平面运动约束，可在线标定左右轮半径、轴距、外参、时间偏移

##### **2.4 GPS 消息处理** (Line 148–166)

```
msgs[i] 属于 GPS topic
    │
    ▼
ROSHelper::NavSatFix2Data() / PoseStamped2Data() → GPSData
    │
    ▼
sys->feed_measurement_gps(data, isGeodetic)     ← SystemManager.cpp:187-218
    ├── 首次 GPS 设置 datum 参考点（WGS-84 → ENU 转换基准）
    ├── MathGPS::GeodeticToEnu()                ← 大地坐标转东北天
    ├── up_gps->add_keyframes()                 ← 初始化阶段：积累关键帧用于 GNSS-VIO 对齐
    ├── up_gps->feed_measurement() + try_update() ← 4-DOF 位姿图优化更新
    └── 初始化完成后清理 keyframes
```

**核心算法模块：**
- **UpdaterGPS** (`update/gps/UpdaterGPS.cpp`): 
  - 4-DOF 变换估计（x, y, z, yaw）：VIO 局部坐标系与 GPS ENU 全局坐标系的对齐
  - 利用 PoseJPL_4DOF 和 JPLQuat_4DOF 实现去耦合的位姿图更新
  - 外参（杆臂）和时间偏移在线标定

##### **2.5 LiDAR 消息处理** (Line 168–179)

```
msgs[i] 属于 LiDAR topic
    │
    ▼
ROSHelper::rosPC2pclPC() → PointCloud<PointXYZ>
    │
    ▼
sys->feed_measurement_lidar(data)               ← SystemManager.cpp:177-185
    ├── up_ldr->feed_measurement(lidar)         ← 点云去畸变（deskew）
    └── up_ldr->try_update()                   ← LiDAR ICP 更新
```

**核心算法模块：**
- **UpdaterLidar** (`update/lidar/UpdaterLidar.cpp`):
  - 基于 **ikd-Tree** 的高效 LiDAR 局部地图管理（增量 kd-tree）
  - 点对平面/点对边缘的 ICP 配准
  - LiDAR 外参和时间偏移在线标定
  - `propagate_map_frame()`: 将局部 LiDAR 地图坐标系对齐到最新的 clone 时间

##### **2.6 Vicon 消息处理** (Line 181–193)

```
msgs[i] 属于 Vicon topic
    │
    ▼
ROSHelper::PoseStamped2Data() → ViconData
    │
    ▼
sys->feed_measurement_vicon(data)               ← SystemManager.cpp:157-165
    ├── up_vcn->feed_measurement(vicon)         ← 存储位姿测量
    └── up_vcn->try_update()                   ← EKF 位姿更新
```

---

#### **阶段3: 最终处理** (Line 196–201)

```
sys->visualize_final()                          ← SystemManager.cpp:601-710
    ├── 打印总处理时间、总行驶时间和距离
    ├── 打印各传感器频率统计
    ├── 打印所有标定结果：
    │    ├── 相机：外参 T_imu_cam、内参、畸变系数、时间偏移
    │    ├── GPS：杆臂 p_GinI、时间偏移
    │    ├── LiDAR：外参 T_imu_lidar、时间偏移
    │    ├── Vicon：外参 T_imu_vicon、时间偏移
    │    └── Wheel：外参 T_imu_wheel、内参（轮半径/轴距）、时间偏移
    └── 保存轨迹和时间分析文件
```

---

### 三、关键算法模块总结

| 模块 | 源文件 | 核心功能 |
|------|--------|----------|
| **主入口** | `run_bag.cpp` | rosbag 读取、消息解析、时间顺序调度 |
| **系统管理器** | `core/SystemManager.cpp` | 传感器数据路由、克隆策略、可视化控制 |
| **IMU传播** | `state/Propagator.cpp` | RK4 数值积分、状态转移矩阵 & 噪声协方差计算、CPI 预积分 |
| **状态** | `state/State.cpp` | 滑动窗口 clones 管理、多项式插值、协方差存储 |
| **状态操作** | `state/StateHelper.cpp` | EKF 传播/更新、clone 增廣、边缘化、特征初始化的 nullspace/Givens 旋转 |
| **初始化** | `init/Initializer.cpp` | 多传感器联合初始化（IMU、IMU-轮速、IMU-视觉-GPS） |
| **视觉更新** | `update/cam/UpdaterCamera.cpp` | MSCKF 视觉更新、SLAM 特征管理 |
| **GPS更新** | `update/gps/UpdaterGPS.cpp` | 4-DOF 位姿图、WGS-84→ENU 转换 |
| **LiDAR更新** | `update/lidar/UpdaterLidar.cpp` | ikd-Tree、ICP 配准、点云去畸变 |
| **轮速更新** | `update/wheel/UpdaterWheel.cpp` | 差速轮模型约束 |
| **Vicon更新** | `update/vicon/UpdaterVicon.cpp` | 外部位姿测量融合 |
| **参数配置** | `options/Options*.cpp` | YAML 参数解析与验证 |
| **辅助工具** | `utils/` | 数据集读取、日志记录、计时分析、ROS 消息转换 |

---

### 四、数据流图总结

```
                      ┌──────────┐
  rosbag 消息序列 ───►│ run_bag  │─── 时间顺序分派 ──┐
                      │  .cpp    │                    │
                      └──────────┘                    │
                                                      ▼
                                          ┌─────────────────────┐
                                          │ SystemManager       │
                                          │  (核心调度器)        │
                                          └──────┬──────────────┘
                                                 │
        ┌────────┬────────┬─────────┬───────────┼───────────┬──────────┐
        ▼        ▼        ▼         ▼           ▼           ▼          ▼
    ┌──────┐┌──────┐┌────────┐┌────────┐┌──────────┐┌────────┐┌──────────┐
    │IMU   ││CAM   ││Wheel   ││GPS     ││LiDAR     ││Vicon   ││Options   │
    │(Prop)││(UpCam)││(UpWhl)││(UpGPS) ││(UpLidar) ││(UpVcn) ││(配置)    │
    └──┬───┘└──┬───┘└───┬────┘└───┬────┘└────┬─────┘└───┬────┘└──────────┘
       │       │        │         │          │          │
       ▼       ▼        ▼         ▼          ▼          ▼
    ┌──────────────────────────────────────────────────────────┐
    │                      State (核心状态)                     │
    │  ┌─ clones (滑动窗口位姿)                                │
    │  ├─ IMU pose (当前IMU位姿, v, bg, ba)                    │
    │  ├─ 各传感器外参/内参/时间偏移 (在线标定)                 │
    │  ├─ SLAM 特征点                                          │
    │  ├─ CPI 预积分数据 (连续时间相对位姿约束)                 │
    │  ├─ 多项式插值数据 (任意时刻位姿查询)                     │
    │  └─ Covariance (EKF 协方差矩阵)                          │
    │                                                          │
    │  StateHelper:                                            │
    │    EKFPropagation / EKFUpdate / augment_clone /          │
    │    marginalize / nullspace_project / Givens_Rotation     │
    └──────────────────────────────────────────────────────────┘
```

这就是 MINS 在 rosbag 回放模式下的完整算法流程。核心是 **MSCKF 滑动窗口框架**：IMU 传播驱动 clone 创建/边缘化，各传感器通过 `StateHelper` 的 EKF 更新机制融合到统一估计中，支持多传感器外参/内参/时间偏移的在线标定。