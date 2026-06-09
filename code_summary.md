# 家庭服务机器人饮料递送系统 — 代码汇总

## 项目文件结构

```
D:\mani_work\
├── ROS机器人开发技术期末作业.docx   # 实验题目文档
├── nav_face.cpp                      # 导航巡点 + 人脸识别 (691行)
├── pc_work.cpp                       # 桌面物品巡检 + 3D检测 + 底盘微调 (1281行)
├── grab_object.cpp                   # 机械臂抓取教学示例 (196行)
├── code_summary.md                   # 本文件
└── (暂无 CMakeLists.txt)
```

---

## 一、实验题目概述

### 核心任务链路

```
自主导航到厨房 → 获取饮料 → 自主导航到客人 → 完成递送
```

### 建议状态机

```
STEP_WAIT → STEP_GOTO_KITCHEN → STEP_VERIFY_DRINK → STEP_GRAB_DRINK
→ STEP_GOTO_GUEST → STEP_VERIFY_GUEST → STEP_DONE
```

可选异常状态：
- `STEP_RETRY_NAVI` — 导航重试
- `STEP_RETRY_GRAB` — 抓取重试
- `STEP_RETRY_VERIFY` — 确认重试
- `STEP_ABORT` — 终止

### 题目要求要点

1. **系统集成**：编写统一的 `home.launch.py` 或 `mission.launch.py`，启动仿真机器人底层、NAV2、航点管理、物体检测、抓取节点及 RViz2
2. **地图**：含 `kitchen` 和 `guest` 两个航点，地图来自自主建图 (`map.pgm` + `map.yaml`)
3. **厨房阶段**：导航→RGB确认饮料区域→3D视觉或 `objects_3d` 获取目标空间位置→底盘+机械臂抓取
4. **客人阶段**：导航到 `guest`→执行客人确认动作（人脸检测/人体存在/二维码/模拟确认信号）→终端输出 "Delivery completed"
5. **鲁棒性**：至少实现导航失败、抓取失败、客人确认失败中两类异常恢复；重试次数≥2次
6. **实验**：两种方案各3次（基础模式 vs 增强鲁棒模式），记录总时间、导航/抓取/递送成功率、失败恢复触发次数

---

## 二、nav_face.cpp — 导航巡点 + 人脸识别

- **节点名**: `waypoint_navigation_node`
- **行数**: 691
- **核心功能**: 顺序访问航点 A/B/C/HOME，每到达一个点停留并执行人脸识别

### 状态机

```
STEP_INIT → STEP_GOTO_A → STEP_STAY_A
         → STEP_GOTO_B → STEP_STAY_B
         → STEP_GOTO_C → STEP_STAY_C
         → STEP_GOTO_HOME
         → STEP_DONE
```

辅助状态: `STEP_RETRY`, `STEP_SKIP`

### 航点列表

```cpp
std::vector<std::string> waypoints = { "A", "B", "C", "HOME" };
```

### 关键参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `strategy_mode` | 1 | 1=阻塞重试，2=跳过并补访 |
| `timeout_sec` | 80.0 | 导航超时时间(s) |
| `stay_time_sec` | 5.0 | 每个航点停留时间(s) |
| `max_retry` | 2 | 最大重试次数 |

### 关键函数

| 函数 | 行号 | 功能 |
|------|------|------|
| `CamRGBCallback()` | 131 | 接收RGB图像，转发到 `/face_detector_input` |
| `SendWaypoint(name)` | 140 | 向 `/waterplus/navi_waypoint` 发布目标航点 |
| `SendNextWaypoint()` | 160 | 按 `current_index` 顺序发送下一航点 |
| `NaviDone()` | 299 | 导航成功处理，进入停留阶段 |
| `RetryCurrentWaypoint()` | 361 | 策略1：超时重试，超过max_retry则终止 |
| `SkipWaypoint()` | 398 | 策略2：跳过当前点，加入补访列表 |
| `ResultCallback()` | 423 | 监听 `/waterplus/navi_result`，"navi done"触发 NaviDone |
| `FacePosCallback()` | 435 | 监听 `/face_position`，记录人脸检测状态 |
| `HandleStayState()` | ~460 | 停留阶段处理：每秒检测人脸，70%概率授权 |
| `TimerCheck()` | 542 | 100ms定时器：超时检测 + 停留处理 |

### 人脸识别逻辑

1. 到达航点 → `staying=true` → 开始停留计时
2. 每1秒检查一次人脸：
   - 收到 `/face_position` (width>0, height>0) → `face_detected=true`
   - 随机模拟：`rand()%10 < 7` → 70%概率 "authorized"，30%概率 "unauthorized face"
3. 停留 `stay_time_sec` 秒后 → 前往下一航点

### 统计输出

- Total Time, Success Waypoints, Return HOME 状态
- Face Detect Count, Authorized Count
- Skipped Waypoints, Revisit Success Count

### 订阅/发布 Topic

| Topic | 类型 | 方向 | 用途 |
|-------|------|------|------|
| `/waterplus/navi_waypoint` | String | Pub | 发送导航目标 |
| `/waterplus/navi_result` | String | Sub | 接收导航结果 |
| `/face_detector_input` | Image | Pub | 转发RGB给人脸检测节点 |
| `/kinect2/qhd/image_raw` | Image | Sub | 接收RGB相机 |
| `/face_position` | RegionOfInterest | Sub | 接收人脸位置检测结果 |

---

## 三、pc_work.cpp — 桌面物品巡检 + 3D检测 + 底盘微调

- **节点名**: `pc_work`
- **行数**: 1281
- **核心功能**: 导航到两张桌子(TABLE_A/B)，RGB视觉确认 + 3D点云物体检测 + 底盘微调对准

### 状态机

```
STEP_INIT → STEP_GOTO_TABLE_A → STEP_STAY_A → STEP_RGB_CHECK_A
         → STEP_PC_DETECT_A → STEP_ALIGN_A
         → STEP_GOTO_TABLE_B → STEP_STAY_B → STEP_RGB_CHECK_B
         → STEP_PC_DETECT_B → STEP_ALIGN_B
         → STEP_GOTO_HOME → STEP_DONE
```

异常状态: `STEP_ABORT`

### 航点列表

```cpp
std::vector<std::string> waypoints = { "TABLE_A", "TABLE_B", "HOME" };
```

### 关键参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `max_retry` | 2 | 最大重试次数 |
| `timeout_sec` | 80.0 | 导航超时 |
| `stay_time_sec` | 5.0 | 停留时间 |
| `max_align_time_` | 8.0 | 微调超时(s) |
| `safety_threshold_` | 0.6 | 激光安全距离(m) |

### 关键函数

| 函数 | 行号 | 功能 |
|------|------|------|
| `SendWaypoint(name)` | 340 | 发布导航目标 |
| `SendNextWaypoint()` | 361 | 按状态机推进 |
| `NaviDone()` | 396 | 导航到达处理 |
| `RetryCurrentWaypoint()` | 449 | 超时重试/失败终止 |
| `ResultCallback()` | 483 | 接收 "navi done" |
| `CamRGBCallback()` | 497 | 存储最新RGB图像 |
| `doRGBCheck()` | 541 | RGB视觉确认（红/绿色检测） |
| `laserCallback()` | 636 | 更新前方最小距离 |
| `pcCallback()` | 660 | 收到点云时调用 processPointCloud |
| `processPointCloud()` | ~670 | **核心3D检测管线** |
| `doMicroAdjustment()` | 1005 | 底盘微调对准目标 |
| `HandleStayState()` | ~? | 停留计时处理 |
| `TimerCheck()` | 1151 | 200ms定时器：超时+停留+RGB+微调 |

### RGB视觉确认 (`doRGBCheck`)

1. 将 RGB 转为 HSV
2. 检测红色：`H ∈ [0,10] ∪ [170,180]`, S∈[70,255], V∈[50,255]
3. 检测绿色：`H ∈ [40,85]`, S∈[70,255], V∈[50,255]
4. 红+绿掩码取并集
5. `countNonZero(mask) > 1200` → 通过
6. 通过后显示窗口 + 保存截图 `TABLE_X_visual_check.jpg`
7. 通过 → 进入 `STEP_PC_DETECT_X`；失败 → 记录失败点，跳过

### 3D点云处理管线 (`processPointCloud`)

```
原始点云 (/kinect2/sd/points)
    │
    ▼
[1] TF变换: camera_frame → base_footprint
    │
    ▼
[2] PassThrough滤波:
    x: [-0.6, 1.8]
    y: [-0.8, 0.8]
    z: [0.3, 1.4]
    │
    ▼
[3] RANSAC平面分割 (SACMODEL_PLANE, distance=0.02)
    → 得到桌面平面方程 + 内点索引
    → plane_height = -coeff->values[3] / coeff->values[2]
    │
    ▼
[4] 提取桌面以上的点 (z > plane_height + 0.02)
    │
    ▼
[5] 欧几里得聚类 (tolerance=0.05, min=50, max=10000)
    │
    ▼
[6] 选离机器人最近的物体 (x最小)
    → 记录 target_centroid_ (x, y, z)
    → target_locked_ = true
    │
    ▼
[7] 记录航向 (map→base_footprint的yaw)
    → 进入 STEP_ALIGN_X
```

### 底盘微调 (`doMicroAdjustment`)

- 目标：将 `target_centroid_.x` 对齐到 0.45，`target_centroid_.y` 对齐到 0
- **线速度**: `err_x * 0.4`，clamp [-0.15, 0.15]
- **角速度**: `-err_y * 1.8`，clamp [-0.5, 0.5]
- **IMU航向锁定**: 保持与检测时相同的 map 朝向
- **激光安全**: 前方 < 0.6m → 急停
- **收敛条件**: `|err_x| < 0.08 && |err_y| < 0.08`
- **超时**: 8s后强制继续（记录失败点）
- 收敛后输出 "Ready for grasping !"

### 统计输出

- 总时间、导航成功率
- TABLE_A/B 物体数
- TABLE_A/B 目标坐标 (x,y,z)
- 失败航点列表
- 导航耗时 (nav_time_A/B)、微调耗时 (align_time_A/B)

### 订阅/发布 Topic

| Topic | 类型 | 方向 | 用途 |
|-------|------|------|------|
| `/waterplus/navi_waypoint` | String | Pub | 导航目标 |
| `/waterplus/navi_result` | String | Sub | 导航结果 |
| `/cmd_vel` | Twist | Pub | 底盘速度控制 |
| `/kinect2/qhd/image_raw` | Image | Sub | RGB相机 |
| `/kinect2/sd/points` | PointCloud2 | Sub | 点云数据 |
| `/scan` | LaserScan | Sub | 激光雷达 |
| TF (`camera→base_footprint` + `map→base_footprint`) | — | — | 坐标变换 |

---

## 四、grab_object.cpp — 机械臂抓取教学示例

- **节点名**: `grab_object_node`
- **行数**: 196
- **核心功能**: 使用 `objects_3d` 数据，通过状态机控制底盘对准 + 机械臂完成抓取

### 状态机

```
STEP_WAIT → STEP_ALIGN_OBJ → STEP_HAND_UP → STEP_FORWARD
         → STEP_GRAB → STEP_OBJ_UP → STEP_BACKWARD → STEP_DONE
```

### 抓取步骤详解

| 步骤 | 动作 | 耗时/条件 |
|------|------|-----------|
| `STEP_WAIT` | 发布 "start objects" 启动物体发布 | 循环等待 |
| `STEP_ALIGN_OBJ` | 底盘P控制器对准目标 (target: x=1.0, y=0) | 直到 dx<0.02, dy<0.01 |
| `STEP_HAND_UP` | 抬臂到 `object_z` 高度，张开夹爪到 0.15 | sleep 8s |
| `STEP_FORWARD` | 底盘以 0.1m/s 前进 | 80次×100ms = 8s |
| `STEP_GRAB` | 夹爪闭合到 0.07（夹紧） | sleep 5s |
| `STEP_OBJ_UP` | 抬臂 `object_z + 0.05`（提起物体） | sleep 5s |
| `STEP_BACKWARD` | 底盘以 -0.1m/s 后退 | 10s |
| `STEP_DONE` | 停止所有运动 | — |

### ObjectCallback 逻辑

- 订阅 `/wpb_home/objects_3d` (类型: `wpr_simulation2::msg::Object`)
- 消息含 `x[]`, `y[]`, `z[]` 数组 — 多个检测到的物体的3D坐标
- 只取 `x[0], y[0], z[0]`（第一个物体）
- 在 `STEP_WAIT` 收到消息 → 进入 `STEP_ALIGN_OBJ`
- 在 `STEP_ALIGN_OBJ` 持续更新坐标

### 底盘对准 (ALIGN_OBJ)

```cpp
float align_x = 1.0;   // 目标x距离
float align_y = 0.0;   // 目标y偏移
float diff_x = object_x - align_x;
float diff_y = object_y - align_y;

// P控制器
vel_msg.linear.x = diff_x * 0.8;
vel_msg.linear.y = diff_y * 0.8;

// 收敛条件
if (fabs(diff_x) > 0.02 || fabs(diff_y) > 0.01)
    // 继续调整
else
    // 发布 "stop objects"，进入 HAND_UP
```

### 机械臂控制 (JointState)

```cpp
mani_msg.name = ["lift", "gripper"];
mani_msg.position = [height, grip_width];
// 高度由 object_z 决定，夹爪宽度:
//   张开: 0.15
//   夹紧: 0.07
```

### 订阅/发布 Topic

| Topic | 类型 | 方向 | 用途 |
|-------|------|------|------|
| `/wpb_home/behavior` | String | Pub | 行为命令 ("start objects" / "stop objects") |
| `/wpb_home/mani_ctrl` | JointState | Pub | 机械臂控制 (lift + gripper) |
| `/cmd_vel` | Twist | Pub | 底盘速度 (含linear.y横向移动) |
| `/wpb_home/objects_3d` | Object | Sub | **3D物体坐标** (封装好的发布器) |

---

## 五、关键 Topic 通信关系汇总

```
                    ┌──────────────────────────────────────┐
                    │           仿真机器人底层               │
                    └──────┬──────────────┬────────────────┘
                           │              │
              ┌────────────┼──────────────┼─────────────────┐
              │            │              │                  │
              ▼            ▼              ▼                  ▼
    /kinect2/qhd/    /kinect2/sd/      /scan         /wpb_home/
     image_raw         points                      objects_3d
         │                │               │               │
         ▼                ▼               ▼               ▼
    ┌─────────┐    ┌──────────┐    ┌──────────┐    ┌───────────┐
    │ 人脸检测 │    │ 点云处理  │    │ 激光安全  │    │ 机械臂抓取 │
    │ (外部)   │    │(pc_work) │    │(pc_work) │    │(grab_obj) │
    └────┬─────┘    └──────────┘    └──────────┘    └─────┬─────┘
         │                                                │
         ▼                                                ▼
  /face_position                              /wpb_home/mani_ctrl
         │                                   /wpb_home/behavior
         ▼
    ┌─────────┐
    │ nav_face│
    └────┬────┘
         │
         ▼
  /waterplus/navi_waypoint  (所有节点共用)
  /waterplus/navi_result    (所有节点共用)
  /cmd_vel                  (pc_work + grab_object 共用)
  /face_detector_input      (nav_face 转发RGB给人脸检测)
```

---

## 六、已明确的设计决策

1. **抓取方案**: 使用封装好的 `objects_publisher / objects_3d` 数据（即 `/wpb_home/objects_3d`），放弃之前不稳定的三维视觉方案
2. **客人确认方案**: 选择**人脸识别**（复用 `nav_face.cpp` 中人脸检测逻辑）
3. **项目语言**: 保持中文注释/中文输出，使用 CMake 构建系统

---

## 七、整合时需要关注的问题

1. **CMakeLists.txt / launch.py 缺失**：目前只有三个独立 `.cpp` 文件，需要创建构建和启动文件
2. **节点合并 vs 多节点**：三个文件目前是三个独立节点，整合时需要考虑是合为一个节点还是保持多个节点
3. **状态机统一**：需要一个顶层的 `mission_control` 状态机来编排整个任务流程
4. **Topic 复用冲突**：`/cmd_vel` 被 `pc_work` 和 `grab_object` 同时使用，整合时需注意控制权切换
5. **objects_3d 消息格式**：`wpr_simulation2::msg::Object`，含 `x[]`, `y[]`, `z[]` 数组
6. **航点命名**：题目要求 `kitchen` 和 `guest`，而现有代码用 `A/B/C` 或 `TABLE_A/B`
