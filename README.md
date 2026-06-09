# 家庭服务机器人饮料递送系统

ROS2 综合项目，整合导航、三维视觉抓取、人脸识别确认，完成 **厨房取饮料 → 客人递送** 全流程。

## 文件结构

```
├── mani.cpp                # 主任务节点（整合导航/抓取/人脸确认）
├── nav_face.cpp            # 参考：导航巡点 + 人脸识别
├── pc_work.cpp             # 参考：桌面点云检测 + 底盘微调
├── grab_object.cpp         # 参考：机械臂 objects_3d 抓取范例
├── CMakeLists.txt          # CMake 构建配置
├── package.xml             # ROS2 包描述
├── launch/
│   └── mani.launch.py      # 启动文件（nav2 + rviz + 航点 + 人脸检测 + objects_publisher）
└── code_summary.md         # 代码功能汇总
```

## 状态机

```
STEP_WAIT → GOTO_KITCHEN → VERIFY_DRINK(RGB) → ALIGN_OBJ → GRAB
→ GOTO_GUEST → VERIFY_GUEST(人脸) → DONE
```

## 启动方式

```bash
# 1. 手动启动 Gazebo 仿真

# 2. 启动支撑组件
ros2 launch cv_pkg mani.launch.py

# 3. 手动启动任务节点
ros2 run cv_pkg mani --ros-args -p strategy_mode:=2
```

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| strategy_mode | 1 | 1=基础模式, 2=增强模式 |
| timeout_sec | 80.0 | 导航超时(s) |
| stay_time_sec | 8.0 | 停留检测时间(s) |
| max_retry | 2 | 最大重试次数 |