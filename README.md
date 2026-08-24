# TurtleBot3 自主巡检导航（tb3_patrol）

> 上海大学 算法组招新作业 —— TurtleBot3 模拟巡检机器人：**代码驱动的 3 目标点自主导航**

## 📌 环境要求

| 组件 | 版本 |
|---|---|
| 操作系统 | Ubuntu 22.04 (Jammy) |
| ROS2 | Humble |
| 仿真器 | Gazebo 11 |
| 导航栈 | Nav2 (navigation2 / nav2-bringup) |
| 建图 | Cartographer (cartographer_ros) |
| 机器人 | TurtleBot3 `waffle` + `turtlebot3_world` |

## 🚀 运行方法

### 1. 一次性环境准备

```bash
# 编译工作空间
cd ~/turtlebot3_ws
colcon build --symlink-install

# 环境变量（已写入 ~/.bashrc）
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=1
```

### 2. 建图（一次性）

```bash
# 终端1: 启动仿真
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端2: 启动 Cartographer
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

# 终端3: 键盘遥控扫图
ros2 run turtlebot3_teleop teleop_keyboard

# 终端4: 保存地图
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### 3. 一键自主巡检（核心）

```bash
# 单条命令：启动 Gazebo + Nav2 + 巡检节点，全程代码驱动
ros2 launch tb3_patrol patrol.launch.py
```

启动后无需任何手动操作：自动设置初始位姿 → 依次访问 3 个目标点 → 打印巡检结果。

### 4. 动态避障演示（加分项）

```bash
# 方法一: 手动指定投放点(地图坐标)
ros2 run tb3_patrol spawn_obstacle --x 2.5 --y 0.5

# 方法二: 智能监控 —— 等机器人接近目标点时自动投放（推荐）
ros2 run tb3_patrol spawn_obstacle --watch 2.5 0.5

# 方法三: 自动投放 —— 在机器人正前方 1.2m 处投放
ros2 run tb3_patrol spawn_obstacle --auto
```

巡检途中投放障碍物后，观察机器人通过局部代价地图实时感知并绕行。

## 📂 项目结构

```
tb3_patrol/
├── launch/
│   └── patrol.launch.py        # 一键启动: gazebo + nav2 + 巡检节点
├── config/
│   ├── nav2_params.yaml        # Nav2 调优参数(控制器/规划器/恢复行为)
│   └── patrol_waypoints.yaml   # 巡检点配置(可修改)
├── scripts/                    # (见 tb3_patrol/ 源码包)
└── tb3_patrol/
    ├── patrol_node.py          # 巡检主节点: 代码驱动初始位姿 + 顺序导航
    └── spawn_obstacle.py       # 动态避障演示: 运行时生成障碍物
```

## 🧠 实现思路

1. **建图**：Cartographer 激光 SLAM，键盘遥控扫图，保存 `~/map.yaml`
2. **定位**：AMCL 粒子滤波，初始位姿由代码自动发布（无需 RViz 手动点选）
3. **规划**：Nav2 NavFn 全局规划 + DWB 局部规划（动态避障）
4. **巡检**：`patrol_node.py` 按配置顺序发送 `NavigateToPose` 目标，带进度日志与结果统计
5. **恢复**：Nav2 行为树内置 Recovery（spin / backup / wait），导航失败自动恢复

## ✅ 已实现功能

- [x] TurtleBot3 仿真环境搭建（gazebo + cartographer + nav2）
- [x] 手动建图并保存
- [x] 代码驱动的 3 目标点自主导航（顺序巡检）
- [x] **巡检行为：到达每个目标点后停留 3s + 原地旋转 360° 扫描**（可配置）
- [x] 巡检进度实时日志 + 成功/失败统计
- [x] 导航失败自动 Recovery（spin/backup/wait/drive_on_heading + 卡死检测重定位）
- [x] 动态障碍物避障演示（三种投放方式）
- [x] 一键启动（单条命令完成全部）

## 🏆 加分项

| 加分方向 | 实现 |
|---|---|
| 路径合理、运动稳定 | DWB 控制器参数调优（限速/限加速度，见 nav2_params.yaml） |
| 全程代码驱动 | 初始位姿 + 目标点均由代码自动发送，零手动操作 |
| 少量命令启动 | `ros2 launch tb3_patrol patrol.launch.py` 一条命令 |
| 合理巡检行为 | 按配置顺序访问多点、进度日志、结果统计 |
| 导航失败 Recovery | ①自定义行为树定制恢复策略 ②巡检层卡死检测→重新全局定位→重试 |
| 源码利用/扩展 | 自定义行为树(patrol_recovery.xml) + 直接发布 /initialpose 绕开 nav2_simple_commander 兼容 bug |
| 动态避障 | 巡检途中投放障碍物，机器人经局部代价地图实时绕行 |

## 💡 优化点 / 技术亮点

- **自适应初始位姿**：实测确定 odom 锚定于 gazebo 世界原点，推导 map↔odom 偏移，初始位姿与真实位置精确匹配（map 坐标系 (0,0)）
- **robust 环境脚本**：`setup_env.sh` 统一 ROS2 运行环境（解决 venv Python 冲突、ROS_DOMAIN_ID 不一致、多播发现受网络切换影响等问题）
- **双重 Recovery**：行为树级（spin/backup/wait/drive_on_heading 定制顺序）+ 应用级（15s 卡死检测 → /reinitialize_global_localization → 最多 3 次重试）
- **配置驱动**：巡检点 / 导航参数 / 行为树全部外部可配置，无需改代码
- **智能投放**：spawn_obstacle 支持 watch 模式（监控机器人位置自动投放）与 auto 模式（正前方投放），适配不同演示场景
