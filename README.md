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

### ⭐ 快速开始：一键启动（推荐）

> **一条命令搞定全部**：自动清理残留进程 → 清理 FastDDS → 设置环境变量 → 启动 Gazebo 仿真 + Nav2 导航 + 巡检节点，全程无需手动操作。

**终端 A —— 一键启动巡检：**

```bash
bash ~/start_patrol.sh
```

启动后无需任何手动操作：自动设置初始位姿 → 依次访问 3 个目标点 → 打印巡检结果。

**终端 B（可选）—— 动态避障投放**（做动态避障演示时才需要，投放时机自己控制）：

```bash
source ~/turtlebot3_ws/setup_env.sh
ros2 run tb3_patrol spawn_obstacle --watch 2.545 0.514
```

> `--watch 2.545 0.514`：等机器人接近投放点 (2.545, 0.514) 时**自动投放**障碍物（演示主力）。`ros2 run` 启动需 15~40s 才就绪，自己定好时机再启动即可。想立即投放则用 `--x 2.545 --y 0.514`，更多投放方式见下文「动态避障演示」。

### 1. 一次性准备（首次使用前做一次）

**编译工作空间：**

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install

# 环境变量（已写入 ~/.bashrc）
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=1
```

**建图**（Cartographer 激光 SLAM，键盘遥控扫图并保存，导航依赖这张地图）：

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

### 2. 手动启动（不使用一键脚本时的等价操作）

`start_patrol.sh` 实际就是下面两步 + 环境设置的打包，不想用脚本时手动执行效果相同：

**步骤 1 —— 清理 FastDDS 残留**（多次运行后 `/dev/shm/fastrtps_*` 累积，会导致地图/Nav2 偶发加载失败、RViz 不显示地图）：

```bash
bash ~/clean_dds.sh
```

**步骤 2 —— 启动巡检**（单条命令：Gazebo + Nav2 + 巡检节点，全程代码驱动）：

```bash
ros2 launch tb3_patrol patrol.launch.py
```

> ⚠️ **RViz 没有地图怎么办**：先 `ros2 lifecycle get /map_server`，若显示 `finalized [4]` 说明 map_server 生命周期加载失败（FastDDS 偶发 bug）。**重启整个仿真即可**：停掉当前 launch（Ctrl+C）→ `bash ~/clean_dds.sh` → 重新 `ros2 launch`。正常时 map_server 应为 `active [3]` 且 `/map` 话题有发布者。

### 3. 动态避障演示（加分项 + 录屏）

**完整演示流程（录视频时照这个顺序执行）：**

**步骤 1（终端 A）—— 一键启动巡检：**

```bash
bash ~/start_patrol.sh
```

**步骤 2（终端 B）—— 动态避障投放**（投放时机自己控制）：

```bash
source ~/turtlebot3_ws/setup_env.sh
# 智能监控: 等机器人接近投放点时自动投放（演示主力, 推荐）
# 投放点 (2.545, 0.514) = 去 wp3 的必经缝隙(东), 机器人会自动绕西缝隙(较远路线)
ros2 run tb3_patrol spawn_obstacle --watch 2.545 0.514
```

> ⚠️ `ros2 run` 启动需 15~40s 才就绪，可以自己定好时机再启动投放。

**可选（替换步骤 2）的其他投放方式：**

```bash
# 方法一: 手动指定投放点(地图坐标), 可换任意路径点
ros2 run tb3_patrol spawn_obstacle --x 2.545 --y 0.514

# 方法二: 自动投放 —— 在机器人正前方 1.2m 处投放
ros2 run tb3_patrol spawn_obstacle --auto
```

> 💡 **spawn_obstacle 已内置防故障**：投放前自动删除残留同名实体（避免 "already exists" 被拒）、服务偶发超时自动重试 3 次。正常时日志依次出现 `已删除残留实体` → `✅ 障碍物已生成`；若仍报"投放失败"，多半是仿真世界里已有同名箱子，重跑一次即可（脚本会自动删除重建）。

> 💡 若 RViz 里出现旋转错位的"重影"色块，那只是局部代价地图图层未对齐的显示（不影响导航）。已默认关闭 `Local Costmap`/`Downsampled Costmap` 图层避免该现象。

**实测效果**：障碍物放于东缝隙 (2.545, 0.514)（wp2→wp3 的必经之路）后，机器人去 wp3 时识别到障碍、不硬闯（全程距障碍最近 0.615m），自动重规划改走**西缝隙（x≈1.445）**这条较远路线绕行，最终 3/3 全到达。

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

## 🎯 巡检点设计（选点说明）

`turtlebot3_world` 中央是 **3×3 的 9 根圆柱阵列**（ROS 标识形，柱间距 1.1m、缝宽约 0.80m）。本次选点让巡检路**线绕柱阵外围行走、不触碰柱子**，到达 3 个相距很远的点，并保持默认膨胀半径（紫色代价地图可视化）：

| 点 | 地图坐标 | 说明 |
|---|---|---|
| 起点 | (0.0, 0.0) | 柱阵以西 |
| wp1-右 | (3.6, 0.5) | 柱阵以东 |
| wp2-下 | (2.0, -1.8) | 柱阵以南 |
| wp3-上 | (2.0, 2.6) | 柱阵以北 |

- **空间跨度**：右-下 2.8m，下-上 4.4m，右-上 2.6m（房间约 6.3m×5.8m，跨度充分）
- **绕开柱阵**：起点→wp1、wp1→wp2、wp2→wp3 三段路径**均绕开 9 柱阵行走、全程不触碰柱子**（0 次碰撞、0 次卡死）
- **代价地图（紫色可视化）**：保持默认膨胀 `robot_radius:0.22, inflation_radius:0.55/1.0`——大膨胀圈把机器人安全地挡在柱外（紫色代价带），既不碰柱、运动也稳定
- **备选优化（穿柱阵）**：曾测试将代价地图按 waffle 真实足迹缩小为 `robot_radius:0.14, inflation_radius:0.15`，打通 0.80m 柱缝后可实现**从 9 柱阵内部穿行**（已验证 3/3 成功）；如需"穿柱阵"效果可平滑切换

## 🏆 加分项

| 加分方向 | 实现 |
|---|---|
| 路径合理、运动稳定 | DWB 控制器参数调优（限速/限加速度）+ 保持默认代价地图膨胀（紫色可视化），机器人绕柱阵行走稳定、全程无碰撞 |
| 全程代码驱动 | 初始位姿 + 目标点均由代码自动发送，零手动操作 |
| 少量命令启动 | `ros2 launch tb3_patrol patrol.launch.py` 一条命令 |
| 合理巡检行为 | 按配置顺序访问多点、进度日志、结果统计 |
| 导航失败 Recovery | ①自定义行为树定制恢复策略 ②巡检层卡死检测→重新全局定位→重试 |
| 源码利用/扩展 | 自定义行为树(patrol_recovery.xml) + 直接发布 /initialpose 绕开 nav2_simple_commander 兼容 bug |
| 动态避障 | 巡检途中投放障碍物，机器人经局部代价地图实时绕行 |

## 💡 优化点 / 技术亮点

- **自适应初始位姿**：实测确定 odom 锚定于 gazebo 世界原点，推导 map↔odom 偏移，初始位姿与真实位置精确匹配（map 坐标系 (0,0)）
- **代价地图膨胀设计**：默认大膨胀（紫/粉色）清晰显示障碍影响范围，同时把机器人安全挡在柱外（绕柱不碰、运动稳定）；另已验证将 inflation 缩至 0.15m 可打通 0.80m 柱缝实现穿柱阵（作为可切换的增强方案）
- **robust 环境脚本**：`setup_env.sh` 统一 ROS2 运行环境（解决 venv Python 冲突、ROS_DOMAIN_ID 不一致、多播发现受网络切换影响等问题）
- **双重 Recovery**：行为树级（spin/backup/wait/drive_on_heading 定制顺序）+ 应用级（15s 卡死检测 → /reinitialize_global_localization → 最多 3 次重试）
- **配置驱动**：巡检点 / 导航参数 / 行为树全部外部可配置，无需改代码
- **智能投放**：spawn_obstacle 支持 watch 模式（监控机器人位置自动投放）与 auto 模式（正前方投放），适配不同演示场景
