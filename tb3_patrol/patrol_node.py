#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TurtleBot3 自主巡检节点（tb3_patrol）

功能（对应作业加分项）:
  1. 全程代码驱动 —— 自动设置 AMCL 初始位姿（无需在 RViz 手点 2D Pose Estimate）
  2. 顺序访问多个目标点 —— 自动发送 Navigation2 Goal（无需在 RViz 手点 Navigation2 Goal）
  3. 巡检行为 —— 按配置文件顺序巡检，实时打印进度，统计成功/失败
  4. 导航失败 Recovery（增强）:
     a. 交给 Nav2 行为树内置恢复（spin / backup / wait / drive_on_heading，见自定义行为树）
     b. 巡检层卡死检测: 若机器人 15s 内几乎不动(定位丢失或卡住),
        自动取消当前任务 -> 重新全局定位(/reinitialize_global_localization) -> 重试(最多3次)

用法:
    ros2 run tb3_patrol patrol [--waypoints <yaml 路径>]
    不传参数时默认读取 tb3_patrol 包内的 config/patrol_waypoints.yaml
"""
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory

import yaml

# 卡死检测参数
STUCK_TIMEOUT = 15.0     # 15 秒无有效移动判定为卡死
STUCK_MIN_MOVE = 0.05    # 15 秒内移动 < 0.05m 视为无进展
# odom -> map 偏移(实测: map = odom + 该偏移); 用于判断机器人是否已接近目标
MAP_TO_ODOM_OFFSET = (1.995, 0.514)
# 机器人距目标点小于该值即视为"已到达"(避免把到达后的静止误判为卡死)
GOAL_ARRIVED_DIST = 0.30
MAX_RETRIES = 3          # 每个目标点最多重试次数
RELOCO_WAIT = 6.0        # 重新全局定位后等待收敛时间


def yaw_to_quaternion(yaw_rad: float):
    """绕 z 轴的偏航角 -> 四元数 [x, y, z, w]"""
    return [0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)]


class PatrolNode(Node):
    def __init__(self, waypoints_path: str):
        super().__init__('tb3_patrol')
        self.waypoints_path = waypoints_path
        self.config = self._load_config()
        # 初始位姿发布器（AMCL 订阅 /initialpose）
        # 注: nav2_simple_commander 1.1.20 的 setInitialPose 有类型 bug,
        #     因此直接向话题发布, 绕开该 API
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        # odom 订阅（卡死检测用）
        self._odom_pos = None
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        # 巡检行为: 原地旋转用
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _odom_cb(self, msg):
        self._odom_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _get_odom_pos(self):
        return self._odom_pos

    def _load_config(self):
        if not os.path.exists(self.waypoints_path):
            self.get_logger().error(f'配置文件不存在: {self.waypoints_path}')
            sys.exit(1)
        with open(self.waypoints_path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        if not cfg or 'waypoints' not in cfg or not cfg['waypoints']:
            self.get_logger().error('配置缺少 waypoints 列表')
            sys.exit(1)
        return cfg

    def _to_pose_stamped(self, nav: BasicNavigator, x, y, yaw_deg, frame='map'):
        """构造一个目标位姿"""
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = nav.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        q = yaw_to_quaternion(math.radians(float(yaw_deg)))
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def _reinitialize_localization(self):
        """调用 AMCL 的全局重新定位服务（粒子均匀撒满地图后重新收敛）"""
        self.get_logger().warn('调用 /reinitialize_global_localization 重新全局定位 ...')
        try:
            client = self.create_client(Empty, '/reinitialize_global_localization')
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('reinitialize 服务不可用')
                return
            req = Empty.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            self.get_logger().info(f'全局定位已触发, 等待 {RELOCO_WAIT}s 收敛 ...')
            time.sleep(RELOCO_WAIT)
        except Exception as e:
            self.get_logger().error(f'重新定位异常: {e}')

    def _navigate_with_recovery(self, nav: BasicNavigator, goal: PoseStamped):
        """导航到目标点（带卡死检测 + 重新定位 + 重试的 Recovery 逻辑）

        返回: TaskResult 结果
        """
        last_result = None
        for attempt in range(1, MAX_RETRIES + 1):
            if attempt > 1:
                self.get_logger().warn(f'—— 第 {attempt}/{MAX_RETRIES} 次尝试 ——')
            nav.goToPose(goal)

            # 卡死检测: 记录起点与最近有效移动时间
            last_pos = self._get_odom_pos()
            last_move_time = time.time()
            stuck_check_count = 0
            last_printed = 1e9
            min_near_goal = [False]  # 是否因"已接近目标"而跳出

            while not nav.isTaskComplete():
                # --- 反馈打印（稀疏） ---
                fb = nav.getFeedback()
                if fb:
                    remain = fb.distance_remaining
                    if remain < last_printed - 0.5:
                        self.get_logger().info(f'    剩余距离 {remain:.2f} m')
                        last_printed = remain
                    elif remain < 0.3 and last_printed > 0.3:
                        self.get_logger().info(f'    即将到达: {remain:.2f} m')
                        last_printed = remain

                # --- 卡死检测 ---
                cur_pos = self._get_odom_pos()
                if cur_pos is not None and last_pos is not None:
                    moved = math.hypot(cur_pos[0] - last_pos[0], cur_pos[1] - last_pos[1])
                    if moved >= STUCK_MIN_MOVE:
                        last_pos = cur_pos
                        last_move_time = time.time()
                    elif time.time() - last_move_time > STUCK_TIMEOUT:
                        # 关键: 先判断是否其实已接近目标点(在到达容差内)——
                        # 机器人到目标附近停下是正常到达, 不是卡死
                        goal_dist = 1e9
                        if cur_pos is not None and goal.pose.position is not None:
                            cur_map = (cur_pos[0] + MAP_TO_ODOM_OFFSET[0],
                                       cur_pos[1] + MAP_TO_ODOM_OFFSET[1])
                            goal_dist = math.hypot(
                                cur_map[0] - goal.pose.position.x,
                                cur_map[1] - goal.pose.position.y)
                        if goal_dist < GOAL_ARRIVED_DIST:
                            self.get_logger().info(
                                f'    已接近目标点 ({goal_dist:.2f}m)，视为到达')
                            min_near_goal[0] = True
                            break
                        stuck_check_count += 1
                        if stuck_check_count >= 3:
                            self.get_logger().warn(
                                f'⚠️ 检测到卡死（{STUCK_TIMEOUT}s 无有效移动），'
                                f'取消任务并进入 Recovery ...')
                            break
                rclpy.spin_once(self, timeout_sec=0.1)

            # 任务完成
            if nav.isTaskComplete():
                return nav.getResult()
            if min_near_goal[0]:
                self.get_logger().info('目标点判定为已到达')
                return TaskResult.SUCCEEDED

            # 卡死: 取消当前任务 + 重新全局定位 + 重试
            self.get_logger().warn('取消当前导航任务 ...')
            nav.cancelTask()
            self._reinitialize_localization()

        return last_result if last_result is not None else TaskResult.FAILED

    def _patrol_behavior(self, wp_name):
        """巡检行为（加分项）: 到达目标点后停留数秒, 并原地旋转一周扫描"""
        pb = self.config.get('patrol_behavior', {})
        dwell = float(pb.get('dwell_time', 3.0))
        self.get_logger().info(f'[巡检行为] {wp_name}: 停留 {dwell}s 观察 ...')
        time.sleep(dwell)
        if pb.get('spin_rotation', True):
            dur = float(pb.get('spin_duration', 4.0))
            self.get_logger().info(f'[巡检行为] {wp_name}: 原地旋转 360° 扫描 ({dur}s) ...')
            self._spin_in_place(dur)

    def _spin_in_place(self, duration=4.0):
        """原地旋转一整圈: 角速度 = 2π / duration
        注意: 用 time.sleep 而非 create_rate().sleep() ——
        rate.sleep() 依赖节点时钟, 在 rclpy 偶发挂起; time.sleep 走系统时钟绝对可靠"""
        twist = Twist()
        twist.angular.z = 2.0 * math.pi / duration
        end = time.time() + duration
        while rclpy.ok() and time.time() < end:
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.05)  # ~20Hz
        self._cmd_vel_pub.publish(Twist())  # 停转
        self.get_logger().info(f'[巡检行为] 旋转完成, 继续下一目标')

    def _wait_amcl_subscriber(self, timeout=150.0):
        """等待 AMCL 订阅 /initialpose（等价于 AMCL 已激活）。
        纯话题检测, 绕开 waitUntilNav2Active() 的 get_state 服务偶发超时卡死"""
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            if self.initial_pose_pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().warn('等待 AMCL 订阅 /initialpose 超时')
        return False

    def _wait_amcl_pose(self, timeout=60.0):
        """等待 AMCL 发布 /amcl_pose（初始定位完成）。
        注意 QoS: AMCL 用 TRANSIENT_LOCAL + RELIABLE 发布, 默认 VOLATILE 订阅会收不到!"""
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        got = {'ok': False}
        def _cb(msg):
            got['ok'] = True
        sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', _cb, qos)
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            if got['ok']:
                break
            rclpy.spin_once(self, timeout_sec=0.5)
        self.destroy_subscription(sub)
        if got['ok']:
            self.get_logger().info('AMCL 初始定位完成')
            return True
        self.get_logger().warn('等待 AMCL 定位超时')
        return False

    def _wait_bt_navigator_active(self, timeout=60.0):
        """等待 bt_navigator 生命周期变为 ACTIVE(通过 get_state 服务轮询)。
        说明: 之前用 transition_event 话题监听, 但它是 VOLATILE 无缓存、
        依赖订阅时机恰好覆盖激活瞬间 —— 存在竞态(订阅晚于激活就漏掉)。
        改为轮询 get_state 服务: 返回当前真实状态, 无论何时查询都准确, 确定性可靠。"""
        from lifecycle_msgs.srv import GetState
        cli = self.create_client(GetState, '/bt_navigator/get_state')
        # 等服务就绪
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/bt_navigator/get_state 服务不可用')
            return False
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            req = GetState.Request()
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
            if fut.result() is not None:
                # id: 1=unconfigured 2=inactive 3=active 4=finalized
                if fut.result().current_state.id == 3:
                    self.get_logger().info('bt_navigator 已激活(ACTIVE)')
                    self.destroy_client(cli)
                    return True
            time.sleep(0.5)
        self.get_logger().warn('等待 bt_navigator 激活超时')
        self.destroy_client(cli)
        return False

    def run(self):
        nav = BasicNavigator()

        # 等待 Nav2 就绪（纯话题检测, 避免服务调用偶发卡死）
        self.get_logger().info('等待 Nav2 就绪(AMCL 订阅 /initialpose) ...')
        self._wait_amcl_subscriber()

        # 等待机器人 odom 可用 —— 关键! 确保 gazebo 已完成生成、odom TF 已发布,
        # 否则初始位姿的 TF 查询会失败 (sim 时间早于机器人生成时刻)
        self.get_logger().info('等待机器人 odom(确认已生成) ...')
        start = time.time()
        while rclpy.ok() and time.time() - start < 90.0:
            if self._odom_pos is not None:
                self.get_logger().info(f'机器人已生成, odom 位置: ({self._odom_pos[0]:.2f}, {self._odom_pos[1]:.2f})')
                break
            rclpy.spin_once(self, timeout_sec=0.5)
        if self._odom_pos is None:
            self.get_logger().error('等待机器人生成超时, 中止巡检')
            return False

        # ---------- 代码驱动：自动设置初始位姿 ----------
        ip = self.config.get('initial_pose', {})
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = 'map'
        # 时间戳必须用 0(使用最新 TF): 本节点用系统时钟, 若打系统时间戳,
        # AMCL 按仿真时间查 TF 会报 "extrapolation into the future" 导致定位失败
        initial_pose_msg.header.stamp = rclpy.time.Time().to_msg()
        initial_pose_msg.pose.pose.position.x = float(ip.get('x', 0.0))
        initial_pose_msg.pose.pose.position.y = float(ip.get('y', 0.0))
        q = yaw_to_quaternion(math.radians(float(ip.get('yaw', 0.0))))
        initial_pose_msg.pose.pose.orientation.z = q[2]
        initial_pose_msg.pose.pose.orientation.w = q[3]
        # 起始位姿相对确定，协方差设小
        initial_pose_msg.pose.covariance[0] = 0.25
        initial_pose_msg.pose.covariance[7] = 0.25
        initial_pose_msg.pose.covariance[35] = 0.06853891945200942

        # 多重发布 + 重试, 确保 AMCL 收到（FastDDS 一次性消息可能丢失）
        localized = False
        for attempt in range(1, 4):
            for _ in range(5):
                self.initial_pose_pub.publish(initial_pose_msg)
                time.sleep(0.5)
            self.get_logger().info(
                f'已发布初始位姿 (x={ip.get("x")}, y={ip.get("y")}, yaw={ip.get("yaw")}°), 第 {attempt}/3 轮'
            )
            if self._wait_amcl_pose(timeout=30.0):
                localized = True
                break
            self.get_logger().warn('AMCL 未定位, 重发初始位姿 ...')

        if not localized:
            self.get_logger().error('AMCL 定位失败, 中止巡检')
            return False

        self.get_logger().info('Nav2 就绪, 开始巡检')
        # 确保导航 Action Server 就绪（bt_navigator 激活）后再发目标
        if not nav.nav_to_pose_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('navigate_to_pose Action Server 不可用, 中止巡检')
            return False
        self.get_logger().info('导航服务器就绪')
        # 关键: 等 bt_navigator 生命周期 ACTIVE, 否则目标会被拒绝
        if not self._wait_bt_navigator_active():
            self.get_logger().error('bt_navigator 未激活, 中止巡检')
            return False

        # ---------- 顺序巡检 ----------
        waypoints = self.config['waypoints']
        total = len(waypoints)
        self.get_logger().info(f'开始巡检，共 {total} 个目标点')

        ok_count = 0
        for i, wp in enumerate(waypoints, start=1):
            name = wp.get('name', f'wp{i}')
            goal = self._to_pose_stamped(nav, wp['x'], wp['y'], wp.get('yaw', 0.0))
            self.get_logger().info(
                f'[{i}/{total}] 前往 {name} @ ({wp["x"]}, {wp["y"]}) ...'
            )

            result = self._navigate_with_recovery(nav, goal)

            if result == TaskResult.SUCCEEDED:
                ok_count += 1
                self.get_logger().info(f'[{i}/{total}] ✅ 成功到达 {name}')
                # 巡检行为: 到达后停留 + 原地旋转一周（加分项）
                self._patrol_behavior(name)
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f'[{i}/{total}] ❌ {name} 任务被取消')
            else:
                self.get_logger().warn(f'[{i}/{total}] ❌ {name} 失败: {result}')

        self.get_logger().info(f'===== 巡检结束: {ok_count}/{total} 个目标点成功 =====')

        nav.lifecycleShutdown()
        return ok_count == total


def main(args=None):
    # 解析 --waypoints 参数
    waypoints_path = None
    if args is None:
        args = sys.argv
    if '--waypoints' in args:
        waypoints_path = args[args.index('--waypoints') + 1]
    else:
        try:
            pkg_share = get_package_share_directory('tb3_patrol')
            waypoints_path = os.path.join(pkg_share, 'config', 'patrol_waypoints.yaml')
        except Exception:
            waypoints_path = os.path.join(
                os.path.dirname(__file__), '..', 'config', 'patrol_waypoints.yaml'
            )

    rclpy.init(args=args)
    node = PatrolNode(waypoints_path)
    try:
        ok = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('巡检被用户中断')
        ok = False
    except Exception as e:
        node.get_logger().error(f'巡检异常: {e}')
        import traceback
        traceback.print_exc()
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if not ok:
            sys.exit(1)
