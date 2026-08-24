#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动态避障演示脚本（加分项）:
  在导航过程中，向 Gazebo 世界 spawn 一个障碍物（红色箱子），
  观察 TurtleBot3 实时重规划绕行，体现 Nav2 的动态避障能力。

坐标说明:
  参数为【地图(map)坐标】, 与 patrol_waypoints.yaml 一致。
  脚本内部自动换算为 gazebo 世界坐标（map = gazebo + (1.995, 0.514)）。

用法（需先启动导航）:
    ros2 run tb3_patrol spawn_obstacle --x 2.0 --y 0.5
"""
import math
import os
import random
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from nav_msgs.msg import Odometry

# map -> gazebo 世界偏移（实测: odom 锚定于 gazebo 世界原点, map = gazebo + 偏移）
MAP_TO_GAZEBO_OFFSET = (1.995, 0.514)

# 自动投放: 机器人正前方距离
AUTO_DISTANCE = 1.2


def quat_to_yaw(msg):
    """从四元数提取偏航角"""
    import math
    z, w = msg.orientation.z, msg.orientation.w
    return 2.0 * math.atan2(z, w)


class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self._odom = None

    def _odom_cb(self, msg):
        self._odom = msg

    def _get_robot_gazebo_pose(self):
        """读取机器人 gazebo 世界位姿(x, y, yaw)"""
        while self._odom is None:
            rclpy.spin_once(self, timeout_sec=0.5)
            self.get_logger().info('等待 /odom 数据 ...')
        return (self._odom.pose.pose.position.x,
                self._odom.pose.pose.position.y,
                quat_to_yaw(self._odom.pose.pose))

    def auto_position(self):
        """在机器人正前方 AUTO_DISTANCE 处投放"""
        x, y, yaw = self._get_robot_gazebo_pose()
        gx = x + AUTO_DISTANCE * math.cos(yaw)
        gy = y + AUTO_DISTANCE * math.sin(yaw)
        return gx, gy

    def watch_and_spawn(self, mx, my, trigger_dist=2.0):
        """监控模式: 机器人(地图坐标)距目标点 < trigger_dist 米时投放。
        实测: 触发距 0.6/1.5m 时障碍物过近会把机器人困在柱阵缝里导致卡死;
        2.0m 最稳(机器人识别到障碍不硬闯, 自动重规划走较远路线绕行)。"""
        target_gx = mx - MAP_TO_GAZEBO_OFFSET[0]
        target_gy = my - MAP_TO_GAZEBO_OFFSET[1]
        self.get_logger().info(
            f'监控模式: 目标点 地图({mx},{my}), 距离 {trigger_dist}m 时投放 ...')
        last_debug = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._odom is not None:
                # odom == gazebo 世界坐标, 换算为 map 坐标
                rx = self._odom.pose.pose.position.x + MAP_TO_GAZEBO_OFFSET[0]
                ry = self._odom.pose.pose.position.y + MAP_TO_GAZEBO_OFFSET[1]
                dist = math.hypot(rx - mx, ry - my)
                # 每 3 秒打印一次调试信息
                if time.time() - last_debug > 3.0:
                    self.get_logger().info(f'[debug] 机器人地图位置 ({rx:.2f},{ry:.2f}), 距目标 {dist:.2f}m')
                    last_debug = time.time()
                if dist < trigger_dist:
                    self.get_logger().info(
                        f'🎯 机器人距目标 {dist:.2f}m, 立即投放障碍物!')
                    return self.spawn_box(target_gx, target_gy, name='dynamic_obstacle')
            else:
                if time.time() - last_debug > 3.0:
                    self.get_logger().warn('[debug] 尚未收到 /odom 数据!')
                    last_debug = time.time()
        return False

    def spawn_box(self, gx, gy, name='dynamic_obstacle'):
        while not self.spawn_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('等待 /spawn_entity 服务 ...')
        req = SpawnEntity.Request()
        req.name = name
        req.xml = self._box_sdf(gx, gy)
        future = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is None:
            self.get_logger().error('spawn 服务调用失败')
            return False
        return future.result().success

    def _box_sdf(self, gx, gy):
        """红色 0.25m 立方体障碍物（gazebo 世界坐标）"""
        return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="dynamic_obstacle">
    <static>true</static>
    <pose>{gx} {gy} 0.125 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>0.25 0.25 0.25</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.25 0.25 0.25</size></box></geometry>
        <material><ambient>0.9 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpawner()

    if '--watch' in sys.argv:
        # 监控模式: 等机器人接近目标点后投放（watch_and_spawn 内部已完成投放）
        wx = 1.5
        wy = 1.8
        if len(sys.argv) > sys.argv.index('--watch') + 2:
            wx = float(sys.argv[sys.argv.index('--watch') + 1])
            wy = float(sys.argv[sys.argv.index('--watch') + 2])
        ok = node.watch_and_spawn(wx, wy)
        if ok:
            node.get_logger().info('✅ 障碍物已生成，观察机器人绕行')
        else:
            node.get_logger().error('投放失败')
        node.destroy_node()
        rclpy.shutdown()
        return
    elif '--auto' in sys.argv:
        # 智能模式: 在机器人正前方投放
        gx, gy = node.auto_position()
        node.get_logger().info(f'智能投放: 机器人正前方 gazebo({gx:.2f}, {gy:.2f}) ...')
    else:
        # 手动模式: 解析 map 坐标参数
        mx = 2.0
        my = 0.5
        if '--x' in sys.argv:
            mx = float(sys.argv[sys.argv.index('--x') + 1])
        if '--y' in sys.argv:
            my = float(sys.argv[sys.argv.index('--y') + 1])
        # map -> gazebo 世界坐标换算
        gx = mx - MAP_TO_GAZEBO_OFFSET[0]
        gy = my - MAP_TO_GAZEBO_OFFSET[1]
        node.get_logger().info(
            f'在 地图({mx}, {my}) = gazebo({gx:.2f}, {gy:.2f}) 生成动态障碍物 ...'
        )

    if node.spawn_box(gx, gy):
        node.get_logger().info('✅ 障碍物已生成，观察机器人绕行')
    else:
        node.get_logger().error('障碍物生成失败')

    node.destroy_node()
    rclpy.shutdown()
