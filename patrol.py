#!/usr/bin/env python3
# patrol_bot.py – 2025-06-03
#
# 6-waypoint patrol, auto↔manual, retry on failure,
# nav_event notifications for web front-end (ROSBridge).

import math, rclpy
from rclpy.node      import Node
from rclpy.action    import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Bool, String


def yaw_to_q(yaw: float):
    """Return z, w parts of a planar quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class PatrolBot(Node):
    def __init__(self):
        super().__init__('patrol_bot')

        # ───────── ROS 介面 ─────────
        self.ac          = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.marker_pub  = self.create_publisher(Marker, '/patrol_markers', 10)
        self.cmd_pub     = self.create_publisher(Twist, '/cmd_vel', 10)
        self.event_pub   = self.create_publisher(String, '/nav_event', 10)
        self.create_subscription(Bool, '/manual_control', self.manual_cb, 10)

        # ───────── 巡邏點 & 順序 ─────────
        pts = [
            ( 1.71, -10.30),   # 0
            ( 5.73,  -7.30),   # 1
            ( 4.52,  -4.88),   # 2
            ( 0.14,  -8.09),   # 3
            ( 3.13,  -2.73),   # 4
            (-1.56,  -5.82),   # 5
        ]
        seq = [0, 1, 2, 3, 2, 4, 5, 4, 1, 0]
        self.goals = [pts[i] for i in seq]

        # ───────── 狀態變數 ─────────
        self.idx          = 0
        self.goal_active  = False
        self.manual_mode  = False
        self.vis_marker_id = 0
        self.retry_timer  = None

        self.create_timer(1.0, self.try_dispatch_goal)
        self.get_logger().info('PatrolBot 啟動完成；/manual_control 進行模式切換')

    # ───────────────────────────────────────────────
    # 模式切換
    def manual_cb(self, msg: Bool):
        if msg.data and not self.manual_mode:          # 進入手動
            self.manual_mode = True
            self._cancel_current_goal()
            self.get_logger().info('→ 手動模式（自動巡邏暫停）')
        elif not msg.data and self.manual_mode:        # 回自動
            self.manual_mode = False
            self.get_logger().info('→ 返回自動巡邏')

    def _cancel_current_goal(self):
        if self.goal_active and getattr(self, 'goal_handle', None):
            self.goal_handle.cancel_goal_async()
        self.goal_active = False

    # ───────────────────────────────────────────────
    # 送出目標
    def try_dispatch_goal(self):
        if self.manual_mode or self.goal_active:
            return
        if self.idx >= len(self.goals):
            self.get_logger().info('所有巡邏點完成，節點結束')
            rclpy.shutdown()
            return
        if not self.ac.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('等待 Nav2 action server…')
            return

        x, y = self.goals[self.idx]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y

        # yaw 指向下一點
        if self.idx < len(self.goals) - 1:
            nx, ny = self.goals[self.idx + 1]
            yaw = math.atan2(ny - y, nx - x)
        else:
            yaw = 0.0
        pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_q(yaw)

        goal = NavigateToPose.Goal(); goal.pose = pose
        self.get_logger().info(f'►  派送巡邏點 {self.idx+1}/{len(self.goals)} ({x:.2f}, {y:.2f})')

        self.event_pub.publish(String(data=f'dispatched:{self.idx}'))
        self.goal_active = True
        self.ac.send_goal_async(goal, feedback_callback=lambda _: None)\
               .add_done_callback(self.goal_resp_cb)

    # ───────────────────────────────────────────────
    # goal handle
    def goal_resp_cb(self, fut):
        self.goal_handle = fut.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('目標被拒絕')
            self.goal_active = False
            return
        self.goal_handle.get_result_async().add_done_callback(self.result_cb)

    # ───────────────────────────────────────────────
    # 處理結果
    def result_cb(self, fut):
        status = fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✓ 抵達')
            self.event_pub.publish(String(data=f'arrived:{self.idx}'))
            self._publish_marker(*self.goals[self.idx])
            self.idx += 1
            self.goal_active = False
        else:
            self.get_logger().warn(f'✗ 導航失敗（code={status}），小步前進後重試')
            self.event_pub.publish(String(data=f'failed:{self.idx}:{status}'))
            self._bump_then_retry()

    # ───────────────────────────────────────────────
    # RViz marker
    def _publish_marker(self, x, y):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns, m.id        = 'patrol', self.vis_marker_id
        m.type, m.action  = Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color   = ColorRGBA(r=0.0, g=0.6, b=1.0, a=1.0)
        self.marker_pub.publish(m)
        self.vis_marker_id += 1

    # ───────────────────────────────────────────────
    # 小步前進再重試
    def _bump_then_retry(self):
        self.cmd_pub.publish(self._twist(0.05))
        if self.retry_timer:
            self.retry_timer.cancel()
        self.retry_timer = self.create_timer(0.5, self._retry_callback)

    def _retry_callback(self):
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer = None
        self.cmd_pub.publish(self._twist(0.0))
        self.goal_active = False     # 讓 try_dispatch_goal() 重新派送

    @staticmethod
    def _twist(x_lin=0.0):
        t = Twist(); t.linear.x = x_lin; return t


def main():
    rclpy.init()
    rclpy.spin(PatrolBot())


if __name__ == '__main__':
    main()
