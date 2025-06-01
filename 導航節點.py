#!/usr/bin/env python3
# patrol_bot.py – 2025-06-01  (auto ↔ manual, retry on failure)

import math, rclpy, time
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Bool

class PatrolBot(Node):
    def __init__(self):
        super().__init__('patrol_bot')

        # ───────── 介面 ─────────
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.marker_pub    = self.create_publisher(Marker, '/patrol_markers', 10)
        self.cmd_pub       = self.create_publisher(Twist,  '/cmd_vel',        10)
        self.create_subscription(Bool, '/manual_control', self.manual_cb, 10)

        # ───────── 巡邏路線 ─────────
        all_pts  = [
            (1.55, -10.5), (5.81, -7.25), (4.24, -5.05),
            (0.0706, -8.33), (3.32, -2.74), (-1.57, -5.91),
        ]
        seq      = [0, 1, 2, 3, 2, 4, 5, 4, 1, 0]
        self.goals = [all_pts[i] for i in seq]

        self.idx               = 0          # 下一個目標 index
        self.visited_marker_id = 0
        self.goal_in_progress  = False
        self.manual_mode       = False
        self.wait_server       = True
        self.goal_handle       = None
        self.retry_timer       = None       # 專給「小步前進」後停用的 timer

        self.create_timer(1.0, self.dispatch_goal)  # 每秒嘗試派新目標
        self.get_logger().info('PatrolBot 已啟動：可透過 /manual_control 切換手動 / 自動')

    # ──────── 模式切換 ────────
    def manual_cb(self, msg: Bool):
        if msg.data and not self.manual_mode:          # 進入手動
            self.manual_mode = True
            self.get_logger().info('→ 手動控制：已暫停自動巡邏')
            if self.goal_in_progress and self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.goal_in_progress = False
            self.idx = min(self.idx + 1, len(self.goals))  # 下次從下一點開始
        elif (not msg.data) and self.manual_mode:      # 返回自動
            self.manual_mode = False
            self.get_logger().info('→ 回到自動巡邏')
            # 下一輪 dispatch_goal 會自動送出

    # ──────── 送出目標 ────────
    def dispatch_goal(self):
        if self.manual_mode or self.goal_in_progress:
            return
        if self.idx >= len(self.goals):
            self.get_logger().info('🎉  所有巡邏點完成，關閉節點')
            rclpy.shutdown()
            return
        if self.wait_server:
            if not self.action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn('等待 Nav2 action server…')
                return
            self.wait_server = False

        x, y = self.goals[self.idx]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y

        # 朝向下一點
        if self.idx < len(self.goals) - 1:
            nx, ny = self.goals[self.idx + 1]
            yaw    = math.atan2(ny - y, nx - x)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            pose.pose.orientation.w = 1.0

        goal_msg      = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f'►  送出第 {self.idx+1}/{len(self.goals)} 點 ({x:.2f}, {y:.2f})')
        self.goal_in_progress = True
        send_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self.goal_response_cb)

    # 封存但不輸出 feedback（可自行加入）
    def feedback_cb(self, _): pass

    # ──────── 接收 goal handle ────────
    def goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('目標被拒絕')
            self.goal_in_progress = False
            return
        self.get_logger().info('已接受目標，開始導航…')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    # ──────── 處理結果 ────────
    def result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✓ 已到達')
            self._mark_reached(self.goals[self.idx])
            self.idx += 1
            self.goal_in_progress = False
        else:
            self.get_logger().warn(f'✗ 導航失敗（code={status}），小步前進後重試')
            self._bump_then_retry()

    # ─── 成功後發 RViz marker ───
    def _mark_reached(self, pt):
        x, y = pt
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns, m.id        = 'patrol', self.visited_marker_id
        m.type, m.action  = Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color   = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)
        self.marker_pub.publish(m)
        self.visited_marker_id += 1

    # ─── 失敗 → 往前 0.05 m 再重試 ───
    def _bump_then_retry(self):
        # 發一次向前 Twist
        twist = Twist()
        twist.linear.x = 0.05
        self.cmd_pub.publish(twist)

        # 0.5 秒後停止並解除 goal_in_progress，dispatch_goal 會重送
        if self.retry_timer:
            self.retry_timer.cancel()
        self.retry_timer = self.create_timer(0.5, self._stop_and_ready)

    def _stop_and_ready(self):
        self.retry_timer.cancel()
        self.retry_timer = None
        self.cmd_pub.publish(Twist())      # 速度歸零
        self.goal_in_progress = False      # 允許重送目前目標

def main(args=None):
    rclpy.init(args=args)
    node = PatrolBot()
    rclpy.spin(node)        # shutdown 由節點自行決定
    # 若節點內呼叫 rclpy.shutdown()，這裡會直接退出

if __name__ == '__main__':
    main()
