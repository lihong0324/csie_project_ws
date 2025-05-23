#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import math
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class PatrolBot(Node):
    def __init__(self):
        super().__init__('patrol_bot')

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.marker_publisher = self.create_publisher(Marker, '/patrol_markers', 10)

        # 所有的巡邏點座標（x, y）
        all_goals = [
            (1.55, -10.5),   # 1
            (5.81, -7.25),   # 2
            (4.24, -5.05),   # 3
            (0.0706, -8.33), # 4
            (3.32, -2.74),   # 5
            (-1.57, -5.91),  # 6
        ]
        # 自訂的巡邏順序（index）
        sequence = [0, 1, 2, 3, 2, 4, 5, 4, 1, 0]
        self.goals = [all_goals[i] for i in sequence]

        self.current_goal_index = 0
        self.visited_index = 0

        # 每秒檢查一次是否可以送出下一個目標
        self.timer = self.create_timer(1.0, self.send_next_goal)

        self.waiting_for_server = True
        self.goal_in_progress = False

        self.get_logger().info("Nav2 巡邏機器人啟動，會在每個點完成後提示並繼續下一個")

    def send_next_goal(self):
        if self.waiting_for_server:
            if not self.action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn("等待 Nav2 Action Server 中...")
                return
            self.waiting_for_server = False

        if self.goal_in_progress or self.current_goal_index >= len(self.goals):
            return

        x, y = self.goals[self.current_goal_index]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        # 設定朝向：朝向下一個點（若存在），否則使用預設方向
        if self.current_goal_index < len(self.goals) - 1:
            # 下一個點的座標
            x2, y2 = self.goals[self.current_goal_index + 1]
            # 計算從目前點到下一點的 yaw（弧度）
            yaw = math.atan2(y2 - y, x2 - x)
            # 將 yaw 轉換為四元數（只需 z, w）
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.get_logger().info(f'第 {self.current_goal_index + 1} 點朝向下一點：yaw={math.degrees(yaw):.2f}°')
        else:
            # 最後一點：保持正前方向
            pose.pose.orientation.w = 1.0

        # 建立導航目標
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f'送出巡邏點 {self.current_goal_index + 1}：({x:.2f}, {y:.2f})')
        self.goal_in_progress = True
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, feedback_msg):
        # 已移除距離回報（避免過多資訊輸出）
        pass

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目標被拒絕，將不前進')
            self.goal_in_progress = False
            return

        self.get_logger().info('已接受目標，開始導航...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('到達目標點！')
            self.publish_marker(self.goals[self.current_goal_index])
            self.current_goal_index += 1
        else:
            self.get_logger().warn(f'導航失敗（狀態碼：{status}），將再次嘗試同一個點')

        self.goal_in_progress = False

    def publish_marker(self, point):
        x, y = point
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'patrol_points'
        marker.id = self.visited_index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)
        self.marker_publisher.publish(marker)
        self.visited_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = PatrolBot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()