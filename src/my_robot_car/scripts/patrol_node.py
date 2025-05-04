#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import math

class PatrolBot(Node):
    def __init__(self):
        super().__init__('patrol_bot')
        self.goal_publisher = self.create_publisher(PoseStamped, '/navigate_to_pose', 10)
        self.marker_publisher = self.create_publisher(Marker, '/patrol_markers', 10)

        # 所有座標點（x, y）
        all_goals = [
            (1.55, -10.5),   # 1
            (5.81, -7.25),   # 2
            (4.24, -5.05),   # 3
            (0.0706, -8.33), # 4
            (3.08, -2.85),   # 5
            (-1.57, -5.91),  # 6
        ]

        # 巡邏順序：1→2→3→4→3→5→6（用索引）
        sequence = [0, 1, 2, 3, 2, 4, 5]
        self.goals = [all_goals[i] for i in sequence]

        self.current_goal_index = 0
        self.visited_index = 0
        self.timer = self.create_timer(10.0, self.publish_goal)

        self.get_logger().info("巡邏機器人啟動，第一點設定朝向第二點")

    def publish_goal(self):
        if not self.goals:
            self.get_logger().warn('目前沒有目標點')
            return

        x, y = self.goals[self.current_goal_index]
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y

        # 第 1 個點時，設定方向朝向第 2 點
        if self.current_goal_index == 0:
            x2, y2 = self.goals[1]
            dx = x2 - x
            dy = y2 - y
            yaw = math.atan2(dy, dx)
            qz = math.sin(yaw / 2)
            qw = math.cos(yaw / 2)
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw
            self.get_logger().info(f"設定第一點朝向第二點：yaw={math.degrees(yaw):.2f}°")
        else:
            # 其他點：不設定方向，讓導航系統自動判斷
            goal.pose.orientation.w = 1.0

        self.goal_publisher.publish(goal)

        # 發送 RViz Marker 畫出已走過點
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
        marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)  # 淺藍色
        self.marker_publisher.publish(marker)

        self.get_logger().info(f'發送巡邏點：({x:.2f}, {y:.2f})')

        self.current_goal_index = (self.current_goal_index + 1) % len(self.goals)
        self.visited_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = PatrolBot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()