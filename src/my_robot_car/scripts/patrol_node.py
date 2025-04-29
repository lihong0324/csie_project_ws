#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PatrolBot(Node):
    def __init__(self):
        super().__init__('patrol_bot')
        self.publisher = self.create_publisher(PoseStamped, '/navigate_to_pose', 10)

        # 定義巡邏點（按你要求：1->2->3->4->3->5->6）
        self.goals = [
            (1.0, 1.0),  # 1
            (3.0, 1.0),  # 2
            (5.0, 2.0),  # 3
            (5.0, 4.0),  # 4
            (5.0, 2.0),  # 3 再次
            (3.0, 5.0),  # 5
            (1.0, 4.0),  # 6
        ]

        self.current_goal_index = 0
        self.timer = self.create_timer(10.0, self.publish_goal)  # 每 10 秒發送下一個目標

    def publish_goal(self):
        x, y = self.goals[self.current_goal_index]
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0  # 無方向需求

        self.publisher.publish(goal)
        
        # 顯示「導航開始」
        self.get_logger().info('導航開始！')
        self.get_logger().info(f'已發送巡邏點：({x}, {y})')

        # 換下一個目標點
        self.current_goal_index += 1
        if self.current_goal_index >= len(self.goals):
            self.current_goal_index = 0  # 循環

def main(args=None):
    rclpy.init(args=args)
    node = PatrolBot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()