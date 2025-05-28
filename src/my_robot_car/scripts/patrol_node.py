#!/usr/bin/env python3
# patrol_and_ball.py
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge


class PatrolAndBall(Node):
    def __init__(self):
        super().__init__('patrol_and_ball')

        # ---------- 1. 目標點與巡邏順序 ----------
        all_goals = [
            (1.55, -10.5),   # 0
            (5.81, -7.25),   # 1
            (4.24, -5.05),   # 2
            (0.0706, -8.33), # 3
            (3.32, -2.74),   # 4
            (-1.57, -5.91),  # 5
        ]
        sequence = [0, 1, 2, 3, 2, 4, 5, 4, 1, 0]
        self.goals = [all_goals[i] for i in sequence]

        # ---------- 2. ROS2 介面 ----------
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/image_raw',
                                                  self.image_cb, 10)
        self.manual_sub = self.create_subscription(Bool, '/manual_control',
                                                   self.manual_cb, 10)

        # ---------- 3. 狀態變數 ----------
        self.bridge = CvBridge()
        self.goal_idx = 0
        self.navigating = False
        self.goal_handle = None

        self.red_ball_detected = False
        self.ball_cx = None
        self.img_w = None

        self.manual_control = False   # False＝自動，True＝手動

        # ---------- 4. 啟動 ----------
        self.get_logger().info('等待 Nav2 action server...')
        self.nav_client.wait_for_server()
        self.timer = self.create_timer(0.3, self.main_loop)
        cv2.namedWindow("Ball View", cv2.WINDOW_NORMAL)
        self.get_logger().info('巡邏 + 紅球偵測 節點啟動完成')

    # ==========================================================
    # 影像回呼：紅球偵測
    # ==========================================================
    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 紅色雙區間
        m1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
        m2 = cv2.inRange(hsv, (160, 100, 100), (179, 255, 255))
        mask = cv2.bitwise_or(m1, m2)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
        self.img_w = frame.shape[1]

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                M = cv2.moments(c)
                self.ball_cx = int(M["m10"] / M["m00"])
                self.red_ball_detected = True
            else:
                self.red_ball_detected = False
        else:
            self.red_ball_detected = False

        # 視窗 + 手動熱鍵
        cv2.imshow("Ball View", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('m'):
            self.manual_control = not self.manual_control
            self.get_logger().info(f'手動模式切換為 {self.manual_control}')

    # ==========================================================
    # 手動控制 Bool topic 回呼
    # ==========================================================
    def manual_cb(self, msg: Bool):
        if msg.data != self.manual_control:
            self.manual_control = msg.data
            self.get_logger().info(f'手動模式切換為 {self.manual_control}')

    # ==========================================================
    # 主循環：決策
    # ==========================================================
    def main_loop(self):
        # ---------- A. 手動模式 ----------
        if self.manual_control:
            return  # 自動導航 / 追球全部暫停

        # ---------- B. 紅球追蹤 ----------
        if self.red_ball_detected:
            # 若正在導航 → 取消
            if self.navigating and self.goal_handle:
                self.goal_handle.cancel_goal_async()
                self.navigating = False
                self.get_logger().info('偵測到紅球，取消當前導航並追蹤')
            self.track_ball()
            return

        # ---------- C. 巡邏導航 ----------
        if not self.navigating and self.goal_idx < len(self.goals):
            self.send_goal(self.goals[self.goal_idx])

    # ==========================================================
    # 追蹤紅球
    # ==========================================================
    def track_ball(self):
        if self.ball_cx is None or self.img_w is None:
            return
        err = self.ball_cx - self.img_w // 2
        twist = Twist()

        # 轉向
        if abs(err) > 50:
            twist.angular.z = -0.3 if err > 0 else 0.3
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.12  # 球大致置中 → 前進

        self.cmd_vel_pub.publish(twist)

    # ==========================================================
    # 送出導航目標
    # ==========================================================
    def send_goal(self, pos):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = pos[0]
        goal.pose.pose.position.y = pos[1]
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'導航至 #{self.goal_idx + 1}: {pos}')
        self.navigating = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_resp_cb)

    def goal_resp_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('導航目標被拒絕')
            self.navigating = False
            return

        self.goal_handle.get_result_async().add_done_callback(
            self.goal_result_cb)

    # ==========================================================
    # 導航結果
    # ==========================================================
    def goal_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'到達第 {self.goal_idx + 1} 點')
            self.goal_idx += 1
        else:
            self.get_logger().warn(f'導航失敗 (status={status})，'
                                   '前進 0.5 s 後重試')
            self.bump_forward()

        self.navigating = False

        # 全部完成 → 結束
        if self.goal_idx >= len(self.goals):
            self.get_logger().info('所有巡邏點完成，結束程式')
            rclpy.shutdown()

    # ==========================================================
    # bump_forward：失敗後向前小步
    # ==========================================================
    def bump_forward(self):
        twist = Twist()
        twist.linear.x = 0.05
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)


# ---------------- 主進入點 ----------------
def main(args=None):
    rclpy.init(args=args)
    node = PatrolAndBall()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()