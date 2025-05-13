#!/usr/bin/env python3
# 偵測紅球節點
# 這個節點會訂閱 USB 攝像頭的影像，並使用 OpenCV 偵測紅色球體
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
import numpy as np

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw', # 使用 USB 相機的影像主題
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.manual_control_active = False  # 初始為自動導航模式
        self.get_logger().info('球體偵測節點已啟動。')

    def toggle_manual_control(self, msg):
        # 切換手動控制狀態
        self.manual_control_active = msg.data
        if msg.data:
            self.get_logger().info('手動控制已啟動。')
        else:
            self.get_logger().info('自動導航已啟動。')

    def image_callback(self, data):
        if self.manual_control_active:
            # 如果處於手動控制狀態，不做任何操作
            return

        # 自動導航模式下進行影像處理
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 定義紅色的範圍
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # 生成紅色範圍的掩膜
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 找到掩膜中的輪廓
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            self.get_logger().info(f'偵測到紅球位置: [{x}, {y}, {w}, {h}]')

            # 根據紅球位置控制機器人
            center_left = 900
            center_right = 1000
            ball_center_x = x + w // 2
            if center_left < ball_center_x < center_right:
                twist.angular.z = 0.0  # 紅球位於中央
                twist.linear.x = 0.0
            elif ball_center_x < center_left:
                twist.angular.z = 0.1  # 向左旋轉
            elif ball_center_x > center_right:
                twist.angular.z = -0.1  # 向右旋轉

            # 根據紅球的大小控制前進或後退
            if w < 190:
                twist.linear.x = 0.2  # 向前移動
            elif w > 210:
                twist.linear.x = -0.2  # 向後移動
        else:
            # 未偵測到紅球時，原地旋轉
            twist.angular.z = 0.5

        self.publisher.publish(twist)
        cv2.imshow("攝像頭視圖", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    ball_detector = BallDetector()

    # 訂閱手動控制狀態的切換
    manual_control_subscriber = ball_detector.create_subscription(
        Bool,
        '/manual_control',
        ball_detector.toggle_manual_control,
        10
    )

    rclpy.spin(ball_detector)
    ball_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()