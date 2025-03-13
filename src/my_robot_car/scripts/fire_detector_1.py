#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
import numpy as np

class FireDetector(Node):
    def __init__(self):
        super().__init__('fire_detector')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # 使用 USB 相機的影像主題
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.manual_control_active = False  # 初始為自動導航模式
        self.get_logger().info('火焰偵測節點已啟動。')

    def toggle_manual_control(self, msg):
        # 切換手動控制狀態
        self.manual_control_active = msg.data
        if msg.data:
            self.get_logger().info('手動控制已啟動。')
        else:
            self.get_logger().info('自動導航已啟動。')

    def image_callback(self, data):
        if self.manual_control_active:
            return  # 如果處於手動控制狀態，不做任何操作

        # 自動導航模式下進行影像處理
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 定義火焰的顏色範圍（黃色 + 橘色 + 紅色）
        lower_fire1 = np.array([0, 120, 200])   # 紅色部分
        upper_fire1 = np.array([10, 255, 255])

        lower_fire2 = np.array([15, 100, 200])  # 橘色部分
        upper_fire2 = np.array([25, 255, 255])

        lower_fire3 = np.array([26, 150, 200])  # 黃色部分
        upper_fire3 = np.array([35, 255, 255])

        # 生成火焰範圍的掩膜
        mask1 = cv2.inRange(hsv, lower_fire1, upper_fire1)
        mask2 = cv2.inRange(hsv, lower_fire2, upper_fire2)
        mask3 = cv2.inRange(hsv, lower_fire3, upper_fire3)
        mask = cv2.bitwise_or(mask1, cv2.bitwise_or(mask2, mask3))

        # 找到掩膜中的輪廓
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            self.get_logger().info(f'偵測到火焰位置: [{x}, {y}, {w}, {h}]')

            # 控制機器人根據火焰的位置移動
            center_left = 900
            center_right = 1000
            fire_center_x = x + w // 2

            if center_left < fire_center_x < center_right:
                twist.angular.z = 0.0  # 火焰位於中央
                twist.linear.x = 0.0
            elif fire_center_x < center_left:
                twist.angular.z = 0.1  # 向左旋轉
            elif fire_center_x > center_right:
                twist.angular.z = -0.1  # 向右旋轉

            # 根據火焰的大小控制前進或後退
            if w < 190:
                twist.linear.x = 0.2  # 向前移動
            elif w > 210:
                twist.linear.x = -0.2  # 向後移動
        else:
            # 未偵測到火焰時，原地旋轉
            twist.angular.z = 0.5

        self.publisher.publish(twist)
        cv2.imshow("攝像頭視圖", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    fire_detector = FireDetector()

    # 訂閱手動控制狀態的切換
    manual_control_subscriber = fire_detector.create_subscription(
        Bool,
        '/manual_control',
        fire_detector.toggle_manual_control,
        10
    )

    rclpy.spin(fire_detector)
    fire_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()