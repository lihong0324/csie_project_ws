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
            '/image_raw', 
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.manual_control_active = False
        self.image_width = None

        cv2.namedWindow("攝像頭偵測視圖", cv2.WINDOW_NORMAL)
        self.get_logger().info('球體偵測節點已啟動，等待 /image_raw 的影像...')

    def toggle_manual_control(self, msg):
        self.manual_control_active = msg.data
        if msg.data:
            self.get_logger().info('手動控制已啟動。')
            stop_twist = Twist()
            self.publisher.publish(stop_twist)
        else:
            self.get_logger().info('自動導航已啟動。')

    def image_callback(self, data):
        if self.manual_control_active:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        if self.image_width is None:
            self.image_width = frame.shape[1]
            self.get_logger().info(f'接收到第一幀影像，寬度設定為: {self.image_width}')

        self.get_logger().info(f"畫面平均亮度: {np.mean(frame):.2f}")

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:
                x, y, w, h = cv2.boundingRect(largest_contour)
                ((cx, cy), radius) = cv2.minEnclosingCircle(largest_contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 0, 255), 2)

                self.get_logger().info(f'偵測到紅球位置: [{int(cx)}, {int(cy)}], 寬度: {w}')

                image_center_x = self.image_width / 2
                tolerance = self.image_width * 0.05 
                left = image_center_x - tolerance
                right = image_center_x + tolerance

                if left < cx < right:
                    twist.angular.z = 0.0
                    twist.linear.x = 0.0
                    self.get_logger().info('紅球在中央，完全停止')
                elif cx < left:
                    twist.angular.z = 0.3
                    self.get_logger().info('紅球在左側，左轉')
                else:
                    twist.angular.z = -0.3
                    self.get_logger().info('紅球在右側，右轉')

                if w < 100:
                    twist.linear.x = 0.15
                    self.get_logger().info(f'球體寬度 {w} < 100，前進')
                elif w > 150:
                    twist.linear.x = -0.15
                    self.get_logger().info(f'球體寬度 {w} > 150，後退')
                else:
                    twist.linear.x = 0.0
                    self.get_logger().info(f'球體寬度 {w} 在期望範圍內')
            else:
                twist.angular.z = 0.0
                twist.linear.x = 0.15
                self.get_logger().info('未偵測到足夠大的紅球，前進尋找紅球。')
        else:
            twist.angular.z = 0.45
            twist.linear.x = 0.0
            self.get_logger().info('未偵測到紅球，原地旋轉。')

        self.publisher.publish(twist)
        cv2.imshow("攝像頭偵測視圖", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    ball_detector = BallDetector()

    manual_control_subscriber = ball_detector.create_subscription(
        Bool,
        '/manual_control',
        ball_detector.toggle_manual_control,
        10
    )

    try:
        rclpy.spin(ball_detector)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        ball_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()