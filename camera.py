#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        timer_period = 0.033  # seconds (約 30 FPS)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # --- 相機設定 ---
        self.cap = cv2.VideoCapture(0)  # 0 通常是第一個 USB 相機
        if not self.cap.isOpened():
            self.get_logger().error('無法開啟 USB 相機！請檢查相機是否連接以及索引是否正確 (例如 /dev/video0)。')
            rclpy.shutdown()
            return
        
        # 您可以在這裡設定相機的解析度，例如：
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        
        # 取得實際設定的解析度並記錄
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'相機已開啟，解析度: {self.frame_width}x{self.frame_height}')
        
        self.bridge = CvBridge()
        self.get_logger().info('相機發佈節點已啟動，將發佈影像到 /image_raw 主題。')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # 將 OpenCV 影像 (BGR) 轉換為 ROS Image 訊息
            # 如果您的 ball_detector 期望 'rgb8'，可以在這裡轉換或在 ball_detector 中轉換
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # 設定訊息的時間戳
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = "camera_frame" # 您可以自訂 frame_id
            
            self.publisher_.publish(ros_image_msg)
        else:
            self.get_logger().warn('無法從相機讀取影像幀。')

    def destroy_node(self):
        self.get_logger().info('正在關閉相機...')
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()