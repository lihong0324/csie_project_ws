#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
import serial
import threading
import numpy as np
from cv_bridge import CvBridge

class FireDetector(Node):
    def __init__(self):
        super().__init__('fire_detector')

        # 初始化影像發布者
        self.image_publisher = self.create_publisher(Image, 'fire_detection/image', 10)
        self.thermal_publisher = self.create_publisher(Float32MultiArray, 'fire_detection/thermal', 10)

        # 設定 USB 相機
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()

        # 設定熱成像 USB 串列通訊
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # 儲存 8x8 溫度矩陣
        self.thermal_matrix = np.zeros((8, 8))

        # 開啟執行緒
        self.stop_flag = False
        self.thread = threading.Thread(target=self.process_data)
        self.thread.start()

        self.get_logger().info("🔥 火焰偵測節點已啟動")

    def process_data(self):
        while not self.stop_flag:
            # 讀取熱成像數據
            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode('utf-8').strip()
                try:
                    pixels = list(map(float, data.split(',')))
                    if len(pixels) == 64:
                        self.thermal_matrix = np.array(pixels).reshape((8, 8))
                        temp_msg = Float32MultiArray(data=self.thermal_matrix.flatten().tolist())
                        self.thermal_publisher.publish(temp_msg)

                except ValueError:
                    continue

            # 讀取相機畫面
            ret, frame = self.cap.read()
            if ret:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_publisher.publish(ros_image)

    def destroy_node(self):
        self.stop_flag = True
        self.thread.join()
        self.cap.release()
        self.ser.close()
        super().destroy_node()
        self.get_logger().info("🔥 火焰偵測節點已關閉")

def main(args=None):
    rclpy.init(args=args)
    node = FireDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()