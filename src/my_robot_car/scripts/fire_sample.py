#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import cv2
import numpy as np
from cv_bridge import CvBridge
'''
# 匯入AMG8833程式庫
import board
import adafruit_amg88xx
'''
class FireDetector(Node):
    def __init__(self):
        # 初始化ROS2節點
        super().__init__('fire_detector')
        
        # 初始化AMG8833熱感測器
        try:
            i2c = board.I2C()
            self.thermal_sensor = adafruit_amg88xx.AMG88XX(i2c)
        except Exception as e:
            # 若初始化失敗，記錄錯誤
            self.get_logger().error(f'無法初始化AMG8833: {e}')
            self.thermal_sensor = None
        
        # 訂閱攝影機影像主題
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # USB攝影機影像主題
            self.image_callback,
            10
        )
        
        # 建立熱影像發布者
        self.thermal_publisher = self.create_publisher(Image, '/thermal_overlay', 10)
        
        # 建立機器人移動指令發布者
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 建立ROS影像與OpenCV影像轉換橋接器
        self.bridge = CvBridge()
        
        # 手動控制標誌
        self.manual_control_active = False
        
        self.get_logger().info('火災偵測節點已初始化。')
    
    def toggle_manual_control(self, msg):
        # 切換手動控制狀態
        self.manual_control_active = msg.data
        if msg.data:
            self.get_logger().info('手動控制已啟動。')
        else:
            self.get_logger().info('自主導航已啟動。')
    
    def create_thermal_colormap(self, thermal_data):
        """
        將熱感測器數據轉換為彩色地圖
        使用從藍色（低溫）到紅色（高溫）的顏色漸層
        """
        # 將熱感測器數據轉換為NumPy陣列
        thermal_array = np.array(thermal_data)
        
        # 找出最小和最大溫度值
        min_temp = np.min(thermal_array)
        max_temp = np.max(thermal_array)
        
        # 將溫度數據標準化到0-255範圍
        normalized = (thermal_array - min_temp) / (max_temp - min_temp) * 255
        normalized = normalized.astype(np.uint8)
        
        # 應用彩色映射
        thermal_colormap = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
        
        return thermal_colormap
    
    def overlay_thermal_on_frame(self, frame, thermal_colormap):
        """
        將熱彩色映射疊加到原始畫面上
        """
        # 調整熱彩色映射大小以符合原始畫面
        thermal_resized = cv2.resize(thermal_colormap, (frame.shape[1], frame.shape[0]), 
                                     interpolation=cv2.INTER_LINEAR)
        
        # 混合原始畫面與熱疊加層
        alpha = 0.5  # 透明度因子
        blended = cv2.addWeighted(frame, 1 - alpha, thermal_resized, alpha, 0)
        
        return blended
    
    def image_callback(self, data):
        # 如果處於手動控制模式，則不執行任何操作
        if self.manual_control_active:
            return
        
        # 將ROS影像轉換為OpenCV畫面
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        # 讀取熱感測器數據
        if self.thermal_sensor:
            try:
                # 取得熱感測器像素數據
                thermal_data = self.thermal_sensor.pixels
                
                # 建立熱彩色映射
                thermal_colormap = self.create_thermal_colormap(thermal_data)
                
                # 將熱數據疊加到畫面上
                frame_with_thermal = self.overlay_thermal_on_frame(frame, thermal_colormap)
            except Exception as e:
                # 處理熱感測器數據時出現錯誤
                self.get_logger().warn(f'處理熱感測器數據時發生錯誤: {e}')
                frame_with_thermal = frame
        else:
            frame_with_thermal = frame
        
        # 將畫面轉換為HSV色彩空間
        hsv = cv2.cvtColor(frame_with_thermal, cv2.COLOR_BGR2HSV)
        
        # 定義火災顏色範圍（黃色 + 橙色 + 紅色）
        lower_fire1 = np.array([0, 120, 200])   # 紅色部分
        upper_fire1 = np.array([10, 255, 255])
        
        lower_fire2 = np.array([15, 100, 200])  # 橙色部分
        upper_fire2 = np.array([25, 255, 255])
        
        lower_fire3 = np.array([26, 150, 200])  # 黃色部分
        upper_fire3 = np.array([35, 255, 255])
        
        # 生成火災顏色範圍遮罩
        mask1 = cv2.inRange(hsv, lower_fire1, upper_fire1)
        mask2 = cv2.inRange(hsv, lower_fire2, upper_fire2)
        mask3 = cv2.inRange(hsv, lower_fire3, upper_fire3)
        mask = cv2.bitwise_or(mask1, cv2.bitwise_or(mask2, mask3))
        
        # 在遮罩中尋找輪廓
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        twist = Twist()
        
        if contours:
            # 找出最大輪廓
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame_with_thermal, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            self.get_logger().info(f'偵測到火災位置：[{x}, {y}, {w}, {h}]')
            
            # 根據火災位置控制機器人
            center_left = 900
            center_right = 1000
            fire_center_x = x + w // 2
            
            if center_left < fire_center_x < center_right:
                twist.angular.z = 0.0  # 火災位於中心
                twist.linear.x = 0.0
            elif fire_center_x < center_left:
                twist.angular.z = 0.1  # 向左旋轉
            elif fire_center_x > center_right:
                twist.angular.z = -0.1  # 向右旋轉
            
            # 根據火災大小控制前進或後退
            if w < 190:
                twist.linear.x = 0.2  # 向前移動
            elif w > 210:
                twist.linear.x = -0.2  # 向後移動
        else:
            # 未偵測到火災時原地旋轉
            twist.angular.z = 0.5
        
        # 發布移動指令
        self.publisher.publish(twist)
        
        # 將處理後的畫面轉換為ROS影像並發布
        try:
            thermal_overlay_msg = self.bridge.cv2_to_imgmsg(frame_with_thermal, encoding='bgr8')
            self.thermal_publisher.publish(thermal_overlay_msg)
        except Exception as e:
            self.get_logger().warn(f'發布熱疊加影像失敗：{e}')
        
        # 顯示畫面（可選，用於除錯）
        cv2.imshow("攝影機畫面與熱疊加", frame_with_thermal)
        cv2.waitKey(1)

def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    fire_detector = FireDetector()
    
    # 訂閱手動控制狀態切換
    manual_control_subscriber = fire_detector.create_subscription(
        Bool,
        '/manual_control',
        fire_detector.toggle_manual_control,
        10
    )
    
    # 持續執行節點
    rclpy.spin(fire_detector)
    fire_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()