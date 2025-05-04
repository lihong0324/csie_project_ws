#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Temperature
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class FireDetectionAndControlNode(Node):
    def __init__(self):
        super().__init__('fire_detection_and_control_node')
        
        # 建立QoS設定
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 訂閱相機影像
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',  # 使用 USB 相機的影像主題
            self.image_callback,
            qos_profile
        )
        
        # 訂閱熱成像溫度數據
        self.thermal_sub = self.create_subscription(
            Temperature,
            '/thermal_camera/temperature',
            self.thermal_callback,
            qos_profile
        )
        
        # 訂閱手動控制狀態
        self.manual_control_sub = self.create_subscription(
            Bool,
            '/manual_control',
            self.toggle_manual_control,
            qos_profile
        )
        
        # 創建機器人移動控制發布者
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile
        )
        
        # 創建馬達控制發布者
        self.servo_pub = self.create_publisher(
            Float32MultiArray,
            '/servo_control',
            qos_profile
        )
        
        # 創建火焰檢測結果發布者
        self.flame_detection_pub = self.create_publisher(
            Bool,
            '/flame_detection',
            qos_profile
        )
        
        # 創建熱成像數據發布者
        self.thermal_data_pub = self.create_publisher(
            Temperature,
            '/thermal_data',
            qos_profile
        )
        
        # 初始化CV橋接器用於轉換ROS圖像訊息
        self.bridge = CvBridge()
        
        # 狀態變量
        self.manual_control_active = False  # 初始為自動導航模式
        self.high_temp_detected = False
        self.flame_detected = False
        self.detection_start_time = None
        self.detection_duration = 0.0
        self.servo_activated = False
        self.fire_position = None
        
        self.get_logger().info('火焰檢測與控制節點已初始化')
        
        # 建立定時器，定期檢查狀態 (10Hz)
        self.timer = self.create_timer(0.1, self.check_detection_status)
    
    def toggle_manual_control(self, msg):
        # 切換手動控制狀態
        self.manual_control_active = msg.data
        if msg.data:
            self.get_logger().info('手動控制已啟動')
        else:
            self.get_logger().info('自動導航已啟動')
            # 重置檢測狀態
            self.detection_start_time = None
            self.detection_duration = 0.0
            self.servo_activated = False
    
    def thermal_callback(self, msg):
        # 檢查溫度是否超過閾值 (80度)
        if msg.temperature >= 80.0:
            self.high_temp_detected = True
            self.get_logger().debug(f'高溫警告: {msg.temperature}°C')
        else:
            self.high_temp_detected = False
        
        # 轉發溫度數據到網頁
        self.thermal_data_pub.publish(msg)
    
    def image_callback(self, data):
        # 如果處於手動控制狀態，不做影像處理
        if self.manual_control_active:
            return
        
        # 自動導航模式下進行影像處理
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # 定義火焰的顏色範圍
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
                
                # 更新火焰位置信息
                self.fire_position = (x, y, w, h)
                
                # 檢測到火焰
                self.flame_detected = True
                
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
                
                self.get_logger().debug(f'偵測到火焰位置: [{x}, {y}, {w}, {h}]')
            else:
                # 未偵測到火焰
                self.flame_detected = False
                self.fire_position = None
                
                # 未偵測到火焰時，原地旋轉
                twist.angular.z = 0.5
            
            # 發布火焰檢測狀態給網頁介面
            flame_msg = Bool()
            flame_msg.data = self.flame_detected
            self.flame_detection_pub.publish(flame_msg)
            
            # 發布移動控制命令
            self.cmd_vel_pub.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f'圖像處理錯誤: {str(e)}')
    
    def check_detection_status(self):
        # 如果處於手動控制模式，不進行檢測
        if self.manual_control_active:
            return
        
        # 檢查是否同時滿足高溫和火焰檢測條件
        if self.high_temp_detected and self.flame_detected:
            current_time = time.time()
            
            # 如果是首次檢測到，記錄開始時間
            if self.detection_start_time is None:
                self.detection_start_time = current_time
                self.get_logger().info('開始檢測高溫和火焰')
            
            # 計算持續時間
            self.detection_duration = current_time - self.detection_start_time
            
            # 如果持續時間超過2秒且尚未觸發馬達，則觸發馬達
            if self.detection_duration >= 2.0 and not self.servo_activated:
                self.activate_servos()
                self.servo_activated = True
                self.get_logger().info(f'火焰已持續檢測 {self.detection_duration:.2f} 秒, 觸發馬達')
        else:
            # 如果條件不滿足，重置計時器
            self.detection_start_time = None
            self.detection_duration = 0.0
    
    def activate_servos(self):
        # 創建馬達控制消息，控制兩個MG90馬達
        # 第一個馬達向左轉動90度，第二個馬達向右轉動90度
        servo_msg = Float32MultiArray()
        
        # 假設角度範圍是0-180度，90度是中間位置
        # 左轉90度，角度從90減到0
        # 右轉90度，角度從90增到180
        servo_msg.data = [0.0, 180.0]  # [左馬達角度, 右馬達角度]
        
        self.servo_pub.publish(servo_msg)
        self.get_logger().info('馬達已啟動: 左馬達向左轉90度，右馬達向右轉90度')
    
    def reset_servos(self):
        # 重置馬達到中間位置
        servo_msg = Float32MultiArray()
        servo_msg.data = [90.0, 90.0]  # 回到中間位置
        
        self.servo_pub.publish(servo_msg)
        self.get_logger().info('馬達已重置到中間位置')

def main(args=None):
    rclpy.init(args=args)
    node = FireDetectionAndControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('關閉火焰檢測與控制節點')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()