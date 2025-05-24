#!/usr/bin/env python3
# 偵測紅球節點
# 這個節點會訂閱 USB 攝像頭的影像，並使用 OpenCV 偵測紅色圓形球體
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
        self.image_height = None

        # 創建帶有自定義屬性的視窗
        cv2.namedWindow("紅球偵測系統", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("紅球偵測系統", 800, 600)
        
        self.get_logger().info('紅球偵測節點已啟動，等待 /image_raw 的影像...')

    def toggle_manual_control(self, msg):
        self.manual_control_active = msg.data
        if msg.data:
            self.get_logger().info('手動控制已啟動。')
            stop_twist = Twist()
            self.publisher.publish(stop_twist)
        else:
            self.get_logger().info('自動導航已啟動。')

    def is_circular(self, contour, min_circularity=0.7):
        """
        檢查輪廓是否接近圓形
        circularity = 4π * area / perimeter²
        完美圓形的值為 1.0
        """
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        if perimeter == 0:
            return False
            
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        return circularity >= min_circularity

    def draw_ui_elements(self, frame):
        """繪製美化的UI元素"""
        h, w = frame.shape[:2]
        
        # 繪製中央區域指示線
        center_x = w // 2
        tolerance = int(w * 0.10)
        
        # 中央容許區域 (綠色)
        cv2.rectangle(frame, 
                     (center_x - tolerance, 0), 
                     (center_x + tolerance, h), 
                     (0, 255, 0), 2)
        
        # 中心線 (白色虛線效果)
        for y in range(0, h, 20):
            cv2.line(frame, (center_x, y), (center_x, y + 10), (255, 255, 255), 2)
        
        # 繪製狀態資訊背景
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (350, 120), (50, 50, 50), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # 標題
        cv2.putText(frame, "Red Ball Detection System", (20, 35), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 模式指示
        mode_text = "Manual Mode" if self.manual_control_active else "Auto Mode"
        mode_color = (0, 165, 255) if self.manual_control_active else (0, 255, 0)
        cv2.putText(frame, f"Mode: {mode_text}", (20, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 2)
        
        # 解析度資訊
        cv2.putText(frame, f"Resolution: {w}x{h}", (20, 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 中央區域說明
        cv2.putText(frame, f"Target Zone: {center_x-tolerance}-{center_x+tolerance}", (20, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    def draw_ball_info(self, frame, cx, cy, radius, w, h, is_circular):
        """繪製球體檢測資訊"""
        # 球體資訊背景
        overlay = frame.copy()
        cv2.rectangle(overlay, (int(cx) - 80, int(cy) - radius - 60), 
                     (int(cx) + 80, int(cy) - radius - 10), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # 球體資訊文字
        info_y = int(cy - radius - 45)
        cv2.putText(frame, f"Ball Detected!", (int(cx) - 75, info_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, f"Pos: ({int(cx)}, {int(cy)})", (int(cx) - 75, info_y + 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(frame, f"Size: {w}x{h}", (int(cx) - 75, info_y + 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # 圓形度指示
        shape_text = "Circular ✓" if is_circular else "Not Circular ✗"
        shape_color = (0, 255, 0) if is_circular else (0, 0, 255)
        cv2.putText(frame, shape_text, (int(cx) - 75, info_y + 45), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, shape_color, 1)

    def image_callback(self, data):
        if self.manual_control_active:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        if self.image_width is None or self.image_height is None:
            self.image_height, self.image_width = frame.shape[:2]
            self.get_logger().info(f'📷 接收到第一幀影像，解析度: {self.image_width}x{self.image_height}')

        # 繪製UI元素
        self.draw_ui_elements(frame)

        # HSV色彩空間轉換
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 定義紅色範圍（兩個範圍以涵蓋色相環兩端的紅色）
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # 建立遮罩
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 形態學操作來去除雜訊
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 尋找輪廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()
        ball_found = False

        if contours:
            # 找到最大的輪廓
            largest_contour = max(contours, key=cv2.contourArea)
            contour_area = cv2.contourArea(largest_contour)
            
            # 檢查面積是否足夠大
            if contour_area > 500:
                # 檢查是否為圓形
                is_circular = self.is_circular(largest_contour, min_circularity=0.6)
                
                if is_circular:
                    ball_found = True
                    
                    # 計算邊界框和最小外接圓
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    ((cx, cy), radius) = cv2.minEnclosingCircle(largest_contour)
                    
                    # 繪製檢測結果
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 255), 3)
                    cv2.circle(frame, (int(cx), int(cy)), 5, (255, 0, 255), -1)  # 中心點
                    
                    # 繪製球體資訊
                    self.draw_ball_info(frame, cx, cy, radius, w, h, is_circular)

                    self.get_logger().info(f'偵測到圓形紅球 - 位置: [{int(cx)}, {int(cy)}], 大小: {w}x{h}, 圓形度: ✓')

                    # 水平位置控制邏輯
                    image_center_x = self.image_width / 2
                    tolerance = self.image_width * 0.10
                    left = image_center_x - tolerance
                    right = image_center_x + tolerance

                    if left < cx < right:
                        twist.angular.z = 0.0
                        self.get_logger().info('紅球在中央區域')
                    elif cx < left:
                        twist.angular.z = 0.15
                        self.get_logger().info('紅球在左側，左轉')
                    else:
                        twist.angular.z = -0.15
                        self.get_logger().info('紅球在右側，右轉')

                    # 距離控制邏輯（基於球體寬度）
                    if w < 100:
                        twist.linear.x = 0.15
                        self.get_logger().info(f'球體太小 ({w} < 100)，前進')
                    elif w > 150:
                        twist.linear.x = -0.15
                        self.get_logger().info(f'球體太大 ({w} > 150)，後退')
                    else:
                        twist.linear.x = 0.0
                        self.get_logger().info(f'球體大小適中 ({w})')
                else:
                    # 找到紅色物體但不是圓形
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(frame, "Red Object (Not Circular)", (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    self.get_logger().info('偵測到紅色物體，但不是圓形')

        # 如果沒有找到符合條件的球體，只進行原地旋轉
        if not ball_found:
            twist.angular.z = 0.45  # 原地旋轉尋找
            twist.linear.x = 0.0    # 不前進
            self.get_logger().info('未偵測到圓形紅球，原地旋轉搜尋中...')
            
            # 顯示搜尋狀態
            cv2.putText(frame, "Searching for red ball...", (self.image_width//2 - 120, self.image_height//2), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # 發布控制指令
        self.publisher.publish(twist)
        
        # 顯示處理後的影像
        cv2.imshow("紅球偵測系統", frame)
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
        print("\n程序被用戶中止")
    finally:
        cv2.destroyAllWindows()
        ball_detector.destroy_node()
        rclpy.shutdown()
        print("紅球偵測節點已關閉")

if __name__ == '__main__':
    main()