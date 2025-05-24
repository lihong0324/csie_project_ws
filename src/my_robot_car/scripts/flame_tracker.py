#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import serial, time, cv2, numpy as np
import glob
from cv_bridge import CvBridge

WIDTH, HEIGHT = 1000, 750
GRID_STEP = 50
TOLERANCE_TICKS = 40  # About 2 seconds (20Hz timer)
TOO_CLOSE_THRESHOLD = 8  # If max strength > 8, move backward

# Enhanced color mapping for better visualization
color_map = [
    (50, 50, 255), (80, 80, 255), (110, 110, 255), (140, 140, 255),
    (0, 150, 255), (0, 180, 255), (0, 210, 255), (0, 240, 255),
    (0, 255, 200), (0, 255, 150), (0, 255, 100), (0, 255, 50),
    (50, 255, 0), (100, 255, 0), (150, 255, 0), (200, 255, 0),
    (255, 255, 0), (255, 200, 0), (255, 150, 0), (255, 100, 0),
    (255, 50, 0), (255, 0, 50), (255, 0, 100), (255, 255, 255)
]

class FlameTracker(Node):
    def __init__(self):
        super().__init__('flame_tracker_visual')

        # Parameters
        self.center_tol = self.declare_parameter('center_tolerance', 40).get_parameter_value().integer_value
        self.angular_spd = self.declare_parameter('angular_speed', 0.35).get_parameter_value().double_value
        self.forward_spd = self.declare_parameter('forward_speed', 0.18).get_parameter_value().double_value
        self.backward_spd = self.declare_parameter('backward_speed', 0.15).get_parameter_value().double_value
        self.shoot_thr = self.declare_parameter('shoot_threshold', 2).get_parameter_value().integer_value

        # Serial connection to Arduino
        self.ser = self.find_serial_port(baudrate=115200)
        if not self.ser:
            self.get_logger().fatal("Failed to find Arduino serial port")
            raise SystemExit

        # ROS publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/manual_control', self.toggle_manual, 10)
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        self.current_frame = None
        
        self.manual_control_active = False
        self.warned_no_fire = False
        self.ready_ticks = 0
        self.ready_to_shoot = False

        # Create windows
        cv2.namedWindow("Flame Tracker - Camera View", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Flame Tracker - Sensor View", cv2.WINDOW_NORMAL)
        
        # Timer for main loop
        self.create_timer(0.05, self.loop)
        self.get_logger().info("Flame tracker started with camera integration.")

    def find_serial_port(self, baudrate):
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        for port in ports:
            try:
                s = serial.Serial(port, baudrate, timeout=1)
                time.sleep(2)
                s.reset_input_buffer()
                test = s.readline().decode('utf-8', errors='ignore').strip()
                if test.count(',') >= 9:
                    self.get_logger().info(f"Successfully connected to {port}")
                    return s
                s.close()
            except Exception as e:
                self.get_logger().warn(f"Skipping {port}: {e}")
        return None

    def image_callback(self, msg: Image):
        """攝影機影像的回呼函式"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def toggle_manual(self, msg: Bool):
        self.manual_control_active = msg.data
        if msg.data:
            self.get_logger().info("手動控制已啟動，停止自主導航.")
            self.cmd_pub.publish(Twist())
        else:
            self.get_logger().info("自主導航模式已啟動.")

    def draw_status_panel(self, frame, status_info):
        """在畫面上繪製狀態資訊面板"""
        panel_height = 200
        panel_width = 400
        
        # Create semi-transparent overlay
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (panel_width, panel_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Draw border
        cv2.rectangle(frame, (10, 10), (panel_width, panel_height), (0, 255, 255), 2)
        
        y_offset = 40
        line_height = 25
        
        # Title
        cv2.putText(frame, "FLAME TRACKER", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        y_offset += line_height + 10
        
        # Status info
        for key, value in status_info.items():
            color = (255, 255, 255)
            if key == "Status":
                if "Ready" in str(value):
                    color = (0, 255, 0)
                elif "Manual" in str(value):
                    color = (255, 255, 0)
                elif "Rotating" in str(value):
                    color = (0, 165, 255)
                elif "Backward" in str(value):
                    color = (0, 0, 255)
            
            cv2.putText(frame, f"{key}: {value}", (20, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
            y_offset += line_height

    def draw_flame_indicators(self, frame, coords, target_ix, max_s):
        """在攝影機畫面上標示火源位置"""
        if not coords:
            return
            
        frame_height, frame_width = frame.shape[:2]
        
        for ix, iy, s in coords:
            # 將感測器座標轉換為攝影機座標
            x = int(iy * frame_width / 1024)
            y = int(ix * frame_height / 1024)
            
            # 根據火焰強度選擇顏色
            color_idx = min(int(s), len(color_map) - 1)
            color = color_map[color_idx]
            
            # 根據火焰強度畫出圓圈
            radius = max(5, min(20, int(s * 2)))
            cv2.circle(frame, (x, y), radius, color, -1)
            cv2.circle(frame, (x, y), radius + 2, (255, 255, 255), 2)
            
            # Draw strength text
            cv2.putText(frame, f"S:{s}", (x + radius + 5, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # 標示目標位置
            if ix == target_ix:
                cv2.circle(frame, (x, y), radius + 10, (0, 255, 0), 3)
                cv2.putText(frame, "TARGET", (x - 30, y + radius + 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def create_sensor_visualization(self, coords, target_ix, max_s):
        """建立感測器數據的視覺化畫面"""
        frame = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        
        # 繪製格線
        for x in range(0, WIDTH, GRID_STEP):
            cv2.line(frame, (x, 0), (x, HEIGHT), (40, 40, 40), 1)
        for y in range(0, HEIGHT, GRID_STEP):
            cv2.line(frame, (0, y), (WIDTH, y), (40, 40, 40), 1)

        # 繪製中央十字線
        cv2.line(frame, (WIDTH//2, 0), (WIDTH//2, HEIGHT), (0, 255, 0), 2)
        cv2.line(frame, (0, HEIGHT//2), (WIDTH, HEIGHT//2), (0, 255, 0), 2)

        # 繪製火焰來源位置
        for ix, iy, s in coords:
            x = int(iy * WIDTH / 1024)
            y = int(ix * HEIGHT / 1024)
            
            color_idx = min(int(s), len(color_map) - 1)
            color = color_map[color_idx]
            
            radius = max(8, min(25, int(s * 3)))
            cv2.circle(frame, (x, y), radius, color, -1)
            cv2.circle(frame, (x, y), radius + 2, (255, 255, 255), 1)
            
            # 高強度火焰加上發光效果
            if s > 5:
                cv2.circle(frame, (x, y), radius + 8, color, 2)
            
            cv2.putText(frame, f"{s}", (x + radius + 5, y + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        frame = cv2.flip(frame, -1)
        return frame

    def loop(self):
        # 讀取感測器資料
        raw = self.ser.readline()
        if not raw:
            return
            
        try:
            parts = list(map(int, raw.decode(errors='ignore').strip().split(',')))
        except:
            return
            
        if len(parts) % 3 != 0:
            return

        # 處理感測器資料
        max_s, target_ix = -1, None
        coords = []
        for i in range(0, len(parts), 3):
            ix, iy, s = parts[i:i+3]
            if ix == 1023 or iy == 1023:
                continue
            coords.append((ix, iy, s))
            if s > max_s:
                max_s, target_ix = s, ix

        # 控制邏輯
        twist = Twist()
        status_str = "Manual"

        if self.manual_control_active:
            self.ready_ticks = 0
            self.ready_to_shoot = False

        elif target_ix is None:
            twist.angular.z = self.angular_spd
            twist.linear.x = 0.0
            status_str = "Rotating - Searching"
            self.ready_ticks = 0
            self.ready_to_shoot = False
            if not self.warned_no_fire:
                self.get_logger().info("No fire source detected, rotating...")
                self.warned_no_fire = True

        else:
            self.warned_no_fire = False
            err = (1023 - target_ix) - 512

            # Check if fire is too close
            if max_s > TOO_CLOSE_THRESHOLD:
                twist.linear.x = -self.backward_spd
                twist.angular.z = 0.0
                status_str = "Backward - Too Close"
                self.ready_ticks = 0
                self.ready_to_shoot = False
            elif abs(err) > self.center_tol:
                twist.angular.z = self.angular_spd if err > 0 else -self.angular_spd
                status_str = "正在對準火源"
                self.ready_ticks = 0
                self.ready_to_shoot = False
            else:
                twist.angular.z = 0.0
                if max_s < self.shoot_thr:
                    twist.linear.x = self.forward_spd
                    status_str = "正在向火源前進"
                    self.ready_ticks = 0
                    self.ready_to_shoot = False
                else:
                    twist = Twist()
                    self.ready_ticks += 1
                    status_str = f"火源置中中 {self.ready_ticks * 0.05:.1f}s"
                    if self.ready_ticks >= TOLERANCE_TICKS:
                        self.ready_to_shoot = True
                        status_str = "READY TO SHOOT!"

        # 狀態資訊
        status_info = {
            "Mode": "Manual" if self.manual_control_active else "Auto",
            "Max Strength": max_s if max_s >= 0 else "None",
            "Target IX": target_ix if target_ix else "None",
            "Status": status_str,
            "Ready": "YES" if self.ready_to_shoot else "NO"
        }

        # 顯示疊加資訊的攝影機畫面
        if self.current_frame is not None:
            camera_display = self.current_frame.copy()
            self.draw_flame_indicators(camera_display, coords, target_ix, max_s)
            self.draw_status_panel(camera_display, status_info)
            
            # 在畫面中央加上準心
            h, w = camera_display.shape[:2]
            cv2.line(camera_display, (w//2 - 20, h//2), (w//2 + 20, h//2), (0, 255, 0), 2)
            cv2.line(camera_display, (w//2, h//2 - 20), (w//2, h//2 + 20), (0, 255, 0), 2)
            
            cv2.imshow("Flame Tracker - Camera View", camera_display)

        # 顯示感測器視覺化畫面
        sensor_frame = self.create_sensor_visualization(coords, target_ix, max_s)
        
        # 在感測器畫面上顯示狀態文字
        cv2.putText(sensor_frame, f"Flame Sensor Visualization", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(sensor_frame, f"Status: {status_str}", (10, 65),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        if self.ready_to_shoot:
            cv2.putText(sensor_frame, "READY TO SHOOT!", (WIDTH//2 - 100, HEIGHT - 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)

        cv2.imshow("Flame Tracker - Sensor View", sensor_frame)
        cv2.waitKey(1)

        # 發佈移動控制指令
        if not self.manual_control_active:
            self.cmd_pub.publish(twist)

    def destroy_node(self):
        cv2.destroyAllWindows()
        if self.ser:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FlameTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()