#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import serial, time, cv2, numpy as np
import glob

WIDTH, HEIGHT = 1000, 750
GRID_STEP = 50
TOLERANCE_TICKS = 40  # ~2 秒（20Hz）

color_map = [
    (50, 50, 255), (100, 100, 255), (130, 150, 255), (0, 200, 255),
    (0, 255, 255), (0, 255, 150), (0, 255, 100), (0, 255, 50),
    (0, 255,   0), (100, 255,  0), (180, 255,  0), (255, 255,   0),
    (255, 200, 0), (255, 150,   0), (255,  80,  0), (255, 255, 255)
]

class FlameTracker(Node):
    def __init__(self):
        super().__init__('flame_tracker_visual')

        self.center_tol = self.declare_parameter('center_tolerance', 80).get_parameter_value().integer_value
        self.angular_spd = self.declare_parameter('angular_speed', 0.15).get_parameter_value().double_value
        self.forward_spd = self.declare_parameter('forward_speed', 0.18).get_parameter_value().double_value  # 原速設為 0.18，火源出現後改慢速
        self.shoot_thr   = self.declare_parameter('shoot_threshold', 2).get_parameter_value().integer_value

        self.ser = self.find_serial_port(baudrate=115200)
        if not self.ser:
            self.get_logger().fatal("找不到可用的 Arduino 串列埠")
            raise SystemExit

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/manual_control', self.toggle_manual, 10)
        self.manual_control_active = False

        self.warned_no_fire = False
        self.ready_ticks = 0
        self.centered_ticks = 0  # 火源穩定計數
        self.slow_mode = False  # 是否已啟用慢速模式
        self.ready_to_shoot = False

        cv2.namedWindow("Flame Tracker", cv2.WINDOW_NORMAL)
        self.create_timer(0.05, self.loop)
        self.get_logger().info("Flame tracker 啟動（畫面美化 + 無終端輸出）")

    def find_serial_port(self, baudrate):
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        for port in ports:
            try:
                s = serial.Serial(port, baudrate, timeout=1)
                time.sleep(2)
                s.reset_input_buffer()
                test = s.readline().decode('utf-8', errors='ignore').strip()
                if test.count(',') >= 9:
                    self.get_logger().info(f"成功連接到 {port}")
                    return s
                s.close()
            except:
                continue
        return None

    def toggle_manual(self, msg: Bool):
        self.manual_control_active = msg.data
        if msg.data:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("手動模式啟動")
        else:
            self.get_logger().info("自動模式啟動")

    
def loop(self):
    ret, frame = self.get_camera_frame()
    if not ret:
        return

    cx, cy, intensity = self.detect_flame(frame)

    twist = Twist()
    status_str = "Searching"
    mode_str = "Auto" if not self.manual_control_active else "Manual"

    # 火源出現才進入慢速模式
    if intensity >= self.shoot_thr:
        if not self.slow_mode:
            self.angular_spd = 0.15
            self.forward_spd = 0.08
            self.slow_mode = True

        if abs(cx - WIDTH // 2) < self.center_tol:
            self.centered_ticks += 1
        else:
            self.centered_ticks = 0

        if self.centered_ticks > 20:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            status_str = "S >= threshold & centered: READY TO FIRE"
            cv2.putText(frame, status_str, (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 3)
            self.cmd_pub.publish(Twist())
            cv2.imshow("Flame Tracker", frame)
            cv2.waitKey(1)
            return
        else:
            # 中心校正中，正常追蹤
            error_x = cx - WIDTH // 2
            twist.angular.z = -self.angular_spd * (error_x / (WIDTH // 2))
            twist.linear.x = self.forward_spd
            status_str = "Centering..."
    else:
        self.slow_mode = False
        self.centered_ticks = 0
        status_str = "No Fire"

    # 顯示基本資訊
    cv2.putText(frame, f"🔥 Flame Tracker ({mode_str})", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
    cv2.putText(frame, f"Intensity: {intensity}", (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
    cv2.putText(frame, f"Center Ticks: {self.centered_ticks}", (10, 95),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
    cv2.putText(frame, f"Status: {status_str}", (10, 125),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 100), 2)

    cv2.imshow("Flame Tracker", frame)
    cv2.waitKey(1)

    if not self.manual_control_active:
        self.cmd_pub.publish(twist)
def main(args=None):
    rclpy.init(args=args)
    node = FlameTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()