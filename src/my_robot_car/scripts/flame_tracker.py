#!/usr/bin/env python3
# ==========================================
#  flame_tracker_visual.py
#  - 讀取 Arduino (AMG8833+SEN0158) 串列資料
#  - 以 Ix 判斷左右誤差讓車子置中
#  - S ≥ 2 & 已置中 → 停止 (準備發射)
#  - 同時用 OpenCV 顯示座標格與火源點 (翻轉後)
# ==========================================
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import serial, time, cv2, numpy as np

# ---------- 顯示參數 ----------
WIDTH, HEIGHT = 1000, 750
GRID_STEP      = 50
color_map = [  # 16 階強度對應色
    (50, 50, 255), (100, 100, 255), (130, 150, 255), (0, 200, 255),
    (0, 255, 255), (0, 255, 150), (0, 255, 100), (0, 255, 50),
    (0, 255,   0), (100, 255,  0), (180, 255,  0), (255, 255,   0),
    (255, 200, 0), (255, 150,   0), (255,  80,  0), (255, 255, 255)
]

class FlameTracker(Node):
    def __init__(self):
        super().__init__('flame_tracker_visual')

        # -------- ROS 參數 (可 launch 覆寫) --------
        port = self.declare_parameter('serial_port', '/dev/ttyUSB0') \
                   .get_parameter_value().string_value
        baud = self.declare_parameter('baud_rate', 500000) \
                   .get_parameter_value().integer_value
        self.center_tol = self.declare_parameter('center_tolerance', 40) \
                             .get_parameter_value().integer_value
        self.angular_spd = self.declare_parameter('angular_speed', 0.35) \
                              .get_parameter_value().double_value
        self.forward_spd = self.declare_parameter('forward_speed', 0.18) \
                              .get_parameter_value().double_value
        self.shoot_thr   = self.declare_parameter('shoot_threshold', 2) \
                              .get_parameter_value().integer_value
        # -----------------------------------------

        # 串列阜
        try:
            self.ser = serial.Serial(port, baud, timeout=0.03)
            time.sleep(2)  # Arduino reset
        except serial.SerialException as e:
            self.get_logger().fatal(f"Serial open failed: {e}")
            raise SystemExit

        # ROS Pub / Sub
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Bool, '/manual_control', self.toggle_manual, 10)
        self.manual_control_active = False

        # OpenCV 視窗
        cv2.namedWindow("🔥 Flame Tracker", cv2.WINDOW_NORMAL)

        # 20 Hz timer
        self.create_timer(0.05, self.loop)
        self.get_logger().info("🔥 Flame tracker with visualization started.")

    # ---------- Manual Control ----------
    def toggle_manual(self, msg: Bool):
        self.manual_control_active = msg.data
        if msg.data:
            self.get_logger().info("手動控制啟動，停止自動導航。")
            self.cmd_pub.publish(Twist())   # 急停
        else:
            self.get_logger().info("自動導航啟動。")

    # ---------- 主迴圈 ----------
    def loop(self):
        # ---- 讀取一行序列 ----
        raw = self.ser.readline()
        if not raw:
            return
        parts = raw.decode(errors='ignore').strip().split(',')
        if not parts or parts[0] != 'T' or 'F' not in parts:
            return
        idx_f = parts.index('F')
        fire_raw = parts[idx_f + 1:]
        if len(fire_raw) != 12:
            return  # 讀取失敗

        # ---- 找最大 S 的點 ----
        max_s, target_ix = -1, None
        coords = []  # 用於畫面顯示
        for i in range(0, 12, 3):
            ix, iy, s = map(int, fire_raw[i:i+3])
            if ix == 1023 or iy == 1023:
                continue
            coords.append((ix, iy, s))
            if s > max_s:
                max_s, target_ix = s, ix

        # ---- 顯示畫面 (翻 180° 與原程式一致) ----
        frame = np.zeros((HEIGHT, WIDTH, 3), np.uint8)

        # 畫格線
        for x in range(0, WIDTH, GRID_STEP):
            cv2.line(frame, (x, 0), (x, HEIGHT), (0, 255, 0), 1)
        for y in range(0, HEIGHT, GRID_STEP):
            cv2.line(frame, (0, y), (WIDTH, y), (0, 255, 0), 1)

        # 先畫點
        for ix, iy, s in coords:
            x = int(iy * WIDTH  / 1024)
            y = int(ix * HEIGHT / 1024)
            clr = color_map[min(s, 15)]
            cv2.circle(frame, (x, y), 10, clr, -1)

        # 整張翻轉
        frame = cv2.flip(frame, -1)

        # 再畫文字 (翻轉後座標)
        for ix, iy, s in coords:
            x = WIDTH  - int(iy * WIDTH  / 1024)
            y = HEIGHT - int(ix * HEIGHT / 1024)
            cv2.putText(frame, f"{x},{y}", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        color_map[min(s, 15)], 2)

        cv2.imshow("🔥 Flame Tracker", frame)
        cv2.waitKey(1)

        # ---- 控制 (若非手動) ----
        if self.manual_control_active:
            return

        twist = Twist()
        if target_ix is None:
            # 沒看到火源：原地旋轉找
            twist.angular.z = self.angular_spd
            twist.linear.x  = 0.0
            self.get_logger().info_once("未偵測到火源，原地旋轉。")
        else:
            # 左右誤差（Ix 左大右小 → 翻轉）
            err = (1023 - target_ix) - 512
            if abs(err) > self.center_tol:
                twist.angular.z = self.angular_spd if err > 0 else -self.angular_spd
            else:
                twist.angular.z = 0.0
                # 進距離控制：S < 門檻時前進
                twist.linear.x = self.forward_spd if max_s < self.shoot_thr else 0.0

            # S 達門檻且置中 → 停
            if max_s >= self.shoot_thr and abs(err) <= self.center_tol:
                twist = Twist()  # 全停
                self.get_logger().info("S >= 2 且已置中，停止準備發射。")

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