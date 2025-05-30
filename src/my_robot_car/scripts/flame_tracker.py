#!/usr/bin/env python3
# flame_tracker.py  – 2025-05-29  (rev. ball-logic)

import rclpy, cv2, serial, time, glob, numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# ---------- 畫面參數 ----------
WIDTH, HEIGHT = 1000, 750
GRID_STEP      = 50
COLOR_MAP = [  # 0-15 強度 ⇒ 顏色
    (50,50,255),(100,100,255),(130,150,255),(0,200,255),
    (0,255,255),(0,255,150),(0,255,100),(0,255,50),
    (0,255,0),(100,255,0),(180,255,0),(255,255,0),
    (255,200,0),(255,150,0),(255,80,0),(255,255,255)
]

class FlameTracker(Node):
    def __init__(self):
        super().__init__('flame_tracker_visual')

        # ---- 可由 launch 覆寫的 ROS 參數 ----
        self.shoot_thr    = self.declare_parameter('shoot_threshold', 2).get_parameter_value().integer_value
        self.forward_spd  = self.declare_parameter('forward_speed', 0.18).get_parameter_value().double_value
        self.backward_spd = self.declare_parameter('backward_speed', 0.10).get_parameter_value().double_value
        self.angular_spd  = self.declare_parameter('angular_speed', 0.35).get_parameter_value().double_value

        # ---- 自動搜尋 /dev/ttyACM0 ----
        self.ser = self._auto_serial(baud=115200)
        if not self.ser:
            self.get_logger().fatal("❌ 找不到 Arduino 串列埠")
            raise SystemExit

        # ---- ROS Pub / Sub ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/manual_control', self._toggle_manual, 10)
        self.manual_mode   = False
        self.fire_detected = False

        # ---- OpenCV 視窗 ----
        cv2.namedWindow("🔥 Flame Tracker", cv2.WINDOW_NORMAL)

        # 20 Hz timer
        self.create_timer(0.05, self._loop)
        self.get_logger().info("🔥 Flame tracker 已啟動（改用 ball_detector 移動邏輯）")

    # ---------- 自動偵測 Serial Port ----------
    def _auto_serial(self, baud:int):
        for p in glob.glob('/dev/ttyACM0'):
            try:
                s = serial.Serial(p, baud, timeout=1)
                time.sleep(2)
                if s.readline().decode('utf-8', 'ignore').count(',') >= 9:
                    self.get_logger().info(f"✅ 連接 {p}")
                    return s
                s.close()
            except: pass
        return None

    # ---------- Manual / Auto 切換 ----------
    def _toggle_manual(self, msg:Bool):
        self.manual_mode = msg.data
        self.cmd_pub.publish(Twist())         # 立即急停
        self.get_logger().info("🛑 手動模式" if msg.data else "✅ 自動模式")

    # ---------- 主迴圈 ----------
    def _loop(self):
        raw = self.ser.readline()
        if not raw:
            return
        try:
            parts = list(map(int, raw.decode(errors='ignore').strip().split(',')))
        except ValueError:
            return
        if len(parts) % 3:
            return

        # 解析火焰座標 & 強度
        coords, max_s = [], -1
        for i in range(0, len(parts), 3):
            ix, iy, s = parts[i:i+3]
            if ix == 1023 or iy == 1023:      # 該像素無效
                continue
            coords.append((ix, iy, s))
            max_s = max(max_s, s)

        # ---------- 視覺化 ----------
        frame = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        for x in range(0, WIDTH, GRID_STEP):
            cv2.line(frame, (x, 0), (x, HEIGHT), (0, 255, 0), 1)
        for y in range(0, HEIGHT, GRID_STEP):
            cv2.line(frame, (0, y), (WIDTH, y), (0, 255, 0), 1)

        for ix, iy, s in coords:
            cx = int(iy * WIDTH  / 1024)
            cy = int(ix * HEIGHT / 1024)
            cv2.circle(frame, (cx, cy), 10, COLOR_MAP[min(s, 15)], -1)
        frame = cv2.flip(frame, -1)  # 180°

        for ix, iy, s in coords:
            cx = WIDTH  - int(iy * WIDTH  / 1024)
            cy = HEIGHT - int(ix * HEIGHT / 1024)
            cv2.putText(frame, f"{cx},{cy}", (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR_MAP[min(s, 15)], 2)

        # ---------- 半透明資訊底 ------
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (420, 170), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.4, frame, 0.6, 0, frame)

        # ---------- 控制邏輯 ----------
        twist       = Twist()
        mode_str    = "Manual Mode" if self.manual_mode else "Auto Mode"
        status_str  = "Manual"

        # 1) 手動模式 → 立即返回
        if self.manual_mode:
            self.fire_detected = False

        # 2) 無火焰（所有 s=15 或無有效座標）→ 原地旋轉
        elif (max_s == 15) or (not coords):
            self.fire_detected = False
            twist.angular.z =  self.angular_spd
            status_str      = "Searching (Rotating)"

        # 3) 有火焰 → ball_detector 風格中心對齊 + 距離判斷
        else:
            self.fire_detected = True

            # 3-a 取「最靠近」（s 最小）的點作為目標
            target   = min(coords, key=lambda c: c[2])
            cx_pixel = WIDTH - int(target[1] * WIDTH / 1024)   # 已經 flip 過
            img_mid  = WIDTH / 2
            tol      = WIDTH * 0.10
            left, right = img_mid - tol, img_mid + tol

            # -- 角速度（左右旋轉）--
            if left < cx_pixel < right:
                twist.angular.z = 0.0
            elif cx_pixel < left:
                twist.angular.z =  self.angular_spd     # 左側 → 左轉
            else:
                twist.angular.z = -self.angular_spd      # 右側 → 右轉

            # -- 線速度（前進 / 後退）--
            if max_s < self.shoot_thr:                   # 太遠 → 前進
                twist.linear.x =  self.forward_spd
                status_str = f"S={max_s} < {self.shoot_thr} → Forward"
            else:                                        # 達門檻 → 後退 + 準備發射
                twist.linear.x = -self.backward_spd
                status_str = f"S≥{self.shoot_thr} → Backward & Shoot"

        # ---------- 畫面文字 ----------
        cv2.putText(frame, f"🔥 Flame Tracker ({mode_str})", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
        cv2.putText(frame, f"Max Strength: {max_s}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        cv2.putText(frame, f"Fire Detected: {self.fire_detected}", (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
        cv2.putText(frame, f"Status: {status_str}", (10, 125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 100), 2)

        if self.fire_detected and max_s >= self.shoot_thr:
            cv2.putText(frame, "🔥 S ≥ 2，準備發射", (10, 155),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("🔥 Flame Tracker", frame)
        cv2.waitKey(1)

        # ---------- 發送速度指令 ----------
        if not self.manual_mode:
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
