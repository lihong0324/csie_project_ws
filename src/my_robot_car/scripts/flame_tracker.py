#!/usr/bin/env python3
# flame_tracker.py  ‒ 2025-05-27
# 保持火源在感測強度 2 的距離區間，15 表示最遠／未偵測

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import serial, time, cv2, numpy as np
import glob

# ---------- 畫面設定 ----------
WIDTH, HEIGHT = 1000, 750
GRID_STEP = 50
COLOR_MAP = [  # BGR
    (50, 50, 255), (100, 100, 255), (130, 150, 255), (0, 200, 255),
    (0, 255, 255), (0, 255, 150), (0, 255, 100), (0, 255, 50),
    (0, 255,   0), (100, 255,   0), (180, 255,   0), (255, 255,   0),
    (255, 200,  0), (255, 150,   0), (255,  80,   0), (255, 255, 255)
]

class FlameTracker(Node):
    def __init__(self):
        super().__init__('flame_tracker_visual')

        # ---- 參數（可用 ros2 param set 調）----
        self.target_s      = self.declare_parameter('target_strength', 2).get_parameter_value().integer_value
        self.forward_spd   = self.declare_parameter('forward_speed', 0.18).get_parameter_value().double_value
        self.backward_spd  = self.declare_parameter('backward_speed', 0.10).get_parameter_value().double_value
        self.angular_spd   = self.declare_parameter('angular_speed', 0.35).get_parameter_value().double_value

        # ---- 串列埠 ----
        self.ser = self._find_serial_port(baudrate=115200)
        if not self.ser:
            self.get_logger().fatal("❌ 找不到可用的 Arduino 串列埠")
            raise SystemExit

        # ---- ROS 通訊 ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/manual_control', self._toggle_manual, 10)

        # ---- 狀態 ----
        self.manual_control_active = False
        self.fire_found = False            # 是否曾偵測到火源

        # ---- UI ----
        cv2.namedWindow("🔥 Flame Tracker", cv2.WINDOW_NORMAL)
        self.create_timer(0.05, self._loop)
        self.get_logger().info("🔥 Flame tracker 啟動（先旋轉搜尋 → 進距離控制）")

    # ------------------------------------------------------------
    # 內部工具
    # ------------------------------------------------------------
    def _find_serial_port(self, baudrate):
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        for port in ports:
            try:
                s = serial.Serial(port, baudrate, timeout=1)
                time.sleep(2)
                s.reset_input_buffer()
                test = s.readline().decode('utf-8', errors='ignore').strip()
                # 至少 10 個逗號 (4 點 * 3 資料 + 結尾)
                if test.count(',') >= 9:
                    self.get_logger().info(f"✅ 成功連接到 {port}")
                    return s
                s.close()
            except Exception:
                continue
        return None

    def _toggle_manual(self, msg: Bool):
        self.manual_control_active = msg.data
        # 進入手動立即停車
        if msg.data:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("🛑 手動模式啟動")
        else:
            self.get_logger().info("✅ 自動模式啟動")

    # ------------------------------------------------------------
    # 主迴圈
    # ------------------------------------------------------------
    def _loop(self):
        raw = self.ser.readline()
        if not raw:
            return

        # --- 解析感測字串 ---
        try:
            parts = list(map(int, raw.decode(errors='ignore').strip().split(',')))
        except Exception:
            return
        if len(parts) % 3 != 0:
            return

        coords, min_s = [], 16  # min_s 預設比 15 再大
        for i in range(0, len(parts), 3):
            ix, iy, s = parts[i:i+3]

            # 1023,1023,15 代表未偵測；或 s==15 視為最遠
            if ix == 1023 or iy == 1023 or s == 15:
                continue

            coords.append((ix, iy, s))
            if s < min_s:
                min_s = s

        # --- 畫格線 + 點位 ---
        frame = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        for x in range(0, WIDTH, GRID_STEP):
            cv2.line(frame, (x, 0), (x, HEIGHT), (0, 255, 0), 1)
        for y in range(0, HEIGHT, GRID_STEP):
            cv2.line(frame, (0, y), (WIDTH, y), (0, 255, 0), 1)

        for ix, iy, s in coords:
            x = int(iy * WIDTH / 1024)
            y = int(ix * HEIGHT / 1024)
            cv2.circle(frame, (x, y), 10, COLOR_MAP[min(s, 15)], -1)

        frame = cv2.flip(frame, -1)  # 上下左右翻
        for ix, iy, s in coords:
            x = WIDTH  - int(iy * WIDTH  / 1024)
            y = HEIGHT - int(ix * HEIGHT / 1024)
            cv2.putText(frame, f"{x},{y}", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_MAP[min(s, 15)], 2)

        # --- 上方透明背景區 ---
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (420, 160), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.4, frame, 0.6, 0, frame)

        # --- 控制邏輯 ---
        twist = Twist()
        mode_str   = "Manual Mode" if self.manual_control_active else "Auto Mode"
        status_str = "Manual"

        if self.manual_control_active:
            self.fire_found = False

        elif not self.fire_found:
            # 尚未偵測火源 → 原地旋轉
            twist.angular.z = self.angular_spd
            status_str = "Searching (Rotating)"
            if coords:
                self.fire_found = True

        else:
            # 已偵測過火源 → 距離控制
            if coords:
                if min_s > self.target_s:
                    twist.linear.x = self.forward_spd
                    status_str = f"🔥 S={min_s} > {self.target_s} → Forward"
                elif min_s < self.target_s:
                    twist.linear.x = -self.backward_spd
                    status_str = f"🔥 S={min_s} < {self.target_s} → Backward"
                else:
                    status_str = f"🔥 S={min_s} == {self.target_s} → Hold"
            else:
                status_str = "Lost fire source"
                twist = Twist()

        # --- 資訊文字 ---
        cv2.putText(frame, f"🔥 Flame Tracker ({mode_str})", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
        cv2.putText(frame, f"Min Strength: {min_s if min_s < 16 else 'N/A'}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        cv2.putText(frame, f"Fire Detected: {self.fire_found}", (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
        cv2.putText(frame, f"Status: {status_str}", (10, 125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 100), 2)

        if self.fire_found and min_s == self.target_s:
            cv2.putText(frame, "🔥 S == 2，準備發射", (10, 155),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("🔥 Flame Tracker", frame)
        cv2.waitKey(1)

        # --- 發佈指令 (自動模式才動) ---
        if not self.manual_control_active:
            self.cmd_pub.publish(twist)

# ------------------------------------------------------------
# 進入點
# ------------------------------------------------------------
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
