#!/usr/bin/env python3
# flame_tracker_fast.py – 2025-06-03
#
# ★ 主要優化 (對照舊版) ───────────────────────────────────────────────
# 1. 以獨立執行緒 SerialReader Thread ⮕ 不中斷 ROS2 timer 迴圈
# 2. serial 解析改用 numpy.fromstring(…, sep=',')  → 文字轉整數快 3~4 倍
# 3. loop 週期 0.02 s (50 Hz) + 無 GUI 時自動降至 0.01 s (100 Hz)
# 4. GUI 開關 (--no-gui) ；且只在需要時才完整重繪 (draw_every_n)
# 5. 射擊置中判定改用移動平均 (deque)  → 降噪、不會錯過瞬間置中
# 6. 其他：gadgets 少用清單推疊、Math in-place，減少小物件配置
#
# 備註：Arduino 端若能把 Baud 提到 230 400 bps 以上，延遲還會再降。
# ───────────────────────────────────────────────────────────────────

import rclpy, cv2, serial, time, glob, numpy as np, requests, argparse, sys
from rclpy.node          import Node
from geometry_msgs.msg   import Twist
from std_msgs.msg        import Bool
from collections         import deque
from threading           import Thread, Event

WIDTH, HEIGHT = 1000, 750
GRID_STEP      = 50
GUI_BG_COLOR   = (0, 150, 0)
COLOR_MAP = [
    (50,50,255),(100,100,255),(130,150,255),(0,200,255),
    (0,255,255),(0,255,150),(0,255,100),(0,255,50),
    (0,255,0),(100,255,0),(180,255,0),(255,255,0),
    (255,200,0),(255,150,0),(255,80,0),(255,255,255)
]

LAUNCH_URL = 'http://localhost:5001/launch'

# ╭─────────────────────────╮
# |  1. 讀取執行緒 (非阻塞)  |
# ╰─────────────────────────╯
class SerialReader(Thread):
    def __init__(self, ser, exit_evt:Event):
        super().__init__(daemon=True)
        self.ser      = ser
        self.exit_evt = exit_evt
        self.latest   = ([], -1, False)  # coords, max_s, flame_seen

    def run(self):
        while not self.exit_evt.is_set():
            raw = self.ser.readline()
            if not raw:         # 逾時無資料
                continue
            try:
                arr = np.fromstring(raw, dtype=np.int16, sep=',')
            except ValueError:
                continue
            if arr.size % 3:     # 格式錯誤
                continue

            # 把 3 元組切出 (ix, iy, s)
            triplets = arr.reshape(-1, 3)
            # 去掉不存在 (1023)
            valid    = triplets[(triplets[:,0] != 1023) & (triplets[:,1] != 1023)]
            if valid.size == 0:
                self.latest = ([], -1, False)
                continue

            # NumPy 轉 Python list 只在需要畫面時才做，其他用 ndarray
            max_s      = int(valid[:,2].min())
            flame_seen = max_s < 15
            self.latest = (valid, max_s, flame_seen)

# ╭─────────────────────────╮
# |  2. ROS2 節點            |
# ╰─────────────────────────╯
class FlameTracker(Node):
    def __init__(self, gui:bool=True, draw_every_n:int=3):
        super().__init__('flame_tracker')

        # ───── 參數 ───────────────────
        self.forward_spd    = self.declare_parameter('forward_speed', 0.18).get_parameter_value().double_value
        self.angular_spd    = self.declare_parameter('angular_speed', 0.40).get_parameter_value().double_value
        self.center_tol_pct = self.declare_parameter('center_tolerance_pct', 9.0).get_parameter_value().double_value
        self.confirm_sec    = self.declare_parameter('confirm_sec', 0.3).get_parameter_value().double_value
        self.lost_sec       = self.declare_parameter('lost_sec', 0.5).get_parameter_value().double_value
        self.center_fire_sec= 1.0
        self.target_s       = 2

        # ───── Serial 連線 ─────────────
        self.ser = self._auto_serial(230400)  # ↗ 提高 Baud
        if not self.ser:
            self.get_logger().fatal("Arduino serial port not found")
            raise SystemExit

        # ─── 非阻塞讀取 (Thread) ───────
        self.exit_evt = Event()
        self.reader   = SerialReader(self.ser, self.exit_evt)
        self.reader.start()

        # ───── ROS Pub/Sub ─────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/manual_control', self._toggle_manual, 10)
        self.manual_mode = False

        # ───── 狀態機 ────────────────
        self.mode            = "SEARCH"
        self.confirm_timer   = None
        self.center_timer    = None
        self.lost_timer      = None
        self.fired           = False

        # 移動平均佇列 (追蹤中心判斷) -----------------
        self.center_buf = deque(maxlen=10)  # 10 個樣本 ≈ 0.2 s

        # ───── GUI ────────────────────
        self.gui           = gui
        self.draw_every_n  = max(1, draw_every_n)
        self.frame_counter = 0
        if self.gui:
            cv2.namedWindow("Flame Tracker", cv2.WINDOW_NORMAL)

        # ───── 主要迴圈 ───────────────
        base_hz = 50 if gui else 100      # GUI 時 50 Hz；無 GUI 時 100 Hz
        self.create_timer(1.0/base_hz, self._loop)
        self.get_logger().info(f"Flame tracker started – GUI={gui} @ {base_hz} Hz")

    # ──────────────────────────────────
    def _auto_serial(self, baud:int):
        for p in glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'):
            try:
                s = serial.Serial(p, baud, timeout=0.05)
                time.sleep(1.5)
                s.reset_input_buffer()
                return s
            except Exception:
                continue
        return None

    def _toggle_manual(self, msg:Bool):
        self.manual_mode = msg.data
        self.cmd_pub.publish(Twist())   # 停車
        self.get_logger().info("Manual mode" if msg.data else "Auto mode")

    # ╭─────────────────────────╮
    # |    主要 Timer 迴圈       |
    # ╰─────────────────────────╯
    def _loop(self):
        coords, max_s, flame_seen = self.reader.latest
        now   = time.time()

        if coords:
            # 最近點 (s 最小) ＝ 火焰
            target  = coords[np.argmin(coords[:,2])]
            cx_pix  = WIDTH - int(target[1] * WIDTH / 1024)
        else:
            cx_pix  = WIDTH // 2

        mid       = WIDTH / 2
        tol_pix   = WIDTH * (self.center_tol_pct / 100.0)
        centered  = abs(cx_pix - mid) < tol_pix

        # ☆ 紀錄中心 buffer，降低搖晃
        self.center_buf.append(centered)
        centered_avg = sum(self.center_buf)/len(self.center_buf) > 0.7  # >70% 置中

        # ───── 狀態機 ───────────────
        if self.mode in ("SEARCH", "APPROACH"):
            if not flame_seen:
                self.mode = "SEARCH"
                self.confirm_timer = None
            else:
                if max_s == self.target_s:
                    self.confirm_timer = self.confirm_timer or now
                    if (now - self.confirm_timer) >= self.confirm_sec:
                        self.mode = "TRACK"
                        self.get_logger().info("Locked (S=2) → TRACK")
                else:
                    self.confirm_timer = None
                    self.mode = "APPROACH"

        elif self.mode == "TRACK":
            if not flame_seen:
                self.lost_timer = self.lost_timer or now
                if (now - self.lost_timer) >= self.lost_sec:
                    self.mode = "SEARCH"
                    self._reset()
                    self.get_logger().info("Flame lost → SEARCH")
            else:
                self.lost_timer = None
                if centered_avg:
                    self.center_timer = self.center_timer or now
                    if (now - self.center_timer) >= self.center_fire_sec and not self.fired:
                        self.mode = "FIRE"
                else:
                    self.center_timer = None

        elif self.mode == "FIRE":
            pass

        # ───── 運動指令 ──────────────
        twist = Twist()
        if self.manual_mode:
            mode_str = "MANUAL"
        else:
            if self.mode == "SEARCH":
                twist.angular.z = self.angular_spd
            elif self.mode == "APPROACH":
                twist.angular.z = self._angle_cmd(cx_pix, mid, tol_pix)
                twist.linear.x  = self.forward_spd
            elif self.mode == "TRACK":
                if abs(cx_pix - mid) > tol_pix:
                    twist.angular.z = self._angle_cmd(cx_pix, mid, tol_pix) * 0.5
            elif self.mode == "FIRE":
                pass
            mode_str = self.mode

        if not self.manual_mode:
            self.cmd_pub.publish(twist)

        # ───── 發射 ─────────────────
        if self.mode == "FIRE" and not self.fired:
            self._send_launch()
            self.fired = True

        # ───── GUI 畫面 ─────────────
        if self.gui:
            self.frame_counter += 1
            if self.frame_counter % self.draw_every_n == 0:
                frame = self._draw_frame(coords, mode_str, flame_seen,
                                         centered_avg, max_s, cx_pix, mid, tol_pix)
                cv2.imshow("Flame Tracker", frame)
                cv2.waitKey(1)

    # ╭─────────────────────────╮
    # |     工具函式             |
    # ╰─────────────────────────╯
    def _reset(self):
        self.confirm_timer = self.center_timer = self.lost_timer = None
        self.fired = False
        self.center_buf.clear()

    def _angle_cmd(self, cx, mid, tol):
        if abs(cx - mid) < tol:
            return 0.0
        return  self.angular_spd if cx < mid - tol else -self.angular_spd

    def _send_launch(self):
        try:
            r = requests.post(LAUNCH_URL, timeout=1.5)
            if r.ok:
                self.get_logger().info("🔥 Launch command sent!")
            else:
                self.get_logger().error(f"Launch failed: {r.text}")
        except requests.RequestException as e:
            self.get_logger().error(f"Launch HTTP error: {e}")

    def _draw_frame(self, coords, mode_str, flame_seen, centered, max_s, cx, mid, tol):
        frame = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        # Grid
        for x in range(0, WIDTH, GRID_STEP):
            cv2.line(frame, (x,0), (x,HEIGHT), GUI_BG_COLOR, 1)
        for y in range(0, HEIGHT, GRID_STEP):
            cv2.line(frame, (0,y), (WIDTH,y), GUI_BG_COLOR, 1)
        # Flame points
        for ix, iy, s in coords[:200]:    # 最多畫 200 點，減輕 GPU
            px = int(iy * WIDTH / 1024)
            py = int(ix * HEIGHT / 1024)
            cv2.circle(frame, (px, py), 10, COLOR_MAP[min(int(s),15)], -1)

        frame = cv2.flip(frame, -1)        # 鏡像

        overlay = frame.copy()
        cv2.rectangle(overlay, (0,0), (560,220), (0,0,0), -1)
        cv2.addWeighted(overlay, .4, frame, .6, 0, frame)

        cv2.putText(frame, f"Mode: {mode_str}", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, .75, (0,255,255), 2)
        cv2.putText(frame, f"Flame seen: {flame_seen}", (10,65),
                    cv2.FONT_HERSHEY_SIMPLEX, .7, (0,165,255), 2)
        cv2.putText(frame, f"Centered: {centered}", (10,95),
                    cv2.FONT_HERSHEY_SIMPLEX, .7, (255,200,0), 2)
        cv2.putText(frame, f"Max S: {max_s}", (10,125),
                    cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,100), 2)
        cv2.line(frame, (int(mid-tol),0), (int(mid-tol),HEIGHT),(255,255,255),1)
        cv2.line(frame, (int(mid+tol),0), (int(mid+tol),HEIGHT),(255,255,255),1)
        cv2.circle(frame, (int(cx), int(HEIGHT/2)), 8, (0,255,255), 2)
        if self.fired:
            cv2.putText(frame, "FIRE!", (int(WIDTH/2)-60, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,255), 3)
        return frame

# ╭─────────────────────────╮
# |        main()           |
# ╰─────────────────────────╯
def main(argv=None):
    argv = argv or sys.argv[1:]
    ap   = argparse.ArgumentParser(description="Fast flame tracker")
    ap.add_argument('--no-gui', action='store_true', help='run headless')
    ap.add_argument('--draw-every-n', type=int, default=3,
                    help='draw every Nth frame (GUI only)')
    args = ap.parse_args(argv)

    rclpy.init()
    node = FlameTracker(gui=not args.no_gui, draw_every_n=args.draw_every_n)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.exit_evt.set()
        node.reader.join(timeout=0.2)
        node.destroy_node()
        if node.gui and cv2.getWindowProperty("Flame Tracker", cv2.WND_PROP_VISIBLE) >= 1:
            cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
