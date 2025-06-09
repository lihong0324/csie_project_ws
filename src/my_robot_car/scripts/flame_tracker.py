#!/usr/bin/env python3
# flame_tracker_fast.py – 2025-06-06  (fixed /dev/ttyACM0, low-latency)
#
# ─────────────────────────────────────────────────────────────────────────
# ★ 主要優化 (承前版) ─ 讀取執行緒、NumPy 解析、100 Hz 無 GUI、移動平均
# ★ 本版增修 ─ 固定序列埠 /dev/ttyACM0 + timeout 0.01 s
# ─────────────────────────────────────────────────────────────────────────

import rclpy, cv2, serial, time, numpy as np, requests, argparse, sys
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

LAUNCH_URL   = 'http://localhost:5001/launch'
SERIAL_PORT  = '/dev/ttyACM0'          # ← 固定序列埠
SERIAL_BAUD  = 115200                  # ↗ 建議提高至 230 400 以上
SERIAL_TOUT  = 0.01                    # 低延遲 (blocking 10 ms)

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
            if not raw:
                continue
            try:
                arr = np.fromstring(raw, dtype=np.int16, sep=',')
            except ValueError:
                continue
            if arr.size % 3:
                continue

            triplets = arr.reshape(-1, 3)
            valid    = triplets[(triplets[:,0] != 1023) & (triplets[:,1] != 1023)]
            if valid.size == 0:
                self.latest = ([], -1, False)
                continue

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

        # ───── Serial 連線 (固定埠) ───
        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=SERIAL_TOUT)
            time.sleep(1.2)                 # USB CDC 初始化
            self.ser.reset_input_buffer()   # 清空緩衝，確保低延遲
        except serial.SerialException as e:
            self.get_logger().fatal(f"Serial open failed: {e}")
            raise SystemExit

        # ─── 非阻塞讀取 ───────
        self.exit_evt = Event()
        self.reader   = SerialReader(self.ser, self.exit_evt)
        self.reader.start()

        # ───── ROS Pub/Sub ─────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/manual_control', self._toggle_manual, 10)
        self.manual_mode = False

        # ───── 狀態機 ───────────
        self.mode            = "SEARCH"
        self.confirm_timer   = None
        self.center_timer    = None
        self.lost_timer      = None
        self.fired           = False

        self.center_buf = deque(maxlen=10)  # 0.2 s 移動平均

        # ───── GUI ──────────────
        self.gui           = gui
        self.draw_every_n  = max(1, draw_every_n)
        self.frame_counter = 0
        if self.gui:
            cv2.namedWindow("Flame Tracker", cv2.WINDOW_NORMAL)

        base_hz = 50 if gui else 100
        self.create_timer(1.0/base_hz, self._loop)
        self.get_logger().info(f"Flame tracker started – GUI={gui} @ {base_hz} Hz / {SERIAL_PORT}")

    # ─────────── 其餘成員函式（完全照舊，無改動） ───────────
    # _toggle_manual, _loop, _reset, _angle_cmd, _send_launch, _draw_frame
    # 〈為節省篇幅，此處略去；若需參考請保留原程式碼段〉
    # …………………………………………………………………………………………………………………

# ╭─────────────────────────╮
# |        main()           |
# ╰─────────────────────────╯
def main(argv=None):
    argv = argv or sys.argv[1:]
    ap   = argparse.ArgumentParser(description="Fast flame tracker (fixed serial)")
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
