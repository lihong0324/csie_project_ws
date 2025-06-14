#!/usr/bin/env python3
# flame_tracker.py – 2025-06-02
# SEARCH → APPROACH → TRACK → FIRE
# ─ 發射條件：TRACK 模式且置中 ≥2 s → POST /launch

import rclpy, cv2, serial, time, glob, numpy as np, requests
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# ---------- GUI ----------
WIDTH, HEIGHT = 1000, 750
GRID_STEP = 50
COLOR_MAP = [
    (50,50,255),(100,100,255),(130,150,255),(0,200,255),
    (0,255,255),(0,255,150),(0,255,100),(0,255,50),
    (0,255,0),(100,255,0),(180,255,0),(255,255,0),
    (255,200,0),(255,150,0),(255,80,0),(255,255,255)
]

# 發射 API
LAUNCH_URL = 'http://localhost:5001/launch'

class FlameTracker(Node):
    def __init__(self):
        super().__init__('flame_tracker')

        # -- parameters --
        self.target_s       = 2
        self.forward_spd    = self.declare_parameter('forward_speed', 0.15).get_parameter_value().double_value
        self.angular_spd    = self.declare_parameter('angular_speed', 0.35).get_parameter_value().double_value
        self.center_tol_pct = self.declare_parameter('center_tolerance_pct', 10.0).get_parameter_value().double_value
        self.confirm_sec    = self.declare_parameter('confirm_sec', 0.4).get_parameter_value().double_value
        self.lost_sec       = self.declare_parameter('lost_sec', 0.6).get_parameter_value().double_value
        self.center_fire_sec= 1.0   # 置中多久才發射

        # -- serial --
        self.ser = self._auto_serial(115200)
        if not self.ser:
            self.get_logger().fatal("Arduino serial port not found")
            raise SystemExit

        # -- ROS pub/sub --
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/manual_control', self._toggle_manual, 10)
        self.manual_mode = False

        # -- state machine --
        self.mode            = "SEARCH"   # SEARCH / APPROACH / TRACK / FIRE
        self.confirm_timer   = None
        self.center_timer    = None
        self.lost_timer      = None
        self.fired           = False

        # -- GUI --
        cv2.namedWindow("Flame Tracker", cv2.WINDOW_NORMAL)
        self.create_timer(0.05, self._loop)  # 20 Hz
        self.get_logger().info("Flame tracker started (auto-fire enabled)")

    # ---------- utilities ----------
    def _auto_serial(self, baud:int):
        for p in glob.glob('/dev/ttyACM0'):
            try:
                s = serial.Serial(p, baud, timeout=1)
                time.sleep(2)
                if s.readline().decode('utf-8','ignore').count(',') >= 9:
                    self.get_logger().info(f"Connected to {p}")
                    return s
                s.close()
            except: pass
        return None

    def _toggle_manual(self, msg:Bool):
        self.manual_mode = msg.data
        self.cmd_pub.publish(Twist())
        self.get_logger().info("Manual mode" if msg.data else "Auto mode")

    # ---------- main loop ----------
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

        # -- parse sensor packet --
        coords, max_s = [], -1
        for i in range(0, len(parts), 3):
            ix, iy, s = parts[i:i+3]
            if ix == 1023 or iy == 1023:
                continue
            coords.append((ix, iy, s))
            max_s = max(max_s, s)

        flame_seen = bool(coords) and max_s < 15
        now        = time.time()

        # -- nearest pixel --
        if coords:
            target = min(coords, key=lambda c: c[2])
            cx_pix = WIDTH - int(target[1] * WIDTH / 1024)
        else:
            cx_pix = WIDTH / 2

        mid     = WIDTH / 2
        tol_pix = WIDTH * (self.center_tol_pct / 100.0)
        centered= abs(cx_pix - mid) < tol_pix

        # ---------- state transitions ----------
        if self.mode in ("SEARCH", "APPROACH"):
            if not flame_seen:
                self.mode = "SEARCH"
                self.confirm_timer = None
            else:
                if max_s == self.target_s:
                    self.confirm_timer = self.confirm_timer or now
                    if (now - self.confirm_timer) >= self.confirm_sec:
                        self.mode = "TRACK"
                        self.get_logger().info("Locked at S=2 → TRACK")
                else:
                    self.confirm_timer = None
                    self.mode = "APPROACH"
        elif self.mode == "TRACK":
            if not flame_seen:
                self.lost_timer = self.lost_timer or now
                if (now - self.lost_timer) >= self.lost_sec:
                    self.mode = "SEARCH"
                    self.reset_timers()
                    self.get_logger().info("Flame lost → SEARCH")
            else:
                self.lost_timer = None
                # 檢查置中時間，決定是否進入 FIRE
                if centered:
                    self.center_timer = self.center_timer or now
                    if (now - self.center_timer) >= self.center_fire_sec and not self.fired:
                        self.mode = "FIRE"
                else:
                    self.center_timer = None
        elif self.mode == "FIRE":
            # 發射一次後停留在 FIRE
            pass

        # ---------- build Twist ----------
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
                # 全停
                pass
            mode_str = self.mode

        if not self.manual_mode:
            self.cmd_pub.publish(twist)

        # ---------- trigger firing ----------
        if self.mode == "FIRE" and not self.fired:
            self._send_launch()
            self.fired = True

        # ---------- GUI ----------
        frame = self._draw_frame(coords, mode_str, flame_seen, centered, max_s, cx_pix, mid, tol_pix)
        cv2.imshow("Flame Tracker", frame)
        cv2.waitKey(1)

    # ---------- helper: send launch ----------
    def _send_launch(self):
        try:
            r = requests.post(LAUNCH_URL, timeout=2)
            if r.ok:
                self.get_logger().info("🔥 Launch command sent!")
            else:
                self.get_logger().error(f"Launch failed: {r.text}")
        except requests.RequestException as e:
            self.get_logger().error(f"Launch HTTP error: {e}")

    def reset_timers(self):
        self.confirm_timer = None
        self.center_timer  = None
        self.lost_timer    = None
        self.fired         = False

    # ---------- helper: angular cmd ----------
    def _angle_cmd(self, cx, mid, tol):
        if abs(cx - mid) < tol:
            return 0.0
        return  self.angular_spd if cx < mid - tol else -self.angular_spd

    # ---------- helper: draw GUI ----------
    def _draw_frame(self, coords, mode_str, flame_seen, centered, max_s, cx, mid, tol):
        frame = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        for x in range(0, WIDTH, GRID_STEP):
            cv2.line(frame, (x,0), (x,HEIGHT), (0,150,0), 1)
        for y in range(0, HEIGHT, GRID_STEP):
            cv2.line(frame, (0,y), (WIDTH,y), (0,150,0), 1)
        for ix, iy, s in coords:
            px = int(iy * WIDTH / 1024); py = int(ix * HEIGHT / 1024)
            cv2.circle(frame, (px, py), 10, COLOR_MAP[min(s,15)], -1)
        frame = cv2.flip(frame, -1)

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

def main(args=None):
    rclpy.init(args=args)
    node = FlameTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if cv2.getWindowProperty("Flame Tracker", cv2.WND_PROP_VISIBLE) >= 1:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()