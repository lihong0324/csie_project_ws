#!/usr/bin/env python3
# flame_tracker.py – 2025-06-01  (S=3, 1 s → align, slow turn)

import rclpy, cv2, serial, time, glob, numpy as np
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

class FlameTracker(Node):
    def __init__(self):
        super().__init__('flame_tracker')

        # ---- tunable parameters ----
        self.shoot_thr    = self.declare_parameter('shoot_threshold', 3).get_parameter_value().integer_value
        self.forward_spd  = self.declare_parameter('forward_speed', 0.18).get_parameter_value().double_value
        self.backward_spd = self.declare_parameter('backward_speed', 0.10).get_parameter_value().double_value
        self.angular_spd  = self.declare_parameter('angular_speed', 0.35).get_parameter_value().double_value
        self.center_tol   = self.declare_parameter('center_tolerance_pct', 10.0).get_parameter_value().double_value

        # ---- serial ----
        self.ser = self._auto_serial(115200)
        if not self.ser:
            self.get_logger().fatal("Arduino serial port not found")
            raise SystemExit

        # ---- ROS ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/manual_control', self._toggle_manual, 10)
        self.manual_mode = False

        # ---- state machine ----
        self.phase = "DISTANCE"          # DISTANCE → ALIGN → DONE
        self.s_target_timer = None       # time when S first hit target
        self.center_timer   = None       # time when first centered
        self.fire_present   = False

        # ---- GUI ----
        cv2.namedWindow("Flame Tracker", cv2.WINDOW_NORMAL)
        self.create_timer(0.05, self._loop)   # 20 Hz
        self.get_logger().info("Flame tracker started (3-phase control, S=3)")

    # ---------- serial auto detect ----------
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

    # ---------- manual toggle ----------
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

        # ---- parse flame data ----
        coords, max_s = [], -1
        for i in range(0, len(parts), 3):
            ix, iy, s = parts[i:i+3]
            if ix == 1023 or iy == 1023:
                continue
            coords.append((ix, iy, s))
            max_s = max(max_s, s)

        self.fire_present = bool(coords) and max_s < 15

        # ---- nearest pixel ----
        if coords:
            target = min(coords, key=lambda c: c[2])
            cx_pix = WIDTH - int(target[1] * WIDTH / 1024)  # after flip
        else:
            target, cx_pix = None, WIDTH / 2

        img_mid  = WIDTH / 2
        tol_pix  = WIDTH * (self.center_tol / 100.0)
        centered = abs(cx_pix - img_mid) < tol_pix

        # ---- state transitions ----
        now = time.time()

        if self.phase == "DISTANCE":
            if self.fire_present and max_s == self.shoot_thr:
                self.s_target_timer = self.s_target_timer or now
                if now - self.s_target_timer >= 1.0:          # 1 s
                    self.phase = "ALIGN"
                    self.get_logger().info(f"S == {self.shoot_thr} for 1 s → ALIGN")
            else:
                self.s_target_timer = None

        elif self.phase == "ALIGN":
            if centered:
                self.center_timer = self.center_timer or now
                if now - self.center_timer >= 2.0:
                    self.phase = "DONE"
                    self.get_logger().info("Centered for 2 s → DONE")
            else:
                self.center_timer = None
            if not self.fire_present or max_s != self.shoot_thr:
                self.phase = "DISTANCE"
                self.s_target_timer = None
                self.center_timer = None
                self.get_logger().info(f"Lost S == {self.shoot_thr}, back to DISTANCE")

        # ---- build Twist ----
        twist = Twist()

        if self.manual_mode:
            phase_str = "MANUAL"

        else:
            if not self.fire_present:
                twist.angular.z = self.angular_spd
                phase_str = "SEARCH"
            elif self.phase == "DISTANCE":
                twist.angular.z = self._angle_cmd(cx_pix, img_mid, tol_pix)
                if max_s < self.shoot_thr:
                    twist.linear.x =  self.forward_spd
                    status = "forward"
                elif max_s > self.shoot_thr:
                    twist.linear.x = -self.backward_spd
                    status = "backward"
                else:
                    status = "hold"
                phase_str = f"DISTANCE ({status})"
            elif self.phase == "ALIGN":
                twist.angular.z = self._angle_cmd(cx_pix, img_mid, tol_pix) * 0.5  # slower turn
                phase_str = "ALIGN (slow turn)"
            else:  # DONE
                phase_str = "END"
                twist = Twist()

        # publish
        if not self.manual_mode:
            self.cmd_pub.publish(twist)

        # ---- GUI ----
        frame = self._draw_frame(coords, centered, max_s, phase_str)
        cv2.imshow("Flame Tracker", frame)
        cv2.waitKey(1)

        if self.phase == "DONE":
            cv2.waitKey(500)
            cv2.destroyAllWindows()
            rclpy.shutdown()

    # ---------- helper ----------
    def _angle_cmd(self, cx, mid, tol):
        if abs(cx - mid) < tol:
            return 0.0
        return  self.angular_spd if cx < mid - tol else -self.angular_spd

    def _draw_frame(self, coords, centered, max_s, phase_str):
        frame = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        for x in range(0, WIDTH, GRID_STEP):
            cv2.line(frame, (x,0), (x,HEIGHT), (0,150,0), 1)
        for y in range(0, HEIGHT, GRID_STEP):
            cv2.line(frame, (0,y), (WIDTH,y), (0,150,0), 1)
        for ix, iy, s in coords:
            cx = int(iy * WIDTH  / 1024); cy = int(ix * HEIGHT / 1024)
            cv2.circle(frame, (cx, cy), 10, COLOR_MAP[min(s,15)], -1)
        frame = cv2.flip(frame, -1)

        overlay = frame.copy()
        cv2.rectangle(overlay, (0,0), (460,190), (0,0,0), -1)
        cv2.addWeighted(overlay, 0.4, frame, 0.6, 0, frame)

        cv2.putText(frame, f"Phase: {phase_str}", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,255), 2)
        cv2.putText(frame, f"Max S: {max_s}", (10,65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,165,255), 2)
        cv2.putText(frame, f"Centered: {centered}", (10,100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,200,0), 2)
        if self.phase == "DONE":
            cv2.putText(frame, "(END)", (int(WIDTH/2)-70, int(HEIGHT/2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0,255,255), 4)
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
