#!/usr/bin/env python3
# flame_tracker.py – 2025-05-30 (stop-at-S2, all text in English)

import rclpy, cv2, serial, time, glob, numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# ---------- GUI parameters ----------
WIDTH, HEIGHT = 1000, 750
GRID_STEP      = 50
COLOR_MAP = [  # 0-15 strength → color (BGR)
    (50,50,255),(100,100,255),(130,150,255),(0,200,255),
    (0,255,255),(0,255,150),(0,255,100),(0,255,50),
    (0,255,0),(100,255,0),(180,255,0),(255,255,0),
    (255,200,0),(255,150,0),(255,80,0),(255,255,255)
]

class FlameTracker(Node):
    def __init__(self):
        super().__init__('flame_tracker_visual')

        # -------- ROS parameters --------
        self.shoot_thr    = self.declare_parameter('shoot_threshold', 2).get_parameter_value().integer_value
        self.forward_spd  = self.declare_parameter('forward_speed', 0.18).get_parameter_value().double_value
        self.backward_spd = self.declare_parameter('backward_speed', 0.10).get_parameter_value().double_value
        self.angular_spd  = self.declare_parameter('angular_speed', 0.35).get_parameter_value().double_value

        # -------- auto-detect /dev/ttyACM0 --------
        self.ser = self._auto_serial(baud=115200)
        if not self.ser:
            self.get_logger().fatal("Arduino serial port not found")
            raise SystemExit

        # -------- ROS pub/sub --------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/manual_control', self._toggle_manual, 10)
        self.manual_mode   = False
        self.fire_detected = False

        # -------- OpenCV window --------
        cv2.namedWindow("Flame Tracker", cv2.WINDOW_NORMAL)

        # 20 Hz loop
        self.create_timer(0.05, self._loop)
        self.get_logger().info("Flame tracker started (Stop at S=2)")

    # ---------- auto-detect serial ----------
    def _auto_serial(self, baud:int):
        for p in glob.glob('/dev/ttyACM0'):
            try:
                s = serial.Serial(p, baud, timeout=1)
                time.sleep(2)
                if s.readline().decode('utf-8', 'ignore').count(',') >= 9:
                    self.get_logger().info(f"Connected to {p}")
                    return s
                s.close()
            except: 
                pass
        return None

    # ---------- manual / auto toggle ----------
    def _toggle_manual(self, msg:Bool):
        self.manual_mode = msg.data
        self.cmd_pub.publish(Twist())       # instant stop
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

        # ---- parse coordinates & strength ----
        coords, max_s = [], -1
        for i in range(0, len(parts), 3):
            ix, iy, s = parts[i:i+3]
            if ix == 1023 or iy == 1023:   # invalid pixel
                continue
            coords.append((ix, iy, s))
            max_s = max(max_s, s)

        # ---------- visualization ----------
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

        # translucent header background
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (440, 170), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.4, frame, 0.6, 0, frame)

        # ---------- control logic ----------
        twist      = Twist()
        mode_str   = "Manual" if self.manual_mode else "Auto"
        status_str = "Manual"

        if self.manual_mode:
            self.fire_detected = False

        # no fire → rotate
        elif (max_s == 15) or (not coords):
            self.fire_detected = False
            twist.angular.z = self.angular_spd
            status_str      = "Searching (rotate)"

        else:
            self.fire_detected = True
            # choose pixel with minimum s (closest)
            target   = min(coords, key=lambda c: c[2])
            cx_pixel = WIDTH - int(target[1] * WIDTH / 1024)    # after flip
            img_mid  = WIDTH / 2
            tol      = WIDTH * 0.10
            left, right = img_mid - tol, img_mid + tol

            # angular velocity: align center
            if left < cx_pixel < right:
                twist.angular.z = 0.0
            elif cx_pixel < left:
                twist.angular.z =  self.angular_spd
            else:
                twist.angular.z = -self.angular_spd

            # linear velocity: forward / backward / hold
            if max_s < self.shoot_thr:           # too far → forward
                twist.linear.x =  self.forward_spd
                status_str     = f"S={max_s} < 2 → Forward"
            elif max_s > self.shoot_thr:         # too close → back
                twist.linear.x = -self.backward_spd
                status_str     = f"S={max_s} > 2 → Backward"
            else:                                # S == 2 → stop
                twist.linear.x  = 0.0
                twist.angular.z = 0.0
                status_str      = "S = 2 → Hold"

        # ---------- on-screen text ----------
        cv2.putText(frame, f"Flame Tracker ({mode_str})", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
        cv2.putText(frame, f"Max Strength: {max_s}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        cv2.putText(frame, f"Fire Detected: {self.fire_detected}", (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
        cv2.putText(frame, f"Status: {status_str}", (10, 125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 100), 2)

        if self.fire_detected and max_s == self.shoot_thr:
            cv2.putText(frame, "S = 2, hold & ready to fire", (10, 155),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("Flame Tracker", frame)
        cv2.waitKey(1)

        # ---------- publish cmd_vel ----------
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
