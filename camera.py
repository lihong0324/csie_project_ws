#!/usr/bin/env python3
# camera.py – 2025-06-09 rev-H
#
# ▸ USB 攝影機：/dev/video0  (低延遲 grab/retrieve, buffer=1)
# ▸ 熱像 Arduino：/dev/ttyACM0  (8×8 溫度陣列)
# ▸ Flask：/raw  /thermal
# ▸ ROS2：/image_raw  /thermal_data(String)  /flame_detection(Bool)

import rclpy, cv2, serial, time, threading, json, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
from flask import Flask, Response, render_template_string, redirect

# ======== 可調參數 ========
CAM_DEVICE      = "/dev/video0"   # Linux V4L2
THERMAL_SERIAL  = "/dev/ttyACM0"  # Arduino 序列埠
BAUD_RATE       = 115200
W, H            = 640, 360
TEMP_TH         = 35.0            # ℃
FIRE_HUE        = [(0, 50)]       # HSV 火焰色範圍
KERNEL          = np.ones((7, 7), np.uint8)
BRIGHT_ALPHA    = 1.30            # 畫面增亮
BRIGHT_BETA     = 35
# ==========================

# ---------- 相機 ----------
class SharedState:
    def __init__(self):
        self.cap = cv2.VideoCapture(CAM_DEVICE)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH , W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE , 1)
        if not self.cap.isOpened():
            raise RuntimeError(f'❌ 無法開啟相機 {CAM_DEVICE}')
        self.lock = threading.Lock()
        self.frame = None
        self.running = True
    def capture_loop(self):
        while self.running:
            _ = self.cap.grab()
            ok, frm = self.cap.retrieve()
            if ok:
                frm = cv2.convertScaleAbs(frm, alpha=BRIGHT_ALPHA, beta=BRIGHT_BETA)
                with self.lock:
                    self.frame = frm
            else:
                time.sleep(0.01)
    def get_frame(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

# ---------- 熱像 ----------
latest_thermal = np.zeros((8, 8), np.float32)
def parse_thermal(buf: bytes):
    try:
        txt = buf.decode(errors='ignore').strip()
        if txt.count(',') != 63:
            return None
        return np.array(list(map(float, txt.split(','))), dtype=np.float32).reshape(8, 8)
    except Exception:
        return None
def serial_loop():
    global latest_thermal
    try:
        ser = serial.Serial(THERMAL_SERIAL, BAUD_RATE, timeout=0.1)
        print(f'[Serial] 連接 {THERMAL_SERIAL}')
    except Exception as e:
        print(f'[Serial] 無法連接：{e}')
        return
    while True:
        line = ser.readline()
        t = parse_thermal(line)
        if t is not None:
            latest_thermal = t
threading.Thread(target=serial_loop, daemon=True).start()

# ---------- 顏色遮罩 ----------
def fire_color_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in FIRE_HUE:
        m |= cv2.inRange(hsv, (lo, 80, 80), (hi, 255, 255))
    return m

# ---------- ROS2 ----------
class CamPublisher(Node):
    def __init__(self, shared, fps=30):
        super().__init__('camera_publisher')
        self.shared = shared
        self.bridge = CvBridge()
        self.pub_img  = self.create_publisher(Image, '/image_raw', 10)
        self.pub_tmp  = self.create_publisher(String, '/thermal_data', 10)
        self.pub_fire = self.create_publisher(Bool, '/flame_detection', 10)
        self.create_timer(1.0 / fps, self.timer_cb)
    def timer_cb(self):
        f = self.shared.get_frame()
        if f is None:
            return
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(f, 'bgr8'))
        self.pub_tmp.publish(String(data=json.dumps(
            {'max_temp': float(np.max(latest_thermal))})))
    def publish_fire(self, is_fire: bool):
        self.pub_fire.publish(Bool(data=is_fire))
    def destroy_node(self):
        self.shared.running = False
        self.shared.cap.release()
        super().destroy_node()

# ---------- Flask ----------
def create_flask_app(shared: SharedState, ros_node: CamPublisher):
    app = Flask(__name__)

    # 原始影像
    def gen_raw():
        while True:
            f = shared.get_frame()
            if f is None:
                time.sleep(0.01); continue
            _, jpg = cv2.imencode('.jpg', f)
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n'

    # 熱像疊圖
    def gen_thermal():
        while True:
            f = shared.get_frame()
            if f is None:
                time.sleep(0.01); continue
            thermal = latest_thermal
            heat = cv2.resize(thermal, (W, H), cv2.INTER_CUBIC)
            mask_hot = (heat > TEMP_TH).astype(np.uint8) * 255
            mask_col = fire_color_mask(f)
            mask_and = cv2.bitwise_and(mask_hot, mask_col)
            mask_and = cv2.dilate(mask_and, KERNEL, iterations=1)
            is_fire = cv2.countNonZero(mask_and) > 200
            ros_node.publish_fire(is_fire)

            heat_vis = cv2.applyColorMap(
                cv2.normalize(heat, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8),
                cv2.COLORMAP_JET)
            out = cv2.addWeighted(f, 0.7, heat_vis, 0.3, 0)
            for c in cv2.findContours(mask_and, cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)[0]:
                if cv2.contourArea(c) > 800: continue
                x, y, w_, h_ = cv2.boundingRect(c)
                cv2.rectangle(out, (x, y), (x + w_, y + h_), (0, 0, 255), 2)
            _, jpg = cv2.imencode('.jpg', out)
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n'

    # Routes
    @app.route('/')
    def index():
        # 預設直接導向熱像疊圖
        return redirect('/thermal')

    @app.route('/raw')
    def raw():
        return Response(gen_raw(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/thermal')
    def thermal():
        return Response(gen_thermal(), mimetype='multipart/x-mixed-replace; boundary=frame')

    return app

# ---------- main ----------
def main():
    shared = SharedState()
    threading.Thread(target=shared.capture_loop, daemon=True).start()

    rclpy.init()
    ros_node = CamPublisher(shared)

    flask_app = create_flask_app(shared, ros_node)
    threading.Thread(target=flask_app.run,
                     kwargs={'host': '0.0.0.0', 'port': 5000,
                             'debug': False, 'use_reloader': False},
                     daemon=True).start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
