#!/usr/bin/env python3
# cam_stream_node.py – 2025-06-01 rev-B
#
# ▸ ROS2：/image_raw + /thermal_data
# ▸ Flask：/raw  /thermal   →   thermal 畫面左上角疊最高溫

import rclpy, cv2, numpy as np, serial, time, threading, json
from rclpy.node        import Node
from sensor_msgs.msg   import Image, String    # ★ 新增 String
from cv_bridge         import CvBridge
from flask             import Flask, Response, render_template_string

# =================== 硬體參數 ===================
CAM_DEVICE      = "/dev/video0"
THERMAL_SERIAL  = "/dev/ttyACM0"
BAUD_RATE       = 115200
FRAME_W, FRAME_H = 1280, 720
TEMP_THRESHOLD   = 40.0
FIRE_HUE_RANGE   = [(0, 50)]
BRIGHTNESS_MIN   = 5
# ===============================================

# ---------- 共用影像 ----------
class SharedState:
    def __init__(self, cap):
        self.cap  = cap
        self.lock = threading.Lock()
        self.frame = None
        self.running = True
    def capture_loop(self):
        while self.running:
            ok, frm = self.cap.read()
            if ok:
                with self.lock:
                    self.frame = frm
            else:
                time.sleep(0.01)
    def get_frame(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

# ---------- 熱像 & 火焰 ----------
def parse_temperatures(line: bytes):
    try:
        txt = line.decode(errors="ignore").strip()
        if txt.count(",") != 63:
            return None
        return np.fromstring(txt, sep=",", dtype=np.float32).reshape(8, 8)
    except Exception:
        return None
def get_color_fire_mask(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in FIRE_HUE_RANGE:
        mask |= cv2.inRange(hsv, (lo, 100, 100), (hi, 255, 255))
    return mask

# ---------- ROS2 節點 ----------
class CamPublisher(Node):
    def __init__(self, shared: SharedState, fps: int = 30):
        super().__init__('camera_publisher')
        self.declare_parameter('fps', fps)
        self.shared  = shared
        self.bridge  = CvBridge()
        self.pub_img = self.create_publisher(Image, '/image_raw',   10)
        self.timer   = self.create_timer(1.0 / fps, self.timer_cb)
        self.get_logger().info(f'Camera publisher 啟動 (/image_raw @{fps} Hz)')
    def timer_cb(self):
        frame = self.shared.get_frame()
        if frame is None or frame.mean() < BRIGHTNESS_MIN:
            return
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
    def destroy_node(self):
        self.shared.running = False
        super().destroy_node()

# ---------- Flask ----------
def create_flask_app(shared: SharedState, thermal_pub):   # ★ thermal_pub 參數
    app = Flask(__name__)
    ser = serial.Serial(THERMAL_SERIAL, BAUD_RATE, timeout=1)
    latest_max = {'val': None}

    def stream_raw():
        while True:
            frm = shared.get_frame()
            if frm is None:
                continue
            _, jpg = cv2.imencode('.jpg', frm)
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n'

    def stream_thermal():
        last_ok = time.time()
        while True:
            frm = shared.get_frame()
            if frm is None:
                continue
            line = ser.readline()
            thermal = parse_temperatures(line)
            if thermal is None:
                if time.time() - last_ok > 2:
                    ser.reset_input_buffer(); last_ok = time.time()
                continue
            last_ok = time.time()

            max_t = float(np.max(thermal))
            latest_max['val'] = max_t

            # ★ 發布 ROS /thermal_data
            thermal_pub.publish(String(data=json.dumps({'max_temp': max_t})))

            # --- 熱像疊圖 ---
            heat_big  = cv2.resize(thermal, (FRAME_W, FRAME_H),
                                   interpolation=cv2.INTER_CUBIC)
            heat_norm = cv2.normalize(heat_big, None, 0, 255, cv2.NORM_MINMAX)
            heat_clr  = cv2.applyColorMap(heat_norm.astype(np.uint8),
                                          cv2.COLORMAP_JET)
            mask_temp  = (heat_big > TEMP_THRESHOLD).astype(np.uint8) * 255
            mask_color = get_color_fire_mask(frm)
            mask_fire  = cv2.bitwise_and(mask_temp, mask_color)
            result = cv2.addWeighted(frm, 0.7, heat_clr, 0.3, 0)

            # ★ 左上角即時溫度
            cv2.putText(result, f'{max_t:.1f} °C', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)

            # 火焰框
            cnts, _ = cv2.findContours(mask_fire, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                if cv2.contourArea(c) < 500:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(result, (x,y), (x+w,y+h), (0,0,255), 2)
                cv2.putText(result, 'Fire', (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

            _, jpg = cv2.imencode('.jpg', result)
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n'

    @app.route('/')
    def index():
        t = latest_max['val']
        show = f'{t:.1f} °C' if t is not None else 'N/A'
        html = """
        <h2>🔥 Fire Detection Stream</h2>
        <p>熱成像目前偵測到的最高溫：<b>{{temp}}</b></p>
        <ul>
          <li><a href="/raw">Raw Camera Feed</a></li>
          <li><a href="/thermal">Thermal Fire Detection</a></li>
        </ul>
        """
        return render_template_string(html, temp=show)

    @app.route('/raw')
    def raw_feed():
        return Response(stream_raw(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/thermal')
    def thermal_feed():
        return Response(stream_thermal(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    return app

# ---------- main ----------
def main():
    cap = cv2.VideoCapture(CAM_DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    if not cap.isOpened():
        raise RuntimeError(f'無法開啟相機 {CAM_DEVICE}')

    shared = SharedState(cap)
    threading.Thread(target=shared.capture_loop, daemon=True).start()

    rclpy.init()
    ros_node = CamPublisher(shared, fps=30)
    thermal_pub = ros_node.create_publisher(String, '/thermal_data', 10)   # ★ 新增 publisher

    app = create_flask_app(shared, thermal_pub)                            # ★ 傳入 publisher
    threading.Thread(target=app.run,
                     kwargs={'host':'0.0.0.0','port':5000,'debug':False,
                             'use_reloader':False},
                     daemon=True).start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        shared.running = False
        cap.release()

if __name__ == '__main__':
    main()
