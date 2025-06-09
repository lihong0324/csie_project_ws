#!/usr/bin/env python3
# cam_stream_node.py – 2025-06-06 rev-F
#
# 低延遲 + 火焰判斷強化 + /flame_detection Bool 佈告

import rclpy, cv2, numpy as np, serial, time, threading, json
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool          # ★ 新增 Bool
from cv_bridge import CvBridge
from flask import Flask, Response, render_template_string

# =================== 硬體參數 ===================
CAM_DEVICE       = "/dev/video0"
THERMAL_SERIAL   = "/dev/ttyACM0"
BAUD_RATE        = 115200
FRAME_W, FRAME_H = 640, 360
TEMP_THRESHOLD   = 35.0
FIRE_HUE_RANGE   = [(0, 50)]
BRIGHTNESS_MIN   = 5
BRIGHT_ALPHA     = 1.30
BRIGHT_BETA      = 35
KERNEL_DILATE    = np.ones((7, 7), np.uint8)
# ===============================================

class SharedState:
    def __init__(self, cap):
        self.cap, self.lock, self.frame, self.running = cap, threading.Lock(), None, True
    def capture_loop(self):
        while self.running:
            ok, frm = self.cap.read()
            if ok:
                frm = cv2.convertScaleAbs(frm, alpha=BRIGHT_ALPHA, beta=BRIGHT_BETA)
                with self.lock: self.frame = frm
            else: time.sleep(0.01)
    def get_frame(self):
        with self.lock: return None if self.frame is None else self.frame.copy()

def parse_temperatures(line: bytes):
    try:
        txt = line.decode(errors="ignore").strip()
        if txt.count(",") != 63: return None
        return np.fromstring(txt, sep=",", dtype=np.float32).reshape(8, 8)
    except: return None

def get_color_fire_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in FIRE_HUE_RANGE:
        m |= cv2.inRange(hsv, (lo, 80, 80), (hi, 255, 255))
    return m

class CamPublisher(Node):
    def __init__(self, shared, fps=30):
        super().__init__('camera_publisher')
        self.shared, self.bridge = shared, CvBridge()
        self.pub_img = self.create_publisher(Image, '/image_raw', 10)
        self.create_timer(1.0/fps, self.timer_cb)
        self.get_logger().info(f'/image_raw started @ {fps} Hz')
    def timer_cb(self):
        f = self.shared.get_frame()
        if f is not None and f.mean() >= BRIGHTNESS_MIN:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(f,'bgr8'))
    def destroy_node(self):
        self.shared.running = False
        super().destroy_node()

def create_flask_app(shared, thermal_pub, flame_pub):
    app = Flask(__name__)
    try:
        ser = serial.Serial(THERMAL_SERIAL, BAUD_RATE, timeout=1)
        thermal_ok = True
        print(f'[Flask] 熱像序列埠 {THERMAL_SERIAL} 已連接')
    except Exception as e:
        ser, thermal_ok = None, False
        print(f'[Flask] 熱像序列埠失敗：{e}')

    latest_max = {'val': None}

    # ---------- Raw ----------
    def stream_raw():
        while True:
            f = shared.get_frame()
            if f is None: time.sleep(0.01); continue
            _, jpg = cv2.imencode('.jpg', f)
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+jpg.tobytes()+b'\r\n'

    # ---------- Thermal ----------
    def stream_thermal_available():
        last_ok = time.time()
        while True:
            f = shared.get_frame()
            if f is None: time.sleep(0.01); continue

            line = ser.readline()
            thermal = parse_temperatures(line)
            if thermal is None:
                if time.time()-last_ok>2:
                    try: ser.reset_input_buffer()
                    except: pass
                    last_ok=time.time()
                time.sleep(0.01); continue
            last_ok = time.time()

            max_t = float(np.max(thermal)); latest_max['val']=max_t
            thermal_pub.publish(String(data=json.dumps({'max_temp':max_t})))

            heat = cv2.resize(thermal,(FRAME_W,FRAME_H),cv2.INTER_CUBIC)
            heat_clr = cv2.applyColorMap(
                cv2.normalize(heat,None,0,255,cv2.NORM_MINMAX).astype(np.uint8),
                cv2.COLORMAP_JET)

            mask_temp   = (heat > TEMP_THRESHOLD).astype(np.uint8)*255
            mask_color  = get_color_fire_mask(f)
            mask_fire   = cv2.bitwise_and(mask_temp,mask_color)
            mask_fire   = cv2.dilate(mask_fire,KERNEL_DILATE,1)

            has_fire = cv2.countNonZero(mask_fire) > 200
            flame_pub.publish(Bool(data=has_fire))        # ★ 即時佈告

            result = cv2.addWeighted(f,0.7,heat_clr,0.3,0)

            for c in cv2.findContours(mask_fire,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]:
                if cv2.contourArea(c)<800: continue
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(result,(x,y),(x+w,y+h),(0,0,255),2)
                cv2.putText(result,'Fire',(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,0,255),2)

            _, jpg = cv2.imencode('.jpg', result)
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+jpg.tobytes()+b'\r\n'

    def stream_thermal_unavailable():
        notice = np.zeros((FRAME_H,FRAME_W,3),np.uint8)
        cv2.putText(notice,'Thermal Device Not Found',(50,FRAME_H//2),
                    cv2.FONT_HERSHEY_SIMPLEX,2,(0,0,255),3)
        _, jpg = cv2.imencode('.jpg', notice)
        while True:
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+jpg.tobytes()+b'\r\n'

    # ---------- Routes ----------
    @app.route('/')
    def index():
        t = latest_max['val']
        return render_template_string('''
            <h2>🔥 Fire Detection Stream</h2>
            最高溫：<b>{{t}}</b>
            <ul><li><a href="/raw">Raw</a></li><li><a href="/thermal">Thermal</a></li></ul>
        ''', t=f'{t:.1f} °C' if t else 'N/A')

    @app.route('/raw')
    def raw():      return Response(stream_raw(),mimetype='multipart/x-mixed-replace; boundary=frame')
    @app.route('/thermal')
    def thermal():  return Response(
            stream_thermal_available() if thermal_ok else stream_thermal_unavailable(),
            mimetype='multipart/x-mixed-replace; boundary=frame')

    return app

def main():
    cap = cv2.VideoCapture(CAM_DEVICE)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH , FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
    if not cap.isOpened(): raise RuntimeError(f'無法開啟相機 {CAM_DEVICE}')

    shared = SharedState(cap); threading.Thread(target=shared.capture_loop,daemon=True).start()

    rclpy.init()
    ros_node   = CamPublisher(shared)
    thermal_pub= ros_node.create_publisher(String,'/thermal_data',10)
    flame_pub  = ros_node.create_publisher(Bool  ,'/flame_detection',10)   # ★ 新增

    app = create_flask_app(shared, thermal_pub, flame_pub)
    threading.Thread(target=app.run, kwargs={'host':'0.0.0.0','port':5000,'debug':False,'use_reloader':False}, daemon=True).start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node(); rclpy.shutdown()
        shared.running=False; cap.release()

if __name__ == '__main__':
    main()
