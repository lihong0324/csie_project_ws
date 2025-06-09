from flask import Flask, Response, render_template_string
import serial, time, threading
import numpy as np, cv2

app = Flask(__name__)

# === 硬體設定 ===
SERIAL_PORT   = '/dev/tty.usbmodem1101'
BAUD_RATE     = 115200
TEMP_TH       = 35.0
FIRE_HUE      = [(0,50)]     # HSV
CAM_ID        = 1
W,H           = 640,360      # 可再改回 1280,720

# === Camera 初始化（低延遲） ===
cap = cv2.VideoCapture(CAM_ID, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH , W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
cap.set(cv2.CAP_PROP_BUFFERSIZE , 1)

# === Serial 背景讀執行緒 ===
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
latest_thermal = np.zeros((8,8), np.float32)

def parse_thermal(line: bytes):
    try:
        txt = line.decode(errors='ignore').strip()
        if txt.count(',') != 63: return None
        return np.array(list(map(float, txt.split(','))), dtype=np.float32).reshape(8,8)
    except: return None

def serial_loop():
    global latest_thermal
    while True:
        line = ser.readline()
        th = parse_thermal(line)
        if th is not None:
            latest_thermal = th

thr = threading.Thread(target=serial_loop, daemon=True)
thr.start()

kernel = np.ones((7,7), np.uint8)
def get_fire_mask_color(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo,hi in FIRE_HUE:
        mask |= cv2.inRange(hsv, (lo, 80, 80), (hi,255,255))
    return mask

latest_max, latest_fire = None, False

# --- 原始影像 ---
def generate_raw():
    while True:
        _ = cap.grab()
        ok, frame = cap.retrieve()
        if not ok: continue
        ok, jpg = cv2.imencode('.jpg', frame)
        if ok:
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n'

# --- 熱成像融合 ---
def generate_thermal():
    global latest_max, latest_fire
    while True:
        _ = cap.grab()
        ok, frame = cap.retrieve()
        if not ok: continue

        thermal = latest_thermal
        max_t   = float(np.max(thermal)); latest_max = max_t

        heat = cv2.resize(thermal, (W,H), cv2.INTER_CUBIC)
        mask_hot = (heat > TEMP_TH).astype(np.uint8)*255
        mask_col = get_fire_mask_color(frame)
        mask_and = cv2.bitwise_and(mask_hot, mask_col)
        mask_and = cv2.dilate(mask_and, kernel, iterations=1)

        has_fire = cv2.countNonZero(mask_and) > 200
        latest_fire = has_fire

        heat_vis = cv2.applyColorMap(
            cv2.normalize(heat, None, 0,255, cv2.NORM_MINMAX).astype(np.uint8),
            cv2.COLORMAP_JET)
        out = cv2.addWeighted(frame, 0.7, heat_vis, 0.3, 0)

        for c in cv2.findContours(mask_and, cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)[0]:
            if cv2.contourArea(c) > 800:
                x,y,w,h_ = cv2.boundingRect(c)
                cv2.rectangle(out,(x,y),(x+w,y+h_),(0,0,255),2)
                cv2.putText(out,"Fire",(x,y-8),cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,(0,0,255),2)

        cv2.putText(out, f"Max {max_t:.1f} C", (10,25),
                    cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)
        cv2.putText(out, f"Fire: {'YES' if has_fire else 'NO'}", (10,50),
                    cv2.FONT_HERSHEY_SIMPLEX,0.7,
                    (0,0,255) if has_fire else (180,180,180),2)

        ok,jpg = cv2.imencode('.jpg', out)
        if ok:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                   + jpg.tobytes() + b'\r\n')

# === Flask Routes ===
@app.route('/')
def idx():
    t = f"{latest_max:.1f} °C" if latest_max else "N/A"
    f = "<span style='color:red;font-weight:bold'>YES</span>" if latest_fire \
        else "<span style='color:gray'>NO</span>"
    return render_template_string('''
      <h2>🔥 Fire Detection – Low-Latency</h2>
      最高溫：<b>{{T}}</b><br>偵測火焰：{{F}}<br><br>
      <ul>
        <li><a href="/raw">原始影像</a></li>
        <li><a href="/thermal">熱成像疊圖</a></li>
      </ul>
    ''', T=t, F=f)

@app.route('/raw')
def raw():
    return Response(generate_raw(),
        mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/thermal')
def thermal():
    return Response(generate_thermal(),
        mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=False)
