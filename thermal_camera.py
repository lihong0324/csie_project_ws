from flask import Flask, Response, render_template_string
import serial, time
import numpy as np
import cv2

app = Flask(__name__)

# ----- 串列埠 & 攝影機設定 -----
SERIAL_PORT   = '/dev/ttyUSB0' # macOS/Linux: '/dev/ttyUSB0' 或 '/dev/ttyACM0'
BAUD_RATE     = 115200
TEMP_THRESHOLD = 40.0
FIRE_HUE_RANGE = [(0, 50)]
CAMERA_INDEX  = 0
FRAME_W, FRAME_H = 1280, 720

cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# ---------- 工具函式 ----------
def parse_temperatures(line: bytes):
    """將 64 個逗號分隔值轉成 8×8 numpy 陣列 (°C)。"""
    try:
        decoded = line.decode(errors='ignore').strip()
        if decoded.count(',') != 63:
            return None
        vals = np.array(list(map(float, decoded.split(',')))).reshape(8, 8)
        return vals
    except Exception as e:
        print(f"[ERROR] parse_temperatures: {e}")
        return None

def get_color_fire_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for low_h, high_h in FIRE_HUE_RANGE:
        mask_total |= cv2.inRange(hsv, (low_h, 100, 100), (high_h, 255, 255))
    return mask_total

# ---------- 影像產生 ----------
latest_max_temp = None  # 用來在首頁顯示

def generate_raw():
    while True:
        ok, frame = cap.read()
        if not ok: continue
        ok, jpg = cv2.imencode('.jpg', frame)
        if not ok: continue
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n'

def generate_thermal():
    global latest_max_temp
    last_ok = time.time()
    while True:
        try:
            ok, frame = cap.read()
            if not ok: continue

            line = ser.readline()
            if not line: continue
            thermal = parse_temperatures(line)
            if thermal is None: continue
            last_ok = time.time()

            # 1️⃣ 最高溫
            max_temp = float(np.max(thermal))
            latest_max_temp = max_temp

            # 2️⃣ 熱像圖
            heat = cv2.resize(thermal, (FRAME_W, FRAME_H),
                              interpolation=cv2.INTER_CUBIC)
            heat_norm = cv2.normalize(heat, None, 0, 255,
                                      cv2.NORM_MINMAX).astype(np.uint8)
            heat_color = cv2.applyColorMap(heat_norm, cv2.COLORMAP_JET)

            fire_mask_temp  = (heat > TEMP_THRESHOLD).astype(np.uint8)*255
            fire_mask_color = get_color_fire_mask(frame)
            fire_mask = cv2.bitwise_and(fire_mask_temp, fire_mask_color)

            result = cv2.addWeighted(frame, 0.7, heat_color, 0.3, 0)
            for cnt in cv2.findContours(fire_mask,
                                        cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)[0]:
                if cv2.contourArea(cnt) > 500:
                    x,y,w,h = cv2.boundingRect(cnt)
                    cv2.rectangle(result, (x,y), (x+w,y+h), (0,0,255), 2)
                    cv2.putText(result, "Fire", (x, y-10),
                                cv2.FONT_HERSHEY_SIMPLEX, .8, (0,0,255), 2)

            # 3️⃣ 疊字顯示最高溫
            cv2.putText(result, f"Max Temp: {max_temp:.1f}C", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            ok, jpg = cv2.imencode('.jpg', result)
            if not ok: continue
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n'

        except Exception as e:
            print(f"[FATAL] generate_thermal: {e}")

        # 若 2 秒無新資料 → 清 buffer
        if time.time() - last_ok > 2:
            ser.reset_input_buffer()
            last_ok = time.time()

# ---------- Flask Routes ----------
@app.route('/')
def index():
    temp_txt = f"{latest_max_temp:.1f} °C" if latest_max_temp is not None else "N/A"
    html = '''
        <h2>🔥 Fire Detection Stream</h2>
        <p>熱成像目前偵測到的最高溫：<b>{{temp}}</b></p>
        <ul>
            <li><a href="/video_feed_raw">Raw Camera Feed</a></li>
            <li><a href="/video_feed_thermal">Thermal Fire Detection</a></li>
        </ul>
    '''
    return render_template_string(html, temp=temp_txt)

@app.route('/video_feed_raw')
def video_feed_raw():
    return Response(generate_raw(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed_thermal')
def video_feed_thermal():
    return Response(generate_thermal(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)