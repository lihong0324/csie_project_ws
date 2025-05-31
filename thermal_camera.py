# fire_stream_cam_serial.py  – 2025-05-30
# 可見光 ➜ USB 相機 (/dev/video1 等)        ← 改成你的相機裝置
# 熱成像 ➜ Arduino 透過序列埠送 64 個溫度值   ← 改成你的序列埠

from flask import Flask, Response, render_template_string
import cv2, serial, time, numpy as np

app = Flask(__name__)

# ======== －－－「把這 3 行改成你的實際裝置」－－－ ========
CAM_DEVICE      = "/dev/video0"   # USB 相機（/dev/video0、/dev/video1…）
THERMAL_SERIAL  = "/dev/ttyACM0"  # Arduino 序列埠
BAUD_RATE       = 115200
# ===========================================================

FRAME_W, FRAME_H    = 1280, 720
TEMP_THRESHOLD      = 40.0               # ℃，超過視為高溫
FIRE_HUE_RANGE      = [(0, 50)]          # 火焰常見顏色 (HSV H 0-50°)

# ---------- 相機 ----------
cap = cv2.VideoCapture(CAM_DEVICE)       # 可直接給路徑
if not cap.isOpened():
    raise RuntimeError(f"[FATAL] 無法開啟相機 {CAM_DEVICE}")
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

# ---------- 序列埠 ----------
ser = serial.Serial(THERMAL_SERIAL, BAUD_RATE, timeout=1)

# ---------- 共用狀態 ----------
latest_max_temp = None   # 首頁顯示

# ---------- 工具 ----------
def parse_temperatures(line: bytes):
    """將 64 個逗號分隔值 → 8×8 numpy.float32 陣列 (°C)。"""
    try:
        txt = line.decode(errors="ignore").strip()
        if txt.count(",") != 63:
            return None
        return np.fromstring(txt, sep=",", dtype=np.float32).reshape(8, 8)
    except Exception as e:
        print("[parse_temperatures]", e)
        return None

def get_color_fire_mask(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in FIRE_HUE_RANGE:
        mask |= cv2.inRange(hsv, (lo, 100, 100), (hi, 255, 255))
    return mask

# ---------- 影像產生 ----------
def stream_raw():
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        _, jpg = cv2.imencode(".jpg", frame)
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n"

def stream_thermal():
    global latest_max_temp
    last_ok = time.time()
    while True:
        try:
            ok, frame = cap.read()
            if not ok:
                continue

            line = ser.readline()
            if not line:
                continue
            thermal = parse_temperatures(line)
            if thermal is None:
                continue
            last_ok = time.time()

            # 1. 最高溫
            max_t = float(np.max(thermal))
            latest_max_temp = max_t

            # 2. 把熱像圖放大到相機解析度並上色
            heat_img   = cv2.resize(
                thermal, (FRAME_W, FRAME_H), interpolation=cv2.INTER_CUBIC
            )
            heat_norm  = cv2.normalize(heat_img, None, 0, 255, cv2.NORM_MINMAX)
            heat_color = cv2.applyColorMap(heat_norm.astype(np.uint8),
                                           cv2.COLORMAP_JET)

            # 3. 高溫遮罩 + 火焰顏色遮罩
            mask_temp  = (heat_img > TEMP_THRESHOLD).astype(np.uint8) * 255
            mask_color = get_color_fire_mask(frame)
            mask_fire  = cv2.bitwise_and(mask_temp, mask_color)

            # 4. 疊圖
            result = cv2.addWeighted(frame, 0.7, heat_color, 0.3, 0)

            # 5. 畫火焰框
            for cnt in cv2.findContours(mask_fire, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)[0]:
                if cv2.contourArea(cnt) < 500:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(result, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(result, "Fire", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # 6. 浮水印最高溫
            cv2.putText(result, f"Max Temp: {max_t:.1f}C", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            _, jpg = cv2.imencode(".jpg", result)
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n"

        except Exception as e:
            print("[stream_thermal]", e)

        # 若 2 秒無有效資料 → 重設序列埠
        if time.time() - last_ok > 2:
            ser.reset_input_buffer()
            last_ok = time.time()

# ---------- Flask ----------
@app.route("/")
def index():
    temp = f"{latest_max_temp:.1f} °C" if latest_max_temp is not None else "N/A"
    html = """
      <h2>🔥 Fire Detection Stream</h2>
      <p>熱成像目前偵測到的最高溫：<b>{{temp}}</b></p>
      <ul>
        <li><a href="/raw">Raw Camera Feed</a></li>
        <li><a href="/thermal">Thermal Fire Detection</a></li>
      </ul>
    """
    return render_template_string(html, temp=temp)

@app.route("/raw")
def raw_feed():
    return Response(stream_raw(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/thermal")
def thermal_feed():
    return Response(stream_thermal(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
