import serial
import numpy as np
import cv2
import threading
from flask import Flask, Response, jsonify, request, redirect, url_for
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

# 設定串口與波特率
ser = serial.Serial('COM6', 115200)

# 終止程序的標誌
stop_flag = False

# 啟動攝影機
cap = cv2.VideoCapture(0)

# 熱成像與攝影機疊加畫面
overlay_frame = None
thermal_matrix = np.zeros((8, 8))  # 存儲最新的 8x8 溫度數據
lock = threading.Lock()

# HSI 影像處理函數
def BGR_to_HSI(image):
    with np.errstate(divide='ignore', invalid='ignore'):
        bgr = np.float32(image) / 255.0
        B, G, R = bgr[:,:,0], bgr[:,:,1], bgr[:,:,2]
        
        num = 0.5 * ((R - G) + (R - B))
        den = np.sqrt((R - G) ** 2 + (R - B) * (G - B))
        theta = np.arccos(num / (den + 1e-6))

        H = np.where(B <= G, theta, 2 * np.pi - theta)
        H = H * 180 / np.pi

        min_val = np.minimum(np.minimum(R, G), B)
        I = (R + G + B) / 3
        S = 1 - (min_val / (I + 1e-6))

        HSI = np.zeros_like(bgr)
        HSI[:,:,0] = H
        HSI[:,:,1] = S
        HSI[:,:,2] = I

    return HSI

def fire_HSI_threshold(hsi_image):
    lower_H, upper_H = 0, 35
    lower_S, upper_S = 0.4, 1.0
    lower_I, upper_I = 0.5, 1.0

    mask = (hsi_image[:,:,0] >= lower_H) & (hsi_image[:,:,0] <= upper_H) & \
           (hsi_image[:,:,1] >= lower_S) & (hsi_image[:,:,1] <= upper_S) & \
           (hsi_image[:,:,2] >= lower_I) & (hsi_image[:,:,2] <= upper_I)

    return np.uint8(mask * 255)

# 熱成像處理函數
def process_data():
    global overlay_frame, thermal_matrix, stop_flag
    while not stop_flag:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            try:
                pixels = list(map(float, data.split(',')))
                if len(pixels) == 64:
                    thermal_matrix = np.array(pixels).reshape((8, 8))

                    thermal_image_normalized = cv2.normalize(thermal_matrix, None, 0, 255, cv2.NORM_MINMAX)
                    thermal_image_resized = cv2.resize(thermal_image_normalized, (640, 480), interpolation=cv2.INTER_LINEAR)
                    color_mapped_image = cv2.applyColorMap(np.uint8(thermal_image_resized), cv2.COLORMAP_JET)

                    ret, camera_frame = cap.read()
                    if not ret:
                        continue

                    camera_frame_resized = cv2.resize(camera_frame, (640, 480))
                    overlay = cv2.addWeighted(camera_frame_resized, 0.6, color_mapped_image, 0.4, 0)

                    # 整合 HSI 火焰辨識
                    hsi_img = BGR_to_HSI(camera_frame_resized)
                    mask = fire_HSI_threshold(hsi_img)

                    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
                    mask = cv2.dilate(mask, kernel)
                    mask = cv2.erode(mask, kernel)

                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    
                    for contour in contours:
                        area = cv2.contourArea(contour)
                        if area > 300:
                            x, y, w, h = cv2.boundingRect(contour)

                            # 將影像座標對應到 8x8 熱成像座標
                            thermal_x = int((x / 640) * 8)
                            thermal_y = int((y / 480) * 8)

                            # 確保座標有效
                            if 0 <= thermal_x < 8 and 0 <= thermal_y < 8:
                                temp = thermal_matrix[thermal_y, thermal_x]
                                if temp >= 50:  # 設定溫度閾值
                                    overlay = cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 0, 255), 3)
                                    cv2.putText(overlay, f"{temp:.1f}C", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    with lock:
                        overlay_frame = overlay

            except ValueError:
                continue
def generate_frames():
    global overlay_frame
    while not stop_flag:
        with lock:
            if overlay_frame is None:
                continue
            _, buffer = cv2.imencode('.jpg', overlay_frame)
            frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# 提供 API 來獲取溫度
@app.route('/get_temperature')
def get_temperature():
    x = int(request.args.get("x", 0))
    y = int(request.args.get("y", 0))
    if 0 <= x < 8 and 0 <= y < 8:
        return jsonify({"temperature": round(float(thermal_matrix[y, x]), 1)})
    return jsonify({"error": "Invalid coordinates"}), 400
@app.route('/')
def index():
    return '''
    <html>
    <head><title>熱成像攝影機</title></head>
    <body>
        <h1>點擊下方觀看影像</h1>
        <a href="/video_feed">觀看影像串流</a>
    </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# 啟動線程
thread = threading.Thread(target=process_data)
thread.start()

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    finally:
        stop_flag = True
        thread.join()
        cap.release()
        ser.close()
        cv2.destroyAllWindows()