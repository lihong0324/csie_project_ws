from flask import Flask, request, jsonify
from flask_cors import CORS
import serial
import serial.tools.list_ports
import time

app = Flask(__name__)
CORS(app)  # 讓所有站點都可以連線

# 設定一個內建變數
ser = None

# 試著連接 Arduino
try:
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if 'usb' in p.device.lower() or 'modem' in p.device.lower():
            ser = serial.Serial(p.device, 115200, timeout=1)
            print(f"已連線到Arduino: {p.device}")
            time.sleep(2)  # 等待 Arduino 重置完成
            break
    if ser is None:
        print("無法找到 Arduino")
except Exception as e:
    print(f"連線 Arduino 時發生錯誤: {e}")


@app.route('/launch', methods=['POST'])
def launch_servo():
    if ser is None:
        return jsonify({'status': 'error', 'message': '無法連線到Arduino'}), 500

    try:
        # 發送指令給 Arduino
        ser.write(b'L')  # 假設 Arduino 等待 'L' 來發起馬達
        return jsonify({'status': 'success', 'message': '馬達發射成功'}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'發送指令錯誤: {e}'}), 500


@app.route('/reset', methods=['POST'])
def reset_servo():
    if ser is None:
        return jsonify({'status': 'error', 'message': '無法連線到Arduino'}), 500

    try:
        ser.write(b'R')  # 假設 Arduino 等待 'R' 來重置馬達
        return jsonify({'status': 'success', 'message': '馬達重置成功'}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'發送重置指令錯誤: {e}'}), 500


if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5001, debug=True)
    except Exception as e:
        print(f"伺服器啟動錯誤: {e}")