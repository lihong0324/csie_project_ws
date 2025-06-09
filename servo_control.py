from flask import Flask, jsonify
from flask_cors import CORS
import serial
import time

app = Flask(__name__)
CORS(app)

# 串列埠設定：請依照實際連接埠修改
SERIAL_PORT = '/dev/cu.usbmodem1101'  # Windows 用戶請改成 'COM3' 等
BAUD_RATE = 115200

# 初始化串列連線
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # 等待 Arduino 重置完成
except serial.SerialException as e:
    print(f"串列連線失敗：{e}")
    ser = None

@app.route('/launch', methods=['POST'])
def launch():
    if ser:
        ser.write(b'launch\n')  # 加上 \n 當作結尾
        return jsonify({'message': '發射命令已送出'})
    else:
        return jsonify({'message': '發射失敗：未連接到 Arduino'}), 500

@app.route('/reset', methods=['POST'])
def reset():
    if ser:
        ser.write(b'reset\n')  # 加上 \n 當作結尾
        return jsonify({'message': '歸位命令已送出'})
    else:
        return jsonify({'message': '歸位失敗：未連接到 Arduino'}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)