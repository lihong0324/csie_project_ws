from flask import Flask, send_from_directory

app = Flask(__name__)

@app.route('/')
def index():
    return send_from_directory('.', 'templates/ros2_map.html')

if __name__ == '__main__':
    # 設定 host 為 0.0.0.0 讓區網內其他裝置可連線
    app.run(host='0.0.0.0', port=8080)