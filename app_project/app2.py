from flask import Flask, render_template

app = Flask(__name__)

@app.route('/')
def index():
    """顯示控制頁面"""
    return render_template('index.html')  # 加載 templates/index.html 文件

if __name__ == '__main__':
    # 允許其他設備通過內網 IP 訪問
    app.run(host='0.0.0.0', port=5001)