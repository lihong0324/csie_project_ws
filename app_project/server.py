from flask import Flask, request, jsonify
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

app = Flask(__name__)

# 初始化 ROS 2 節點
rclpy.init()
ros_node = Node("esp8266_listener")
publisher = ros_node.create_publisher(String, "esp8266_data", 10)

@app.route("/data", methods=["POST"])
def receive_data():
    data = request.get_json()
    print(f"接收到的資料: {data}")

    # 將資料發布到 ROS 2 主題
    msg = String()
    msg.data = str(data)
    publisher.publish(msg)
    print("資料已發布到 ROS 2")

    return jsonify({"status": "success", "data": data})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000)