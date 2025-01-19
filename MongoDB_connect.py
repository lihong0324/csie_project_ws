import rclpy
from rclpy.node import Node
from pymongo import MongoClient
from std_msgs.msg import String
import json

class MongoDBClient(Node):
    def __init__(self):
        super().__init__('mongodb_client')

        # 設定 ROS 2 發佈器
        self.publisher_ = self.create_publisher(String, 'mongodb_data', 10)
        
        # 定時器，每 10 秒進行一次查詢並發佈
        self.timer = self.create_timer(10, self.query_data)

        # 修改為遠端伺服器的 IP 位址和連接埠，並指定使用者名稱和密碼
        self.client = MongoClient('mongodb://192.168.1.100:27017/')
        self.db = self.client['ros_database']  # 資料庫名稱
        self.collection = self.db['data_collection']  # 集合名稱
        
        # 插入一些初始數據
        self.insert_data()

    def insert_data(self):
        # 模擬插入資料
        data = {
            "name": "ROS Robot",
            "status": "active",
            "location": {"x": 1.0, "y": 2.0, "z": 0.0},
            "timestamp": self.get_clock().now().to_msg().sec
        }
        self.collection.insert_one(data)
        self.get_logger().info('已插入初始資料到 MongoDB。')

    def query_data(self):
        # 查詢 MongoDB 中的最新資料
        latest_data = self.collection.find().sort([('_id', -1)]).limit(1)  # 取得最新一筆資料
        for data in latest_data:
            msg = String()
            msg.data = json.dumps(data, default=str)  # 將資料轉成 JSON 格式
            self.publisher_.publish(msg)
            self.get_logger().info(f'從 MongoDB 發佈數據: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    mongodb_client = MongoDBClient()
    rclpy.spin(mongodb_client)
    
    # 關閉節點和資料庫連接
    mongodb_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()