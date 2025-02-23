import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

class FireTrackingNode(Node):
    def __init__(self):
        super().__init__('fire_tracking_node')
        
        self.fire_detected = False
        self.lidar_distance = float('inf')  # 初始距離無限遠
        self.searching = False

        # 訂閱火焰偵測結果
        self.create_subscription(Bool, '/fire_detected', self.fire_callback, 10)
        
        # 訂閱 LiDAR 掃描數據
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # 發布機器車移動指令
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Fire Tracking Node Started")

    def fire_callback(self, msg):
        self.fire_detected = msg.data
        if not self.fire_detected:
            # 如果火焰消失，開始旋轉尋找火焰
            self.start_searching()

    def lidar_callback(self, msg):
        # 取得正前方的距離（取中間一部分的數據）
        center_index = len(msg.ranges) // 2
        self.lidar_distance = msg.ranges[center_index]
        
        if self.fire_detected:
            self.maintain_distance()

    def maintain_distance(self):
        """保持機器車與火焰 5 公尺距離"""
        twist = Twist()
        target_distance = 5.0  # 目標距離 5 公尺
        tolerance = 0.5  # 允許誤差 0.5 公尺

        if self.lidar_distance > target_distance + tolerance:
            # 火焰太遠，前進
            twist.linear.x = 0.2
        elif self.lidar_distance < target_distance - tolerance:
            # 火焰太近，後退
            twist.linear.x = -0.2
        else:
            # 保持距離，不動
            twist.linear.x = 0.0
        
        self.cmd_pub.publish(twist)

    def start_searching(self):
        """當火焰消失時，機器車往右轉尋找火焰"""
        if not self.searching:
            self.get_logger().info("Fire lost, rotating to search...")
            self.searching = True

        twist = Twist()
        twist.angular.z = -0.3  # 向右旋轉
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FireTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()