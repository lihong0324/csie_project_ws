#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class FireDetector(Node):
    def __init__(self):
        super().__init__('fire_detector')
        self.get_logger().info("火焰偵測節點已啟動")

    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info("火焰偵測節點已關閉")

def main(args=None):
    rclpy.init(args=args)
    node = FireDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
