import rclpy
from rclpy.node import Node

class LauncherController(Node):
    def __init__(self):
        super().__init__('launcher_controller')
        self.declare_parameter('launch_command', 'fire')
        self.subscription = self.create_subscription(
            String, 'launcher_command', self.launch_callback, 10)

    def launch_callback(self, msg):
        if msg.data == self.get_parameter('launch_command').value:
            self.get_logger().info("Firing launcher!")
            # 加入 Gazebo 發射模擬指令

def main(args=None):
    rclpy.init(args=args)
    node = LauncherController()
    rclpy.spin(node)
    rclpy.shutdown()