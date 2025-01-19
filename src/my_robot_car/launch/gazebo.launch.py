import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 定義 URDF 文件的完整路徑
    package_path = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],  # 預設取第一個工作區
        "share", "my_robot_car", "urdf", "my_robot_car.urdf"
    )
    
    # 檢查 URDF 文件是否存在
    if not os.path.exists(package_path):
        raise FileNotFoundError(f"URDF file not found at: {package_path}")
    
    return LaunchDescription([
        # 啟動 Gazebo 模擬器
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.dylib'],
            output='screen'
        ),
        # 加載 URDF 文件並生成實體
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "robot_car",
                "-file", package_path,
            ],
            output="screen",
        )
    ])