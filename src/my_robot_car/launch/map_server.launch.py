import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 取得套件共享目錄
    package_name = 'my_robot_car'
    share_dir = get_package_share_directory(package_name)
    
    # 地圖檔案路徑
    map_file = os.path.join(share_dir, 'maps', 'your_map.yaml')
    
    # 啟動參數
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # SLAM工具箱節點
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'mode': 'mapping',  # 建圖模式
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'use_sim_time': use_sim_time
        }]
    )
    
    # Map Server節點
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Rosbridge WebSocket節點
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0'
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        slam_toolbox_node,
        map_server_node,
        rosbridge_node
    ])