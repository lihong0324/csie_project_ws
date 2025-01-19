from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-file', 'urdf/my_robot_car.urdf', '-entity', 'robot_car'],
            output='screen')
    ])