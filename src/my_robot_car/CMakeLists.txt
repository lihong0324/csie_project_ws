cmake_minimum_required(VERSION 3.5)
project(my_robot_car)

# 查找需要的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 添加 Python 腳本可執行檔
install(
  PROGRAMS
    scripts/fire_detection_control_node.py
    scripts/fire_detector_2.py
    scripts/fire_sample.py
    scripts/launcher_controller.py
    scripts/patrol_node.py  # 新增巡邏腳本
  DESTINATION lib/${PROJECT_NAME}
)

# 安裝 launch 檔案
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()