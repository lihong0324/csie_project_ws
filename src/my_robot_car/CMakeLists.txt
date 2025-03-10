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
    scripts/ball_detector.py  # 確保這個路徑是正確的
    scripts/fire_detector.py  # 新增這一行
  DESTINATION lib/${PROJECT_NAME}
)
# 安裝 Python 腳本
install(
    PROGRAMS
    scripts/launcher_controller.py
    DESTINATION lib/${PROJECT_NAME}
)
# 安裝 launch 檔案
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()