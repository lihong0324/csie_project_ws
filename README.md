# 機器人控制介面專案

## 簡介

本專案旨在建立一個機器人控制介面，結合了 ROS2、網頁開發（HTML）、Python 等技術。專案主要目標是開發一個可以控制機器人的介面，並在 Gazebo 模擬環境中進行驗證。此外，本專案還包含了一些 Python 代碼，用於實現球體偵測和熱感偵測的功能。

## 技術棧

-   **ROS2**: 用於機器人操作系統，處理機器人的運動控制、傳感器數據等。
-   **網頁開發 (HTML)**: 用於構建機器人控制介面的使用者介面。
-   **Python**: 用於編寫控制邏輯、球體偵測、熱感偵測等功能。
-   **Gazebo**: 用作機器人模擬環境，測試機器人控制程式。

## 主要功能

-   **機器人控制介面**: 提供一個使用者介面，方便使用者控制機器人。
-   **Gazebo 模擬**: 使用 Gazebo 作為模擬環境，測試機器人的控制和運動。
-   **球體偵測**: 透過 Python 實現球體偵測功能。
-   **熱感偵測**: 透過 Python 實現熱感偵測功能。

## 專案文件

-   `MongoDB_connect.py`
-   `ROS2指令.txt`
-   `RobotControl.vue`
-   `Thermal.html`
-   `ball_detector.py`
-   `connection.html`
-   `fire_thermal.py`
-   `i2c_bus.py`
-   `index.html`
-   `map.html`
-   `my_robot.html`
-   `my_robot_car.urdf`
-   `my_robot_new.html`
-   `ros2_slam.html`
-   `web_video_server_qos.yaml`
-   `.vscode/settings.json`
-   `__pycache__/app.cpython-311.pyc`
-   `__pycache__/flask.cpython-311.pyc`
-   `app_project/app2.py`
-   `app_project/server.py`
-   `app_project/static/style.css`
-   `app_project/templates/index.html`
-   `src/my_robot_car/CMakeLists.txt`
-   `src/my_robot_car/package.xml`
-   `src/my_robot_car/setup.py`
-   `src/my_robot_car/config/control.yaml`
-   `src/my_robot_car/launch/gazebo.launch.py`
-   `src/my_robot_car/launch/map_server.launch.py`
-   `src/my_robot_car/scripts/__init__.py`
-   `src/my_robot_car/scripts/fire_detection_control_node.py`
-   `src/my_robot_car/scripts/fire_detector_2.py`
-   `src/my_robot_car/scripts/fire_sample.py`
-   `src/my_robot_car/scripts/launcher_controller.py`
-   `src/my_robot_car/urdf/my_robot_car.urdf`
-   `src/my_robot_car/worlds/fire_detection_world.world`