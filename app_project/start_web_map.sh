#!/bin/bash

# ROS2 網頁地圖啟動腳本
# 使用方法: chmod +x start_web_map.sh && ./start_web_map.sh

echo "啟動 ROS2 網頁地圖系統..."

# 檢查 ROS2 環境
if ! command -v ros2 &> /dev/null; then
    echo "ROS2 未安裝或未 source"
    echo "請執行: source /opt/ros/humble/setup.bash"
    exit 1
fi

# 函數：在新終端執行命令
run_in_new_terminal() {
    local cmd="$1"
    local title="$2"
    
    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --title="$title" -- bash -c "$cmd; exec bash"
    elif command -v xterm &> /dev/null; then
        xterm -T "$title" -e "$cmd; bash" &
    else
        echo "請手動在新終端執行: $cmd"
    fi
    
    sleep 2  # 等待服務啟動
}

echo "1 啟動 ROS Bridge..."
run_in_new_terminal "ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090" "ROS Bridge"

echo "2 檢查地圖話題..."
sleep 3

# 檢查是否有地圖話題
if ros2 topic list | grep -q "/map"; then
    echo "發現地圖話題 /map"
else
    echo "未發現地圖話題，啟動地圖服務器..."
    
    # 檢查是否有地圖文件
    if [ -f "$HOME/map.yaml" ]; then
        run_in_new_terminal "ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$HOME/map.yaml" "Map Server"
    else
        echo "啟動 SLAM 建圖..."
        run_in_new_terminal "ros2 launch slam_toolbox online_async_launch.py" "SLAM"
    fi
fi

echo "3 檢查定位話題..."
sleep 2

if ! ros2 topic list | grep -q "/amcl_pose"; then
    echo "啟動 AMCL 定位..."
    run_in_new_terminal "ros2 run nav2_amcl amcl" "AMCL"
fi

echo "4 檢查代價地圖話題..."
sleep 2

if ! ros2 topic list | grep -q "costmap"; then
    echo "啟動導航系統..."
    run_in_new_terminal "ros2 launch nav2_bringup navigation_launch.py" "Navigation"
fi

echo "5 啟動網頁服務器..."
sleep 3

if [ -f "app.py" ]; then
    echo "啟動 Flask 服務器..."
    python3 app.py &
    FLASK_PID=$!
    sleep 2
    
    echo "系統啟動完成！"
    echo ""
    echo "請在瀏覽器開啟: http://localhost:8080"
    echo "RViz2 (可選): rviz2"
    echo ""
    echo "有用的命令:"
    echo "   查看話題: ros2 topic list"
    echo "   查看地圖: ros2 topic echo /map"
    echo "   查看機器人位置: ros2 topic echo /amcl_pose"
    echo ""
    echo "按 Ctrl+C 停止所有服務"
    
    # 等待用戶中斷
    trap "echo '正在停止服務...'; kill $FLASK_PID 2>/dev/null; exit" INT
    wait
    
else
    echo "未找到 app.py 文件"
    echo "請確保在正確的目錄中運行此腳本"
fi