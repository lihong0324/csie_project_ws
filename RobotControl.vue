<template>
  <div class="robot_control">
    <h1>ROS 2 Robot Control</h1>

    <img 
      id="camera" 
      :src="cameraUrl" 
      width="672" 
      height="384" 
      alt="Robot Camera View" 
    />

    <div id="control-panel">
      <button id="up" class="control-btn" @click="toggleMovement('up')"><strong>▲</strong></button>
      <button id="left" class="control-btn" @click="toggleMovement('left')"><strong>◀︎</strong></button>
      <button id="down" class="control-btn" @click="toggleMovement('down')"><strong>▼</strong></button>
      <button id="right" class="control-btn" @click="toggleMovement('right')"><strong>►</strong></button>
    </div>

    <button id="toggle-btn" @click="toggleControlMode">
      {{ manualControl ? '切換到自動導航' : '切換到手動控制' }}
    </button>
  </div>
</template>

<script>
import ROSLIB from "roslib";

export default {
  name: "RobotControl",
  data() {
    return {
      manualControl: false, // 控制模式（手動或自動）
      isMoving: { up: false, down: false, left: false, right: false }, // 控制方向的狀態
      ros: null, // ROS 連接對象
      cmdVel: null, // /cmd_vel 主題
      manualControlTopic: null, // /manual_control 主題
      twist: { // 預設速度消息
        linear: { x: 0.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.0 },
      },
      cameraUrl: "http://192.168.0.101:8080/stream?topic=/camera/image_raw", // 相機的串流網址
    };
  },
  methods: {
    initROS() {
      // 初始化 ROS 連接
      this.ros = new ROSLIB.Ros({
        url: "ws://192.168.0.101:9090",
      });

      this.ros.on("connection", () => {
        console.log("Connected to ROS");
      });

      this.ros.on("error", (error) => {
        console.error("Error connecting to ROS:", error);
      });

      this.ros.on("close", () => {
        console.log("Connection to ROS closed");
      });

      // 初始化 /cmd_vel 主題
      this.cmdVel = new ROSLIB.Topic({
        ros: this.ros,
        name: "/cmd_vel",
        messageType: "geometry_msgs/Twist",
      });

      // 初始化 /manual_control 主題
      this.manualControlTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/manual_control",
        messageType: "std_msgs/Bool",
      });
    },
    toggleMovement(direction) {
      if (this.isMoving[direction]) {
        // 停止移動
        this.twist.linear.x = 0.0;
        this.twist.angular.z = 0.0;
        this.isMoving[direction] = false;
      } else {
        // 根據方向設置速度
        switch (direction) {
          case "up":
            this.twist.linear.x = 0.5;
            this.twist.angular.z = 0.0;
            break;
          case "down":
            this.twist.linear.x = -0.5;
            this.twist.angular.z = 0.0;
            break;
          case "left":
            this.twist.linear.x = 0.0;
            this.twist.angular.z = 0.5;
            break;
          case "right":
            this.twist.linear.x = 0.0;
            this.twist.angular.z = -0.5;
            break;
        }
        this.isMoving[direction] = true;
      }
      this.cmdVel.publish(new ROSLIB.Message(this.twist)); // 發布速度消息
    },
    toggleControlMode() {
      // 切換手動/自動控制模式
      this.manualControl = !this.manualControl;
      this.manualControlTopic.publish(
        new ROSLIB.Message({ data: this.manualControl })
      );
    },
  },
  mounted() {
    this.initROS(); // 初始化 ROS 連接
  },
};
</script>

<style scoped>
.robot_control {
  font-family: Arial, sans-serif;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  height: 100vh;
  margin: 0;
}

#camera {
  border: 2px solid black;
  margin-bottom: 20px;
  border-radius: 20px;
}

#control-panel {
  display: grid;
  grid-template-columns: repeat(3, 100px);
  grid-template-rows: repeat(3, 100px);
  grid-gap: 10px;
  justify-items: center;
  align-items: center;
}

.control-btn {
  width: 80px;
  height: 80px;
  border-radius: 50%;
  font-size: 16px;
  text-align: center;
  line-height: 80px;
  background-color: #8cbee9d2;
  border: 1px solid black;
  cursor: pointer;
}

#toggle-btn {
  margin-top: 20px;
  padding: 10px 20px;
  font-size: 18px;
  background-color: #f0ad4e;
  border: none;
  cursor: pointer;
}
</style>

