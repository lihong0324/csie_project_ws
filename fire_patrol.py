#!/usr/bin/env python3
# patrol_fire_node.py – 2025-06-03 (再修改版)
# Nav2 巡邏 + 動態火焰追蹤 + 自動發射（火焰只偵測最強強度、導航失敗時先往前一小步再重試同一點）

import rclpy
import cv2
import serial
import time
import glob
import numpy as np
import requests
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
try:
    # 新版 simple-commander
    from nav2_simple_commander.robot_navigator import TaskResult as NavigationResult
except ImportError:
    # 舊版（原本的 NavigationResult 還在）
    from nav2_simple_commander.robot_navigator import NavigationResult

# === 影像 / GUI 參數（同 flame_tracker） ===
WIDTH, HEIGHT, GRID_STEP = 1000, 750, 50
COLOR_MAP = [
    (50, 50, 255), (100, 100, 255), (130, 150, 255), (0, 200, 255),
    (0, 255, 255), (0, 255, 150), (0, 255, 100), (0, 255, 50),
    (0, 255, 0), (100, 255, 0), (180, 255, 0), (255, 255, 0),
    (255, 200, 0), (255, 150, 0), (255, 80, 0), (255, 255, 255)
]

LAUNCH_URL = 'http://localhost:5001/launch'  # servo_control.py 的端點

class PatrolFire(Node):
    def __init__(self):
        super().__init__('patrol_fire')

        # ---------- 可調參數 ----------
        self.target_s        = 2
        self.forward_spd     = 0.15
        self.angular_spd     = 0.35
        self.center_tol_pct  = 10.0
        self.confirm_sec     = 0.4
        self.lost_sec        = 0.6
        self.center_fire_sec = 2.0

        # ---------- 火焰感測序列埠 ----------
        self.ser = self._auto_serial(115200)
        if not self.ser:
            self.get_logger().fatal("找不到 Arduino 串列埠！")
            raise SystemExit

        # ---------- cmd_vel Publisher ----------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---------- Nav2 Navigator ----------
        self.nav = BasicNavigator()
        self.get_logger().info('等待 Nav2…')
        self.nav.waitUntilNav2Active()

        # ---------- 巡邏點 ----------
        all_goals = [
            (1.55, -10.5), (5.81, -7.25), (4.24, -5.05),
            (0.0706, -8.33), (3.32, -2.74), (-1.57, -5.91)
        ]
        seq = [0, 1, 2, 3, 2, 4, 5, 4, 1, 0]
        self.goals = [self._make_pose(*all_goals[i]) for i in seq]
        self.goal_idx = 0

        # ---------- 狀態機 ----------
        # NAVIGATE / APPROACH / TRACK / FIRE / DONE_NAV / DONE
        self.mode          = 'NAVIGATE'
        self.confirm_timer = None
        self.center_timer  = None
        self.lost_timer    = None
        self.fired         = False

        # ---------- 初始化火焰資料 ----------
        self.coords = []
        self.max_s = 15

        # ---------- OpenCV 視窗 ----------
        cv2.namedWindow('Flame Patrol', cv2.WINDOW_NORMAL)

        # ---------- Timers ----------
        self.create_timer(0.05, self._loop)  # 20 Hz
        self.send_next_goal()
        self.get_logger().info('Patrol-fire node 已啟動')

    # === 工具：自動搜尋 Arduino 串列埠 ===
    def _auto_serial(self, baud):
        for p in glob.glob('/dev/ttyACM0'):
            try:
                s = serial.Serial(p, baud, timeout=1)
                time.sleep(2)
                # 確認讀到火焰資料格式（至少有 9 個逗號）
                if s.readline().decode('utf-8', 'ignore').count(',') >= 9:
                    self.get_logger().info(f'Connected {p}')
                    return s
                s.close()
            except:
                pass
        return None

    # === 工具：產生 PoseStamped ===
    def _make_pose(self, x, y, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = math.cos(yaw / 2)
        pose.pose.orientation.z = math.sin(yaw / 2)
        return pose

    # === 主迴圈 ===
    def _loop(self):
        # 如果尚未發射，讀取火焰資料；若已發射，清空 coords
        if not self.fired:
            self._read_flame_packet()
        else:
            self.coords, self.max_s = [], 15

        self._nav_state_machine()
        self._gui_update()

    # ---------- 讀取並只保留最強火焰座標 ----------
    def _read_flame_packet(self):
        raw = self.ser.readline()
        if not raw:
            self.coords, self.max_s = [], 15
            return
        try:
            parts = list(map(int, raw.decode(errors='ignore').strip().split(',')))
        except ValueError:
            self.coords, self.max_s = [], 15
            return
        if len(parts) % 3:
            self.coords, self.max_s = [], 15
            return

        candidates = []
        for i in range(0, len(parts), 3):
            ix, iy, s = parts[i : i + 3]
            if ix == 1023 or iy == 1023:
                continue
            candidates.append((ix, iy, s))

        if not candidates:
            self.coords, self.max_s = [], 15
            return

        # 取最強（最小 s 值）的火焰像素
        strongest = min(candidates, key=lambda c: c[2])
        self.coords = [strongest]
        self.max_s = strongest[2]

    # ---------- 狀態機：巡邏 + 火焰偵測 + 發射 ---------- 
    def _nav_state_machine(self):
        now = time.time()
        flame_seen = (not self.fired) and bool(self.coords) and self.max_s < 15

        # 計算當前最強火焰 x 座標（若有）
        if self.coords:
            ix, iy, s = self.coords[0]
            self.cx = WIDTH - int(iy * WIDTH / 1024)
        else:
            self.cx = WIDTH / 2

        mid = WIDTH / 2
        tol_pix = WIDTH * (self.center_tol_pct / 100)

        # ============ NAVIGATE 模式 ============ #
        if self.mode == 'NAVIGATE':
            # 1) 如果 Nav2 完成 → 發送下一個目標或結束
            if self.nav.isTaskComplete():
                res = self.nav.getResult()
                if res == NavigationResult.SUCCEEDED:
                    # 如果已經是最後一點，進入 DONE
                    if self.goal_idx == len(self.goals) - 1:
                        self.get_logger().info('抵達最後一個導航點，任務結束')
                        self.mode = 'DONE'
                        return
                    # 否則前往下一點
                    self.goal_idx += 1
                    self.send_next_goal()

                elif res == NavigationResult.FAILED:
                    # 導航失敗 → 先往前小步，再重試同一點
                    self.get_logger().warn('導航失敗，先往前小步，再重試同一點')
                    self._small_forward_and_retry()

            # 2) 尚未發射且偵測到火焰 → Abort Nav2, 進入 APPROACH
            if flame_seen:
                self.get_logger().info('🔥 火焰偵測，暫停導航')
                self.nav.cancelTask()
                self.mode = 'APPROACH'
                self.confirm_timer = None
                self.center_timer  = None
                self.lost_timer    = None
                return

        # ============ FIRE 模式（APPROACH / TRACK / FIRE）=========== #
        if self.mode in ('APPROACH', 'TRACK', 'FIRE'):
            twist = Twist()

            # ---- 火焰消失 → 誤報，回 NAVIGATE ----
            if not flame_seen:
                self.lost_timer = self.lost_timer or now
                if (now - self.lost_timer) >= self.lost_sec:
                    self.get_logger().info('火焰遺失，回到導航')
                    self.mode = 'NAVIGATE'
                    self.confirm_timer = None
                    self.center_timer  = None
                    self.lost_timer    = None
                # 否則持續等待
            else:
                self.lost_timer = None

            # APPROACH 邏輯：遠離火焰 → 逐步接近
            if self.mode == 'APPROACH':
                if self.max_s > self.target_s:
                    twist.linear.x  = self.forward_spd
                    twist.angular.z = self._angle_cmd(self.cx, mid, tol_pix)
                else:
                    # 當最強像素強度達目標 → 進入 TRACK
                    self.mode = 'TRACK'
                    self.get_logger().info('S <= target_s → TRACK')
                self.cmd_pub.publish(twist)

            # TRACK 邏輯：對準中心，等待發射時機
            elif self.mode == 'TRACK':
                if abs(self.cx - mid) > tol_pix:
                    twist.angular.z = self._angle_cmd(self.cx, mid, tol_pix) * 0.5
                    self.center_timer = None
                else:
                    twist.angular.z = 0.0
                    self.center_timer = self.center_timer or now
                    if (now - self.center_timer) >= self.center_fire_sec and not self.fired:
                        self.mode = 'FIRE'
                self.cmd_pub.publish(twist)

            # FIRE：發射一次後跳到最後一點
            elif self.mode == 'FIRE' and not self.fired:
                self._launch_ball()
                self.fired = True
                # 直接將目標移到最後一點，繼續導航
                self.goal_idx = len(self.goals) - 1
                self.send_next_goal()
                self.mode = 'NAVIGATE'

        # ============ DONE 模式 ============ #
        if self.mode == 'DONE':
            # 發送零速度，確保車子停下
            stop_twist = Twist()
            stop_twist.linear.x  = 0.0
            stop_twist.angular.z = 0.0
            self.cmd_pub.publish(stop_twist)
            return

    # ---------- 小步向前並重試同一點 ---------- 
    def _small_forward_and_retry(self):
        # 發送「往前小步」指令
        bump_twist = Twist()
        bump_twist.linear.x = 0.1   # 小步速率
        bump_twist.angular.z = 0.0
        duration = 0.5             # 持續 0.5 秒

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_pub.publish(bump_twist)
            # 每次 loop 間隔 0.05 秒
            time.sleep(0.05)

        # 停止前進
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_pub.publish(stop_twist)

        # 再次嘗試同一個導航點
        self.send_next_goal()

    # ---------- 發射 HTTP ---------- 
    def _launch_ball(self):
        try:
            r = requests.post(LAUNCH_URL, timeout=2)
            if r.ok:
                self.get_logger().info('🔥 Ball launched!')
            else:
                self.get_logger().error(f'Launch failed: {r.text}')
        except requests.RequestException as e:
            self.get_logger().error(f'HTTP error: {e}')

    # ---------- 角速度計算 ---------- 
    def _angle_cmd(self, cx, mid, tol):
        if abs(cx - mid) < tol:
            return 0.0
        return self.angular_spd if cx < mid - tol else -self.angular_spd

    # ---------- 發送下一個目標 ---------- 
    def send_next_goal(self):
        if self.goal_idx >= len(self.goals):
            return
        pose = self.goals[self.goal_idx]
        self.nav.goToPose(pose)
        self.get_logger().info(f'導航至 #{self.goal_idx + 1}/{len(self.goals)}')

    # ---------- GUI 更新 ---------- 
    def _gui_update(self):
        frame = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        for x in range(0, WIDTH, GRID_STEP):
            cv2.line(frame, (x, 0), (x, HEIGHT), (0, 120, 0), 1)
        for y in range(0, HEIGHT, GRID_STEP):
            cv2.line(frame, (0, y), (WIDTH, y), (0, 120, 0), 1)
        for ix, iy, s in self.coords:
            px = int(iy * WIDTH / 1024)
            py = int(ix * HEIGHT / 1024)
            cv2.circle(frame, (px, py), 10, COLOR_MAP[min(s, 15)], -1)
        frame = cv2.flip(frame, -1)

        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (580, 240), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.4, frame, 0.6, 0, frame)

        cv2.putText(frame, f"Mode: {self.mode}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
        cv2.putText(frame, f"Goal: {self.goal_idx + 1}/{len(self.goals)}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
        cv2.putText(frame, f"Max S: {self.max_s}", (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 100), 2)
        if self.fired:
            cv2.putText(frame, "FIREED!", (int(WIDTH / 2) - 60, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3)
        cv2.imshow('Flame Patrol', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PatrolFire()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
