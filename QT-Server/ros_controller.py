import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped 
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import socket
import time
import math
import requests
import threading
from collections import deque

UDP_PORT_UWB = 44444
UDP_PORT_QT = 55555
BUFFER_SIZE = 1024
SERVER_IP = "192.168.123.43"
API_REPORT_URL = f"http://{SERVER_IP}:8000/dashboard/bot/report"
API_WEIGHT_URL = f"http://{SERVER_IP}:8000/cart/check_weight"

TAG_HEIGHT = 1.85
ANCHOR_Z = 2.37
H_DIFF = abs(ANCHOR_Z - TAG_HEIGHT)

# 고정 오프셋 보정값 
FIXED_OFFSETS = {0: -7.00, 1: -7.00, 2: -7.50}
# 앵커 절대 좌표 
ANCHORS_POS = {
    0: {'x': 0.00, 'y': 0.00},
    1: {'x': 6.00, 'y': 0.00},
    2: {'x': 3.00, 'y': 15.00}
}
MIN_UPDATE_DIST = 1.2
NAV_UPDATE_INTERVAL = 2.0

class IntegratedRosController(Node):
    def __init__(self):
        super().__init__('integrated_ros_controller_node')

        amcl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL 
        )

        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.battery_sub = self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose', 
            self.amcl_callback, 
            amcl_qos 
        )

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.sock_uwb = self.create_udp_socket(UDP_PORT_UWB)
        self.sock_qt = self.create_udp_socket(UDP_PORT_QT)

        self.current_mode = 0
        self.battery_level = 0.0
        self.current_speed = 0.0
        self.qt_addr = None

        self.robot_x = 0.0  
        self.robot_y = 0.0
        self.human_x = 0.0  
        self.human_y = 0.0

        self.x_his = deque(maxlen=15)
        self.y_his = deque(maxlen=5)
        self.uwb_dists = [0.0, 0.0, 0.0]
        self.uwb_last_update = [0.0, 0.0, 0.0]

        self.nav_goal_handle = None
        self.last_goal_time = 0.0
        self.last_target_pos = [0.0, 0.0]
        
        self.is_weight_ok = True  
        self.last_weight_check_time = 0.0
        self.weight_check_interval = 2.0 

        self.get_logger().info("Waiting for Nav2 Server...")
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 Server not ready yet.")

        self.create_timer(0.05, self.control_loop)
        self.create_timer(1.0, self.send_bot_status)

        self.get_logger().info("ROS Controller Started")

    def create_udp_socket(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', port))
        sock.setblocking(False)
        return sock

    # 무게 체크 
    def trigger_async_weight_check(self):
        def _check():
            try:
                response = requests.get(API_WEIGHT_URL, timeout=0.2) 
                if response.status_code == 200:
                    data = response.json()
                    movable = data.get("movable", True)
                    self.is_weight_ok = movable 
                    
                    if not movable:
                        self.send_udp_to_qt("ERROR:WEIGHT_MISMATCH")
            except: pass 
            
        threading.Thread(target=_check, daemon=True).start()

    def send_udp_to_qt(self, message):
        if self.qt_addr:
            try: self.sock_qt.sendto(message.encode(), self.qt_addr)
            except: pass

    def battery_callback(self, msg):
        self.battery_level = int(msg.percentage) if msg.percentage >= 0 else 0.0

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def amcl_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    # 로봇 상태 전송
    def send_bot_status(self):
        def _post():
            try:
                payload = {
                    "status": "ONLINE",
                    "mode": self.current_mode,
                    "battery": round(self.battery_level, 1),
                    "speed": round(self.current_speed, 2),
                    "x": self.robot_x,      
                    "y": self.robot_y,      
                    "human_x": self.human_x, 
                    "human_y": self.human_y 
                }
                requests.post(API_REPORT_URL, json=payload, timeout=0.1)
            except: pass
        
        # 스레드 비동기 전송 후 종료 
        threading.Thread(target=_post, daemon=True).start()

    # 삼변측량 함수 
    def calculate_trilateration(self, distances):
        try:
            x1, y1, r1 = ANCHORS_POS[0]['x'], ANCHORS_POS[0]['y'], distances[0]
            x2, y2, r2 = ANCHORS_POS[1]['x'], ANCHORS_POS[1]['y'], distances[1]
            x3, y3, r3 = ANCHORS_POS[2]['x'], ANCHORS_POS[2]['y'], distances[2]
            A, B = 2*(x2-x1), 2*(y2-y1)
            C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2
            D, E = 2*(x3-x2), 2*(y3-y2)
            F = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2
            den = (A * E) - (B * D)
            if abs(den) < 0.0001: return None
            return ((C * E) - (B * F)) / den, ((A * F) - (C * D)) / den
        except: return None

    def transform_to_map(self, ux, uy):
        mx = 3.25 + (ux * -0.9725)
        my = -7.35 + (uy * 0.5966)
        return mx, my

    # UWB 데이터 수신
    def receive_uwb_data(self):
        try:
            for _ in range(10): 
                data, _ = self.sock_uwb.recvfrom(BUFFER_SIZE)
                msg = data.decode('utf-8').strip()
                idx = -1
                if msg.startswith("A1:"): idx = 0
                elif msg.startswith("A2:"): idx = 1
                elif msg.startswith("A3:"): idx = 2
                
                if idx != -1:
                    raw_val = float(msg.split(":")[1])
                    corrected_dist = raw_val - FIXED_OFFSETS[idx]
                    h_dist = math.sqrt(corrected_dist**2 - H_DIFF**2) if corrected_dist > H_DIFF else 0.1
                    self.uwb_dists[idx] = h_dist
                    self.uwb_last_update[idx] = time.time()
        except BlockingIOError: pass
        except: pass

    def receive_qt_command(self):
        try:
            while True: 
                data, addr = self.sock_qt.recvfrom(BUFFER_SIZE)
                self.qt_addr = addr 
                text = data.decode('utf-8').strip()
                if text.startswith("MODE:"): self.change_mode(int(text.split(":")[1]))
                elif text.startswith("GOAL:"):
                    if self.current_mode != 2: self.change_mode(2)
                    p = text.split(":")[1].split(",")
                    self.start_navigation(float(p[0]), float(p[1]))
                elif text.startswith("STOP"): self.change_mode(0)
        except BlockingIOError: pass
        except: pass

    def control_loop(self):
        self.receive_uwb_data()
        self.receive_qt_command()
        
        now = time.time()

        if now - self.last_weight_check_time > self.weight_check_interval:
            self.trigger_async_weight_check()
            self.last_weight_check_time = now

        if all(now - self.uwb_last_update[i] < 1.0 for i in range(3)):
            pos = self.calculate_trilateration(self.uwb_dists)
            if pos:
                raw_x = max(0, min(pos[0], 6.0))
                raw_y = max(0, min(pos[1], 15.0))
                self.x_his.append(raw_x)
                self.y_his.append(raw_y)
                avg_x = sum(self.x_his) / len(self.x_his)
                avg_y = sum(self.y_his) / len(self.y_his)
                
                final_uwb_x = (avg_x / 6.0) * (3.80 - 0.10) + 0.10
                final_uwb_y = avg_y
                
                mx, my = self.transform_to_map(final_uwb_x, final_uwb_y)
                self.human_x = mx
                self.human_y = my

                if self.current_mode == 1:
                    dist_diff = math.sqrt((mx - self.last_target_pos[0])**2 + (my - self.last_target_pos[1])**2)
                    
                    if self.is_weight_ok:
                        if (now - self.last_goal_time > NAV_UPDATE_INTERVAL) and (dist_diff > MIN_UPDATE_DIST):
                            self.get_logger().info(f"Target Moved -> Goal: ({mx:.2f}, {my:.2f})")
                            self.start_navigation(mx, my)
                            self.last_goal_time = now
                            self.last_target_pos = [mx, my]
        
        if self.current_mode == 0:
            if self.nav_goal_handle: self.cancel_navigation()
            self.stop_robot()

    def change_mode(self, new_mode):
        if self.current_mode == new_mode: return
        self.get_logger().info(f"Mode Switching: {self.current_mode} -> {new_mode}")
        self.cancel_navigation()
        self.stop_robot()
        self.current_mode = new_mode
        self.last_target_pos = [0.0, 0.0]
        self.trigger_async_weight_check()

    # 네비게이션 실행
    def start_navigation(self, x, y):
        if not self.is_weight_ok: return

        if not self.nav_to_pose_client.server_is_ready():
            self.get_logger().error("Nav2 Server not ready!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if handle.accepted:
            self.nav_goal_handle = handle
            self._get_result_future = handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == 4:
            self.send_udp_to_qt("STATUS:ARRIVED")
        self.nav_goal_handle = None 

    def cancel_navigation(self):
        if self.nav_goal_handle:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None

    def stop_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.cmd_vel_pub.publish(msg)

def main():
    rclpy.init()
    node = IntegratedRosController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()