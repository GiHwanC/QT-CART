import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped 
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import BatteryState  
from nav_msgs.msg import Odometry         
import socket
import requests
import threading 

UDP_PORT_QT = 55555
UDP_PORT_UWB = 44444
BUFFER_SIZE = 1024

API_URL = "http://localhost:8000/dashboard/bot/report"

class RosController(Node):
    def __init__(self):
        super().__init__('ros_controller_node')
        
        # TwistStamped 타입 사용
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # 실제 배터리 정보 구독
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )

        # 실제 주행 속도 확인을 위한 오도메트리 구독
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 네비게이션 액션 클라이언트 생성
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Nav2 액션 클라이언트
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # UDP 소켓
        self.sock_qt = self.create_udp_socket(UDP_PORT_QT)
        self.sock_uwb = self.create_udp_socket(UDP_PORT_UWB)

        # 상태 변수 
        self.current_mode = 2
        self.uwb_L = 0.0
        self.uwb_R = 0.0
        self.nav_goal_handle = None # 현재 진행 중인 네비게이션 핸들

        # 센서 데이터 저장 변수
        self.battery_level = 0.0
        self.current_speed = 0.0

        #  20Hz (0.05s)
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("ROS Controller Started!")

        self.create_timer(1.0, self.send_bot_status) 

    # 배터리 상태 콜백 함수
    def battery_callback(self, msg):
        if msg.percentage >= 0:
            self.battery_level = int(msg.percentage)

    # 오도메트리(속도) 콜백 함수
    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def trigger_status_thread(self):
        thread = threading.Thread(target=self.send_bot_status)
        thread.daemon = True # 프로그램 종료 시 같이 종료됨
        thread.start()

    def send_bot_status(self):
        try:
            # 실제 수신한 배터리와 속도 데이터를 전송
            payload = {
                "status": "ONLINE",
                "battery": round(self.battery_level, 1),
                "speed": round(self.current_speed, 2)
            }
            requests.post(API_URL, json=payload, timeout=0.2)
        except:
            pass

    def create_udp_socket(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', port))
        sock.setblocking(False) 
        return sock

    # --- 메인 제어 루프 ---
    def control_loop(self):
        self.receive_uwb_data()
        self.receive_qt_command()
        
        # print(f"\rL: {self.uwb_L:>5.2f}m | R: {self.uwb_R:>5.2f}m   ", end='', flush=True)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link' 
        
        if self.current_mode == 0:
            # 대기 모드: 정지 명령 지속 발행 (밀림 방지)
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            
        elif self.current_mode == 1:   # 안내 모드 (UWB 추종)
            # 자율주행 중이었다면 취소
            if self.nav_goal_handle:
                self.cancel_navigation()
                
            self.process_follow_mode(msg) # msg 내부 데이터를 채움
            self.cmd_vel_pub.publish(msg) # 발행
            
        elif self.current_mode == 2:   # 자율주행 모드 (Nav2)
            pass

    def filter_uwb_value(self, current_val, new_val):
        if new_val <= 0.0: return current_val
        # uwb 값이 0이거나 이전 값과 3m 이상 차이가 나면 직전에 온 정상값으로 대체
        if current_val != 0.0 and abs(new_val - current_val) > 3.0: return current_val 
        
        # 지수이동평균(LPF): 이전값 30% + 새로운 값 70%으로 부드럽게 필터링
        alpha = 0.7
        return (current_val * (1.0 - alpha)) + (new_val * alpha)

    def receive_uwb_data(self):
        try:
            while True:
                data, _ = self.sock_uwb.recvfrom(BUFFER_SIZE)
                text = data.decode('utf-8').strip()
                if text.startswith("L:"):
                    self.uwb_L = self.filter_uwb_value(self.uwb_L, float(text[2:]))
                elif text.startswith("R:"):
                    self.uwb_R = self.filter_uwb_value(self.uwb_R, float(text[2:]))
        except (BlockingIOError, socket.error, ValueError): pass

    def receive_qt_command(self):
        try:
            while True:
                data, _ = self.sock_qt.recvfrom(BUFFER_SIZE)
                text = data.decode('utf-8').strip()
                
                # 모드 변경 명령
                if text.startswith("MODE:"):
                    self.change_mode(int(text.split(":")[1]))
                
                # 네비게이션 목표 명령 (예: GOAL:1.5,2.0)
                elif text.startswith("GOAL:"):
                    parts = text.split(":")[1].split(",")
                    self.start_navigation(float(parts[0]), float(parts[1]))
                    
        except (BlockingIOError, socket.error): pass

    def change_mode(self, new_mode):
        if self.current_mode == new_mode: return
        self.get_logger().info(f"Mode Change: {self.current_mode} -> {new_mode}")
        
        # 모드가 바뀌면 기존 네비게이션은 취소
        if self.nav_goal_handle:
            self.cancel_navigation()
            
        self.current_mode = new_mode

    # 네비게이션 Action Client
    def start_navigation(self, x, y):
        if self.current_mode != 2: 
            self.current_mode = 2
            
        # Nav2 서버 연결 확인
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 Action Server not available!")
            return

        # 목표 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0 # 정면 보기

        self.get_logger().info(f"Sending Goal: ({x}, {y})")

        # 액션 전송 
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # 목표 수신 콜백
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.nav_goal_handle = goal_handle
        
        # 결과 기다리기
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # 최종 도착 콜백
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation Result: {result}')
        self.nav_goal_handle = None # 도착 후 핸들 초기화

    # 네비게이션 취소 
    def cancel_navigation(self):
        if self.nav_goal_handle:
            self.get_logger().info('Canceling navigation')
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None

    def process_follow_mode(self, msg):
        l, r = self.uwb_L, self.uwb_R

        if l <= 0.01 or r <= 0.01:
            self.cmd_vel_pub.publish(msg); return
        
        avg = (l + r) / 2.0
        diff = r - l 
        is_stopping = False
        if avg > 1.2: msg.twist.linear.x = -0.18 
        elif avg < 0.8:
            msg.twist.linear.x = 0.0 
            is_stopping = True
        
        if is_stopping:
            pass 
        else:
            if abs(diff) > 0.05: msg.twist.angular.z = -(diff * 4.0) 
        
        self.cmd_vel_pub.publish(msg)

def main():
    rclpy.init()
    node = RosController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()