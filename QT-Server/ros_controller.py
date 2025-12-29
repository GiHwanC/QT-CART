import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped # TwistStamped 추가
from nav2_simple_commander.robot_navigator import BasicNavigator
import socket

UDP_PORT_QT = 55555
UDP_PORT_UWB = 44444
BUFFER_SIZE = 1024

class RosController(Node):
    def __init__(self):
        super().__init__('ros_controller_node')
        
        # ROS 2 퍼블리셔 
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.navigator = BasicNavigator()
        
        # UDP 소켓 설정
        self.sock_qt = self.create_udp_socket(UDP_PORT_QT)
        self.sock_uwb = self.create_udp_socket(UDP_PORT_UWB)

        # 상태 변수
        self.current_mode = 0
        self.uwb_L = 0.0
        self.uwb_R = 0.0

        # 20Hz 주기로 제어 루프 실행
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"ROS Controller Started! (TwistStamped Mode)")

    def create_udp_socket(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', port))
        sock.setblocking(False) 
        return sock

    def control_loop(self):
        self.receive_uwb_data()
        self.receive_qt_command()
        
        # 발행할 메시지 틀 생성 
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg() # 현재 시간 필수
        msg.header.frame_id = 'base_link'
        
        if self.current_mode == 0:     # 정지
            self.cmd_vel_pub.publish(msg)
            
        elif self.current_mode == 1:   # 따라가기
            self.process_follow_mode(msg)
            
        elif self.current_mode == 2:   # 자율주행
            if self.navigator.isTaskComplete(): pass

    def receive_uwb_data(self):
        try:
            while True:
                data, _ = self.sock_uwb.recvfrom(BUFFER_SIZE)
                text = data.decode('utf-8').strip()
                if text.startswith("L:"): self.uwb_L = float(text[2:])
                elif text.startswith("R:"): self.uwb_R = float(text[2:])
        except (BlockingIOError, socket.error): pass

    def receive_qt_command(self):
        try:
            while True:
                data, _ = self.sock_qt.recvfrom(BUFFER_SIZE)
                text = data.decode('utf-8').strip()
                if text.startswith("MODE:"):
                    self.change_mode(int(text.split(":")[1]))
                elif text.startswith("GOAL:"):
                    parts = text.split(":")[1].split(",")
                    self.start_navigation(float(parts[0]), float(parts[1]))
        except (BlockingIOError, socket.error): pass

    def change_mode(self, new_mode):
        if self.current_mode == new_mode: return
        self.get_logger().info(f"Mode Change: {self.current_mode} -> {new_mode}")
        if self.current_mode == 2 and new_mode != 2:
            self.navigator.cancelTask()
        self.current_mode = new_mode

    def process_follow_mode(self, msg):
        l, r = self.uwb_L, self.uwb_R
        
        # 센서 데이터가 유효하지 않으면 정지
        if l <= 0.01 or r <= 0.01:
            self.cmd_vel_pub.publish(msg)
            return
        
        avg = (l + r) / 2.0
        diff = r - l  # r이 크면 사용자가 왼쪽에 가까이 있음 (l이 작으므로)
        
        if abs(diff) > 0.05: 
            msg.twist.angular.z = -(diff * 2.5) 

        # 2. 전진/후진 제어 (역방향)
        if avg > 1.2:          # 사용자가 1.2m보다 멀어지면
            msg.twist.linear.x = -0.2 # 뒷걸음질로 따라가기
        elif avg < 1.0:        # 정지거리
            msg.twist.linear.x = 0.0   
        
        self.cmd_vel_pub.publish(msg)

    def start_navigation(self, x, y):
        if self.current_mode != 2: self.change_mode(2)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.navigator.goToPose(goal)

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