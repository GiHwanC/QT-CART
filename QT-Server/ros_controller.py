import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped 
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

        self.current_mode = 0
        self.uwb_L = 0.0
        self.uwb_R = 0.0

        # 20Hz 주기로 제어 루프 실행
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"ROS Controller Started!")

    def create_udp_socket(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', port))
        sock.setblocking(False) 
        return sock

    def control_loop(self):
        self.receive_uwb_data()
        self.receive_qt_command()
        
        # 한 줄 로그 출력
        print(f"L: {self.uwb_L:>5.2f}m | R: {self.uwb_R:>5.2f}m", end='', flush=True)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        if self.current_mode == 0:
            self.cmd_vel_pub.publish(msg)
        elif self.current_mode == 1:
            self.process_follow_mode(msg)
        elif self.current_mode == 2:
            if self.navigator.isTaskComplete(): pass

    # uwb 값이 0이거나 이전 값과 3m 이상 차이가 나면 직전에 온 정상값으로 대체 
    def filter_uwb_value(self, current_val, new_val):
        if new_val <= 0.0:
            return current_val
        
        if current_val != 0.0 and abs(new_val - current_val) > 3.0:
            return current_val 
            
        # 지수이동평균(LPF): 이전값 30% + 새로운 값 70%으로 부드럽게 필터링
        alpha = 0.7
        return (current_val * (1.0 - alpha)) + (new_val * alpha)

    def receive_uwb_data(self):
        try:
            while True:
                data, _ = self.sock_uwb.recvfrom(BUFFER_SIZE)
                text = data.decode('utf-8').strip()
                
                if text.startswith("L:"):
                    raw_val = float(text[2:])
                    self.uwb_L = self.filter_uwb_value(self.uwb_L, raw_val)
                elif text.startswith("R:"):
                    raw_val = float(text[2:])
                    self.uwb_R = self.filter_uwb_value(self.uwb_R, raw_val)
                    
        except (BlockingIOError, socket.error, ValueError):
            pass

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
        
        if l <= 0.01 or r <= 0.01:
            self.cmd_vel_pub.publish(msg)
            return
        
        avg = (l + r) / 2.0
        diff = r - l 
        
        # 전진/후진 제어 (역방향)
        is_stopping = False
        if avg > 1.2:
            msg.twist.linear.x = -0.18   # 최대 속도는 0.22지만 부드러운 방향전환을 위해 
        elif avg < 0.8:
            msg.twist.linear.x = 0.0 
            is_stopping = True
        
        # 방향 제어
        if is_stopping:
            self.stop_rotate_counter += 1
            # 멈춤 상태에서는 발작 방지를 위해 5번 중 1번만 회전 명령
            if self.stop_rotate_counter % 5 == 0:
                if abs(diff) > 0.05: 
                    msg.twist.angular.z = -(diff * 4.0) 
                else:
                    msg.twist.angular.z = 0.0
            else:
                msg.twist.angular.z = 0.0
        else:
            self.stop_rotate_counter = 0
            if abs(diff) > 0.05: 
                msg.twist.angular.z = -(diff * 4.0) 
        
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