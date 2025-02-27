import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
import time

class PX4OffboardHover(Node):
    def __init__(self):
        super().__init__('px4_offboard_hover')

        # MAVROS 상태 구독
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # QoS 설정 추가 (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # self.position_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.position_callback, qos_profile)
        self.position_sub = self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self.position_callback, qos_profile)


        # 속도 명령 퍼블리셔
        self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # 서비스 클라이언트 (Arming 및 Offboard 모드 설정)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_state = State()
        self.target_altitude = 1.2  # 목표 고도
        self.max_speed = 0.5  # 최대 상승 속도 (m/s)
        self.offboard_enabled = False
        self.armed = False
        self.takeoff_complete = False
        self.current_altitude = 0.0

        # 주기적인 명령 전송 타이머 설정
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        self.get_logger().info("PX4 Offboard Hover Node Initialized")

    def state_callback(self, msg):
        """ MAVROS 상태를 업데이트하는 콜백 함수 """
        self.current_state = msg

    def position_callback(self, msg):
        """ 현재 드론의 고도(Z) 값 업데이트 """
        self.current_altitude = msg.pose.position.z
        # self.get_logger().info(f"Received PoseStamped: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")

    def control_loop(self):
        """ Offboard 모드, 이륙, 호버링을 수행하는 메인 루프 """

        if not self.current_state.connected:
            self.get_logger().warn("Waiting for FCU connection...")
            return

        # Offboard 모드 설정
        if not self.offboard_enabled and self.current_state.mode != "OFFBOARD":
            self.set_offboard_mode()

        # Arm 명령 전송
        if not self.armed and not self.current_state.armed:
            self.arm()

        self.takeoff()
        

    def set_offboard_mode(self):
        """ Offboard 모드 활성화 """
        if self.set_mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = "OFFBOARD"
            future = self.set_mode_client.call_async(req)
            future.add_done_callback(self.offboard_mode_response)
        else:
            self.get_logger().warn("SetMode service not available")

    def offboard_mode_response(self, future):
        """ Offboard 모드 전환 후 상태 확인 """
        try:
            response = future.result()
            if response.mode_sent:
                self.offboard_enabled = True
                self.get_logger().info("Offboard mode enabled")
            else:
                self.get_logger().warn("Failed to set Offboard mode")
        except Exception as e:
            self.get_logger().error(f"SetMode call failed: {e}")

    def arm(self):
        """ Arm 명령 전송 """
        if self.arming_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            future.add_done_callback(self.arm_response)
        else:
            self.get_logger().warn("Arming service not available")

    def arm_response(self, future):
        """ Arm 명령 전송 후 상태 확인 """
        try:
            response = future.result()
            if response.success:
                self.armed = True
                self.get_logger().info("Vehicle armed")
            else:
                self.get_logger().warn("Failed to arm vehicle")
        except Exception as e:
            self.get_logger().error(f"Arming call failed: {e}")

    def takeoff(self):
        """ 속도 명령을 사용하여 고도 1.5m까지 상승 """
        vel_cmd = TwistStamped()
        vel_cmd.header.stamp = self.get_clock().now().to_msg()

        
        altitude_error = self.target_altitude - self.current_altitude  # 현재 고도와 목표 고도의 차이
        speed = self.max_speed * altitude_error# 거리 비율 기반 속도 계산
        speed =min(speed, self.max_speed) # 최소 0.1m/s 이상, 최대 max_speed 이하

        vel_cmd.twist.linear.z = speed

        self.vel_pub.publish(vel_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PX4OffboardHover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
