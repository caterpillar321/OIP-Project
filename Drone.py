import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
import time
import math 
from rclpy.qos import qos_profile_sensor_data

class Drone(Node):
    def __init__(self):
        super().__init__('drone_ros_node')
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.ck = 0
        self.count = 0

        # ROS2 퍼블리셔 생성
        self.velocity_publisher = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.get_logger().info("Velocity publisher initialized.")

        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile_sensor_data
        )

        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')  # 착륙 서비스 클라이언트 추가
        # 서비스가 실행될 때까지 대기
        while not self.set_mode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')
        while not self.arming_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming service...')
        while not self.land_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('Waiting for /mavros/cmd/land service...')

    def pose_callback(self, msg):
        q = msg.pose.orientation
    
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.yaw = yaw
        
        self.count += 1
        if self.count % 50 == 0 :
            self.get_logger().info(f"Calculated yaw: {yaw} radians")
    
    def move_drone(self, x, y, z, yaw_rate):
        # body frame -> 월드(고정) 좌표계 변환
        x_global = x * math.cos(self.yaw) - y * math.sin(self.yaw)  # modified
        y_global = x * math.sin(self.yaw) + y * math.cos(self.yaw)  # modified

        #self.get_logger().info(f"Moving Drone: x_global={x_global}, y_global={y_global}, z={z}, yaw_rate={yaw_rate}")
        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = self.get_clock().now().to_msg()
        velocity_msg.twist.linear.x = x_global  # modified
        velocity_msg.twist.linear.y = y_global  # modified
        velocity_msg.twist.linear.z = z
        velocity_msg.twist.angular.z = yaw_rate
        self.ck += 1

        self.velocity_publisher.publish(velocity_msg)

        if self.ck == 5:
            self.enable_offboard_mode()
            self.arm_drone()
    
    def operate(self, data):
        x = data.get('x', 0.0)
        y = data.get('y', 0.0)
        z = data.get('z', 0.0)
        yaw = data.get('yaw', 0.0)
        self.move_drone(x, y, z, yaw)

    def enable_offboard_mode(self):
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = "OFFBOARD"

        # call_async로 비동기 요청
        future = self.set_mode_client.call_async(request)
        # 응답이 오면 offboard_callback 실행
        future.add_done_callback(self.offboard_callback)

    def offboard_callback(self, future):
        try:
            result = future.result()
            if result is not None and result.mode_sent:
                self.get_logger().info("OFFBOARD 모드로 변경됨!")
            else:
                self.get_logger().error("OFFBOARD 모드 변경 실패!")
        except Exception as e:
            self.get_logger().error(f"OFFBOARD 모드 요청 중 예외 발생: {str(e)}")

    def arm_drone(self):
        request = CommandBool.Request()
        request.value = True

        # 마찬가지로 비동기로 요청
        future = self.arming_client.call_async(request)
        future.add_done_callback(self.arm_callback)

    def arm_callback(self, future):
        try:
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info("드론이 Arming 됨!")
            else:
                self.get_logger().error("드론 Arming 실패!")
        except Exception as e:
            self.get_logger().error(f"드론 Arming 요청 중 예외 발생: {str(e)}")


    def land_drone(self):
        """드론 착륙 요청"""
        self.get_logger().info("Landing Drone...")
        request = CommandTOL.Request()

        # 착륙 서비스 요청
        future = self.land_client.call_async(request)
        future.add_done_callback(self.land_callback)

    def land_callback(self, future):
        try:
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info("드론이 착륙 중...")
            else:
                self.get_logger().error("드론 착륙 실패!")
        except Exception as e:
            self.get_logger().error(f"드론 착륙 요청 중 예외 발생: {str(e)}")
