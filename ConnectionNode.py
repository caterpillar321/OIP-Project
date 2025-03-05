import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

import socket
import time
import struct
import json
import threading
import os
import signal
import subprocess


class ConnectionNode(Node):
    def __init__(self, host="0.0.0.0"):
        super().__init__('connection_node')
        self.host = host

        self.sock = None
        self.sock_info = None
        self.is_connected = False
        
        self.connection_event = threading.Event()
        
        self.cmd_publisher = self.create_publisher(Twist, 'drone_cmd', 10)
        self.land_publisher = self.create_publisher(String, 'drone_land', 10)

        self.drone_info_subscriber = self.create_subscription(
            Vector3,
            'drone_info',
            self.drone_info_callback,
            10
        )

        threading.Thread(target=self.start_server, daemon=True).start()
        threading.Thread(target=self.recv_data, daemon=True).start()

        
    def start_server(self):
        self.get_logger().info("Starting server...")

        self.kill_process_on_port(5001)
        self.kill_process_on_port(5010)

        tsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tsock.bind((self.host, 5001))
        tsock.listen(1)

        tsock_info = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tsock_info.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tsock_info.bind((self.host, 5010))
        tsock_info.listen(1)

        threading.Thread(target=self.accept_connection, args=(tsock, 'sock'), daemon=True).start()
        threading.Thread(target=self.accept_connection, args=(tsock_info, 'sock_info'), daemon=True).start()

        if not self.connection_event.wait(timeout=20):
            self.get_logger().error("서버 연결 타임아웃")
        else:
            self.get_logger().info("Server connection established")

        tsock.close()
        tsock_info.close()


    def accept_connection(self, sock, attr_name):
        conn, addr = sock.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        setattr(self, attr_name, conn)
        self.get_logger().info(f"Connection established on {attr_name} with {addr}")

        if self.sock is not None and self.sock_info is not None:
            self.is_connected = True
            self.connection_event.set()


    def kill_process_on_port(self, port):
        try:
            result = subprocess.check_output(["lsof", "-t", f"-i:{port}"])
            pids = result.decode().strip().split("\n")
            for pid in pids:
                try:
                    os.kill(int(pid), signal.SIGKILL)
                    print(f"Process {pid} on port {port} has been terminated.")
                except OSError as e:
                    print(f"Error terminating process {pid}: {e}")
        except subprocess.CalledProcessError:
            pass 


    def recvall(self, sock, n):
        data = b''
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

    def recv_data(self):
        self.connection_event.wait()

        if not self.sock or not self.is_connected:
            self.get_logger().error("Error: Connection not established in recv_data()")
            return

        ck = 0
        while self.is_connected:
            try:
                sizebuffer = self.recvall(self.sock, 4)
                if not sizebuffer:
                    break
                size = struct.unpack('>I', sizebuffer)[0]
                message_bytes = self.recvall(self.sock, size)
                if not message_bytes:
                    break
                
                buffer = message_bytes.decode()
                data = json.loads(buffer)

                ck += 1
                if ck % 3 == 0:
                    self.get_logger().info(f"Received data: {data}")

                if data.get("command") == "land":
                    self.get_logger().info("Landing command received, publishing to drone_land")
                    land_msg = String()
                    land_msg.data = "land"
                    self.land_publisher.publish(land_msg)
                    continue 

                if data.get("config") == "stream":
                    self.get_logger().info("change streaming config")
                    if data.get("option") == "360":
                        self.change_camera_params(640, 360, 30, 5005)
                    elif data.get("option") == "480":
                        self.change_camera_params(850, 480, 30, 5005)
                    elif data.get("option") == "720":
                        self.change_camera_params(1280, 720, 30, 5005)

                twist_msg = Twist()
                twist_msg.linear.x = data.get('x', 0.0)
                twist_msg.linear.y = data.get('y', 0.0)
                twist_msg.linear.z = data.get('z', 0.0)
                twist_msg.angular.z = data.get('yaw', 0.0) 
                
                self.cmd_publisher.publish(twist_msg)

                time.sleep(0.07)

            except Exception as e:
                self.get_logger().error(f"Error in recv_data: {e}")
                self.is_connected = False
                break

    def drone_info_callback(self, msg):
        if not self.is_connected or not self.sock_info:
            return
        
        try:
            roll = msg.x
            pitch = msg.y
            yaw = msg.z

            roll = roll if roll >= 0 else roll + 360
            pitch = pitch if pitch >= 0 else pitch + 360
            yaw = yaw if yaw >= 0 else yaw + 360

            message = json.dumps({
                "roll": f"{roll:03.0f}",
                "pitch": f"{pitch:03.0f}",
                "yaw": f"{yaw:03.0f}",
            })

            size = len(message.encode())
            if self.sock_info.fileno() == -1:
                return

            self.sock_info.sendall(struct.pack(">L", size))
            self.sock_info.sendall(message.encode())

        except Exception as e:
            self.get_logger().error(f"Error in drone_info_callback: {e}")
            self.is_connected = False


    def change_camera_params(self, width, height, fps, portV):
        """StreamCam 노드의 파라미터를 변경하는 함수"""
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name="width", value=ParameterValue(type=Parameter.Type.INTEGER, integer_value=width)),
            Parameter(name="height", value=ParameterValue(type=Parameter.Type.INTEGER, integer_value=height)),
            Parameter(name="fps", value=ParameterValue(type=Parameter.Type.INTEGER, integer_value=fps)),
            Parameter(name="portV", value=ParameterValue(type=Parameter.Type.INTEGER, integer_value=portV))
        ]

        # 비동기 요청
        future = self.set_param_client.call_async(request)
        future.add_done_callback(self.param_change_callback)

    def param_change_callback(self, future):
        """파라미터 변경 요청 후 결과 처리"""
        try:
            response = future.result()
            self.get_logger().info("Camera parameters updated successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to update camera parameters: {e}")


def main(args=None):
    rclpy.init(args=args)
    connection_node = ConnectionNode()
    try:
        rclpy.spin(connection_node)
    except KeyboardInterrupt:
        pass
    finally:
        connection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
