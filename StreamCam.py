import rclpy
from rclpy.node import Node
import socket
import cv2
import struct

class StreamCam(Node):
    def __init__(self):
        super().__init__('stream_cam')

        self.declare_parameter('host', '0.0.0.0') 
        self.declare_parameter('portV', 5005)  
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        self.host = self.get_parameter('host').value
        self.portV = self.get_parameter('portV').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value


    def start_server(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.portV))
        self.server_socket.listen()

        self.get_logger().info(f"TCP 서버 시작: {self.host}:{self.portV}")
        
        self.client_socket, addr = self.server_socket.accept()
        self.get_logger().info(f"클라이언트 연결됨: {addr}")

        self.start_streaming()

    
    def start_streaming(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다!")
            return

        self.get_logger().info("웹캠 스트리밍 시작")

        frame_count = 0
        try:
            while rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    break

                # JPEG 압축 후 바이트 변환
                _, img_encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
                data = img_encoded.tobytes()
                size = len(data)
                frame_count += 1

                # 30프레임마다 로그 출력
                if frame_count % 30 == 0:
                    self.get_logger().info(f"프레임 {frame_count} 전송: {size} bytes")

                # 데이터 크기와 이미지 전송
                self.client_socket.sendall(struct.pack(">L", size))
                self.client_socket.sendall(data)
                
        except (ConnectionResetError, ConnectionAbortedError, OSError):
            self.get_logger().warn("클라이언트 연결 종료됨. 서버 재시작 중...")
            self.client_socket.close()
            cap.release()
            self.start_server()  # 클라이언트 재연결 대기

        cap.release()
        self.client_socket.close()
        self.server_socket.close()



def main(args=None):
    rclpy.init(args=args)
    stream_cam = StreamCam()
    try:
        stream_cam.start_server()
    except KeyboardInterrupt:
        pass
    finally:
        stream_cam.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()