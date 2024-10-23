import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TCPServerROS2(Node):
    def __init__(self):
        super().__init__('tcp_server')
        self.publisher_ = self.create_publisher(String, '/traffic_results', 10)
        self.server_socket = None

    def setup_tcp_server(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = ('127.0.0.1', 2345)
        self.server_socket.bind(server_address)
        self.server_socket.listen(1)
        self.get_logger().info("Waiting for a connection...")
        connection, client_address = self.server_socket.accept()
        self.get_logger().info(f"Connection from {client_address}")

        try:
            while True:
                data = connection.recv(1024)
                if data:
                    received_str = data.decode()
                    self.get_logger().info(f"Received: {received_str}")
                    msg = String()
                    msg.data = received_str
                    self.publisher_.publish(msg)
                else:
                    break
        finally:
            connection.close()
            self.get_logger().info("Connection closed")

def main(args=None):
    rclpy.init(args=args)
    tcp_server_node = TCPServerROS2()
    tcp_server_node.setup_tcp_server()
    rclpy.spin(tcp_server_node)
    tcp_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
