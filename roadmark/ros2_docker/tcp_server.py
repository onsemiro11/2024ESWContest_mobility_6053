import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json 

class TCPServerROS2_Mark(Node):
    def __init__(self):
        super().__init__('tcp_server_mark')
        self.publisher_r = self.create_publisher(String, '/road_results', 10)
        self.publisher_v = self.create_publisher(String, '/vehicle_results', 10)
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
                    self.get_logger().info(f"Received: {received_str.split('|')}")
                    received_str = received_str.split('|')
                    msg1 = String()
                    msg1.data = received_str[0]
                    msg2 = String()
                    msg2.data = received_str[1]
                    self.publisher_r.publish(msg1)
                    self.publisher_v.publish(msg2)
                else:
                    break
        finally:
            connection.close()
            self.get_logger().info("Connection closed")

def main(args=None):
    rclpy.init(args=args)
    tcp_server_node = TCPServerROS2_Mark()
    tcp_server_node.setup_tcp_server()
    rclpy.spin(tcp_server_node)
    tcp_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
