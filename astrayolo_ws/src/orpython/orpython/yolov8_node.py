# yolov8_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLOv8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')
        
        # YOLOv8 모델 로드 (사전 학습된 모델 사용)
        self.model = YOLO('yolov8n.pt')  # 모델 파일 경로를 넣어주세요.

        # 이미지 구독자 설정 (raw 이미지 구독)
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # 토픽 이름을 맞게 수정하세요.
            self.image_callback,
            10
        )

        # OpenCV와 ROS 2 간의 이미지 변환을 위한 CvBridge 객체
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # ROS Image 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # YOLOv8 모델로 객체 감지
        results = self.model(cv_image)

        # 결과 이미지 생성
        result_image = results[0].plot()

        # 결과 이미지를 화면에 출력
        cv2.imshow("YOLOv8 Detection", result_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    yolov8_node = YOLOv8Node()
    rclpy.spin(yolov8_node)
    yolov8_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

