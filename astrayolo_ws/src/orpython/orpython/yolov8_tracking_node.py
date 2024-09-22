import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from collections import defaultdict
from orros.msg import YoloBox, YoloBoxes


# 클래스 이름 리스트 (YOLOv8에서 사용하는 클래스명 목록)
CLASS_NAMES = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", 
               "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter",
               "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", 
               "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", 
               "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", 
               "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", 
               "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", 
               "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", 
               "cake", "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", 
               "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", 
               "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", 
               "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]

class YOLOv8TrackingNode(Node):
    def __init__(self):
        super().__init__('yolov8_tracking_node')

        # YOLOv8 모델 로드 및 GPU로 이동
        self.model = YOLO('yolov8n.pt').to('cuda')

        # OpenCV와 ROS 2 간의 이미지 변환을 위한 CvBridge 객체
        self.bridge = CvBridge()

        # 이미지 구독자 설정 (컬러 이미지)
        self.color_subscriber = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_image_callback, 10)

        # 이미지 구독자 설정 (깊이 이미지)
        self.depth_subscriber = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_image_callback, 10)

        # YoloBox 메시지 퍼블리셔 생성
        self.box_publisher = self.create_publisher(YoloBoxes, 'yolo_boxes', 10)

        # 깊이 이미지 초기화
        self.depth_image = None

        # ID 매핑 테이블과 누적 ID 카운터 초기화
        self.id_mapping = {}
        self.object_id_counter = 1

    def color_image_callback(self, msg):
        # ROS Image 메시지를 OpenCV 이미지로 변환 (컬러 이미지)
        color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # YOLOv8 모델로 객체 감지 및 추적 수행 (GPU 사용)
        results = self.model.track(source=color_image, device='cuda', tracker="bytetrack.yaml")

        # 감지된 객체 정보를 저장할 YoloBoxes 메시지 준비
        yolo_boxes_msg = YoloBoxes()

        # 감지된 객체들의 바운딩 박스 및 클래스 이름 처리
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()  # 바운딩 박스 좌표 [xmin, ymin, xmax, ymax]
            class_ids = result.boxes.cls.cpu().numpy()  # 각 객체의 클래스 ID
            ids = result.boxes.id.int().cpu().numpy() if result.boxes.id is not None else [None] * len(boxes)

            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = map(int, box)
                track_id = int(ids[i])if ids[i] is not None else -1

                # 클래스 ID 및 이름 가져오기
                class_id = int(class_ids[i])
                class_name = CLASS_NAMES[class_id]

                # YoloBox 메시지 생성
                yolo_box = YoloBox()
                yolo_box.id = track_id
                yolo_box.class_name = class_name
                yolo_box.xmin = x1
                yolo_box.ymin = y1
                yolo_box.xmax = x2
                yolo_box.ymax = y2

                # 깊이 이미지에서 깊이 값을 가져오기 (중앙 지점 기준)
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                if self.depth_image is not None:
                    depth_value = self.depth_image[center_y, center_x]
                    yolo_box.depth = float(depth_value) if depth_value > 0 else -1.0
                else:
                    yolo_box.depth = -1.0

                # yolo_box 메시지를 yolo_boxes_msg에 추가
                yolo_boxes_msg.boxes.append(yolo_box)

                # 이미지에 객체 정보 표시
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'{class_name} ID: {track_id}, Depth: {yolo_box.depth:.2f}m'
                cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # 감지된 객체 정보 퍼블리시
        self.box_publisher.publish(yolo_boxes_msg)

        # 결과 이미지 출력
        cv2.imshow("YOLOv8 Tracking and Depth", color_image)
        cv2.waitKey(1)

    def depth_image_callback(self, msg):
        # ROS Image 메시지를 OpenCV 이미지로 변환 (깊이 이미지)
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        self.depth_image = self.depth_image.astype(np.float32) / 1000.0  # 밀리미터를 미터로 변환

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8TrackingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
