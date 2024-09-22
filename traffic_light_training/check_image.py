import os
import sys
import cv2
import numpy as np
import random

def draw_boxes(image_path, label_path, class_names):
    # 이미지 읽기
    image = cv2.imread(image_path)
    height, width, _ = image.shape

    # 라벨 파일 읽기
    with open(label_path, 'r') as f:
        lines = f.readlines()

    for line in lines:
        class_id, x_center, y_center, box_width, box_height = map(float, line.strip().split())
        
        # YOLO 형식을 픽셀 좌표로 변환
        x_center *= width
        y_center *= height
        box_width *= width
        box_height *= height

        x1 = int(x_center - box_width / 2)
        y1 = int(y_center - box_height / 2)
        x2 = int(x_center + box_width / 2)
        y2 = int(y_center + box_height / 2)

        # 바운딩 박스 그리기
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 클래스 이름 표시
        class_name = class_names[int(class_id)]
        cv2.putText(image, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    return image

def main(dataset_type):
    base_path = f'/home/hyundo/traffic_light/{dataset_type}'
    images_path = os.path.join(base_path, 'images')
    labels_path = os.path.join(base_path, 'labels')

    class_names =['veh_go','veh_goLeft','veh_noSign','veh_stop','veh_stopLeft','veh_stopWarning','veh_warning']
    # 이미지 파일 목록 가져오기
    image_files = [f for f in os.listdir(images_path) if f.endswith('.jpg')]

    # 임의의 이미지 선택
    random_image = random.choice(image_files)
    image_path = os.path.join(images_path, random_image)
    label_file = os.path.splitext(random_image)[0] + '.txt'
    label_path = os.path.join(labels_path, label_file)

    if os.path.exists(label_path):
        annotated_image = draw_boxes(image_path, label_path, class_names)

        # 결과 이미지 표시
        cv2.imshow('Annotated Image', annotated_image)
        print(f"Displaying image: {random_image}")
        print("Press any key to close the window.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print(f"Label file not found for {random_image}")

if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in ['train', 'valid']:
        print("Usage: python check_random_annotation.py [train|valid]")
        sys.exit(1)

    dataset_type = sys.argv[1]
    main(dataset_type)
