import os
import numpy as np
import sys
import random
import cv2
import matplotlib.pyplot as plt

# 명령줄 인자 처리 (train 또는 valid)
if len(sys.argv) != 2:
    print("Usage: python3 visualize_yolo_labels.py <train/valid>")
    sys.exit(1)

dataset_type = sys.argv[1]
if dataset_type not in ['train', 'valid']:
    print("Invalid argument. Please specify 'train' or 'valid'.")
    sys.exit(1)

# 이미지 및 라벨 경로 설정
image_dir = f'{dataset_type}/images/'
label_dir = f'{dataset_type}/labels/'

# 이미지 크기 설정 (필요에 따라 수정)
img_width = 1280
img_height = 720

# 랜덤으로 라벨 선택
label_files = os.listdir(label_dir)
if not label_files:
    print(f"No label files found in {label_dir}")
    sys.exit(1)

random_label = random.choice(label_files)
label_path = os.path.join(label_dir, random_label)

# 대응하는 이미지 파일 찾기
image_name = random_label.replace('.txt', '.jpg')
image_path = os.path.join(image_dir, image_name)

# 이미지 불러오기
if not os.path.exists(image_path):
    print(f"Image file {image_name} not found in {image_dir}")
    sys.exit(1)

image = cv2.imread(image_path)
if image is None:
    print(f"Failed to load image {image_path}")
    sys.exit(1)

# YOLO 라벨 파일 파싱
with open(label_path, 'r') as f:
    labels = f.readlines()

# 폴리곤 그리기
for label in labels:
    label_info = label.strip().split()
    class_id = int(label_info[0])
    polygon = [float(x) for x in label_info[1:]]
    
    # 좌표를 YOLO 포맷에서 이미지 크기에 맞게 변환
    points = []
    for i in range(0, len(polygon), 2):
        x = int(polygon[i] * img_width)
        y = int(polygon[i+1] * img_height)
        points.append((x, y))

    # 다각형 그리기
    points = np.array(points, np.int32)
    points = points.reshape((-1, 1, 2))
    cv2.polylines(image, [points], isClosed=True, color=(0, 255, 0), thickness=2)

# 이미지 출력
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title(f"Segmentation from {random_label}")
plt.show()
