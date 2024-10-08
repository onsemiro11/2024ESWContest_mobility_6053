import json
import os
import sys
from tqdm import tqdm

# 이미지 크기 (예시로 1280x720으로 설정)
img_width = 1280
img_height = 720

# 명령줄 인자 처리 (train 또는 valid)
if len(sys.argv) != 2:
    print("Usage: python3 convert_yolo.py <train/valid>")
    sys.exit(1)

dataset_type = sys.argv[1]
if dataset_type not in ['train', 'valid']:
    print("Invalid argument. Please specify 'train' or 'valid'.")
    sys.exit(1)

# JSON 파일 경로 및 출력할 txt 파일 경로
input_json = f'lane_{dataset_type}.json'
output_dir = f'{dataset_type}/labels'
image_dir = f'{dataset_type}/images/'

# 디렉토리가 없는 경우 생성
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# YOLO 포맷으로 변환하는 함수
def convert_to_yolo(vertices, img_width, img_height):
    x_coords = [v[0] for v in vertices]
    y_coords = [v[1] for v in vertices]
    
    x_min = min(x_coords) / img_width
    x_max = max(x_coords) / img_width
    y_min = min(y_coords) / img_height
    y_max = max(y_coords) / img_height

    center_x = (x_min + x_max) / 2
    center_y = (y_min + y_max) / 2
    width = x_max - x_min
    height = y_max - y_min
    
    return center_x, center_y, width, height

# JSON 파일 열기
with open(input_json, 'r') as f:
    data = json.load(f)

# 라벨을 YOLO 형식으로 변환
for item in tqdm(data):
    image_name = item['name']
    
    # 'labels' 키가 없는 경우 이미지 삭제
    if 'labels' not in item:
        image_path = os.path.join(image_dir, image_name)
        if os.path.exists(image_path):
            os.remove(image_path)
        continue
    
    labels = item['labels']
    
    output_file = os.path.join(output_dir, image_name.replace('.jpg', '.txt'))
    
    with open(output_file, 'w') as out_f:
        for label in labels:
            if 'poly2d' in label:
                for poly in label['poly2d']:
                    vertices = poly['vertices']
                    # Class index is set to 0 as an example
                    class_index = 0
                    # Convert vertices to the required format
                    vertices_str = ' '.join(f"{x/img_width} {y/img_height}" for x, y in vertices)
                    # Write to file
                    out_f.write(f"{class_index} {vertices_str}\n")

print(f"변환 완료! '{dataset_type}' 데이터셋이 YOLO 형식으로 변환되었습니다.")
