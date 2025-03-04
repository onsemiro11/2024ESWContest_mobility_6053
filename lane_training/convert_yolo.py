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
input_json = f'/home/hyundo/ADAS_data/lane_data/lane_{dataset_type}.json'
output_dir = f'/home/hyundo/ADAS_data/lane_data/{dataset_type}/labels'
image_dir = f'/home/hyundo/ADAS_data/lane_data/{dataset_type}/images/'

# 디렉토리가 없는 경우 생성
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Define the categories to filter
allowed_categories = {
    "double other",
    "double white",
    "double yellow",
    "single other",
    "single white",
    "single yellow"
}

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
            # Filter labels based on category
            if label['category'] not in allowed_categories:
                continue
            
            if 'poly2d' in label:
                for poly in label['poly2d']:
                    vertices = poly['vertices']
                    # Class index is set to 0 as an example
                    class_index = 0
                    
                    # Check if there are exactly two vertices
                    if len(vertices) == 2:
                        # Create four new points by adding and subtracting 0.05
                        new_vertices = [
                            (vertices[0][0] - 3, vertices[0][1]),
                            (vertices[0][0] + 3, vertices[0][1]),
                            (vertices[1][0] + 3, vertices[1][1]),
                            (vertices[1][0] - 3, vertices[1][1])
                        ]
                        # Replace the original vertices with the new ones
                        vertices = new_vertices
                    
                    # Convert vertices to the required format
                    vertices_str = ' '.join(f"{max(0, min(1, x/img_width))} {max(0, min(1, y/img_height))}" for x, y in vertices)
                    # Write to file
                    out_f.write(f"{class_index} {vertices_str}\n")

print(f"변환 완료! '{dataset_type}' 데이터셋이 YOLO 형식으로 변환되었습니다.")
