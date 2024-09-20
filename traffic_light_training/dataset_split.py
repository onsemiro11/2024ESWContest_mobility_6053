import os
import shutil
import random

def split_dataset(base_dir, train_ratio=0.8):
    image_dir = os.path.join(base_dir, 'images')
    label_dir = os.path.join(base_dir, 'labels')
    valid_dir = os.path.join(base_dir, '../valid')

    # 이미지와 라벨 파일 목록 가져오기
    images = os.listdir(image_dir)
    labels = os.listdir(label_dir)

    # 실제 폴더 안에 있는 데이터 파일 개수 출력
    print(f"Number of image files: {len(images)}")
    print(f"Number of label files: {len(labels)}")

    # 이미지와 라벨 쌍을 랜덤하게 섞기
    data_pairs = list(zip(images, labels))
    random.shuffle(data_pairs)

    # 학습 데이터와 검증 데이터 나누기
    train_size = int(len(data_pairs) * train_ratio)
    valid_pairs = data_pairs[train_size:]

    # 검증 데이터 복사
    os.makedirs(valid_dir, exist_ok=True)
    for img, lbl in valid_pairs:
        shutil.move(os.path.join(image_dir, img), os.path.join(valid_dir, 'images', img))
        shutil.move(os.path.join(label_dir, lbl), os.path.join(valid_dir, 'labels', lbl))

    print(f"After Number of image files: {len(os.listdir(image_dir))}")
    print(f"After Number of label files: {len(os.listdir(label_dir))}")

# 사용 예시
split_dataset('/home/hyundo/traffic_light/train')