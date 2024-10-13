import os
import shutil
import random
from tqdm import tqdm

def split_dataset(base_dir, train_ratio=0.8):
    image_dir = os.path.join(base_dir, 'train/images')
    label_dir = os.path.join(base_dir, 'train/labels')
    valid_dir = os.path.join(base_dir, 'valid')
    valid_image_dir = os.path.join(valid_dir, 'images')
    valid_label_dir = os.path.join(valid_dir, 'labels')

    # 검증 데이터 디렉토리 생성
    os.makedirs(valid_image_dir, exist_ok=True)
    os.makedirs(valid_label_dir, exist_ok=True)

    # 이미지와 라벨 파일 목록 가져오기
    images = os.listdir(image_dir)
    labels = os.listdir(label_dir)

    # 이미지와 라벨 파일 이름에서 확장자 제거
    image_basenames = {os.path.splitext(img)[0] for img in images}
    label_basenames = {os.path.splitext(lbl)[0] for lbl in labels}

    # 이미지와 라벨 파일이 매칭되는지 확인
    matched_images = [img for img in images if os.path.splitext(img)[0] in label_basenames]

    # 실제 폴더 안에 있는 데이터 파일 개수 출력
    print(f"Number of image files: {len(images)}")
    print(f"Number of label files: {len(labels)}")
    print(f"Number of matched images: {len(matched_images)}")

    # 이미지와 라벨 쌍을 랜덤하게 섞기
    random.shuffle(matched_images)

    # 학습 데이터와 검증 데이터 나누기
    train_size = int(len(matched_images) * train_ratio)
    valid_images = matched_images[train_size:]

    # 검증 데이터 복사
    for img in tqdm(valid_images, desc="Moving validation data"):
        lbl = os.path.splitext(img)[0] + '.txt'
        shutil.move(os.path.join(image_dir, img), os.path.join(valid_image_dir, img))
        shutil.move(os.path.join(label_dir, lbl), os.path.join(valid_label_dir, lbl))

    print(f"After Number of image files: {len(os.listdir(image_dir))}")
    print(f"After Number of label files: {len(os.listdir(label_dir))}")

# 사용 예시
split_dataset('/home/hyundo/ADAS_data/traffic_light/')