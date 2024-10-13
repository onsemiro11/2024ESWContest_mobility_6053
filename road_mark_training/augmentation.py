import os
import cv2
import random
from glob import glob
import numpy as np
from collections import Counter
import sys
from tqdm import tqdm

def augment_image(image):
    # 밝기 조절 (Random Brightness ±20%)
    alpha = 1 + random.uniform(-0.2, 0.2)
    bright_image = cv2.convertScaleAbs(image, alpha=alpha, beta=0)
    return bright_image

def get_rare_classes(labels_path):
    class_counts = Counter()
    for label_file in os.listdir(labels_path):
        if label_file.endswith('.txt'):
            with open(os.path.join(labels_path, label_file), 'r') as f:
                labels = f.readlines()
                for label in labels:
                    class_id = int(label.split()[0])
                    class_counts[class_id] += 1
    
    # 클래스 수가 평균보다 적은 클래스를 희귀 클래스로 정의
    avg_count = sum(class_counts.values()) / len(class_counts)
    rare_classes = set(class_id for class_id, count in class_counts.items() if count < avg_count)
    return rare_classes

def main(dataset_type):
    # 폴더 경로 설정
    base_path = f'/home/hyundo/ADAS_data/road_mark/{dataset_type}'
    images_path = os.path.join(base_path, 'images')
    labels_path = os.path.join(base_path, 'labels')

    # 초기 파일 개수 출력
    initial_image_count = len(glob(os.path.join(images_path, '*.jpg')))
    initial_label_count = len(glob(os.path.join(labels_path, '*.txt')))
    print(f"Initial file count - Images: {initial_image_count}, Labels: {initial_label_count}")

    # 희귀 클래스 목록 가져오기
    rare_classes = get_rare_classes(labels_path)

    # 이미지 파일 목록 가져오기
    image_files = glob(os.path.join(images_path, '*.jpg'))

    # 증강 목표: 3배 증강
    augmentation_factor = 3

    # 이미지와 라벨 증강 및 저장
    for img_file in tqdm(image_files, desc=f"Augmenting {dataset_type} dataset"):
        base_name = os.path.basename(img_file).replace('.jpg', '')
        label_file = os.path.join(labels_path, f"{base_name}.txt")
        
        # 라벨 파일에 희귀 클래스가 포함되어 있는지 확인
        if os.path.exists(label_file):
            with open(label_file, 'r') as f:
                labels = f.readlines()
            if any(int(label.split()[0]) in rare_classes for label in labels):
                image = cv2.imread(img_file)
                
                for i in range(augmentation_factor - 1):
                    # 이미지 증강
                    augmented_image = augment_image(image)

                    # 새로운 파일명 생성
                    new_img_name = f"aug_{base_name}_{i}.jpg"
                    new_img_path = os.path.join(images_path, new_img_name)
                    
                    new_label_name = f"aug_{base_name}_{i}.txt"
                    new_label_path = os.path.join(labels_path, new_label_name)

                    # 증강된 이미지 저장
                    cv2.imwrite(new_img_path, augmented_image)
                    
                    # 라벨 파일 복사
                    with open(new_label_path, 'w') as f:
                        f.writelines(labels)

    # 최종 파일 개수 출력
    final_image_count = len(glob(os.path.join(images_path, '*.jpg')))
    final_label_count = len(glob(os.path.join(labels_path, '*.txt')))
    print(f"Final file count - Images: {final_image_count}, Labels: {final_label_count}")

if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in ['train', 'valid']:
        print("Usage: python selective_augmentation.py [train|valid]")
        sys.exit(1)

    dataset_type = sys.argv[1]
    main(dataset_type)
