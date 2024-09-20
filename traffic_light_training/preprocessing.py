import os
from tqdm import tqdm
import argparse

def rename_files(images_path, labels_path, dataset_type):
    # 이미지와 라벨 파일 목록 가져오기
    image_files = sorted([f for f in os.listdir(images_path) if f.endswith('.jpg')])
    label_files = sorted([f for f in os.listdir(labels_path) if f.endswith('.txt')])

    # 파일 개수 확인 (이미지 파일과 라벨 파일 수가 일치해야 함)
    assert len(image_files) == len(label_files), "이미지 파일과 라벨 파일의 개수가 일치하지 않습니다."

    # 파일 이름 변경
    for idx, (img_file, lbl_file) in enumerate(tqdm(zip(image_files, label_files), total=len(image_files), desc="Renaming files")):
        prefix = 'v_' if dataset_type == 'valid' else ''
        new_name = f"{prefix}{idx}"  # 'valid'일 경우 'v_' 접두사 추가

        # 기존 경로
        img_src_path = os.path.join(images_path, img_file)
        lbl_src_path = os.path.join(labels_path, lbl_file)

        # 새로운 경로
        img_dst_path = os.path.join(images_path, f"{new_name}.jpg")
        lbl_dst_path = os.path.join(labels_path, f"{new_name}.txt")

        # 파일 이름 변경
        os.rename(img_src_path, img_dst_path)
        os.rename(lbl_src_path, lbl_dst_path)

        print(f"Renamed: {img_file} -> {new_name}.jpg, {lbl_file} -> {new_name}.txt")

def process_label_file(file_path, ori_class, rmv_class, new_class):
    with open(file_path, 'r') as f:
        labels = f.readlines()

    filtered_labels = []
    for label in labels:
        parts = label.split()
        if ori_class[int(parts[0])] not in rmv_class:
            new_class_id = new_class.index(ori_class[int(parts[0])])
            parts[0] = str(new_class_id)
            filtered_labels.append(' '.join(parts)+'\n')

    with open(file_path, 'w') as f:
        f.writelines(filtered_labels)

    return len(labels), len(filtered_labels)

def main(dataset_type):
    base_path = '/home/hyundo/traffic_light'
    images_path = os.path.join(base_path, dataset_type, 'images')
    labels_path = os.path.join(base_path, dataset_type, 'labels')

    ori_class = ['veh_go','veh_goLeft','veh_noSign','veh_stop','veh_stopLeft','veh_stopWarning','veh_warning','ped_go','ped_noSign','ped_stop','bus_go','bus_noSign,','bus_stop','bus_warning']
    rmv_class = []
    new_class = ['veh_go','veh_goLeft','veh_noSign','veh_stop','veh_stopLeft','veh_stopWarning','veh_warning','ped_go','ped_noSign','ped_stop','bus_go','bus_noSign,','bus_stop','bus_warning']

    rename_files(images_path, labels_path, dataset_type)

    print("\n **** class remove **** \n")

    total_processed = 0
    total_kept = 0

    label_files = [f for f in os.listdir(labels_path) if f.endswith('.txt')]
    for filename in tqdm(label_files, desc="Processing label files"):
        file_path = os.path.join(labels_path, filename)
        processed, kept = process_label_file(file_path, ori_class, rmv_class, new_class)
        total_processed += processed
        total_kept += kept

    print(f"\nTotal: Processed {total_processed} labels, kept {total_kept}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Preprocess dataset")
    parser.add_argument("dataset_type", choices=["train", "valid"], help="Type of dataset to process (train or valid)")
    args = parser.parse_args()

    main(args.dataset_type)
