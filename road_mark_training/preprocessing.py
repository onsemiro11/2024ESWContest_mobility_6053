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

def class_remove(labels_path, ori_class, rmv_class, new_class):
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

def image_delete(images_path,labels_path, new_class):
    image_files = [f for f in os.listdir(images_path) if f.endswith('.jpg')]
    for filename in tqdm(image_files, desc="Deleting images"):
        file_path = os.path.join(images_path, filename)
        label_file_path = os.path.join(labels_path, filename.replace('.jpg', '.txt'))

        if os.path.exists(label_file_path):
            with open(label_file_path, 'r') as f:
                labels = f.readlines()

            # Check if all labels are either Vehicle_Bus or Vehicle_Car
            all_valid = all(int(line.split()[0]) in [new_class.index('Vehicle_Bus'), new_class.index('Vehicle_Car')] for line in labels)

            if all_valid:
                os.remove(file_path)
                os.remove(label_file_path)
                print(f"Deleted: {filename} and {label_file_path}")

def main(dataset_type):
    base_path = '/home/hyundo/road_mark'
    images_path = os.path.join(base_path, dataset_type, 'images')
    labels_path = os.path.join(base_path, dataset_type, 'labels')

    ori_class = ['Pedestrian_Bicycle', 'Pedestrian_Pedestrian', 'RoadMarkArrow_Else', 'RoadMarkArrow_Left', 'RoadMarkArrow_Right', 'RoadMarkArrow_Straight', 'RoadMarkArrow_StraightLeft', 'RoadMarkArrow_StraightRight', 'RoadMarkArrow_Uturn', 'RoadMark_Character', 'RoadMark_Crosswalk', 'RoadMark_Number', 'RoadMark_StopLine', 'TrafficLight_Arrow', 'TrafficLight_Green', 'TrafficLight_GreenArrow', 'TrafficLight_Red', 'TrafficLight_RedArrow', 'TrafficLight_Yellow', 'TrafficSign_Else', 'TrafficSign_Speed', 'Vehicle_Bus', 'Vehicle_Car', 'Vehicle_Motorcycle', 'Vehicle_Unknown']
    rmv_class = ['Pedestrian_Pedestrian','Pedestrian_Bicycle', 'RoadMarkArrow_Else', 'RoadMark_Character', 'RoadMark_Number',  'Vehicle_Motorcycle', 'Vehicle_Unknown','TrafficSign_Else', 'TrafficSign_Speed','TrafficLight_Yellow','TrafficLight_RedArrow','TrafficLight_Red','TrafficLight_GreenArrow','TrafficLight_Green','TrafficLight_Arrow','RoadMarkArrow_Uturn','RoadMark_StopLine','RoadMark_Crosswalk']
    new_class = ['RoadMarkArrow_Left', 'RoadMarkArrow_Right', 'RoadMarkArrow_Straight', 'RoadMarkArrow_StraightLeft', 'RoadMarkArrow_StraightRight', 'Vehicle_Bus', 'Vehicle_Car']

    # 파일 이름 변경
    # rename_files(images_path, labels_path, dataset_type)
    # 클래스 제거
    # class_remove(labels_path, ori_class, rmv_class, new_class)
    # image delete
    image_delete(images_path,labels_path,new_class)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Preprocess dataset")
    parser.add_argument("dataset_type", choices=["train", "valid"], help="Type of dataset to process (train or valid)")
    args = parser.parse_args()

    main(args.dataset_type)
