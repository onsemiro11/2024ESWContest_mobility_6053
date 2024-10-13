import os
from tqdm import tqdm
import argparse

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
    base_path = '/home/hyundo/ADAS_data/traffic_light'
    images_path = os.path.join(base_path, dataset_type, 'images')
    labels_path = os.path.join(base_path, dataset_type, 'labels')

    ori_class = ['veh_go','veh_goLeft','veh_noSign','veh_stop','veh_stopLeft','veh_stopWarning','veh_warning','ped_go','ped_noSign','ped_stop','bus_go','bus_noSign,','bus_stop','bus_warning']
    rmv_class = ['ped_go','ped_noSign','ped_stop','bus_go','bus_noSign,','bus_stop','bus_warning']
    new_class = ['veh_go','veh_goLeft','veh_noSign','veh_stop','veh_stopLeft','veh_stopWarning','veh_warning']

    print("\n **** class remove **** \n")

    total_processed = 0
    total_kept = 0

    label_files = [f for f in os.listdir(labels_path) if f.endswith('.txt')]
    for filename in tqdm(label_files, desc="Processing label files"):
        file_path = os.path.join(labels_path, filename)
        processed, kept = process_label_file(file_path, ori_class, rmv_class, new_class)
        total_processed += processed
        total_kept += kept

        # Remove image and label file if filtered_labels is empty
        if kept == 0:
            img_file = file_path.replace('.txt', '.jpg').replace('labels','images')  # Assuming image files are .jpg
            os.remove(file_path)
            os.remove(img_file)
            print(f"Removed empty label and image: {file_path}, {img_file}")

    print(f"\nTotal: Processed {total_processed} labels, kept {total_kept}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Preprocess dataset")
    parser.add_argument("dataset_type", choices=["train", "valid"], help="Type of dataset to process (train or valid)")
    args = parser.parse_args()

    main(args.dataset_type)
