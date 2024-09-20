import os
from collections import Counter
import sys

def get_class_distribution(labels_path):
    class_counts = Counter()
    for label_file in os.listdir(labels_path):
        if label_file.endswith('.txt'):
            with open(os.path.join(labels_path, label_file), 'r') as f:
                labels = f.readlines()
                for label in labels:
                    class_id = int(label.split()[0])
                    class_counts[class_id] += 1
    return class_counts

class_data = ['RoadMarkArrow_Left', 'RoadMarkArrow_Right', 'RoadMarkArrow_Straight', 'RoadMarkArrow_StraightLeft', 'RoadMarkArrow_StraightRight', 'Vehicle_Bus', 'Vehicle_Car']

if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in ['train', 'valid']:
        print("Usage: python check_class.py [train|valid]")
        sys.exit(1)

    dataset_type = sys.argv[1]
    labels_path = f'/home/hyundo/road_mark/{dataset_type}/labels'
    
    class_distribution = get_class_distribution(labels_path)
    matched_distribution = {class_data[k]: v for k, v in class_distribution.items()}
    
    c = 0
    for class_name, count in matched_distribution.items():
        print(f"{class_name}: {count}")
        c += 1
    print("count_total_class : ", c)
