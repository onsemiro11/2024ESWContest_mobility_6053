from ultralytics import YOLO

# 모델 로드
# model = YOLO('/home/hyundo/2024ESWContest_mobility_TRAVI/runs/detect/train12/weights/last.pt')  # yolov8s.pt, yolov8m.pt 등 원하는 모델로 교체 가능
model = YOLO('/home/hyundo/2024ESWContest_mobility_TRAVI/road_mark_training/yolov8s.pt')  # yolov8s.pt, yolov8m.pt 등 원하는 모델로 교체 가능

# 학습 실행
model.train(
    data='/home/hyundo/ADAS_data/road_mark/data.yaml',  # 로드마크 데이터 설정 파일 경로
    epochs=150,  # 학습할 에포크 수
    imgsz=640,  # 이미지 크기
    batch=16,  # 배치 크기
    device='0'  # GPU 번호 (GPU가 여러 개 있는 경우)
)
