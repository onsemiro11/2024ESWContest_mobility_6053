import onnx

# 모델 로드
model = onnx.load("./lane_segmentation/lane.onnx")

# IR 버전 확인 (현재 버전 10일 가능성이 높음)
print(f"Current IR version: {model.ir_version}")

# IR 버전 9로 변경
model.ir_version = 8

# 변경된 모델 저장
onnx.save(model, "./lane_segmentation/lane8.onnx")

print("Model IR version downgraded to 9 and saved as lane8.onnx")

