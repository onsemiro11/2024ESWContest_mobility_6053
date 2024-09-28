import onnx

# 모델 로드
model = onnx.load("yolov8n.onnx")

# IR 버전 확인 (현재 버전 10일 가능성이 높음)
print(f"Current IR version: {model.ir_version}")

# IR 버전 9로 변경
model.ir_version = 9

# 변경된 모델 저장
onnx.save(model, "yolov8n9.onnx")

print("Model IR version downgraded to 9 and saved as best_ir9.onnx")

