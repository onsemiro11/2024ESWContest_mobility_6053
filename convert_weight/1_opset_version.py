import onnx

# ONNX 모델 로드
model_path = './lane_segmentation/lane8.onnx'
model = onnx.load(model_path)

# 모델의 Opset 버전 확인
print("Current opset version:", model.opset_import[0].version)

# Opset 버전 변경 (17로 설정)
model.opset_import[0].version = 17

# 다운그레이드된 모델 저장
onnx.save(model, './lane_segmentation/lane8_opset17.onnx')
print("Opset version downgraded to 17.")

