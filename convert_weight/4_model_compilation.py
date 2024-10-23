from hailo_sdk_client import ClientRunner

model_name = 'lane8_opset17'
quantized_model_har_path = f'./lane_segmentation/{model_name}_quantized.har'
runner = ClientRunner(har=quantized_model_har_path)

hef = runner.compile()

file_name = f'./lane_segmentation/{model_name}.hef'
with open(file_name, 'wb') as f:
    f.write(hef)              
