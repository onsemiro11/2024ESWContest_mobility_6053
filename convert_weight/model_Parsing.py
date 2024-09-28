from hailo_sdk_client import ClientRunner

model_name = 'yolov8n'
onnx_path = f'{model_name}.onnx'
chosen_hw_arch = 'hailo8l'

runner = ClientRunner(hw_arch=chosen_hw_arch)
hn, npz = runner.translate_onnx_model(
    onnx_path,
    model_name,
    start_node_names=['images'],
    end_node_names=["/model.22/cv2.0/cv2.0.2/Conv","/model.22/cv3.0/cv3.0.2/Conv","/model.22/cv2.1/cv2.1.2/Conv",
    '/model.22/cv3.1/cv3.1.2/Conv','/model.22/cv2.2/cv2.2.2/Conv','/model.22/cv3.2/cv3.2.2/Conv'],
    net_input_shapes={'images': [1, 3, 640, 640]})

runner.save_har(f'{model_name}.har')
