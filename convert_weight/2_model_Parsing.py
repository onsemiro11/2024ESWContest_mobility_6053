from hailo_sdk_client import ClientRunner

model_name = 'lane8_opset17'
onnx_path = f'./lane_segmentation/{model_name}.onnx'
chosen_hw_arch = 'hailo8l'

runner = ClientRunner(hw_arch=chosen_hw_arch)
hn, npz = runner.translate_onnx_model(
    onnx_path,
    model_name,
    start_node_names=['images'],
    #object detection (yolov8n)
    #end_node_names=["/model.22/cv2.0/cv2.0.2/Conv","/model.22/cv3.0/cv3.0.2/Conv","/model.22/cv2.1/cv2.1.2/Conv",
    #'/model.22/cv3.1/cv3.1.2/Conv','/model.22/cv2.2/cv2.2.2/Conv','/model.22/cv3.2/cv3.2.2/Conv'],
    #net_input_shapes={'images': [1, 3, 640, 640]})
    
    #instance segmentation (yolov8n-seg)
    end_node_names=["/model.22/cv4.0/cv4.0.2/Conv","/model.22/cv4.1/cv4.1.2/Conv","/model.22/cv4.2/cv4.2.2/Conv",
    '/model.22/cv2.0/cv2.0.2/Conv','/model.22/cv3.0/cv3.0.2/Conv','/model.22/cv2.1/cv2.1.2/Conv','/model.22/cv3.1/cv3.1.2/Conv','/model.22/cv2.2/cv2.2.2/Conv','/model.22/cv3.2/cv3.2.2/Conv','/model.22/proto/cv3/conv/Conv'],
    net_input_shapes={'images': [1, 3, 640, 640]})

runner.save_har(f'./lane_segmentation/{model_name}.har')
