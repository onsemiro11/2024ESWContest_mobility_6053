from hailo_sdk_client import ClientRunner
import numpy as np

model_name = 'lane8_opset17'
alls = [
  'normalization1 = normalization([0.0, 0.0, 0.0], [255.0, 255.0, 255.0])\n',
  #object detection(yolov8n)
  #'nms_postprocess(meta_arch=yolov8, engine=cpu, nms_scores_th=0.2, nms_iou_th=0.4, classes=8)\n',
  
  #instance segmentation(yolov8n-seg)
  #'nms_postprocess(meta_arch=yolov8_seg, engine=cpu, nms_scores_th=0.2, nms_iou_th=0.4)\n',
]
har_path = f'./lane_segmentation/{model_name}.har'
calib_dataset = np.load('./lane_segmentation/calib_dataset.npy')

runner = ClientRunner(har=har_path)
runner.load_model_script(''.join(alls))
runner.optimize(calib_dataset)

runner.save_har(f'./lane_segmentation/{model_name}_quantized.har')
