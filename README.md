# 2024ESWContest_mobility_TRAVI

## 작품명
### 최전방 차량으로 신호 대기 시에 신호 변경 알림 시스템 개발​

## 작품 설명
![project_overview](https://github.com/user-attachments/assets/7b191eea-1220-42e4-9ae8-e456e6370367)

### 개발 소개
- 카메라 기반으로 Road mark를 감지하여 현재 차량이 위치한 차로 판단​
- 신호등의 신호를 감지하여 현재 차량의 차로와 관련된 신호를 인지하고 출발 여부 판단​
- Raspberry pi 3대만 활용하여 개발 진행​
- 1대는 최종 영상 송출 및 판단, 나머지 두대는 각각 Road mark와 신호 감지 기능​
- 각 PC의 결과 메시지는 모두 TCP 및 ROS를 활용하고 AI모델은 Hailo 칩으로 Inference

## 디렉토리 구조
```
├─astrayolo_ws
│  └─src
│      ├─OrbbecSDK_ROS2

└─travi_ui
│  CMakeLists.txt
│  package.xml
├─include
│
├─media
│  └─img
└─src
│     beep.mp3
│     TRAVI_ui.cpp

├─convert_weight
│  │  00_calibration_dataset.py
│  │  0_onnx_version.py
│  │  1_opset_version.py
│  │  2_model_Parsing.py
│  │  3_model_optimization.py
│  │  4_model_compilation.py
│  │

│  ├─road_mark
│  │      [mark.pt](http://mark.pt/)
│  │      mark8.onnx
│  │      mark8_opset17.har
│  │      mark8_opset17.hef
│  │      mark8_opset17.onnx
│  │

│  ├─traffic_light
│  │      best_9.har
│  │      best_9.onnx
│  │      best_9_quantized.har
│  │      light.hef
│  │

├─road_mark_training
│      [augmentation.py](http://augmentation.py/)
│      check_class.py
│      check_image.py
│      [preprocessing.py](http://preprocessing.py/)
│      yolov8_train.py

├─traffic_light_training
│      [augmentation.py](http://augmentation.py/)
│      check_class.py
│      check_image.py
│      dataset_split.py
│      [preprocessing.py](http://preprocessing.py/)
│      [yolov8n.pt](http://yolov8n.pt/)
│      yolov8_train.py

├─roadmark
│  ├─hailo8l_detection
│  │  │  [detection.py](http://detection.py/)
│  │  │  hailo_rpi_common.py
│  │  │  requirements.txt
│  │  │  run_detection.sh
│  │  │  setup_env.sh
│  │  └─resources
│  └─ros2_docker
│      │  Dockerfile
│      │  tcp_server.py
│      └─src
│          └─my_ros2_package
│              └─scripts
│                  │  camera_image_publisher2.py

├─traffic_light
│  ├─hailo8l_detection
│  │  │  [detection.py](http://detection.py/)
│  │  │  hailo_rpi_common.py
│  │  │  requirements.txt
│  │  │  run_detection.sh
│  │  │  setup_env.sh
│  │  └─resources
│  └─ros2_docker
│      │  Dockerfile
│      │  tcp_server.py
│      │
│      └─src
│          └─my_ros2_package
│              └─scripts
│                      camera_image_publisher2.py
```
## AI 모델 설명
![Screenshot from 2024-10-28 12-44-09](https://github.com/user-attachments/assets/d1e3d2cb-a5d7-4905-b46e-fb15864427e1)


## 시스템 구성도
![Screenshot from 2024-10-28 12-43-22](https://github.com/user-attachments/assets/ada5a764-c178-4de2-b81b-0c9cb25d0598)

## SW 흐름도
![Screenshot from 2024-10-28 12-48-06](https://github.com/user-attachments/assets/bfe40c47-dd1a-4b8f-83dd-1d35d55cc0a0)

## 시연 영상 링크
https://youtu.be/9DhYVkQZaqg
