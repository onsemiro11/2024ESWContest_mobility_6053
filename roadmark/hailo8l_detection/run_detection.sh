#!/bin/bash

# source setup_env.sh &&  python detection.py --hef-path ./resources/yolov8n_traffic.hef --labels-json ./resources/labels.json --input ./resources/videoplayback1.mp4
source setup_env.sh &&  python detection.py --hef-path ./resources/yolov8n_mark.hef --labels-json ./resources/mark_labels.json --input tcp://127.0.0.1:1234
