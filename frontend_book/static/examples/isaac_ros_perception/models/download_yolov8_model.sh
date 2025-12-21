#!/bin/bash
# Download and convert YOLOv8 to TensorRT engine

set -e

echo "Downloading YOLOv8n ONNX..."
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.onnx

echo "Converting to TensorRT FP16..."
/usr/src/tensorrt/bin/trtexec \
  --onnx=yolov8n.onnx \
  --saveEngine=yolov8n_fp16.engine \
  --fp16 \
  --workspace=4096

echo "Model ready: yolov8n_fp16.engine"
echo "Expected FPS: 25-30 on RTX 3060"
