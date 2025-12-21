# Isaac ROS Perception Examples

GPU-accelerated perception with Visual SLAM, DNN inference, and stereo depth.

## Quick Start

```bash
# Pull Isaac ROS Docker
cd ~/workspaces
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
cd isaac_ros_common
./scripts/run_dev.sh isaac_ros_dev-x86_64

# Inside Docker: Install packages
cd /workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git
colcon build && source install/setup.bash

# Launch Visual SLAM (30 Hz)
ros2 launch visual_slam.launch.py

# Launch DNN Inference (20+ FPS)
bash models/download_yolov8_model.sh
ros2 launch dnn_inference.launch.py model_file_path:=models/yolov8n_fp16.engine

# Visualize in RViz
ros2 run rviz2 rviz2
```

## Performance (RTX 3060)

- Visual SLAM: 30 Hz, <2% drift
- YOLOv8n FP16: 25-30 FPS
- Stereo Depth: 15-20 Hz

## Troubleshooting

**CUDA OOM**: Use YOLOv8n (not YOLOv8m), reduce resolution
**GPU access**: `sudo apt install nvidia-docker2 && sudo systemctl restart docker`
