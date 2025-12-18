# Module 1: URDF Humanoid Robot Example

This directory contains a complete URDF (Unified Robot Description Format) example for teaching robot modeling in ROS 2.

## Overview

The `simple_humanoid.urdf` file defines a simplified humanoid robot with:
- **1 torso** (base_link) - blue, 5 kg
- **2 arms** (left and right) - gray, 1.5 kg each (upper arms)
- **2 forearms** (left and right) - gray, 1.0 kg each
- **4 revolute joints** (2 shoulders, 2 elbows)

This example demonstrates:
- Link definitions with visual, collision, and inertial properties
- Revolute joint configuration with limits
- Kinematic tree structure
- RViz visualization
- Programmatic joint control

## Directory Structure

```
module_1_urdf/
├── simple_humanoid.urdf          # Complete URDF robot description
├── launch/
│   └── visualize_humanoid.launch.py  # Launch file for RViz visualization
├── joint_controller_example.py   # Programmatic joint control example
├── humanoid.rviz                 # RViz configuration (optional)
└── README.md                     # This file
```

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10+

### Required ROS 2 Packages

```bash
# Install URDF validation tools
sudo apt-get install liburdfdom-tools

# Install robot state publisher
sudo apt-get install ros-humble-robot-state-publisher

# Install joint state publisher with GUI
sudo apt-get install ros-humble-joint-state-publisher-gui

# Install RViz2
sudo apt-get install ros-humble-rviz2

# Install TF2 tools
sudo apt-get install ros-humble-tf2-tools
```

## Quick Start

### 1. Validate the URDF

Before visualizing, validate the URDF syntax:

```bash
cd ~/ros2_ws/ros2-examples/module_1_urdf
check_urdf simple_humanoid.urdf
```

**Expected output:**
```
robot name is: simple_humanoid
---------- Successfully Parsed XML ---------------
root Link: base_link has 2 child(ren)
    child(1):  left_upper_arm
        child(1):  left_forearm
    child(2):  right_upper_arm
        child(1):  right_forearm
```

### 2. Visualize in RViz

Launch the visualization:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Navigate to this directory
cd ~/ros2_ws/ros2-examples/module_1_urdf

# Run the launch file
ros2 launch module_1_urdf visualize_humanoid.launch.py
```

**What you'll see:**
- **Joint State Publisher GUI**: Window with sliders for controlling each joint
- **RViz**: 3D visualization of the robot (blue torso, gray arms)

### 3. Control Joints Manually

Use the sliders in the Joint State Publisher GUI:
- **left_shoulder_joint**: -1.57 to 1.57 rad (-90° to +90°)
- **right_shoulder_joint**: -1.57 to 1.57 rad (-90° to +90°)
- **left_elbow_joint**: 0.0 to 2.35 rad (0° to 135°)
- **right_elbow_joint**: 0.0 to 2.35 rad (0° to 135°)

### 4. Programmatic Control (Optional)

To control joints programmatically with animated motion:

```bash
# Terminal 1: Launch visualization without joint_state_publisher_gui
# (Modify launch file to exclude joint_state_publisher_gui node)

# Terminal 2: Run custom joint controller
cd ~/ros2_ws/ros2-examples/module_1_urdf
python3 joint_controller_example.py
```

The robot arms will wave automatically using sine wave animation.

## Robot Specifications

### Links

| Link Name | Geometry | Dimensions (m) | Mass (kg) | Color |
|-----------|----------|----------------|-----------|-------|
| base_link (torso) | Box | 0.3 × 0.2 × 0.4 | 5.0 | Blue |
| left_upper_arm | Box | 0.1 × 0.1 × 0.3 | 1.5 | Gray |
| right_upper_arm | Box | 0.1 × 0.1 × 0.3 | 1.5 | Gray |
| left_forearm | Box | 0.08 × 0.08 × 0.25 | 1.0 | Gray |
| right_forearm | Box | 0.08 × 0.08 × 0.25 | 1.0 | Gray |

### Joints

| Joint Name | Type | Parent | Child | Axis | Limits (rad) |
|------------|------|--------|-------|------|--------------|
| left_shoulder_joint | Revolute | base_link | left_upper_arm | Z | -1.57 to 1.57 |
| right_shoulder_joint | Revolute | base_link | right_upper_arm | Z | -1.57 to 1.57 |
| left_elbow_joint | Revolute | left_upper_arm | left_forearm | Y | 0.0 to 2.35 |
| right_elbow_joint | Revolute | right_upper_arm | right_forearm | Y | 0.0 to 2.35 |

### Coordinate Frame Convention

- **X-axis**: Forward (right is positive, left is negative)
- **Y-axis**: Left (left is positive, right is negative)
- **Z-axis**: Up (up is positive, down is negative)
- **RPY**: Roll (X), Pitch (Y), Yaw (Z) angles in radians

## Validation and Debugging

### Check URDF Structure

Generate a visual graph of the kinematic tree:

```bash
urdf_to_graphiz simple_humanoid.urdf
```

This creates `simple_humanoid.pdf` showing the robot structure.

### View TF Tree

Generate a TF tree diagram:

```bash
# With visualization running
ros2 run tf2_tools view_frames
```

This creates `frames.pdf` showing all coordinate frames and their relationships.

### Monitor Joint States

View joint state messages:

```bash
ros2 topic echo /joint_states
```

### Check Robot Description Parameter

Verify the robot description is loaded:

```bash
ros2 param get /robot_state_publisher robot_description
```

## Troubleshooting

### Issue: RViz shows "No transform from [link] to [base_link]"

**Cause**: `robot_state_publisher` is not running or URDF is malformed.

**Solution**:
1. Check if `robot_state_publisher` is running: `ros2 node list`
2. Validate URDF: `check_urdf simple_humanoid.urdf`
3. Restart the launch file

### Issue: Joints don't move in RViz

**Cause**: No joint state publisher is running.

**Solution**:
```bash
# Check if joint states are being published
ros2 topic list | grep joint_states

# If not present, start joint_state_publisher_gui
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### Issue: Robot appears at origin with wrong orientation

**Cause**: Fixed Frame in RViz is not set to `base_link`.

**Solution**: In RViz, set **Global Options → Fixed Frame** to `base_link`.

### Issue: `check_urdf` fails with "command not found"

**Cause**: `liburdfdom-tools` not installed.

**Solution**:
```bash
sudo apt-get update
sudo apt-get install liburdfdom-tools
```

## Learning Exercises

1. **Add legs**: Extend the URDF to include:
   - 2 thigh links connected via hip joints
   - 2 shin links connected via knee joints
   - Visualize in RViz

2. **Add a head**: Create a head link connected via a neck joint (revolute, Z-axis).

3. **Add sensors**: Include a camera link attached to the torso with a fixed joint.

4. **Mesh geometry**: Replace box primitives with mesh files (.stl or .dae).

5. **Modify joint limits**: Change elbow joint limits to allow full flexion (0° to 180°).

6. **Create custom animation**: Modify `joint_controller_example.py` to create:
   - Arms moving out of phase
   - Asymmetric motion patterns
   - Stop-and-go behavior

## Related Documentation

- **ROS 2 URDF Documentation**: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html
- **robot_state_publisher**: https://github.com/ros/robot_state_publisher
- **joint_state_publisher**: https://github.com/ros/joint_state_publisher
- **RViz2 User Guide**: https://github.com/ros2/rviz

## License

Apache 2.0

## Author

Physical AI & Humanoid Robotics Textbook
Governor Sindh IT Initiative - Quarter 4

---

**Next Steps**: After mastering URDF basics, explore:
- XACRO (XML Macros) for parameterized URDF
- Gazebo simulation with physics plugins
- MoveIt 2 for motion planning
- ros2_control for hardware interfaces
