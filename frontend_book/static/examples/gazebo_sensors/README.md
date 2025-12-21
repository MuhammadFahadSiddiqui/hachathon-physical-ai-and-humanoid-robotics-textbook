# Gazebo Sensor Simulation Examples

This example demonstrates realistic sensor simulation in Gazebo Classic for **Module 2 - Chapter 3: Sensor Simulation**. It includes three sensor configurations: LiDAR, depth camera, and IMU.

## Contents

- `humanoid_with_lidar.urdf` - Robot with 2D planar LiDAR (240° FOV, 4m range)
- `humanoid_with_depth_camera.urdf` - Robot with RGB-D camera (1280x720, 87° FOV)
- `humanoid_with_imu.urdf` - Robot with 6-DOF IMU (gyroscope + accelerometer)
- `launch/sensor_demo.launch.py` - ROS 2 launch file for any sensor configuration
- `README.md` - This file (usage instructions and troubleshooting)

## Prerequisites

Before running this example, ensure you have:

1. **ROS 2 Humble** installed and sourced
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Gazebo Classic 11** with sensor plugins
   ```bash
   sudo apt install gazebo11 ros-humble-gazebo-ros-pkgs
   ```

3. **RViz 2** for sensor visualization
   ```bash
   sudo apt install ros-humble-rviz2
   ```

## Quick Start

### Option 1: LiDAR Sensor

```bash
# Navigate to example directory
cd gazebo_sensors

# Launch Gazebo with LiDAR-equipped robot
ros2 launch launch/sensor_demo.launch.py sensor_type:=lidar

# In new terminal: verify /scan topic
ros2 topic list | grep scan
# Expected: /scan

# Check LiDAR publish rate
ros2 topic hz /scan
# Expected: average rate: 10.000 Hz

# Echo scan data
ros2 topic echo /scan --once
```

**Expected Result**:
- Gazebo opens with humanoid robot
- Black cylindrical LiDAR visible on top of torso
- Red laser rays visible in Gazebo (if `visualize:=true`)
- `/scan` topic publishes LaserScan messages at 10 Hz

### Option 2: Depth Camera

```bash
# Launch Gazebo with depth camera-equipped robot
ros2 launch launch/sensor_demo.launch.py sensor_type:=depth

# Verify camera topics
ros2 topic list | grep camera
# Expected:
# /camera/depth/camera_info
# /camera/depth/image_raw
# /camera/points
# /camera/rgb/camera_info
# /camera/rgb/image_raw

# Check depth image publish rate
ros2 topic hz /camera/depth/image_raw
# Expected: average rate: 30.000 Hz
```

**Expected Result**:
- Small black camera box visible on front of torso
- Green camera frustum visible in Gazebo
- 5 camera topics publishing at 30 Hz

### Option 3: IMU

```bash
# Launch Gazebo with IMU-equipped robot
ros2 launch launch/sensor_demo.launch.py sensor_type:=imu

# Verify /imu topic
ros2 topic echo /imu --once
```

**Expected Output**:
```yaml
header:
  stamp:
    sec: 5
    nanosec: 123000000
  frame_id: imu_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0  # Quaternion (no rotation when stationary)
angular_velocity:
  x: 0.0  # rad/s
  y: 0.0
  z: 0.0
linear_acceleration:
  x: 0.0
  y: 0.0
  z: 9.81  # Gravity in Z-axis (m/s²)
```

---

## Visualizing Sensors in RViz

### LiDAR Visualization

```bash
# Launch RViz
rviz2

# Configure RViz:
1. Set Fixed Frame: "base_link" (in Global Options)
2. Add → LaserScan
   - Topic: /scan
   - Size: 0.05
   - Color: Red
3. Add → TF (to see sensor frame)
```

**Expected**: Red dots showing LiDAR ray hit points in 240° arc.

### Depth Camera Visualization

```bash
# Launch RViz
rviz2

# Configure RViz:
1. Set Fixed Frame: "camera_optical_frame"
2. Add → Image
   - Topic: /camera/rgb/image_raw (color image)
3. Add → DepthCloud
   - Topic: /camera/points (3D point cloud)
   - Color: Flat Color → White
4. Add → Camera
   - Topic: /camera/rgb/camera_info (shows camera frustum)
```

**Expected**: Color image in Image panel, 3D point cloud in main view.

### IMU Visualization

IMU data is numerical (not visual). Use ROS 2 plotting tools:

```bash
# Install rqt_plot
sudo apt install ros-humble-rqt-plot

# Plot angular velocity
rqt_plot /imu/angular_velocity/x /imu/angular_velocity/y /imu/angular_velocity/z

# Rotate robot in Gazebo (drag with mouse) and observe changes
```

---

## Sensor Testing

### 1. LiDAR Range Accuracy Test

**Objective**: Verify LiDAR detects obstacles at correct distances.

**Procedure**:
1. Launch LiDAR demo: `ros2 launch launch/sensor_demo.launch.py sensor_type:=lidar`
2. In Gazebo GUI: Insert → Model → Box
3. Place box 1.0 meter directly in front of robot (Y = 1.0)
4. Check laser scan data:
   ```bash
   ros2 topic echo /scan | grep "ranges:"
   ```
5. **Expected**: Central range value ≈ 1.0m (±0.05m tolerance per spec SC-006)

**Troubleshooting**:
- If range shows `inf` or `nan`: Box is outside 4m max range or closer than 0.1m min range
- If range is incorrect: Check box position with Gazebo "Translation Mode" tool

### 2. Depth Camera Calibration Check

**Objective**: Verify depth image accuracy.

**Procedure**:
1. Launch depth camera demo: `ros2 launch launch/sensor_demo.launch.py sensor_type:=depth`
2. In Gazebo: Insert → Model → Sphere (radius 0.5m)
3. Place sphere 2.0 meters in front of camera
4. View depth image in RViz (Add → Image → /camera/depth/image_raw)
5. **Expected**: Sphere appears as bright spot (closer = brighter)
6. Verify depth value:
   ```bash
   ros2 topic echo /camera/depth/image_raw --once | grep "data:" | head -n 1
   ```
   (Depth values in millimeters; 2000mm = 2.0m)

**Acceptance Criteria**: Depth error <10% (per spec SC-007)

### 3. IMU Orientation Test

**Objective**: Verify IMU quaternion changes with robot rotation.

**Procedure**:
1. Launch IMU demo: `ros2 launch launch/sensor_demo.launch.py sensor_type:=imu`
2. Echo IMU data: `ros2 topic echo /imu`
3. Initial orientation (robot upright): `w ≈ 1.0, x ≈ 0, y ≈ 0, z ≈ 0`
4. In Gazebo, use "Rotation Mode" to tilt robot 45° around X-axis
5. **Expected**: Quaternion changes (x and w values non-zero)
6. Reset robot pose: Gazebo → Edit → Reset Model Poses

**Note**: Quaternions are normalized (√(w²+x²+y²+z²) = 1.0)

---

## Troubleshooting

### Issue: "/scan topic not publishing"

**Symptoms**: `ros2 topic list` does not show `/scan` after launching LiDAR example

**Cause**: LiDAR plugin not loaded, or Gazebo crashed

**Fix**:
1. Check Gazebo terminal for errors:
   ```
   [ERROR] Failed to load plugin libgazebo_ros_ray_sensor.so
   ```
2. Install missing plugin:
   ```bash
   sudo apt install ros-humble-gazebo-plugins
   ```
3. Restart Gazebo

**Verification**:
```bash
# Check if plugin library exists
ls /opt/ros/humble/lib/libgazebo_ros_ray_sensor.so
# Expected: File exists
```

---

### Issue: "Depth camera shows blank/black images in RViz"

**Symptoms**: `/camera/rgb/image_raw` publishes, but RViz Image display is black

**Cause**: Camera not pointing at any objects, or clipping planes misconfigured

**Fix**:
1. **Add objects to scene**:
   - Gazebo → Insert → Model → Table
   - Place table 2m in front of robot (within camera FOV)
2. **Check camera orientation**:
   - In Gazebo, select camera_link
   - Verify camera is facing forward (green frustum should point toward scene)
3. **Verify clipping planes**:
   - Near clip: 0.3m (objects closer are invisible)
   - Far clip: 10.0m (objects farther are invisible)

---

### Issue: "IMU linear_acceleration shows zero instead of gravity"

**Symptoms**: `linear_acceleration.z` is 0.0 instead of 9.81 m/s²

**Cause**: Gazebo gravity not enabled for IMU link

**Fix**:
1. Check URDF `<gazebo reference="imu_link">` tag includes:
   ```xml
   <gravity>true</gravity>
   ```
2. Restart Gazebo simulation

**Expected**: When robot is stationary and upright, IMU should read:
- `linear_acceleration.z ≈ 9.81` (gravity pulling down)
- `angular_velocity.x/y/z ≈ 0.0` (no rotation)

---

### Issue: "Sensor data has no noise (values are perfect)"

**Symptoms**: LaserScan ranges are exact integers (1.0000, 2.0000), no variation

**Cause**: Noise model disabled in URDF

**Fix**:
1. Verify URDF `<noise>` tag exists for sensor
2. For LiDAR:
   ```xml
   <noise>
     <type>gaussian</type>
     <mean>0.0</mean>
     <stddev>0.01</stddev>  <!-- 1cm noise -->
   </noise>
   ```
3. For depth camera:
   ```xml
   <noise>
     <type>gaussian</type>
     <stddev>0.007</stddev>  <!-- 7mm noise -->
   </noise>
   ```

**Expected**: Sensor values vary slightly each update (realistic noise)

---

## Sensor Specifications Summary

| Sensor | Type | FOV | Range | Resolution | Update Rate | Noise |
|--------|------|-----|-------|------------|-------------|-------|
| **LiDAR** | 2D Ray | 240° H | 0.1-4.0m | 640 samples | 10 Hz | σ=0.01m |
| **Depth Camera** | RGB-D | 87° H × 58° V | 0.3-10.0m | 1280×720 | 30 Hz | σ=0.007m |
| **IMU** | 6-DOF | N/A | N/A | ±2000°/s gyro, ±16g accel | 100 Hz | Bias drift |

---

## Hardware Requirements

**Minimum** (for sensor simulation):
- 8GB RAM
- Integrated GPU (Intel HD Graphics)
- Ubuntu 22.04 or WSL2
- 2 CPU cores

**Recommended** (for multiple sensors + RViz):
- 16GB RAM
- 4 CPU cores
- Dedicated GPU (for point cloud visualization)

---

## Next Steps

After successfully running this example:

1. **Read Chapter 3** of Module 2 to understand:
   - Gazebo sensor plugin architecture
   - Noise model configuration (Gaussian, bias drift)
   - ROS 2 sensor message formats
   - Sensor fusion concepts (multi-sensor SLAM)

2. **Experiment with Sensor Parameters**:
   - Modify LiDAR FOV to 360° (change `min_angle`, `max_angle`, `samples`)
   - Increase depth camera resolution to 1920×1080 (Full HD)
   - Adjust IMU noise parameters (gyroscope stddev, accelerometer bias)

3. **Multi-Sensor Integration**:
   - Create URDF with LiDAR + depth camera + IMU on same robot
   - Launch synchronized sensors, visualize in RViz
   - Explore sensor fusion (e.g., combine LiDAR + depth for 3D mapping)

4. **Real Robot Comparison**:
   - Compare simulated LiDAR to real RPLIDAR/Hokuyo specs
   - Compare depth camera to Intel RealSense D435 datasheet
   - Validate noise models against real sensor datasheets

---

## Support

For issues or questions:
- **Module 2 Documentation**: See Chapter 3 for detailed explanations
- **Gazebo Sensor Tutorials**: http://classic.gazebosim.org/tutorials?cat=sensors
- **ROS 2 Sensor Msgs**: https://docs.ros.org/en/humble/p/sensor_msgs/
- **ROS Answers Forum**: https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:sensors/

---

**Author**: AI-Driven Robotics Textbook
**Module**: 2 - The Digital Twin (Gazebo & Unity)
**Chapter**: 3 - Sensor Simulation
**Last Updated**: 2025-12-19
