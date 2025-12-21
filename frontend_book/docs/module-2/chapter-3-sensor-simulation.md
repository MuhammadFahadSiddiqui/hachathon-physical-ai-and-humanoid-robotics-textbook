---
title: Chapter 3 - Sensor Simulation
sidebar_position: 4
---

# Chapter 3: Sensor Simulation – LiDAR, Depth Cameras, IMUs

## Learning Objectives

By the end of this chapter, you will:

- Understand why sensor simulation is critical for SLAM, perception, and autonomous navigation development
- Configure Gazebo sensor plugins for LiDAR, depth cameras, and IMUs with realistic noise models
- Validate sensor accuracy by comparing simulated data to known ground truth distances
- Visualize sensor data in RViz (laser scans, depth images, point clouds, IMU orientations)
- Understand sensor fusion concepts and coordinate frame transforms
- Debug common sensor simulation issues (missing topics, incorrect ranges, unrealistic noise)

**Prerequisites**: Completed Chapter 1 (Gazebo Physics Simulation), ROS 2 Humble installed, basic understanding of ROS 2 topics and messages

**Time to Complete**: 2-3 hours (installation + tutorial + testing)

**Difficulty**: Intermediate (requires understanding of sensor coordinate systems and ROS 2 message formats)

---

## 1. Introduction to Sensor Simulation

Robotic sensors like LiDAR, depth cameras, and IMUs are expensive ($500-$5000 per unit) and time-consuming to set up. Sensor simulation in Gazebo allows students and developers to:

1. **Prototype algorithms faster**: Test SLAM, object detection, and localization without hardware
2. **Generate perfect ground truth**: Know exact object positions, orientations, and robot poses for algorithm validation
3. **Simulate sensor failures**: Test fault tolerance by adding noise, dropouts, or misalignment
4. **Save costs**: Develop with simulated sensors before purchasing real hardware
5. **Accelerate learning**: Understand sensor behavior through parameter tuning (FOV, range, noise)

### Why Simulate Sensors?

**SLAM Development**: Simultaneous Localization and Mapping (SLAM) algorithms like GMapping, Cartographer, and ORB-SLAM require thousands of test runs to tune parameters. Running these tests on real robots is slow (battery life, manual resets) and risky (collisions). Simulated sensors provide:
- **Repeatable environments**: Same obstacles, same starting pose, deterministic results
- **Faster iteration**: No battery recharging, no manual robot repositioning
- **Perfect ground truth**: Compare SLAM-estimated map to true Gazebo world

**Perception Testing**: Computer vision algorithms (object detection, semantic segmentation) need diverse training data. Simulated depth cameras can generate:
- **Synthetic datasets**: Millions of labeled images with perfect bounding boxes
- **Domain randomization**: Vary lighting, textures, object placements automatically
- **Edge cases**: Simulate rare scenarios (e.g., robot at night, foggy conditions)

**Control System Validation**: IMU-based controllers (e.g., quadcopter stabilization, self-balancing robots) require accurate angular velocity and acceleration measurements. Simulated IMUs provide:
- **Safe testing**: Tune PID gains in simulation before deploying to real robot
- **Noise characterization**: Model sensor bias drift and random walk before integration
- **Failure scenarios**: Test controller response to IMU glitches or calibration errors

### Sensor Types Overview

| Sensor | Physical Principle | ROS 2 Message | Typical Use Cases |
|--------|-------------------|---------------|-------------------|
| **LiDAR** | Time-of-flight laser ranging | `sensor_msgs/LaserScan` | 2D SLAM, obstacle avoidance, wall following |
| **Depth Camera** | Structured light or stereo vision | `sensor_msgs/Image` (depth), `sensor_msgs/PointCloud2` | 3D mapping, object grasping, human detection |
| **IMU** | MEMS accelerometer + gyroscope | `sensor_msgs/Imu` | Orientation estimation, dead reckoning, vibration monitoring |

### Simulation Fidelity Considerations

Gazebo sensor simulation is **not perfect**. Key limitations:

- **Physics timestep**: Gazebo physics runs at 1kHz (1ms timestep), but real sensors may sample at 10kHz (e.g., high-end IMUs)
- **Ray tracing performance**: LiDAR simulation uses GPU ray casting, which is slower than real laser scanners on complex scenes
- **Material properties**: Gazebo doesn't model sensor-specific surface properties (e.g., LiDAR reflectivity varies with material color, angle)
- **Synchronization**: In Gazebo, all sensors are perfectly time-synchronized; real sensors have clock drift

**Best Practice**: Use Gazebo sensors for algorithm development, then validate on real hardware before deployment.

---

## 2. LiDAR Sensor Simulation

LiDAR (Light Detection and Ranging) sensors emit laser beams and measure time-of-flight to calculate distances. Gazebo simulates LiDAR using a **ray sensor plugin** that casts virtual rays and computes intersections with scene geometry.

### Gazebo Ray Sensor Plugin

The `libgazebo_ros_ray_sensor.so` plugin converts Gazebo's ray sensor to ROS 2 `LaserScan` messages. Key configuration parameters:

**Horizontal Scan Parameters**:
- **`samples`**: Number of laser rays per scan (e.g., 640)
- **`min_angle`, `max_angle`**: Angular range in radians (e.g., -2.0944 to +2.0944 = 240°)
- **Angular resolution**: `(max_angle - min_angle) / samples` (e.g., 240° / 640 ≈ 0.375° per ray)

**Range Limits**:
- **`min`**: Minimum detection range (e.g., 0.10m = 10cm, below this returns `inf`)
- **`max`**: Maximum detection range (e.g., 4.0m, beyond this returns `inf`)
- **`resolution`**: Range measurement precision (e.g., 0.01 = 1cm)

**Gaussian Noise Model**:
- **`mean`**: Systematic bias (usually 0.0 for unbiased sensor)
- **`stddev`**: Standard deviation (e.g., 0.01 = 1cm noise, models laser jitter and timing errors)

### Example Configuration (from `humanoid_with_lidar.urdf`)

```xml
<gazebo reference="lidar_link">
  <sensor type="gpu_ray" name="lidar_sensor">
    <update_rate>10.0</update_rate>  <!-- 10 Hz scan rate -->
    <ray>
      <scan>
        <horizontal>
          <samples>640</samples>  <!-- 640 laser beams -->
          <min_angle>-2.0944</min_angle>  <!-- -120° -->
          <max_angle>2.0944</max_angle>   <!-- +120° -->
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>  <!-- 10cm minimum range -->
        <max>4.0</max>   <!-- 4m maximum range -->
      </range>
      <noise>
        <type>gaussian</type>
        <stddev>0.01</stddev>  <!-- 1cm noise -->
      </noise>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=/scan</remapping>
      </ros>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### ROS 2 Topic Output: `sensor_msgs/LaserScan`

**Message Fields**:
```yaml
header:
  stamp: {sec: 10, nanosec: 500000000}
  frame_id: "lidar_link"
angle_min: -2.0944  # Radians (-120°)
angle_max: 2.0944   # Radians (+120°)
angle_increment: 0.006544  # Radians (~0.375°)
time_increment: 0.0  # Time between rays (0 for instantaneous scan)
scan_time: 0.1  # Total scan duration (1 / update_rate = 1 / 10 Hz)
range_min: 0.1  # Meters
range_max: 4.0  # Meters
ranges: [1.5, 1.48, 1.51, ..., inf, inf]  # 640 distance measurements
intensities: []  # Empty (Gazebo doesn't simulate laser intensity)
```

**Interpreting Ranges**:
- **Finite value (e.g., 1.5m)**: Obstacle detected at this distance
- **`inf` (infinity)**: No obstacle within `range_max` (laser ray hit background or nothing)
- **`nan` (not a number)**: Invalid measurement (rare in Gazebo, indicates sensor error)

### Angular Resolution Calculation

Given 640 samples over 240° FOV:
- Angular resolution: `240° / 640 = 0.375° per sample`
- At 1m distance, this translates to: `tan(0.375°) × 1m ≈ 6.5mm` separation between adjacent rays

**Implication**: LiDAR cannot resolve objects smaller than this angular resolution.

---

## 3. Depth Camera Simulation

Depth cameras (also called RGB-D cameras) capture both color images and per-pixel depth information. Gazebo simulates depth cameras using a **camera sensor plugin** with depth buffer rendering.

### Gazebo Camera Plugin for Depth Images

The `libgazebo_ros_camera.so` plugin generates both RGB and depth images. Depth is computed using GPU z-buffer (distance from camera to each pixel).

**Key Configuration Parameters**:

**Image Resolution**:
- **`width`, `height`**: Image dimensions (e.g., 1280×720 = HD 720p)
- Higher resolution → more detail, but slower rendering

**Field of View (FOV)**:
- **`horizontal_fov`**: Horizontal viewing angle in radians (e.g., 1.51844 rad = 87°)
- Vertical FOV calculated from aspect ratio: `vertical_fov = 2 × atan(tan(horizontal_fov/2) × height/width)`

**Clipping Planes (Depth Range)**:
- **`near`**: Minimum depth (e.g., 0.3m, closer objects are clipped)
- **`far`**: Maximum depth (e.g., 10.0m, farther objects are clipped)

**Depth Noise**:
- **`stddev`**: Gaussian noise applied to depth values (e.g., 0.007m = 7mm, models stereo matching errors)

### Example Configuration (from `humanoid_with_depth_camera.urdf`)

```xml
<gazebo reference="camera_link">
  <sensor type="depth" name="depth_camera_sensor">
    <update_rate>30.0</update_rate>  <!-- 30 Hz (video framerate) -->
    <camera>
      <horizontal_fov>1.51844</horizontal_fov>  <!-- 87° -->
      <image>
        <width>1280</width>
        <height>720</height>
      </image>
      <clip>
        <near>0.3</near>  <!-- 30cm minimum depth -->
        <far>10.0</far>   <!-- 10m maximum depth -->
      </clip>
      <noise>
        <stddev>0.007</stddev>  <!-- 7mm depth noise -->
      </noise>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <remapping>depth/image_raw:=/camera/depth/image_raw</remapping>
        <remapping>color/image_raw:=/camera/rgb/image_raw</remapping>
        <remapping>points:=/camera/points</remapping>
      </ros>
      <frame_name>camera_optical_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### ROS 2 Topic Outputs

**1. Color Image (`/camera/rgb/image_raw`)**:
- Message Type: `sensor_msgs/Image`
- Encoding: `rgb8` (24-bit color, 8 bits per channel)
- Dimensions: 1280×720 pixels
- Data: Raw pixel array (width × height × 3 bytes)

**2. Depth Image (`/camera/depth/image_raw`)**:
- Message Type: `sensor_msgs/Image`
- Encoding: `16UC1` (16-bit unsigned integer, single channel)
- Dimensions: 1280×720 pixels
- Data: Depth in millimeters (e.g., value 2000 = 2.0m)

**3. Point Cloud (`/camera/points`)**:
- Message Type: `sensor_msgs/PointCloud2`
- Fields: `x, y, z, rgb` (3D point + color)
- Frame: `camera_optical_frame`
- Generated by unprojecting depth image using camera intrinsics

### Camera Intrinsics

Gazebo automatically calculates camera intrinsics from FOV and resolution:

**Focal Length**:
```
fx = width / (2 × tan(horizontal_fov / 2))
fy = height / (2 × tan(vertical_fov / 2))
```

For 1280×720 @ 87° FOV:
```
fx ≈ 1280 / (2 × tan(1.51844 / 2)) ≈ 730 pixels
fy ≈ 720 / (2 × tan(vertical_fov / 2)) ≈ 730 pixels
```

**Principal Point** (image center):
```
cx = width / 2 = 640 pixels
cy = height / 2 = 360 pixels
```

These intrinsics are published to `/camera/rgb/camera_info` and `/camera/depth/camera_info` topics (message type: `sensor_msgs/CameraInfo`).

---

## 4. IMU Sensor Simulation

Inertial Measurement Units (IMUs) measure orientation, angular velocity, and linear acceleration. Gazebo simulates IMUs using a **physics-based sensor plugin** that reads rigid body dynamics from the ODE physics engine.

### Gazebo IMU Plugin

The `libgazebo_ros_imu_sensor.so` plugin outputs `sensor_msgs/Imu` messages with three components:

**1. Orientation (Quaternion)**:
- Computed from Gazebo rigid body rotation matrix
- Format: `(w, x, y, z)` where `w² + x² + y² + z² = 1.0`
- Represents robot orientation relative to world frame (or initial frame if `initial_orientation_as_reference:=true`)

**2. Angular Velocity (Gyroscope)**:
- Measured in rad/s (radians per second)
- Typical range: ±2000°/s = ±34.9 rad/s
- Noise: Gaussian + bias drift (models MEMS gyroscope imperfections)

**3. Linear Acceleration (Accelerometer)**:
- Measured in m/s² (meters per second squared)
- **Includes gravity**: When robot is stationary, `linear_acceleration.z ≈ 9.81 m/s²` (upward)
- Typical range: ±16g = ±156.96 m/s²
- Noise: Gaussian + bias drift

### Noise Modeling

Real IMUs suffer from two noise types:

**1. White Noise (Random Walk)**:
- Modeled as Gaussian with `stddev` parameter
- Example: Gyroscope stddev = 0.0003 rad/s/√Hz
- Effect: Measurements fluctuate randomly around true value

**2. Bias Drift**:
- Systematic offset that changes slowly over time
- Modeled with `bias_mean` and `bias_stddev`
- Example: Gyroscope bias_mean = 0.00005 rad/s
- Effect: Integrated angular velocity accumulates error (drift)

### Example Configuration (from `humanoid_with_imu.urdf`)

```xml
<gazebo reference="imu_link">
  <gravity>true</gravity>  <!-- Measure gravity as acceleration -->
  <sensor type="imu" name="imu_sensor">
    <update_rate>100.0</update_rate>  <!-- 100 Hz -->
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <stddev>0.0003</stddev>  <!-- 0.0003 rad/s/√Hz -->
            <bias_mean>0.00005</bias_mean>  <!-- 0.00005 rad/s bias -->
          </noise>
        </x>
        <!-- Y and Z axes have identical noise parameters -->
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <stddev>0.002</stddev>  <!-- 0.002 m/s²/√Hz -->
            <bias_mean>0.0001</bias_mean>  <!-- 0.0001 m/s² bias -->
          </noise>
        </x>
        <!-- Y and Z axes have identical noise parameters -->
      </linear_acceleration>
    </imu>
    <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <remapping>~/out:=/imu</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### ROS 2 Topic Output: `sensor_msgs/Imu`

**Message Fields**:
```yaml
header:
  stamp: {sec: 15, nanosec: 250000000}
  frame_id: "imu_link"
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0  # Quaternion (no rotation)
orientation_covariance: [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]  # 3×3 matrix (row-major)
angular_velocity:
  x: 0.0002  # rad/s (gyroscope X-axis)
  y: -0.0001
  z: 0.0003
angular_velocity_covariance: [0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001]
linear_acceleration:
  x: 0.05  # m/s² (includes noise)
  y: -0.02
  z: 9.81  # Gravity (robot upright, stationary)
linear_acceleration_covariance: [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
```

**Covariance Matrices**: Represent measurement uncertainty (diagonal elements are variances, off-diagonal are correlations). Gazebo populates these based on noise parameters.

### Quaternion to Euler Conversion

ROS 2 uses quaternions for orientation, but humans think in Euler angles (roll, pitch, yaw). Conversion:

```python
import math

def quaternion_to_euler(x, y, z, w):
    # Roll (rotation around X-axis)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation around Y-axis)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp)

    # Yaw (rotation around Z-axis)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw  # Radians
```

**ROS 2 CLI**:
```bash
ros2 topic echo /imu | python3 -c "
import sys, math
for line in sys.stdin:
    if 'orientation:' in line:
        q = [float(sys.stdin.readline().split(':')[1]) for _ in range(4)]
        # Convert q[x,y,z,w] to Euler and print
"
```

---

## 5. Validating Sensor Data

Simulation is only useful if sensor data is accurate. This section covers hands-on validation tests.

### LiDAR Range Accuracy Test

**Objective**: Verify LiDAR measures correct distances to obstacles.

**Procedure**:
1. Launch LiDAR example: `ros2 launch launch/sensor_demo.launch.py sensor_type:=lidar`
2. In Gazebo GUI: **Insert → Model → Box**
3. Use **Translation Mode** to place box exactly 1.0 meter in front of robot (Y = 1.0)
4. Inspect scan data:
   ```bash
   ros2 topic echo /scan --once | grep "ranges:"
   ```
5. **Expected**: Central range value (index 320 of 640) ≈ 1.0m

**Acceptance Criteria** (from spec SC-006):
- Error < 5cm for objects 0.5-3.0m away
- If measured range = 1.02m, error = 2cm ✅ PASS

**Troubleshooting**:
- If range shows `inf`: Box is outside 4m max range or closer than 0.1m min range
- If error > 5cm: Check noise stddev in URDF (should be 0.01m)

### Depth Camera Calibration Check

**Objective**: Verify depth image accuracy.

**Procedure**:
1. Launch depth camera example: `ros2 launch launch/sensor_demo.launch.py sensor_type:=depth`
2. Insert sphere (radius 0.5m) at known distance (e.g., 2.0m) from camera
3. View depth image in RViz:
   ```bash
   rviz2
   # Add → Image → Topic: /camera/depth/image_raw
   ```
4. Click on sphere center pixel, note depth value (in millimeters)
5. **Expected**: Depth ≈ 2000mm ± 200mm (10% error, per spec SC-007)

**Acceptance Criteria**:
- Depth error < 10% for objects 0.5-5.0m away
- Example: Measured 2050mm, true 2000mm → error = 2.5% ✅ PASS

### IMU Orientation Test

**Objective**: Verify IMU quaternion changes with robot rotation.

**Procedure**:
1. Launch IMU example: `ros2 launch launch/sensor_demo.launch.py sensor_type:=imu`
2. Initial state (robot upright):
   ```bash
   ros2 topic echo /imu | grep "orientation:" -A 4
   # Expected: w ≈ 1.0, x ≈ 0, y ≈ 0, z ≈ 0
   ```
3. In Gazebo, use **Rotation Mode** to tilt robot 90° around X-axis (roll)
4. **Expected**: Quaternion changes to approximate `(w=0.707, x=0.707, y=0, z=0)`
5. Reset robot: **Edit → Reset Model Poses**

**Note**: For exact quaternion values, use quaternion calculator: https://www.andre-gaschler.com/rotationconverter/

---

## 6. Sensor Fusion Concepts

Real robots use multiple sensors simultaneously. **Sensor fusion** combines data from LiDAR, depth cameras, and IMUs to achieve better localization and mapping than any single sensor.

### Multi-Sensor SLAM Example

**Scenario**: Autonomous mobile robot with LiDAR + IMU navigation.

**Sensor Roles**:
- **LiDAR**: Provides 2D map of environment (walls, obstacles)
- **IMU**: Provides orientation and angular velocity for dead reckoning between LiDAR scans

**Fusion Algorithm (Extended Kalman Filter)**:
1. **Prediction Step**: Use IMU angular velocity to predict robot orientation change
2. **Update Step**: Use LiDAR scan matching to correct position drift
3. **Output**: Fused pose estimate (better than LiDAR-only or IMU-only)

**ROS 2 Implementation** (using `robot_localization` package):
```bash
sudo apt install ros-humble-robot-localization

# Configure EKF to fuse /imu and /scan
# Edit ekf_config.yaml:
#   imu0: /imu
#   imu0_config: [false, false, false, true, true, true, ...]  # Use orientation + angular velocity
#   odom0: /odom  # Generated from LiDAR scan matching

ros2 run robot_localization ekf_node --ros-args --params-file ekf_config.yaml
```

### Coordinate Frame Transforms (/tf Tree)

Sensors publish data in their own frames (e.g., `lidar_link`, `camera_optical_frame`). To fuse data, we need transforms between frames.

**Example TF Tree**:
```
map → odom → base_link → lidar_link
                      → camera_link → camera_optical_frame
                      → imu_link
```

**Visualizing TF Tree**:
```bash
# Install tf2_tools
sudo apt install ros-humble-tf2-tools

# Print TF tree
ros2 run tf2_tools view_frames
# Generates frames.pdf showing all coordinate frames

# Check transform between two frames
ros2 run tf2_ros tf2_echo base_link lidar_link
# Expected: Translation (0, 0, 1.05), Rotation (0, 0, 0, 1)
```

### Synchronized Data Capture

Sensor data arrives asynchronously. For fusion, we need time-synchronized measurements.

**ROS 2 Message Filters** (`message_filters` package):
```python
from message_filters import ApproximateTimeSynchronizer, Subscriber

# Subscribe to LiDAR and IMU
scan_sub = Subscriber('/scan', LaserScan)
imu_sub = Subscriber('/imu', Imu)

# Synchronize with 0.1 second tolerance
sync = ApproximateTimeSynchronizer([scan_sub, imu_sub], queue_size=10, slop=0.1)
sync.registerCallback(fusion_callback)

def fusion_callback(scan_msg, imu_msg):
    # Both messages have timestamps within 0.1 seconds
    print(f"Synchronized: LiDAR at {scan_msg.header.stamp}, IMU at {imu_msg.header.stamp}")
```

---

## 7. Troubleshooting

### Issue: "/scan topic not publishing"

**Symptoms**: `ros2 topic list` does not show `/scan` after launching LiDAR example

**Cause**: LiDAR plugin not loaded, or URDF syntax error

**Fix**:
1. Check Gazebo terminal for plugin errors:
   ```
   [ERROR] Failed to load plugin libgazebo_ros_ray_sensor.so
   ```
2. Install plugin package:
   ```bash
   sudo apt install ros-humble-gazebo-plugins
   ```
3. Verify URDF syntax:
   ```bash
   check_urdf humanoid_with_lidar.urdf
   ```
4. Restart Gazebo

---

### Issue: "Depth camera shows blank images"

**Symptoms**: `/camera/rgb/image_raw` topic exists, but RViz shows black image

**Cause**: No objects in camera FOV, or near/far clipping incorrect

**Fix**:
1. Add objects to scene (Gazebo → Insert → Model → Table)
2. Position objects 1-5m from camera (within 0.3-10m depth range)
3. Check camera is facing objects (green frustum should point toward scene)

---

### Issue: "IMU shows zero acceleration instead of gravity"

**Symptoms**: `linear_acceleration.z = 0.0` instead of `9.81 m/s²`

**Cause**: `<gravity>true</gravity>` missing in URDF

**Fix**:
1. Edit `humanoid_with_imu.urdf`, add to `<gazebo reference="imu_link">`:
   ```xml
   <gravity>true</gravity>
   ```
2. Restart Gazebo

---

## 8. Key Takeaways

This chapter covered sensor simulation in Gazebo for robotics development:

**Core Concepts**:
1. **Sensor Plugins**: Gazebo uses plugins (`libgazebo_ros_ray_sensor.so`, `libgazebo_ros_camera.so`, `libgazebo_ros_imu_sensor.so`) to generate ROS 2 sensor messages
2. **Noise Models**: Gaussian noise (stddev) and bias drift (bias_mean/stddev) simulate real sensor imperfections
3. **ROS 2 Integration**: Sensors publish to standard topics (`/scan`, `/camera/depth/image_raw`, `/imu`) with standard message types
4. **Validation**: Compare simulated sensor data to known ground truth (obstacle distances, robot orientation) to verify accuracy
5. **Sensor Fusion**: Combine LiDAR, depth cameras, and IMUs using TF transforms and synchronized callbacks for robust localization

**When to Use Each Sensor**:
- **LiDAR**: 2D SLAM (GMapping, Cartographer), wall following, obstacle avoidance
- **Depth Camera**: 3D object grasping, human detection, visual SLAM (ORB-SLAM3)
- **IMU**: Orientation estimation (complementary filter, Madgwick filter), dead reckoning, vibration monitoring

**Simulation vs Real Hardware**:
- **Advantages**: Repeatable, perfect ground truth, safe testing, cost-effective
- **Limitations**: No real-world noise (dust on lens, vibration), simplified material properties, perfect time synchronization

**Best Practice**: Develop algorithms in simulation with realistic noise models, then validate on real hardware before deployment.

---

## Practice Exercises

1. **Modify LiDAR FOV**: Edit `humanoid_with_lidar.urdf` to create a 360° LiDAR (min_angle: -π, max_angle: +π). Increase samples to 1080 to maintain angular resolution. Launch and verify full circular scan in RViz.

2. **Depth Camera Super-Resolution**: Change depth camera resolution to 1920×1080 (Full HD). Measure FPS drop in Gazebo Stats panel (Window → Stats). What is the performance cost of 2x pixel count?

3. **IMU Noise Tuning**: Increase gyroscope stddev to 0.01 rad/s (10× higher noise). Launch IMU example, rotate robot, observe orientation drift in RViz. Compare to baseline stddev=0.0003.

4. **Multi-Sensor Robot**: Create `humanoid_with_all_sensors.urdf` combining LiDAR + depth camera + IMU. Launch in Gazebo, verify all topics publish simultaneously. Visualize all sensors in single RViz window.

5. **Range Accuracy Benchmark**: Place boxes at 0.5m, 1.0m, 1.5m, 2.0m distances from LiDAR. Record measured ranges, calculate mean error and standard deviation. Does error increase with distance?

---

## Download Example Files

📦 **[gazebo_sensors.zip](/examples/gazebo_sensors.zip)** - LiDAR, depth camera, and IMU sensor examples

**Contents**:
- `humanoid_with_lidar.urdf` - 2D LiDAR configuration (240° FOV, 4m range, 10 Hz)
- `humanoid_with_depth_camera.urdf` - RGB-D camera configuration (1280×720, 30 Hz)
- `humanoid_with_imu.urdf` - 6-DOF IMU configuration (100 Hz, realistic noise)
- `launch/sensor_demo.launch.py` - Launch file with sensor selection argument
- `README.md` - Setup instructions, verification steps, troubleshooting

---

## References

1. **Koenig, N. & Howard, A.** (2004). "Design and Use Paradigms for Gazebo, An Open-Source Multi-Robot Simulator." *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 2149-2154. DOI: 10.1109/IROS.2004.1389727

2. **Ivaldi, S., et al.** (2016). "Tools for Dynamics Simulation of Robots: A Survey Based on User Feedback." *Robotics and Autonomous Systems*, 74, 1-21. DOI: 10.1016/j.robot.2015.08.011 [Open Access: https://arxiv.org/abs/1402.7050]

3. **Zhang, J. & Singh, S.** (2017). "Low-drift and Real-time Lidar Odometry and Mapping." *Autonomous Robots*, 41(2), 401-416. DOI: 10.1007/s10514-016-9548-2 [Open Access: https://frc.ri.cmu.edu/~zhangji/publications/AR_2017.pdf]

4. **Gazebo Classic** (2023). "Sensor Simulation Tutorial." Official Documentation. http://classic.gazebosim.org/tutorials?cat=sensors

---

**Next Chapter**: Module 3 - Control Systems (Coming in future releases)

**Module 2 Complete!** You've learned physics simulation (Gazebo), rendering (Unity), and sensor simulation. You're now ready to develop SLAM, perception, and control algorithms.
