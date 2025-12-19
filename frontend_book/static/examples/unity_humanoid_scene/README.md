# Unity Humanoid Scene Example

This example demonstrates photorealistic rendering of the `simple_humanoid` robot in Unity with ROS 2 bridge integration for **Module 2 - Chapter 2: Unity Rendering**.

## ⚠️ Important Notes

- **Priority**: P2 (Optional) - This chapter is for advanced students with higher-end hardware
- **Hardware Requirements**: 16GB RAM, dedicated GPU (NVIDIA GTX 1050 or better recommended)
- **Alternative**: If Unity is too resource-intensive, use Gazebo rendering from Chapter 1 instead
- **Purpose**: Unity provides photorealistic visualization for human-robot interaction (HRI) demos, perception testing, and marketing materials

## Contents

- `UnityProject/` - Unity 2021 LTS project directory
  - `Assets/Scenes/` - Unity scene files (to be created in Unity Editor)
  - `Assets/Scripts/` - C# scripts for ROS 2 integration
    - `ROSCameraPublisher.cs` - Publishes camera images to ROS 2
    - `ROSJointSubscriber.cs` - Subscribes to joint states from ROS 2
  - `Assets/Robots/simple_humanoid/` - Robot prefab directory (URDF import target)
  - `Packages/manifest.json` - Unity package dependencies (ROS-TCP-Connector)
- `README.md` - This file (setup instructions)

## Prerequisites

Before running this example, ensure you have:

1. **Unity 2021 LTS** installed (Unity 2021.3.x recommended)
   - Download: https://unity.com/releases/editor/archive
   - Choose Unity 2021.3 LTS → Unity Hub or Direct Download
   - Install with modules: Windows/Mac/Linux Build Support, Universal Render Pipeline

2. **Unity Hub** (optional but recommended)
   - Download: https://unity.com/download
   - Simplifies Unity version management

3. **ROS 2 Humble** installed and sourced (from Module 1)
   ```bash
   source /opt/ros/humble/setup.bash
   ```

4. **ROS-TCP-Endpoint** installed
   ```bash
   sudo apt install ros-humble-ros-tcp-endpoint
   # Or install from source:
   # cd ~/ros2_ws/src
   # git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint
   # cd ~/ros2_ws && colcon build --packages-select ros_tcp_endpoint
   ```

5. **simple_humanoid URDF** from Module 1
   - Located in: `ros2_workspace/src/simple_humanoid_description/urdf/simple_humanoid.urdf`

## Quick Start

### Step 1: Open Unity Project

```bash
# Extract this example (if downloaded as .zip)
unzip unity_humanoid_scene.zip
cd unity_humanoid_scene

# Open with Unity Hub:
# 1. Open Unity Hub
# 2. Click "Add" → Select "UnityProject/" folder
# 3. Click on project to open in Unity 2021 LTS

# Or open directly with Unity Editor:
unity-editor -projectPath "$(pwd)/UnityProject"
```

**Expected Result**: Unity Editor opens with the project. Package Manager will automatically install ROS-TCP-Connector and URDF-Importer (this may take 2-5 minutes on first launch).

### Step 2: Import URDF into Unity

Since Unity scenes are binary files, you'll need to manually import the URDF and create the scene:

1. **Import URDF Package** (if not auto-installed):
   - In Unity Editor: Window → Package Manager
   - Click "+" → "Add package from git URL"
   - Enter: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`
   - Click "Add"

2. **Import simple_humanoid URDF**:
   - In Unity Editor: Assets → Import Robot from URDF
   - Browse to: `ros2_workspace/src/simple_humanoid_description/urdf/simple_humanoid.urdf`
   - Click "Import"
   - **Expected**: `simple_humanoid` GameObject appears in Hierarchy with all links and joints

3. **Configure ArticulationBody**:
   - Select `simple_humanoid` in Hierarchy
   - In Inspector: Add Component → ArticulationBody
   - Set "Immovable" to **true** (root link should be fixed)
   - For each child link (shoulders, elbows):
     - Add Component → ArticulationBody
     - Set Joint Type to "Revolute"
     - Configure joint limits (e.g., -90° to +90° for shoulders)

### Step 3: Create Unity Scene

1. **Create New Scene**:
   - File → New Scene → 3D (Universal Render Pipeline)
   - Save as: `Assets/Scenes/HumanoidDemo.unity`

2. **Add Ground Plane**:
   - Right-click in Hierarchy → 3D Object → Plane
   - Rename to "Ground"
   - Scale: (10, 1, 10)
   - Position: (0, 0, 0)

3. **Add Humanoid Robot**:
   - Drag `simple_humanoid` prefab from Project window to Hierarchy
   - Position: (0, 1, 0) - 1 meter above ground

4. **Configure Lighting**:
   - Select "Directional Light" in Hierarchy
   - In Inspector:
     - Intensity: 1.5
     - Shadow Type: Soft Shadows
     - Rotation: (50, -30, 0) for realistic lighting angle
   - Window → Rendering → Lighting
     - Environment → Skybox Material: Default-Skybox
     - Ambient Source: Skybox
     - Click "Generate Lighting" (bottom right)

5. **Configure Main Camera**:
   - Select "Main Camera" in Hierarchy
   - Position: (3, 2, 3)
   - Rotation: (15, -45, 0) - angled to view robot
   - **Add ROSCameraPublisher script**:
     - Click "Add Component" → Search "ROSCameraPublisher"
     - Set Image Width: 640
     - Set Image Height: 480
     - Set Publish Rate: 10 Hz

6. **Add ROS Joint Subscriber to Robot**:
   - Select `simple_humanoid` root in Hierarchy
   - Click "Add Component" → Search "ROSJointSubscriber"
   - In Inspector:
     - Drag each ArticulationBody (shoulders, elbows) into "Articulation Bodies" array
     - Set Topic Name: `/joint_states`
     - Enable Debug Mode (for testing)

### Step 4: Configure ROS-TCP-Connector

1. **Open ROS Settings**:
   - In Unity Editor: Robotics → ROS Settings

2. **Configure TCP Endpoint**:
   - Protocol: ROS2
   - ROS IP Address: `127.0.0.1` (localhost)
   - ROS Port: `10000` (default)
   - Show HUD: **Enabled** (to see connection status)

3. **Save Settings**:
   - File → Save Project

### Step 5: Start ROS 2 TCP Endpoint

In a new terminal (Ubuntu or WSL2):

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Start ROS-TCP-Endpoint server
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Expected Output:
# [INFO] [ros_tcp_endpoint]: ROS-TCP Endpoint started on port 10000
```

**Keep this terminal running** - Unity will connect to this endpoint.

### Step 6: Test Unity Scene

1. **Press Play** in Unity Editor (top center ▶ button)

2. **Verify Connection**:
   - Check Unity Console (Window → General → Console)
   - **Expected**: `ROSCameraPublisher: Initialized - Publishing to '/camera/rgb/image_raw' at 10 Hz`
   - **Expected**: `ROSJointSubscriber: Subscribed to '/joint_states' - Monitoring 4 joints`
   - **Expected**: ROS connection HUD in Game view shows "Connected"

3. **Verify Camera Publishing**:
   ```bash
   # In another terminal
   ros2 topic list
   # Expected: /camera/rgb/image_raw

   ros2 topic hz /camera/rgb/image_raw
   # Expected: ~10 Hz
   ```

4. **Verify Joint Subscription**:
   ```bash
   # Publish test joint command
   ros2 topic pub /joint_states sensor_msgs/msg/JointState \
   "{name: ['left_shoulder_joint'], position: [0.5]}" --once

   # Expected: Robot's left shoulder rotates in Unity scene
   ```

5. **Visualize in RViz** (optional):
   ```bash
   rviz2
   # Add → Image → Topic: /camera/rgb/image_raw
   # Expected: Unity camera view appears in RViz
   ```

## Verification Steps

### 1. Check Unity Console Logs

**Success Indicators**:
```
ROSCameraPublisher: Initialized - Publishing to '/camera/rgb/image_raw' at 10 Hz
ROSJointSubscriber: Subscribed to '/joint_states' - Monitoring 4 joints
ROS-TCP-Connector: Connected to 127.0.0.1:10000
```

**Error Indicators**:
```
[ERROR] ROS-TCP-Connector: Connection refused
Fix: Ensure ros2 run ros_tcp_endpoint is running

[WARN] ROSJointSubscriber: No ArticulationBodies assigned in Inspector
Fix: Drag ArticulationBody components into script inspector

[ERROR] ROSCameraPublisher: No Camera component found
Fix: Attach script to GameObject with Camera component
```

### 2. Test ROS 2 Topics

```bash
# List all topics (should include Unity topics)
ros2 topic list

# Monitor camera images
ros2 topic hz /camera/rgb/image_raw
# Expected: average rate: 10.000 Hz

# Echo joint states (will show Unity-published joint positions)
ros2 topic echo /joint_states --once
```

### 3. Test Rendering Quality

In Unity Game view (during Play mode):
- ✅ Robot renders with smooth shading (no pixelation)
- ✅ Shadows appear under robot on ground plane
- ✅ Lighting creates realistic highlights on robot surfaces
- ✅ Framerate ≥30 FPS (check Stats panel: Window → Analysis → Stats)

## Troubleshooting

### Issue: "ROS-TCP-Connector: Connection refused"

**Cause**: ROS 2 TCP endpoint not running or wrong IP/port

**Fix**:
1. Start endpoint in terminal:
   ```bash
   ros2 run ros_tcp_endpoint default_server_endpoint
   ```
2. Verify IP in Unity (Robotics → ROS Settings):
   - ROS IP Address: `127.0.0.1` (for localhost)
   - If Unity is on different machine: use Ubuntu machine's IP (e.g., `192.168.1.100`)
   - ROS Port: `10000`

---

### Issue: "URDF Importer package not found"

**Cause**: ROS-TCP-Connector or URDF-Importer packages not installed

**Fix**:
1. Open Unity Package Manager (Window → Package Manager)
2. Click "+" → "Add package from git URL"
3. Add ROS-TCP-Connector:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   ```
4. Add URDF-Importer:
   ```
   https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
   ```
5. Wait for packages to install (check progress in bottom-right of Unity Editor)

---

### Issue: "Unity is very slow (< 10 FPS)"

**Cause**: Insufficient GPU, high graphics settings, or too many lights/shadows

**Fix**:
1. **Reduce Quality Settings**:
   - Edit → Project Settings → Quality
   - Select "Low" or "Medium" preset
   - Disable: Anti-Aliasing, Soft Particles, Realtime Reflections

2. **Optimize Lighting**:
   - Disable real-time shadows: Directional Light → Shadow Type: No Shadows
   - Reduce shadow distance: Edit → Project Settings → Quality → Shadow Distance: 50

3. **Use Headless Mode** (no rendering, ROS bridge only):
   - Create build: File → Build Settings
   - Check "Server Build" (Linux) or "Headless Mode" (Windows)
   - Build and run executable in terminal

4. **Switch to Gazebo**: If Unity remains too slow, use Gazebo from Chapter 1 (physics + basic rendering)

---

### Issue: "Robot joints are not moving when publishing to /joint_states"

**Cause**: ArticulationBody not configured, or joint names don't match URDF

**Fix**:
1. Select robot in Hierarchy
2. Check ROSJointSubscriber script in Inspector:
   - Verify "Articulation Bodies" array has all joints (not empty)
   - Enable "Debug Mode"
3. In Unity Console, look for:
   ```
   ROSJointSubscriber: Unknown joint 'left_shoulder_joint' - ignoring
   ```
4. Match GameObject names to URDF joint names:
   - In Hierarchy, rename GameObject to exact URDF joint name
   - Example: "left_shoulder_link" → "left_shoulder_joint"

---

### Issue: "Camera image is black or not publishing"

**Cause**: Camera not rendering, or RenderTexture not assigned

**Fix**:
1. Select Main Camera in Hierarchy
2. Check Camera component in Inspector:
   - Target Texture: Should be auto-assigned by ROSCameraPublisher (or leave None)
   - Culling Mask: Everything (not "Nothing")
3. Check ROSCameraPublisher script:
   - Image Width/Height: 640x480 (not 0)
   - Publish Rate: 10 Hz (not 0)
4. In Unity Console, verify:
   ```
   ROSCameraPublisher: Initialized - Publishing to '/camera/rgb/image_raw' at 10 Hz
   ```

---

## Hardware Requirements

**Minimum** (for Unity rendering):
- 16GB RAM
- NVIDIA GTX 1050 / AMD RX 560 or better (dedicated GPU required)
- Ubuntu 22.04, Windows 10/11, or macOS (Unity 2021 LTS supported)
- 4 CPU cores

**Recommended**:
- 32GB RAM
- NVIDIA RTX 3060 / AMD RX 6700 XT or better
- 8+ CPU cores for real-time rendering at 60 FPS

**Note**: If your hardware doesn't meet these requirements, **skip Chapter 2** and use Gazebo (Chapter 1) for visualization.

---

## Next Steps

After successfully running this example:

1. **Read Chapter 2** of Module 2 to understand:
   - Unity rendering pipeline (lighting, materials, post-processing)
   - ROS-Unity bridge architecture (TCP/IP, message serialization)
   - ArticulationBody vs Gazebo physics comparison
   - When to use Unity vs Gazebo

2. **Experiment with Rendering**:
   - Modify Directional Light intensity and rotation
   - Change Skybox: Window → Rendering → Lighting → Environment → Skybox Material
   - Add post-processing effects: Window → Package Manager → Install "Post Processing" package
   - Import custom materials from Unity Asset Store

3. **Test ROS Integration**:
   - Publish continuous joint commands from ROS 2 Python node
   - Visualize camera feed in RViz alongside Gazebo simulation
   - Create synchronized dual rendering (Gazebo physics + Unity visuals)

4. **Proceed to Chapter 3**: Sensor Simulation (LiDAR, depth cameras, IMU)

---

## Comparison: Unity vs Gazebo

| Feature | Gazebo (Chapter 1) | Unity (Chapter 2) |
|---------|-------------------|-------------------|
| **Physics Simulation** | ✅ Excellent (ODE, Bullet engines) | ⚠️ Basic (PhysX, not robotics-optimized) |
| **Rendering Quality** | ⚠️ Basic (OpenGL, limited materials) | ✅ Photorealistic (URP, HDRP pipelines) |
| **Sensor Simulation** | ✅ LiDAR, depth cameras, IMU plugins | ⚠️ Cameras only (requires custom scripts) |
| **ROS 2 Integration** | ✅ Native (gazebo_ros_pkgs) | ⚠️ Via TCP bridge (ROS-TCP-Connector) |
| **Hardware Requirements** | ✅ Low (8GB RAM, integrated GPU) | ❌ High (16GB RAM, dedicated GPU) |
| **Use Cases** | Physics testing, SLAM, path planning | HRI demos, perception, marketing videos |

**Recommendation**: Use Gazebo as primary simulator (P1), Unity for visualization only (P2, optional).

---

## Support

For issues or questions:
- **Module 2 Documentation**: See Chapter 2 for detailed explanations
- **Unity Robotics Hub**: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- **ROS-TCP-Connector Docs**: https://github.com/Unity-Technologies/ROS-TCP-Connector
- **Unity Forums**: https://forum.unity.com/forums/robotics.623/

---

**Author**: AI-Driven Robotics Textbook
**Module**: 2 - The Digital Twin (Gazebo & Unity)
**Chapter**: 2 - Unity Rendering
**Last Updated**: 2025-12-19
