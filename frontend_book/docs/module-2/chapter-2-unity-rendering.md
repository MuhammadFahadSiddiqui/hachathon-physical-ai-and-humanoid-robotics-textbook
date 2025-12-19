---
title: Chapter 2 - Unity Rendering
sidebar_position: 3
---

# Chapter 2: Unity Rendering – High-Fidelity Visualization

:::warning Priority P2 - Optional Content
This chapter is **optional** for students with high-end hardware (16GB RAM, dedicated GPU). If Unity is too resource-intensive for your system, **skip this chapter** and use Gazebo rendering from Chapter 1 instead. Unity is ideal for human-robot interaction (HRI) demos, perception testing, and marketing materials, but not required for core robotics learning.
:::

## Learning Objectives

By the end of this chapter, you will:

- Understand Unity's role in robotic visualization and when to choose Unity over Gazebo
- Install Unity 2021 LTS and configure the ROS-TCP-Connector package for ROS 2 integration
- Import URDF robot models into Unity and configure ArticulationBody components for joints
- Apply photorealistic rendering techniques (lighting, materials, shadows, post-processing)
- Build a Unity-ROS 2 bridge to publish camera images and subscribe to joint states
- Test and debug Unity-ROS integration with verification commands

**Prerequisites**: Completed Chapter 1 (Gazebo Physics Simulation), ROS 2 Humble installed, `simple_humanoid.urdf` from Module 1

**Time to Complete**: 3-4 hours (installation + tutorial + testing)

**Difficulty**: Advanced (requires 3D graphics understanding and Unity Editor familiarity)

---

## 1. Introduction to Unity for Robotics

Unity is a cross-platform game engine widely used in robotics for **photorealistic visualization**, **human-robot interaction (HRI) simulations**, and **synthetic data generation for perception**. While Gazebo excels at physics-based simulation, Unity specializes in high-fidelity rendering, making it ideal for applications where visual realism matters more than physics accuracy.

### What is Unity?

Unity is a **real-time 3D development platform** originally designed for game development, now extensively adopted in robotics, autonomous vehicles, and digital twins. Unity provides:

- **Universal Render Pipeline (URP)**: Optimized rendering for real-time graphics with realistic lighting, shadows, and reflections
- **High Dynamic Range Imaging (HDRI)**: Environment-based lighting for photorealistic scenes
- **Post-Processing**: Visual effects like depth of field, motion blur, color grading, and bloom
- **Cross-Platform Deployment**: Build for Windows, macOS, Linux, WebGL, mobile devices

For robotics, Unity integrates with ROS 2 via the **ROS-TCP-Connector** package, which bridges Unity GameObjects to ROS 2 topics over TCP/IP.

### Unity vs Gazebo: When to Use Each

| Criteria | Gazebo (Chapter 1) | Unity (Chapter 2) |
|----------|-------------------|-------------------|
| **Physics Accuracy** | ✅ Excellent (ODE, Bullet, Simbody engines) | ⚠️ Basic (PhysX, not robotics-optimized) |
| **Rendering Quality** | ⚠️ Basic OpenGL (limited materials, simple shadows) | ✅ Photorealistic (URP/HDRP, PBR materials, global illumination) |
| **Sensor Simulation** | ✅ LiDAR, depth cameras, IMU, GPS (native plugins) | ⚠️ RGB cameras only (depth/LiDAR require custom shaders) |
| **ROS 2 Integration** | ✅ Native (gazebo_ros_pkgs, zero latency) | ⚠️ TCP bridge (100-200ms latency, network overhead) |
| **Hardware Requirements** | ✅ Low (8GB RAM, integrated GPU sufficient) | ❌ High (16GB+ RAM, dedicated GPU required for 30+ FPS) |
| **Primary Use Cases** | Physics simulation, SLAM, path planning, control testing | HRI demos, perception datasets, marketing videos, VR/AR |

**Key Insight**: Use **Gazebo for development and testing** (accurate physics, low latency), then use **Unity for visualization and deployment** (realistic rendering, public demos). Advanced workflows run Gazebo physics in headless mode while Unity renders the same scene via ROS 2 bridge.

### When Unity Adds Value

Unity is particularly valuable for:

1. **Human-Robot Interaction (HRI)**: Realistic avatars, facial expressions, hand gestures for social robots
2. **Perception Research**: Generating synthetic datasets with perfect ground truth (object poses, segmentation masks)
3. **Marketing and Demos**: High-quality videos and interactive experiences for stakeholders
4. **Virtual Reality (VR)**: Immersive teleoperation interfaces with Oculus/Vive headsets
5. **Sim-to-Real Transfer**: Training vision models on Unity-rendered data, deploying to real robots

---

## 2. Installing Unity 2021 LTS

Unity 2021 LTS (Long-Term Support) is the recommended version for ROS 2 robotics projects. It provides stable ROS-TCP-Connector support and compatibility with Ubuntu 22.04.

### Step 1: Download Unity Hub

Unity Hub is a management tool that simplifies installing multiple Unity versions and organizing projects.

**Windows/macOS**:
1. Visit: https://unity.com/download
2. Click "Download Unity Hub"
3. Run installer (Unity Hub 3.x)

**Linux (Ubuntu 22.04)**:
```bash
# Download Unity Hub AppImage
wget -O UnityHub.AppImage https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

# Make executable
chmod +x UnityHub.AppImage

# Run Unity Hub
./UnityHub.AppImage
```

### Step 2: Install Unity 2021.3 LTS

1. Open Unity Hub
2. Navigate to **Installs** tab
3. Click **Install Editor** → **Official Releases** → Select **2021.3.x LTS** (latest 2021.3 version)
4. Choose modules to install:
   - ✅ **Linux Build Support** (for Ubuntu)
   - ✅ **Windows Build Support** (if on Windows)
   - ✅ **Universal Render Pipeline** (included by default)
   - ✅ **Documentation** (optional, for offline reference)
5. Click **Install** (download size: ~5 GB, requires 15+ GB disk space)

**Installation time**: 15-30 minutes depending on internet speed.

### Step 3: Verify Unity Installation

```bash
# Check Unity version
unity-editor --version
# Expected: 2021.3.x LTS

# Create test project
unity-editor -createProject ~/UnityTestProject -quit
ls ~/UnityTestProject
# Expected: Assets/, Packages/, ProjectSettings/ directories
```

### Step 4: Install ROS-TCP-Connector Package

The ROS-TCP-Connector enables Unity to communicate with ROS 2 via TCP/IP. Unlike Gazebo's native ROS integration, Unity requires this external package.

**Installation via Unity Package Manager**:
1. Open Unity project (or create new 3D project)
2. Navigate: **Window → Package Manager**
3. Click **+ (plus icon)** → **Add package from git URL**
4. Enter URL:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   ```
5. Click **Add**
6. Wait for package import (1-2 minutes)
7. Repeat for URDF Importer package:
   ```
   https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
   ```

**Verification**:
- In Unity Project window: `Packages/ROS-TCP-Connector` and `Packages/URDF-Importer` should appear
- Top menu bar: **Robotics** menu should be available

---

## 3. Importing URDF into Unity

Unity's URDF Importer package converts ROS robot descriptions (URDF files) into Unity GameObjects with ArticulationBody components for physics and joints.

### Understanding URDF to Unity Conversion

URDF and Unity use different conventions:

| URDF Element | Unity Equivalent | Conversion Notes |
|--------------|------------------|------------------|
| `<link>` | GameObject with MeshRenderer | Visual/collision geometries become Unity meshes |
| `<joint>` | ArticulationBody component | Revolute → ArticulationJointType.RevoluteJoint |
| `<inertial>` | ArticulationBody mass/inertia | Mass in kg, inertia tensor (3x3 matrix) |
| `<material>` | Material asset (PBR shader) | Colors converted, textures require manual assignment |
| Coordinate System | Left-handed Y-up (Unity) vs Right-handed Z-up (ROS) | Automatic conversion by URDF Importer |

### Step-by-Step URDF Import

Assume you have `simple_humanoid.urdf` from Module 1 at:
```
~/ros2_workspace/src/simple_humanoid_description/urdf/simple_humanoid.urdf
```

**1. Import URDF File**:
- In Unity: **Assets → Import Robot from URDF**
- **Browse** to URDF file location
- Click **Import URDF**
- **Expected**: Unity creates a GameObject hierarchy matching URDF structure:
  ```
  simple_humanoid (root GameObject)
  ├── base_link (torso)
  │   ├── left_shoulder_link
  │   │   └── left_elbow_link
  │   └── right_shoulder_link
  │       └── right_elbow_link
  ```

**2. Configure ArticulationBody for Root Link**:
- Select `simple_humanoid` (root GameObject) in Hierarchy
- In Inspector: **Add Component** → **ArticulationBody**
- Set **Immovable**: ✅ Enabled (root link is fixed in space)
- **Articulation Body Type**: Fixed
- **Mass**: 10 kg (from URDF `<inertial>` tag)

**3. Configure ArticulationBody for Joints**:

For each child link (e.g., `left_shoulder_link`):
- Select link in Hierarchy
- ArticulationBody should be auto-added by URDF Importer
- Verify settings:
  - **Joint Type**: Revolute (for rotational joints)
  - **Anchor Rotation**: Auto-configured from URDF `<origin>` tag
  - **X Drive** (joint limits):
    - **Lower Limit**: -90° (from URDF `<limit lower="-1.57"/>`, converted to degrees)
    - **Upper Limit**: 90°
    - **Stiffness**: 10000 (joint resistance)
    - **Damping**: 100 (joint friction)

**4. Assign Materials (Optional)**:

Unity imports URDF materials as basic colors. For photorealism:
- Create new Material: **Right-click in Project → Create → Material**
- Name: `HumanoidMetal`
- Shader: **Universal Render Pipeline/Lit** (PBR shader)
- Set properties:
  - **Base Map**: Light grey color (#CCCCCC)
  - **Metallic**: 0.8 (metallic surface)
  - **Smoothness**: 0.6 (slightly reflective)
- Drag material onto robot meshes in Scene view

### Troubleshooting URDF Import

**Issue**: "URDF file could not be parsed"
- **Fix**: Validate URDF syntax: `check_urdf simple_humanoid.urdf`
- Ensure all mesh files referenced in `<visual>` tags exist

**Issue**: "Joints are flipping or unstable"
- **Fix**: Check joint limits in ArticulationBody match URDF `<limit>` tags
- Increase **Solver Iterations**: Edit → Project Settings → Physics → Solver Iterations: 10

---

## 4. Configuring Photorealistic Rendering

Unity's rendering capabilities far exceed Gazebo's. This section covers lighting, shadows, materials, and post-processing for photorealistic robot visualization.

### Lighting Systems in Unity

Unity uses three lighting types:

1. **Directional Light**: Simulates sunlight (parallel rays, infinite distance)
   - **Use for**: Outdoor scenes, primary light source
   - **Settings**:
     - **Intensity**: 1.5 lux (brightness)
     - **Color**: Warm white (#FFF8DC) for natural sunlight
     - **Shadow Type**: Soft Shadows (realistic, more expensive)
     - **Shadow Strength**: 0.8 (subtle shadows)

2. **Point Light**: Omnidirectional light (like a light bulb)
   - **Use for**: Indoor scenes, spotlights on robot
   - **Settings**:
     - **Range**: 10 meters (light radius)
     - **Intensity**: 2.0 lux
     - **Shadow Type**: Hard Shadows (sharp edges, less expensive)

3. **Ambient Light**: Global illumination from environment
   - **Use for**: Fill light, preventing completely black shadows
   - **Settings** (Window → Rendering → Lighting):
     - **Environment Lighting → Source**: Skybox
     - **Intensity Multiplier**: 1.0
     - **Ambient Mode**: Realtime or Baked (Baked is faster)

**Example Scene Setup**:
```
Hierarchy:
├── Directional Light (intensity 1.5, rotation (50, -30, 0))
├── Point Light (position (0, 3, 0), range 10m, intensity 2.0)
└── Main Camera (with ROSCameraPublisher script)
```

### Material Properties (PBR Shading)

Unity uses **Physically Based Rendering (PBR)** for realistic materials. Key properties:

- **Base Map (Albedo)**: Surface color or texture
- **Metallic**: 0 = dielectric (plastic, wood), 1 = metallic (steel, aluminum)
- **Smoothness**: 0 = rough/matte, 1 = polished/glossy
- **Normal Map**: Simulates surface details (scratches, dents) without extra geometry
- **Emission**: Self-illuminated surfaces (LEDs, screens)

**Example Robot Materials**:

1. **Metal Body** (for torso, shoulders):
   - Base Map: Light grey (#AAAAAA)
   - Metallic: 0.9
   - Smoothness: 0.7
   - Result: Polished aluminum look

2. **Plastic Joints** (for elbows):
   - Base Map: Dark grey (#555555)
   - Metallic: 0.0
   - Smoothness: 0.4
   - Result: Matte plastic

### Shadow Configuration

Shadows add depth perception and realism.

**Shadow Quality Settings** (Edit → Project Settings → Quality):
- **Shadow Resolution**: Very High Resolution (2048x2048 shadow maps)
- **Shadow Distance**: 150 meters (objects beyond this won't cast shadows)
- **Shadow Cascades**: Two Cascades (balance quality vs performance)
- **Soft Shadows**: Enabled (smooth shadow edges)

**Per-Light Shadow Settings**:
- Select Directional Light → Inspector:
  - **Shadow Type**: Soft Shadows
  - **Strength**: 0.8 (not completely black)
  - **Bias**: 0.05 (prevents shadow acne artifacts)
  - **Normal Bias**: 0.4 (prevents peter-panning)

### Post-Processing Effects

Post-processing enhances visual fidelity after rendering.

**Install Post-Processing Package**:
1. Window → Package Manager → Unity Registry
2. Search "Post Processing"
3. Click Install

**Add Post-Processing Volume**:
1. Create empty GameObject: **Right-click Hierarchy → Create Empty**
2. Rename: `Post Processing Volume`
3. Add Component: **Post-process Volume**
4. Set **Is Global**: ✅ Enabled (affects entire scene)
5. Add effects:
   - **Bloom**: Simulates light bleeding (glowing highlights)
     - Intensity: 0.3
     - Threshold: 1.0
   - **Color Grading**: Adjust overall tone
     - Temperature: +5 (warmer tones)
     - Contrast: +10 (deeper shadows, brighter highlights)
   - **Ambient Occlusion**: Darkens crevices and contact points
     - Intensity: 0.5
     - Radius: 1.0

**Enable Post-Processing on Camera**:
- Select Main Camera → Add Component: **Post-process Layer**
- **Layer**: Everything

**Performance Impact**: Post-processing reduces FPS by 10-20%. Disable for low-end GPUs.

### Baking Lighting (Optional)

For static scenes (non-moving robot), bake lighting to improve performance:

1. Mark static objects: Select Ground → Inspector → **Static**: ✅ Enabled
2. Window → Rendering → Lighting Settings
3. **Baked Global Illumination**: ✅ Enabled
4. **Lightmap Resolution**: 40 texels/unit
5. Click **Generate Lighting** (takes 5-10 minutes)

**Result**: Indirect lighting (light bounces) calculated offline, improving real-time FPS.

---

## 5. Unity-ROS 2 Bridge Setup

The ROS-TCP-Connector bridges Unity and ROS 2 by serializing ROS messages to JSON and transmitting over TCP/IP. This section covers configuration and creating publisher/subscriber scripts.

### Unity-ROS Bridge Architecture

```mermaid
graph LR
    subgraph Unity
        A[GameObject: Main Camera] -->|Capture Image| B[ROSCameraPublisher.cs]
        C[GameObject: simple_humanoid] -->|Joint Positions| D[ROSJointSubscriber.cs]
        B -->|Publish| E[ROS-TCP-Connector]
        E -->|Subscribe| D
    end

    subgraph ROS 2 Humble
        F[/camera/rgb/image_raw<br/>sensor_msgs/Image] -->|TCP/IP Port 10000| E
        E -->|TCP/IP Port 10000| G[/joint_states<br/>sensor_msgs/JointState]
        H[ros_tcp_endpoint Node] -->|Serialize/Deserialize| F
        H -->|Serialize/Deserialize| G
    end

    E -.->|Network| H

    style A fill:#90EE90
    style C fill:#90EE90
    style B fill:#FFD700
    style D fill:#FFD700
    style E fill:#87CEEB
    style H fill:#FFA07A
```

**Architecture Explanation**:
- **Unity GameObjects** (green): Camera and robot hierarchy with attached scripts
- **C# Scripts** (yellow): Publisher/subscriber logic for ROS topics
- **ROS-TCP-Connector** (blue): Unity package handling message serialization and network communication
- **ros_tcp_endpoint** (orange): ROS 2 node running on Ubuntu, bridges Unity to native ROS 2 topics

**Key Difference from Gazebo**: Unlike Gazebo's native ROS integration (direct C++ calls), Unity uses TCP/IP network communication, introducing 100-200ms latency. This is acceptable for visualization but unsuitable for real-time control.

### Configuring ROS-TCP-Connector

1. **Open ROS Settings**:
   - In Unity Editor: **Robotics → ROS Settings**

2. **Configure TCP Endpoint**:
   - **Protocol**: ROS2
   - **ROS IP Address**: `127.0.0.1` (localhost if ROS and Unity on same machine)
     - If Unity on Windows and ROS on WSL2: Use WSL2 IP (e.g., `172.28.192.1`)
     - If Unity on different machine: Use ROS machine's network IP (e.g., `192.168.1.100`)
   - **ROS Port**: `10000` (default port for ros_tcp_endpoint)
   - **Show HUD**: ✅ Enabled (displays connection status in Game view)

3. **Save Settings**:
   - File → Save Project

### Using ROSCameraPublisher and ROSJointSubscriber

The example includes two pre-written C# scripts (see `Assets/Scripts/` in downloadable example):

**ROSCameraPublisher.cs** (frontend_book/static/examples/unity_humanoid_scene:219):
- Attaches to Main Camera GameObject
- Captures camera view to RenderTexture at specified resolution (default: 640x480)
- Publishes `sensor_msgs/Image` to `/camera/rgb/image_raw` topic at 10 Hz
- Displays connection status in Unity Console and Game view HUD

**ROSJointSubscriber.cs** (frontend_book/static/examples/unity_humanoid_scene:313):
- Attaches to robot root GameObject
- Subscribes to `/joint_states` topic
- Updates ArticulationBody joint positions/velocities based on incoming messages
- Maps joint names (string) to ArticulationBody components

**Usage**:
1. Attach ROSCameraPublisher to Main Camera (drag script from Project window)
2. Attach ROSJointSubscriber to simple_humanoid root GameObject
3. In Inspector (ROSJointSubscriber):
   - Expand "Articulation Bodies" array
   - Set Size: 4 (for 4 joints: left_shoulder, left_elbow, right_shoulder, right_elbow)
   - Drag each ArticulationBody component from Hierarchy into array slots

---

## 6. Testing Unity Scene

This section walks through verification steps to ensure Unity-ROS bridge is working correctly.

### Step 1: Start ROS 2 TCP Endpoint

In Ubuntu terminal (or WSL2):

```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Start ROS-TCP-Endpoint server
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Expected Output**:
```
[INFO] [ros_tcp_endpoint]: Starting server on 0.0.0.0:10000
[INFO] [ros_tcp_endpoint]: Server started, waiting for connection...
```

**Keep this terminal running** during Unity testing.

### Step 2: Run Unity Scene

1. In Unity Editor, open `HumanoidDemo.unity` scene (Assets/Scenes/)
2. Click **Play** button (▶ at top center)
3. **Expected**:
   - Game view shows rendered robot with lighting and shadows
   - Unity Console logs:
     ```
     ROSCameraPublisher: Initialized - Publishing to '/camera/rgb/image_raw' at 10 Hz
     ROSJointSubscriber: Subscribed to '/joint_states' - Monitoring 4 joints
     ```
   - ROS TCP Endpoint terminal shows:
     ```
     [INFO] [ros_tcp_endpoint]: Client connected from <Unity IP>
     ```

### Step 3: Verify Camera Publishing

In a new Ubuntu terminal:

```bash
# List ROS 2 topics (Unity-published topics should appear)
ros2 topic list
# Expected: /camera/rgb/image_raw (among others)

# Check camera publish rate
ros2 topic hz /camera/rgb/image_raw
# Expected: average rate: 10.000 Hz (±0.5 Hz)

# Inspect message structure
ros2 topic echo /camera/rgb/image_raw --once
# Expected: sensor_msgs/Image with width=640, height=480, encoding='rgb8'
```

**Visualize in RViz**:
```bash
rviz2
# In RViz:
# 1. Add → Image
# 2. Set Topic: /camera/rgb/image_raw
# 3. Expected: Unity camera view appears in RViz window
```

### Step 4: Test Joint Subscription

Publish test joint commands from ROS 2 to move robot in Unity:

```bash
# Move left shoulder joint to 0.5 radians (~29°)
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
"{name: ['left_shoulder_joint'], position: [0.5]}" --once

# Expected: Robot's left shoulder rotates in Unity Game view

# Move all joints (left shoulder, left elbow, right shoulder, right elbow)
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
"{name: ['left_shoulder_joint', 'left_elbow_joint', 'right_shoulder_joint', 'right_elbow_joint'], \
position: [0.5, -0.3, -0.5, 0.3]}" --once

# Expected: Robot assumes asymmetric pose
```

**Debug Mode Verification**:
- In Unity Console (Window → General → Console), look for:
  ```
  ROSJointSubscriber: Set 'left_shoulder_joint' position = 0.500 rad
  ROSJointSubscriber: Set 'left_elbow_joint' position = -0.300 rad
  ```

### Step 5: Measure Performance

**Check Frame Rate**:
- In Unity Game view: Window → Analysis → Stats
- **Rendering Stats**:
  - FPS (Frames Per Second): Should be ≥30 FPS on 16GB RAM + GTX 1050
  - Draw Calls: &lt;100 for simple scene
  - Triangles: &lt;50k for simple_humanoid

**Check ROS Latency**:
```bash
# Publish joint command and observe Unity response time
time ros2 topic pub /joint_states sensor_msgs/msg/JointState \
"{name: ['left_shoulder_joint'], position: [0.0]}" --once

# Expected: Robot movement visible in Unity within 100-200ms
```

---

## 7. Troubleshooting

### Issue: "ROS-TCP-Connector: Connection refused"

**Symptoms**: Unity Console shows `[ERROR] Failed to connect to 127.0.0.1:10000`

**Cause**: ROS 2 TCP endpoint not running or incorrect IP/port configuration

**Fix**:
1. **Verify endpoint is running**:
   ```bash
   # In Ubuntu terminal
   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
   ```
2. **Check firewall** (if Unity on Windows, ROS on WSL2):
   ```bash
   # Allow port 10000 in Windows Firewall
   # Control Panel → Windows Defender Firewall → Advanced Settings
   # → Inbound Rules → New Rule → Port → TCP 10000 → Allow
   ```
3. **Verify IP address**:
   - Same machine: Use `127.0.0.1`
   - WSL2: Use `hostname -I` in WSL2 to get IP
   - Different machines: Use `ifconfig` or `ip addr` to get ROS machine's LAN IP

---

### Issue: "Camera image is black or not updating"

**Symptoms**: `/camera/rgb/image_raw` topic publishes, but RViz shows black image

**Cause**: Camera not rendering, or culling mask excludes robot

**Fix**:
1. **Check Camera configuration** in Unity:
   - Select Main Camera → Inspector
   - **Culling Mask**: Everything (not "Nothing")
   - **Clear Flags**: Skybox (not "Solid Color" with black)
2. **Verify robot is visible**:
   - In Unity Scene view, select Main Camera
   - Check if robot is in camera frustum (visible cone)
3. **Check ROSCameraPublisher script**:
   - Image Width/Height: Not zero
   - Publish Rate: >0 Hz

---

### Issue: "Robot joints not responding to /joint_states commands"

**Symptoms**: Publishing to `/joint_states` has no effect on robot in Unity

**Cause**: ArticulationBody not configured, or joint names don't match

**Fix**:
1. **Enable Debug Mode**:
   - Select robot root GameObject
   - ROSJointSubscriber script → **Debug Mode**: ✅ Enabled
2. **Check Unity Console** for warnings:
   ```
   ROSJointSubscriber: Unknown joint 'left_shoulder_joint' - ignoring
   ```
   - **Fix**: Rename GameObject to match URDF joint name exactly
3. **Verify ArticulationBody array**:
   - ROSJointSubscriber Inspector → "Articulation Bodies" array should have all joints
   - If empty, drag ArticulationBody components from Hierarchy

---

### Issue: "Unity is extremely slow (< 10 FPS)"

**Symptoms**: Game view stutters, framerate below 10 FPS

**Cause**: Insufficient GPU, or rendering settings too high

**Fix**:
1. **Reduce Quality**:
   - Edit → Project Settings → Quality → Select "Low" preset
   - Disable: Anti-Aliasing, Soft Shadows, Post-Processing
2. **Simplify Lighting**:
   - Disable Directional Light shadows: Shadow Type → No Shadows
   - Remove Point Lights (use only 1 Directional Light)
3. **Switch to Gazebo**: If performance remains poor, use Chapter 1 (Gazebo) instead

---

### Issue: "URDF Importer package not found"

**Symptoms**: "Assets → Import Robot from URDF" menu item missing

**Cause**: URDF-Importer package not installed

**Fix**:
1. Window → Package Manager
2. Click "+" → Add package from git URL
3. Enter:
   ```
   https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
   ```
4. Wait for installation (1-2 minutes)

---

## 8. Key Takeaways

This chapter introduced Unity for robotic visualization, focusing on photorealistic rendering and ROS 2 integration:

**Core Concepts**:
1. **Unity vs Gazebo**: Unity excels at rendering (photorealism, VR), Gazebo at physics (accuracy, sensor simulation). Use both together: Gazebo for simulation, Unity for visualization.
2. **ROS-TCP-Connector**: Bridges Unity to ROS 2 via TCP/IP, enabling camera publishing and joint subscription, but introduces 100-200ms latency (unsuitable for real-time control).
3. **URDF Import**: Unity's URDF-Importer converts robot descriptions to ArticulationBody components, enabling joint-based physics and kinematics in Unity.
4. **Photorealistic Rendering**: PBR materials (metallic, smoothness), directional/point lighting, soft shadows, and post-processing (bloom, ambient occlusion) create realistic visuals.
5. **Performance Trade-offs**: Unity requires 16GB+ RAM and dedicated GPU for 30+ FPS, making it optional (P2 priority) for students with limited hardware.

**When to Use Unity**:
- Human-robot interaction demos (realistic avatars, gestures)
- Perception dataset generation (synthetic images with ground truth)
- Marketing videos and stakeholder demos
- Virtual reality teleoperation interfaces

**When to Use Gazebo**:
- Physics simulation and dynamics testing
- SLAM, localization, and path planning
- Sensor simulation (LiDAR, IMU, depth cameras)
- Real-time control and feedback loops

**Best Practice**: Run Gazebo physics in headless mode (no GUI) while Unity renders the same scene via ROS 2 bridge, combining accurate simulation with photorealistic visualization.

---

## Practice Exercises

1. **Lighting Experiment**: Modify the Directional Light intensity from 1.5 to 0.5 lux. How does this affect shadow visibility and overall scene brightness? Try different light colors (red, blue, green).

2. **Material Customization**: Create a new material for the robot torso with **Emission** enabled (self-illuminating LED effect). Set Emission Color to cyan (#00FFFF) with intensity 2.0. Observe the glow effect in Game view.

3. **Multi-Robot Scene**: Import a second instance of simple_humanoid, position it at (2, 1, 0), and publish different joint states to each robot using namespace prefixes (`/robot1/joint_states`, `/robot2/joint_states`). Modify ROSJointSubscriber to support namespaces.

4. **RViz Synchronization**: Launch Gazebo (Chapter 1 example) and Unity simultaneously, publishing the same joint commands to both `/joint_states` topics. Compare physics accuracy and rendering quality side-by-side.

5. **Performance Profiling**: Use Unity Profiler (Window → Analysis → Profiler) to identify bottlenecks. Record FPS with post-processing enabled vs disabled. What is the performance cost of each effect (bloom, ambient occlusion)?

---

## Download Example Files

📦 **[unity_humanoid_scene.zip](/examples/unity_humanoid_scene.zip)** - Unity project with ROS 2 bridge scripts

**Contents**:
- Unity 2021 LTS project structure
- C# scripts: ROSCameraPublisher, ROSJointSubscriber
- Packages manifest with ROS-TCP-Connector dependency
- README with setup instructions and troubleshooting

**Note**: This is a template project (scene files not included). Follow Chapter 2 instructions to create the scene manually, or use the example scripts as reference for your own Unity projects.

---

## References

1. **Koenig, N. & Howard, A.** (2004). "Design and Use Paradigms for Gazebo, An Open-Source Multi-Robot Simulator." *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 2149-2154. DOI: 10.1109/IROS.2004.1389727

2. **Martinez-Gonzalez, A., et al.** (2020). "UnrealROX: An Extremely Photorealistic Virtual Reality Environment for Robotics Simulations and Synthetic Data Generation." *Virtual Reality*, 24(2), 271-288. DOI: 10.1007/s10055-019-00399-5 [Open Access: https://arxiv.org/abs/1810.06936]

3. **Rosen, E., et al.** (2019). "Communicating Robot Arm Motion Intent Through Mixed Reality Head-mounted Displays." *Robotics Research*, Springer, pp. 301-316. DOI: 10.1007/978-3-030-28619-4_26

4. **Unity Technologies** (2023). "Unity Robotics Hub." Official Documentation. https://github.com/Unity-Technologies/Unity-Robotics-Hub

---

**Next Chapter**: [Chapter 3 - Sensor Simulation](/docs/module-2/chapter-3-sensor-simulation) (LiDAR, Depth Cameras, IMU)
