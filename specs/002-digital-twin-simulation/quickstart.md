# Quickstart: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: 002-digital-twin-simulation
**Date**: 2025-12-19
**Phase**: Phase 1 - Design & Contracts
**For**: Developers implementing Module 2 documentation

## Overview

This quickstart guide walks through creating Module 2: The Digital Twin (Gazebo & Unity) documentation and example files. This module teaches students physics simulation (Gazebo), photorealistic rendering (Unity), and sensor simulation through three chapters.

**Time to Complete**: 60-90 minutes (documentation structure + first example file)

---

## Prerequisites

Before starting Module 2 implementation:

- [x] Module 1 completed and deployed (simple_humanoid.urdf exists)
- [x] Docusaurus site running locally (`npm start` in `frontend_book/`)
- [x] Git branch `002-digital-twin-simulation` created and checked out
- [x] Gazebo Classic 11 installed locally for testing examples (Ubuntu 22.04 + ROS 2 Humble)
- [x] Unity 2021 LTS installed (optional, for testing Unity examples)
- [x] ROS 2 workspace set up from Module 1

**Verify Prerequisites**:
```bash
# Check Docusaurus is working
cd frontend_book && npm start

# Check Gazebo is installed
gazebo --version  # Should show Gazebo 11.x

# Check ROS 2 is sourced
ros2 --version    # Should show ROS 2 Humble

# Check Module 1 URDF exists
ls ../ros2_workspace/src/simple_humanoid_description/urdf/simple_humanoid.urdf
```

---

## Step 1: Create Module 2 Documentation Structure

### 1.1 Create Module 2 Directory

```bash
cd frontend_book/docs
mkdir -p module-2
cd module-2
```

### 1.2 Create Chapter Files

```bash
# Create module landing page
touch index.md

# Create chapter files
touch chapter-1-gazebo-physics.md
touch chapter-2-unity-rendering.md
touch chapter-3-sensor-simulation.md
```

**Expected Structure**:
```
frontend_book/docs/module-2/
├── index.md                           # Module 2 overview
├── chapter-1-gazebo-physics.md        # Chapter 1: Gazebo Simulation
├── chapter-2-unity-rendering.md       # Chapter 2: Unity Rendering
└── chapter-3-sensor-simulation.md     # Chapter 3: Sensor Simulation
```

### 1.3 Update Sidebar Configuration

Edit `frontend_book/sidebars.js` to add Module 2:

```javascript
// frontend_book/sidebars.js
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/index',
        'module-1/chapter-1-ros2-fundamentals',
        'module-1/chapter-2-python-agents',
        'module-1/chapter-3-urdf-modeling',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/index',
        'module-2/chapter-1-gazebo-physics',
        'module-2/chapter-2-unity-rendering',
        'module-2/chapter-3-sensor-simulation',
      ],
    },
  ],
};

module.exports = sidebars;
```

**Verify Sidebar**:
```bash
cd frontend_book
npm start
# Navigate to http://localhost:3000 and verify Module 2 appears in sidebar
```

---

## Step 2: Write Module 2 Landing Page (index.md)

Create `frontend_book/docs/module-2/index.md` with module overview:

```markdown
---
id: index
title: Module 2 - The Digital Twin (Gazebo & Unity)
sidebar_label: Overview
sidebar_position: 1
---

# Module 2: The Digital Twin (Gazebo & Unity)

## Learning Objectives

By the end of this module, you will:

- Simulate humanoid robots in realistic physics environments using Gazebo Classic
- Understand gravity, collision detection, and joint dynamics in physics simulation
- Render photorealistic robot environments with Unity for human-robot interaction (HRI)
- Configure and validate simulated sensors (LiDAR, depth cameras, IMUs) in Gazebo
- Integrate Unity with ROS 2 using the ROS-TCP-Connector bridge

## Prerequisites

Before starting this module, you should have:

1. **Completed Module 1**: ROS 2 fundamentals, Python agents, URDF modeling
2. **simple_humanoid.urdf**: The humanoid robot model from Module 1 Chapter 3
3. **ROS 2 Humble** installed on Ubuntu 22.04 (or WSL2)
4. **Gazebo Classic 11** installed (see installation guide below)
5. **Basic 3D graphics understanding**: Coordinate systems, transformations (covered in Module 1)

## Module Structure

This module consists of three chapters:

### Chapter 1: Gazebo Simulation – Physics, Gravity, Collisions

Learn to simulate humanoid robots in Gazebo with realistic physics. Topics include:
- Installing and configuring Gazebo Classic 11
- Creating Gazebo world files with ground planes and lighting
- Adding physics properties to URDF models (friction, inertia, damping)
- Testing collision detection and gravity simulation
- Launching Gazebo with ROS 2 integration

**Time**: 2-3 hours | **Difficulty**: Intermediate

### Chapter 2: Unity Rendering – High-Fidelity Visualization

Learn to create photorealistic robot visualizations in Unity. Topics include:
- Installing Unity 2021 LTS and ROS-TCP-Connector
- Importing URDF models into Unity scenes
- Configuring materials, lighting, and shadows for realism
- Setting up Unity-ROS 2 bidirectional communication
- Publishing camera feeds from Unity to ROS 2

**Time**: 3-4 hours | **Difficulty**: Advanced | **Optional**: Can skip if hardware limited

### Chapter 3: Sensor Simulation – LiDAR, Depth Cameras, IMUs

Learn to simulate robot sensors with realistic noise models. Topics include:
- Configuring LiDAR sensor plugins in Gazebo URDFs
- Setting up depth cameras with configurable noise parameters
- Adding IMU sensors for orientation and acceleration data
- Visualizing sensor data in RViz
- Validating sensor accuracy with known test environments

**Time**: 2-3 hours | **Difficulty**: Intermediate

## Installation Quick Links

- [Gazebo Classic 11 Installation](http://classic.gazebosim.org/tutorials?tut=install_ubuntu)
- [Unity 2021 LTS Download](https://unity.com/releases/lts)
- [ROS-TCP-Connector Setup](https://github.com/Unity-Technologies/ROS-TCP-Connector)

## Next Steps

Start with [Chapter 1: Gazebo Simulation](./chapter-1-gazebo-physics.md) to learn physics-based robot simulation.
```

**Verify**:
```bash
# Check markdown renders correctly
npm start
# Navigate to http://localhost:3000/docs/module-2/
```

---

## Step 3: Create Example Files Directory

### 3.1 Create Static Examples Directory

```bash
cd frontend_book/static
mkdir -p examples
cd examples
```

### 3.2 Set Up Example File Structure

```bash
# Create directories for each example archive
mkdir -p gazebo_humanoid_world
mkdir -p gazebo_sensors
mkdir -p unity_humanoid_scene
```

**Expected Structure**:
```
frontend_book/static/examples/
├── gazebo_humanoid_world/
│   ├── simple_humanoid.world
│   ├── simple_humanoid_gazebo.urdf
│   ├── launch/
│   │   └── gazebo_demo.launch.py
│   └── README.md
├── gazebo_sensors/
│   ├── humanoid_with_lidar.urdf
│   ├── humanoid_with_depth_camera.urdf
│   ├── humanoid_with_imu.urdf
│   ├── launch/
│   │   └── sensor_demo.launch.py
│   └── README.md
└── unity_humanoid_scene/
    ├── UnityProject/
    │   ├── Assets/
    │   ├── Packages/
    │   └── ProjectSettings/
    └── README.md
```

---

## Step 4: Create First Example - Gazebo World File

### 4.1 Copy Module 1 URDF

```bash
# Copy simple_humanoid.urdf from Module 1 as base
cp ../../ros2_workspace/src/simple_humanoid_description/urdf/simple_humanoid.urdf \
   frontend_book/static/examples/gazebo_humanoid_world/simple_humanoid_base.urdf
```

### 4.2 Create Gazebo-Enhanced URDF

Create `frontend_book/static/examples/gazebo_humanoid_world/simple_humanoid_gazebo.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Import base URDF structure from Module 1 -->
  <!-- (In production, use xacro:include or copy full URDF here) -->

  <!-- BASE LINK (Torso) with Gazebo physics -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>  <!-- 10 kg torso -->
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 1.0"/>  <!-- 30cm wide, 20cm deep, 1m tall torso -->
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 1.0"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo-specific properties for base_link -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
    <mu1>0.8</mu1>  <!-- Friction coefficient (concrete-like) -->
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>  <!-- Contact stiffness -->
    <kd>1.0</kd>        <!-- Contact damping -->
  </gazebo>

  <!-- LEFT ARM SHOULDER LINK -->
  <link name="left_shoulder_link">
    <inertial>
      <mass value="2.0"/>  <!-- 2 kg upper arm -->
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>  <!-- 5cm radius, 30cm long -->
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- LEFT SHOULDER JOINT (revolute) -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder_link"/>
    <origin xyz="0 0.15 0.9" rpy="0 0 0"/>  <!-- Attach at top-left of torso -->
    <axis xyz="0 1 0"/>  <!-- Pitch rotation -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
    <dynamics damping="0.7" friction="0.5"/>
  </joint>

  <!-- Gazebo joint properties -->
  <gazebo reference="left_shoulder_joint">
    <provideFeedback>true</provideFeedback>  <!-- Enable joint state feedback -->
  </gazebo>

  <!-- (Add right arm, elbow joints following same pattern) -->

</robot>
```

### 4.3 Create Gazebo World File

Create `frontend_book/static/examples/gazebo_humanoid_world/simple_humanoid.world`:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="simple_humanoid_world">
    <!-- Physics engine settings -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>  <!-- Earth gravity -->
      <max_step_size>0.001</max_step_size>  <!-- 1ms time step -->
      <real_time_update_rate>1000</real_time_update_rate>  <!-- 1000 Hz -->
      <real_time_factor>1.0</real_time_factor>  <!-- Real-time simulation -->
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun (directional light) -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Spawn simple_humanoid robot -->
    <!-- Note: In practice, robot is spawned via ros2 launch, not hardcoded here -->
    <!-- This is for testing Gazebo world standalone -->
    <model name="simple_humanoid">
      <pose>0 0 1.0 0 0 0</pose>  <!-- Spawn 1m above ground -->
      <include>
        <uri>file://simple_humanoid_gazebo.urdf</uri>
      </include>
    </model>

  </world>
</sdf>
```

### 4.4 Create ROS 2 Launch File

Create `frontend_book/static/examples/gazebo_humanoid_world/launch/gazebo_demo.launch.py`:

```python
#!/usr/bin/env python3
"""
Launch Gazebo with simple_humanoid robot for Module 2 Chapter 1.

Usage:
  ros2 launch gazebo_demo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to Gazebo world file
    world_file = os.path.join(
        os.path.dirname(__file__), '..', 'simple_humanoid.world'
    )

    # Path to URDF file
    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', 'simple_humanoid_gazebo.urdf'
    )

    return LaunchDescription([
        # Launch Gazebo server with custom world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn robot in Gazebo (alternative to hardcoded model in .world)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'simple_humanoid',
                '-file', urdf_file,
                '-x', '0', '-y', '0', '-z', '1.0'  # Spawn 1m above ground
            ],
            output='screen'
        ),

        # Publish robot state to ROS 2
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}],
            output='screen'
        ),
    ])
```

### 4.5 Create README for Example

Create `frontend_book/static/examples/gazebo_humanoid_world/README.md`:

```markdown
# Gazebo Humanoid World Example

This example demonstrates physics-based simulation of the `simple_humanoid` robot in Gazebo Classic.

## Contents

- `simple_humanoid.world`: Gazebo world file with ground plane, lighting, and physics settings
- `simple_humanoid_gazebo.urdf`: URDF with Gazebo physics extensions (friction, damping, materials)
- `launch/gazebo_demo.launch.py`: ROS 2 launch file to start Gazebo and spawn the robot

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- gazebo_ros_pkgs installed (`sudo apt install ros-humble-gazebo-ros-pkgs`)

## Usage

### Option 1: Launch via ROS 2 (Recommended)

```bash
# Navigate to example directory
cd gazebo_humanoid_world

# Launch Gazebo with robot
ros2 launch launch/gazebo_demo.launch.py
```

You should see:
- Gazebo GUI opens with ground plane and directional light
- `simple_humanoid` robot spawns 1 meter above ground
- Robot falls due to gravity and lands on ground plane (realistic physics)

### Option 2: Launch Gazebo Standalone

```bash
# Start Gazebo with world file
gazebo simple_humanoid.world
```

## Verification

Check that physics is working:

```bash
# In another terminal, list ROS 2 topics
ros2 topic list

# You should see:
# /joint_states       (robot joint positions)
# /tf                 (coordinate transforms)
# /clock              (Gazebo simulation time)

# Test velocity commands (if controller loaded)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" --once
```

## Troubleshooting

**Robot falls through ground**:
- Check that `<collision>` geometry matches `<visual>` in URDF
- Verify ground plane is loaded: `ros2 topic echo /gazebo/model_states`

**Gazebo crashes on startup**:
- Ensure Gazebo 11 is installed: `gazebo --version`
- Check for missing plugins: `gazebo --verbose simple_humanoid.world` and review logs

## Next Steps

See Module 2 Chapter 1 for detailed explanation of Gazebo physics parameters.
```

---

## Step 5: Test Example Locally

### 5.1 Test Gazebo World

```bash
# Navigate to example directory
cd frontend_book/static/examples/gazebo_humanoid_world

# Test Gazebo world loads without errors
gazebo simple_humanoid.world --verbose

# Expected output:
# - Gazebo GUI opens
# - Ground plane visible
# - No error messages in terminal
```

### 5.2 Validate URDF

```bash
# Check URDF syntax
check_urdf simple_humanoid_gazebo.urdf

# Expected output:
# robot name is: simple_humanoid
# ---------- Successfully Parsed XML ---------------
# root Link: base_link has 1 child(ren)
#     child(1):  left_shoulder_link
# ...
```

---

## Step 6: Create Archive for Download

### 6.1 Create ZIP Archive

```bash
cd frontend_book/static/examples

# Create downloadable archive
powershell -Command "Compress-Archive -Path 'gazebo_humanoid_world' -DestinationPath 'gazebo_humanoid_world.zip' -Force"

# Verify archive created
ls -lh gazebo_humanoid_world.zip
```

### 6.2 Link Archive in Documentation

In `frontend_book/docs/module-2/chapter-1-gazebo-physics.md`, add download link:

```markdown
## Example: Gazebo Humanoid World

Download the complete example:

📦 **[gazebo_humanoid_world.zip](/examples/gazebo_humanoid_world.zip)** (includes world file, URDF, launch file, README)

After downloading, extract and follow the README for usage instructions.
```

---

## Step 7: Verify Documentation Build

### 7.1 Build Docusaurus Site

```bash
cd frontend_book

# Clean build
npm run build

# Expected output:
# [SUCCESS] Generated static files in "build".
# [INFO] Use `npm run serve` to test your build locally.
```

### 7.2 Test Build Locally

```bash
npm run serve

# Navigate to http://localhost:3000/docs/module-2/
# Verify:
# - Module 2 landing page renders
# - Sidebar shows all chapters
# - Download links work (/examples/gazebo_humanoid_world.zip downloads)
```

---

## Step 8: Commit Progress

```bash
# Stage all files
git add .

# Commit Module 2 initial structure
git commit -m "feat: Add Module 2 structure and Gazebo example

- Created Module 2 documentation structure (index + 3 chapters)
- Updated sidebar.js for Module 2 navigation
- Added gazebo_humanoid_world example with URDF, world file, launch script
- Includes README and downloadable .zip archive

Relates to: 002-digital-twin-simulation"

# Push to feature branch
git push origin 002-digital-twin-simulation
```

---

## Next Steps

After completing this quickstart:

1. **Write Chapter 1 Content** (2500-4000 words):
   - Gazebo installation guide
   - Physics parameters explained (gravity, friction, damping, inertia)
   - URDF Gazebo extensions (`<gazebo>` tags)
   - Step-by-step tutorial using the example
   - Troubleshooting section

2. **Create Sensor Examples** (Chapter 3):
   - `humanoid_with_lidar.urdf`
   - `humanoid_with_depth_camera.urdf`
   - `humanoid_with_imu.urdf`
   - Sensor validation scripts

3. **Create Unity Example** (Chapter 2):
   - Unity 2021 LTS project
   - Simple humanoid imported as prefab
   - ROS-TCP-Connector configured
   - Example scene with lighting and camera

4. **Add Diagrams**:
   - Mermaid.js diagrams for simulation pipelines
   - Static images for physics concepts
   - Screenshots of Gazebo/Unity UIs

5. **Review and Polish**:
   - Proofread all chapters
   - Verify all examples work on clean Ubuntu 22.04
   - Test download links
   - Run Docusaurus build

---

## File Checklist

After completing this quickstart, you should have:

- [ ] `frontend_book/docs/module-2/index.md` (landing page)
- [ ] `frontend_book/docs/module-2/chapter-1-gazebo-physics.md` (empty, ready for content)
- [ ] `frontend_book/docs/module-2/chapter-2-unity-rendering.md` (empty, ready for content)
- [ ] `frontend_book/docs/module-2/chapter-3-sensor-simulation.md` (empty, ready for content)
- [ ] `frontend_book/sidebars.js` (updated with Module 2)
- [ ] `frontend_book/static/examples/gazebo_humanoid_world/simple_humanoid.world`
- [ ] `frontend_book/static/examples/gazebo_humanoid_world/simple_humanoid_gazebo.urdf`
- [ ] `frontend_book/static/examples/gazebo_humanoid_world/launch/gazebo_demo.launch.py`
- [ ] `frontend_book/static/examples/gazebo_humanoid_world/README.md`
- [ ] `frontend_book/static/examples/gazebo_humanoid_world.zip` (downloadable archive)

---

## Troubleshooting

### Issue: Docusaurus build fails with "Broken links"

**Solution**: Verify all internal links use correct paths:
- Cross-module links: `[Module 1](../module-1/index.md)`
- Download links: `[file.zip](/examples/file.zip)` (starts with `/`)

### Issue: Example ZIP doesn't download

**Solution**: Ensure ZIP is in `frontend_book/static/examples/` (not `docs/` or `src/`)

### Issue: Gazebo can't find URDF file

**Solution**: Use absolute paths in launch files or ensure working directory is correct:
```python
urdf_file = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'simple_humanoid_gazebo.urdf'))
```

---

## Summary

This quickstart established:
- Module 2 documentation structure in Docusaurus
- Example file organization in `/static/examples/`
- First working example (Gazebo world + URDF + launch file)
- Download mechanism for student-facing examples

**Next**: Implement `/sp.tasks` to break down chapter content creation into actionable tasks.
