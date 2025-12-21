# Gazebo Humanoid World Example

This example demonstrates physics-based simulation of the `simple_humanoid` robot in Gazebo Classic for **Module 2 - Chapter 1: Gazebo Physics Simulation**.

## Contents

- `simple_humanoid.world` - Gazebo world file with ground plane, lighting, and physics settings
- `simple_humanoid_gazebo.urdf` - URDF file with Gazebo physics extensions (friction, damping, materials)
- `launch/gazebo_demo.launch.py` - ROS 2 launch file to start Gazebo and spawn the robot
- `README.md` - This file (usage instructions and troubleshooting)

## Prerequisites

Before running this example, ensure you have:

1. **Ubuntu 22.04 LTS** (or WSL2 with Ubuntu 22.04)
2. **ROS 2 Humble** installed and sourced
   ```bash
   source /opt/ros/humble/setup.bash
   ```
3. **Gazebo Classic 11** installed
   ```bash
   sudo apt install gazebo11
   ```
4. **gazebo_ros_pkgs** for ROS 2 integration
   ```bash
   sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros
   ```

## Quick Start

### Option 1: Launch via ROS 2 (Recommended)

```bash
# Navigate to the example directory
cd gazebo_humanoid_world

# Launch Gazebo with the humanoid robot
ros2 launch launch/gazebo_demo.launch.py
```

**Expected Result**:
- Gazebo GUI opens with ground plane and directional light
- `simple_humanoid` robot spawns 1 meter above ground
- Robot falls due to gravity and lands on ground plane
- You can see realistic physics: gravity, collision detection, joint dynamics

### Option 2: Launch Gazebo Standalone (Testing Only)

```bash
# Start Gazebo with the world file (no robot spawning)
gazebo simple_humanoid.world
```

**Note**: This only loads the environment. To spawn the robot, use Option 1 or manually spawn via Gazebo GUI.

## Verification Steps

After launching, verify the simulation is working correctly:

### 1. Check ROS 2 Topics

```bash
# In a new terminal, list all active ROS 2 topics
ros2 topic list
```

**You should see**:
- `/joint_states` - Robot joint positions and velocities
- `/tf` and `/tf_static` - Coordinate transformations between robot links
- `/clock` - Gazebo simulation time

### 2. Monitor Joint States

```bash
# Echo joint state messages
ros2 topic echo /joint_states
```

**Expected Output**:
```yaml
header:
  stamp:
    sec: ...
    nanosec: ...
  frame_id: ''
name:
- left_shoulder_joint
- left_elbow_joint
- right_shoulder_joint
- right_elbow_joint
position: [0.0, 0.0, 0.0, 0.0]  # Joint angles in radians
velocity: [0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0, 0.0]
```

### 3. Test Velocity Commands (If Controller Loaded)

```bash
# Publish a test velocity command (if diff_drive or similar controller is configured)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

**Note**: The basic URDF does not include controllers by default. This command demonstrates the ROS 2 topic interface. See Chapter 1 for adding controllers.

### 4. Visualize in RViz (Optional)

```bash
# Launch RViz to visualize the robot's TF tree
rviz2
```

In RViz:
1. Set **Fixed Frame** to `base_link`
2. Add **RobotModel** display
3. Add **TF** display to see coordinate frames
4. You should see the humanoid robot structure with all links

## Physics Testing

### Gravity Test

**What to expect**: Robot falls from 1m height and lands on ground plane.

**Verification**:
- Robot's `base_link` z-position should decrease from 1.0m to ~0.5m (half the torso height)
- No penetration through ground (collision detection working)
- Robot remains stable after landing (realistic contact forces)

### Collision Test

**Manual Test**:
1. In Gazebo GUI, select the **Translation Mode** (toolbar icon)
2. Click on the robot and try to drag it through the ground
3. **Expected**: Physics engine prevents interpenetration

### Joint Movement Test

**Manual Test**:
1. In Gazebo GUI, go to **World → simple_humanoid → Joints**
2. Adjust joint positions using sliders (e.g., `left_shoulder_joint`)
3. **Expected**: Joints move smoothly within defined limits (-90° to +90° for shoulders)

## File Structure Explained

```
gazebo_humanoid_world/
├── simple_humanoid.world          # Gazebo SDF world file
│   ├── Physics settings (gravity, solver, timestep)
│   ├── Ground plane model
│   ├── Sun (directional light)
│   └── Camera view configuration
│
├── simple_humanoid_gazebo.urdf    # Enhanced URDF with Gazebo tags
│   ├── <link> tags: Visual, collision, inertial properties
│   ├── <joint> tags: Limits, damping, friction
│   └── <gazebo> tags: Material, mu1/mu2 (friction), kp/kd (contact)
│
├── launch/gazebo_demo.launch.py  # ROS 2 Python launch file
│   ├── Starts Gazebo server
│   ├── Spawns robot from URDF
│   ├── Publishes robot state to TF
│   └── Publishes joint states
│
└── README.md                      # This file
```

## Troubleshooting

### Issue: "Model is sinking through the ground plane"

**Cause**: Missing or incorrect collision geometry in URDF.

**Fix**:
1. Verify each `<link>` has a `<collision>` tag matching the `<visual>` geometry
2. Check that inertia values are realistic (not zero or extremely large)
3. Ensure Gazebo friction coefficients are set: `<mu1>0.8</mu1>` and `<mu2>0.8</mu2>`

**Verification**:
```bash
# Check for collision geometry in URDF
grep -A 5 "<collision>" simple_humanoid_gazebo.urdf
```

---

### Issue: "[WARN] Controller Spawner couldn't find the expected controller_manager ROS interface"

**Cause**: Gazebo control plugin not loaded, or ROS 2 namespace mismatch.

**Fix**:
1. This warning is expected if you haven't added controllers (Chapter 1 focuses on physics only)
2. To add controllers, include `libgazebo_ros_control.so` in a `<gazebo>` tag
3. See Module 2 Chapter 2 or ROS 2 Control documentation for advanced controller setup

**Workaround**: Ignore this warning for basic physics simulation (not needed for gravity/collision testing).

---

### Issue: "Gazebo crashes on startup" or "Segmentation fault"

**Cause**: Missing Gazebo plugins, incompatible Gazebo version, or missing GL drivers.

**Fix**:
1. Verify Gazebo version: `gazebo --version` should show `Gazebo multi-robot simulator, version 11.x.x`
2. Install missing plugins:
   ```bash
   sudo apt install ros-humble-gazebo-plugins ros-humble-gazebo-ros
   ```
3. If running in WSL2, ensure X11 forwarding is set up:
   ```bash
   export DISPLAY=:0
   # Or use VcXsrv/Xming on Windows
   ```
4. Check Gazebo logs for specific errors:
   ```bash
   gazebo --verbose simple_humanoid.world 2>&1 | tee gazebo_log.txt
   ```

---

### Issue: "/scan or /camera topics not publishing"

**Cause**: Sensor plugins not included in this basic example.

**Fix**: This example focuses on physics simulation (gravity, collisions). For sensor simulation (LiDAR, cameras, IMU), see **Module 2 - Chapter 3: Sensor Simulation** and the `gazebo_sensors.zip` example.

---

### Issue: "URDF parse error" or "check_urdf: command not found"

**Cause**: URDF syntax error or liburdfdom-tools not installed.

**Fix**:
1. Install URDF validation tools:
   ```bash
   sudo apt install liburdfdom-tools
   ```
2. Validate URDF syntax:
   ```bash
   check_urdf simple_humanoid_gazebo.urdf
   ```
3. Expected output:
   ```
   robot name is: simple_humanoid
   ---------- Successfully Parsed XML ---------------
   root Link: base_link has 2 child(ren)
       child(1):  left_shoulder_link
           child(1):  left_elbow_link
       child(2):  right_shoulder_link
           child(1):  right_elbow_link
   ```

---

### Issue: "Gazebo GUI is laggy or freezing (< 10 FPS)"

**Cause**: Insufficient hardware resources or complex physics world.

**Fix**:
1. **Reduce physics update rate** in `simple_humanoid.world`:
   ```xml
   <real_time_update_rate>500</real_time_update_rate>  <!-- Default: 1000 -->
   ```
2. **Use headless mode** (no GUI, faster):
   ```bash
   # Replace 'gazebo' with 'gzserver' in launch file, visualize in RViz
   gzserver simple_humanoid.world
   ```
3. **Simplify collision geometry**: Use primitives (box, cylinder) instead of mesh files
4. **Disable shadows** in world file:
   ```xml
   <scene><shadows>false</shadows></scene>
   ```

**Expected Performance**: 20+ FPS on 8GB RAM laptop with integrated GPU (spec SC-002).

---

## Hardware Requirements

**Minimum** (for Gazebo physics simulation):
- 8GB RAM
- Integrated GPU (Intel HD Graphics or better)
- Ubuntu 22.04 (or WSL2)
- 2 CPU cores

**Recommended**:
- 16GB RAM
- Dedicated GPU (NVIDIA, AMD with OpenGL support)
- 4+ CPU cores for faster physics simulation

---

## Next Steps

After successfully running this example:

1. **Read Chapter 1** of Module 2 to understand:
   - Gazebo world file structure and physics parameters
   - URDF Gazebo extensions (`<gazebo>` tags, friction, materials)
   - ROS 2 launch file architecture
   - Troubleshooting common Gazebo errors

2. **Experiment with Physics**:
   - Modify gravity in `simple_humanoid.world`: `<gravity>0 0 -1.62</gravity>` (Moon gravity)
   - Change friction coefficients: `<mu1>0.1</mu1>` (slippery surface)
   - Adjust joint damping: `<dynamics damping="2.0" />` (stiffer joints)

3. **Proceed to Chapter 2**: Unity Rendering (optional, if hardware supports)

4. **Proceed to Chapter 3**: Sensor Simulation (LiDAR, depth cameras, IMU)

---

## Support

For issues or questions:
- **Module 2 Documentation**: See Chapter 1 for detailed explanations
- **ROS 2 Humble Docs**: https://docs.ros.org/en/humble/
- **Gazebo Classic Tutorials**: http://classic.gazebosim.org/tutorials
- **ROS Answers Forum**: https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:gazebo/

---

**Author**: AI-Driven Robotics Textbook
**Module**: 2 - The Digital Twin (Gazebo & Unity)
**Chapter**: 1 - Gazebo Physics Simulation
**Last Updated**: 2025-12-19
