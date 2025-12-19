---
title: Chapter 1 - Gazebo Simulation
sidebar_position: 2
---

# Chapter 1: Gazebo Simulation – Physics, Gravity, Collisions

## Introduction to Gazebo Classic

Gazebo is a powerful open-source 3D robotics simulator that provides accurate physics simulation, high-quality graphics rendering, and seamless integration with the Robot Operating System (ROS). Originally developed by Open Robotics (formerly known as Open Source Robotics Foundation), Gazebo has become the industry standard for testing robot algorithms, designing robots, and performing regression testing using realistic scenarios.

### What is Gazebo?

Gazebo simulates robots in complex indoor and outdoor environments with realistic physics engines. It can accurately model sensor data from cameras, LiDAR, depth sensors, IMUs, and more. Unlike simple 2D simulators or game engines not designed for robotics, Gazebo provides:

- **Realistic Physics Simulation**: Accurate gravity, inertia, friction, collision detection, and contact dynamics using ODE (Open Dynamics Engine), Bullet, DART, or Simbody physics engines
- **Sensor Simulation**: Ray-based LiDAR, depth cameras, RGB cameras, IMUs, GPS, and force/torque sensors with configurable noise models
- **ROS 2 Integration**: Native support for publishing sensor data and subscribing to actuator commands via ROS 2 topics and services
- **Programmatic Control**: Plugins written in C++ or Python can control robots, spawn models, and modify world properties during simulation
- **Headless Operation**: Run simulations without GUI (gzserver only) for automated testing and cloud deployments

### Why Use Physics Simulation?

Before deploying a robot to real hardware, simulation offers critical advantages:

1. **Safety**: Test dangerous behaviors (falling, high-speed collisions, edge-of-cliff navigation) without risking expensive hardware
2. **Cost**: Iterate on designs and algorithms without building physical prototypes
3. **Speed**: Run physics faster than real-time or pause/step through scenarios frame-by-frame
4. **Reproducibility**: Exact same initial conditions for every test run, eliminating environmental variability
5. **Scale**: Test thousands of scenarios (different terrains, lighting, obstacle placements) impractical in the real world

Real-world robotics development follows this workflow:

```
Design (URDF/CAD) → Simulate (Gazebo) → Validate Algorithms → Deploy to Hardware → Iterate
```

By catching design flaws and algorithmic bugs in simulation, teams save months of development time and avoid costly hardware failures.

### Gazebo Classic vs Gazebo Sim (Ignition)

In 2021, Open Robotics began transitioning from **Gazebo Classic** (versions 1-11) to **Gazebo Sim** (formerly called Ignition Gazebo), a complete rewrite with improved performance, modularity, and modern rendering (Ogre 2.x). However, for this textbook, we use **Gazebo Classic 11** for the following reasons:

- **ROS 2 Humble Compatibility**: Gazebo Classic 11 has stable, well-tested integration with ROS 2 Humble (Long-Term Support release until 2027)
- **Maturity**: Extensive documentation, community plugins, and solved edge cases
- **Widespread Adoption**: Most online tutorials, university courses, and open-source projects still use Gazebo Classic
- **Lower Learning Curve**: Simpler XML syntax (SDF 1.6) compared to Gazebo Sim's modular architecture

**Migration Path**: Once you master Gazebo Classic in this chapter, transitioning to Gazebo Sim is straightforward—core concepts (world files, physics engines, sensor plugins) remain the same.

---

## Installing Gazebo Classic 11

### Prerequisites

Before installing Gazebo, ensure you have:

- **Ubuntu 22.04 LTS** (Jammy Jellyfish) - recommended for ROS 2 Humble
- **ROS 2 Humble** installed and sourced (see Module 1 Setup Guide)
- **8GB RAM minimum** (16GB recommended for complex worlds)
- **OpenGL-compatible GPU** (integrated graphics sufficient for basic simulations)

For **Windows users**, we strongly recommend using **WSL2 (Windows Subsystem for Linux)** with Ubuntu 22.04. Native Windows Gazebo builds exist but lack ROS 2 integration.

### Installation Steps

Gazebo Classic 11 is available in Ubuntu's official repositories and can be installed via `apt`:

```bash
# Update package lists
sudo apt update

# Install Gazebo Classic 11
sudo apt install gazebo11

# Install Gazebo development libraries (for custom plugins)
sudo apt install libgazebo11-dev

# Install ROS 2 - Gazebo integration packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros
```

### Verify Installation

Check that Gazebo is correctly installed:

```bash
# Check Gazebo version
gazebo --version
# Expected output: Gazebo multi-robot simulator, version 11.x.x

# Launch Gazebo GUI (empty world)
gazebo
```

If Gazebo GUI opens successfully, you're ready to proceed. Close the GUI with `Ctrl+C` in the terminal or click the X button.

### WSL2 Users: Enable Graphics

If you're using WSL2, you need X11 forwarding to display Gazebo's GUI:

1. **Install VcXsrv or Xming** on Windows (X server for GUI applications)
2. **Set DISPLAY variable** in WSL2:
   ```bash
   export DISPLAY=:0
   # Add to ~/.bashrc to persist across sessions
   echo 'export DISPLAY=:0' >> ~/.bashrc
   ```
3. **Launch VcXsrv** on Windows with "Disable access control" enabled
4. **Test Gazebo**:
   ```bash
   gazebo
   ```

---

## Gazebo World Files

A **Gazebo world** defines the simulation environment: terrain, lighting, physics properties, and spawned models. World files use the **SDF (Simulation Description Format)**, an XML-based format similar to URDF but with additional physics and sensor tags.

### Anatomy of a World File

Let's examine the `simple_humanoid.world` file from our downloadable example:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="simple_humanoid_world">

    <!-- Physics engine configuration -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>  <!-- Earth gravity: -9.81 m/s² in z-axis -->
      <max_step_size>0.001</max_step_size>  <!-- 1ms timestep -->
      <real_time_update_rate>1000</real_time_update_rate>  <!-- 1000 Hz -->
    </physics>

    <!-- Ground plane (infinite flat surface) -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Directional light source -->
    <include>
      <uri>model://sun</uri>
    </include>

  </world>
</sdf>
```

#### Key Components Explained

**1. Physics Engine Settings**

```xml
<physics type="ode">
  <gravity>0 0 -9.81</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

- **`type="ode"`**: Selects the physics engine. Options: `ode` (default, fast), `bullet` (better for soft bodies), `dart` (supports advanced constraints), `simbody` (biomechanics)
- **`gravity`**: Acceleration vector in m/s². Format: `x y z`. Earth standard: `0 0 -9.81`. Moon: `0 0 -1.62`. Space: `0 0 0`
- **`max_step_size`**: Physics timestep in seconds. Smaller = more accurate but slower. Default 0.001s (1ms) balances speed and accuracy
- **`real_time_update_rate`**: How many physics updates per second. 1000 Hz means 1000 updates/sec. Lower values (500 Hz) improve performance on weak hardware

**2. Ground Plane**

```xml
<include>
  <uri>model://ground_plane</uri>
</include>
```

- **`model://`**: Gazebo searches for models in `~/.gazebo/models/` and `/usr/share/gazebo-11/models/`
- **`ground_plane`**: Infinite flat surface at z=0 with friction. Essential for testing collisions

**3. Lighting**

```xml
<include>
  <uri>model://sun</uri>
</include>
```

- **`sun`**: Directional light source simulating sunlight. Creates realistic shadows

You can add additional lights:

```xml
<light name="point_light" type="point">
  <pose>0 0 3 0 0 0</pose>  <!-- 3 meters above origin -->
  <diffuse>0.8 0.8 0.8 1</diffuse>  <!-- White light -->
  <cast_shadows>true</cast_shadows>
</light>
```

### Loading Custom World Files

To load a custom world:

```bash
gazebo /path/to/your_world.world
```

Or in a ROS 2 launch file:

```python
ExecuteProcess(
    cmd=['gazebo', '--verbose', world_file],
    output='screen'
)
```

---

## Adding Physics to URDF Models

In Module 1, we created `simple_humanoid.urdf` with visual and collision geometry. To simulate physics in Gazebo, we extend the URDF with **`<gazebo>` tags** that specify friction, material, and contact properties.

### URDF vs Gazebo-Enhanced URDF

**Base URDF** (from Module 1):
```xml
<link name="base_link">
  <visual>
    <geometry><box size="0.3 0.2 1.0"/></geometry>
  </visual>
  <collision>
    <geometry><box size="0.3 0.2 1.0"/></geometry>
  </collision>
</link>
```

**Gazebo-Enhanced URDF**:
```xml
<link name="base_link">
  <inertial>
    <mass value="10.0"/>  <!-- Required for physics! -->
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <inertia ixx="0.1" iyy="0.1" izz="0.1" .../>
  </inertial>
  <visual>...</visual>
  <collision>...</collision>
</link>

<!-- Gazebo-specific properties -->
<gazebo reference="base_link">
  <material>Gazebo/Grey</material>  <!-- Visual material in Gazebo -->
  <mu1>0.8</mu1>  <!-- Friction coefficient 1 -->
  <mu2>0.8</mu2>  <!-- Friction coefficient 2 -->
  <kp>1000000.0</kp>  <!-- Contact stiffness (N/m) -->
  <kd>1.0</kd>  <!-- Contact damping (N*s/m) -->
</gazebo>
```

### Inertial Properties (Critical!)

Physics engines require **mass** and **inertia** for every non-fixed link:

```xml
<inertial>
  <mass value="10.0"/>  <!-- kg -->
  <origin xyz="0 0 0.5" rpy="0 0 0"/>  <!-- Center of mass offset -->
  <inertia ixx="0.1" ixy="0.0" ixz="0.0"
           iyy="0.1" iyz="0.0" izz="0.1"/>  <!-- Inertia tensor (kg*m²) -->
</inertial>
```

**Calculating Inertia**: For simple shapes, use these formulas:

- **Box** (width w, depth d, height h):
  `ixx = (1/12) * mass * (d² + h²)`
  `iyy = (1/12) * mass * (w² + h²)`
  `izz = (1/12) * mass * (w² + d²)`

- **Cylinder** (radius r, height h, axis along z):
  `ixx = iyy = (1/12) * mass * (3r² + h²)`
  `izz = (1/2) * mass * r²`

For complex meshes, use CAD software (SolidWorks, Fusion 360) to compute inertia automatically.

### Friction Coefficients

```xml
<mu1>0.8</mu1>  <!-- Primary friction direction -->
<mu2>0.8</mu2>  <!-- Secondary friction direction -->
```

- **`mu1` and `mu2`**: Friction coefficients (0.0 = frictionless ice, 1.0+ = rubber on concrete)
- **Typical values**: Wood: 0.4, Concrete: 0.8, Rubber: 1.0-1.5
- **Anisotropic friction**: Use different `mu1`/`mu2` values to simulate directional friction (e.g., tank treads)

### Contact Parameters

```xml
<kp>1000000.0</kp>  <!-- Contact stiffness (N/m) -->
<kd>1.0</kd>  <!-- Contact damping (N*s/m) -->
```

- **`kp` (stiffness)**: How hard surfaces resist penetration. Higher = harder contact (less "sinking")
- **`kd` (damping)**: Dissipates energy on impact. Higher = less bouncing

**Troubleshooting**: If objects penetrate each other, increase `kp`. If they bounce unrealistically, increase `kd`.

### Joint Dynamics

```xml
<joint name="left_shoulder_joint" type="revolute">
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  <dynamics damping="0.7" friction="0.5"/>  <!-- Joint resistance -->
</joint>
```

- **`damping`**: Opposes joint velocity (like viscosity). Higher = slower joint movement
- **`friction`**: Constant resistance regardless of velocity. Simulates motor gearbox friction

---

## Gazebo Simulation Pipeline

Before diving into launch files, let's understand how Gazebo integrates with ROS 2 for robotics simulation:

```mermaid
graph LR
    A[URDF Model] --> B[Gazebo Parser]
    B --> C[SDF Internal Format]
    C --> D[Physics Engine: ODE/Bullet]
    D --> E[Collision Detection]
    E --> F[Contact Forces]
    F --> G[Joint Updates]
    G --> H[/joint_states Topic]
    D --> I[Link Transforms]
    I --> J[/tf Topic]

    K[ROS 2 Topics] --> L[/cmd_vel Commands]
    L --> D
    M[Sensor Plugins] --> N[/scan /camera Topics]
    D --> M

    style D fill:#f9f,stroke:#333,stroke-width:4px
    style K fill:#bbf,stroke:#333,stroke-width:2px
    style M fill:#bfb,stroke:#333,stroke-width:2px
```

This pipeline shows how URDF models are parsed into Gazebo's internal SDF format, processed by the physics engine (ODE), and published back to ROS 2 topics for robot state and sensor data. Understanding this flow helps debug issues when robots don't behave as expected in simulation.

---

## Launching Gazebo with ROS 2

The recommended way to start Gazebo for ROS 2 projects is via **Python launch files**, which automate starting Gazebo, spawning models, and launching supporting nodes (robot_state_publisher, joint_state_publisher).

### Minimal Launch File

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo with world file
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'path/to/world.world'],
            output='screen'
        ),

        # Spawn robot from URDF
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_humanoid', '-file', 'path/to/robot.urdf',
                       '-x', '0', '-y', '0', '-z', '1.0'],  # Spawn at (0,0,1)
            output='screen'
        ),
    ])
```

### Our Example Launch File

The `gazebo_demo.launch.py` from the downloadable example includes:

1. **Gazebo Server**: Starts physics simulation with custom world
2. **Entity Spawner**: Loads robot from URDF at specified position
3. **Robot State Publisher**: Publishes TF transforms for all robot links
4. **Joint State Publisher**: (Optional) Publishes joint angles

**Usage**:
```bash
ros2 launch launch/gazebo_demo.launch.py
```

### Gazebo ROS Plugins

To enable ROS 2 communication, Gazebo needs two plugins:

```bash
gazebo -s libgazebo_ros_init.so -s libgazebo_ros_factory.so
```

- **`libgazebo_ros_init.so`**: Initializes ROS 2 node for Gazebo
- **`libgazebo_ros_factory.so`**: Allows spawning/deleting models via ROS 2 services

These are automatically included in the launch file.

---

## Testing Physics Simulation

Once Gazebo is running with the simple_humanoid robot, verify physics is working correctly.

### Test 1: Gravity

**What to expect**: Robot spawns at z=1.0m and falls due to gravity.

**Verification**:
1. Watch the Gazebo GUI: robot should fall and land on ground plane
2. Final z-position of `base_link` should be ~0.5m (half the torso height of 1.0m)
3. No penetration through ground

**Check via ROS 2**:
```bash
ros2 topic echo /tf_static | grep base_link
# Look for z-coordinate in translation field
```

### Test 2: Collision Detection

**What to expect**: Robot cannot pass through ground plane or obstacles.

**Manual Test**:
1. In Gazebo GUI, click **Translation Mode** (toolbar)
2. Try to drag the robot downward through the ground
3. Physics engine should resist, preventing interpenetration

### Test 3: Joint Limits

**What to expect**: Joints respect limits defined in URDF (e.g., shoulder: -90° to +90°).

**Manual Test**:
1. Go to **World → simple_humanoid → Joints** in Gazebo GUI
2. Move the `left_shoulder_joint` slider
3. Joint should stop at -1.57 rad (-90°) and +1.57 rad (+90°)

### Test 4: Friction

**What to expect**: Robot doesn't slide on ground (friction coefficient mu=0.8 holds it in place).

**Verification**: With no velocity commands, robot should remain stationary after landing (not drift).

---

## Troubleshooting

### Issue: Robot Sinks Through Ground

**Symptoms**: Robot falls through ground plane or penetrates floor.

**Causes**:
1. Missing `<collision>` geometry in URDF
2. Zero or missing `<inertial>` properties
3. Contact parameters (`kp`/`kd`) too low

**Fixes**:
- Ensure every `<link>` has `<collision>` tags matching visual geometry
- Add realistic mass and inertia tensors
- Increase contact stiffness: `<kp>1000000.0</kp>` in `<gazebo>` tags
- Verify ground plane is loaded: `ros2 topic echo /gazebo/model_states`

### Issue: Gazebo Crashes on Startup

**Symptoms**: Segmentation fault, "plugin not found", or immediate termination.

**Causes**:
1. Missing Gazebo plugins
2. WSL2 without X11 forwarding
3. Incompatible Gazebo version

**Fixes**:
```bash
# Install missing plugins
sudo apt install ros-humble-gazebo-plugins ros-humble-gazebo-ros

# For WSL2: Enable X server
export DISPLAY=:0

# Verify Gazebo version is 11.x
gazebo --version
```

### Issue: Physics Simulation is Extremely Slow (<10 FPS)

**Symptoms**: Gazebo GUI freezes, physics lags real-time.

**Causes**:
1. Complex collision meshes
2. High physics update rate on weak hardware
3. Too many active contacts

**Fixes**:
- **Reduce physics rate**: Change `<real_time_update_rate>` from 1000 to 500 in world file
- **Use headless mode**: Replace `gazebo` with `gzserver` (no GUI), visualize in RViz
- **Simplify collision geometry**: Use primitive shapes (box, cylinder) instead of mesh files
- **Disable shadows**: `<shadows>false</shadows>` in `<scene>` tag

---

## Key Takeaways

After completing this chapter, you should understand:

1. **Gazebo World Files**: How to define simulation environments with physics settings, ground planes, and lighting using SDF XML format
2. **URDF Gazebo Extensions**: How to add `<gazebo>` tags for friction (`mu1`/`mu2`), materials, and contact properties (`kp`/`kd`)
3. **Inertial Properties**: Why mass and inertia tensors are critical for physics simulation, and how to calculate them for basic shapes
4. **ROS 2 Launch Files**: How to start Gazebo, spawn robots, and integrate with robot_state_publisher using Python launch files
5. **Physics Testing**: How to verify gravity, collision detection, joint limits, and friction are working correctly

**Real-World Application**: These skills enable you to test robot locomotion algorithms (walking, balancing), manipulator motion planning (collision-free paths), and sensor-based navigation (obstacle detection) in safe, reproducible simulation environments before deploying to expensive hardware.

---

## Practice Exercises

1. **Modify Gravity**: Change the `<gravity>` vector in `simple_humanoid.world` to simulate:
   - Moon gravity: `0 0 -1.62`
   - Mars gravity: `0 0 -3.71`
   - Microgravity: `0 0 -0.1`

   **Question**: How does reduced gravity affect the robot's fall time and landing impact?

2. **Experiment with Friction**: Set `<mu1>0.1</mu1>` (ice-like surface) in the base_link Gazebo tag.
   **Question**: Does the robot slide after landing? Why?

3. **Add Obstacles**: Include a box model in the world file:
   ```xml
   <include>
     <uri>model://box</uri>
     <pose>1 0 0.5 0 0 0</pose>  <!-- 1m in front of robot -->
   </include>
   ```
   **Task**: Launch Gazebo and verify the robot collides with the box (doesn't pass through).

4. **Adjust Joint Damping**: Increase `<dynamics damping="5.0" />` for the shoulder joint.
   **Question**: How does this affect the joint's response when you manually move it in Gazebo GUI?

5. **Performance Tuning**: Run Gazebo on low-end hardware and reduce `<real_time_update_rate>` from 1000 to 250.
   **Task**: Measure FPS improvement using `gazebo --verbose` (look for "Real time factor" in logs).

---

## Next Steps

Now that you understand Gazebo physics simulation, you can:

- **Proceed to Chapter 2**: Learn Unity rendering for photorealistic visualization (optional, requires dedicated GPU)
- **Proceed to Chapter 3**: Add simulated sensors (LiDAR, depth cameras, IMU) to the humanoid robot
- **Explore Advanced Gazebo**: Terrain generation, weather plugins, multi-robot simulations (see Gazebo Classic Tutorials)

**Download the example**: [gazebo_humanoid_world.zip](/examples/gazebo_humanoid_world.zip)

**Support Resources**:
- [Gazebo Classic Tutorials](http://classic.gazebosim.org/tutorials)
- [ROS 2 Humble + Gazebo Integration](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [SDF Format Specification](http://sdformat.org/)

---

## References

[1] N. Koenig and A. Howard, "Design and use paradigms for Gazebo, an open-source multi-robot simulator," in *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Sendai, Japan, 2004, pp. 2149-2154. [Online]. Available: https://ieeexplore.ieee.org/document/1389727

[2] S. Ivaldi, V. Padois, and F. Nori, "Tools for dynamics simulation of robots: a survey based on user feedback," in *IEEE Robotics & Automation Magazine*, vol. 23, no. 4, pp. 13-25, Dec. 2016. [Online]. Available: https://arxiv.org/abs/1402.7050 (Open Access Preprint)

[3] Open Robotics, "Gazebo Classic Documentation," 2023. [Online]. Available: http://classic.gazebosim.org/

[4] J. M. O'Kane, "A Gentle Introduction to ROS," University of South Carolina, 2014. [Online]. Available: https://www.cse.sc.edu/~jokane/agitr/ (Free Online Book)
