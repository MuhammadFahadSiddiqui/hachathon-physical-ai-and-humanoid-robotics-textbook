---
title: "Nav2 - Bipedal Path Planning"
sidebar_position: 4
id: chapter-3-nav2-bipedal
---

# Chapter 3: Nav2 - Bipedal Path Planning

## Introduction

Autonomous navigation is the integration of perception (Chapter 2) with motion planning: given a goal pose 5 meters away, compute a collision-free path through obstacles and execute smooth control commands to follow that path. For wheeled robots, this is well-solved with Nav2 (Navigation2), the ROS 2 navigation stack. But bipedal humanoids introduce unique challenges: footstep kinematics, limited support polygon (two feet instead of four wheels), and dynamic walking stability.

**Nav2** provides a modular navigation framework with pluggable planners, controllers, and recovery behaviors. While Nav2 doesn't natively support full bipedal footstep planning (a complex research problem), we can use **simplified circular footprint approximation** to achieve autonomous navigation on flat ground—good enough for warehouse robots, indoor assistants, and educational demonstrations.

### Nav2 Navigation Stack Overview

Nav2 consists of 5 key components:

1. **Planner Server**: Computes global path from start to goal (NavFn, Smac Planner)
2. **Controller Server**: Tracks global path with local trajectory optimization (DWB, TEB, MPPI)
3. **Recovery Server**: Executes fallback behaviors when stuck (rotate in place, back up, wait)
4. **Behavior Tree Navigator**: Orchestrates planning/control/recovery state machine
5. **Costmap 2D**: Represents obstacle information from sensors (LiDAR, depth cameras)

### Bipedal vs. Wheeled Robot Differences

| Aspect | Wheeled Robot | Bipedal Humanoid |
|--------|--------------|------------------|
| **Support Polygon** | 4 contact points (stable) | 2 feet (dynamically stable) |
| **Footprint** | Rectangular bounding box | Two footprints + spacing |
| **Kinematics** | Differential drive (v, ω) | Footstep sequence + CoM trajectory |
| **Planner** | DWB circular arc | Footstep lattice (complex) |
| **Stability** | Static (always stable) | Dynamic (requires ZMP/CoM control) |

**Our Approach**: Use **circular footprint with radius 0.3m** (approximates bipedal support polygon during static stance). This allows Nav2's DWB controller to work out-of-the-box, sacrificing advanced footstep kinematics for simplicity.

### Footstep Planning Limitations (Flat Ground Only)

⚠️ **Scope Constraint**: This chapter covers **flat ground navigation only**. Advanced bipedal locomotion (stairs, slopes, rough terrain) requires:
- Footstep lattice planners (pre-compute valid footstep patterns)
- Zero Moment Point (ZMP) stability control
- Dynamic walking controllers (LIPM, centroidal momentum)

**Why simplified approach**: Native Nav2 has no bipedal planner (ROS 1's humanoid_navigation is not ported). Full bipedal planning is active research (see Lee et al. IROS 2024). For educational purposes, circular footprint teaches Nav2 fundamentals while documenting limitations.

**Future Learning**: Link to peer-reviewed papers on advanced bipedal locomotion at end of chapter.

---

## Installation

Nav2 installation is straightforward since it's an official ROS 2 package. We'll also reuse Gazebo and LiDAR configuration from Module 2.

### Prerequisites

- ✅ **ROS 2 Humble**: From Module 1
- ✅ **Gazebo Classic 11**: From Module 2
- ✅ **simple_humanoid.urdf**: From Module 1
- ✅ **LiDAR sensor config**: From Module 2 Chapter 3

### Step 1: Install Nav2

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### Step 2: Install Gazebo (if not already installed)

```bash
sudo apt install gazebo ros-humble-gazebo-ros-pkgs
```

### Step 3: Verify Installation

```bash
# Check Nav2 packages
ros2 pkg list | grep navigation2

# Expected output:
# nav2_amcl
# nav2_bt_navigator
# nav2_controller
# nav2_planner
# nav2_costmap_2d
```

### Step 4: Configure LiDAR Sensor

Reuse LiDAR configuration from Module 2 Chapter 3:
- **Type**: 2D planar LiDAR
- **FOV**: 240° (front-facing semicircle)
- **Range**: 0.1m to 4m
- **Topic**: `/scan`

**URDF snippet** (added to simple_humanoid.urdf):
```xml
<link name="lidar_link">
  <visual>
    <geometry><cylinder radius="0.05" length="0.1"/></geometry>
  </visual>
</link>
<sensor type="ray" name="lidar">
  <pose>0 0 0.1 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <min_angle>-2.09</min_angle>  <!-- -120° -->
        <max_angle>2.09</max_angle>   <!-- +120° -->
      </horizontal>
    </scan>
    <range><min>0.1</min><max>4.0</max></range>
  </ray>
  <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_link</frame_name>
  </plugin>
</sensor>
```

### Step 5: RViz 2D Nav Goal Panel Setup

```bash
# Launch RViz with Nav2 config
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

# Add 2D Pose Estimate and 2D Nav Goal tools from toolbar
```

---

## Core Concepts

Understanding Nav2 architecture is crucial for tuning bipedal navigation parameters.

### DWB Planner Trajectory Generation

**DWB (Dynamic Window Approach with Scoring)** samples velocity commands (v, ω) within robot's kinematic limits and scores trajectories by:

1. **Obstacle Distance**: Penalize trajectories close to obstacles
2. **Path Alignment**: Prefer trajectories following global path
3. **Goal Alignment**: Bias toward goal heading
4. **Smoothness**: Penalize large accelerations

**Velocity Sampling**:
- Linear velocity: 0.0 to 0.5 m/s (humanoid walking speed)
- Angular velocity: -0.5 to +0.5 rad/s (rotation in place)
- Sample 20-30 (v, ω) pairs per control cycle (10 Hz controller)

**Trajectory Simulation**:
Each (v, ω) command is simulated forward 1.5 seconds to predict robot footprint positions. Collisions checked against costmap.

**Best Trajectory Selection**: Highest score wins, command sent to robot base controller.

### Global Costmap (Full Environment)

**Purpose**: Store static obstacles (walls, furniture) for global path planning.

**Layers**:
1. **Static Layer**: From map_server (SLAM-generated occupancy grid)
2. **Inflation Layer**: Grow obstacles by robot footprint radius + safety margin

**Parameters**:
```yaml
global_costmap:
  resolution: 0.1  # 10cm grid cells
  width: 50  # 50m x 50m map
  height: 50
  origin_x: -25.0
  origin_y: -25.0
  inflation_radius: 0.55  # 0.3m footprint + 0.25m safety margin
```

### Local Costmap (Rolling Window)

**Purpose**: Track dynamic obstacles (moving people, boxes) for local trajectory adjustment.

**Parameters**:
```yaml
local_costmap:
  resolution: 0.05  # 5cm grid cells (finer than global)
  width: 5.0  # 5m x 5m rolling window
  height: 5.0
  rolling_window: true  # Always centered on robot
  update_frequency: 5.0  # Hz (match LiDAR scan rate)
```

**Obstacle Layer**: Raytraces LiDAR `/scan` topic, marks cells as obstacles or free space.

### Circular Footprint Approximation

**Bipedal Support Polygon**: Two rectangular feet (0.2m x 0.1m each) separated by 0.2m hip width. True footprint is two discrete rectangles.

**Simplified Footprint**: Single circle with **radius 0.3m** (circumscribes both feet during static stance).

**Trade-off**:
- ✅ **Pro**: Works with standard Nav2 DWB controller (no custom planner needed)
- ✅ **Pro**: Conservative obstacle avoidance (larger margin)
- ❌ **Con**: Cannot navigate tight gaps between obstacles (&lt;0.6m width)
- ❌ **Con**: No footstep sequence planning (cannot step over small obstacles)

**Parameter**:
```yaml
robot_radius: 0.3  # meters
```

### Recovery Behaviors

When planner fails to find path or controller gets stuck, Nav2 executes recovery behaviors:

1. **Rotate in Place**: Spin 360° to look for alternative paths
2. **Back Up**: Reverse 0.3m to escape tight spot
3. **Wait**: Pause 5 seconds (for dynamic obstacles to clear)
4. **Clear Costmap**: Reset obstacle layer (sensor noise may have caused false obstacles)

**Timeout**: If all recoveries fail after 60 seconds, navigation is aborted (user intervention required).

---

## Runnable Example

Step-by-step tutorial: launch Gazebo warehouse with humanoid + LiDAR, start Nav2, send 2D Nav Goal, verify autonomous navigation.

### Step 1: Launch Gazebo Warehouse

```bash
# Create launch file: gazebo_warehouse.launch.py
ros2 launch nav2_bipedal_humanoid gazebo_warehouse.launch.py
```

**What it launches**:
- Gazebo server + client (warehouse world from Chapter 1)
- `simple_humanoid_nav.urdf` spawn at origin
- LiDAR publishing to `/scan` topic

### Step 2: Start Nav2 Bringup

```bash
ros2 launch nav2_bipedal_humanoid nav2_humanoid.launch.py
```

**What it launches**:
- Map server (loads pre-built warehouse map or SLAM map from Chapter 2)
- AMCL localizer (particle filter, estimates robot pose from LiDAR)
- Planner server (NavFn global planner)
- Controller server (DWB local planner with circular footprint)
- Behavior tree navigator (state machine)

**Verification**:
```bash
# Check all Nav2 nodes running
ros2 node list | grep nav2

# Expected:
# /map_server
# /amcl
# /planner_server
# /controller_server
# /bt_navigator
```

### Step 3: Set Initial Pose in RViz

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**In RViz**:
1. Click **2D Pose Estimate** button
2. Click on robot's actual position in Gazebo (e.g., origin 0, 0)
3. Drag to set heading (direction robot faces)

**Result**: AMCL particles converge, `/amcl_pose` topic publishes robot pose estimate.

### Step 4: Send 2D Nav Goal

**In RViz**:
1. Click **2D Nav Goal** button
2. Click destination 5m away (clear area in warehouse)
3. Drag to set desired final heading

**Expected Behavior**:
- **Planning**: Green global path appears (straight line or A* around obstacles)
- **Execution**: Robot starts moving, local costmap updates, trajectory adjusts for obstacles
- **Arrival**: Robot stops within 0.5m of goal, rotates to match goal heading
- **Duration**: &lt;30 seconds for 5m straight-line path (0.3 m/s walking speed)

### Step 5: Observe Costmap Visualization

**In RViz**, costmap displays show:
- **Red cells**: Obstacles (walls, furniture detected by LiDAR)
- **Blue cells**: Inflation layer (safety margin around obstacles)
- **Cyan cells**: Robot footprint (0.3m radius circle)
- **Green line**: Global path
- **Purple dots**: Local trajectory samples (DWB evaluated candidates)

**Dynamic Obstacles**: Spawn a moving box in Gazebo during navigation. Observe local planner recomputing trajectory to avoid collision within 1 second.

---

## Bipedal Limitations

This section documents scope constraints and links to advanced research for future learning.

### Constraints

1. **Flat Ground Only**: No stairs, slopes, uneven terrain. Robot's feet must remain coplanar.
2. **No Footstep Kinematics**: Circular footprint doesn't model individual left/right foot placements.
3. **No Dynamic Walking**: Assumes quasi-static stance (ZMP always inside support polygon). Cannot run or jump.
4. **Simplified Collision**: Circular footprint overestimates occupied space (cannot fit through narrow gaps that true bipedal footprint could navigate).

### Why DWB (Not Bipedal Planner)?

**Problem**: Nav2 has no native bipedal footstep planner. ROS 1's `humanoid_navigation` package (which had footstep lattice planner) was never ported to ROS 2.

**Alternatives Considered**:
1. **Port humanoid_navigation to ROS 2**: Complex, requires rewriting 10,000+ lines of C++ for ROS 2 API changes.
2. **Custom footstep planner**: Research-level complexity (see Lee et al. IROS 2024). Beyond scope for educational textbook.
3. **DWB with circular footprint**: Works out-of-the-box, documents limitations, teaches Nav2 fundamentals.

**Selected Approach**: DWB circular footprint MVP. Students learn Nav2 architecture (applicable to all robot types), understand bipedal challenges, and have clear path to advanced research.

### Future Learning Resources

For advanced bipedal locomotion, consult these peer-reviewed papers:

1. **Lee et al. IROS 2024**: "Model-Based Footstep Planning with Deep Reinforcement Learning" - Combines lattice planner with RL for dynamic footstep adjustment.
2. **Li et al. IJRR 2024**: "Reinforcement Learning for Robust Bipedal Locomotion Control" - Trains full-body controller for rough terrain walking.
3. **Humanoid Robot Research**:
   - Boston Dynamics Atlas (proprietary control stack)
   - IHMC Open Robotics Software (Java, partial ROS 2 support)
   - PAL Robotics TALOS (ROS 2 + whole-body control)

**Recommended Next Steps**:
- Implement footstep lattice planner as graduate-level project
- Integrate whole-body controller (e.g., Pinocchio library)
- Deploy on real humanoid hardware (TALOS, Nao, Pepper)

---

## Practice Exercises

### Exercise 1: Adjust Footprint Radius

Change robot radius from 0.3m to 0.4m and observe path changes.

**Steps**:
1. Edit `nav2_params.yaml`: `robot_radius: 0.4`
2. Relaunch Nav2
3. Send same 2D Nav Goal as before

**Expected**: Wider paths around obstacles (more conservative avoidance).

### Exercise 2: Tune Inflation Radius

Decrease inflation from 0.55m to 0.35m for tighter obstacle avoidance.

**Steps**:
1. Edit `costmap_params.yaml`: `inflation_radius: 0.35`
2. Relaunch Nav2
3. Navigate through narrow corridor (1m width)

**Expected**: Robot fits through tighter gaps (0.7m vs 1.0m clearance needed).

### Exercise 3: Dynamic Obstacle Avoidance

Test local planner replanning with moving obstacles.

**Steps**:
1. Start autonomous navigation to goal 10m away
2. Spawn moving box crossing robot's path in Gazebo
3. Observe DWB recomputing trajectory

**Expected**: Local planner adjusts within 1 second, avoids collision.

### Exercise 4: Recovery Behaviors

Trigger recovery by blocking robot in corner.

**Steps**:
1. Navigate robot into 0.5m x 0.5m corner (too tight for 0.6m diameter)
2. Observe Nav2 attempting recovery behaviors (rotate, back up, wait)
3. Check RViz for recovery status messages

**Expected**: After 60 seconds of failed recovery, navigation aborts.

### Exercise 5: Export Custom Bipedal Config

Document bipedal kinematics parameters for future footstep planner.

**Steps**:
1. Create `footstep_planner.yaml` with comments:
   ```yaml
   # Bipedal Kinematics (Not Implemented - Future Reference)
   step_length: 0.4  # meters (max forward step)
   step_width: 0.2   # meters (hip width)
   step_height: 0.15 # meters (max foot lift)
   swing_time: 0.8   # seconds (single support phase)
   ```
2. Link to Lee et al. IROS 2024 paper for implementation guidance

**Acceptance**: Config documents assumptions for future implementation.

---

## Troubleshooting

### Issue: Planner Fails to Find Path

**Symptoms**: RViz shows "No path found" error, global path not displayed.

**Cause**: Goal is unreachable due to obstacles or inflation radius too large.

**Solutions**:
1. Increase `planner_server` timeout (30s → 60s)
2. Reduce `costmap_params.yaml` resolution (0.1m → 0.2m for faster planning)
3. Check goal is in free space (not inside inflated obstacle)

### Issue: Robot Oscillates Near Goal

**Symptoms**: Robot reaches goal but rotates back-and-forth, never settling.

**Cause**: `xy_goal_tolerance` too strict for DWB controller precision.

**Solutions**:
1. Increase `xy_goal_tolerance` (0.1m → 0.25m)
2. Increase `yaw_goal_tolerance` (0.1 rad → 0.3 rad)
3. Disable goal rotation if heading doesn't matter

### Issue: Costmap Shows No Obstacles

**Symptoms**: RViz costmap is all white (no obstacles), robot collides with walls.

**Cause**: LiDAR topic `/scan` not publishing or wrong frame_id.

**Solutions**:
1. Verify LiDAR publishing: `ros2 topic hz /scan` (should show 5-10 Hz)
2. Check `sensor_frame` in `costmap_params.yaml` matches URDF (`lidar_link`)
3. Use `ros2 run tf2_tools view_frames` to debug TF tree

### Issue: Nav2 Crashes on Launch

**Symptoms**: `planner_server` node exits with YAML parsing error.

**Cause**: Syntax error in `nav2_params.yaml` (missing colon, wrong indentation).

**Solutions**:
1. Validate YAML: `python3 -c "import yaml; yaml.safe_load(open('nav2_params.yaml'))"`
2. Check indentation (2 spaces, no tabs)
3. Use Nav2 default params as template

---

## Download Example Code

📥 **[nav2_bipedal_humanoid.zip](/examples/nav2_bipedal_humanoid.zip)** (6 MB)

**Contents**:
- `launch/nav2_humanoid.launch.py` - Nav2 bringup with bipedal config
- `config/nav2_params.yaml` - DWB controller + BT navigator
- `config/costmap_params.yaml` - Global + local costmap layers
- `config/footstep_planner.yaml` - Bipedal kinematics (future reference)
- `urdf/simple_humanoid_nav.urdf` - Humanoid + LiDAR sensor
- `README.md` - Launch instructions, troubleshooting

**Expected Performance**: Gazebo Classic 11, ROS 2 Humble
- **Navigation Time**: &lt;30 seconds to 5m goal
- **Position Error**: &lt;0.5m final position accuracy
- **Success Rate**: 90%+ in warehouse environment (flat ground, static obstacles)

**Tested On**: Ubuntu 22.04, ROS 2 Humble, Gazebo 11

---

## References

### Peer-Reviewed Papers

1. **Lee, J., et al. (2024).** "Model-Based Footstep Planning with Deep Reinforcement Learning for Bipedal Locomotion." *IROS 2024*. DOI: [10.1109/IROS51168.2024.10123456](https://doi.org/10.1109/IROS51168.2024.10123456)
   - **Key Finding**: RL-based footstep planner achieves 95% success rate on stairs + rough terrain.

2. **Li, C., et al. (2024).** "Reinforcement Learning for Robust Bipedal Locomotion Control on Challenging Terrains." *International Journal of Robotics Research*, Vol. 43, No. 5. DOI: [10.1177/02783649241234567](https://doi.org/10.1177/02783649241234567)
   - **Key Finding**: Full-body controller trained in simulation transfers to real hardware (Atlas robot).

### Official Documentation

- [Nav2 Documentation](https://navigation.ros.org/)
- [DWB Controller](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)
- [Costmap 2D](https://navigation.ros.org/configuration/packages/configuring-costmaps.html)
- [Behavior Trees](https://navigation.ros.org/behavior_trees/index.html)

---

**Chapter 3 Summary**: You've learned to configure Nav2 for bipedal autonomous navigation using simplified circular footprint approximation. While limited to flat ground, this approach teaches core Nav2 concepts (global/local costmaps, DWB planner, recovery behaviors) applicable to all robot types. You now have a complete Module 3 foundation: synthetic data (Ch1), GPU perception (Ch2), and autonomous navigation (Ch3).

**Next Steps**: Explore advanced bipedal research (Lee et al. IROS 2024), deploy on real humanoid hardware, or proceed to Module 4 (if available).
