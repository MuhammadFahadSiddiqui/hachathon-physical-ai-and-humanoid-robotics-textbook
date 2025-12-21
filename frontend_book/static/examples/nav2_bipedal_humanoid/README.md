# Nav2 Bipedal Humanoid Navigation Examples

Autonomous navigation for bipedal humanoid robot using Nav2 with simplified circular footprint.

## Features

- **DWB Controller**: Dynamic Window Approach trajectory planner with bipedal-safe velocity limits
- **Circular Footprint**: Simplified 0.3m radius approximation (no complex footstep planning)
- **2D Planar LiDAR**: 240° FOV, 4m range for obstacle detection
- **Conservative Tuning**: Max velocity 0.3 m/s, max angular velocity 0.5 rad/s for stability
- **Flat Ground Only**: Does not handle stairs, slopes, or uneven terrain

## Quick Start

### 1. Install Nav2

```bash
sudo apt update
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-gazebo-ros-pkgs
```

### 2. Launch Gazebo with Humanoid Robot

```bash
# Terminal 1: Start Gazebo
ros2 launch gazebo_ros gazebo.launch.py

# Load humanoid URDF
ros2 run gazebo_ros spawn_entity.py -entity humanoid \
  -file simple_humanoid_nav.urdf \
  -x 0 -y 0 -z 1.0
```

### 3. Start Nav2 Navigation Stack

```bash
# Terminal 2: Launch Nav2
ros2 launch nav2_bipedal_humanoid nav2_humanoid.launch.py
```

### 4. Visualize in RViz and Send Goals

```bash
# Terminal 3: Open RViz
ros2 run rviz2 rviz2

# In RViz:
# 1. Set Fixed Frame to "map"
# 2. Add -> By topic -> /map -> Map
# 3. Add -> By topic -> /local_costmap/costmap -> Map
# 4. Add -> By topic -> /scan -> LaserScan
# 5. Add -> By display type -> RobotModel
# 6. Click "2D Nav Goal" button and click target location
```

### 5. Monitor Navigation Status

```bash
# Check if navigation is active
ros2 topic echo /navigate_to_pose/_action/status

# Monitor velocity commands
ros2 topic echo /cmd_vel

# View costmap updates
ros2 topic echo /local_costmap/costmap_updates
```

## Configuration Files

- `launch/nav2_humanoid.launch.py` - Nav2 bringup with bipedal configuration
- `config/nav2_params.yaml` - DWB controller, costmaps, behavior tree
- `config/costmap_params.yaml` - Global + local costmap parameters
- `config/footstep_planner.yaml` - Future reference for advanced footstep planning
- `urdf/simple_humanoid_nav.urdf` - Humanoid model with 2D LiDAR

## Performance Expectations

| Metric | Value |
|--------|-------|
| Planning Frequency | 20 Hz |
| Max Linear Velocity | 0.3 m/s |
| Max Angular Velocity | 0.5 rad/s |
| Goal Tolerance | 0.25 m |
| Costmap Update Rate | 5 Hz (local), 1 Hz (global) |

## Tuning Parameters

### For Faster Navigation (Less Stable)
Edit `config/nav2_params.yaml`:
```yaml
max_vel_x: 0.5        # Increase from 0.3
max_vel_theta: 1.0    # Increase from 0.5
```

### For More Cautious Navigation
Edit `config/nav2_params.yaml`:
```yaml
inflation_radius: 0.8  # Increase from 0.55
robot_radius: 0.35     # Increase from 0.3
```

### For Better Obstacle Avoidance
Edit `config/nav2_params.yaml`:
```yaml
update_frequency: 10.0  # Increase from 5.0 (local costmap)
sim_time: 2.0          # Increase from 1.7 (DWB trajectory time)
```

## Troubleshooting

### Robot Won't Move

**Problem**: Navigation goal accepted but no movement
**Solution**:
```bash
# Check if cmd_vel is being published
ros2 topic echo /cmd_vel

# Verify localization
ros2 topic echo /odom

# Check if planner found a valid path
ros2 topic echo /plan
```

### Robot Oscillates or Spins

**Problem**: Unstable trajectory generation
**Solution**: Reduce velocity limits and increase DWB critic weights
```yaml
# In nav2_params.yaml
max_vel_x: 0.2
PathAlign.scale: 64.0   # Increase from 32.0
GoalDist.scale: 48.0    # Increase from 24.0
```

### Costmap Not Updating

**Problem**: No obstacles visible in local costmap
**Solution**:
```bash
# Verify LiDAR data
ros2 topic echo /scan

# Check frame transforms
ros2 run tf2_ros tf2_echo map base_link

# Restart costmap node
ros2 lifecycle set /local_costmap/lifecycle_manager shutdown
ros2 lifecycle set /local_costmap/lifecycle_manager startup
```

### Planner Fails to Find Path

**Problem**: "No valid path found" error
**Solution**: Increase planner tolerance or check map connectivity
```yaml
# In nav2_params.yaml (planner_server)
tolerance: 1.0          # Increase from 0.5
allow_unknown: true     # Enable planning through unknown space
```

### Robot Gets Stuck Near Obstacles

**Problem**: Robot stops far from obstacles
**Solution**: Reduce inflation radius
```yaml
# In costmap_params.yaml
inflation_radius: 0.4   # Reduce from 0.55
```

## Limitations

1. **Flat Ground Only**: No terrain adaptation or footstep kinematics
2. **Static Balance**: Does not handle dynamic walking gait
3. **Simplified Footprint**: Circular approximation ignores leg configuration
4. **No Step Planning**: Cannot navigate stairs or step over obstacles

## Advanced Features (Future Research)

For full bipedal navigation with footstep planning, see:

- **Lee et al. (2024)**: "Adaptive Footstep Planning for Humanoid Navigation in Cluttered Environments"
  https://ieeexplore.ieee.org/document/10610234

- **Li et al. (2024)**: "Real-Time ZMP-Based Footstep Planning for Bipedal Robots"
  https://doi.org/10.1177/02783649231225678

Recommended ROS 2 packages for advanced bipedal navigation:
- `humanoid_navigation` - Footstep planner with ZMP stability
- `bipedal_locomotion_framework` - Full-body motion planning
- `whole_body_state_msgs` - Extended state representation

## Testing Checklist

- [ ] Gazebo launches successfully with humanoid model
- [ ] LiDAR data visible on `/scan` topic (240° FOV)
- [ ] Nav2 stack starts without errors
- [ ] RViz shows map, costmaps, and robot model
- [ ] 2D Nav Goal triggers path planning
- [ ] Robot reaches goal within 0.25m tolerance
- [ ] Dynamic obstacles trigger replanning
- [ ] No collisions with obstacles during navigation

## License

Educational use - MIT License
Based on ROS 2 Humble and Nav2 navigation stack.
