# Research: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: 002-digital-twin-simulation
**Date**: 2025-12-19
**Phase**: Phase 0 - Technology Research and Decision Documentation

## Overview

This document captures research findings and technology decisions for implementing Module 2: The Digital Twin (Gazebo & Unity) as part of the AI-Driven Book with Embedded RAG Chatbot project. Module 2 focuses on delivering instructional content for physics simulation (Gazebo), photorealistic rendering (Unity), and sensor simulation through Docusaurus documentation.

## Clarification: Project Scope

**IMPORTANT**: This module creates **educational documentation**, not simulation software installation.

- **What we're building**: Markdown chapters teaching Gazebo/Unity usage
- **What we're NOT building**: Gazebo/Unity installed within Docusaurus
- **Deliverables**: Documentation (.md files), example files (URDF, world files, Unity scenes), diagrams
- **Target audience**: Students with laptops who will install Gazebo/Unity locally

## Key Research Areas

### 1. Gazebo Integration Strategy for Educational Content

**Decision**: Provide downloadable example files + installation documentation, NOT embedded Gazebo runtime

**Rationale**:
- **Separation of Concerns**: Gazebo is a C++ physics simulator (requires system installation), Docusaurus is a static site generator (JavaScript/React)
- **Student Hardware**: Students install Gazebo locally on Ubuntu 22.04 (per Module 1 prerequisites)
- **Example Files**: Provide `.world` files and Gazebo-enabled URDFs as downloadable `.zip` archives in `frontend_book/static/examples/`
- **Documentation Approach**: Step-by-step tutorials explaining Gazebo installation, world file syntax, physics parameters

**Alternatives Considered**:
- **WebAssembly Gazebo**: Not production-ready; Gazebo Sim (Ignition) doesn't have stable WASM builds
- **Cloud Gazebo (AWS RoboMaker)**: Violates free-tier principle (per constitution)
- **Embedded 3D viewer**: Could show URDF models but can't simulate physics; misleading to students

**Rejection Reasoning**: Gazebo must run locally for realistic physics simulation. Docusaurus provides documentation and examples only.

---

### 2. Unity Integration Strategy for Educational Content

**Decision**: Provide Unity project template as downloadable archive + ROS-Unity bridge setup documentation

**Rationale**:
- **Unity Licensing**: Students use Unity Personal (free <$100k revenue) installed locally
- **Unity-ROS 2 Bridge**: Document ROS-TCP-Connector package installation and configuration
- **Example Unity Scene**: Provide Unity project with simple_humanoid imported, materials configured, cameras set up
- **Platform Compatibility**: Unity runs on Windows/macOS/Linux; documentation covers all platforms
- **Distribution**: Unity project as `.zip` in `frontend_book/static/examples/unity_humanoid_scene.zip`

**Unity Version**: Unity 2021 LTS (Long-Term Support until 2024, stable ROS integration)

**Alternatives Considered**:
- **WebGL Unity Build**: Could embed in Docusaurus, but doesn't support ROS-Unity bridge (needs native networking)
- **Unity Simulation (Cloud)**: Paid service, violates constitution
- **Video Demonstrations Only**: Less interactive, doesn't let students practice

**Rejection Reasoning**: Students need hands-on Unity practice with ROS 2 integration. Downloadable project templates enable this without violating free-tier constraints.

---

### 3. Sensor Simulation Documentation Approach

**Decision**: Document Gazebo sensor plugins with configuration examples + ROS 2 topic verification commands

**Rationale**:
- **Gazebo Plugins**: Use standard gazebo_ros_pkgs plugins (libgazebo_ros_ray for LiDAR, libgazebo_ros_camera for depth, libgazebo_ros_imu for IMU)
- **Documentation Structure**: For each sensor type:
  1. Conceptual explanation (how LiDAR works, ray-tracing simulation)
  2. URDF plugin syntax with annotated XML
  3. Configurable parameters (noise models, update rates, ranges)
  4. ROS 2 topic inspection commands (`ros2 topic echo /scan`, `ros2 topic hz /camera/depth`)
- **Example URDFs**: Extend simple_humanoid.urdf with `<gazebo>` tags for each sensor type

**Sensor Specifications** (matching real-world sensors for realism):
- **LiDAR**: Hokuyo URG-04LX specs (240° FOV, 4m range, 0.36° angular resolution)
- **Depth Camera**: Intel RealSense D435 specs (87° FOV, 0.3-10m range, 1280x720 resolution)
- **IMU**: Bosch BNO055 specs (±2000°/s gyro, ±16g accel, quaternion output)

**Alternatives Considered**:
- **Generic Sensor Parameters**: Less educational; students wouldn't learn real sensor characteristics
- **Multiple Sensor Vendors**: Too complex for introductory module; stick to one well-documented example per type
- **Custom Sensor Plugins**: Over-engineering; standard Gazebo plugins sufficient

---

### 4. Docusaurus Content Organization

**Decision**: Three chapters as separate Markdown files under `frontend_book/docs/module-2/`

**File Structure**:
```
frontend_book/docs/module-2/
├── index.md                     # Module 2 landing page (overview, prerequisites, learning objectives)
├── chapter-1-gazebo-physics.md  # Chapter 1: Gazebo Simulation (2500-4000 words)
├── chapter-2-unity-rendering.md # Chapter 2: Unity Rendering (2500-4000 words)
└── chapter-3-sensor-simulation.md # Chapter 3: Sensor Simulation (2500-4000 words)
```

**Sidebar Configuration** (in `sidebars.js`):
```javascript
{
  module2: [
    'module-2/index',
    'module-2/chapter-1-gazebo-physics',
    'module-2/chapter-2-unity-rendering',
    'module-2/chapter-3-sensor-simulation',
  ],
}
```

**Rationale**:
- **Consistency**: Mirrors Module 1 structure (3 chapters per module)
- **Navigation**: Clear sidebar hierarchy
- **SEO**: Separate files improve search indexing
- **Maintenance**: Easier to update individual chapters

**Alternatives Considered**:
- **Single Long File**: Harder to navigate, worse for incremental reading
- **Nested Subdirectories**: Over-complicates structure for 3 chapters
- **MDX Components**: Deferred to future modules; keep Module 2 simple Markdown for consistency with Module 1

---

### 5. Diagram and Visualization Tools

**Decision**: Mermaid.js for architecture diagrams + static images (PNG/SVG) for physics concepts

**Mermaid Use Cases**:
- **Simulation Pipeline**: ROS 2 → Gazebo → Physics Engine → Sensor Plugins → ROS 2 Topics
- **Unity-ROS Bridge**: ROS 2 Topics ← TCP/IP → Unity ROS-TCP-Connector → Unity GameObjects
- **Data Flow**: URDF → Gazebo Parser → SDF → Physics Solver → Collision Detection

**Static Image Use Cases**:
- **Physics Concepts**: Gravity vectors, collision geometry, joint constraints (created with draw.io or Excalidraw)
- **Unity UI Screenshots**: Inspector panel, Scene view, Game view (actual Unity screenshots with annotations)
- **Sensor Ray-Casting**: LiDAR ray visualization, depth camera frustum (3D diagrams)

**Image Storage**: `frontend_book/static/img/module-2/` with descriptive filenames (e.g., `gazebo-physics-pipeline.png`, `lidar-ray-casting.svg`)

**Rationale**:
- **Mermaid**: Text-based, version-controllable, renders in browser
- **Static Images**: Better for complex visuals that don't map well to Mermaid syntax
- **Free Tools**: draw.io (web-based), Excalidraw (open-source), Unity screenshots (free)

---

### 6. Example File Distribution Strategy

**Decision**: Store examples in `frontend_book/static/examples/` as downloadable `.zip` archives

**Example Files to Provide**:

1. **gazebo_humanoid_world.zip**:
   - `simple_humanoid.world` (Gazebo world file with ground plane, lighting, humanoid spawn)
   - `simple_humanoid_gazebo.urdf` (Module 1 URDF + Gazebo physics tags)
   - `launch/gazebo_demo.launch.py` (ROS 2 launch file to start Gazebo with humanoid)
   - `README.md` (setup instructions, how to run)

2. **gazebo_sensors.zip**:
   - `humanoid_with_lidar.urdf` (URDF + LiDAR plugin)
   - `humanoid_with_depth_camera.urdf` (URDF + depth camera plugin)
   - `humanoid_with_imu.urdf` (URDF + IMU plugin)
   - `launch/sensor_demo.launch.py` (Launch file to visualize sensor data in RViz)
   - `README.md` (sensor configuration guide)

3. **unity_humanoid_scene.zip**:
   - `UnityProject/` (Unity 2021 LTS project folder)
     - `Assets/Robots/simple_humanoid/` (Imported URDF as Unity prefab)
     - `Assets/Scenes/HumanoidDemo.unity` (Scene with humanoid, lighting, camera)
     - `Packages/manifest.json` (ROS-TCP-Connector package reference)
   - `README.md` (Unity installation, ROS-Unity bridge setup, scene usage)

**Download Links in Documentation**:
```markdown
Download the Gazebo world example: [gazebo_humanoid_world.zip](/examples/gazebo_humanoid_world.zip)
```

**Rationale**:
- **Self-Contained**: Each `.zip` has everything needed to run the example
- **Offline Access**: Students can download once, work offline
- **Version Control**: Binary `.zip` files tracked in Git (small sizes: <5MB each)
- **Docusaurus Static Assets**: `/static/` folder served as-is, no processing

---

### 7. Code Example Best Practices

**Decision**: Annotated XML (URDF/world files) with inline comments + Python launch scripts

**URDF Example Format**:
```xml
<robot name="simple_humanoid">
  <!-- Torso link with realistic inertia for physics simulation -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>  <!-- 10 kg torso -->
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <!-- ... -->
  </link>

  <!-- Gazebo-specific physics properties -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>  <!-- Visual material in Gazebo -->
    <mu1>0.8</mu1>  <!-- Friction coefficient (concrete-like surface) -->
    <mu2>0.8</mu2>
  </gazebo>
</robot>
```

**Python Launch File Format** (consistent with Module 1):
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo server with physics
        IncludeLaunchDescription('gazebo_ros', 'gzserver.launch.py'),

        # Spawn humanoid robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_humanoid', '-file', 'simple_humanoid_gazebo.urdf'],
            output='screen'  # Print spawn logs to console
        ),
    ])
```

**Rationale**:
- **Inline Comments**: Every parameter explained (students learn by reading)
- **Consistent Style**: Matches Module 1 Python examples
- **Runnable**: Copy-paste into ROS 2 workspace, execute with `ros2 launch`

---

### 8. ROS 2 Integration Testing Commands

**Decision**: Provide verification commands for students to validate their setup

**Verification Commands** (included in each chapter):

**Chapter 1 (Gazebo Physics)**:
```bash
# Verify Gazebo is running
ros2 node list | grep gazebo

# Check robot model is loaded
ros2 topic list | grep /joint_states

# Publish velocity command to test physics response
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" --once
```

**Chapter 3 (Sensor Simulation)**:
```bash
# Verify LiDAR is publishing
ros2 topic hz /scan  # Should show ~10 Hz update rate

# Inspect LiDAR data
ros2 topic echo /scan --once

# Check depth camera resolution
ros2 topic echo /camera/depth/image_raw --once | grep height
```

**Rationale**:
- **Hands-On Validation**: Students confirm setup works before proceeding
- **Debugging Aid**: If commands fail, students know where to troubleshoot
- **ROS 2 Skill Building**: Reinforces `ros2 topic` CLI usage from Module 1

---

### 9. Troubleshooting Section Content

**Decision**: Curate common errors from Gazebo/Unity community forums

**Research Sources**:
- **Gazebo Answers**: https://answers.gazebosim.org/
- **ROS Answers (Gazebo tag)**: https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:gazebo/
- **Unity-ROS GitHub Issues**: https://github.com/Unity-Technologies/ROS-TCP-Connector/issues

**Top 5 Errors for Module 2**:

1. **Gazebo: "Model is sinking through the ground plane"**
   - **Cause**: Missing or incorrect collision geometry in URDF
   - **Fix**: Add `<collision>` tags matching `<visual>` geometry, ensure inertia values are realistic

2. **Gazebo: "[WARN] Controller Spawner couldn't find the expected controller_manager ROS interface"**
   - **Cause**: Gazebo plugin not loaded or ROS 2 namespace mismatch
   - **Fix**: Verify `libgazebo_ros_control.so` is in `<gazebo>` tag, check `ros2 node list`

3. **Unity: "ROS-TCP-Connector connection failed: ECONNREFUSED"**
   - **Cause**: ROS 2 TCP endpoint not running or firewall blocking
   - **Fix**: Start `ros2 run ros_tcp_endpoint default_server_endpoint`, check port 10000

4. **Gazebo Sensors: "/scan topic not publishing"**
   - **Cause**: Sensor plugin not loaded or incorrect URDF syntax
   - **Fix**: Verify `<plugin filename="libgazebo_ros_ray.so">` is in `<gazebo>` tag, check Gazebo logs

5. **Unity: "URDF import failed: No robot description found"**
   - **Cause**: URDF file path incorrect or malformed XML
   - **Fix**: Validate URDF with `check_urdf`, use absolute path in Unity import settings

**Troubleshooting Section Format** (in each chapter):
```markdown
## Troubleshooting

### Error: [Symptom]
**Cause**: [Why it happens]
**Fix**:
1. [Step 1]
2. [Step 2]
3. [Verification command]
```

---

### 10. Performance Guidelines for Student Hardware

**Decision**: Provide hardware recommendations and optimization tips

**Minimum Hardware** (from spec assumptions):
- **Gazebo**: 8GB RAM, integrated GPU, Ubuntu 22.04
- **Unity**: 16GB RAM, dedicated GPU (GTX 1050 or better), any OS

**Performance Targets** (from spec success criteria):
- **Gazebo**: 20+ FPS with simple_humanoid + 10 environment objects
- **Unity**: 30+ FPS with high-quality lighting and shadows

**Optimization Tips** (documented in each chapter):

**Gazebo Optimization**:
- Reduce physics update rate: `<real_time_update_rate>500</real_time_update_rate>` (default 1000)
- Simplify collision geometry: Use primitive shapes (boxes, cylinders) instead of meshes
- Limit camera rendering: Set `<update_rate>10</update_rate>` for sensor cameras (not 60 FPS)

**Unity Optimization**:
- Use baked lighting: Pre-compute shadows (Lighting → Generate Lighting)
- Reduce shadow quality: Edit → Project Settings → Quality → Shadows = "Hard Shadows Only"
- Occlusion culling: Only render visible objects (Camera → Occlusion Culling)

**Fallback for Low-End Hardware**:
- **Gazebo**: Use headless mode (`gzserver` only, no GUI), visualize in RViz
- **Unity**: Reduce resolution to 1280x720, disable post-processing effects

---

### 11. Citation and Reference Strategy

**Decision**: Cite official documentation + 2 peer-reviewed papers per chapter

**Required Citations** (per spec FR-012):

**Chapter 1 (Gazebo)**:
- Official Gazebo Classic Tutorials: http://classic.gazebosim.org/tutorials
- Paper 1: "Simulation-Based Testing in Robotics" (if available on arXiv/open access)
- Paper 2: "Validation of Physics Simulation for Robot Manipulation" (open access)

**Chapter 2 (Unity)**:
- Unity-ROS Integration Documentation: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- Paper 1: "Photorealistic Rendering for Robot Perception Training" (open access)
- Paper 2: "Virtual Environments for Human-Robot Interaction" (open access)

**Chapter 3 (Sensors)**:
- gazebo_plugins Documentation: http://wiki.ros.org/gazebo_plugins
- Paper 1: "Sensor Noise Modeling in Robot Simulation" (open access)
- Paper 2: "LiDAR Simulation for Autonomous Navigation" (open access)

**Citation Format** (IEEE style in Markdown):
```markdown
## References

[1] "Gazebo Classic Tutorials," Open Robotics, 2023. [Online]. Available: http://classic.gazebosim.org/tutorials

[2] A. Smith et al., "Simulation-Based Testing in Robotics," *IEEE Robotics and Automation Letters*, vol. 7, no. 2, pp. 1234-1241, 2022. [Online]. Available: https://arxiv.org/abs/2201.12345
```

**Open Access Strategy**:
- Prioritize arXiv preprints, ResearchGate uploads, or institutional repositories
- Avoid paywalled journals (IEEE Xplore, Springer) unless open access version exists
- Verify links work before publication

---

## Technology Stack Summary

| Component | Technology | Version | License | Rationale |
|-----------|-----------|---------|---------|-----------|
| Static Site Generator | Docusaurus | 3.x | MIT | Existing project infrastructure |
| Gazebo Simulator | Gazebo Classic | 11.x | Apache 2.0 | ROS 2 Humble compatibility, stable physics |
| Unity Engine | Unity | 2021 LTS | Proprietary (Free Personal) | Industry-standard rendering, ROS integration |
| ROS-Unity Bridge | ROS-TCP-Connector | 0.7.0+ | Apache 2.0 | Official Unity-ROS 2 integration package |
| ROS 2 Distribution | Humble Hawksbill | Humble | Apache 2.0 | Consistent with Module 1 |
| Diagram Tool | Mermaid.js | 10.x | MIT | Text-based, Docusaurus native |
| URDF Validation | check_urdf | 1.13+ | BSD | Standard ROS 2 tool |
| Visualization | RViz 2 | 11.x | BSD | ROS 2 standard 3D viewer |

---

## Open Questions Resolved

### Q1: Should we embed Gazebo/Unity in the browser?
**Answer**: No. Provide downloadable examples + installation documentation. Gazebo requires native physics engine (ODE/Bullet), Unity WebGL doesn't support ROS-TCP networking. Students install locally.

### Q2: How to handle Unity licensing for students?
**Answer**: Document Unity Personal (free for revenue <$100k). Educational institutions can optionally use Unity Student plan. No paid features required for examples.

### Q3: What if students can't install Gazebo (Windows users)?
**Answer**: Recommend WSL2 with Ubuntu 22.04 (documented in Module 1 prerequisites). Provide troubleshooting for X11 forwarding to run Gazebo GUI on Windows.

### Q4: Should Unity chapter be optional?
**Answer**: Yes, as stated in spec risk mitigation. Unity is P2 priority, Gazebo (P1) stands alone. Students with low-end hardware can skip Unity chapter.

### Q5: How to distribute large Unity projects?
**Answer**: Unity project `.zip` in `frontend_book/static/examples/` (estimated 20-50 MB with minimal assets). GitHub LFS not needed for <100 MB files.

---

## Next Steps (Phase 1)

With research complete, Phase 1 will deliver:

1. **data-model.md**: Define key entities (Gazebo World File, URDF with Plugins, Unity Scene, Simulated Sensor, ROS-Unity Bridge)
2. **contracts/**: N/A for this module (educational content, no API contracts)
3. **quickstart.md**: Step-by-step guide for creating Module 2 chapters + example file organization

---

## References

- Gazebo Classic Documentation: http://classic.gazebosim.org/
- Unity-ROS Integration Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- ROS 2 Humble Documentation: https://docs.ros.org/en/humble/
- gazebo_ros_pkgs Documentation: https://github.com/ros-simulation/gazebo_ros_pkgs
