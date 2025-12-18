# Research: Module 1 - The Robotic Nervous System (ROS 2)

**Feature**: 001-ros2-robotic-nervous-system
**Date**: 2025-12-17
**Phase**: Phase 0 - Technology Research and Decision Documentation

## Overview

This document captures research findings and technology decisions for implementing Module 1: The Robotic Nervous System (ROS 2) as part of the AI-Driven Book with Embedded RAG Chatbot project. Module 1 focuses on delivering instructional content for ROS 2 middleware, Python agent-controller integration, and URDF-based humanoid robot modeling through a Docusaurus static site.

## Key Research Areas

### 1. Static Site Generator Selection

**Decision**: Docusaurus 3.x

**Rationale**:
- **Constitution Compliance**: Open-source (MIT license), free-tier GitHub Pages hosting ✓
- **Developer-Friendly**: React-based, familiar to modern web developers. Markdown/MDX support for embedding interactive components.
- **Code Highlighting**: Built-in Prism.js integration with excellent Python and XML (URDF) syntax highlighting
- **Versioning**: Native support for documentation versioning (future-proof for Module 2, 3, etc.)
- **Search**: Built-in Algolia DocSearch integration (free for open-source projects)
- **Customization**: Extensible with React components (future RAG chatbot embedding)
- **Performance**: Static site generation ensures fast page loads (<2 seconds target)

**Alternatives Considered**:
- **VuePress**: Vue.js-based, lighter weight, but less ecosystem support for technical documentation
- **MkDocs**: Python-based, simpler, but lacks React extensibility needed for future chatbot embedding
- **GitBook**: Cloud-hosted, easier setup, but violates free-tier constraint (limited to 100 requests/month on free plan)
- **Jekyll**: GitHub Pages default, mature, but Ruby-based (adds complexity for Python/ROS 2 project context)

**Rejection Reasoning**: Docusaurus chosen for React extensibility (future chatbot integration), superior code highlighting, and strong technical documentation ecosystem (used by React, Jest, Babel, etc.).

---

### 2. ROS 2 Distribution Selection

**Decision**: ROS 2 Humble Hawksbill (LTS)

**Rationale**:
- **Long-Term Support**: Humble is the current LTS release (supported until May 2027), ensuring examples remain valid for years
- **Ubuntu Compatibility**: Targets Ubuntu 22.04 Jammy Jellyfish (widely adopted, stable)
- **Python 3.10 Support**: Aligns with spec requirement (FR-009: Python 3.10+)
- **rclpy Stability**: Mature rclpy API with extensive documentation and community examples
- **Tooling Support**: check_urdf, RViz, Gazebo, colcon all fully supported

**Alternatives Considered**:
- **ROS 2 Iron Irwini**: Newer release (May 2023), but shorter support window (Nov 2024). Risk of examples becoming outdated sooner.
- **ROS 2 Jazzy Jalisco**: Latest release (May 2024), but too new for stable learning material. Limited community examples.
- **ROS 2 Foxy Fitzroy**: Previous LTS, but EOL in May 2023. Already deprecated.

**Rejection Reasoning**: Humble selected for maximum example longevity and widest Ubuntu 22.04 adoption.

---

### 3. Diagram and Visualization Tools

**Decision**: Mermaid.js for conceptual diagrams, static images (PNG/SVG) for complex architecture

**Rationale**:
- **Mermaid.js**: Text-based diagram-as-code, renders in-browser, supported natively by Docusaurus 3.x. Ideal for flow charts (ROS 2 message flow, agent decision trees).
- **Static Images**: For ROS 2 architecture diagrams (DDS middleware layers, executor model), hand-crafted diagrams (draw.io, Excalidraw) provide better clarity.
- **Free and Open-Source**: Both approaches free, no licensing constraints.

**Alternatives Considered**:
- **PlantUML**: Powerful, but requires Java runtime (adds setup complexity for readers)
- **GraphViz**: Low-level, steep learning curve for non-technical contributors
- **Lucidchart/Figma**: Cloud-based, violates open-source/free-tier principle for contributors

**Rejection Reasoning**: Mermaid.js chosen for simplicity and Docusaurus native integration. Static images retained for complex diagrams requiring manual layout.

---

### 4. ROS 2 Code Example Best Practices

**Decision**: Minimal working examples (MWE) pattern with inline comments

**Research Findings**:
- **ROS 2 Tutorials**: Official ROS 2 documentation uses MWE approach (single-file nodes, minimal dependencies)
- **rclpy Patterns**: Community consensus: use `rclpy.spin()` for simple examples, `MultiThreadedExecutor` only when needed
- **Package Structure**: Standard ROS 2 package layout (setup.py, package.xml, resource/, module/)
- **QoS Profiles**: Default to `RELIABLE` + `VOLATILE` for tutorials. Explicitly demonstrate `BEST_EFFORT` in sensor examples (Chapter 2).

**Best Practices Applied**:
1. **One Concept Per Example**: Chapter 1 publisher example focuses only on publishing. Subscriber example only on subscribing. Service example only on client-server.
2. **Inline Comments**: Every `rclpy` API call annotated with explanation (e.g., `rclpy.create_node('minimal_publisher')  # Initialize ROS 2 node`)
3. **Colcon Compatibility**: All packages follow standard layout for `colcon build && colcon test`
4. **No External Dependencies**: Examples use only ROS 2 base packages (rclpy, std_msgs, geometry_msgs, sensor_msgs). No third-party libraries.
5. **Error Handling**: Demonstrate graceful shutdown with `try/except KeyboardInterrupt` and `rclpy.shutdown()`

**Alternatives Considered**:
- **Jupyter Notebooks**: Interactive, but require JupyterROSpy kernel (adds complexity). Not standard in ROS 2 community.
- **C++ Examples**: More performant, but Python chosen for accessibility (per spec: Python 3.10+ target audience)

---

### 5. URDF Modeling Approach

**Decision**: Hand-crafted minimal humanoid URDF with STL/DAE meshes for visual geometry

**Rationale**:
- **Learning Focus**: Simple humanoid (torso + 2 arms, 4 joints total) teaches URDF syntax without overwhelming complexity
- **Mesh Formats**: STL for collision geometry (simple, widely supported), DAE (COLLADA) for visual geometry (supports colors, textures)
- **Joint Types**: Demonstrate revolute joints (shoulder, elbow) as most common in humanoid robotics
- **Inertial Properties**: Provide realistic inertia tensors (even if not perfect) to ensure Gazebo compatibility

**URDF Structure**:
```xml
<robot name="simple_humanoid">
  <link name="base_link"/> <!-- Torso -->
  <link name="left_shoulder_link"/>
  <link name="left_elbow_link"/>
  <link name="right_shoulder_link"/>
  <link name="right_elbow_link"/>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder_link"/>
    <axis xyz="0 1 0"/> <!-- Pitch rotation -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>
  <!-- Mirror for right side -->
  <!-- Elbow joints -->
</robot>
```

**Alternatives Considered**:
- **Xacro Macros**: More powerful (parameterized URDF), but adds learning curve. Deferred to advanced modules.
- **Full Humanoid (legs, hands)**: Too complex for first module. Stick to upper body.
- **Procedural Generation**: Python scripts to generate URDF (robot_description_builder) - over-engineering for tutorial purposes.

**Rejection Reasoning**: Hand-crafted minimal URDF chosen for transparency and pedagogical clarity.

---

### 6. ROS 2 Launch File Format

**Decision**: Python launch files (launch.py) over XML

**Rationale**:
- **ROS 2 Standard**: Python launch files are the ROS 2 default (XML deprecated from ROS 1)
- **Flexibility**: Programmatic launch logic (conditional nodes, parameterization)
- **Consistency**: Matches Python code examples in chapters

**Example Launch File**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': '<robot>...</robot>'}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'path/to/config.rviz']
        )
    ])
```

---

### 7. Syntax Highlighting and Code Presentation

**Decision**: Prism.js with Python and XML language packs

**Rationale**:
- **Docusaurus Default**: Prism.js pre-configured in Docusaurus 3.x
- **Language Support**: Python (code examples), XML (URDF), Bash (setup commands)
- **Themes**: Use `prism-tomorrow` theme for code blocks (readable, accessibility-friendly)

**Code Block Configuration**:
```js
// docusaurus.config.js
prism: {
  theme: prismThemes.github,
  darkTheme: prismThemes.dracula,
  additionalLanguages: ['python', 'xml', 'bash'],
}
```

---

### 8. Troubleshooting Section Content

**Decision**: Curate common errors from ROS 2 Discourse and Stack Overflow

**Research Sources**:
- **ROS 2 Discourse**: https://discourse.ros.org/ (official community forum)
- **ROS Answers**: https://answers.ros.org/questions/ (Q&A site)
- **GitHub Issues**: ros2/rclpy, ros2/geometry2, ros-visualization/rviz

**Top 5 Errors for Module 1**:
1. **"ModuleNotFoundError: No module named 'rclpy'"** → Source ROS 2 setup.bash
2. **"[WARN] Publisher/Subscriber QoS mismatch"** → Explain QoS compatibility matrix
3. **"check_urdf: command not found"** → Install liburdfdom-tools (apt package)
4. **"RViz crashes on startup"** → Missing libGL drivers (common in Docker/WSL)
5. **"Service call times out"** → Check server node is running, ros2 service list

---

## Technology Stack Summary

| Component | Technology | Version | License | Rationale |
|-----------|-----------|---------|---------|-----------|
| Static Site Generator | Docusaurus | 3.x | MIT | React-based, extensible, excellent code highlighting |
| ROS 2 Distribution | Humble Hawksbill | Humble | Apache 2.0 | LTS until 2027, Ubuntu 22.04 support |
| Python | CPython | 3.10+ | PSF | ROS 2 Humble requires Python 3.10+ |
| Diagram Tool | Mermaid.js | 10.x | MIT | Text-based, renders in-browser, Docusaurus native |
| URDF Validation | check_urdf | 1.13+ | BSD | Standard ROS 2 tool for URDF syntax validation |
| Visualization | RViz 2 | 11.x (Humble) | BSD | Standard ROS 2 3D visualization tool |
| Build Tool | colcon | 0.12+ | Apache 2.0 | ROS 2 standard build system |
| Syntax Highlighting | Prism.js | 1.29+ | MIT | Docusaurus default, Python + XML support |

---

## Open Questions Resolved

### Q1: Should examples use ros2 run or ros2 launch?
**Answer**: Both. Chapter 1 uses `ros2 run` for simplicity (direct node execution). Chapter 3 uses `ros2 launch` for URDF visualization (requires coordinating robot_state_publisher + RViz + joint_state_publisher_gui).

### Q2: Mock data or real simulator (Gazebo)?
**Answer**: Mock data for Chapter 1-2 (simple publisher for sensor data). Gazebo optional for Chapter 3 (RViz sufficient for URDF visualization, Gazebo recommended for physics validation).

### Q3: How to handle Windows users?
**Answer**: Recommend WSL2 (Windows Subsystem for Linux) with Ubuntu 22.04. Provide troubleshooting section for WSL-specific issues (X11 forwarding for RViz, USB passthrough not needed since simulation-only).

### Q4: Version control for code examples?
**Answer**: All ROS 2 examples committed to main repository in `ros2-examples/` directory. Readers can `git clone` and run directly. Docusaurus site references examples via relative links (e.g., "Download: [minimal_publisher.zip](/examples/minimal_publisher.zip)").

---

## Next Steps (Phase 1)

With research complete, Phase 1 will deliver:
1. **data-model.md**: Define key entities (ROS 2 Node, Topic, Service, URDF Model, etc.) with attributes
2. **contracts/**: N/A for this module (static content, no API contracts)
3. **quickstart.md**: Step-by-step guide for Docusaurus setup + ROS 2 workspace creation

---

## References

- ROS 2 Humble Documentation: https://docs.ros.org/en/humble/
- Docusaurus Documentation: https://docusaurus.io/docs
- URDF Tutorials: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html
- rclpy API Reference: https://docs.ros2.org/humble/api/rclpy/
