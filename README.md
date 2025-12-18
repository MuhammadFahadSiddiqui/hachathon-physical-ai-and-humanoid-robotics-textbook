# Physical AI & Humanoid Robotics Textbook

**An interactive technical textbook teaching ROS 2, Python agents, and URDF humanoid modeling through hands-on examples.**

![License](https://img.shields.io/badge/license-Apache%202.0-blue)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)
![Python](https://img.shields.io/badge/python-3.10%2B-blue)
![Docusaurus](https://img.shields.io/badge/docusaurus-3.9.2-green)

---

## Overview

This repository contains a comprehensive educational resource for learning Physical AI and Humanoid Robotics through ROS 2 (Robot Operating System 2). The textbook is built with Docusaurus and includes complete, runnable code examples for hands-on learning.

### Module 1: The Robotic Nervous System (ROS 2)

**Chapters:**
1. **ROS 2 Fundamentals** - Nodes, topics, services, and asynchronous/synchronous communication
2. **Python Agents & Controllers** - Building intelligent control loops with sensor-agent-actuator pipelines
3. **Humanoid Robot Modeling with URDF** - Modeling robots with kinematic chains, joints, and visual representations

**Learning Outcomes:**
- Build and run ROS 2 nodes in Python
- Implement publisher-subscriber and client-server patterns
- Design intelligent agents that integrate perception and action
- Create URDF models for humanoid robots
- Visualize robots in RViz with joint control

---

## Quick Start

### For Readers (Students)

#### 1. View the Textbook Online
Visit the deployed documentation site (once GitHub Pages is enabled):
```
https://governor-sindh-it-initiative.github.io/hachathon-physical-ai-and-humanoid-robotics-textbook/
```

#### 2. Download Code Examples
Navigate to any chapter and download the zip files:
- [📦 minimal_publisher.zip](/static/examples/minimal_publisher.zip)
- [📦 minimal_subscriber.zip](/static/examples/minimal_subscriber.zip)
- [📦 service_example.zip](/static/examples/service_example.zip)
- [📦 obstacle_avoidance_agent.zip](/static/examples/obstacle_avoidance_agent.zip)
- [📦 simple_humanoid_urdf.zip](/static/examples/simple_humanoid_urdf.zip)

#### 3. Set Up ROS 2 Environment
```bash
# Ubuntu 22.04 LTS required
# Install ROS 2 Humble
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo apt update
sudo apt install ros-humble-desktop

# Source ROS 2
source /opt/ros/humble/setup.bash
```

#### 4. Build and Run Examples
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Extract downloaded packages here
unzip ~/Downloads/minimal_publisher.zip

# Build
cd ~/ros2_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Run example
ros2 run minimal_publisher publisher_node
```

### For Developers

#### Local Development

```bash
# Clone repository
git clone https://github.com/governor-sindh-it-initiative/hachathon-physical-ai-and-humanoid-robotics-textbook.git
cd hachathon-physical-ai-and-humanoid-robotics-textbook

# Install Docusaurus dependencies
cd frontend_book
npm install

# Start development server
npm start

# Build for production
npm run build
```

#### GitHub Pages Deployment

Push to `main` branch - GitHub Actions will automatically build and deploy:

```bash
git add .
git commit -m "Update documentation"
git push origin main
```

---

## Repository Structure

```
hachathon-physical-ai-and-humanoid-robotics-textbook/
│
├── frontend_book/                 # Docusaurus documentation site
│   ├── docs/
│   │   ├── intro.md               # Homepage
│   │   ├── setup-guide.md         # ROS 2 installation
│   │   └── module-1/              # Module 1 chapters
│   ├── static/examples/           # Downloadable code packages
│   └── package.json
│
├── ros2-examples/                 # ROS 2 code examples
│   ├── src/
│   │   ├── minimal_publisher/
│   │   ├── minimal_subscriber/
│   │   ├── service_example/
│   │   └── obstacle_avoidance_agent/
│   └── module_1_urdf/             # URDF examples
│
├── specs/                         # Project specifications
│   └── 001-ros2-robotic-nervous-system/
│
├── .github/workflows/             # CI/CD workflows
├── IMPLEMENTATION_SUMMARY.md      # Detailed implementation report
└── README.md                      # This file
```

---

## Features

### Educational
- ✅ **Progressive Learning**: 3 chapters building from basics to advanced concepts
- ✅ **Hands-On Code**: 4 complete ROS 2 packages (1,500+ lines)
- ✅ **Visual Diagrams**: 8+ Mermaid diagrams explaining architecture and workflows
- ✅ **Troubleshooting**: 12+ common error scenarios with solutions
- ✅ **Practice Exercises**: 12+ exercises for self-directed learning
- ✅ **Best Practices**: 15+ best practices for ROS 2, Python, and URDF

### Technical
- ✅ **Production Code**: Extensively commented, PEP 8 compliant Python
- ✅ **Downloadable Examples**: All code available as ready-to-use zip files
- ✅ **Navigation**: Bidirectional chapter links and table of contents
- ✅ **Responsive Design**: Works on desktop and mobile
- ✅ **Dark Mode**: Auto-detection of user preference
- ✅ **Search**: Built-in search functionality

---

## Technology Stack

**Frontend:**
- Docusaurus 3.9.2 (TypeScript)
- Mermaid.js (diagrams)
- KaTeX (math equations)
- Prism (syntax highlighting)

**ROS 2:**
- ROS 2 Humble Hawksbill (LTS)
- Python 3.10+
- rclpy (Python client library)
- colcon (build system)

**Deployment:**
- GitHub Actions (CI/CD)
- GitHub Pages (hosting)

---

## Prerequisites

### For Reading Documentation
- Modern web browser
- Internet connection

### For Running Code Examples
- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **Python**: 3.10+
- **Tools**: colcon, robot_state_publisher, joint_state_publisher_gui, rviz2

See `docs/setup-guide.md` for detailed installation instructions.

---

## Content Overview

### Chapter 1: ROS 2 Fundamentals (1,300+ lines)
**Topics:**
- ROS 2 Nodes and communication patterns
- Publisher-Subscriber (Topics)
- Client-Server (Services)
- ROS 2 Architecture Overview

**Code Examples:**
- minimal_publisher: String message publisher
- minimal_subscriber: String message subscriber
- service_example: Integer addition service

**Learning Time**: ~60 minutes

---

### Chapter 2: Python Agents & Controllers (910+ lines)
**Topics:**
- Agent vs Controller distinction
- Sensor-Agent-Actuator pipeline
- Timer-based decision loops
- QoS policies and error handling

**Code Example:**
- obstacle_avoidance_agent: Complete agent with mock sensors
  - Subscribes to `/scan` (LaserScan)
  - Publishes to `/cmd_vel` (Twist)
  - Implements obstacle detection and avoidance logic

**Learning Time**: ~45 minutes

---

### Chapter 3: Humanoid Robot Modeling with URDF (1,530+ lines)
**Topics:**
- URDF syntax and structure
- Links, joints, and kinematic trees
- Visual, collision, and inertial properties
- RViz visualization
- Joint control

**Code Example:**
- simple_humanoid.urdf: 5-link humanoid robot
  - Torso (base_link)
  - 2 arms with shoulder and elbow joints
  - Launch file for RViz visualization
  - Programmatic joint controller

**Learning Time**: ~75 minutes

---

## Contributing

This repository is part of the Governor Sindh IT Initiative Quarter 4 Hackathon. Contributions are welcome!

**How to Contribute:**
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/your-feature`)
3. Commit your changes (`git commit -m 'Add some feature'`)
4. Push to the branch (`git push origin feature/your-feature`)
5. Open a Pull Request

**Guidelines:**
- Follow existing code style (PEP 8 for Python)
- Add comments and docstrings to all code
- Test all ROS 2 examples before submitting
- Update documentation as needed

---

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

---

## Support

**Issues**: Report bugs and request features in [GitHub Issues](https://github.com/governor-sindh-it-initiative/hachathon-physical-ai-and-humanoid-robotics-textbook/issues)

**Documentation**: Visit the [online textbook](https://governor-sindh-it-initiative.github.io/hachathon-physical-ai-and-humanoid-robotics-textbook/)

**ROS 2 Help**: See the [ROS 2 Documentation](https://docs.ros.org/en/humble/)

---

## Project Stats

- **Total Lines of Documentation**: 3,800+
- **Total Lines of Code**: 1,500+
- **ROS 2 Packages**: 4 complete packages
- **Downloadable Examples**: 5 zip files
- **Diagrams**: 8+ Mermaid visualizations
- **Implementation Tasks**: 78/80 completed (97.5%)

---

## Acknowledgments

- **Governor Sindh IT Initiative** - Project sponsor and hackathon organizer
- **ROS 2 Community** - Technical foundation and inspiration
- **Docusaurus Team** - Excellent documentation framework
- **Open Robotics** - ROS 2 development and maintenance

---

## Related Projects

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Gazebo Simulation](https://gazebosim.org/)
- [MoveIt 2 Motion Planning](https://moveit.ros.org/)
- [Navigation 2](https://navigation.ros.org/)

---

**Project Status**: ✅ **COMPLETE** - Ready for deployment

**Last Updated**: December 19, 2025

**Maintained by**: Governor Sindh IT Initiative

---

**Happy Learning! 🤖**

For questions or feedback, please open an issue or reach out through the Governor Sindh IT Initiative channels.
