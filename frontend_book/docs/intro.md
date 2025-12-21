---
sidebar_position: 1
---

# Welcome to Physical AI & Humanoid Robotics

Welcome to the comprehensive textbook on **Physical AI & Humanoid Robotics**. This learning resource is designed to guide students and developers through the fundamentals of building intelligent robotic systems, from middleware architecture to physical robot modeling.

## What You'll Learn

This textbook covers the complete robotics development pipeline:

1. **ROS 2 Fundamentals** - Master the Robot Operating System 2, the industry-standard middleware for robotic applications
2. **Python Agents & Controllers** - Build intelligent control systems that bridge high-level decision-making with low-level hardware
3. **URDF Modeling** - Model humanoid robots using the Unified Robot Description Format for simulation and real-world deployment
4. **Digital Twin Simulation** - Test robots in Gazebo physics engine and Unity rendering before hardware deployment
5. **Sensor Simulation** - Configure realistic LiDAR, depth cameras, and IMU sensors with accurate noise models

## Course Structure

### Module 1: The Robotic Nervous System (ROS 2)

Module 1 introduces ROS 2 as the "nervous system" of robotic applications. You'll learn how ROS 2 enables communication between different components of a robot through a publish-subscribe architecture.

**Chapters:**

- **[Chapter 1: ROS 2 Fundamentals](./module-1/chapter-1-ros2-fundamentals.md)** - Nodes, topics, services, and the ROS 2 graph
- **[Chapter 2: Python Agents & Controllers](./module-1/chapter-2-python-agents-controllers.md)** - Building intelligent control loops with rclpy
- **[Chapter 3: Humanoid Robot Modeling with URDF](./module-1/chapter-3-humanoid-modeling-urdf.md)** - Kinematic chains, joints, and visual models

### Module 2: The Digital Twin (Gazebo & Unity)

Module 2 teaches you to simulate humanoid robots in realistic physics environments before deploying to hardware. You'll master Gazebo for accurate physics, Unity for photorealistic rendering, and sensor simulation for SLAM and perception algorithms.

**Chapters:**

- **[Chapter 1: Gazebo Physics](./module-2/chapter-1-gazebo-physics.md)** - Gravity, collision detection, joint dynamics, and ROS 2 integration
- **[Chapter 2: Unity Rendering](./module-2/chapter-2-unity-rendering.md)** - Photorealistic visualization, materials, lighting, and ROS-TCP bridge
- **[Chapter 3: Sensor Simulation](./module-2/chapter-3-sensor-simulation.md)** - LiDAR, depth cameras, IMU with realistic noise models

**Downloads:** [gazebo_humanoid_world.zip](/examples/gazebo_humanoid_world.zip) | [unity_humanoid_scene.zip](/examples/unity_humanoid_scene.zip) | [gazebo_sensors.zip](/examples/gazebo_sensors.zip)

## Prerequisites

Before starting this textbook, you should have:

- **Python 3.10+** - Basic programming knowledge in Python
- **Linux Environment** - Ubuntu 22.04 LTS recommended (or Windows with WSL2)
- **ROS 2 Humble** - Follow our [Setup Guide](./setup-guide.md) to install ROS 2

## How to Use This Textbook

Each chapter follows a consistent structure:

1. **Conceptual Overview** - Theory and architecture explained in plain language
2. **Practical Examples** - Runnable Python code with line-by-line comments
3. **Hands-On Exercises** - Build working examples in your own ROS 2 workspace
4. **Visual Aids** - Diagrams illustrating system architecture and data flow
5. **Troubleshooting** - Common errors and how to resolve them

## Getting Started

Ready to begin? Follow this learning path:

1. **[Setup Guide](./setup-guide.md)** - Install ROS 2 Humble and required tools
2. **[Module 1: ROS 2 Fundamentals](./module-1/chapter-1-ros2-fundamentals.md)** - Learn ROS 2 basics, Python agents, and URDF modeling
3. **[Module 2: The Digital Twin](./module-2/index.md)** - Master Gazebo physics, Unity rendering, and sensor simulation

All code examples and downloadable .zip files are included in each chapter.

## About This Project

This textbook is part of the **Governor Sindh IT Initiative** hackathon project on Physical AI and Humanoid Robotics. It follows a spec-driven development approach, ensuring all content is accurate, reproducible, and aligned with industry best practices.

**Technologies Used:**
- **Docusaurus 3.x** - Static site generation with React
- **ROS 2 Humble** - Long-term support release of Robot Operating System 2
- **Python 3.10+** - Primary programming language for ROS 2 examples
- **Gazebo Classic 11** - Physics simulation engine with ODE/Bullet/Simbody support
- **Unity 2021 LTS** - Photorealistic rendering engine with ROS-TCP bridge
- **Mermaid.js** - Diagram rendering for architecture visualizations

---

**Let's build intelligent robots together!**
