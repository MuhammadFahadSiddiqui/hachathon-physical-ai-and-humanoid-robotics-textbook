# Quickstart: Module 1 - The Robotic Nervous System (ROS 2)

**Feature**: 001-ros2-robotic-nervous-system
**Date**: 2025-12-17
**Phase**: Phase 1 - Setup and Validation Guide

## Overview

This quickstart guide walks through setting up the development environment for Module 1, including Docusaurus installation, ROS 2 workspace creation, and validation of all code examples. Target completion time: 30-45 minutes.

---

## Prerequisites

Before starting, ensure you have:

### System Requirements
- **Operating System**: Ubuntu 22.04 Jammy Jellyfish (or WSL2 with Ubuntu 22.04)
- **Disk Space**: Minimum 10 GB free (5 GB for ROS 2, 2 GB for Node.js/Docusaurus, 3 GB for examples/build artifacts)
- **RAM**: Minimum 4 GB (8 GB recommended for running RViz + Gazebo)
- **Network**: Broadband internet for package downloads (approx. 2 GB total)

### Software Prerequisites
- **Git**: Version 2.25+ (`sudo apt install git`)
- **Python**: 3.10+ (included in Ubuntu 22.04)
- **Node.js**: 18.x LTS (`curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash - && sudo apt install -y nodejs`)

---

## Part 1: Docusaurus Setup (Book Website)

### Step 1.1: Clone Repository

```bash
# Clone the project repository
git clone https://github.com/<your-org>/hachathon-physical-ai-and-humanoid-robotics-textbook.git
cd hachathon-physical-ai-and-humanoid-robotics-textbook

# Switch to Module 1 feature branch
git checkout 001-ros2-robotic-nervous-system
```

### Step 1.2: Initialize Docusaurus

```bash
# Navigate to project root (if not already there)
cd hachathon-physical-ai-and-humanoid-robotics-textbook

# Initialize Docusaurus 3.x site
npx create-docusaurus@latest docusaurus-book classic --typescript

# Move into Docusaurus directory
cd docusaurus-book

# Install additional dependencies
npm install --save @docusaurus/theme-mermaid remark-math rehype-katex
```

**Expected Output**:
```
Success! Created docusaurus-book at .../docusaurus-book
Inside that directory, you can run several commands:
  npm start - Starts the development server.
  npm run build - Bundles your website into static files for production.
```

### Step 1.3: Configure Docusaurus

Edit `docusaurus-book/docusaurus.config.js`:

```javascript
// @ts-check
const {themes} = require('prism-react-renderer');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Learn ROS 2, Python Agents, and URDF Modeling',
  favicon: 'img/favicon.ico',

  url: 'https://<your-github-username>.github.io',
  baseUrl: '/hachathon-physical-ai-and-humanoid-robotics-textbook/',

  organizationName: '<your-github-username>',
  projectName: 'hachathon-physical-ai-and-humanoid-robotics-textbook',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          editUrl: 'https://github.com/<your-org>/hachathon-physical-ai-and-humanoid-robotics-textbook/tree/main/docusaurus-book/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title: 'Physical AI Textbook',
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Modules',
          },
          {
            href: 'https://github.com/<your-org>/hachathon-physical-ai-and-humanoid-robotics-textbook',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        copyright: `Built with Docusaurus. Open-source under MIT License.`,
      },
      prism: {
        theme: themes.github,
        darkTheme: themes.dracula,
        additionalLanguages: ['python', 'xml', 'bash'],
      },
    }),

  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],
};

module.exports = config;
```

### Step 1.4: Create Module 1 Chapter Files

```bash
# Create Module 1 directory
mkdir -p docs/module-1

# Create chapter files (placeholders for now)
cat > docs/module-1/chapter-1-ros2-fundamentals.md << 'EOF'
---
sidebar_position: 1
---

# Chapter 1: ROS 2 Fundamentals

Content will be added during implementation phase.

## Topics Covered
- Nodes, Topics, and Services
- ROS 2 Architecture Overview
- Example Python Agents
EOF

cat > docs/module-1/chapter-2-python-agents-controllers.md << 'EOF'
---
sidebar_position: 2
---

# Chapter 2: Python Agents & ROS 2 Controllers

Content will be added during implementation phase.
EOF

cat > docs/module-1/chapter-3-humanoid-modeling-urdf.md << 'EOF'
---
sidebar_position: 3
---

# Chapter 3: Humanoid Robot Modeling

Content will be added during implementation phase.
EOF
```

### Step 1.5: Configure Sidebar Navigation

Edit `docusaurus-book/sidebars.js`:

```javascript
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/chapter-1-ros2-fundamentals',
        'module-1/chapter-2-python-agents-controllers',
        'module-1/chapter-3-humanoid-modeling-urdf',
      ],
    },
  ],
};

module.exports = sidebars;
```

### Step 1.6: Start Development Server

```bash
# Start Docusaurus dev server
npm start
```

**Expected Output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

Open `http://localhost:3000/` in your browser. You should see the Docusaurus homepage with "Module 1: The Robotic Nervous System (ROS 2)" in the sidebar.

---

## Part 2: ROS 2 Environment Setup

### Step 2.1: Install ROS 2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop (includes RViz, Gazebo, tutorials)
sudo apt update
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y
```

**Installation Time**: 10-15 minutes (depending on network speed)

### Step 2.2: Source ROS 2 Environment

```bash
# Source ROS 2 setup (add to ~/.bashrc for persistence)
source /opt/ros/humble/setup.bash

# Verify installation
ros2 --version
# Expected output: ros2 cli version 0.18.x

# Test with demo nodes
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener

# You should see:
# [INFO] [1234567890.123456789] [talker]: Publishing: 'Hello World: 0'
# [INFO] [1234567890.234567890] [listener]: I heard: [Hello World: 0]

# Kill demo nodes
killall talker
```

### Step 2.3: Install Additional ROS 2 Tools

```bash
# Install URDF tools
sudo apt install liburdfdom-tools -y

# Install joint_state_publisher_gui (for Chapter 3)
sudo apt install ros-humble-joint-state-publisher-gui -y

# Install rqt tools (for debugging)
sudo apt install ros-humble-rqt* -y

# Verify check_urdf installation
check_urdf --help
# Expected output: Usage: check_urdf <urdf file>
```

---

## Part 3: ROS 2 Workspace Setup (Code Examples)

### Step 3.1: Create Colcon Workspace

```bash
# Navigate back to project root
cd ~/hachathon-physical-ai-and-humanoid-robotics-textbook

# Create ROS 2 workspace
mkdir -p ros2-examples/src
cd ros2-examples

# Initialize workspace (creates build/, install/, log/ directories on first build)
```

### Step 3.2: Create Minimal Publisher Package (Chapter 1 Example)

```bash
cd src

# Create Python package
ros2 pkg create --build-type ament_python minimal_publisher --dependencies rclpy std_msgs

# Navigate into package
cd minimal_publisher/minimal_publisher

# Create publisher node (placeholder)
cat > publisher_node.py << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

### Step 3.3: Configure Package Entry Point

Edit `ros2-examples/src/minimal_publisher/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'minimal_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Minimal ROS 2 publisher example',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = minimal_publisher.publisher_node:main',
        ],
    },
)
```

### Step 3.4: Build Workspace

```bash
# Navigate to workspace root
cd ~/hachathon-physical-ai-and-humanoid-robotics-textbook/ros2-examples

# Build packages
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

**Expected Output**:
```
Starting >>> minimal_publisher
Finished <<< minimal_publisher [1.23s]

Summary: 1 package finished [1.45s]
```

### Step 3.5: Test Minimal Publisher

```bash
# Run publisher node
ros2 run minimal_publisher publisher_node
```

**Expected Output**:
```
[INFO] [1234567890.123456789] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1234567890.623456789] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1234567891.123456789] [minimal_publisher]: Publishing: "Hello World: 2"
```

Press `Ctrl+C` to stop.

### Step 3.6: Verify with ros2 topic echo

```bash
# In terminal 1, run publisher
ros2 run minimal_publisher publisher_node

# In terminal 2, echo topic
ros2 topic echo /chatter
```

**Expected Output (Terminal 2)**:
```
data: Hello World: 5
---
data: Hello World: 6
---
```

**Success!** Chapter 1 minimal publisher example is working.

---

## Part 4: Validation Checklist

### Docusaurus Validation
- [ ] `npm start` launches dev server at `http://localhost:3000/`
- [ ] Module 1 chapters appear in sidebar navigation
- [ ] Clicking chapter links loads placeholder content
- [ ] Code block syntax highlighting works (test with a Python snippet)

### ROS 2 Environment Validation
- [ ] `ros2 --version` shows 0.18.x (Humble)
- [ ] `check_urdf --help` displays usage instructions
- [ ] `ros2 run demo_nodes_cpp talker` publishes messages
- [ ] `ros2 topic list` shows active topics

### ROS 2 Workspace Validation
- [ ] `colcon build` completes without errors
- [ ] `ros2 run minimal_publisher publisher_node` publishes to /chatter
- [ ] `ros2 topic echo /chatter` receives messages
- [ ] `ros2 node list` shows `/minimal_publisher` node

---

## Part 5: Next Steps

With environment setup complete, you're ready for:

1. **Implementation Phase** (`/sp.tasks`):
   - Write full chapter content (Chapter 1, 2, 3 Markdown files)
   - Create remaining code examples (subscriber, service, agent, URDF)
   - Generate diagrams (Mermaid + static images)
   - Write troubleshooting sections

2. **Build and Deploy**:
   - `npm run build` to create production Docusaurus build
   - Deploy to GitHub Pages via GitHub Actions
   - Validate on live URL

3. **Testing**:
   - Manual review of all chapters (constitution Principle III compliance)
   - Run all ROS 2 examples in clean environment
   - Validate URDF files with check_urdf
   - Accessibility audit (alt-text for images, keyboard navigation)

---

## Troubleshooting

### Issue: `npm start` fails with "Cannot find module"
**Solution**: Delete `node_modules` and `package-lock.json`, then re-run `npm install`.

### Issue: `ros2` command not found
**Solution**: Ensure you sourced ROS 2 setup: `source /opt/ros/humble/setup.bash`. Add to `~/.bashrc` for persistence.

### Issue: `colcon build` fails with "Package not found"
**Solution**: Verify `package.xml` and `setup.py` exist in package directory. Check package name consistency.

### Issue: RViz crashes on startup (WSL2)
**Solution**: Install Mesa OpenGL drivers: `sudo apt install mesa-utils libgl1-mesa-glx`. Enable X11 forwarding in WSL2.

### Issue: `check_urdf` not found
**Solution**: Install liburdfdom-tools: `sudo apt install liburdfdom-tools`.

---

## Estimated Setup Time

| Task | Duration |
|------|----------|
| Part 1: Docusaurus Setup | 10 minutes |
| Part 2: ROS 2 Installation | 15 minutes |
| Part 3: Workspace Setup | 10 minutes |
| Part 4: Validation | 5 minutes |
| **Total** | **40 minutes** |

---

## Reference Links

- **Docusaurus Documentation**: https://docusaurus.io/docs
- **ROS 2 Humble Installation**: https://docs.ros.org/en/humble/Installation.html
- **colcon Tutorial**: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
- **rclpy API Reference**: https://docs.ros2.org/humble/api/rclpy/

---

**Status**: Environment ready for `/sp.tasks` phase. Proceed with chapter content authoring and code example implementation.
