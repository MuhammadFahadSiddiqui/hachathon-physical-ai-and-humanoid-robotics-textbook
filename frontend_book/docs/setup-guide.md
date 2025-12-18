---
sidebar_position: 2
---

# Setup Guide: Installing ROS 2 Humble

This guide walks you through installing **ROS 2 Humble Hawksbill**, the long-term support (LTS) release of the Robot Operating System 2. ROS 2 Humble is supported until May 2027, making it the ideal choice for learning and production robotics applications.

## System Requirements

Before installing ROS 2 Humble, ensure your system meets these requirements:

- **Operating System**: Ubuntu 22.04 Jammy Jellyfish (recommended)
  - Alternative: Windows 10/11 with WSL2 (Ubuntu 22.04)
  - Alternative: Docker container with Ubuntu 22.04 base image
- **Architecture**: x86_64 (amd64) or ARM64
- **Disk Space**: Minimum 5 GB free space
- **RAM**: Minimum 4 GB (8 GB recommended for simulation)
- **Python**: Python 3.10+ (included with Ubuntu 22.04)

## Installation Methods

You can install ROS 2 Humble using one of two methods:

1. **Debian Packages** (Recommended) - Pre-built binaries for quick setup
2. **Build from Source** - For advanced users who need custom builds

This guide covers the **Debian package installation**, which is sufficient for all examples in this textbook.

---

## Step 1: Set Locale

Ensure your system uses a UTF-8 locale:

```bash
locale  # Check current settings

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # Verify settings
```

---

## Step 2: Add ROS 2 APT Repository

Enable the Ubuntu Universe repository:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ROS 2 GPG key:

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the ROS 2 repository to your sources list:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

---

## Step 3: Install ROS 2 Humble

Update your package index:

```bash
sudo apt update
sudo apt upgrade
```

Install ROS 2 Humble Desktop (includes visualization tools like RViz):

```bash
sudo apt install ros-humble-desktop
```

**Alternative installations:**

- **ROS 2 Base** (no GUI tools, lightweight):
  ```bash
  sudo apt install ros-humble-ros-base
  ```

- **Development Tools** (colcon, rosdep, etc.):
  ```bash
  sudo apt install ros-dev-tools
  ```

**Installation time**: 5-15 minutes depending on network speed.

---

## Step 4: Environment Setup

After installation, you must source the ROS 2 setup script to use ROS 2 commands.

### Option A: Source Manually (Each Terminal Session)

```bash
source /opt/ros/humble/setup.bash
```

### Option B: Automatic Sourcing (Recommended)

Add the source command to your `.bashrc` file so it runs automatically:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verification:**

Check that the `ROS_DISTRO` environment variable is set:

```bash
printenv | grep ROS
```

Expected output should include:
```
ROS_DISTRO=humble
ROS_VERSION=2
ROS_PYTHON_VERSION=3
```

---

## Step 5: Install Additional Dependencies

Install `colcon` (ROS 2 build tool) and `rosdep` (dependency management):

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
```

Initialize rosdep (first-time setup only):

```bash
sudo rosdep init
rosdep update
```

---

## Step 6: Verify Installation

Test your ROS 2 installation with the demo nodes.

### Terminal 1: Run a Talker Node

Open a terminal and run:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

You should see output like:
```
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
...
```

### Terminal 2: Run a Listener Node

Open a **second terminal** and run:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

You should see:
```
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
...
```

**Success!** The talker and listener nodes are communicating via ROS 2 topics.

Press `Ctrl+C` in both terminals to stop the nodes.

---

## Step 7: Set Up Your Workspace

Create a ROS 2 workspace for this textbook's examples:

```bash
# Create workspace directory
mkdir -p ~/ros2_textbook_ws/src
cd ~/ros2_textbook_ws

# Clone the textbook examples repository
cd src
git clone https://github.com/governor-sindh-it-initiative/hachathon-physical-ai-and-humanoid-robotics-textbook.git
cd ..

# Build the workspace
colcon build --symlink-install

# Source the workspace overlay
source install/setup.bash
```

**Note**: The `--symlink-install` flag creates symbolic links to Python scripts, allowing you to modify code without rebuilding.

---

## Troubleshooting

### Issue: `ros2: command not found`

**Solution**: Source the ROS 2 setup script:
```bash
source /opt/ros/humble/setup.bash
```

### Issue: GPG key verification failed

**Solution**: Manually download and add the GPG key:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Issue: Permission denied when running `rosdep init`

**Solution**: Use `sudo`:
```bash
sudo rosdep init
rosdep update
```

### Issue: Slow performance in WSL2

**Solution**: Store your workspace in the Linux filesystem (`~/ros2_textbook_ws`), not in the Windows filesystem (`/mnt/c/...`). Linux filesystem access is significantly faster in WSL2.

---

## Next Steps

Now that ROS 2 Humble is installed and verified, you're ready to start learning!

Continue to **[Chapter 1: ROS 2 Fundamentals](./module-1/chapter-1-ros2-fundamentals.md)** to build your first ROS 2 nodes.

---

## Additional Resources

- **Official ROS 2 Documentation**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
- **ROS 2 Tutorials**: [https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)
- **ROS Discourse (Community Forum)**: [https://discourse.ros.org/](https://discourse.ros.org/)
- **ROS 2 GitHub Repository**: [https://github.com/ros2](https://github.com/ros2)

**Need help?** Check the troubleshooting section above or ask questions in the ROS Discourse community.
