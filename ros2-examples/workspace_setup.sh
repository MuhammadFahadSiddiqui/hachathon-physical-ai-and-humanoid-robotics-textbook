#!/bin/bash
# ROS 2 Workspace Setup Helper Script
# This script helps initialize a colcon workspace for Module 1 ROS 2 examples

set -e

echo "========================================="
echo "ROS 2 Workspace Setup for Module 1"
echo "========================================="

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS 2 not sourced. Please run:"
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✅ ROS 2 distribution detected: $ROS_DISTRO"

# Check if we're in the ros2-examples directory
if [ ! -d "src" ]; then
    echo "❌ Please run this script from the ros2-examples/ directory"
    exit 1
fi

echo "✅ Running from ros2-examples/ directory"

# Build workspace
echo ""
echo "Building ROS 2 packages with colcon..."
colcon build --symlink-install

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "✅ Workspace built successfully!"
    echo "========================================="
    echo ""
    echo "To use the workspace, run:"
    echo "   source install/setup.bash"
    echo ""
    echo "Available packages:"
    ls -1 src/ | grep -v "^$"
    echo ""
    echo "Run examples with:"
    echo "   ros2 run <package_name> <node_name>"
    echo ""
else
    echo "❌ Build failed. Check error messages above."
    exit 1
fi
