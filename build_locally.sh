#!/bin/sh

# TODO: Expand this script to work beyond ROS Kinetic

# Install dependencies
sudo apt install ros-kinetic-rosdoc-lite

# Setup Environment
rm -rf build

# Build
rosdoc_lite -o build .

# Run
xdg-open ./build/html/index.html
