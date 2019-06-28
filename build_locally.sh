#!/bin/sh

# Install rosdoc_lite if it isn't there yet
test -x `which rosdoc_lite` || sudo apt install ros-$ROS_DISTRO-rosdoc-lite

# Setup Environment
rm -rf build

# Build
rosdoc_lite -o build .

# Run
xdg-open ./build/html/index.html
