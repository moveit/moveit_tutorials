#!/bin/sh

# Install rosdoc_lite if it isn't there yet
if ! command -v rosdoc_lite &> /dev/null
then
  echo "Installing rosdoc_lite"
  sudo apt install ros-$ROS_DISTRO-rosdoc-lite
fi

# Setup Environment
rm -rf build

# Build
rosdoc_lite -o build .

# Run
xdg-open ./build/html/index.html
