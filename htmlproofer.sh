#!/bin/bash
set -e

# Define some config vars
export NOKOGIRI_USE_SYSTEM_LIBRARIES=true
export REPOSITORY_NAME=${PWD##*/}
echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"
sudo -E sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep python-wstool python-catkin-tools
# Setup rosdep
sudo rosdep init
rosdep update
# Install htmlpoofer
gem update --system
gem --version
gem install html-proofer
# Install ROS's version of sphinx
sudo apt-get -qq install ros-kinetic-rosdoc-lite
source /opt/ros/kinetic/setup.bash

# Test build with non-ROS wrapped Sphinx command to allow warnings and errors to be caught
sphinx-build -W -b html . native_build
# Test build with ROS-version of Sphinx command so that it is generated same as ros.org
rosdoc_lite -o build .
# Run HTML tests on generated build output to check for 404 errors, etc
test "$TRAVIS_PULL_REQUEST" == false || URL_SWAP="--url-swap https\://github.com/ros-planning/moveit_tutorials/blob/master/:file\://$PWD/build/html/"
htmlproofer ./build --only-4xx --check-html --file-ignore ./build/html/genindex.html,./build/html/search.html --alt-ignore '/.*/' --url-ignore '#' $URL_SWAP

# Tell GitHub Pages (on deploy) to bypass Jekyll processing
touch build/html/.nojekyll
