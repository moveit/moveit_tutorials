#!/bin/bash
set -e

# Define some config vars
export NOKOGIRI_USE_SYSTEM_LIBRARIES=true
export REPOSITORY_NAME=${PWD##*/}
echo "Testing branch ${GITHUB_BASE_REF:-$GITHUB_HEAD_REF} of $REPOSITORY_NAME"

# Install htmlpoofer
sudo gem update --system --no-document
gem --version
sudo gem install html-proofer -v 3.19 # newer 4.x requires different cmdline options
# Install ROS's version of sphinx
sudo apt-get -qq install "ros-$ROS_DISTRO-rosdoc-lite"
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Test build with non-ROS wrapped Sphinx command to allow warnings and errors to be caught
sphinx-build -W -b html . native_build
# Test build with ROS-version of Sphinx command so that it is generated same as ros.org
rosdoc_lite -o build .

# Run HTML tests on generated build output to check for 404 errors, etc
URL_SWAP="--url-swap https\://github.com/ros-planning/moveit_tutorials/blob/master/:file\://$PWD/build/html/"
htmlproofer ./build --only-4xx --check-html --file-ignore ./build/html/genindex.html,./build/html/search.html --alt-ignore '/.*/' --url-ignore '#' $URL_SWAP

# Tell GitHub Pages (on deploy) to bypass Jekyll processing
touch build/html/.nojekyll
