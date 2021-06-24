Debugging Tests
===============

How to debug when a test is failing.

**Note:** *This is not meant as an exhaustive tutorial on software testing, instead this focuses on methods that will help you debug tests in MoveIt or similar ROS projects.*

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

CI Failures
-----------

Our CI runs in Travis and uses scripts found in the `moveit_ci repo <https://github.com/ros-planning/moveit_ci.git>`.  These tests build and run various tests in various environments.  Often something that works locally won't work in CI in a different environment.  To troubleshoot a failure from CI it is useful to use docker to run in the same environment.

For troubleshooting a specific travis test it is helpful to look at the .travis.yml config file and test output to understand what environment variables are being set in your test.

Launch Docker Environment
-------------------------

To start docker container for kinetic:

  docker pull moveit/moveit:kinetic-ci
  docker run -it moveit/moveit:kinetic-ci /bin/bash

To start docker container for melodic:

  docker pull moveit/moveit:melodic-ci
  docker run -it moveit/moveit:melodic-ci /bin/bash

Setup Environment
-----------------

The first thing you should do is update debians and install tools you'll need for debugging.  The update is important because that is what we do in CI.

  apt-get update
  apt-get dist-upgrade
  apt-get install -y python-catkin-tools ssh git gdb valgrind vim

Next create the folder structure for your ROS environment:

  CATKIN_WS=/root/ros_ws
  mkdir -p ${CATKIN_WS}/src
  cd ${CATKIN_WS}/src

Checkout Code
-------------

In this step we use git and wstool to get the code we will be testing.  You'll replace the remote and branch with yours if you are debugging a PR.  Note that on the wstool update step we'll need to skip replacing moveit if we are testing a specific branch from a PR.

  cd ${CATKIN_WS}/src
  wstool init .
  git clone https://github.com/ros-planning/moveit.git -b master
  wstool merge -t . moveit/moveit.rosinstall
  wstool update

Install Dependencies
--------------------

Here we install debian dependencies with rosdep:

  cd ${CATKIN_WS}/src
  rosdep install -y -q -n --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}

Configure and Build
-------------------

Now we configure and build our workspace.  Note that we set extra CXXFLAGS to be the same as the ones used by moveit_ci.

  cd $CATKIN_WS
  export CXXFLAGS="-Wall -Wextra -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls"
  catkin config --extend /opt/ros/${ROS_DISTRO} --no-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
  catkin build --summarize

Build the Tests
---------------

Here is the command to build the tests in the workspace:

  cd ${CATKIN_WS}
  catkin build --summarize --make-args tests

Run the Tests
-------------

To run all the tests we can use the run_tests cmake arg.  Here we should specify a specific package we want to test as that will speed up this run.

  catkin build --summarize --catkin-make-args run_tests -- moveit_ros_planning_interface

Run One Test
------------

You can also use rostest to run a specific test.  The text argument sends output to the console instead of an xml output file.  To do this you'll have to source the devel workspace.

  cd ${CATKIN_WS}
  source devel/setup.bash
  rostest moveit_ros_planning_interface move_group_pick_place_test.test --text
