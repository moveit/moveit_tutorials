# MoveIt! Tutorials

This repo is automatically built by the ROS build farm and its output is hosted here: http://docs.ros.org/indigo/api/moveit_tutorials/html/

## Travis Continuous Integration

[![Build Status](https://travis-ci.org/ros-planning/moveit_tutorials.svg?branch=master)](https://travis-ci.org/ros-planning/moveit_tutorials)

## ROS Buildfarm

[![Build Status](http://build.ros.org/buildStatus/icon?job=Idoc__moveit_tutorials__ubuntu_trusty_amd64&build=2)](http://build.ros.org/job/Idoc__moveit_tutorials__ubuntu_trusty_amd64/2/)

## Build

If you want to test the tutorials by generating the html pages locally on your machine, install [rosdoc_lite](http://wiki.ros.org/rosdoc_lite):

    sudo apt-get install ros-kinetic-rosdoc-lite

and run in the root of the package:

    rosdoc_lite -o build .

Then open ``LOCAL_PACKAGE_PATH/build/html/index.html`` in your web browser.

## Deployment

For deploying documentation changes to the web, [Section 3 of rosdoc_lite wiki](http://wiki.ros.org/rosdoc_lite) says that "rosdoc_lite is automatically run for packages in repositories that have rosinstall files listed in the rosdistro repository." This is done about once every 24 hours, [overnight](http://wiki.ros.org/rosdistro/Tutorials/Indexing%20Your%20ROS%20Repository%20for%20Documentation%20Generation).
