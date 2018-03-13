Prerequisites
==============

Install ROS and MoveIt!
^^^^^^^^^^^^^^^^^^^^^^^

Install ROS `Indigo <http://wiki.ros.org/indigo/Installation/Ubuntu>`_, `Kinetic <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_, or `Lunar <http://wiki.ros.org/lunar/Installation/Ubuntu>`_. (We recommend Kinetic) Please make sure you have followed all steps and have the latest versions of packages installed::

  rosdep update 
  sudo apt-get update
  sudo apt-get dist-upgrade

Install MoveIt! ::

  sudo apt-install ros-<YOUR DISTRO>-moveit

Advanced users might want to `install MoveIt! from source <http://moveit.ros.org/install/source/>`_.

Create A Catkin Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^

You will need to have a `Catkin <wiki.ros.org/catkin>`_ workspace setup::

  mkdir -p ~/ws_moveit/src
  cd ~/ws_moveit/src

If you do not have a workspace already setup, follow the "Prerequisites" section on the  `MoveIt! source install page <http://moveit.ros.org/install/source/>`_ and be sure to then source the workspace as documented at the bottom of that page under "Source the Catkin Workspace."

Download and Build the Example Code
^^^^^^^^^^^^^^^^^^^^^^^^^

Within your `Catkin <wiki.ros.org/catkin>`_ workspace, download these tutorials::
  
  cd ~/ws_moveit/src
  git clone https://github.com/ros-planning/moveit_tutorials.git
  rosdep install -y --from-paths . --ignore-src --rosdistro lunar
  cd ..
  catkin config --extend /opt/ros/lunar --cmake-args -DCMAKE_BUILD_TYPE=Release
  catkin build

Source the Catkin Workspace:: 

  source ~/ws_moveit/devel/setup.bash

To source your Catkin workspace every time you open a new terminal, add this last command to your .bashrc ::

  echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc

Panda on Kinetic Instructions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You will also need a **ROBOT_moveit_config** package to follow along with these tutorials. These tutorials will use the Panda robot from Franka Emika. To follow along there are a few ways to get a working **panda_movit_config** package. You can install from source, create your own, or install from pre-built binaries (Debian).

To install from Debian (available for Kinetic on Ubuntu 16.04)::

  sudo apt install ros-kinetic-panda-moveit-config

To download and install from source::

  cd ~/ws_moveit/src
  git clone https://github.com/PickNikRobotics/panda_moveit_config

To create your own, follow the instructions in the `MoveIt! Setup Assistant tutorial.
<../setup_assistant/setup_assistant_tutorial.html>`_

Install Dependencies and Build
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Scans your `Catkin <wiki.ros.org/catkin>`_ workspace for missing packages before compiling new code::

  cd ~/ws_moveit/src
  rosdep install --from-paths . --ignore-src --rosdistro kinetic
  cd ~/ws_moveit
  catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release
  catkin build

Once you have a local copy of Panda, make sure you re-sourced the setup files::

  source ~/ws_moveit/devel/setup.bash
