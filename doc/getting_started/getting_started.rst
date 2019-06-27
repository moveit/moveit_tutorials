Getting Started
===============

Install ROS and Catkin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Install ROS Kinetic <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_.
It is easy to miss steps when going through the ROS installation tutorial. If you run into errors in the next few steps, a good place to start is to go back and make sure you have installed ROS correctly.

Once you have ROS installed, make sure you have the most up to date packages: ::

  rosdep update
  sudo apt-get update
  sudo apt-get dist-upgrade

Install `catkin <http://wiki.ros.org/catkin>`_ the ROS build system: ::

  sudo apt-get install ros-kinetic-catkin python-catkin-tools

Install MoveIt!
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The simplest way to install MoveIt! is from pre-built binaries (Debian): ::

  sudo apt install ros-kinetic-moveit

Advanced users might want to `install MoveIt! from source <http://moveit.ros.org/install/source/>`_.

Create A Catkin Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^
You will need to have a `catkin <http://wiki.ros.org/catkin>`_ workspace setup: ::

  mkdir -p ~/ws_moveit/src

Download the Example Code
^^^^^^^^^^^^^^^^^^^^^^^^^
Within your `catkin <http://wiki.ros.org/catkin>`_ workspace, download these tutorials: ::

  cd ~/ws_moveit/src
  git clone -b kinetic-devel https://github.com/ros-planning/moveit_tutorials.git

You will also need a ``panda_moveit_config`` package to follow along with these tutorials: ::

  git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git

.. note:: For now we will use a pre-generated ``panda_moveit_config`` package but later we will learn how to make our own in the `MoveIt! Setup Assistant tutorial <../setup_assistant/setup_assistant_tutorial.html>`_.

Build your Catkin Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following will attempt to install from Debian any package dependencies not already in your workspace: ::

  cd ~/ws_moveit/src
  rosdep install -y --from-paths . --ignore-src --rosdistro kinetic

The next command will configure your catkin workspace: ::

  cd ~/ws_moveit
  catkin config --extend /opt/ros/kinetic
  catkin build

Source the catkin workspace: ::

  source ~/ws_moveit/devel/setup.bash

Add the previous command to your ``.bashrc``: ::

   echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc

.. note:: Sourcing the ``setup.bash`` automatically in your ``~/.bashrc`` is
   not required and often skipped by advanced users who use more than one
   Catkin workspace at a time, but we recommend it for simplicity

Next Step
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Visualize a robot with the interactive motion planning plugin for RViz <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_
