Getting Started
===============

.. note:: You are on the latest **ROS Melodic** version of the tutorials, which is less stable. For beginners we recommmend the more stable `ROS Kinetic tutorials <http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html>`_ (Requires Ubuntu 16.04). For older computers on Ubuntu 14.04 see `ROS Indigo tutorials with the PR2 <http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/ikfast_tutorial.html>`_.

Install ROS and Catkin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Install ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`_.
It is easy to miss steps when going through the ROS installation tutorial. If you run into errors in the next few steps, a good place to start is to go back and make sure you have installed ROS correctly.

Once you have ROS installed, make sure you have the most up to date packages: ::

  rosdep update
  sudo apt-get update
  sudo apt-get dist-upgrade

Install `catkin <http://wiki.ros.org/catkin>`_ the ROS build system: ::

  sudo apt-get install ros-melodic-catkin python-catkin-tools

Install MoveIt!
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The simplest way to install MoveIt! is from pre-built binaries (Debian): ::

  sudo apt install ros-melodic-moveit

Advanced users might want to `install MoveIt! from source <http://moveit.ros.org/install/source/>`_.

Create A Catkin Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^
You will need to have a `catkin <http://wiki.ros.org/catkin>`_ workspace setup: ::

  mkdir -p ~/ws_moveit/src

Download the Example Code
^^^^^^^^^^^^^^^^^^^^^^^^^
Within your `catkin <http://wiki.ros.org/catkin>`_ workspace, download these tutorials: ::

  cd ~/ws_moveit/src
  git clone https://github.com/ros-planning/moveit_tutorials.git -b melodic-devel

You will also need a ``panda_moveit_config`` package to follow along with these tutorials: ::

  git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel

  .. note:: Oct 26, 2018: franka_description is `not fully available for ROS Melodic <https://github.com/frankaemika/franka_ros/issues/34>`_.

Until then please manually clone the franka description package: ::

  git clone https://github.com/frankaemika/franka_ros

.. note:: For now we will use a pre-generated ``panda_moveit_config`` package but later we will learn how to make our own in the `MoveIt! Setup Assistant tutorial <../setup_assistant/setup_assistant_tutorial.html>`_.

Build your Catkin Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following will install from Debian any package dependencies not already in your workspace: ::

  cd ~/ws_moveit/src
  rosdep install -y --from-paths . --ignore-src --rosdistro melodic

The next command will configure your catkin workspace: ::

  cd ~/ws_moveit
  catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
  catkin build

Source the catkin workspace: ::

  source ~/ws_moveit/devel/setup.bash

Optional: add the previous command to your ``.bashrc``: ::

   echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc

.. note:: Sourcing the ``setup.bash`` automatically in your ``~/.bashrc`` is
   not required and often skipped by advanced users who use more than one
   catkin workspace at a time, but we recommend it for simplicity.

Next Step
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Visualize a robot with the interactive motion planning plugin for RViz <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_
