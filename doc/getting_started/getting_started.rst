Getting Started
===============

This tutorial will install MoveIt and create a workspace sandbox to run the tutorials and example robot.

Install ROS and Catkin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Install ROS Noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>`_.
It is easy to miss steps when going through the ROS installation tutorial. If you run into errors in the next few steps, a good place to start is to go back and make sure you have installed ROS correctly.

Once you have ROS installed, make sure you have the most up to date packages: ::

  rosdep update
  sudo apt update
  sudo apt dist-upgrade

Install `catkin <http://wiki.ros.org/catkin>`_ the ROS build system: ::

  sudo apt install ros-noetic-catkin python3-catkin-tools

Install `wstool <http://wiki.ros.org/wstool>`_ : ::

  sudo apt install python3-wstool

Create A Catkin Workspace and Download MoveIt Source
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Because the version of the tutorials uses the ``master`` branch which is being actively developed, you will most likely need to build all of MoveIt from soruce. You will need to have a `catkin <http://wiki.ros.org/catkin>`_ workspace setup: ::

  mkdir -p ~/ws_moveit/src
  cd ~/ws_moveit/src

  wstool init .
  wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
  wstool remove  moveit_tutorials  # this is clone'd in the next section
  wstool update -t .

Download Example Code
^^^^^^^^^^^^^^^^^^^^^

To easily follow along with these tutorials, you will need a **ROBOT_moveit_config** package. The default demo robot is the Panda arm from Franka Emika. To get a working **panda_moveit_config** package, we recommend you install from source.

Within your `catkin <http://wiki.ros.org/catkin>`_ workspace, download the tutorials as well as the ``panda_moveit_config`` package: ::

  cd ~/ws_moveit/src
  git clone https://github.com/ros-planning/moveit_tutorials.git -b master
  git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel

.. note:: For now we will use a pre-generated ``panda_moveit_config`` package but later we will learn how to make our own in the `MoveIt Setup Assistant tutorial <../setup_assistant/setup_assistant_tutorial.html>`_.

Build your Catkin Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following will install from Debian any package dependencies not already in your workspace: ::

  cd ~/ws_moveit/src
  rosdep install -y --from-paths . --ignore-src --rosdistro noetic

**Note** In case an upstream package is not (yet) available from the standard ROS repositories or if you experience any build errors in those packages, please try to fetch the latest release candidates from the `ROS testing repositories <http://wiki.ros.org/TestingRepository>`_ instead: ::

        sudo sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt update

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
