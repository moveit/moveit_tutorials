CHOMP Planner
===============

.. image:: chomp.png
   :width: 700px

Covariant Hamiltonian optimization for motion planning (CHOMP) is a novel gradient-based trajectory optimization procedure that makes many everyday motion planning problems both simple and trainable (Ratliff et al., 2009c). While most high-dimensional motion planners separate trajectory generation into distinct planning and optimization stages, this algorithm capitalizes on covariant gradient and functional gradient approaches to the optimization stage to design a motion planning algorithm based entirely on trajectory optimization. Given an infeasible naive trajectory, CHOMP reacts to the surrounding environment to quickly pull the trajectory out of collision while simultaneously optimizing dynamical quantities such as joint velocities and accelerations. It rapidly converges to a smooth collision-free trajectory that can be executed efficiently on the robot. Integration into latest version of MoveIt! is `work in progress <https://github.com/ros-planning/moveit/issues/702>`_. `More info <http://www.nathanratliff.com/thesis-research/chomp>`_

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

You should also have gone through the steps in `Visualization with MoveIt! RViz Plugin <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_

Prerequisites
--------------
 1. You must have the latest version of MoveIt! installed. On ROS Kinetic you will need to build MoveIt! from source. A build from source is required as CHOMP is not officially released so ``apt-get install`` for moveIt would not be appropriate here. We will go through the steps for doing this below.
 2. To use CHOMP with your robot you must already have a MoveIt! configuration package for your robot already. For example, if you have a Panda robot, it's probably called ``panda_moveit_config``. This is typically built using the `MoveIt! Setup Assistant <../setup_assistant/setup_assistant_tutorial.html>`_.

Installing MoveIt! from Source
------------------------------
As you add and remove packages from your workspace you will need to clean your workspace and re-run the command to install new missing dependencies. Clean your workspace to remove references to the system wide installation of MoveIt!: ::

  cd ~/ws_moveit/src
  catkin clean

Now follow the instructions on the MoveIt! homepage for `installing MoveIt! Kinetic from source <http://moveit.ros.org/install/source/>`_. Note that you can skip the **Prerequisites** section since you should already have a Catkin workspace.

Re-source the setup files: ::

  source ~/ws_moveit/devel/setup.bash

Using CHOMP with Your Robot
---------------------------
**Note:** if you are following this demo using the ``panda_moveit_config`` from the `ros-planning/panda_moveit_config <https://github.com/ros-planning/panda_moveit_config>`_ repository, these steps are already done for you and you can skip this section.

#. Simply download `chomp_planning_pipeline.launch.xml <https://github.com/ros-planning/panda_moveit_config/blob/master/launch/chomp_planning_pipeline.launch.xml>`_ file into the launch directory of your MoveIt! config package. In our case, we will save this file in the ``panda_moveit_config/launch`` directory.
#. Adjust the line ``<rosparam command="load" file="$(find panda_moveit_config)/config/chomp_planning.yaml" />`` to ``<rosparam command="load" file="$(find <robot_moveit_config>)/config/chomp_planning.yaml" />`` replacing ``<robot_moveit_config>`` with the name of your MoveIt! configuration package.
#. Download `chomp_planning.yaml <https://github.com/ros-planning/panda_moveit_config/blob/master/config/chomp_planning.yaml>`_ file into the config directory of your MoveIt! config package. In our case, we will save this file in the ``panda_moveit_config/config`` directory.
#. Open ``chomp_planning.yaml`` in your favorite editor and change ``animate_endeffector_segment: "panda_rightfinger"`` to the appropriate link for your robot.
#. Copy the ``demo.launch`` file to ``demo_chomp.launch``. Note that this file is also in the launch directory of your MoveIt! config package. In our case, the ``panda_moveit_config/launch`` directory.
#. Find the lines where ``move_group.launch`` is included and change it to: ::

    <!-- Replace <robot_moveit_config> with the name of your MoveIt! configuration package -->
    <include file="$(find <robot_moveit_config>)/launch/move_group.launch">
      <arg name="allow_trajectory_execution" value="true"/>
      <arg name="fake_execution" value="true"/>
      <arg name="info" value="true"/>
      <arg name="debug" value="$(arg debug)"/>
      <arg name="planner" value="chomp" />
    </include>
#. Open the ``move_group.launch`` file in your ``<robot_moveit_config>/launch/`` folder and make two changes.

 7.1. First, add ``<arg name="planner" default="ompl" />`` just under the ``<launch>`` tag, and,

 7.2. Second, within the ``<include ns="move_group">`` tag replace ``<arg name="pipeline" value="ompl" />`` with ``<arg name="pipeline" value="$(arg planner)" />``.

Running the Demo
----------------
If you have the ``panda_moveit_config`` from the `ros-planning/panda_moveit_config <https://github.com/ros-planning/panda_moveit_config>`_ repository you should be able to simply run the demo: ::

  roslaunch panda_moveit_config demo_chomp.launch

Testing CHOMP with Obstacles in the Scene
-----------------------------------------
To test CHOMP in an evironment with obstacles, you can run any of the sample python scripts (`collision_scene_test_1.py <./collision_scene_test1.py>`_ or `collision_scene_test_2.py <./collision_scene_test2.py>`_). The first scripts creates a complex scene with four ostacles. The second script creates a simple environment with one obstacle. One can change the position/size of the obstacles to change the scene. 


To run the CHOMP planner with obstacles, do the following in two seperate terminals: ::

  roslaunch panda_moveit_config demo_chomp.launch
  python collision_scene_test_1.py OR python collision_scene_test_2.py
