MoveIt Grasps
=======================

.. image:: moveit_grasps.png
   :width: 500pt

MoveIt Grasps is a grasp generator for objects such as blocks or cylinders and can be used as a replacement for the MoveIt pick and place pipeline. MoveIt Grasps provides functionality for filtering grasps based on reachability and Cartesian planning of approach, lift and retreat motions.

The grasp generation algorithm is based on simple cuboid shapes and does not consider friction cones or other grasp dynamics.

MoveIt Grasps can be used with both parallel finger grippers and suction grippers.

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

Installing MoveIt Grasps
------------------------

Install From Source
^^^^^^^^^^^^^^^^^^^

Clone the `moveit_grasps <https://github.com/ros-planning/moveit_grasps>`_ repository into a `catkin workspace <https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#create-a-catkin-workspace>`_. For this tutorial we use Franka Emika's Panda robot setup from `panda_moveit_config <https://github.com/ros-planning/panda_moveit_config>`_::

    cd ~/ws_moveit/src
    git clone -b $ROS_DISTRO-devel https://github.com/ros-planning/moveit_grasps.git

Use the rosdep tool to automatically install its dependencies::

    rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO

Build the workspace::

    catkin build

Install From Debian
^^^^^^^^^^^^^^^^^^^

**Note:** this package has not been released as of 4/11/19::

    sudo apt-get install ros-$ROS_DISTRO-moveit-grasps

Setup
-----

MoveIt Grasps is based on the three main components *Grasp Generator*, *Grasp Filter*, and *Grasp Planner*.
The *Grasp Generator* uses the end effector kinematic and the object shape for sampling grasp poses and optimizing them using geometric scoring functions.
The *Grasp Filter* validates the feasibility of grasp candidates by searching for IK solutions to verify their reachability.
The *Grasp Planner* computes Cartesian approach, lift, and retreat trajectories that compose a complete grasp motion.

In order to run the full grasp pipeline the three components need to be applied in sequence.

An example for generating, filtering and planning grasp motions can be found inside the file `src/grasp_pipeline_demo.cpp  <https://github.com/ros-planning/moveit_grasps/blob/melodic-devel/src/demo/grasp_pipeline_demo.cpp>`_. The grasp pipeline demo can be run by launching `launch/grasp_pipeline_demo.launch <https://github.com/ros-planning/moveit_grasps/blob/melodic-devel/launch/grasp_pipeline_demo.launch>`_.

Robot-Agnostic Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

MoveIt Grasps requires two configuration files to be specified at launch. One describes the robot's end effector geometry and the other configures the *Grasp Generator*, *Grasp Filter* and *Grasp Planner*.

An example end effector configuration for Franka Emika's Panda can be found under `config_robot/panda_grasp_data.yaml <https://github.com/ros-planning/moveit_grasps/blob/melodic-devel/config_robot/panda_grasp_data.yaml>`_.

In that file you will find all of the gripper specific parameters necessary for customizing MoveIt Grasps with suction or finger grippers.

An example configuration file for the *Grasp Generator*, *Grasp Filter* and *Grasp Planner* can be found in `config/moveit_grasps_config.yaml <https://github.com/ros-planning/moveit_grasps/blob/melodic-devel/config/moveit_grasps_config.yaml>`_.

See the comments in both files for further explanation of the parameters.

To apply your configuration simply load them as rosparams with your grasping node.
For an example see the segment below from the file `launch/grasp_pipeline_demo.launch <https://github.com/ros-planning/moveit_grasps/blob/melodic-devel/launch/grasp_pipeline_demo.launch>`_::

    <node name="moveit_grasps_demo" launch-prefix="$(arg launch_prefix)" pkg="moveit_grasps"
    type="moveit_grasps_pipeline_demo" output="screen" args="$(arg command_args)">
      <param name="ee_group_name" value="hand"/>
      <param name="planning_group_name" value="panda_arm"/>
      <rosparam command="load" file="$(find moveit_grasps)/config_robot/panda_grasp_data.yaml"/>
      <rosparam command="load" file="$(find moveit_grasps)/config/moveit_grasps_config.yaml"/>
    </node>

Note that also the robot's planning group and end effector group must be specified under the parameters ``ee_group_name`` and ``planning_group_name``.

Since the set of parameters is quite extensive there are different demo launch files that you can use to visualize the effects. You can apply your configuration to the launch files ``grasp_generator_demo.launch``, ``grasp_poses_visualizer_demo.launch``, or ``grasp_pipeline_demo.launch`` and run them. More on the MoveIt Grasps demos below.

Notes on Some Important Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**tcp_to_eef_mount_transform**

The ``tcp_to_eef_mount_transform`` represents the transform from the tool center point used for grasp poses to the mount link of the end effector.
This parameter is provided to allow different URDF end effectors to all work together without recompiling code.

In MoveIt the EE always has a parent link, typically the wrist link or palm link.
That parent link should have its Z-axis pointing towards the object you want to grasp i.e. where your pointer finger is pointing.

This is the convention laid out in "Robotics" by John Craig in 1955.
However, a lot of URDFs do not follow this convention, so this transform allows you to fix it.

Additionally, the x-axis should be pointing up along the grasped object, i.e. the circular axis of a (beer) bottle if you were holding it.

The y-axis should be point towards one of the fingers.

**Switch from Bin to Shelf Picking**

The ``setIdealGraspPoseRPY()`` and ``setIdealGraspPose()`` methods in GraspGenerator can be used to select an ideal grasp orientation for picking.

These methods is used to score grasp candidates favoring grasps that are closer to the desired orientation.

This is useful in applications such as bin and shelf picking where you would want to pick the objects from a bin with a grasp that is vertically alligned and you would want to pick obejects from a shelf with a grasp that is horozontally alligned.

Demo Scripts
------------

We have provided 4 demo scripts showcasing MoveIt Grasps, and for visualizing MoveIt Grasps configuration parameters.

Before running any of the Demos, you must first start Rviz with::

    roslaunch moveit_grasps rviz.launch

**NOTE:** The released versions of `panda_moveit_config <https://github.com/ros-planning/panda_moveit_config>`_ may lag behind the source versions. If you have issues with the demos, a good first step would be to download and build `panda_moveit_config <https://github.com/ros-planning/panda_moveit_config>`_ from source.

1) The Entire MoveIt Grasps Pipeline
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To see the entire MoveIt Grasps pipeline in action run::

    roslaunch moveit_grasps grasp_pipeline_demo.launch

.. image:: grasp_pipeline_demo.gif
   :width: 500pt


2) Visualize Gripper Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To visualize gripper specific parameters::

    roslaunch moveit_grasps grasp_poses_visualizer_demo.launch

The result should look something like this:

.. image:: moveit_grasps_poses.jpg
   :width: 500pt

3) Visualize Grasp Generation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To visualize grasp generation for randomly placed blocks::

    roslaunch moveit_grasps grasp_generator_demo.launch

.. image:: grasp_generator_demo.png
   :width: 500pt

4) Grasp Filter
^^^^^^^^^^^^^^^
To demo the grasp filtering::

    roslaunch moveit_grasps grasp_filter_demo.launch

When filtered, the colors represent the following:

* RED - grasp filtered by ik
* PINK - grasp filtered by collision
* MAGENTA - grasp filtered by cutting plane
* YELLOW - grasp filtered by orientation
* BLUE - pregrasp filtered by ik
* CYAN - pregrasp filtered by collision
* GREEN - valid

Tested Robots
-------------

* UR5
* Jaco2
* Baxter
* `REEM <http://wiki.ros.org/Robots/REEM>`_
* Panda
