MoveIt Grasps
=======================

MoveIt Grasps is a grasp generator for objects such as blocks or cylinders for use with the MoveIt pick and place pipeline. It also contains its own components for filtering grasps based on reachability and approach/retreat motions.
The grasp generation algorithm is based on simple cuboid shapes and does not consider friction cones or other grasp dynamics.

Its current implementation takes as input a pose of a cuboid, and the cuboid's size, and generates a large number of potential grasp approaches and directions.
Also included is a grasp filter for removing kinematically infeasible grasps via threaded IK solvers.

.. image:: moveit_grasps.png
   :width: 500pt
   
Installing MoveIt Grasps
^^^^^^^^^^^^^^^^^^^^^^^^

Install From Source
--------------------

Clone the `moveit_grasps <https://github.com/ros-planning/moveit_grasps>`_ repository into a `catkin workspace <https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#create-a-catkin-workspace>`_. For this tutorial we use Franka Emika's Panda robot setup from `panda_moveit_config <https://github.com/ros-planning/panda_moveit_config>`_::

    git clone https://github.com/ros-planning/moveit_grasps

Use the rosdep tool to automatically install its dependencies::

    rosdep install --from-paths src --ignore-src --rosdistro kinetic
    
Build the workspace::

    catkin build

Setup
^^^^^^^^^^^^^^^^

MoveIt Grasps is based on the three main components *Grasp Generator*, *Grasp Filter*, and *Grasp Planner*.
The *Grasp Generator* uses the end effector kinematic and the object shape for sampling grasp poses and optimizing them using geometric scoring functions.
The *Grasp Filter* validates the feasibility of grasp candidates by searching for IK solutions to verify their reachability.
The *Grasp Planner* computes Cartesian approach, lift, and retreat trajectories that compose a complete grasp motion.

In order to run the full grasp pipeline the three components need to be applied in sequence.
An example for generating, filtering and planning grasp motions can be found inside the file `src/grasp_pipeline_demo.cpp  <https://github.com/ros-planning/moveit_grasps/blob/kinetic-devel/src/demo/grasp_pipeline_demo.cpp>`_. The grasp pipeline demo can be run by launching `launch/grasp_pipeline_demo.launch <https://github.com/ros-planning/moveit_grasps/blob/kinetic-devel/launch/grasp_pipeline_demo.launch>`_.

Robot-Agnostic Configuration
----------------------------

MoveIt Grasps requires two configuration files to be specified at launch.
One describes the robot's end effector geometry and the other configures the *Grasp Generator*, *Grasp Filter* and *Grasp Planner*.
An example end effector configuration for Franka Emika's Panda can be found under `config_robot/panda_grasp_data.yaml <https://github.com/ros-planning/moveit_grasps/blob/kinetic-devel/config_robot/panda_grasp_data.yaml>`_.
In that file you will find all of the gripper specific parameters necessary for customizing MoveIt Grasps with suction or finger grippers.
The three grasp components are configured inside `config/moveit_grasps_config.yaml <https://github.com/ros-planning/moveit_grasps/blob/kinetic-devel/config/moveit_grasps_config.yaml>`_.
See the comments in both files for further explanation of the parameters. 

To apply your configuration simply load them as rosparams with your grasping node.
For an example see the segment below from the file `launch/grasp_pipeline_demo.launch <https://github.com/ros-planning/moveit_grasps/blob/kinetic-devel/launch/grasp_pipeline_demo.launch>`_::

    ...
    <node name="mmmmoveit_grasps_demo" launch-prefix="$(arg launch_prefix)" pkg="moveit_grasps"
    type="moveit_grasps_pipeline_demo" output="screen" args="$(arg command_args)">
      <param name="ee_group_name" value="hand"/>
      <param name="planning_group_name" value="panda_arm"/>
      <rosparam command="load" file="$(find moveit_grasps)/config_robot/panda_grasp_data.yaml"/>
      <rosparam command="load" file="$(find moveit_grasps)/config/moveit_grasps_config.yaml"/>
    </node>
    ...
    
 Note that also the robot's planning group and end effector group must be specified under the parameters ``ee_group_name`` and ``planning_group_name``.

Since the set of parameters is quite extensive there are different demo launch files that you can use to visualize the effects. 
For this you can apply your configuration to the launch files ``grasp_generator_demo.launch``, ``grasp_poses_visualizer_demo.launch``, or ``grasp_pipeline_demo.launch`` and run them.
The results should resemble the image below:

.. image:: https://raw.githubusercontent.com/ros-planning/moveit_grasps/kinetic-devel/resources/moveit_grasps_poses.jpeg
   :width: 500pt

Some Important Parameters:
---------------------------

**grasp_pose_to_eef_transform**

The ``grasp_pose_to_eef_transform`` represents the transform from the wrist to the end-effector.
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
^^^^^^^^^^^^

There are four demo scripts in this package. To view the tests, first start Rviz with::

    roslaunch moveit_grasps rviz.launch

To see the entire MoveIt Grasps pipeline in action::

    roslaunch moveit_grasps grasp_pipeline_demo.launch

To visualize gripper specific parameters::

    roslaunch moveit_grasps grasp_poses_visualizer_demo.launch

To test just grasp generation for randomly placed blocks::

    roslaunch moveit_grasps demo_grasp_generator.launch

To test the grasp filtering::

    roslaunch moveit_grasps demo_filter.launch

Grasp Filter
------------

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
