MoveIt! Grasps
=======================

MoveIt! Grasps is a grasp generator for objects such as blocks or cylinders for use with the MoveIt! pick and place pipeline. It also contains its own components for filtering grasps based on reachability and approach/retreat motions.
The grasp generation algorithm is based on simple cuboid shapes and does not consider friction cones or other grasp dynamics.

Its current implementation takes as input a pose of a cuboid, and the cuboid's size, and generates a large number of potential grasp approaches and directions.
Also included is a grasp filter for removing kinematically infeasible grasps via threaded IK solvers.

.. image:: https://raw.githubusercontent.com/PickNikRobotics/moveit_grasps/kinetic-devel/resources/demo.png
   :width: 500pt
   
Installing MoveIt! Grasps
^^^^^^^^^^^^^^^^^^^^^^^^

Ubuntu Debian
--------------

   Note: this package has not been released yet
::

    sudo apt-get install ros-kinetic-moveit-grasps


Install From Source
--------------------

Clone the `moveit_grasps <https://github.com/PickNikRobotics/moveit_grasps>`_ repository into a `catkin workspace <http://wiki.ros.org/catkin/Tutorials/create_a_workspace>`_::

    git clone https://github.com/PickNikRobotics/moveit_grasps

Use the rosdep tool to automatically install its dependencies::

    rosdep install --from-paths src --ignore-src --rosdistro kinetic
    
Build the workspace::

    catkin build

Setup
^^^^^^^^^^^^^^^^

Robot-Agnostic Configuration
----------------------------

You will first need a configuration file that described your robot's end effector geometry.
Currently an example format can be seen in this repository at `config_robot/baxter_grasp_data.yaml <https://github.com/PickNikRobotics/moveit_grasps/blob/kinetic-devel/config_robot/baxter_grasp_data.yaml>`_.
See the comments within that file for explanations.

To load that file at launch, you copy the example in the file `launch/grasp_test.launch <https://github.com/PickNikRobotics/moveit_grasps/blob/kinetic-devel/launch/load_panda.launch>`_ where you should see the line::

    ...
    
    <rosparam command="load" file="$(find moveit_grasps)/config_robot/panda_grasp_data.yaml"/>
    
    ...

Within that file you will find all of the gripper specific parameters necessary for customizing MoveIt! Grasps with any suction or finger gripper.

These values can be visualized by launching ``grasp_generator_demo.launch``, ``grasp_poses_visualizer_demo.launch``, and ``grasp_pipeline_demo.launch``.
The result should look like the following:

.. image:: https://raw.githubusercontent.com/PickNikRobotics/moveit_grasps/kinetic-devel/resources/moveit_grasps_poses.jpeg
   :width: 500pt

Some Important Parameters:
---------------------------

**grasp_pose_to_eef_transform**

The ``grasp_pose_to_eef_transform`` represents the transform from the wrist to the end-effector.
This parameter is provided to allow different URDF end effectors to all work together without recompiling code.
In MoveIt! the EE always has a parent link, typically the wrist link or palm link.
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

To see the entire MoveIt! Grasps pipeline in action::

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
* `Baxter <https://github.com/davetcoleman/baxter_cpp>`_
* `REEM <http://wiki.ros.org/Robots/REEM>`_
* Panda

Example Code
------------

The most current example for using MoveIt! Grasps is the ``grasp_pipeline_demo`` which can be found `here <https://github.com/PickNikRobotics//moveit_grasps/kinetic-devel/src/grasp_pipeline_demo.cpp>`_.

There are other example implementations:

* `baxter_pick_place <https://github.com/davetcoleman/baxter_cpp/tree/kinetic-devel/baxter_pick_place>`_
* `reem_tabletop_grasping <https://github.com/pal-robotics/reem_tabletop_grasping>`_
