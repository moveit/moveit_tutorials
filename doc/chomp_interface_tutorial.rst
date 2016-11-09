CHOMP Interface
===============

**Note:** *The chomp planner has not been tested extensively yet.*

Run Generic Demo for Fanuc M-10iA
---------------------------------

To run the demo you'll need the `moveit_resources <https://github.com/ros-planning/moveit_resources>`_ package.

Once you have this package simply run::

 roslaunch moveit_resources demo_chomp.launch

Assumptions:
------------

 1. You have the latest version of moveit installed. On ROS kinetic you may need to build it from source.
 2. You have a moveit configuration package for your robot already. For example, if you have a Kinova Jaco arm, it's probably called "jaco_moveit_config". This is typically built using the Moveit Setup Assistant.
 3. Lets assume that you are using the **jaco** manipulator. And hence, the moveit config package is *jaco_moveit_config*.


Using CHOMP with your own robot
-------------------------------

1. Simply download `chomp_planning_pipeline.launch.xml <https://github.com/ros-planning/moveit_resources/blob/master/fanuc_moveit_config/launch/chomp_planning_pipeline.launch.xml>`_ file into the launch directory of your moveit config package. So into the *jaco_moveit_config/launch* directory.
2. Copy the *demo.launch* file to *demo_chomp.launch*. Note that this file is also in the launch directory of the *jaco_moveit_config* package.
3. Find the lines where *move_group.launch* is included and change it to::

    <include file="$(find jaco_moveit_config)/launch/move_group.launch">
      <arg name="allow_trajectory_execution" value="true"/>
      <arg name="fake_execution" value="true"/>
      <arg name="info" value="true"/>
      <arg name="debug" value="$(arg debug)"/>
      <arg name="planner" value="chomp" />
    </include>

You probably only need to change the planner arg to chomp.

4. Run the demo::
       
    roslaunch jaco_moveit_config demo_chomp.launch
