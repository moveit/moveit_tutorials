Planning Scene ROS API
==================================

In this tutorial, we will examine the use of planning scene diffs to perform
two operations:

 * Adding and removing objects into the world
 * Attaching and detaching objects to the robot

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

Running the code
----------------
Open two shells. In the first shell start RViz and wait for everything to finish loading: ::

  roslaunch panda_moveit_config demo.launch

In the second shell, run the launch file for this demo: ::

  roslaunch moveit_tutorials planning_scene_ros_api_tutorial.launch

**Note:** This tutorial uses the **RvizVisualToolsGui** panel to step through the demo. To add this panel to RViz, follow the instructions in the `Visualization Tutorial <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html#rviz-visual-tools>`_.

After a short moment, the RViz window should appear and look similar to the one at the top of this page. To progress through each demo step either press the **Next** button in the **RvizVisualToolsGui** panel at the bottom of the screen or select **Key Tool** in the **Tools** panel at the top of the screen and then press **N** on your keyboard while RViz is focused.

Expected Output
---------------
In RViz, you should be able to see the following:
 * Object appear in the planning scene
 * Object gets attached to the robot
 * Object gets detached from the robot
 * Object is removed from the planning scene

.. role:: red

**Note:** You may see an error message reading :red:`Found empty JointState message`. This is a known bug and will be fixed soon.

The entire code
---------------
The entire code can be seen :codedir:`here in the MoveIt GitHub project<planning_scene_ros_api>`.

.. tutorial-formatter:: ./src/planning_scene_ros_api_tutorial.cpp

The launch file
---------------
The entire launch file is :codedir:`here <planning_scene_ros_api/launch/planning_scene_ros_api_tutorial.launch>` on GitHub. All the code in this tutorial can be compiled and run from the moveit_tutorials package.

Debugging the Planning Scene Monitor
------------------------------------
To aid in debugging detached and attached collision objects, a command-line tool will help introspect your system: ::

  rosrun moveit_ros_planning moveit_print_planning_scene_info
