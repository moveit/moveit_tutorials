ROS API Planning Scene Tutorial
==================================

In this tutorial, we will examine the use of planning scene diffs to perform
two operations:

 * Adding and removing objects into the world
 * Attaching and detaching objects to the robot

.. tutorial-formatter:: ../planning_scene_ros_api_tutorial.cpp

The entire code
^^^^^^^^^^^^^^^
The entire code can be seen :codedir:`here in the moveit_pr2 github project<planning>`.

Compiling the code
^^^^^^^^^^^^^^^^^^
Follow the `instructions for compiling code from source <http://moveit.ros.org/install/>`_.

The launch file
^^^^^^^^^^^^^^^
The entire launch file is `here <https://github.com/ros-planning/moveit_tutorials/tree/kinetic-devel/doc/pr2_tutorials/planning/launch/planning_scene_ros_api_tutorial.launch>`_ on github. All the code in this tutorial can be compiled and run from the moveit_tutorials package
that you have as part of your MoveIt! setup.

Running the code
^^^^^^^^^^^^^^^^

Roslaunch the launch file to run the code directly from moveit_tutorials::

 roslaunch moveit_tutorials planning_scene_ros_api_tutorial.launch

Expected Output
^^^^^^^^^^^^^^^

In rviz, you should be able to see the following:
 * Object appear in the planning scene
 * Object gets attached to the robot
 * Object gets detached from the robot
 * Object is removed from the planning scene
