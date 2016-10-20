Motion Planning Pipeline Tutorial
==================================

In MoveIt!, the motion planners are setup to plan paths. However, there are often
times when we may want to pre-process the motion planning request or post-process
the planned path (e.g. for time parameterization). In such cases, we use
the planning pipeline which chains a motion planner with pre-processing and post-processing
stages. The pre and post-processing stages, called planning request adapters, can
be configured by name from the ROS parameter server. In this tutorial, we will
run you through the C++ code to instantiate and call such a planning pipeline.

.. tutorial-formatter:: ../planning_pipeline_tutorial.cpp

The entire code
^^^^^^^^^^^^^^^
The entire code can be seen :codedir:`here in the moveit_pr2 github project<planning>`.

Compiling the code
^^^^^^^^^^^^^^^^^^
Follow the `instructions for compiling code from source <http://moveit.ros.org/install/>`_.

The launch file
^^^^^^^^^^^^^^^
The entire launch file is `here <https://github.com/ros-planning/moveit_tutorials/tree/kinetic-devel/doc/pr2_tutorials/planning/launch/planning_pipeline_tutorial.launch>`_ on github. All the code in this tutorial can be compiled and run from the moveit_tutorials package that you have as part of your MoveIt! setup.

Running the code
^^^^^^^^^^^^^^^^

Roslaunch the launch file to run the code directly from moveit_tutorials::

 roslaunch moveit_tutorials motion_planning_api_tutorial.launch

Expected Output
^^^^^^^^^^^^^^^

In Rviz, we should be able to see three trajectories being replayed eventually:

 1. The robot moves its right arm to the pose goal in front of it,
 2. The robot moves its right arm to the joint goal to the side,
 3. The robot moves its right arm back to the original pose goal in front of it,
