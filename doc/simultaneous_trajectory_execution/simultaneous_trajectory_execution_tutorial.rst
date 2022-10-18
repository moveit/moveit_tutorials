Simultaneous Trajectory Execution
==================================

Introduction
------------
MoveIt now allows simultaneous execution of trajectories, as long as, each trajectory uses a different set of controllers. For example, in a dual arm environment, each arm can execute a different set of trajectories without needing to wait for the other arm to finish moving or manually synchronizing the motion of both arm into a single trajectory. Optionally, a collision check is performed right before execution of new trajectories to prevent collisions with active trajectories.


.. only:: html

   .. figure:: simultaneous-execution-rviz.gif

   Simultaneous execution of several trajectories through Rviz plugin.


Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

Setup
---------------
The simultaneous trajectory execution feature can be enabled or disabled through the dynamic reconfigure parameter `/move_group/trajectory_execution/enable_simultaneous_execution`.
Optionally, an extra layer of collision checking, done right before execution of trajectories, can be enabled through the dynamic reconfigure parameter `/move_group/trajectory_execution/enable_collision_checking`.

Running the code
----------------
Open two shells. In the first shell start RViz and wait for everything to finish loading: ::

  roslaunch moveit_resources_dual_panda_moveit_config demo.launch

In the second shell, run the launch file for this demo: ::

  roslaunch moveit_tutorials simultaneous_trajectory_execution_tutorial.launch

Expected Output
---------------
Though, two independent trajectories for two different joint groups have been planned, both can be simultaneously executed.

The entire code
---------------
The entire code can be seen :codedir:`here in the MoveIt GitHub project<simultaneous_trajectory_execution>`.

.. tutorial-formatter:: ./src/simultaneous_trajectory_execution_tutorial.cpp

The launch file
---------------
The entire launch file is :codedir:`here <simultaneous_trajectory_execution/launch/simultaneous_trajectory_execution_tutorial.launch>` on GitHub. All the code in this tutorial can be compiled and run from the moveit_tutorials package.
