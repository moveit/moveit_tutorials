Integration/Unit Tests
======================

How to test changes to MoveIt! on various robots, including unit and integration tests.

**Note:** *This is a stub tutorial, to be expanded upon in the future*

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

Integration Test
----------------

A Python-based integration test is available for testing higher-level move_group functionality in MoveIt! - to run: ::

 rostest moveit_ros_planning_interface python_move_group.test

Test Robots
-----------

**Panda**

From the package `panda_moveit_config <https://github.com/ros-planning/panda_moveit_config>`_: ::

  roslaunch panda_moveit_config demo.launch

**Fanuc M-10iA**

From the package `moveit_resources <https://github.com/ros-planning/moveit_resources>`_: ::

  roslaunch moveit_resources demo.launch

Unit Tests
----------

To run unit tests locally on the entire MoveIt! catkin workspace using catkin-tools: ::

  catkin run_tests -iv

To run a test for just 1 package::

  catkin run_tests --no-deps --this -iv

Kinematic Tests
---------------

An additional test suite for the KinematicBase features in MoveIt! is available in the package `moveit_kinematic_tests <https://github.com/ros-planning/moveit_kinematics_tests>`_.
