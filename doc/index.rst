MoveIt! Tutorials
=================

These tutorials will run you through how to use MoveIt! with your robot. It is assumed that you have already configured MoveIt! for your robot - check the `list of robots running MoveIt! <http://moveit.ros.org/robots/>`_ to see whether MoveIt! is already available for your robot. Otherwise, skip to the tutorial on Setting up MoveIt! for your robot. If you just want to test MoveIt!, use the PR2 as your quick-start robot.

Beginner
--------

The primary user interface to MoveIt! is through the move_group_interface. You can use this interface both through C++ and Python. A GUI-based interface is available through the use of the MoveIt! Rviz Plugin. We will walk through each of these interfaces in detail:

.. toctree::
   :maxdepth: 1

   ros_visualization/visualization_tutorial
   pr2_tutorials/planning/src/doc/move_group_interface_tutorial
   pr2_tutorials/planning/scripts/doc/move_group_python_interface_tutorial

Advanced
--------

This set of advanced tutorials is meant for developers who are using MoveIt!’s C++ API. Most users wanting to access MoveIt! in C++ or Python should use the move_group_interface (above).

.. toctree::
   :maxdepth: 1

   pr2_tutorials/kinematics/src/doc/kinematics_tutorial
   pr2_tutorials/planning/src/doc/planning_scene_tutorial
   pr2_tutorials/planning/src/doc/planning_scene_ros_api_tutorial
   pr2_tutorials/planning/src/doc/motion_planning_api_tutorial
   pr2_tutorials/planning/src/doc/planning_pipeline_tutorial
   fake_controller_manager_tutorial
   ros_visualization/joystick.rst
   custom_constraint_samplers

Integration with New Robot
--------------------------

Before attempting to integrate a new robot with MoveIt!, check whether your robot has already been setup (see the `list of robots running MoveIt! <http://moveit.ros.org/robots/>`_). Otherwise, follow the tutorials in this section to integrate your robot with MoveIt! (and share your results on the MoveIt! mailing list)

.. toctree::
   :maxdepth: 1

   setup_assistant/setup_assistant_tutorial
   pr2_tutorials/planning/src/doc/controller_configuration
   pr2_tutorials/planning/src/doc/perception_configuration
   ikfast_tutorial
   trac_ik_tutorial

Configuration
-------------

.. toctree::
   :maxdepth: 1

   pr2_tutorials/kinematics/src/doc/kinematics_configuration
   ompl_interface_tutorial

Attribution
-----------

The original MoveIt! tutorials were created by Sachin Chitta, Dave Hershberger, and Acorn Pooley at SRI International. Further improvements have been made by Dave Coleman, Michael Gorner, and Francisco Suarez. Please help us improve these docs!
