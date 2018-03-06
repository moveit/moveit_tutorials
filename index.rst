MoveIt! Tutorials
=================

These tutorials will run you through how to use MoveIt! with your robot. It is assumed that you have already configured MoveIt! for your robot - check the `list of robots running MoveIt! <http://moveit.ros.org/robots/>`_ to see whether MoveIt! is already available for your robot. Otherwise, skip to the tutorial on Setting up MoveIt! for your robot. If you just want to test MoveIt!, use the Panda as your quick-start robot.

.. note:: All tutorials referencing the Panda have only been tested with ROS Kinetic. For earlier versions of ROS (eg. indigo) See `the ROS Indigo tutorials with the PR2 here <http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/ikfast_tutorial.html>`_

Beginner
--------

The primary user interface to MoveIt! is through the move_group_interface. You can use this interface both through C++ and Python. A GUI-based interface is available through the use of the MoveIt! Rviz Plugin. We will walk through each of these interfaces in detail:

.. toctree::
   :maxdepth: 1

   doc/visualization/visualization_tutorial
   doc/move_group_interface/move_group_interface_tutorial
   doc/move_group_python_interface/move_group_python_interface_tutorial
   doc/moveit_commander/moveit_commander_tutorial

Advanced
--------

This set of advanced tutorials is meant for developers who are using MoveIt!’s C++ API. Most users wanting to access MoveIt! in C++ or Python should use the move_group_interface (above).

.. toctree::
   :maxdepth: 1

   doc/kinematic_model/kinematic_model_tutorial
   doc/planning_scene/planning_scene_tutorial
   doc/planning_scene_ros_api/planning_scene_ros_api_tutorial
   doc/motion_planning_api/motion_planning_api_tutorial
   doc/motion_planning_pipeline/motion_planning_pipeline_tutorial
   doc/collision_contact/collision_contact_tutorial
   doc/time_parameterization/time_parameterization_tutorial
   doc/fake_controller_manager/fake_controller_manager_tutorial
   doc/joystick_control_teleoperation/joystick_control_teleoperation_tutorial
   doc/planning_with_approximated_constraint_manifolds/planning_with_approximated_constraint_manifolds_tutorial
   doc/custom_constraint_samplers/custom_constraint_samplers_tutorial
   doc/benchmarking/benchmarking_tutorial
   doc/tests/tests_tutorial

Integration with New Robot
--------------------------

Before attempting to integrate a new robot with MoveIt!, check whether your robot has already been setup (see the `list of robots running MoveIt! <http://moveit.ros.org/robots/>`_). Otherwise, follow the tutorials in this section to integrate your robot with MoveIt! (and share your results on the MoveIt! mailing list)

.. toctree::
   :maxdepth: 1

   doc/setup_assistant/setup_assistant_tutorial
   doc/controller_configuration/controller_configuration_tutorial
   doc/perception_configuration/perception_configuration_tutorial
   doc/ikfast/ikfast_tutorial
   doc/trac_ik/trac_ik_tutorial

Configuration
-------------

.. toctree::
   :maxdepth: 1

   doc/urdf_srdf/urdf_srdf_tutorial
   doc/kinematics_configuration/kinematics_configuration_tutorial
   doc/ompl_interface/ompl_interface_tutorial
   doc/chomp_interface/chomp_interface_tutorial

Attribution
-----------
Major contributors to the MoveIt! tutorials are listed in chronological order: Sachin Chitta, Dave Hershberger, Acorn Pooley, Dave Coleman, Michael Gorner, Francisco Suarez, Mike Lautman. Help us improve these docs and we'll be happy to include you here also!

The tutorials had a major update in 2018 during a codesprint sponsored by Franka Emika in collaboration with PickNik (`Check out the blog post! <http://moveit.ros.org/moveit!/ros/2018/02/26/tutorials-documentation-codesprint.html>`_)

.. image:: ./_static/franka_logo.png
   :width: 300px


