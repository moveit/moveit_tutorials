MoveIt Tutorials
=================

These tutorials will step you through using and learning the MoveIt Motion Planning Framework.

.. image:: doc/quickstart_in_rviz/rviz_plugin_head.png
   :width: 700px

In these tutorials, the Franka Emika Panda robot is used as a quick-start demo. Alternatively, you can easily use any robot that has already been configured to work with MoveIt - check the `list of robots running MoveIt <http://moveit.ros.org/robots/>`_ to see whether MoveIt is already available for your robot. Otherwise, you can setup MoveIt to work with your custom robot in the tutorial section "Integration with a New Robot", below.

Getting Started with MoveIt and RViz
-------------------------------------
.. toctree::
   :maxdepth: 1

   doc/getting_started/getting_started
   doc/quickstart_in_rviz/quickstart_in_rviz_tutorial

MoveGroup - ROS Wrappers in C++ and Python
------------------------------------------
The simplest way to use MoveIt through scripting is using the ``move_group_interface``. This interface is ideal for beginners and provides unified access to many of the features of MoveIt.

.. toctree::
   :maxdepth: 1

   doc/move_group_interface/move_group_interface_tutorial
   doc/move_group_python_interface/move_group_python_interface_tutorial
   doc/moveit_commander_scripting/moveit_commander_scripting_tutorial

Using MoveIt Directly Through the C++ API
------------------------------------------
Building more complex applications with MoveIt often requires developers to dig into MoveIt’s C++ API. As an added plus, using the C++ API directly skips many of the ROS Service/Action layers resulting in significantly faster performance.

.. toctree::
   :maxdepth: 1

   doc/robot_model_and_robot_state/robot_model_and_robot_state_tutorial
   doc/planning_scene/planning_scene_tutorial
   doc/planning_scene_ros_api/planning_scene_ros_api_tutorial
   doc/motion_planning_api/motion_planning_api_tutorial
   doc/motion_planning_pipeline/motion_planning_pipeline_tutorial
   doc/visualizing_collisions/visualizing_collisions_tutorial
   doc/time_parameterization/time_parameterization_tutorial
   doc/planning_with_approximated_constraint_manifolds/planning_with_approximated_constraint_manifolds_tutorial
   doc/pick_place/pick_place_tutorial

Integration with a New Robot
----------------------------
Before attempting to integrate a new robot with MoveIt, check whether your robot has already been setup (see the `list of robots running MoveIt <http://moveit.ros.org/robots/>`_). Otherwise, follow the tutorials in this section to integrate your robot with MoveIt (and share your results on the MoveIt mailing list)

.. toctree::
   :maxdepth: 1

   doc/setup_assistant/setup_assistant_tutorial
   doc/urdf_srdf/urdf_srdf_tutorial
   doc/controller_configuration/controller_configuration_tutorial
   doc/perception_pipeline/perception_pipeline_tutorial
   doc/ikfast/ikfast_tutorial
   doc/trac_ik/trac_ik_tutorial

Configuration
-------------
.. toctree::
   :maxdepth: 1

   doc/kinematics_configuration/kinematics_configuration_tutorial
   doc/fake_controller_manager/fake_controller_manager_tutorial
   doc/custom_constraint_samplers/custom_constraint_samplers_tutorial
   doc/ompl_interface/ompl_interface_tutorial
   doc/chomp_planner/chomp_planner_tutorial
   doc/stomp_planner/stomp_planner_tutorial
   doc/planning_adapters/planning_adapters_tutorial.rst

Miscellaneous
----------------------------

.. toctree::
   :maxdepth: 1

   doc/joystick_control_teleoperation/joystick_control_teleoperation_tutorial
   doc/benchmarking/benchmarking_tutorial
   doc/tests/tests_tutorial

Attribution
-----------
Major contributors to the MoveIt tutorials are listed in chronological order: Sachin Chitta, Dave Hershberger, Acorn Pooley, Dave Coleman, Michael Gorner, Francisco Suarez, Mike Lautman. Help us improve these docs and we'll be happy to include you here also!

The tutorials had a major update in 2018 during a code sprint sponsored by Franka Emika in collaboration with PickNik (`Check out the blog post! <http://moveit.ros.org/moveit!/ros/2018/02/26/tutorials-documentation-codesprint.html>`_)

.. image:: ./_static/franka_logo.png
   :width: 300px
