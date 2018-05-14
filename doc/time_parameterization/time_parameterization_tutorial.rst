Time Parameterization
==============================

MoveIt! is currently primarily a *kinematic* motion planning framework - it plans for joint or end effector positions but not velocity or acceleration. However, MoveIt! does utilize *post*-processing to time parameterize kinematic trajectories for velocity and acceleration values. Below we explain the settings and components involved in this part of MoveIt!.

Speed Control
-------------

From File
^^^^^^^^^
By default MoveIt! sets the velocity and acceleration of a joint trajectory to the default allowed in the robot's URDF or ``joint_limits.yaml``. The ``joint_limits.yaml`` is generated from the Setup Assistant and is initially an exact copy of the values within the URDF. The user can then modify those values to be less than the original URDF values if special constraints are needed. Specific joint properties can be changed with the keys ``max_position, min_position, max_velocity, max_acceleration``. Joint limits can be turned on or off with the keys ``has_velocity_limits, has_acceleration_limits``.

During Runtime
^^^^^^^^^^^^^^
The speed of a parameterized kinematic trajectory can also be modified during runtime as a fraction of the max velocity and acceleration set in the configuration values, as a value between 0-1. To change the speed on a per-motion plan basis, you can set the two scaling factors as described in `MotionPlanRequest.msg <http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/MotionPlanRequest.html>`_. Spinboxes for setting both of those factors are also available in the `MoveIt! MotionPlanning RViz plugin <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_.

Time Parameterization Algorithms
--------------------------------
MoveIt! can support different algorithms for post-processing a kinematic trajectory to add timestamps and velocity/acceleration values. Currently two are available by default in MoveIt!: `Iterative Parabolic Time Parameterization <https://github.com/ros-planning/moveit/blob/kinetic-devel/moveit_core/trajectory_processing/src/iterative_time_parameterization.cpp>`_, and `Iterative Spline Parameterization <https://github.com/ros-planning/moveit/blob/kinetic-devel/moveit_core/trajectory_processing/src/iterative_spline_parameterization.cpp>`_.

The Iterative Parabolic Time Parameterization algorithm is used by default in the `Motion Planning Pipeline <../motion_planning_pipeline/motion_planning_pipeline_tutorial.html>`_ as a Planning Request Adapter as documented in `this tutorial <../motion_planning_pipeline/motion_planning_pipeline_tutorial.html#using-a-planning-request-adapter>`_. Although the Iterative Parabolic Time Parameterization algorithm MoveIt! uses has been used by hundreds of robots over the years, there is known `bug with it <https://github.com/ros-planning/moveit/issues/160>`_.

The Iterative Spline Parameterization algorithm was recently merged into mainstream MoveIt! in `PR 382 <https://github.com/ros-planning/moveit/pull/382>`_ to deal with these issues. While preliminary experiments are very promising, we are waiting for more feedback from the community before replacing the Iterative Parabolic Time Parameterization algorithm completely.