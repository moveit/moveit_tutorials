Time Parameterization
==============================

MoveIt is currently primarily a *kinematic* motion planning framework - it plans for joint or end effector positions but not velocity or acceleration. However, MoveIt does utilize *post*-processing to time parameterize kinematic trajectories for velocity and acceleration values. Below we explain the settings and components involved in this part of MoveIt.

Speed Control
-------------

From File
^^^^^^^^^
By default MoveIt sets the velocity and acceleration of a joint trajectory to the default allowed in the robot's URDF or ``joint_limits.yaml``. The ``joint_limits.yaml`` is generated from the Setup Assistant and is initially an exact copy of the values within the URDF. The user can then modify those values to be less than the original URDF values if special constraints are needed. Specific joint properties can be changed with the keys ``max_position, min_position, max_velocity, max_acceleration``. Joint limits can be turned on or off with the keys ``has_velocity_limits, has_acceleration_limits``.

During Runtime
^^^^^^^^^^^^^^
The speed of a parameterized kinematic trajectory can also be modified during runtime as a fraction of the max velocity and acceleration set in the configuration values, as a value between 0-1. To change the speed on a per-motion plan basis, you can set the two scaling factors as described in :rosdocs:`MotionPlanRequest.msg </moveit_msgs/html/msg/MotionPlanRequest.html>`. Spinboxes for setting both of those factors are also available in the `MoveIt MotionPlanning RViz plugin <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_.

Time Parameterization Algorithms
--------------------------------
MoveIt can support different algorithms for post-processing a kinematic trajectory to add timestamps and velocity/acceleration values. Currently there are three available by default in MoveIt:

* :moveit_codedir:`Iterative Parabolic Time Parameterization<moveit_core/trajectory_processing/src/iterative_time_parameterization.cpp>`
* :moveit_codedir:`Iterative Spline Parameterization<moveit_core/trajectory_processing/src/iterative_spline_parameterization.cpp>`
* :moveit_codedir:`Time-optimal Trajectory Generation<moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp>`

The Iterative Parabolic Time Parameterization algorithm is used by default in the `Motion Planning Pipeline <../motion_planning_pipeline/motion_planning_pipeline_tutorial.html>`_ as a Planning Request Adapter as documented in `this tutorial <../motion_planning_pipeline/motion_planning_pipeline_tutorial.html#using-a-planning-request-adapter>`_. Although the Iterative Parabolic Time Parameterization algorithm MoveIt uses has been used by hundreds of robots over the years, there is known `bug with it <https://github.com/moveit/moveit/issues/160>`_.

The Iterative Spline Parameterization algorithm was merged with `PR 382 <https://github.com/moveit/moveit/pull/382>`_ as an approach to deal with these issues. While preliminary experiments are very promising, we are waiting for more feedback from the community before replacing the Iterative Parabolic Time Parameterization algorithm completely.

Time-optimal Trajectory Generation introduced in PRs `#809 <https://github.com/moveit/moveit/pull/809>`_ and `#1365 <https://github.com/moveit/moveit/pull/1365>`_ produces trajectories with very smooth and continuous velocity profiles. The method is based on fitting path segments to the original trajectory and then sampling new waypoints from the optimized path. This is different from strict time parameterization methods as resulting waypoints may divert from the original trajectory within a certain tolerance. As a consequence, additional collision checks might be required when using this method.

Jerk-Limited Trajectory Smoothing
---------------------------------
A time parameterization algorithm such as TOTG calculates velocities and accelerations for a trajectory, but none of the time parameterization algorithms support jerk limits. This is not ideal -- large jerks  along a trajectory can cause jerky motion or damage hardware. As a further post-processing step, the Ruckig jerk-limited smoothing algorithm can be appliied to limit joint jerks over the trajectories.

To apply the Ruckig smoothing algorithm, a jerk limit parameter should be defined for every joint. It can go in a launch file and the parameter name is ``ruckig/YOUR_JOINT_NAME/jerk_limit``. If you do not specify a jerk limit for any joint, a reasonable default will be applied and a warning printed.

Finally, add the Ruckig smoothing algorithm to the list of planning request adapters (typically in ``ompl_planning_pipeline.launch.xml`` or ``ompl_planning.yaml``). The Ruckig smoothing algorithm should run last, so put it at the top of list:

    <arg name="planning_adapters"
        default="default_planner_request_adapters/AddRuckigTrajectorySmoothing
                default_planner_request_adapters/AddTimeParameterization
                default_planner_request_adapters/ResolveConstraintFrames
