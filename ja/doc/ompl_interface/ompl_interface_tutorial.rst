OMPL Planner
============

The Open Motion Planning Library is a powerful collection of state-of-the-art sampling-based motion planning algorithms and is the default planner in MoveIt. For more information see `project webpage <http://ompl.kavrakilab.org/>`_.

OMPL Settings
-------------

Here we review important configuration settings for OMPL. These settings can typically be found in the ``ompl_planning.yaml`` file located in your robots ``moveit_config`` package.

Longest Valid Segment Fraction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``longest_valid_segment_fraction`` defines the discretization of robot motions used for collision checking and greatly affects the performance and reliability of OMPL-based solutions. A ``motion`` in this context can be thought of as an edge between two nodes in a graph, where nodes are waypoints along a trajectory. The default motion collision checker in OMPL simply discretizes the edge into a number of sub-states to collision check. No continuous collision checking is currently available in OMPL/MoveIt, though this is an area of current `discussion <https://github.com/ros-planning/moveit/issues/29>`_.

Specifically, ``longest_valid_segment_fraction`` is the fraction of the robot's state space that, given the robot isn't currently in collision, we assume the robot can travel while remaining collision free. For example, if ``longest_valid_segment_fraction = 0.01``, then we assume that if an edge between two nodes is less than 1/100th of the state space, then we don't need to explicity check any sub-states along that edge, just the two nodes it connects.

In addition to the ``longest_valid_segment_fraction`` parameter in the ``ompl_planning.yaml`` file, there is also the ``maximum_waypoint_distance``, found in the :moveit_codedir:`dynamic reconfigure file <moveit_planners/ompl/ompl_interface/cfg/OMPLDynamicReconfigure.cfg#L9>`. ``maximum_waypoint_distance`` defines the same discretization of robot motions for collision checking, but it does so at an absolute level instead of using fractions of the state space. For example, if ``maximum_waypoint_distance = 0.1``, then if an edge is shorter than ``0.1`` in state space distance, then we don't explicitly check any sub-states along that edge.

If both ``longest_valid_segment_fraction`` and ``maximum_waypoint_distance`` are set, then the variable that produces the most conservative discretization (the one that would generate the most states to collision check on a given edge) is chosen.

Set ``longest_valid_segment_fraction`` (or ``maximum_waypoint_distance``) too low, and collision checking / motion planning will be very slow. Set too high and collisions will be missed around small or narrow objects. In addition, a high collision checking resolution will cause the path smoothers to output incomprehensible motions because they are able to "catch" the invalid path and then attempt to repair them by sampling around it, but imperfectly.

A quick analysis of the effect of this parameter on two of the MoveIt tutorial examples is documented `here <https://github.com/ros-planning/moveit/pull/337>`_.

Projection Evaluator
^^^^^^^^^^^^^^^^^^^^

The ``projection_evaluator`` can take in a list of joints or links to approximate the coverage of a configuration space. This settings is used by planners such as KPIECE, BKPIECE, LBKPIECE, and PDST. For more information read the corresponding publications.

Enforce Planning in Joint Space
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Depending on the planning problem MoveIt chooses between ``joint space`` and ``cartesian space`` for problem representation.
Setting the group parameter ``enforce_joint_model_state_space`` enforces the use of ``joint space`` for all plans.

By default planning requests with orientation path constraints are sampled in ``cartesian space`` so that invoking IK serves as a generative sampler.

By enforcing ``joint space`` the planning process will use rejection sampling to find valid requests.
Please not that this might increase planning time considerably.

Other Settings
^^^^^^^^^^^^^^

Depending on the planner you are using, other settings are available for tuning/parameter sweeping. The default values for these settings are auto-generated in the MoveIt Setup Assistant and are listed in the ``ompl_planning.yaml`` file - you are encouraged to tweak them.

Smoothing Paths
---------------

Here we discuss various approaches to improve common issues of indirect, non-optimized paths in OMPL. We encourage core developers and researchers to help improve the OMPL and MoveIt code bases so that this becomes less of a common issue and more of a solved problem :-)

Determinism
^^^^^^^^^^^

The planners in OMPL are inherently probablistic and will not always return the same solution. Other libraries such as the Search Based Planning Library (SBPL) provide deterministic results in that given the same environment, start, and goal you will always get the same path. SBPL is Astar-based, so you will get optimal results up to your chosen search resolution. However SBPL has plenty of downsides, such as the difficulty of tuning special hueristics.

OMPL Optimization Objectives
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Several planners that are part of the OMPL planning library are capable of optimizing for a specified optimization objective. This tutorial describes that steps that are needed to configure these objectives. The asymptotically optimal planners that are currently exposed to MoveIt are:

* RRT*
* PRM*

Other optimal planners in OMPL but not exposed in MoveIt yet:

* LazyPRM*
* RRT#
* RRTX
* Informed RRT*
* Batch Informed Trees (BIT*)
* Lower Bound Tree RRT (LBTRRT)
* Sparse Stable RRT
* Transition-based RRT (T-RRT)
* SPARS
* SPARS2
* FMT*
* CForest

And the following optimization objectives are available:

* PathLengthOptimizationObjective (Default)
* MechanicalWorkOptimizationObjective
* MaximizeMinClearanceObjective
* StateCostIntegralObjective
* MinimaxObjective

The configuration of these optimization objectives can be done in the *ompl_planning.yaml*. A parameter with the name **optimization_objective** is added as a configuration parameter. The value of the parameter is set to be the name of the selected optimization objective. For example, to configure RRTstar to use the *MaximizeMinClearanceObjective*, the planner entry in the ompl_planning.yaml will look like: ::

	RRTstarkConfigDefault:
	    type: geometric::RRTstar
	    optimization_objective: MaximizeMinClearanceObjective
	    range: 0.0
	    goal_bias: 0.05
	    delay_collision_checking: 1

For more information on the OMPL optimal planners, the reader is referred to the
`OMPL - Optimal Planning documentation <http://ompl.kavrakilab.org/optimalPlanning.html>`_.

Post-Processing Smoothing
^^^^^^^^^^^^^^^^^^^^^^^^^

Note there is a limit to how much smoothing can help reduce indirect routes. Note also that here we discuss geometric(kinematic)-based only smoothing. Velocity/acceleration/jerk smoothing is handled elsewhere, see `Time Parameterization <../time_parameterization/time_parameterization_tutorial.html>`_.

You can adjust the amount of time MoveIt spends on smoothing by increasing the planning time. Any remaining time after an initial plan is found, but before the ``allowed_planning_time`` is exhausted, will be used for smoothing. MoveIt also does path hybridization, taking the best parts of N different planning runs and splicing them together. Therefore, ``num_planning_attempts`` affects the quality as well.

Although not currently exposed at the top levels of MoveIt (TODO), more smoothing can be accomplished by setting the simplification duration to 0 (unlimited) in ``model_based_planning_context.cpp``. This will enable OMPL's ``simplifyMax()`` function.

Besides the internal OMPL smoothers, recent efforts have been made to do post-proccessing with STOMP/CHOMP. See `this blog post <http://moveit.ros.org/moveit!/ros/2018/10/25/gsoc-motion-planning-support.html>`_.
