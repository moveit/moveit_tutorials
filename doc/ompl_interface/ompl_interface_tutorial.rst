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

Trade-offs in speed and optimality
----------------------------------

Many planners in OMPL (including the default one) favor speed of finding a solution path over path quality. A feasible path is smoothened and shortened in a post-processing stage to obtain a path that is closer to optimal. However, there is no guarantee that a global optimum is found or that the same solution is found each time since the algorithms in OMPL are probabilistic. Other libraries such as the Search Based Planning Library (SBPL) provide deterministic results in that given the same environment, start, and goal you will always get the same path. SBPL is A*-based, so you will get optimal results up to your chosen search resolution. However, SBPL has downsides as well, such as the difficulty of defining a state space lattice at an appropriate resolution (e.g., how do you define a good discretization of joint angles or end effector poses?) and tuning special heuristics.

There are several planners in OMPL that *can* give theoretical optimality guarantees, but often only asymptotically: they converge to an optimal solution, but convergence can be slow. The optimization objective used by these planners is typically the minimization of path length, but other optimization objectives can be used as well.

OMPL Optimization Objectives
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Several planners that are part of the OMPL planning library are capable of optimizing for a specified optimization objective. This tutorial describes that steps that are needed to configure these objectives. The asymptotically (near-)optimal planners that are currently exposed to MoveIt are:

* RRT*
* PRM*
* LazyPRM*
* BFMT
* FMT
* Lower Bound Tree RRT (LBTRRT)
* SPARS
* SPARS2
* Transition-based RRT (T-RRT)

OMPL also provides a meta-optimization algorithm called AnytimePathShortening, which repeatedly runs several planners in parallel interleaved with path shortcutting and path hybridization, two techniques that locally optimize a solution path. Although not *proven* optimal, it is often an effective strategy in practice to obtaining near-optimal solution paths.

Other optimal planners in OMPL but not exposed in MoveIt yet:

* RRT#
* RRTX
* Informed RRT*
* Batch Informed Trees (BIT*)
* Sparse Stable RRT
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

Other optimization objectives can be defined programmatically. For more information on the OMPL optimal planners, the reader is referred to the `OMPL - Optimal Planning documentation <http://ompl.kavrakilab.org/optimalPlanning.html>`_.

OMPL Planner Termination Conditions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The planners in OMPL typically terminate when a given time limit has been exceeded. However, it possible to specify an additional planner termination condition per planner
configuration in ompl_planning.yaml via the ``termination_condition`` parameter. Possible values are:

* ``Iteration[num]``: Terminate after ``num`` iterations. Here, ``num`` should be replaced with a positive integer.
* ``CostConvergence[solutionsWindow,epsilon]``: Terminate after the cost (as specified by an optimization objective) has converged. The parameter ``solutionsWindow`` specifies the minimum number of solutions to use in deciding whether a planner has converged. The parameter ``epsilon`` is the threshold to consider for convergence. This should be a positive number close to 0. If the cumulative moving average does not change by a relative fraction of epsilon after a new better solution is found, convergence has been reached. *This termination condition is only available in OMPL 1.5.0 and newer.*
* ``ExactSolution``: Terminate as soon as an exact solution is found or a timeout occurs. This modifies the behavior of anytime/optimizing planners to terminate upon discovering the first feasible solution.

In all cases, the planner will terminate when either the user-specified termination condition is satisfied or the time limit specified by ``timeout`` has been reached, whichever occurs first.

For example, to specify that RRTstar should terminate upon convergence, the following settings could be used: ::

	RRTstarkConfigDefault:
	    type: geometric::RRTstar
	    termination_condition: CostConvergence[10,.1]
	    range: 0.0
	    goal_bias: 0.05
	    delay_collision_checking: 1

Note that no optimization objective is specified, so the default one, PathLengthOptimizationObjective, will be used.

Post-Processing Smoothing
^^^^^^^^^^^^^^^^^^^^^^^^^

Note there is a limit to how much smoothing can help reduce indirect routes. Note also that here we discuss geometric(kinematic)-based only smoothing. Velocity/acceleration/jerk smoothing is handled elsewhere, see `Time Parameterization <../time_parameterization/time_parameterization_tutorial.html>`_.

You can adjust the amount of time MoveIt spends on smoothing by increasing the planning time. Any remaining time after an initial plan is found, but before the ``allowed_planning_time`` is exhausted, will be used for smoothing. MoveIt also does path hybridization, taking the best parts of N different planning runs and splicing them together. Therefore, ``num_planning_attempts`` affects the quality as well.

Although not currently exposed at the top levels of MoveIt (TODO), more smoothing can be accomplished by setting the simplification duration to 0 (unlimited) in ``model_based_planning_context.cpp``. This will enable OMPL's ``simplifyMax()`` function.

Besides the internal OMPL smoothers, recent efforts have been made to do post-proccessing with STOMP/CHOMP. See `this blog post <http://moveit.ros.org/moveit!/ros/2018/10/25/gsoc-motion-planning-support.html>`_.

Persistent Roadmaps
-------------------

By default the planning algorithms start from scratch for each motion planning request. However, for certain planners that build a roadmap of the environment, it may be beneficial to reuse the roadmap from previous motion planning requests if the planning scene is more or less static. Consider the following planning configurations: ::

	PersistentLazyPRMstar: # use this with a representative environment to create a roadmap
	    type: geometric::LazyPRMstar
	    multi_query_planning_enabled: true
	    store_planner_data: true
	    load_planner_data: false
	    planner_data_path: /tmp/roadmap.graph
	PersistentLazyPRM: # use this to load a previously created roadmap
	    type: geometric::LazyPRM
	    multi_query_planning_enabled: true
	    store_planner_data: false
	    load_planner_data: true
	    planner_data_path: /tmp/roadmap.graph
	SemiPersistentLazyPRMstar: # reuses roadmap during lifetime of node but doesn't save/load roadmap to/from disk
	    type: geometric::LazyPRMstar
	    multi_query_planning_enabled: true
	    store_planner_data: false
	    load_planner_data: false
	SemiPersistentLazyPRM: # reuses roadmap during lifetime of node but doesn't save/load roadmap to/from disk
	    type: geometric::LazyPRM
	    multi_query_planning_enabled: true
	    store_planner_data: false
	    load_planner_data: false

The first planner configuration, `PersistentLazyPRMstar`, will use LazyPRM* to keep growing a roadmap of asymptotically optimal paths between sampled robot configurations with each motion planning request. Upon destruction of the planner instance, it will save the roadmap to disk. The `PersistentLazyPRM` configuration is similar, except it will *load* a roadmap from disk but not *save* it upon destruction. The `SemiPersistent` planner configurations do not load/save roadmaps, but do keep extending a roadmap with each motion planning request (rather than the default behavior of clearing it before planning). The four planners that support the persistent planning features are: PRM, PRM*, LazyPRM, and LazyPRM*. The critical difference between them is that the lazy variants will re-validate the validity of nodes and edges as needed when searching the roadmap for a valid path. The non-lazy variants will not check if the roadmap is still valid for the current environment. In other words, use the non-lazy variants for static environments, the lazy variants for environments with small changes, and a non-persistent planner if the environment can change significantly.

*Note that saving and loading roadmaps is only available in OMPL 1.5.0 and newer.*
