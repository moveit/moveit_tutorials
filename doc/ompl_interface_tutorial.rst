OMPL Interface Tutorial
=======================

The Open Motion Planning Library is a powerful collection of state-of-the-art sampling-based motion planning algorithms and is the default planner in MoveIt!. For more information see `project webpage <http://ompl.kavrakilab.org/>`_.

OMPL Settings
-------------

Here we review important configuration settings for OMPL. These settings can typically be found in the ``ompl_planning.yaml`` file located in your robots ``moveit_config`` package.

Longest Valid Segment Fraction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``longest_valid_segement_fraction`` defines the discretization of robot motions used for collision checking, and greatly affects the performance and reliability of OMPL-based solutions. A ``motion`` in this context can be thought of as an edge between two nodes in a graph, where nodes are discretized robot states that randomly sampled and collision checked for validity. The default motion collision checker in OMPL simply discretizes the edge into a number of sub-states to collision check. Currently no continuous collision checking is available in OMPL/MoveIt!, though this is an area of current `discussion <https://github.com/ros-planning/moveit/issues/29>`_.

Set ``longest_valid_segement_fraction`` too low, and collision checking / motion planning will be very slow. Set too high and collisions will be missed around small or narrow objects. In addition, a high collision checking resolution will cause the path smoothers to output incomprehensible motions because they are able to "catch" the invalid path and then attempt to repair them by sampling around it, but imperfectly.

A quick analysis of the effect of this parameter on two of the MoveIt! tutorial examples is documented `here <https://github.com/ros-planning/moveit/pull/337>`_.

Projection Evaluator
^^^^^^^^^^^^^^^^^^^^

The ``projection_evaluator`` can take in a list of joints or links to approximate the coverage of a configuration space. This settings is used by planners such as KPIECE, BKPIECE, LBKPIECE, and PDST. For more information read the corresponding publications.

Other Settings
^^^^^^^^^^^^^^

Depending on the planner you are using, other settings are available for tuning/parameter sweeping. The default values for these settings are auto-generated in the MoveIt! Setup Assistant and are listed in the ``ompl_planning.yaml`` file - you are encouraged to tweak them.

OMPL Optimization Objectives
----------------------------

Several planners that are part of the OMPL planning library are capable of optimizing for a specified optimization objective. This tutorial describes that steps that are needed to configure these objectives. The optimal planners that are currently exposed to MoveIt! are:

* geometric::RRTstar
* geometric::PRMstar

And the following optimization objectives are available:

* PathLengthOptimizationObjective (Default)
* MechanicalWorkOptimizationObjective
* MaximizeMinClearanceObjective
* StateCostIntegralObjective
* MinimaxObjective

The configuration of these optimization objectives can be done in the *ompl_planning.yaml*. A parameter with the name **optimization_objective** is added as a configuration parameter. The value of the parameter is set to be the name of the selected optimization objective. For example, to configure RRTstar to use the *MaximizeMinClearanceObjective*, the planner entry in the ompl_planning.yaml will look like::

	RRTstarkConfigDefault:
	    type: geometric::RRTstar
	    optimization_objective: MaximizeMinClearanceObjective
	    range: 0.0
	    goal_bias: 0.05
	    delay_collision_checking: 1

For more information on the OMPL optimal planners, the reader is referred to the
`OMPL - Optimal Planning documentation <http://ompl.kavrakilab.org/optimalPlanning.html>`_.
