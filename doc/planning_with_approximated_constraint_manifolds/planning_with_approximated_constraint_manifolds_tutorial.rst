Planning with Approximated Constraint Manifolds
===============================================

OMPL supports custom constraints to enable planning trajectories that follow a desired behavior.
Constraints can be defined in joint space and Cartesian space where the latter is either orientation or position based.
While planning a trajectory each joint state needs to follow all of the set constraints, which is performed by rejection sampling by default.
This however might lead to very long planning times, especially when the constraints are very restrictive and the rejection rate is correspondingly high.

`Sucan et al <http://ioan.sucan.ro/files/pubs/constraints_iros2012.pdf>`_ present an approach where they compute an approximation of the constraint manifold beforehand and perform trajectory planning in that.
The OMPL plugin contains the functionality to do that for a given set of constraints and save it in a database.
In later instances the database can be loaded to use for constrained planning with any OMPL planner which strongly reduces planning time.

This tutorial includes the steps on how to construct the constraint approximation database and on how to use it for constrained trajectory planning.
For more information on how to plan with path constraints in general, take a look at `here <../move_group_interface/move_group_interface_tutorial.html#planning-with-path-constraints>`_.

Creating the Constraint Database
--------------------------------

A sample implementation on how to construct an approximation database from a constraint can be found inside ``demo_construct_state_database.cpp``.

The main functionality is implemented in the `ConstraintsLibrary <http://docs.ros.org/melodic/api/moveit_planners_ompl/html/classompl__interface_1_1ConstraintsLibrary.html>`_ class.

Constraints are added by calling ``addConstraintApproximation()`` which can be called subsequently to include multiple constraints in the approximation.
The function requires four parameters:

* constraints message (moveit_msgs::Constraints)
* robot description (std::string)
* planning scene (planning_scene::PlanningScenePtr)
* construction options (ompl_interface::ConstraintApproximationConstructionOptions)

The robot description is the name of the move group and the planning scene should be initialized as usual with the corresponding robot model.

Initialization of the constraints message and the options is explained below.

Constraints message
^^^^^^^^^^^^^^^^^^^

The constraints message object can be initialized as usual with any type and required tolerances.
The critical point is that the ``name`` of the message should be descriptive and unique to the constraint.
That ``name`` is used later on to reference the correct constraint when planning with the approximation database.

Construction Options
^^^^^^^^^^^^^^^^^^^^

The ``ompl_interface::ConstraintApproximationConstructionOptions`` object specifies various features of the approximation manifold, as for instance size, density, space parameterization type and others.
Below is an overview over each of the options:

* **unsigned int** samples - size of the approximation graph
* **unsigned int** edges_per_sample - degree of the approximation graph
* **double** max_edge_length - distance threshold for edge insertion
* **bool** explicit_motions - defines if edges should follow constraints
* **double** explicit_points_resolution - interpolation resolution of edges for constraint checks
* **unsigned int** max_explicit_points - maximum points of an edge to check

Graph size
""""""""""

Obviously stable planning results require a detailed approximation, thus the higher the **samples** is the more reliable the performance.
However higher values lead to linearly longer construction time for the database.
Finding an appropriate size of the manifold is a problem that is highly dependent on how restrictive the constraints are.
For most constraints it should suffice to use values in range of 1000 to 10000 with no noticeable improvements with higher values as suggested in the paper.

Edges
"""""

Adding edges to the manifold is **optional** and can be disabled by setting **edges_per_sample** to 0.
Trajectory planning will work without edges in most cases just fine since the sampling process only needs the states to function.
**max_edge_length** defines the maximum distance of two states that allows an edge between them to be added.
By setting **explicit_motions** to *true* the edges are also enforced to match the constraints, making them represent valid paths between adjacent states.
That is advantageous especially in approximations that are very sparse with many regions that are hard to reach since absolute distance is not necessary a measure for reachability.
The check if an edge matches a constraint is done by testing linearly interpolated points between the state pair.
The number of these interpolated points is set to **explicit_points_resolution** times the edge length and is limited by **max_explicit_points**.

Adding edges increases the construction time of the database tremendously whereas increasing adding explicit motion checks even has an additional impact on that.
When experimenting with edges, keep in mind that the **edges_per_sample** and **max_edge_length** values should be adjusted so that there are always just enough states close enough to be connected.
That requires analysis of the size of the approximation space in terms of density and adapting to the actual distance between the states.

Database Construction
^^^^^^^^^^^^^^^^^^^^^
After adding the constraints to the ConstraintsLibrary object the database can be constructed by calling ``saveApproximationConstraints()`` which only takes the relative directory in which the database should be saved.


Database Loading and Usage
--------------------------
The constraints database must be loaded at launch of the move group node by setting the ros parameter::

 <param name="move_group/constraint_approximations_path" value="<path_to_database>"/>

To verify if the database was found an successfully loaded check if the named constraint is shown in the log.

For planning just initialize the constraints message as always and set the messages name to the exact name that was used to construct the database.
Also you need to specify the same values and tolerances again since by default the planner just samples over the states but does not necessary follow the constraints during interpolation for path planning.
A correctly named constraint message without initialized constraints would use the database but can therefore lead to invalid trajectories anyway.
