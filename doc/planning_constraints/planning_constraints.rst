Representation and Evaluation of Constraints
============================================

Constraints are an integral component of MoveIt and OMPL. They are used both to constrain robot motion as well as to define planning goals. There following set of constraints are defined in the :code:`kinematic_constraints` namespace:

- kinematic_constraints::JointConstraint
- kinematic_constraints::OrientationConstraint
- kinematic_constraints::PositionConstraint
- kinematic_constraints::VisibilityConstraint

All of these constraints inherit from the :code:`kinematic_constraints::KinematicConstraint` base class and thus more constraint types can be added by the user by providing their own derived classes. The main operation each constraint implements is the :code:`KinematicConstraint::decide()` function, which decides whether a constraint is satisfied, and optionally returns a distance (an error) when a constraint is not satisfied.

Often multiple constraints need to be imposed on a particular motion plan or for a particular goal. The class :code:`kinematic_constraints::KinematicConstraintSet` facilitates operating with sets of constraints.

A related functionality to representing and evaluating constraints is :code:`constraint_sampling` "generating samples" that satisfy those constraints.


.. figure:: fingertip_collision.png
   :width: 400px

   An example of a Visibility Constraint, whereby the motion of the arm is planned to avoid obscuring the workpiece from the cameras.

See Also
--------
- `How to plan with path constraints in general <../move_group_interface/move_group_interface_tutorial.html#planning-with-path-constraints>`_.
- `Planning with Approximated Constraint Manifolds <../planning_with_approximated_constraint_manifolds/planning_with_approximated_constraint_manifolds_tutorial.html>`_.
