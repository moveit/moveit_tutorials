MoveIt Task Constructor
=======================

.. image:: mtc_example.png
   :width: 700px

The Task Constructor framework provides a flexible and transparent way to define and plan actions that consist of multiple interdependent subtasks. It draws on the planning capabilities of MoveIt to solve individual subproblems in black-box planning stages. A common interface, based on MoveIt's PlanningScene is used to pass solution hypotheses between stages. The framework enables the hierarchical organization of basic stages using containers, allowing for sequential as well as parallel compositions. For more details, please refer to the associated `ICRA 2019 publication`_.

.. _ICRA 2019 publication: https://pub.uni-bielefeld.de/download/2918864/2933599/paper.pdf

Getting Started
---------------

If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

Installing MoveIt Task Constructor
----------------------------------

**Note:** Currently, it's required to build `moveit` and `moveit_msgs` from source for this tutorial to function.

Install From Source
^^^^^^^^^^^^^^^^^^^

Go into your catkin workspace and initialize wstool if necessary (assuming `~/catkin_ws` as workspace path): ::

  cd ~/catkin_ws/src
  wstool init

Clone MoveIt Task Constructor and source dependencies: ::

  git clone https://github.com/ros-planning/moveit_task_constructor.git
  wstool merge moveit_task_constructor/moveit_task_constructor.rosinstall
  wstool update

Install missing packages with rosdep: ::

  rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO

Build the workspace: ::

  catkin build


Running the Demo
----------------

The MoveIt Task Constructor package contains an example pick-and-place application.
You can try it out by running: ::

  roslaunch moveit_task_constructor_demo demo.launch

.. image:: add_mtc_panel.png
   :width: 700px

On the right side you should see the `Motion Planning Tasks` outlining the stages of the **pick_place_task**.
When you select the task or a single stage the right colum lists the individual subsolutions.
You can simply click on a subsolution to visualize a planned trajectory of the selected segment.

.. image:: mtc_show_stages.gif
   :width: 700px

Basic Concepts
--------------

The fundamental idea of MTC is that complex motion planning problems can be composed into a set of simpler subproblems.
The top-level planning problem is specified as a **Task** while all subproblems are specified by **Stages**.

Stages
^^^^^^

Primitive **Stages** are low-level implementations of actual problem solvers.
There are three types of stages: generator, propagator, and connecting stages, named after the direction their results are passed.

**Generator** stages compute their resulting state independent of their neighbor stages and pass the result backwards and forwards.
An example is an IK sampler for geometric poses where approaching and departing motions depend on the solution.

**Propagator** stages use the result of one neighbor stage for solving a problem and then propagate the result on to the neighbor on the opposite site.
Depending on the implementation propagating stages can pass solutions forward, backward or in both directions separately.
An example is a stage that computes a Cartesian path based on either a start or a goal state.

**Connecting** stages are solvers that don’t pass any results to their neighbors, but rather attempt to bridge the gap between the resulting states of both neighbor stages.
A typical example is to compute a free motion plan from one state to another.

Container stages are higher-level solvers where the solution is based on one or multiple subordinate stages.
For instance, container stages allow combining a set of stages to selecting one desired result or to merge all solutions if the subproblems are independent.
There are three hierarchy types:

**Wrapper:** Wraps a single subordinate stage and modifies or filters its results.

**Serial Container:** Contains a sequence of subordinate stages and only passes end-to-end results.

**Parallel Container:** Contains a set of subordinate stages and can be used for passing the best of alternative results, running fallback solvers or to merge multiple independent solutions.

.. image:: mtc_stage_types.png
   :width: 700px

Next to motion planning problems, stages can also be used for all kinds of state transitions, as for instance modifying the planning scene.
Finally, stages support class inheritance which is helpful for specifying a more generic problem (i.e. adding a constraint to a motion plan).
