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

**Note:** Currently you must build MoveIt and MoveIt Messages from source for this tutorial to function.

Install From Source
^^^^^^^^^^^^^^^^^^^
Clone MoveIt Task Constructor into you catkin workspace: ::

  git clone https://github.com/ros-planning/moveit_task_constructor.git

Use the rosdep tool to automaticall install its dependencies: ::

  rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO

Build the workspace: ::

  catkin build

Install From Debian
^^^^^^^^^^^^^^^^^^^

**Note:** This package has not been released as of 7/16/19::

  sudo apt-get install ros-$ROS_DISTRO-moveit-task-constructor

Running the Code
----------------
In a shell start RViz and the demo with the following command: ::

  roslaunch moveit_task_constructor_demo demo.launch

.. image:: add_mtc_panel.png
   :width: 700px

In Rviz click the add button and select the Motion Planning Tasks. When the plans are all made RViz will show you those 10 plans in the panel.
If you click on a stage, or the full plan it will show you the visualization.

.. image:: mtc_show_stages.gif
   :width: 700px

How MoveIt Task Constructor Works
---------------------------------

Tasks
^^^^^
A task is a specified complex planning problem that consists of a sequence of high level steps called stages.

Stages
^^^^^^
A stage is a low-level implementation of a high-level planning step. The individual stages compute subsolutions that can **generate**,
**propogate**, or **connect** `InterfaceStates`. They also can combine or edit contained substages.

InterfaceState
^^^^^^^^^^^^^^
Stages interface each other via the InterfaceState class, which is a snapshot of the planning scene, the robot state, and some named properties.

Stage Types
^^^^^^^^^^^
There are three basic types of stages: In addition to **generator**, **propagator**, and **connector** stages,
there exist container stages (**wrapper**, **serial**, and **parallel** container), which allow to structure the task pipeline in a hierarchical fashion.

.. image:: mtc_stage_types.png
   :width: 700px

**Generator:** This is a stage that generates information that is passed to the stages above and below.

**Propogator:** This stage takes information from one or both sides, modifies it, and passes it along to the opposite side.

**Connector:** This stage takes information from both sides and plans a connecting trajectory between the robot states of both sides.

**Wrapper:** Wraps a single subordinate stage and modifies its output.

**Serial Container:** Allows to define a sequence of subordinate task.

**Parallel Container:** Allows to run several stages in parallel before choosing which substage to use the results from.

The Entire Code
---------------
The entire code can be seen in the moveit_task_constructor_ repository.

.. _moveit_task_constructor: https://github.com/ros-planning/moveit_task_constructor

**Note:** If the added task monitor does not find the /solutions topic replace `""` with `"/moveit_task_constructor_demo/pick_place_task/solution"` in line 82 of `moviet_task_constructor/visualization/motion_planning_tasks/src/task_display.cpp`.
