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
A stage is a low-level implementation of a high-level plannign step. The indevidual stages compute subSolutions that can **generate**,
**propogate**, or **connect** interfacesStates. They also can combine or edit contained substages.

InterfaceState
^^^^^^^^^^^^^^
This is the connector between stages. It is a snapshot of the planning scene, robot state and properties that gives that information from one stage to the other for planning.

Stage Types
^^^^^^^^^^^
There are three basic types of stages. In addition to **generator**, **propogator**, or **connector**, there are also **wrapper**, **serial container**, and **parallel container** stages which are ways to simplify one or more stages to one contained stage.

.. image:: mtc_stage_types.png
   :width: 700px

**Generator:** This is a stage that generates information which is passed to the stage above and the stage bellow.

**Propogator:** This is a stage that takes information from one or both sides, modifies it and passes it along to the other side.

**Connector:** This stage takes information from both sides and plans a connection from one to the other.

**Wrapper:** This contains a single subordinate stage and modifies the output.

**Serial Container:** The stage wraps several sequential stages into one larger more complicated set of actions.

**Parallel Container:** This stage runs several stages in parallel before choosing which substage to use the results from.

The Entire Code
---------------
The entire code can be seen in the moveit_task_constructor_ repository.

.. _moveit_task_constructor: https://github.com/ros-planning/moveit_task_constructor

**Note:** If the added task monitor does not find the /solutions topic replace `""` with `"/moveit_task_constructor_demo/pick_place_task/solution"` in line 82 of `moviet_task_constructor/visualization/motion_planning_tasks/src/task_display.cpp`.
