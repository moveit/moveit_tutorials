Planning Adapter Tutorials
==========================

Planning Request Adapters is a concept in MoveIt which can be used to modify the trajectory (pre-processing and/or post-processing) for a motion planner. Some examples of existing planning adapters in MoveIt include AddTimeParameterization, FixWorkspaceBounds, FixStartBounds, FixStartStateCollision, FixStartStatePathConstraints, CHOMPOptimizerAdapter, etc. ! Using the concepts of Planning Adapters, multiple motion planning algorithms can be used in a pipeline to produce robust motion plans. For example, a sample pipeline of motion plans might include an initial plan produced by OMPL which can then be optimized by CHOMP to produce a motion plan which would likely be better than a path produced by OMPL or CHOMP alone. Similarly, using the concept of Planning Adapters, other motion planners can be mixed and matched depending on the environment the robot is operating in. This section provides a step by step tutorial on using a mix and match of different motion planners and also provides insights on when to use which particular motion planners.

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

You should also have gone through the steps in `Visualization with MoveIt RViz Plugin <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_

Prerequisites
--------------
 1. You must have the latest version of MoveIt installed. On ROS Melodic you will need to build MoveIt from source. A build from source is required as CHOMP and STOMP are not a part of the official release yet. It is therefore not included in the binary packages. We will go through the steps for doing this below.
 2. To use Planning Adapters with your robot you must already have a MoveIt configuration package for your robot already. For example, if you have a Panda robot, it's probably called ``panda_moveit_config``. This is typically built using the `MoveIt Setup Assistant <../setup_assistant/setup_assistant_tutorial.html>`_.

Installing MoveIt from Source
------------------------------
As you add and remove packages from your workspace you will need to clean your workspace and re-run the command to install new missing dependencies. Clean your workspace to remove references to the system-wide installation of MoveIt: ::

  cd ~/ws_moveit/src
  catkin clean

Now follow the instructions on the MoveIt homepage for `installing MoveIt Melodic from source <http://moveit.ros.org/install/source/>`_. Note that you can skip the **Prerequisites** section since you should already have a Catkin workspace.

Re-source the setup files: ::

  source ~/ws_moveit/devel/setup.bash

Using Planning Request Adapter with Your Motion Planner
-------------------------------------------------------

In this section we provide different ways the user can mix and match different motion planners. Major focus of this tutorial is made on OMPL, CHOMP and STOMP as these are the only planners currently supported by MoveIt.

Running OMPL as a pre-processor for CHOMP
+++++++++++++++++++++++++++++++++++++++++

Here, it is demonstrated that CHOMP can be used as a post-processing optimization technique for plans obtained by other planning algorithms. The intuition behind this is that some randomized planning algorithm produces an initial guess for CHOMP. CHOMP then takes this initial guess and further optimizes the trajectory.
To achieve this, follow the steps:

#. Open the ``ompl_planning_pipeline.launch`` file in the ``<robot_moveit_config>/launch`` folder of your robot. For the Panda robot it is this `file <https://github.com/ros-planning/panda_moveit_config/blob/melodic-devel/launch/ompl_planning_pipeline.launch.xml>`_. Edit this launch file, find the lines where ``<arg name="planning_adapters">`` is mentioned and change it to: ::

    <arg name="planning_adapters"
         value="default_planner_request_adapters/AddTimeParameterization
		default_planner_request_adapters/FixWorkspaceBounds
                default_planner_request_adapters/FixStartStateBounds
                default_planner_request_adapters/FixStartStateCollision
                default_planner_request_adapters/FixStartStatePathConstraints
                chomp/OptimizerAdapter" />

#. The values of the ``planning_adapters`` is the order in which the mentioned adapters are called / invoked. Order here matters. Inside the CHOMP adapter, a :moveit_codedir:`call <moveit_planners/chomp/chomp_optimizer_adapter/src/chomp_optimizer_adapter.cpp#L169>` to OMPL is made before invoking the CHOMP optimization solver, so CHOMP takes the initial path computed by OMPL as the starting point to further optimize it. 

#. Find the line where ``<rosparam command="load" file="$(find panda_moveit_config)/config/ompl_planning.yaml"/>`` is mentioned and after this line, add the following: ::

    <rosparam command="load" file="$(find panda_moveit_config)/config/chomp_planning.yaml"/>

#. These additions will add a CHOMP Optimization adapter and load the corresponding CHOMP planner's parameters. To do this with your own robot replace ``panda_moveit_config`` to ``<my_robot>_moveit_config`` of your robot.

#. In the ``move_group.launch`` file of ``<robot_moveit_config>/launch`` folder for your robot, make sure that the default planner is ``ompl``.

#. In the ``chomp_planning.yaml`` file of ``<robot_moveit_config>/config`` folder for your robot, add the following line: ::

    trajectory_initialization_method: "fillTrajectory"

#. After making these requisite changes to the launch files, open a terminal and execute the following: ::

    roslaunch panda_moveit_config demo.launch pipeline:=chomp

This will launch RViz, select OMPL in the Motion Planning panel under the Context tab. Set the desired start and goal states by moving the end-effector around in the same way as was done for CHOMP above. Finally click on the Plan button to start planning. The planner will now first run OMPL, then run CHOMP on OMPL's output to produce an optimized path.

Running CHOMP as a post-processor for STOMP
+++++++++++++++++++++++++++++++++++++++++++

Now, it is demonstrated that CHOMP can be used as a post-processing optimization technique for plans obtained by STOMP. The intuition behind this is that STOMP produces an initial path for CHOMP. CHOMP then takes this initial path and further optimizes this trajectory.
To achieve this, follow the steps:

#. Open the ``stomp_planning_pipeline.launch`` file in the ``<robot_moveit_config>/launch`` folder of your robot. For the Panda robot it is `this <https://github.com/ros-planning/panda_moveit_config/blob/melodic-devel/launch/stomp_planning_pipeline.launch.xml>`_ file. Edit this launch file, find the lines where ``<arg name="planning_adapters">`` is mentioned and change it to: ::

    <arg name="planning_adapters" value="default_planner_request_adapters/AddTimeParameterization
                   default_planner_request_adapters/FixWorkspaceBounds
                   default_planner_request_adapters/FixStartStateBounds
                   default_planner_request_adapters/FixStartStateCollision
                   default_planner_request_adapters/FixStartStatePathConstraints
                   chomp/OptimizerAdapter" />

#. The values of the ``planning_adapters`` is the order in which the mentioned adapters are called / invoked. Order here matters. Inside the CHOMP adapter, a call to STOMP is made before invoking the CHOMP optimization solver, so CHOMP takes the initial path computed by STOMP as the starting point to further optimize it.

#. Find the line where ``<rosparam command="load" file="$(find panda_moveit_config)/config/stomp_planning.yaml"/>`` is mentioned and after this line, add the following: ::

    <rosparam command="load" file="$(find panda_moveit_config)/config/chomp_planning.yaml"/>

#. These additions will add a CHOMP Optimization adapter and load the corresponding CHOMP planner's parameters. To do this with your own robot replace ``panda_moveit_config`` to ``<my_robot>_moveit_config`` of your robot.

#. In the ``move_group.launch`` file of ``<robot_moveit_config>/launch`` folder for your robot, make sure that the default planner is ``stomp``.

#. In the ``chomp_planning.yaml`` file of ``<robot_moveit_config>/config`` folder for your robot, add the following line: ::

    trajectory_initialization_method: "fillTrajectory"

#. After making these requisite changes to the launch files, open a terminal and execute the following: ::

    roslaunch panda_moveit_config demo.launch

This will launch RViz, select STOMP in the Motion Planning panel under the Context tab. Set the desired start and goal states by moving the end-effector around. Finally click on the Plan button to start planning. The planner will now first run STOMP, then run CHOMP on STOMP's output to produce an optimized path.

Running OMPL as a pre-processor for STOMP
+++++++++++++++++++++++++++++++++++++++++

NOTE: The STOMP Smoothing Adapter is a work in progress.

Here, it is demonstrated that STOMP can be used as a post-processing smoothing technique for plans obtained by other planning algorithms. The intuition behind this is that some randomized planning algorithm produces an initial path for STOMP. STOMP then takes this initial path and further smoothens the trajectory.
To achieve this, follow the steps:

#. Open the ``ompl_planning_pipeline.launch`` file in the ``<robot_moveit_config>/launch`` folder of your robot. For the Panda robot it is this `file <https://github.com/ros-planning/panda_moveit_config/blob/melodic-devel/launch/ompl_planning_pipeline.launch.xml>`_. Edit this launch file, find the lines where ``<arg name="planning_adapters">`` is mentioned and change it to: ::

    <arg name="planning_adapters" value="default_planner_request_adapters/AddTimeParameterization
                   default_planner_request_adapters/FixWorkspaceBounds
                   default_planner_request_adapters/FixStartStateBounds
                   default_planner_request_adapters/FixStartStateCollision
                   default_planner_request_adapters/FixStartStatePathConstraints
                   stomp_moveit/StompSmoothingAdapter" />

#. The values of the ``planning_adapters`` is the order in which the mentioned adapters are called / invoked. Order here matters. Inside the STOMP adapter, a call to OMPL is made before invoking the STOMP smoothing solver, so STOMP takes the initial path computed by OMPL as the starting point to further optimize it.

#. Find the line where ``<rosparam command="load" file="$(find panda_moveit_config)/config/ompl_planning.yaml"/>`` is mentioned and after this line, add the following: ::

    <rosparam command="load" file="$(find panda_moveit_config)/config/stomp_planning.yaml"/>

#. These additions will add a STOMP Smoothing adapter and load the corresponding STOMP planner's parameters. To do this with your own robot replace ``panda_moveit_config`` to ``<my_robot>_moveit_config`` of your robot.

#. In the ``move_group.launch`` file of ``<robot_moveit_config>/launch`` folder for your robot, make sure that the default planner is ``ompl``.

#. In the ``stomp_planning.yaml`` file of ``<robot_moveit_config>/config`` folder for your robot, replace the following line: ::

    initialization_method: 1 #[1 : LINEAR_INTERPOLATION, 2 : CUBIC_POLYNOMIAL, 3 : MININUM_CONTROL_COST]

 with this line: ::

	initialization_method: 4 #[1 : LINEAR_INTERPOLATION, 2 : CUBIC_POLYNOMIAL, 3 : MININUM_CONTROL_COST, 4 : FILL_TRACJECTORY]

7. After making these requisite changes to the launch files, open a terminal and execute the following: ::

    roslaunch panda_moveit_config demo.launch

This will launch RViz, select OMPL in the Motion Planning panel under the Context tab. Set the desired start and goal states by moving the end-effector around. Finally click on the Plan button to start planning. The planner will now first run OMPL, then run STOMP on OMPL's output to produce an smooth path.

Running STOMP as a post-processor for CHOMP
+++++++++++++++++++++++++++++++++++++++++++

NOTE: The STOMP Smoothing Adapter is a work in progress.

Here, it is demonstrated that STOMP can be used as a post-processing smoothing technique for plans obtained by CHOMP.
To achieve this, follow the steps:

#. Open the ``chomp_planning_pipeline.launch`` file in the ``<robot_moveit_config>/launch`` folder of your robot. For the Panda robot it is `this file <https://github.com/ros-planning/panda_moveit_config/blob/melodic-devel/launch/chomp_planning_pipeline.launch.xml>`_. Edit this launch file, find the lines where ``<arg name="planning_plugins">`` is mentioned and add the following lines below it: ::

    <arg name="planning_adapters" value="stomp_moveit/StompSmoothingAdapter" />
    <param name="request_adapters" value="$(arg planning_adapters)" />

#. The values of the ``planning_adapters`` is the order in which the mentioned adapters are called / invoked. Order here matters. Inside the STOMP adapter, a call to CHOMP is made before invoking the STOMP smoothing solver, so STOMP takes the initial path computed by CHOMP as the starting point to further smoothen it.

#. Find the line where ``<rosparam command="load" file="$(find panda_moveit_config)/config/chomp_planning.yaml"/>`` is mentioned and after this line, add the following: ::

    <rosparam command="load" file="$(find panda_moveit_config)/config/stomp_planning.yaml"/>

#. These additions will add a STOMP Smoothing adapter and load the corresponding STOMP planner's parameters. To do this with your own robot replace ``panda_moveit_config`` to ``<my_robot>_moveit_config`` of your robot.

#. In the ``move_group.launch`` file of ``<robot_moveit_config>/launch`` folder for your robot, make sure that the default planner is ``ompl``.

#. In the ``stomp_planning.yaml`` file of ``<robot_moveit_config>/config`` folder for your robot, replace the following line: ::

    initialization_method: 1 #[1 : LINEAR_INTERPOLATION, 2 : CUBIC_POLYNOMIAL, 3 : MININUM_CONTROL_COST]

 with this line: ::

	initialization_method: 4 #[1 : LINEAR_INTERPOLATION, 2 : CUBIC_POLYNOMIAL, 3 : MININUM_CONTROL_COST, 4 : FILL_TRACJECTORY]

7. After making these requisite changes to the launch files, open a terminal and execute the following: ::

    roslaunch panda_moveit_config demo.launch

This will launch RViz, select CHOMP in the Motion Planning panel under the Context tab. Set the desired start and goal states by moving the end-effector around. Finally click on the Plan button to start planning. The planner will now first run CHOMP, then run STOMP on CHOMP's output to produce a smooth path.


Planning Insights for different motion planners and planners with planning adapters
-----------------------------------------------------------------------------------

This section has insights as to when to use which planner and how using certain planning request adapters in a certain pipeline can lead to producing robust paths overall. Here we consider using OMPL, STOMP, CHOMP seperately and together to produce robust smooth optimized paths obtained from the planner. For each planner, a basic insight is provided which gives the user an intuition to use a particular planner in a specific situation.

- **CHOMP**: CHOMP is an optimization algorithm which optimizes a given initial trajectory. Based on the environment CHOMP rapidly tries to pull the initial trajectory out of collisions. However an important point to pay attention here is that the parameter ``ridge_factor`` needs to be more than or equal to 0.001 for avoiding obstacles. Doing this CHOMP is able to find paths while avoiding obstacles. It should be noted here even though CHOMP can avoid obstacles successfully but it fails to provide smooth paths often leading to jerky paths in the presence of obstacles. For CHOMP collision avoidance comes at the cost of the trajectory's velocity smoothness.

- **STOMP**: STOMP produces smooth well behaved collision free paths within reasonable times. The approach relies on generating noisy trajectories to explore the space around an initial (possibly infeasible) trajectory which are then combined to produce an updated trajectory with lower cost.

- **OMPL** is a open source library for sampling based / randomized motion planning algorithms as discussed in the ompl planning tutorials. Sampling based algorithms are probabilistically complete: a solution would be eventually found if one exists, however non-existence of a solution cannot be reported. These algorithms are efficient and usually find a solution quickly.

For more information on each of these motion planners, refer to their individual tutorial pages `OMPL <../ompl_interface/ompl_interface_tutorial.html>`_, `CHOMP <../chomp_planner/chomp_planner_tutorial.html>`_ and `STOMP <../stomp_planner/stomp_planner_tutorial.html>`_.

- **OMPL as a pre-processor for CHOMP**: OMPL can used as a base planner to produce an initial motion plan which can act as a initial guess for CHOMP. CHOMP can then produce optimized paths. In most cases, the quality of such a path produced should be better than that produced by OMPL alone or CHOMP alone.

- **OMPL as a pre-processor for STOMP**: As stomp can used as a smoothing algorithm, it can be used to smoothen the plans produced by other motion planners. OMPL first produces a path, STOMP can then generate a smoothened version of that path. Such a path in most cases should be better than a path produced by either just OMPL or STOMP alone.

- **STOMP as a pre-processor for CHOMP**: For this case, a path can be initially produced by STOMP, CHOMP can then take this as an initial guess and produce an optimized version of the smoothened path produced by STOMP.

- **CHOMP as a pre-processor for STOMP**: CHOMP can be used to produce a path and then STOMP can be used to smoothen the path. This helps in getting rid of the jerky motion of the trajectories produced by CHOMP alone in the presence of obstacles.

A video demonstrating different planners working under different situations will be posted here soon once work on STOMP smoothing adapter is finished.
