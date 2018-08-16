Planning Adapter Tutorials
==========================

Planning Request Adapters is a concept in MoveIt which can be used to modify the trajectory for a motion planner. Some examples of existing planning adapters in MoveIt include AddTimeParameterization, FixWorkspaceBounds, FixStartBounds, FixStartStateCollision, FixStartStatePathConstraints, CHOMPOptimizerAdapter, etc. ! 
Planning Adapter for STOMP is currently work in progress.

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

You should also have gone through the steps in `Visualization with MoveIt! RViz Plugin <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_

Prerequisites
--------------
 1. You must have the latest version of MoveIt! installed. On ROS Kinetic you will need to build MoveIt! from source. A build from source is required as CHOMP is not part of the official release yet. It is therefore not included in the binary packages. We will go through the steps for doing this below.
 2. To use CHOMP with your robot you must already have a MoveIt! configuration package for your robot already. For example, if you have a Panda robot, it's probably called ``panda_moveit_config``. This is typically built using the `MoveIt! Setup Assistant <../setup_assistant/setup_assistant_tutorial.html>`_.

Installing MoveIt! from Source
------------------------------
As you add and remove packages from your workspace you will need to clean your workspace and re-run the command to install new missing dependencies. Clean your workspace to remove references to the system-wide installation of MoveIt!: ::

  cd ~/ws_moveit/src
  catkin clean

Now follow the instructions on the MoveIt! homepage for `installing MoveIt! Kinetic from source <http://moveit.ros.org/install/source/>`_. Note that you can skip the **Prerequisites** section since you should already have a Catkin workspace.

Re-source the setup files: ::

  source ~/ws_moveit/devel/setup.bash

Using Planning Request Adapter with Your Motion Planner
-------------------------------------------------------


Running the Demo
----------------
If you have the ``panda_moveit_config`` from the `ros-planning/panda_moveit_config <https://github.com/ros-planning/panda_moveit_config>`_ repository you should be able to simply run the demo: ::

  roslaunch panda_moveit_config demo_chomp.launch

Running OMPL as a pre-processor for CHOMP
+++++++++++++++++++++++++++++++++++++++++

Here, it is demonstrated that CHOMP can also be used as a post-processing optimization technique for plans obtained by other planning algorithms. The intuition behind this is that some randomized planning algorithm produces an initial guess for CHOMP. CHOMP then takes this initial guess and further optimizes the trajectory. 
To achieve this, follow the steps:

#. Open the ``ompl_planning_pipeline.launch`` file in the ``<robot_moveit_config>/launch`` folder of your robot. For the Panda robot it is `this <https://github.com/ros-planning/panda_moveit_config/blob/master/launch/ompl_planning_pipeline.launch.xml>`_ file. Edit this launch file, find the lines where ``<arg name="planning_adapters">`` is mentioned and change it to: ::

    <arg name="planning_adapters" value="default_planner_request_adapters/AddTimeParameterization
                   default_planner_request_adapters/FixWorkspaceBounds
                   default_planner_request_adapters/FixStartStateBounds
                   default_planner_request_adapters/FixStartStateCollision
                   default_planner_request_adapters/FixStartStatePathConstraints
                   default_planner_request_adapters/CHOMPOptimizerAdapter" />

#. The values of the ``planning_adapters`` is the order in which the mentioned adapters are called / invoked. Order here matters. Inside the CHOMP adapter, a `call <https://github.com/ros-planning/moveit/tree/kinetic-devel/moveit_ros/planning/planning_request_adapter_plugins/src/chomp_optimizer_adapter.cpp#L174>`_ to OMPL is made before invoking the CHOMP optimization solver, so CHOMP takes the initial path computed by OMPL as the starting point to further optimize it. 

#. Find the line where ``<rosparam command="load" file="$(find panda_moveit_config)/config/ompl_planning.yaml"/>`` is mentioned and after this line, add the following: ::

    <rosparam command="load" file="$(find panda_moveit_config)/config/chomp_planning.yaml"/>

#. These additions will add a CHOMP Optimization adapter and load the corresponding CHOMP planner's parameters. To do this with your own robot replace ``panda_moveit_config`` to ``<my_robot>_moveit_config`` of your robot.

#. In the ``move_group.launch`` file of ``<robot_moveit_config>/launch`` folder for your robot, make sure that the default planner is ``ompl``.

#. In the ``chomp_planning.yaml`` file of ``<robot_moveit_config>/config`` folder for your robot, add the following line: :: 

    trajectory_initialization_method: "fillTrajectory"

#. After making these requisite changes to the launch files, open a terminal and execute the following: ::

    roslaunch panda_moveit_config demo_chomp.launch

This will launch RViz, select OMPL in the Motion Planning panel under the Context tab. Set the desired start and goal states by moving the end-effector around in the same way as was done for CHOMP above. Finally click on the Plan button to start planning. The planner will now first run OMPL, then run CHOMP on OMPL's output to produce an optimized path.



Running OMPL as a pre-processor for STOMP
+++++++++++++++++++++++++++++++++++++++++

This section illustrates using OMPL as a pre-processor algrithm for STOMP.

Running CHOMP as a pre-processor for STOMP or vice versa
++++++++++++++++++++++++++++++++++++++++++++++++++++++++

CHOMP has some optimization parameters associated with it. These can be modified for the given environment/robot you are working with and is normally present in the `chomp_planning.yaml <https://github.com/ros-planning/panda_moveit_config/blob/master/config/chomp_planning.yaml>`_ file in config folder of the robot you are working with. If this file does not exist for your robot, you can create it and set the parameter values as you want. The following are some of the insights to set up these parameter values for some of them:


Planning Insights for different motion planners and planners with planning adapters
-----------------------------------------------------------------------------------

Need to change this section.
This section has insights as to when to use which planner and how using certain planning request adapters in a certain pipeline can lead to producing robust paths overall. Here we consider using OMPL, STOMP, CHOMP seperately and together to produce robust smooth optimized paths obtained from the planner. Videos for using a certain motion planner over another is available here (ADD LINK to youtube video). 

Optimizing planners optimize a cost function that may sometimes lead to surprising results: moving through a thin obstacle might be lower cost than a long, winding trajectory that avoids all collisions. In this section we make a distinction between paths obtained from CHOMP and contrast it to those obtained from OMPL.

OMPL is a open source library for sampling based / randomized motion planning algorithms. Sampling based algorithms are probabilistically complete: a solution would be eventually found if one exists, however non-existence of a solution cannot be reported. These algorithms are efficient and usually find a solution quickly. OMPL does not contain any code related to collision checking or visualization as the designers of OMPL did not want to tie it to a any particular collision checker or visualization front end. The library is designed so it can be easily integrated into systems that provide the additional components. MoveIt integrates directly with OMPL and uses the motion planners from OMPL as its default set of planners. The planners in OMPL are abstract; i.e. OMPL has no concept of a robot. Instead, MoveIt! configures OMPL and provides the back-end for OMPL to work with problems in Robotics.

CHOMP: While most high-dimensional motion planners separate trajectory generation into distinct planning and optimization stages, CHOMP capitalizes on covariant gradient and functional gradient approaches to the optimization stage to design a motion planning algorithm based entirely on trajectory optimization. Given an infeasible naive trajectory, CHOMP reacts to the surrounding environment to quickly pull the trajectory out of collision while simultaneously optimizing dynamical quantities such as joint velocities and accelerations. It rapidly converges to a smooth collision-free trajectory that can be executed efficiently on the robot. A covariant update rule ensures that CHOMP quickly converges to a locally optimal trajectory. 

For scenes containing obstacles, CHOMP often generates paths which do not prefer smooth trajectories by addition of some noise (*ridge_factor*) in the cost function for the dynamical quantities of the robot (like acceleration, velocity). CHOMP is able to avoid obstacle in most cases but it can fail if it gets stuck in the local minima due to a bad initial guess for the trajectory. OMPL can be used to generate collision-free seed trajectories for CHOMP to mitigate this issue.


