Introduction
------------
In MoveIt, we can plan motions for multiple robot arms, but there are pre-required steps to prepare robot models and configure ros controllers. This tutorial provides ROS beginners with the steps to model multiple arms, configure controllers, and plan motions using MoveIt.

.. image:: images/multiple_arms_start.png
   :width: 600pt
   :align: center

Watch this `Quick YouTube video <https://www.youtube.com/watch?v=h8zlsuzeW3U>`_ of the multiple arms being controlled in Gazebo with the MoveIt Rviz plugin.

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.


The steps of setting multiple arms environments to use MoveIt motion planning are as follows:

1. Build the Xacro/URDF model of the multiple arms.

2. Prepare the MoveIt config package using MoveIt setup Assistant. 

3. Write the ros controllers configuration and launch files for the multiple arms. 

4. Integrate the simulation in Gazebo with MoveIt motion planning.

5. Plan arms motions with MoveIt Move Group Interface.

This tutorial explains every step to help set up your multiple robot arms environment. 

1. Build the Xacro/URDF model of the multiple arms
--------------------------------------------------

The Panda robot arm is used in the following explanation, but the same applies to preparing other types of robot arms.

To start building your multiple arms model, create a new ``panda_multiple_arms`` package as follows: :: 

    cd ~/ws_moveit/src
    catkin create pkg panda_multiple_arms
    cd panda_multiple_arms
    mkdir robot_description
    touch panda_multiple_arms.xacro

To prepare your multiple robot arms xacro file (model), you need to have the single arm's xacro file. In the following part, we will build a multiple arms panda robot description file consisting of two identical arms.

..
    It is worth mentioning that the difference between xacro and URDF is that TODO1. This property makes it easier to include multiple robot arms models in the same file, with a different prefix. 

Our multiple arms model has ``rgt_arm`` and ``lft_arm`` models. Each arm is equipped with a gripper. The xacro files can get lengthy. Here is a link to the multiple_arms_ xacro file. 

.. _multiple_arms: https://github.com/Robotawi/panda_arms_ws/blob/master/src/panda_multiple_arms/robot_description/panda_multiple_arms.xacro 


Notes: 

1. Two xacro arguments ``rgt_arm`` and ``lft_arm`` are defined as prefixes to differentiate the arms and hands names. 
   
2. The arms and hands models are loaded from the ``franka_description`` package, which is installed as a dependency of the ``panda_moveit_config`` package. When modeling your robot, make sure the robot_description package is available in your ROS workspace.

3. We usually need to have a careful look at the robot's xacro file to understand the xacro parameters to use. Here is an example from the ``panda_arm.xacro`` in the ``franka_description`` package. ::
      
    <xacro:macro name="panda_arm" params="arm_id:='panda' description_pkg:='franka_description' connected_to:='' xyz:='0 0 0' rpy:='0 0 0' gazebo:=false safety_distance:=0">


We can search those parameters in the xacro macro to understand the function of each. The ``arm_id`` sets a prefix to the arm name to enable reusing the same model. This is essential for our purpose of modeling multiple robots. The ``connected_to`` parameter allows the robot base to be attached to a given link. In our multiple arms model, each robot is connected to the box shaped base. The gazebo parameter determines whether to load the gazebo simulation required information (e.g links inertias and joints transmissions) or not. 

After knowing the xacro macro for the arm, and understanding the input parameters, we can use it as follows to load the arms. ::

    <xacro:panda_arm arm_id="$(arg arm_id_1)" connected_to="base" xyz="0 -0.5 1" gazebo="true" safety_distance="0.03" />

    <xacro:panda_arm arm_id="$(arg arm_id_2)" connected_to="base" xyz="0 0.5 1" gazebo="true" safety_distance="0.03" />


The same applies to loading the grippers/hands models, and other robots that are defined with xacro macros. 

At this point, it is recommended to check our xacro model is working as expected. This can be done in three simple steps; convert your xacro model to URDF, check the connections between links and joints are correct, and visualize it (as described before). Run the following commands to check the URDF has no problems. ::
    
    cd ~/ws_moveit
    catkin build 
    source devel/setup.bash
    roscd panda_multiple_arms/robot_description
    rosrun xacro xacro panda_multiple_arms.xacro -o panda_multiple_arms.urdf
    check_urdf panda_multiple_arms.urdf


The ``check_urdf`` shows the links tree and indicats if there are any errors: ::

    robot name is: panda_multiple_arms
    ---------- Successfully Parsed XML ---------------
    root Link: world has 1 child(ren)
        child(1):  base
            child(1):  lft_arm_link0
                child(1):  lft_arm_link1
                    child(1):  lft_arm_link2
                        child(1):  lft_arm_link3
                            child(1):  lft_arm_link4
                                child(1):  lft_arm_link5
                                    child(1):  lft_arm_link6
                                        child(1):  lft_arm_link7
                                            child(1):  lft_arm_link8
                                                child(1):  lft_arm_hand
                                                    child(1):  lft_arm_leftfinger
                                                    child(2):  lft_arm_rightfinger
                                                    child(3):  lft_arm_hand_sc
                                                    child(4):  lft_arm_hand_tcp
                                            child(2):  lft_arm_link7_sc
                                        child(2):  lft_arm_link6_sc
                                    child(2):  lft_arm_link5_sc
                                child(2):  lft_arm_link4_sc
                            child(2):  lft_arm_link3_sc
                        child(2):  lft_arm_link2_sc
                    child(2):  lft_arm_link1_sc
                child(2):  lft_arm_link0_sc
            child(2):  rgt_arm_link0
                child(1):  rgt_arm_link1
                    child(1):  rgt_arm_link2
                        child(1):  rgt_arm_link3
                            child(1):  rgt_arm_link4
                                child(1):  rgt_arm_link5
                                    child(1):  rgt_arm_link6
                                        child(1):  rgt_arm_link7
                                            child(1):  rgt_arm_link8
                                                child(1):  rgt_arm_hand
                                                    child(1):  rgt_arm_leftfinger
                                                    child(2):  rgt_arm_rightfinger
                                                    child(3):  rgt_arm_hand_sc
                                                    child(4):  rgt_arm_hand_tcp
                                            child(2):  rgt_arm_link7_sc
                                        child(2):  rgt_arm_link6_sc
                                    child(2):  rgt_arm_link5_sc
                                child(2):  rgt_arm_link4_sc
                            child(2):  rgt_arm_link3_sc
                        child(2):  rgt_arm_link2_sc
                    child(2):  rgt_arm_link1_sc
                child(2):  rgt_arm_link0_sc


To visually check your multiple robot model, run the command: ::

    roslaunch urdf_tutorial display.launch model:=panda_multiple_arms.urdf

Once Rviz GUI starts, set the fixed frame on the upper left corner to be ``base``. 

.. image:: images/rviz_fixed_frame.png
   :width: 300pt
   :align: center

If the model is correctly prepared, it should show up as follows. 

.. image:: images/rviz_start.png
   :width: 500pt
   :align: center


This concludes the step of building the model and verifying it. 

Step 2: Prepare the MoveIt config package using MoveIt Setup Assistant 
----------------------------------------------------------------------

If you are not familiar with MoveIt Setup Assistant, please refer to this `tutorial <https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html>`_. 

MoveIt Setup Assistant is used to configure our multiple robot arms for using the MoveIt pipeline. 

- Start the MoveIt Setup Assistant: ::

    roslaunch moveit_setup_assistant setup_assistant.launch

Follow the MoveIt Setup Assistant tutorial to configure the arms. Note that we will be making a separate move group for each arm and hand. The groups are called ``rgt_arm``, ``lft_arm``, ``rgt_hand``, and ``lft_hand``. 


I want to consider two more point along with the the Setup Assistant tutorial 

1. Define a practical `ready` pose for both arms with joint values {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}.

The arms should look as follows at the `ready` pose.

.. image:: images/rgt_lft_arms_ready_poses.png
   :width: 500pt
   :align: center


1. Define ``open`` and ``close`` poses for the ``rgt_hand`` and ``lft_hand`` move groups. The ``open`` pose with joint1 value set to 0.35, and the ``close`` has the joint1 set to 0.0. Notice that the hand joint2 mimics the value of joint1.  Therefore, there is no need to fix joint2 in the hand move_group poses.  The defined poses for the arms and hand can be as follows. You can add other poses of interest for the arms, if needed.

.. image:: images/move_groups_poses.png
   :width: 500pt
   :align: center

Name the Moveit config package ``panda_multiple_arms_moveit_config`` and generate the files using the Setup Assistant. 

Step 3: Write the ros controllers configuration and launch files for the multiple arms 
--------------------------------------------------------------------------------------

This step creates ``ros_control`` configuration files and ``roslaunch`` files to start them. We need two controller types. The first is a *joint state controller*, which publishes the state of all joints. The second is *joint trajectory controller* type, which executes joint-space trajectories on a group of robot joints.

In the following configuration files, the controllers names are ``joint_state_controller``, ``rgt_arm_trajectory_controller``, and ``lft_arm_trajectory_controller``. Under each trajectory controller, we need to specify its hardware interface type, joint groups, and constraints. For more about ros controllers,  refer to their documentation_. Let's create the controllers configuration and their launch file in systematic steps and with descriptive names. Some comments are added after the steps not to break the flow. 

.. _documentation: http://wiki.ros.org/ros_control  

- The joint state controller:
   
1. Create the controller configuration files ``joint_state_controller.yaml`` inside the ``panda_multiple_arms/config`` package as follows::

    cd ~/ws_moveit
    cd src/panda_multiple_arms
    mkdir config && cd config
    touch joint_state_controller.yaml 

2. Open the ``joint_state_controller.yaml`` and copy the controller configuration to it ::

    joint_state_controller:
        type: joint_state_controller/JointStateController
        publish_rate: 50  

- The joint trajectory controller: 

3. Create the controller configuration file ``trajectory_controller.yaml`` in the ``panda_multiple_arms/config`` package as follows::

    cd ~/ws_moveit
    cd src/panda_multiple_arms/config
    touch trajectory_controller.yaml


4. Open the ``trajectory_controller.yaml`` and copy the controller configuration to it ::

    rgt_arm_trajectory_controller:
    type: "position_controllers/JointTrajectoryController"
    joints:
        - rgt_arm_joint1
        - rgt_arm_joint2
        - rgt_arm_joint3
        - rgt_arm_joint4
        - rgt_arm_joint5
        - rgt_arm_joint6
        - rgt_arm_joint7
    constraints:
        goal_time: 0.6
        stopped_velocity_tolerance: 0.05
        rgt_arm_joint1: {trajectory: 0.1, goal: 0.1}
        rgt_arm_joint2: {trajectory: 0.1, goal: 0.1}
        rgt_arm_joint3: {trajectory: 0.1, goal: 0.1}
        rgt_arm_joint4: {trajectory: 0.1, goal: 0.1}
        rgt_arm_joint5: {trajectory: 0.1, goal: 0.1}
        rgt_arm_joint6: {trajectory: 0.1, goal: 0.1}
        rgt_arm_joint7: {trajectory: 0.1, goal: 0.1}
    stop_trajectory_duration: 0.5
    state_publish_rate:  25
    action_monitor_rate: 10

    lft_arm_trajectory_controller:
    type: "position_controllers/JointTrajectoryController"
    joints:
        - lft_arm_joint1
        - lft_arm_joint2
        - lft_arm_joint3
        - lft_arm_joint4
        - lft_arm_joint5
        - lft_arm_joint6
        - lft_arm_joint7
    constraints:
        goal_time: 0.6
        stopped_velocity_tolerance: 0.05
        lft_arm_joint1: {trajectory: 0.1, goal: 0.1}
        lft_arm_joint2: {trajectory: 0.1, goal: 0.1}
        lft_arm_joint3: {trajectory: 0.1, goal: 0.1}
        lft_arm_joint4: {trajectory: 0.1, goal: 0.1}
        lft_arm_joint5: {trajectory: 0.1, goal: 0.1}
        lft_arm_joint6: {trajectory: 0.1, goal: 0.1}
        lft_arm_joint7: {trajectory: 0.1, goal: 0.1}
    stop_trajectory_duration: 0.5
    state_publish_rate:  25
    action_monitor_rate: 10

    #notice that the grippers joint2 mimics joint1
    #this is why it is not listed under the hand controllers
    rgt_hand_controller:
    type: "effort_controllers/JointTrajectoryController"
    joints:
        - rgt_arm_finger_joint1
    gains:
        rgt_arm_finger_joint1:  {p: 50.0, d: 1.0, i: 0.01, i_clamp: 1.0}

    lft_hand_controller:
    type: "effort_controllers/JointTrajectoryController"
    joints:
        - lft_arm_finger_joint1
    gains:
        lft_arm_finger_joint1:  {p: 50.0, d: 1.0, i: 0.01, i_clamp: 1.0}



5. Create the  launch ``control_utils.launch`` file to start the robot state publisher, load the controllers, and spawn them. ::

    <?xml version="1.0"?>
    <launch>

    <!-- Robot state publisher -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
        <param name="publish_frequency" type="double" value="50.0" />
        <param name="tf_prefix" type="string" value="" />
    </node>

    <!-- Joint state controller -->
    <rosparam file="$(find panda_multiple_arms)/config/joint_state_controller.yaml" command="load" />
    <node name="joint_state_controller_spawner" pkg="controller_manager" type="spawner" args="joint_state_controller" respawn="false" output="screen" />

    <!-- Joint trajectory controller -->
    <rosparam file="$(find panda_multiple_arms)/config/trajectory_controller.yaml" command="load" />
    <node name="arms_trajectory_controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen" args="rgt_arm_trajectory_controller lft_arm_trajectory_controller rgt_hand_controller lft_hand_controller" />

    </launch>

The joint state controller publishes the robot joint values on the ``/joint_states`` topic, and the robot state publisher uses them to calculate forward kinematics and publish the poses/transforms of the robot links. The joint trajectory controller enables executing joint-space trajectories on a group of joints.

..
    Please be careful with the namespace (ns) and the controllers names when doing this step. Those names must match the names in the trajectory_controller.yaml file. 

The remaining part of this step presents guidance how to modify the auto-generated control-related files in the moveit config package for interfacing the arm using MoveIt to Gazebo. Also in a systematic way, we need to modify two files, ``ros_controllers.yaml``, and ``simple_moveit_controllers.yaml`` 

- The ros_controllers.yaml 

  The ``ros_controllers.yaml`` file is autogenerated in the  ``panda_multiple_arms_moveit_config/config``. According to the MoveIt Setup Assistant authors, this file is meant for the ros control configuration (this means its content should exactly match the content of ``joint_state_controller.yaml`` and ``trajectory_controller.yaml``). Then, just copy the two files contents into this file. The file should look as follows ::
    
    joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50  
    
    rgt_arm_trajectory_controller:
    type: "position_controllers/JointTrajectoryController"
    joints:
        - rgt_arm_joint1
        - rgt_arm_joint2
        - rgt_arm_joint3
        - rgt_arm_joint4
        - rgt_arm_joint5
        - rgt_arm_joint6
        - rgt_arm_joint7
    constraints:
        goal_time: 0.6
        stopped_velocity_tolerance: 0.05
        rgt_arm_joint1: {trajectory: 0.1, goal: 0.1}
        rgt_arm_joint2: {trajectory: 0.1, goal: 0.1}
        rgt_arm_joint3: {trajectory: 0.1, goal: 0.1}
        rgt_arm_joint4: {trajectory: 0.1, goal: 0.1}
        rgt_arm_joint5: {trajectory: 0.1, goal: 0.1}
        rgt_arm_joint6: {trajectory: 0.1, goal: 0.1}
        rgt_arm_joint7: {trajectory: 0.1, goal: 0.1}
    stop_trajectory_duration: 0.5
    state_publish_rate:  25
    action_monitor_rate: 10

    lft_arm_trajectory_controller:
    type: "position_controllers/JointTrajectoryController"
    joints:
        - lft_arm_joint1
        - lft_arm_joint2
        - lft_arm_joint3
        - lft_arm_joint4
        - lft_arm_joint5
        - lft_arm_joint6
        - lft_arm_joint7
    constraints:
        goal_time: 0.6
        stopped_velocity_tolerance: 0.05
        lft_arm_joint1: {trajectory: 0.1, goal: 0.1}
        lft_arm_joint2: {trajectory: 0.1, goal: 0.1}
        lft_arm_joint3: {trajectory: 0.1, goal: 0.1}
        lft_arm_joint4: {trajectory: 0.1, goal: 0.1}
        lft_arm_joint5: {trajectory: 0.1, goal: 0.1}
        lft_arm_joint6: {trajectory: 0.1, goal: 0.1}
        lft_arm_joint7: {trajectory: 0.1, goal: 0.1}
    stop_trajectory_duration: 0.5
    state_publish_rate:  25
    action_monitor_rate: 10

    #notice that the grippers joint2 mimics joint1
    #this is why it is not listed under the hand controllers
    rgt_hand_controller:
    type: "effort_controllers/JointTrajectoryController"
    joints:
        - rgt_arm_finger_joint1
    gains:
        rgt_arm_finger_joint1:  {p: 50.0, d: 1.0, i: 0.01, i_clamp: 1.0}

    lft_hand_controller:
    type: "effort_controllers/JointTrajectoryController"
    joints:
        - lft_arm_finger_joint1
    gains:
        lft_arm_finger_joint1:  {p: 50.0, d: 1.0, i: 0.01, i_clamp: 1.0}
    
.. 
    Notice that the namespace and controller names correspond to the names in ``trajectory_controller.yaml`` file.
- The simple_moveit_controllers.yaml 

  This file is autogenerated in the ``panda_multiple_arms_moveit_config/config``. Moveit requires a trajectory controller which has a FollowJointTrajectoryAction interface. After motion planning, the FollowJointTrajectoryAction interface sends the generated trajectory to the robot ros controller (written above).
  
  This is the file that configures the controllers to be used by MoveIt controller manager to operate the robot. The controllers names should correspond to the ros controllers in the previous ``ros_controllers.yaml``, which is same as the ``trajectory_control.yaml``. 


Modify the file contents to be as follows. :: 
    
    controller_list:
      - name: rgt_arm_trajectory_controller
          action_ns: follow_joint_trajectory
          type: FollowJointTrajectory
          default: True
          joints:
          - rgt_arm_joint1
          - rgt_arm_joint2
          - rgt_arm_joint3
          - rgt_arm_joint4
          - rgt_arm_joint5
          - rgt_arm_joint6
          - rgt_arm_joint7
      - name: lft_arm_trajectory_controller
          action_ns: follow_joint_trajectory
          type: FollowJointTrajectory
          default: True
          joints:
          - lft_arm_joint1
          - lft_arm_joint2
          - lft_arm_joint3
          - lft_arm_joint4
          - lft_arm_joint5
          - lft_arm_joint6
          - lft_arm_joint7

      #notice that the grippers joint2 mimics joint1
      #this is why it is not listed under the hand controllers

      - name: rgt_hand_controller
          action_ns: follow_joint_trajectory
          type: FollowJointTrajectory
          default: true
          joints:
          - rgt_arm_finger_joint1

      - name: lft_hand_controller
          action_ns: follow_joint_trajectory
          type: FollowJointTrajectory
          default: true
          joints:
          - lft_arm_finger_joint1

The last step is to let the ``ros_controllers.launch`` spawn the ros controllers configured in the ``ros_controller.yaml`` file. This is simply done by adding the controller names as arguments in the spawner node as shown below. ::

    <?xml version="1.0"?>
    <launch>

        <!-- Load joint controller configurations from YAML file to parameter server -->
        <rosparam file="$(find panda_multiple_arms_moveit_config)/config/ros_controllers.yaml" command="load"/>

        <!-- Load the controllers -->
        <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
            output="screen" args=" rgt_arm_trajectory_controller lft_arm_trajectory_controller rgt_hand_controller lft_hand_controller"/>

    </launch>




Step 4: Integrate the simulation in Gazebo with Moveit motion planning
----------------------------------------------------------------------

We need to prepare a launch file to start three required components for the integration to work. Those components are the simulated robot in Gazebo, ros controllers, and moveit motion plannig executable. We have already prepared the ``control_utils.launch`` file to load the ros controllers, and the required moveit motion planning file ``move_group.launch`` is auto generated. Then, our tasks here are to start the simulated robot in gazebo world, and prepare a launch file that launches the above mentioned three components.

1. Start the simulated a robot in an empty Gazebo world 

To spawn the panda multiple arms model in a gazebo, we need to prepare a launch file in the ``panda_multiple_arms`` package. Let's call it ``panda_multiple_arms_empty_world.launch``. Here are the steps to prepar this file. :: 

    cd ~/ws_moveit
    cd src/panda_multiple_arms/launch 
    touch panda_multiple_arms_empty_world.launch

The ``panda_multiple_arms_empty_world.launch`` file launches an empty world file, loads the robot description, and spawns the robot in the empty world. Its contents are as follows::

    <?xml version="1.0"?>
    <launch>
        <!-- Launch empty Gazebo world -->
        <include file="$(find gazebo_ros)/launch/empty_world.launch">
            <arg name="use_sim_time" value="true" />
            <arg name="gui" value="true" />
            <arg name="paused" value="false" />
            <arg name="debug" value="false" />
        </include>

        <!-- Find my robot Description-->
        <param name="robot_description" command="$(find xacro)/xacro  '$(find panda_multiple_arms)/robot_description/panda_multiple_arms.xacro'" />

        <!-- Spawn The robot over the robot_description param-->
        <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" args="-urdf -param robot_description -model panda_multiple_arms" />
        
    </launch>

2. Prepare a ``bringup_moveit.launch`` file to start the three components. Create the file in the ``panda_multiple_arms/launch`` directory as follows then copy the contents into it. ::

    cd ~/ws_moveit
    cd src/panda_multiple_arms/launch 
    touch bringup_moveit.launch

The ``bringup_moveit.launch`` contents are as follows. ::
    
    <?xml version="1.0"?>
    <launch>

        <!-- Run the main MoveIt executable with trajectory execution -->
        <include file="$(find panda_multiple_arms_moveit_config)/launch/move_group.launch">
            <arg name="allow_trajectory_execution" value="true" />
            <arg name="moveit_controller_manager" value="ros_control" />
            <arg name="fake_execution_type" value="interpolate" />
            <arg name="info" value="true" />
            <arg name="debug" value="false" />
            <arg name="pipeline" value="ompl" />
            <arg name="load_robot_description" value="true" />
        </include>

        <!-- Start the simulated robot in an empty Gazebo world -->
        <include file="$(find panda_multiple_arms)/launch/panda_multiple_arms_empty_world.launch" />

        <!-- Start the controllers and robot state publisher-->
        <include file="$(find panda_multiple_arms)/launch/control_utils.launch"/>

        <!-- Start moveit_rviz with the motion planning plugin -->
        <include file="$(find panda_multiple_arms_moveit_config)/launch/moveit_rviz.launch">
            <arg name="rviz_config" value="$(find panda_multiple_arms_moveit_config)/launch/moveit.rviz" />
        </include>

    </launch>


To run the Moveit Gazebo integration, run the ``bringup_moveit.launch``. ::

    roslaunch panda_multiple_arms bringup_moveit.launch

If all steps are done, this should bringup all the required components for the integration. Then, we can plan motions for the arms and hands using MoveIt's rviz plugin and execute those motions on the simulated robots in Gazebo as shown in `this video <https://www.youtube.com/watch?v=h8zlsuzeW3U>`_.


Step 5: Plan arms motions with MoveIt Move Group Interface.
-----------------------------------------------------------

After ensuring our integration is correct, the most interesting part is to plan robot motion with the Moveit API and see our robots moving in Gazebo. This step shows how to prepare the dependenies and write code for planning simple motions for the arms and hands.

We need to include some dependenies in the robot's package ``CMakeLists.txt`` file. They are packages to enable using moveit group interface and utility package to describe the arms target poses. Here is a link to a `minimal CMakeLists.txt <https://github.com/Robotawi/panda_arms_ws/blob/master/src/panda_multiple_arms/CMakeLists.txt>`_ file used in this step. 

For the motion planning, please refer to Move Group Interface `tutorial <https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html>`_ for more details about MoveIt's move group C++ interface. We are using a separate move group for every arm and every hand.

This is the `file <https://github.com/Robotawi/panda_arms_ws/blob/master/src/panda_multiple_arms/src/plan_simple_motion.cpp>`_ used for planning the simple motions. The code in this file does the following.

1. Set the move groups names for arms and hands (considering same naming in step 2).::
   
    static const std::string rgt_arm_group = "rgt_arm";
    static const std::string rgt_hand_group = "rgt_hand";

    static const std::string lft_arm_group = "lft_arm";
    static const std::string lft_hand_group = "lft_hand";


2. Declare MoveGroupInterface objects for every arm and hand.::
    
    moveit::planning_interface::MoveGroupInterface rgt_arm_move_group_interface(rgt_arm_group);
    moveit::planning_interface::MoveGroupInterface rgt_hand_move_group_interface(rgt_hand_group);

    moveit::planning_interface::MoveGroupInterface lft_arm_move_group_interface(lft_arm_group);
    moveit::planning_interface::MoveGroupInterface lft_hand_move_group_interface(lft_hand_group);

3. Set the arms goal poses to the pre-defined ``ready`` pose.::
   
    rgt_arm_move_group_interface.setNamedTarget("ready");
    lft_arm_move_group_interface.setNamedTarget("ready");

4. Plan the arms motions, and if the planning is successful move arms and open grippers.::
   
    bool rgt_success = (rgt_arm_move_group_interface.plan(rgt_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    bool lft_success = (lft_arm_move_group_interface.plan(lft_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (rgt_success)
    {
        rgt_arm_move_group_interface.execute(rgt_arm_plan);

        rgt_hand_move_group_interface.setNamedTarget("open");
        rgt_hand_move_group_interface.move();
    }

    if (lft_success)
    {
        lft_arm_move_group_interface.execute(lft_arm_plan);

        lft_hand_move_group_interface.setNamedTarget("open");
        lft_hand_move_group_interface.move();
    }

5. In the last step, the arms are tasked to move arbitary motion with respect to theie current poses. The right arm moves 0.10 meter up, and the left arm moves 0.10 forward. Here is the code for moving the right arm up. ::
   
    geometry_msgs::PoseStamped current_rgt_arm_pose = rgt_arm_move_group_interface.getCurrentPose();
    geometry_msgs::PoseStamped target_rgt_arm_pose = current_rgt_arm_pose;

    target_rgt_arm_pose.pose.position.z += 0.10;

    rgt_arm_move_group_interface.setPoseTarget(target_rgt_arm_pose);
    rgt_success = (rgt_arm_move_group_interface.plan(rgt_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (rgt_success)
    {
        rgt_arm_move_group_interface.execute(rgt_arm_plan);
    }


This `short YouTube video <https://youtu.be/sxUQh91oQxM>`_ shows the described arms and hands motions using MoveIt move group interface. You may think the arms should move in straight lines between current and target poses. This is can be accomplished using the MoveIt Cartesian Planners, which is also explained in the Move Group Interface `tutorials <https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html>`_, and you are strongly encouraged to implement it. 


..
    Tutorial for multiple robot arms
    While there are some ROS Answers posts and examples floating around, there is no definitive resource on how to set up multiple manipulators with MoveIt (and especially MoveIt2). The goal of this project is to write a tutorial that should become the reference.
    Expected outcome: A ROS beginner can read the tutorial and set up a ros2_control / MoveIt pipeline without additional help.
    Project size: medium (175 hours)
    Difficulty: easy
    Preferred skills: Technical Writing, ROS, MoveIt, Python, and YAML
    Mentor: Andy Zelenak