Low Level Controllers
=====================
In this section, we will walk through configuring MoveIt with the controllers on your robot. We will assume that your robot offers a ``FollowJointTrajectory`` action service for the arms on your robot and (optionally) a ``GripperCommand`` service for your gripper. If your robot does not offer this we recommend the `ROS control <http://wiki.ros.org/ros_control>`_ framework for easily adding this functionality around your hardware communication layer.

YAML Configuration
------------------
The first file to create is a YAML configuration file (call it ``controllers.yaml`` and place it in the ``robot_moveit_config/config`` directory of your MoveIt robot config package). This will specify the controller configuration for your robot. Here's an example file for configuring a ``FollowJointTrajectory`` action controller for the ``panda_arm`` and a ``GripperCommand`` gripper controller for its ``hand``: ::

 controller_list:
  - name: panda_arm_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - panda_joint1
      - panda_joint2
      - panda_joint3
      - panda_joint4
      - panda_joint5
      - panda_joint6
      - panda_joint7
  - name: hand_controller
    action_ns: gripper_action
    type: GripperCommand
    default: true
    parallel: true
    joints:
      - panda_finger_joint1
      - panda_finger_joint2

We will walk through the parameters for both types of controllers.

FollowJointTrajectory Controller Interface
------------------------------------------
The parameters are:
 * *name*: The name of the controller.  (See debugging information below for important notes).
 * *action_ns*: The action namespace for the controller. (See debugging information below for important notes).
 * *type*: The type of action being used (here FollowJointTrajectory).
 * *default*: The default controller is the primary controller chosen by MoveIt for communicating with a particular set of joints.
 * *joints*: Names of all the joints that are being addressed by this interface.

GripperCommand Controller Interface
-----------------------------------
The parameters are:
 * *name*: The name of the controller.  (See debugging information below for important notes).
 * *action_ns*: The action namespace for the controller. (See debugging information below for important notes).
 * *type*: The type of action being used (here GripperCommand).
 * *default*: The default controller is the primary controller chosen by MoveIt for communicating with a particular set of joints.
 * *joints*: Names of all the joints that are being addressed by this interface.
 * *parallel*: When this is set, *joints* should be of size 2, and the command will be the sum of the two joints.

Optional Allowed Trajectory Execution Duration Parameters
---------------------------------------------------------

For each controller it is optionally possible to set the *allowed_execution_duration_scaling* and *allowed_goal_duration_margin* parameters. These are controller-specific overrides of the global values *trajectory_execution/allowed_execution_duration_scaling* and *trajectory_execution/allowed_goal_duration_margin*. As opposed to the global values, the contoller-specific ones cannot be dynamically reconfigured at runtime. The parameters are used to compute the allowed trajectory execution duration by scaling the expected execution duration and adding the margin afterwards. If this duration is exceeded the trajectory will be cancelled. The controller-specific parameters can be set as follows ::

 controller_list:
  - name: arm_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    allowed_execution_duration_scaling: 1.2
    allowed_goal_duration_margin: 0.5

Create the Controller launch file
---------------------------------
Now, create the controller launch file (call it ``robot_moveit_controller_manager.launch.xml`` where ``robot`` is the name of your robot as specified when you created your MoveIt robot config package).

Add the following lines to this file: ::

 <launch>
  <!-- Set the param that trajectory_execution_manager needs to find the controller plugin -->
  <arg name="moveit_controller_manager" default="moveit_simple_controller_manager/MoveItSimpleControllerManager" />
  <param name="moveit_controller_manager" value="$(arg moveit_controller_manager)"/>
  <!-- load controller_list -->
  <rosparam file="$(find robot_moveit_config)/config/controllers.yaml"/>
 </launch>

MAKE SURE to replace ``robot_moveit_config`` with the correct name of your MoveIt robot config package.

Now, you should be ready to have MoveIt talk to your robot.

Debugging Information
---------------------
The ``FollowJointTrajectory`` or ``GripperCommand`` interfaces on your robot must be communicating in the namespace: ``/name/action_ns``. In the above example, you should be able to see the following topics (using *rostopic list*) on your robot:

 * /panda_arm_controller/follow_joint_trajectory/goal
 * /panda_arm_controller/follow_joint_trajectory/feedback
 * /panda_arm_controller/follow_joint_trajectory/result
 * /hand_controller/gripper_action/goal
 * /hand_controller/gripper_action/feedback
 * /hand_controller/gripper_action/result

You should also be able to see (using ``rostopic info topic_name``) that the topics are published/subscribed to by the controllers on your robot and also by the **move_group** node.

Remapping /joint_states topic
-----------------------------

When you run a `move group node <../move_group_interface/move_group_interface_tutorial.html>`_, you may need to remap the topic /joint_states to /robot/joint_states, otherwise MoveIt won't have feedback from the joints. To do this remapping you could make a simple launch file for your node as follows: ::

  <node pkg="moveit_ros_move_group" type="move_group" name="any_name" output="screen">
    <remap from="joint_states" to="robot/joint_states"/>
  </node>

Or you can make a subscriber with the correct topic name and then ensure that the starting robot state for your move group corresponds to a correct joints angle by using the call back of this subscriber.

Trajectory Execution Manager Options
------------------------------------

There are several options for tuning the behavior and safety checks of the execution pipeline in MoveIt. In your ``moveit_config`` package edit the ``trajectory_execution.launch.xml`` file to change the following parameters:

 - ``execution_duration_monitoring``: when false, will not throw error is trajectory takes longer than expected to complete at the low-level controller side
 - ``allowed_goal_duration_margin``: Allow more than the expected execution time before triggering a trajectory cancel (applied after scaling)
 - ``allowed_start_tolerance``: Allowed joint-value tolerance for validation that trajectory's first point matches current robot state. If set to zero will skip waiting for robot to stop after execution
