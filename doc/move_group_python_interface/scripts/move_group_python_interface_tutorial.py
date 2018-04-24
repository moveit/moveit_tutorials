#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import rospy and some messages that we will use:
##
## .. _moveit_commander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
## END_SUB_TUTORIAL

from std_msgs.msg import String

try:
  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy:
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
  ## the robot:
  robot = moveit_commander.RobotCommander()

  ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
  ## to the world surrounding the robot:
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the Panda
  ## arm.  This interface can be used to plan and execute motions on the Panda:
  group = moveit_commander.MoveGroupCommander("panda_arm")

  ## We create a ``DisplayTrajectory`` publisher which is used later to publish
  ## trajectories for RViz to visualize:
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)

  print "============ Press Enter to Begin the Tutorial..."
  raw_input()
  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ## We can get the name of the reference frame for this robot:
  planning_frame = group.get_planning_frame()
  print "============ Reference frame: %s" % planning_frame

  ## We can also print the name of the end-effector link for this group:
  eef_link = group.get_end_effector_link()
  print "============ End effector: %s" % eef_link

  ## We can get a list of all the groups in the robot:
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot:
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"


  ## Planning to a Pose Goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
  ## thing we want to do is move it to a slightly better configuration.
  ## We can get the joint values from the group and adjust some of the values:
  joints = group.get_current_joint_values()
  joints[1] = -pi/4
  joints[3] = -pi/2
  joints[5] = pi/3

  ## The go command can be called with joint values, poses, or without any
  ## parameters if you have already set the pose or joint target for the
  ## group
  print "============ Commanding the robot with a joint command"
  group.go(joints, wait=True)

  print "============ Press Enter to Continue the Tutorial After Panda Finishes Moving..."
  raw_input()
  group.stop()

  ## We can plan a motion for this group to a desired pose for the
  ## end-effector:
  print "============ Generating plan 2"
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.w = 1.0
  pose_target.position.x = 0.3
  pose_target.position.y = -0.05
  pose_target.position.z = 0.4
  group.set_pose_target(pose_target)

  ## Now, we call the planner to compute the plan and visualize it if successful.
  ## Note that we are just planning, not asking move_group to actually move the robot yet:
  plan1 = group.plan()

  print "============ Press Enter to Continue the Tutorial After Rviz Displays Plan1 ..."
  raw_input()

  ## You can ask Rviz to visualize a plan (aka trajectory) for you. But the
  ## group.plan() method does this automatically so this is not that useful
  ## here (it just displays the same trajectory again):
  print "============ Visualizing plan1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);

  print "============ Press Enter to Continue the Tutorial After RViz Displays Plan1 (again) ..."
  raw_input()

  ## Moving to a Pose Goal
  ## ^^^^^^^^^^^^^^^^^^^^^
  ## Moving to a pose goal is similar to the step above
  ## except we now use the ``go()`` function. Note that
  ## the pose goal we had set earlier is still active
  ## and so the robot will try to move to that goal. We will
  ## not use that function in this tutorial since it is
  ## a blocking function and requires a controller to be active
  ## and report success on execution of a trajectory.

  # Uncomment below line when working with a real robot:
  # group.go(wait=True)

  # Use execute instead if you would like the robot to follow
  # the plan that has already been computed:
  group.execute(plan1)

  ## Planning to a Joint-Space Goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ## Let's set a joint space goal and move towards it.
  ## First, we will clear the pose target we had just set:
  print "============ Press Enter to Continue the Tutorial After Panda Finishes Executing Plan1 ..."
  raw_input()
  group.clear_pose_targets()

  ## Then, we will get the current set of joint values for the group:
  group_variable_values = group.get_current_joint_values()
  print "============ Joint values: ", group_variable_values

  ## Now, let's modify one of the joints, plan to the new joint
  ## space goal and visualize the plan:
  group_variable_values[0] = 1.0
  group.set_joint_value_target(group_variable_values)
  plan2 = group.plan()

  print "============ Press Enter to Continue the Tutorial After RViz Displays Plan2 ..."
  raw_input()

  ## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a Cartesian path directly by specifying a list of waypoints
  ## for the end-effector to go through:
  waypoints = []

  # first move down (-z):
  wpose = group.get_current_pose().pose
  wpose.position.z -= 0.2
  waypoints.append(copy.deepcopy(wpose))

  # second move forwards:
  wpose.position.x = 0.40
  waypoints.append(copy.deepcopy(wpose))

  # third move to the side:
  wpose.position.y += 0.3
  waypoints.append(copy.deepcopy(wpose))

  ## We want the Cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in Cartesian
  ## translation.  We will specify the jump threshold to 0.0 disabling it:
  (plan3, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold

  print "============ Press Enter to Continue the Tutorial After RViz Displays Plan3 ..."
  raw_input()

  # Uncomment the line below to execute this plan on a real robot:
  # group.execute(plan3)

  ## Adding/Removing Objects and Attaching/Detaching Objects
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ## First, we will create a box in the planning scene at the location of the left finger:
  box_pose = geometry_msgs.msg.PoseStamped()
  box_pose.header.frame_id = "panda_leftfinger"
  box_pose.pose.orientation.w = 1.0
  box_name = "box"
  scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

  ## We wait for the box to be displayed in RViz. If this node dies before the
  ## message has time to get serviced, the message could get lost and no box will appear.
  print "============ Press Enter to Continue the Tutorial After the Box Appears ..."
  raw_input()

  ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
  ## robot be able to touch them without the planning scene reporting the contact as a
  ## collision. By adding link names to the ``touch_links`` array, we are telling the
  ## planning scene to ignore collisions between those links and the box. To disable
  touch_links = robot.get_link_names(group='hand')
  scene.attach_box(eef_link, box_name, touch_links=touch_links)
  print "============ Press Enter to Continue the Tutorial After the Box Changes Color ..."
  raw_input()

  ## We can move the arm with the box attached:
  waypoints = []
  wpose = group.get_current_pose().pose
  wpose.position.x += 0.1
  waypoints.append(copy.deepcopy(wpose))
  wpose.position.z += 0.1
  waypoints.append(copy.deepcopy(wpose))

  (plan4, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
  group.execute(plan4)
  print "============ Press Enter to Continue the Tutorial After Rviz Displays Plan4 ..."

  ## We can also detach and remove the object from the planning scene:
  scene.remove_attached_object(eef_link, name=box_name)
  scene.remove_world_object(box_name)

  print "============ Press Enter to Continue the Tutorial After Panda Finishes Executing Plan4 ..."

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"

except rospy.ROSInterruptException:
  pass

