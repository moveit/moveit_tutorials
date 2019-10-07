########################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2012, Willow Garage, Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

## Author: Ioan Sucan, Ridhwan Luthra
## Contributor: Rick Staa (Translated code from cpp to python)

## Python standard library imports ##
import sys
import math

## Ros ##
import rospy

## MoveIt and TF ##
import moveit_commander
from tf.transformations import quaternion_from_euler

## Import msgs and srvs ##
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import (JointTrajectory, JointTrajectoryPoint)
from tf2_geometry_msgs import tf2_geometry_msgs
from moveit_msgs.msg import (CollisionObject, DisplayTrajectory, Grasp, PlaceLocation)
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import Quaternion

def openGripper(posture):
	"""Open the gripper

	Parameters
	----------
	posture : trajectory_msgs.msg.JointTrajectory
		Gripper posture.
	"""

	## - BEGIN_SUB_TUTORIAL open_gripper- ##
  	## Add both finger joints of panda robot ##
  	posture.joint_names = [str for i in range(2)]
  	posture.joint_names[0] = "panda_finger_joint1"
  	posture.joint_names[1] = "panda_finger_joint2"

  	## Set them as open, wide enough for the object to fit. ##
  	posture.points = [JointTrajectoryPoint()]
  	posture.points[0].positions = [float for i in range(2)]
  	posture.points[0].positions[0] = 0.04
  	posture.points[0].positions[1] = 0.04
  	posture.points[0].time_from_start = rospy.Duration(0.5)
  	## - END_SUB_TUTORIAL - ##

def closedGripper(posture):
  	"""Close the gripper.

	Parameters
	----------
	posture : trajectory_msgs.msg.JointTrajectory
		Gripper posture.
	"""

	## - BEGIN_SUB_TUTORIAL open_gripper - ##
  	## Add both finger joints of panda robot. ##
  	posture.joint_names = [str for i in range(2)]
  	posture.joint_names[0] = "panda_finger_joint1"
  	posture.joint_names[1] = "panda_finger_joint2"

  	## Set them as closed. ##
  	posture.points = [JointTrajectoryPoint()]
  	posture.points[0].positions = [float for i in range(2)]
  	posture.points[0].positions[0] = 0.00
  	posture.points[0].positions[1] = 0.00
  	posture.points[0].time_from_start = rospy.Duration(0.5)
	## - END_SUB_TUTORIAL - ##

def pick(move_group):
  	"""Place an object.

  	Parameters
  	----------
  	Group : moveit_commander.RobotCommander
		Moveit_commander move group.
  	"""

	## - BEGIN_SUB_TUTORIAL pick1 - ##
  	## Create a vector of grasps to be attempted, currently only creating single grasp. ##
  	# This is essentially useful when using a grasp generator to generate and test multiple grasps.
  	grasps = [Grasp() for i in range(1)]

  	## Setting grasp pose ##
  	# This is the pose of panda_link8. |br|
  	# From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
  	# of the cube). |br|
  	# Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
  	# extra padding)
  	grasps[0].grasp_pose.header.frame_id = "panda_link0"
	orientation = quaternion_from_euler(-math.pi / 2, -math.pi / 4, -math.pi / 2)
  	grasps[0].grasp_pose.pose.orientation.x = orientation[0]
  	grasps[0].grasp_pose.pose.orientation.y = orientation[1]
  	grasps[0].grasp_pose.pose.orientation.z = orientation[2]
  	grasps[0].grasp_pose.pose.orientation.w = orientation[3]
  	grasps[0].grasp_pose.pose.position.x = 0.415
  	grasps[0].grasp_pose.pose.position.y = 0
  	grasps[0].grasp_pose.pose.position.z = 0.5

  	## Setting pre-grasp approach ##
  	# Defined with respect to frame_id
  	grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0"
  	grasps[0].pre_grasp_approach.direction.vector.x = 1.0 # Direction is set as positive x axis
  	grasps[0].pre_grasp_approach.min_distance = 0.095
  	grasps[0].pre_grasp_approach.desired_distance = 0.115

  	## Setting post-grasp retreat ##
  	# Defined with respect to frame_id
  	grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0"
  	grasps[0].post_grasp_retreat.direction.vector.z = 1.0 # Direction is set as positive z axis
  	grasps[0].post_grasp_retreat.min_distance = 0.1
  	grasps[0].post_grasp_retreat.desired_distance = 0.25

  	## Setting posture of eef before grasp ##
  	openGripper(grasps[0].pre_grasp_posture)
	## - END_SUB_TUTORIAL - ##

	## - BEGIN_SUB_TUTORIAL pick2 - ##
  	## Setting posture of eef during grasp ##
  	closedGripper(grasps[0].grasp_posture)

  	## Set support surface as table1. ##
  	move_group.set_support_surface_name("table1")

  	## Call pick to pick up the object using the grasps given
  	move_group.pick("object", grasps)
	## - END_SUB_TUTORIAL - ##

def place(group):
  	"""Place an object.

  	Parameters
  	----------
  	Group : moveit_commander.RobotCommander
		Moveit_commander move group.
  	"""

	## - BEGIN_SUB_TUTORIAL place - ##
  	# TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  	# location in
  	# verbose mode." This is a known issue and we are working on fixing it. |br|
  	# Create a vector of placings to be attempted, currently only creating single place location.
  	place_location = [PlaceLocation() for i in range(1)]

  	## Setting place location pose ##
  	place_location[0].place_pose.header.frame_id = "panda_link0"
	orientation = quaternion_from_euler(0, 0, math.pi / 2)
  	place_location[0].place_pose.pose.orientation.x = orientation[0]
  	place_location[0].place_pose.pose.orientation.y = orientation[1]
  	place_location[0].place_pose.pose.orientation.z = orientation[2]
  	place_location[0].place_pose.pose.orientation.w = orientation[3]

  	## While placing it is the exact location of the center of the object. ##
  	place_location[0].place_pose.pose.position.x = 0
  	place_location[0].place_pose.pose.position.y = 0.5
  	place_location[0].place_pose.pose.position.z = 0.5

  	## Setting pre-place approach ##
  	# Defined with respect to frame_id
  	place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0"
  	place_location[0].pre_place_approach.direction.vector.z = - \
		  1.0  # Direction is set as negative z axis
  	place_location[0].pre_place_approach.min_distance = 0.095
  	place_location[0].pre_place_approach.desired_distance = 0.115

  	## Setting post-grasp retreat
  	# Defined with respect to frame_id
  	place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0"
  	place_location[0].post_place_retreat.direction.vector.y = -1.0  # Direction is set as negative y axis
  	place_location[0].post_place_retreat.min_distance = 0.1
  	place_location[0].post_place_retreat.desired_distance = 0.25

  	## Setting posture of eef after placing object
  	# Similar to the pick case
  	openGripper(place_location[0].post_place_posture)

  	## Set support surface as table2 ##
  	group.set_support_surface_name("table2")

  	## Call place to place the object using the place locations given. ##
  	group.place("object", place_location[0])
	## - END_SUB_TUTORIAL - ##

def addCollisionObjects(planning_scene_interface):
  	"""Add collision objects to the scene."""

  	## - BEGIN_SUB_TUTORIAL table1 - ##
  	# Creating Environment

	## Create vector to hold 3 collision objects. ##
	collision_objects_names = [str for i in range(3)]
	collision_object_sizes = [str for i in range(3)]
	collision_objects = [PoseStamped() for i in range(3)]

	## Add the first table where the cube will originally be kept. ##
	collision_objects_names[0] = "table1"
	collision_objects[0].header.frame_id = "panda_link0"

	## Define the primitive and its dimensions. ##
	collision_object_sizes[0] = (0.2, 0.4, 0.4) # Box size

	## Define the pose of the table. ##
	collision_objects[0].pose.position.x = 0.5
	collision_objects[0].pose.position.y = 0
	collision_objects[0].pose.position.z = 0.2
	## - END_SUB_TUTORIAL - ##

	## - BEGIN_SUB_TUTORIAL table2 - ##
	## Add the second table where we will be placing the cube. ##
	collision_objects_names[1] = "table2"
	collision_objects[1].header.frame_id = "panda_link0"

	## Define the primitive and its dimensions. ##
	collision_object_sizes[1] = (0.4, 0.2, 0.4) # Box size

	## Define the pose of the table. ##
	collision_objects[1].pose.position.x = 0
	collision_objects[1].pose.position.y = 0.5
	collision_objects[1].pose.position.z = 0.2

	## Define the object that we will be manipulating ##
	collision_objects_names[2] = "object"
	collision_objects[2].header.frame_id = "panda_link0"

	## Define the primitive and its dimensions. ##
	collision_object_sizes[2] = (0.02, 0.02, 0.2) # Box size

	## Define the pose of the object. ##
	collision_objects[2].pose.position.x = 0.5
	collision_objects[2].pose.position.y = 0
	collision_objects[2].pose.position.z = 0.5
	## - END_SUB_TUTORIAL - ##

	## Add collision objects to scene ##
	for (name, pose, size) in zip(collision_objects_names, collision_objects, collision_object_sizes):
		planning_scene_interface.add_box(name=name, pose=pose, size=size)

def main(arg):
	"""Main function"""

	## Initialize ros node ##
	rospy.init_node("panda_arm_pick_place")

	## initialize moveit_commander ##
	moveit_commander.roscpp_initialize(arg)

	## Connect to moveit services ##
	rospy.loginfo(	  "Conneting moveit default moveit \'apply_planning_scene\' service.")
	rospy.wait_for_service('apply_planning_scene')
	try:
		planning_scene_srv = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
		rospy.loginfo("Moveit \'apply_planning_scene\' service found!")
	except rospy.ServiceException as e:
		rospy.logerr(
			  "Moveit \'apply_planning_scene\' service initialization failed: %s" % e)
		shutdown_msg = "Shutting down %s node because %s service connection failed." % (rospy.get_name(), planning_scene_srv.resolved_name)
		rospy.logerr(shutdown_msg)
		sys.exit(0)

	## Create robot commander ##
	robot = moveit_commander.RobotCommander(
		robot_description="robot_description", ns="/")
	rospy.logdebug("Robot Groups: %s", robot.get_group_names())

	## Create scene commanders ##
	# Used to get information about the world and update the robot
	# its understanding of the world.
	move_group = robot.get_group("panda_arm")
	planning_scene_interface = moveit_commander.PlanningSceneInterface(ns="/")

	## Specify the planner we want to use ##
	move_group.set_planner_id("TRRTkConfigDefault")

	## Create a `DisplayTrajectory`_ ROS publisher to display the plan in RVIZ ##
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
												   DisplayTrajectory,
												   queue_size=20)

    ## Wait a bit for ROS to initialize the planning_scene_interface ##
	rospy.sleep(1.0)

	## Add collision objects ##
	addCollisionObjects(planning_scene_interface)

	## Wait a bit ##
	rospy.sleep(1.0)

	## Pick ##
	pick(move_group)

	## Wait a bit ##
	rospy.sleep(1.0)

	## Place ##
	place(move_group)

	## Spin till shutdown ##
	rospy.spin()

if __name__ == "__main__":
    main(sys.argv[1:])