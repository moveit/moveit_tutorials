#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, KU Leuven
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
#  * Neither the name of KU Leuven nor the names of its
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
# Author: Jeroen De Maeyer

from __future__ import print_function

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import visualization_msgs.msg
import std_msgs.msg

from math import pi
from tf.transformations import quaternion_from_euler, quaternion_multiply
from six.moves import input  # Python 3 compatible alternative for raw_input

# define some colours for convenience
COLOR_RED = std_msgs.msg.ColorRGBA(1.0, 0.0, 0.0, 1.0)
COLOR_GREEN = std_msgs.msg.ColorRGBA(0.0, 1.0, 0.0, 1.0)
COLOR_TRANSLUCENT = std_msgs.msg.ColorRGBA(0.0, 0.0, 0.0, 0.5)


## BEGIN_SUB_TUTORIAL setup
##
## Setup a RobotCommander and a MoveGroupCommander, see `move group Python interface`_ tutorial for more details.
## Everything is wrappen in a class to make it easily reusable.
##
class ConstrainedPlanningTutorial(object):
    """ Wrapper class for the tutorial. """

    def __init__(self, group_name="panda_arm"):
        """ Start the ROS node and create object to handle the robot and the planning scene. """

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ompl_constrained_planning_example", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.scene = moveit_commander.PlanningSceneInterface()

        # Create a publisher to visualize the position constraints in Rviz
        self.marker_publisher = rospy.Publisher(
            "/visualization_marker",
            visualization_msgs.msg.Marker,
            queue_size=20,
        )
        rospy.sleep(0.5)  # publisher needs some time to connect Rviz
        self.remove_all_markers()
        self.marker_id_counter = 0  # give each marker a unique idea

        # Save some commenly used variables in the setup class
        self.ref_link = self.move_group.get_pose_reference_frame()
        self.ee_link = self.move_group.get_end_effector_link()
        self.obstacle_name = "obstacle"

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL start_state
    ## Instead of using the current robot state as start state, we use a fixed state for the panda robot defined in the srdf config file.
    ## The :code:`get_named_target_values` returns a dictionary with joint names and values for the "ready" position.
    def create_start_state(self, named_target="ready"):
        ready = self.move_group.get_named_target_values(named_target)

        # Now create a robot state from these joint positions
        joint_state = sensor_msgs.msg.JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = self.move_group.get_pose_reference_frame()
        joint_state.name = [key for key in ready.keys()]
        joint_state.position = [val for val in ready.values()]

        state = moveit_msgs.msg.RobotState()
        state.joint_state = joint_state

        return state

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL pose_goal
    ##
    ## To keep things simple, we use the current end-effector pose to quickly create a reasonable goal.
    ## We also visualize the start and goal position of the end-effector in Rviz with a simple sphere.
    ## We assume that when the `demo.launch` file for the panda robot is launched, the robot is in the "ready" position.
    def create_pose_goal(self):
        self.move_group.clear_pose_targets()
        pose = self.move_group.get_current_pose()

        self.display_sphere(pose.pose, color=COLOR_RED)

        pose.pose.position.y += 0.3
        pose.pose.position.z -= 0.3

        self.display_sphere(pose.pose)

        return pose

    ## For the second planning problem with the tilted plane, we need to create a pose goal that lies in this plane.
    ## The plane is tilted by 45 degrees, so moving an equal amount in the y and z direction should be ok.
    def create_pose_goal_in_plane(self):
        self.move_group.clear_pose_targets()
        pose = self.move_group.get_current_pose()

        self.display_sphere(pose.pose, color=COLOR_RED)

        pose.pose.position.x += 0.2
        pose.pose.position.y += 0.3
        pose.pose.position.z -= 0.3

        self.display_sphere(pose.pose)

        return pose

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL ori_con
    ## First we create simple orientation constraints on the current end-effector link (:code:`self.ee_link = "panda_link8"`).
    def create_orientation_constraints(self):
        ocm = moveit_msgs.msg.OrientationConstraint()
        ocm.header.frame_id = self.ref_link
        ocm.link_name = self.ee_link

        current_pose = self.move_group.get_current_pose()
        ocm.orientation = current_pose.pose.orientation

        print("Quaternion desired: ", ocm.orientation)

        # quat = quaternion_from_euler(pi / 4, 0, 0)
        # ocm.orientation = quat
        # ocm.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)
        # Allow rotation of 45 degrees around the x and y axis
        ocm.absolute_x_axis_tolerance = 1  # rotation "to the front"
        ocm.absolute_y_axis_tolerance = 1  # rotation "to the side"
        ocm.absolute_z_axis_tolerance = pi  # rotation around vertical axis

        # ocm.parameterization = moveit_msgs.msg.OrientationConstraint.XYZ_EULER_ANGLES
        ocm.parameterization = moveit_msgs.msg.OrientationConstraint.ROTATION_VECTOR
        # XYZ_EULER_ANGLES = 0
        # ROTATION_VECTOR = 1
        # The tilt constraint is the only constraint
        ocm.weight = 1

        return ocm

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL pos_con
    ## First we create simple box constraints on the current end-effector link (:code:`self.ee_link = "panda_link8"`).
    def create_simple_box_constraints(self):
        pcm = moveit_msgs.msg.PositionConstraint()
        pcm.header.frame_id = self.ref_link
        pcm.link_name = self.ee_link

        cbox = shape_msgs.msg.SolidPrimitive()
        cbox.type = shape_msgs.msg.SolidPrimitive.BOX
        cbox.dimensions = [0.1, 0.4, 0.4]
        pcm.constraint_region.primitives.append(cbox)

        current_pose = self.move_group.get_current_pose()

        cbox_pose = geometry_msgs.msg.Pose()
        cbox_pose.position.x = current_pose.pose.position.x
        cbox_pose.position.y = 0.15
        cbox_pose.position.z = 0.45
        cbox_pose.orientation.w = 1.0
        pcm.constraint_region.primitive_poses.append(cbox_pose)

        # display the constraints in rviz
        self.display_box(cbox_pose, cbox.dimensions)

        return pcm

    def create_box_constraints(self):
        pcm = moveit_msgs.msg.PositionConstraint()
        pcm.header.frame_id = self.ref_link
        pcm.link_name = self.ee_link

        cbox = shape_msgs.msg.SolidPrimitive()
        cbox.type = shape_msgs.msg.SolidPrimitive.BOX
        cbox.dimensions = [0.4, 0.1, 0.4]
        pcm.constraint_region.primitives.append(cbox)

        current_pose = self.move_group.get_current_pose()

        cbox_pose = geometry_msgs.msg.Pose()
        cbox_pose.position.x = 0.4
        cbox_pose.position.y = current_pose.pose.position.y
        cbox_pose.position.z = 0.45
        cbox_pose.orientation.w = 1.0
        pcm.constraint_region.primitive_poses.append(cbox_pose)

        # display the constraints in rviz
        self.display_box(cbox_pose, cbox.dimensions)

        return pcm

    ## If you make a box really thin along one dimension, you get something plane like.
    ## We create a plane perpendicular to the y-axis and tilt it by 45 degrees in the function below.
    ## When solving the problem, you can tell the planner to model this really thin box as an equality constraint.
    ## The magic number :code:`0.0005` is explained later.
    def create_plane_constraints(self):
        pcm = moveit_msgs.msg.PositionConstraint()
        pcm.header.frame_id = self.ref_link
        pcm.link_name = self.ee_link

        cbox = shape_msgs.msg.SolidPrimitive()
        cbox.type = shape_msgs.msg.SolidPrimitive.BOX
        cbox.dimensions = [1.0, 0.0005, 1.0]
        pcm.constraint_region.primitives.append(cbox)

        current_pose = self.move_group.get_current_pose()

        cbox_pose = geometry_msgs.msg.Pose()
        cbox_pose.position.x = current_pose.pose.position.x
        cbox_pose.position.y = current_pose.pose.position.y
        cbox_pose.position.z = current_pose.pose.position.z

        # turn the constraint region 45 degrees around the x-axis.
        quat = quaternion_from_euler(pi / 4, 0, 0)
        cbox_pose.orientation.x = quat[0]
        cbox_pose.orientation.y = quat[1]
        cbox_pose.orientation.z = quat[2]
        cbox_pose.orientation.w = quat[3]
        pcm.constraint_region.primitive_poses.append(cbox_pose)

        # display the constraints in rviz
        self.display_box(cbox_pose, cbox.dimensions)

        return pcm

    ## Building on the previous constraint, we can make it a line, by also reducing the dimension of the box in the x-direction.
    def create_line_constraints(self):
        pcm = moveit_msgs.msg.PositionConstraint()
        pcm.header.frame_id = self.ref_link
        pcm.link_name = self.ee_link

        cbox = shape_msgs.msg.SolidPrimitive()
        cbox.type = shape_msgs.msg.SolidPrimitive.BOX
        cbox.dimensions = [0.0005, 0.0005, 1.0]
        pcm.constraint_region.primitives.append(cbox)

        current_pose = self.move_group.get_current_pose()

        cbox_pose = geometry_msgs.msg.Pose()
        cbox_pose.position.x = current_pose.pose.position.x
        cbox_pose.position.y = current_pose.pose.position.y
        cbox_pose.position.z = current_pose.pose.position.z
        quat = quaternion_from_euler(pi / 4, 0, 0)
        cbox_pose.orientation.x = quat[0]
        cbox_pose.orientation.y = quat[1]
        cbox_pose.orientation.z = quat[2]
        cbox_pose.orientation.w = quat[3]
        pcm.constraint_region.primitive_poses.append(cbox_pose)

        # display the constraints in rviz
        self.display_box(cbox_pose, cbox.dimensions)

        return pcm

    ## END_SUB_TUTORIAL

    def display_box(self, pose, dimensions):
        """ Utility function to visualize position constraints. """
        assert len(dimensions) == 3

        # setup cube / box marker type
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "/"
        marker.id = self.marker_id_counter
        marker.type = visualization_msgs.msg.Marker.CUBE
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.color = COLOR_TRANSLUCENT
        marker.header.frame_id = self.ref_link

        # fill in user input
        marker.pose = pose
        marker.scale.x = dimensions[0]
        marker.scale.y = dimensions[1]
        marker.scale.z = dimensions[2]

        # publish it!
        self.marker_publisher.publish(marker)
        self.marker_id_counter += 1

    def display_sphere(self, pose, radius=0.05, color=COLOR_GREEN):
        """ Utility function to visualize the goal pose"""

        # setup sphere marker type
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "/"
        marker.id = self.marker_id_counter
        marker.type = visualization_msgs.msg.Marker.SPHERE
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.header.frame_id = self.ref_link

        # fill in user input
        marker.color = color
        marker.pose = pose
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = radius

        # publish it!
        self.marker_publisher.publish(marker)
        self.marker_id_counter += 1

    def remove_all_markers(self):
        """ Utility function to remove all Markers that we potentially published in a previous run of this script. """
        # setup cube / box marker type
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "/"
        # marker.id = 0
        # marker.type = visualization_msgs.msg.Marker.CUBE
        marker.action = visualization_msgs.msg.Marker.DELETEALL
        self.marker_publisher.publish(marker)

    def add_obstacle(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.ref_link
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.6
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.6
        self.obstacle_name = "obstacle"
        self.scene.add_box(self.obstacle_name, box_pose, size=(0.2, 0.4, 0.1))

        # Give the planning scene some time to update
        rospy.sleep(0.5)

    def remove_obstacle(self):
        self.scene.remove_world_object(self.obstacle_name)
        # Give the planning scene some time to update
        rospy.sleep(0.5)

    def create_pose_goal_rotated(self):
        self.move_group.clear_pose_targets()
        pose = self.move_group.get_current_pose()

        self.display_sphere(pose.pose, color=COLOR_RED)

        pose.pose.position.x += 0.2
        pose.pose.position.y += 0.0
        pose.pose.position.z -= 0.3

        # rotate the pose around the x axis?
        quat_ee = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
        quat_rotate = quaternion_from_euler(-pi / 2, 0, 0)
        quat_new = quaternion_multiply(quat_rotate, quat_ee)

        pose.pose.orientation.x = quat_new[0]
        pose.pose.orientation.y = quat_new[1]
        pose.pose.orientation.z = quat_new[2]
        pose.pose.orientation.w = quat_new[3]

        self.display_sphere(pose.pose)

        return pose

    def create_pose_goal_under_obstacle(self):
        self.move_group.clear_pose_targets()
        pose = self.move_group.get_current_pose()

        self.display_sphere(pose.pose, color=COLOR_RED)

        pose.pose.position.x += 0.2
        pose.pose.position.y += 0.0
        pose.pose.position.z -= 0.3

        # rotate the pose around the x axis?
        quat_ee = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
        quat_rotate = quaternion_from_euler(-pi / 2, 0, 0)
        # quat_new = quaternion_multiply(quat_rotate, quat_ee)
        quat_new = quat_ee

        pose.pose.orientation.x = quat_new[0]
        pose.pose.orientation.y = quat_new[1]
        pose.pose.orientation.z = quat_new[2]
        pose.pose.orientation.w = quat_new[3]

        self.display_sphere(pose.pose)

        return pose

    def create_vertical_plane_constraints(self):
        pcm = moveit_msgs.msg.PositionConstraint()
        pcm.header.frame_id = self.ref_link
        pcm.link_name = self.ee_link

        cbox = shape_msgs.msg.SolidPrimitive()
        cbox.type = shape_msgs.msg.SolidPrimitive.BOX
        cbox.dimensions = [1.0, 0.0005, 1.0]
        pcm.constraint_region.primitives.append(cbox)

        current_pose = self.move_group.get_current_pose()

        cbox_pose = geometry_msgs.msg.Pose()
        cbox_pose.position.x = current_pose.pose.position.x
        cbox_pose.position.y = current_pose.pose.position.y
        cbox_pose.position.z = current_pose.pose.position.z

        cbox_pose.orientation.w = 1.0
        pcm.constraint_region.primitive_poses.append(cbox_pose)

        # display the constraints in rviz
        self.display_box(cbox_pose, cbox.dimensions)

        return pcm


def run_tutorial():
    ## BEGIN_SUB_TUTORIAL main
    ##
    ## Now we can use the different pieces we just defined to solve some planning problems.
    # First create an instance of the Tutorial class
    tutorial = ConstrainedPlanningTutorial()

    # Copy move group variable for readability
    move_group = tutorial.move_group

    ## Create the first planning problem
    start_state = tutorial.create_start_state()
    path_constraints = moveit_msgs.msg.Constraints()

    # Now wait for the user (you) to press enter before doing trying the position constraints.
    print("============ Press enter to start the box constrained planning problem.")
    input()

    pose_goal = tutorial.create_pose_goal_rotated()

    # Let's try the simple box constraints first!
    pcm = tutorial.create_box_constraints()

    # We need two wrap the constraints in a generic `Constraints` message.
    path_constraints = moveit_msgs.msg.Constraints()
    path_constraints.position_constraints.append(pcm)

    # Now we have everything we need to configure and solve a planning problem
    move_group.set_start_state(start_state)
    move_group.set_pose_target(pose_goal)

    # Don't forget the path constraints! That's the whole point of this tutorial.
    move_group.set_path_constraints(path_constraints)

    # And let the planner find a solution.
    # The move_group node should automatically visualize the solution in Rviz if a path is found.
    move_group.plan()

    # Clear the path constraints for our next experiment
    move_group.clear_path_constraints()

    # Now wait for the user (you) to press enter before doing trying the position constraints.
    print(
        "============ Press enter to continue with the plane constrained planning problem."
    )
    input()
    # remove all markers in Rviz before starting the next tutorial
    tutorial.remove_all_markers()

    ## In the second problem we plan with the end-effector constrained to a plane.
    ## Remember we created a special pose goal that lies in the constraint plane.
    ## We did that because any goal or start state should also satisfy the path constraints.
    ##
    ## Solving the problem using equality constraints is a bit more complicated. (Or should I say, hacky?)
    ## We need to tell the planner explicitly that we want to use equality constraints for the small dimensions.
    ## This is achieved by setting the name of the constraint to :code:`"use_equality_constraints"`.
    ## In addition, any dimension of the box below a treshold of :code:`0.001` will be considered an equality constraint.
    ## However, if we make it too small, the box will be thinner that the tolerance used by OMPL to evaluate constraints (:code:`1e-4` by default).
    ## MoveIt will use the stricter tolerance (the box width) to check the constraints, and many states will appear invalid.
    ## That's where the magic number :code:`0.0005` comes from, it is between :code:`0.00001` and :code:`0.001`.
    # pose_goal = tutorial.create_pose_goal_in_plane()
    # pcm = tutorial.create_plane_constraints()  # this function uses the 'magic' number
    pose_goal = tutorial.create_pose_goal_rotated()
    pcm = tutorial.create_vertical_plane_constraints()

    path_constraints = moveit_msgs.msg.Constraints()
    path_constraints.position_constraints.append(pcm)

    path_constraints.name = "use_equality_constraints"

    # And again, configure and solve the planning problem.
    move_group.set_start_state(start_state)
    move_group.set_pose_target(pose_goal)
    move_group.set_path_constraints(path_constraints)
    move_group.set_planning_time(1)
    move_group.plan()
    move_group.clear_path_constraints()

    print(
        "============ Press enter to continue with the plane constrained planning problem with obstacle."
    )
    input()

    tutorial.remove_all_markers()
    tutorial.add_obstacle()

    pose_goal = tutorial.create_pose_goal_under_obstacle()
    pcm = tutorial.create_vertical_plane_constraints()

    # We need two wrap the constraints in a generic `Constraints` message.
    path_constraints = moveit_msgs.msg.Constraints()
    path_constraints.position_constraints.append(pcm)
    path_constraints.name = "use_equality_constraints"

    move_group.set_start_state(start_state)
    move_group.set_pose_target(pose_goal)
    move_group.set_path_constraints(path_constraints)
    move_group.set_planning_time(15)
    move_group.plan()
    move_group.clear_path_constraints()

    print(
        "============ Press enter to continue with the line constrained planning problem."
    )
    input()
    tutorial.remove_all_markers()
    tutorial.remove_obstacle()
    ## Finally we can also plan along the line.
    pose_goal = tutorial.create_pose_goal()

    pcm = tutorial.create_line_constraints()

    path_constraints = moveit_msgs.msg.Constraints()
    path_constraints.position_constraints.append(pcm)

    path_constraints.name = "use_equality_constraints"

    move_group.set_start_state(start_state)
    move_group.set_pose_target(pose_goal)
    move_group.set_path_constraints(path_constraints)
    move_group.set_planning_time(1)
    move_group.plan()
    move_group.clear_path_constraints()

    # print("============ Press enter to continue with the box constrained planning problem with obstacle.")
    # input()

    # tutorial.remove_all_markers()
    # tutorial.add_obstacle()

    # pose_goal = tutorial.create_pose_goal_under_obstacle()
    # pcm = tutorial.create_box_constraints()

    # # We need two wrap the constraints in a generic `Constraints` message.
    # path_constraints = moveit_msgs.msg.Constraints()
    # path_constraints.position_constraints.append(pcm)

    # move_group.set_start_state(start_state)
    # move_group.set_pose_target(pose_goal)
    # move_group.set_path_constraints(path_constraints)
    # move_group.set_planning_time(60)
    # move_group.plan()
    # move_group.clear_path_constraints()

    # Now wait for the user (you) to press enter before doing trying the position constraints.
    print(
        "============ Press enter to continue with the orientation constrained planning problem."
    )
    input()
    tutorial.remove_all_markers()
    tutorial.remove_obstacle()

    pose_goal = tutorial.create_pose_goal()

    ocm = tutorial.create_orientation_constraints()

    path_constraints = moveit_msgs.msg.Constraints()
    path_constraints.orientation_constraints.append(ocm)
    pose_goal = tutorial.create_pose_goal_under_obstacle()
    # pcm = tutorial.create_vertical_plane_constraints()

    move_group.set_start_state(start_state)
    move_group.set_pose_target(pose_goal)
    move_group.set_path_constraints(path_constraints)
    move_group.set_planning_time(1)
    move_group.plan()
    move_group.clear_path_constraints()

    print(
        "============ Press enter to continue with the orientation constrained planning problem with obstacle."
    )
    input()

    tutorial.remove_all_markers()
    tutorial.add_obstacle()

    pose_goal = tutorial.create_pose_goal_under_obstacle()
    ocm = tutorial.create_orientation_constraints()

    # We need two wrap the constraints in a generic `Constraints` message.
    path_constraints = moveit_msgs.msg.Constraints()
    path_constraints.orientation_constraints.append(ocm)

    move_group.set_start_state(start_state)
    move_group.set_pose_target(pose_goal)
    move_group.set_path_constraints(path_constraints)
    move_group.set_planning_time(10)
    move_group.plan()
    move_group.clear_path_constraints()

    # print("============ Press enter to continue with the pose constrained planning problem.")
    # input()
    # tutorial.remove_all_markers()
    # tutorial.remove_obstacle()

    # pcm = tutorial.create_vertical_plane_constraints()
    # ocm = tutorial.create_orientation_constraints()

    # path_constraints = moveit_msgs.msg.Constraints()
    # path_constraints.orientation_constraints.append(ocm)
    # path_constraints.position_constraints.append(pcm)
    # pose_goal = tutorial.create_pose_goal_under_obstacle()
    # # pcm = tutorial.create_vertical_plane_constraints()

    # move_group.set_start_state(start_state)
    # move_group.set_pose_target(pose_goal)
    # move_group.set_path_constraints(path_constraints)
    # move_group.set_planning_time(1)
    # move_group.plan()
    # move_group.clear_path_constraints()

    # print("============ Press enter to continue with the pose constrained planning problem with obstacle.")
    # input()
    # tutorial.remove_all_markers()
    # tutorial.add_obstacle()

    # pcm = tutorial.create_vertical_plane_constraints()
    # ocm = tutorial.create_orientation_constraints()

    # path_constraints = moveit_msgs.msg.Constraints()
    # path_constraints.orientation_constraints.append(ocm)
    # path_constraints.position_constraints.append(pcm)
    # pose_goal = tutorial.create_pose_goal_under_obstacle()
    # # pcm = tutorial.create_vertical_plane_constraints()

    # move_group.set_start_state(start_state)
    # move_group.set_pose_target(pose_goal)
    # move_group.set_path_constraints(path_constraints)
    # move_group.set_planning_time(60)
    # move_group.plan()
    # move_group.clear_path_constraints()

    print("Done!")
    ## END_SUB_TUTORIAL


def main():
    """ Catch interupt when the user presses `ctrl-c`. """
    try:
        run_tutorial()
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

## BEGIN_TUTORIAL
##
## Setup
## *****
## CALL_SUB_TUTORIAL setup
##
## Create a planning problem
## ***************************
## CALL_SUB_TUTORIAL start_state
## CALL_SUB_TUTORIAL pose_goal
##
## Create position constraints
## ***************************
## CALL_SUB_TUTORIAL pos_con
##
## Create orientation constraints
## ***************************
## CALL_SUB_TUTORIAL ori_con
##
## Finally, solve a planning problem!
## **********************************
## CALL_SUB_TUTORIAL main
##
## END_TUTORIAL
