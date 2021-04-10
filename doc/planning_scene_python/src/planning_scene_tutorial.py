#!/usr/bin/env python
import sys

import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
import moveit.core
from moveit.core import kinematic_constraints, collision_detection, robot_model
from moveit.core.planning_scene import PlanningScene


def main():
    rospy.init_node("planning_scene_tutorial")
    moveit_commander.roscpp_initialize(sys.argv)

    # BEGIN_TUTORIAL
    #
    # Setup
    # ^^^^^
    #
    # The :moveit_python:`core.planning_scene.PlanningScene` class can be easily setup and
    # configured using a URDF and
    # SRDF. This is, however, not the recommended way to instantiate a
    # PlanningScene. At the time of writing there are not yet python bindings
    # for the PlanningSceneMonitoir, which is is the recommended method to
    # create and maintain the current planning scene
    # using data from the robot's joints and the sensors on the robot. In
    # this tutorial, we will instantiate a PlanningScene class directly,
    # but this method of instantiation is only intended for illustration.

    urdf_path = '/opt/ros/noetic/share/moveit_resources_panda_description/urdf/panda.urdf'
    srdf_path = '/opt/ros/noetic/share/moveit_resources_panda_moveit_config/config/panda.srdf'
    kinematic_model = moveit.core.load_robot_model(urdf_path, srdf_path)
    planning_scene = PlanningScene(kinematic_model, collision_detection.World())
    current_state = planning_scene.getCurrentStateNonConst()
    joint_model_group = current_state.getJointModelGroup("panda_arm")

    # Collision Checking
    # ^^^^^^^^^^^^^^^^^^
    #
    # Self-collision checking
    # ~~~~~~~~~~~~~~~~~~~~~~~
    #
    # The first thing we will do is check whether the robot in its
    # current state is in *self-collision*, i.e. whether the current
    # configuration of the robot would result in the robot's parts
    # hitting each other. To do this, we will construct a
    # :moveit_python:`core.collision_detection.CollisionRequest` object and a
    # :moveit_python:`core.collision_detection.CollisionResult` object and pass them
    # into the collision checking function. Note that the result of
    # whether the robot is in self-collision or not is contained within
    # the result. Self collision checking uses an *unpadded* version of
    # the robot, i.e. it directly uses the collision meshes provided in
    # the URDF with no extra padding added on.

    collision_request = collision_detection.CollisionRequest()
    collision_result = collision_detection.CollisionResult()
    planning_scene.checkSelfCollision(collision_request, collision_result)
    rospy.loginfo(f"Test 1: Current state is {'in' if collision_result.collision else 'not in'} self collision")

    # Now, we can get contact information for any collisions that might
    # have happened at a given configuration of the Panda arm. We can ask
    # for contact information by filling in the appropriate field in the
    # collision request and specifying the maximum number of contacts to
    # be returned as a large number.

    collision_request.contacts = True
    collision_request.max_contacts = 1000

    collision_result.clear()
    planning_scene.checkSelfCollision(collision_request, collision_result)
    for (first_name, second_name), contacts in collision_result.contacts.items():
        rospy.loginfo(f"Contact between {first_name} and {second_name}")

    # Change the state
    # ~~~~~~~~~~~~~~~~
    #
    # Now, let's change the current state of the robot. The planning
    # scene maintains the current state internally. We can get a
    # reference to it and change it and then check for collisions for the
    # new robot configuration. Note in particular that we need to clear
    # the collision_result before making a new collision checking
    # request.

    current_state.setToRandomPositions(joint_model_group)

    collision_result.clear()
    planning_scene.checkSelfCollision(collision_request, collision_result)
    rospy.loginfo(f"Test 2: Current state is {'in' if collision_result.collision else 'not in'} self collision")

    # Checking for a group
    # ~~~~~~~~~~~~~~~~~~~~
    #
    # Now, we will do collision checking only for the hand of the
    # Panda, i.e. we will check whether there are any collisions between
    # the hand and other parts of the body of the robot. We can ask
    # for this specifically by adding the group name "hand" to the
    # collision request.

    collision_request.group_name = "hand"
    current_state.setToRandomPositions(joint_model_group)
    collision_result.clear()
    planning_scene.checkSelfCollision(collision_request, collision_result)
    rospy.loginfo(f"Test 3: Current state is {'in' if collision_result.collision else 'not in'} self collision")

    # Getting Contact Information
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #
    # First, manually set the Panda arm to a position where we know
    # internal (self) collisions do happen. Note that this state is now
    # actually outside the joint limits of the Panda, which we can also
    # check for directly.

    joint_values = np.array([0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0])
    joint_model_group = current_state.getJointModelGroup("panda_arm")
    current_state.setJointGroupPositions(joint_model_group, joint_values)
    rospy.loginfo(
        f"Test 4: Current state is {'valid' if current_state.satisfiesBounds(joint_model_group) else 'not valid'}")

    # Modifying the Allowed Collision Matrix
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #
    # The :moveit_python:`core.collision_detection.AllowedCollisionMatrix` (ACM)
    # provides a mechanism to tell the collision world to ignore
    # collisions between certain object: both parts of the robot and
    # objects in the world. We can tell the collision checker to ignore
    # all collisions between the links reported above, i.e. even though
    # the links are actually in collision, the collision checker will
    # ignore those collisions and return not in collision for this
    # particular state of the robot.
    #
    # Note also in this example how we are making copies of both the
    # allowed collision matrix and the current state and passing them in
    # to the collision checking function.

    acm = planning_scene.getAllowedCollisionMatrix()
    copied_state = planning_scene.getCurrentState()

    # for (it2 = collision_result.contacts.begin() it2 != collision_result.contacts.end() + +it2)
    for (first_name, second_name), contacts in collision_result.contacts:
        acm.setEntry(first_name, second_name, True)
    collision_result.clear()
    planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm)
    rospy.loginfo(f"Test 6: Current state is {'in' if collision_result.collision else 'not in'} self collision")

    # Full Collision Checking
    # ~~~~~~~~~~~~~~~~~~~~~~~
    #
    # While we have been checking for self-collisions, we can use the
    # checkCollision functions instead which will check for both
    # self-collisions and for collisions with the environment (which is
    # currently empty).  This is the set of collision checking
    # functions that you will use most often in a planner. Note that
    # collision checks with the environment will use the padded version
    # of the robot. Padding helps in keeping the robot further away
    # from obstacles in the environment.
    collision_result.clear()
    planning_scene.checkCollision(collision_request, collision_result, copied_state, acm)
    rospy.loginfo(f"Test 7: Current state is {'in' if collision_result.collision else 'not in'} self collision")

    # Constraint Checking
    # ^^^^^^^^^^^^^^^^^^^
    #
    # The PlanningScene class also includes easy to use function calls for checking constraints. The constraints can be
    # of two types: (a) constraints constructed using :moveit_python:`core.kinematic_constraints`, or (b) user defined
    # constraints specified through a callback. We will first look at an example with a simple
    # KinematicConstraint.
    #
    # Checking Kinematic Constraints
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #
    # We will first define a simple position and orientation constraint
    # on the end-effector of the panda_arm group of the Panda robot. Note the
    # use of convenience functions for filling up the constraints

    end_effector_name = joint_model_group.getLinkModelNames()[-1]

    desired_pose = PoseStamped()
    desired_pose.pose.orientation.w = 1.0
    desired_pose.pose.position.x = 0.3
    desired_pose.pose.position.y = -0.185
    desired_pose.pose.position.z = 0.5
    desired_pose.header.frame_id = "panda_link0"
    goal_constraint = kinematic_constraints.constructGoalConstraints(end_effector_name, desired_pose)

    # Now, we can check a state against this constraint using the
    # isStateConstrained functions in the PlanningScene class.
    copied_state.setToRandomPositions(joint_model_group)
    copied_state.update()
    constrained = planning_scene.isStateConstrained(copied_state, goal_constraint)
    rospy.loginfo(f"Test 8: Random state is {'constrained' if constrained else 'not constrained'}")

    # There's a more efficient way of checking constraints (when you want
    # to check the same constraint over and over again, e.g. inside a
    # planner). We first construct a KinematicConstraintSet which
    # pre-processes the ROS Constraints messages and sets it up for quick
    # processing.

    kinematic_constraint_set = kinematic_constraints.KinematicConstraintSet(kinematic_model)
    kinematic_constraint_set.add(goal_constraint, planning_scene.getTransforms())
    constrained_2 = planning_scene.isStateConstrained(copied_state, kinematic_constraint_set)
    rospy.loginfo(f"Test 9: Random state is {'constrained' if constrained_2 else 'not constrained'}")

    # There's a direct way to do this using the KinematicConstraintSet
    # class.

    constraint_eval_result = kinematic_constraint_set.decide(copied_state)
    rospy.loginfo(
        f"Test 10: Random state is {'constrained' if constraint_eval_result.satisfied else 'not constrained'}")

    # Whenever isStateValid is called, three checks are conducted: (a)
    # collision checking (b) constraint checking and (c) feasibility
    # checking using the user-defined callback.

    state_valid = planning_scene.isStateValid(copied_state, kinematic_constraint_set, "panda_arm")
    rospy.loginfo(f"Test 12: Random state is {'valid' if state_valid else 'not valid'}")

    # Note that all the planners available through MoveIt and OMPL will
    # currently perform collision checking, constraint checking and
    # feasibility checking using user-defined callbacks.
    # END_TUTORIAL

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
