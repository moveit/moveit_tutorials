/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, SRI International
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of SRI International nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman, Simon Goldstein */

// BEGIN_SUB_TUTORIAL includes
// Includes
// ^^^^^^^^
//
// In C++ to use the move_group_interface it and several other files must be included.
// Both the move_group and planning_scene are accessed through their appropriate interface.
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// We also include visual_tools and several other useful message types.
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
// END_SUB_TUTORIAL

// The rviz_vizual_tools namespace is shortened for code readability.
namespace rvt = rviz_visual_tools;

class MoveGroupInterfaceTutorial
{
public:
  MoveGroupInterfaceTutorial()
  {
  }

  void setup_visualization()
  {
    // Set the location where text is diplayed to above the robot
    text_pose.translation().z() = 1.25;

    // BEGIN_SUB_TUTORIAL visualization
    // Visualization
    // ^^^^^^^^^^^^^
    //
    // Within MoveItVisualTools, remote control is an introspection tool that allows users to
    // step through a high level script via buttons and keyboard shortcuts in RViz.
    visual_tools.loadRemoteControl();

    // A variety of commands can be used to display anything from text, points or even paths.
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();
    // END_SUB_TUTORIAL
  }

  void print_basic_info()
  {
    // BEGIN_SUB_TUTORIAL basic_info
    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Move_group holds a lot of information about the robot. Accessing it is easy.
    // For example we can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We even can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    // Start the Demo
    // ^^^^^^^^^^^^^^
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    // END_SUB_TUTORIAL
  }

  void plan_joint_space_goal()
  {
    // Update the text in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // BEGIN_SUB_TUTORIAL joint_space
    // Planning to a joint-space goal
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // One of the easiest ways to move the robot is by adjusting the joint angles. To start, we'll
    // create an pointer that references the current robot's state. RobotState is the object that
    // contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the
    // plan.
    joint_group_positions[0] = -1.0;  // radians
    move_group.setJointValueTarget(joint_group_positions);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // END_SUB_TUTORIAL

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the window to move to the joint space goal");
  }

  void plan_pose_goal()
  {
    // BEGIN_SUB_TUTORIAL pose_goal
    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Instead of changing the joint angles a Pose goal chooses a final location and orientation
    // to end at. The pose contains a quaternion orientation and an xyz location. Here we set
    // the geometry_msgs::Pose target_pose to our destination and set it to the target.
    target_pose.orientation.w = 0.0;
    target_pose.orientation.x = 0.383;
    target_pose.orientation.y = -0.9240;
    target_pose.orientation.z = -0.0;
    target_pose.position.x = 0.28;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.5;

    move_group.setPoseTarget(target_pose);
    // Setting the plan with a Pose goal is identical to planning with a joint_space goal ::
    //
    //     bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //
    // END_SUB_TUTORIAL

    // Visualizing the target location axis in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishAxisLabeled(target_pose, "pose");
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the window to plan to the set Pose");

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");

    // RVis
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the window to move to the set Pose");
  }

  void plan_constrained_path()
  {
    // Generating and setting the new Pose goal
    target_pose.orientation.w = 0.0;
    target_pose.orientation.x = 0.383;
    target_pose.orientation.y = -0.9240;
    target_pose.orientation.z = -0.0;
    target_pose.position.x = -0.38;
    target_pose.position.y = -0.49;
    target_pose.position.z = 0.38;
    move_group.setPoseTarget(target_pose);

    // Rviz communication
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishAxisLabeled(target_pose, "point");
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to plan a constrained path");

    // BEGIN_SUB_TUTORIAL constrained_path
    // Planning with Path Constraints
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Path constraints can easily be specified for a link on the robot.
    // Let's specify a path constraint for our group.
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "panda_link7";
    ocm.header.frame_id = "panda_link0";
    ocm.orientation.w = 0.0;
    ocm.orientation.x = 0.383;
    ocm.orientation.y = -0.924;
    ocm.orientation.z = -0.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    // Now, set it as the path constraint for the group.
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);

    // Planning with constraints can be slow because every sample must call an inverse kinematics
    // solver. Lets increase the planning time from the default 5 seconds to be sure the planner
    // has enough time to succeed.
    move_group.setPlanningTime(10.0);

    // Now we will plan to a new pose. As a note both the start and the end of the path should
    // follow the constraint rules
    move_group.setPoseTarget(target_pose);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // When done with the path constraint be sure to clear it.
    move_group.clearPathConstraints();

    // For comparison an unconstrained path to this point can be made by planning to the already set Pose target.
    //
    // END_SUB_TUTORIAL

    // RViz
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to compare the constrained path with an unconstrained path");
  }

  void plan_unconstrained()
  {
    // This plans to the same existing goal
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (comparison) %s", success ? "" : "FAILED");

    // RViz, This time we don't delete the prevous marks to shows the two paths side by side
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
  }

  void plan_cartesian_path()
  {
    // Visualize the plan in RViz
    visual_tools.prompt("Press 'next' to plan a cartesian path");
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Planning", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Set the target Pose to our current location
    target_pose.orientation.w = 0.0;
    target_pose.orientation.x = 0.383;
    target_pose.orientation.y = -0.9240;
    target_pose.orientation.z = -0.0;
    target_pose.position.x = 0.28;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.5;

    // BEGIN_SUB_TUTORIAL cartesian_path
    // Cartesian Paths
    // ^^^^^^^^^^^^^^^
    //
    // You can plan a Cartesian path directly by specifying a list of waypoints
    // for the end-effector to go through. Note The initial pose (target_pose) does not
    // need to be added to the waypoint list but adding it can help with visualizations
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);

    target_pose.position.z -= 0.2;
    waypoints.push_back(target_pose);  // down

    target_pose.position.y -= 0.2;
    waypoints.push_back(target_pose);  // right

    target_pose.position.z += 0.2;
    target_pose.position.y += 0.2;
    target_pose.position.x -= 0.2;
    waypoints.push_back(target_pose);  // up, left, and back

    // Cartesian motions are frequently needed to be slower for actions such as approach and
    // retreat grasp motions. Here we demonstrate how to reduce the speed of the robot arm via
    // a scaling factor of the maxiumum speed of each joint. Note this is not the speed of the
    // end effector point.
    move_group.setMaxVelocityScalingFactor(0.1);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue.
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // While we didn't use the planner to create this trajectory it still can be added to a plan
    my_plan.trajectory_ = trajectory;

    // END_SUB_TUTORIAL

    // Visualize the plan in RViz, becuase there are multiple points we use a loop to show them all
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
      visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to move along the cartesian path");
  }

  void create_obstacle()
  {
    // BEGIN_SUB_TUTORIAL add_obj

    // Remember to change the velocity scaling factor back after moving to return to full speed.
    move_group.setMaxVelocityScalingFactor(1);

    // Adding or Removing Objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // First Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    // Then set the id of the object is used to identify it.
    collision_object.id = "box1";

    // Now we can Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.02;
    primitive.dimensions[1] = 0.4;
    primitive.dimensions[2] = 0.4;

    // Define a pose for the box. (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.3;
    box_pose.position.y = -0.25;
    box_pose.position.z = 0.5;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);
    object_ids.push_back(collision_object.id);

    // Now, let's add the collision object into the world.
    planning_scene_interface.addCollisionObjects(collision_objects);
    // END_SUB_TUTORIAL

    // RViz
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Add Object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' once the object appears to plan and move around the obstacle");
  }

  void plan_obstacle()
  {
    // Set the target
    target_pose.orientation.w = 0.707;
    target_pose.orientation.x = 0.000;
    target_pose.orientation.y = 0.000;
    target_pose.orientation.z = 0.707;
    target_pose.position.x = 0.5;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.5;
    move_group.setPoseTarget(target_pose);

    // Plan
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz'
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Pose Goal Avoiding Obstacle", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishAxisLabeled(target_pose, "point");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
  }

  void remove_obstacle()
  {
    ROS_INFO_NAMED("tutorial", "Remove the object from the world");

    // BEGIN_SUB_TUTORIAL remove_obj
    //
    // Now, let's remove the collision object from the world.
    planning_scene_interface.removeCollisionObjects(object_ids);
    object_ids.pop_back();
    collision_objects.pop_back();
    // **Note:** Objects in the world doesn't change how to plan or execute that plan.
    //
    // END_SUB_TUTORIAL

    // RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Object Removed", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to create a new object");
  }

  void create_parcel()
  {
    // Define a collision object ROS message
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    // The id of the object is used to identify it
    collision_object.id = "box2";

    // Define a box to add to the world
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.1;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = -0.2;
    box_pose.position.z = 0.6;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);
    object_ids.push_back(collision_object.id);

    // Add the collision object into the world
    ROS_INFO_NAMED("tutorial", "Adding an object into the world");
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Attach Object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    planning_scene_interface.addCollisionObjects(collision_objects);
  }

  void attach_parcel()
  {
    visual_tools.prompt("Press 'next' to attach the object");

    // BEGIN_SUB_TUTORIAL attach_obj
    // Attaching and detaching objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Attaching or detaching an already existing object is easy.
    //
    // To attach an object.
    move_group.attachObject(collision_objects.back().id);
    // END_SUB_TUTORIAL
  }

  void plan_parcel()
  {
    // Rviz
    visual_tools.prompt("Press 'next' to plan and move with the attached object");

    // target_pose repositioning and planning
    target_pose.position.y += 0.2;
    target_pose.position.z += 0.2;
    move_group.setPoseTarget(target_pose);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (pose goal with attached cuboid) %s", success ? "" : "FAILED");

    // Rviz
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose, "point");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.publishText(text_pose, "Pose Goal with Attached Object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
  }

  void detach_parcel()
  {
    ROS_INFO_NAMED("tutorial", "Detach the object from the robot");

    // BEGIN_SUB_TUTORIAL detach_obj
    // To detach an object.
    move_group.detachObject(collision_objects.back().id);
    // END_SUB_TUTORIAL

    // RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Object Detached", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
  }

  void remove_parcel()
  {
    visual_tools.prompt("Press 'next' to remove the object");

    // Remove the collision object from the world.
    ROS_INFO_NAMED("tutorial", "Remove the object from the world");
    planning_scene_interface.removeCollisionObjects(object_ids);
    object_ids.pop_back();
    collision_objects.pop_back();

    // Rviz
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to complete the demo");
  }

  void move()
  {
    // BEGIN_SUB_TUTORIAL movement
    // Moving the robot
    // ^^^^^^^^^^^^^^^^
    //
    // We can now use the plan we just created with movegroup to move the robot.
    move_group.execute(my_plan);
    /* move_group.move(); // calls plan and execute at the same time */

    // Keeping the startstate updated with the robot is important. If the startstate isn't where
    // the robot is then it will be unable to move.
    move_group.setStartState(*move_group.getCurrentState());
    /* start_state.setFromIK(joint_model_group); // A different way to change the start state */

    // END_SUB_TUTORIAL

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  }

private:
  // BEGIN_SUB_TUTORIAL setup
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an
  // object called the :joint_model_group:`JointModelGroup`. Throughout MoveIt the terms "planning group" and
  // "joint model group" are used interchangably.
  inline static const std::string PLANNING_GROUP = "panda_arm";
  // The :move_group_interface:`MoveGroupInterface` class can be easily setup
  // using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group =
      moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // These vectors hold additional information about the objects added to the "virtual world".
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  std::vector<std::string> object_ids;

  // The package :moveit_visual_tools:`MoveItVisualTools` provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection
  // of a script
  moveit_visual_tools::MoveItVisualTools visual_tools = moveit_visual_tools::MoveItVisualTools("panda_link0");
  // END_SUB_TUTORIAL

  // A few other pieces of data are stored for convenience.
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  geometry_msgs::Pose target_pose;
};

int main(int argc, char** argv)
{
  // ROS startup
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO_NAMED("tutorial", "Starting demo!");

  // Create and setup the MoveGroupInterfaceTutuorial instance
  MoveGroupInterfaceTutorial move_group_interface_tutorial;
  move_group_interface_tutorial.setup_visualization();
  move_group_interface_tutorial.print_basic_info();

  // Plan and move to Joint space goal
  move_group_interface_tutorial.plan_joint_space_goal();
  move_group_interface_tutorial.move();

  // Plan and move to Pose goal
  move_group_interface_tutorial.plan_pose_goal();
  move_group_interface_tutorial.move();

  // Plan to a point with a constrained and an unconstrained path
  move_group_interface_tutorial.plan_constrained_path();
  move_group_interface_tutorial.plan_unconstrained();

  // Plan and move along a cartesian series of paths
  move_group_interface_tutorial.plan_cartesian_path();
  move_group_interface_tutorial.move();

  // Create an obstacle and move around it
  move_group_interface_tutorial.create_obstacle();
  move_group_interface_tutorial.plan_obstacle();
  move_group_interface_tutorial.move();
  move_group_interface_tutorial.remove_obstacle();

  // Create an object to "pick up" and move
  move_group_interface_tutorial.create_parcel();
  move_group_interface_tutorial.attach_parcel();
  move_group_interface_tutorial.plan_parcel();
  move_group_interface_tutorial.move();
  move_group_interface_tutorial.detach_parcel();
  move_group_interface_tutorial.remove_parcel();

  ros::shutdown();
  return 0;
}

// BEGIN_TUTORIAL
//
// CALL_SUB_TUTORIAL includes
// CALL_SUB_TUTORIAL setup
// CALL_SUB_TUTORIAL visualization
// CALL_SUB_TUTORIAL basic_info
// CALL_SUB_TUTORIAL joint_space
// CALL_SUB_TUTORIAL movement
// CALL_SUB_TUTORIAL pose_goal
// CALL_SUB_TUTORIAL constrained_path
// CALL_SUB_TUTORIAL cartesian_path
// CALL_SUB_TUTORIAL add_obj
// CALL_SUB_TUTORIAL remove_obj
// CALL_SUB_TUTORIAL attach_obj
// CALL_SUB_TUTORIAL detach_obj
//
// END_TUTORIAL
