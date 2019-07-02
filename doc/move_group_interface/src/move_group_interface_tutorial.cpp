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

// The rviz_vizual_tools namespace is shortened for code readability.
namespace rvt = rviz_visual_tools;

class MoveGroupInterfaceTutorial
{
public:

  /**
  * Basic constucter
  **/
  MoveGroupInterfaceTutorial()
  {
  }
  /**
  * This initializes visual_tools_ and sets the text_pose_ to above the robot
  */
  void setupVisualization()
  {
    // BEGIN_SUB_TUTORIAL viz_setup
    //
    // Visualization
    // ^^^^^^^^^^^^^
    //
    // Within MoveItVisualTools, remote control is an introspection tool that allows users to
    // step through a high level script via buttons and keyboard shortcuts in RViz.
    // A variety of commands can be used to display anything from text, points or even paths.
    //
    visual_tools_.loadRemoteControl();
    // END_SUB_TUTORIAL

    // Set the location where text is diplayed to above the robot
    text_pose_.translation().z() = 1.25;
  }
  /**
  * Publishes the given message above the robot
  **/
  void publishToRViz(std::string msg)
  {
    visual_tools_.deleteAllMarkers();
    // To publish a text message inside Rviz
    visual_tools_.publishText(text_pose_, msg, rvt::WHITE, rvt::XLARGE);
    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools_.trigger();
    // This prompts the user before continuing.
    visual_tools_.prompt("Press 'next' in the window to continue");
  }

  /**
  * Publishes the given message above the robot, and displays a path
  **/
  void publishToRViz(std::string msg, const moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    // Remove old paths and text
    visual_tools_.deleteAllMarkers();
    // To publish a text message inside Rviz
    visual_tools_.publishText(text_pose_, msg, rvt::WHITE, rvt::XLARGE);
    // To publish a planed path
    visual_tools_.publishTrajectoryLine(plan.trajectory_, joint_model_group_);
    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools_.trigger();
    // This prompts the user before continuing.
    visual_tools_.prompt("Press 'next' in the window to continue");
  }

  /**
  * Publishes the given message above the robot, displays a path, and points
  **/
  void publishToRViz(std::string msg, const moveit::planning_interface::MoveGroupInterface::Plan& plan, std::vector<geometry_msgs::Pose> points)
  {
    // BEGIN_SUB_TUTORIAL visualization

    // There are many different ways to publish information to RViz. These are some commonly used commands.
    //
    // Remove old paths and text.
    visual_tools_.deleteAllMarkers();
    // To publish a text message inside Rviz.
    visual_tools_.publishText(text_pose_, msg, rvt::WHITE, rvt::XLARGE);
    // To publish a planed path.
    visual_tools_.publishTrajectoryLine(plan.trajectory_, joint_model_group_);
    // To publish points.
    for( geometry_msgs::Pose point : points )
      visual_tools_.publishAxisLabeled(point, "pose");
    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations.
    // This publishes all the changes just added.
    visual_tools_.trigger();
    // This prompts the user before continuing.
    visual_tools_.prompt("Press 'next' in the window to continue");
    // END_SUB_TUTORIAL
  }

  /**
  * Publishes the given path without removing old content.
  */
  void publishPath(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    // To publish the path
    visual_tools_.publishTrajectoryLine(plan.trajectory_, joint_model_group_);
    visual_tools_.trigger();
    visual_tools_.prompt("Press 'next' in the window to continue");
  }

  /**
  * Prints off basic information about move_group_
  **/
  void printBasicInfo()
  {
    // BEGIN_SUB_TUTORIAL basic_info
    //
    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // move_group holds a lot of information about the robot.
    //
    // For example we can print the name of the reference frame for this robot.
    ROS_INFO_NAMED(name_, "Planning frame: %s", move_group_.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED(name_, "End effector link: %s", move_group_.getEndEffectorLink().c_str());

    // We even can get a list of all the groups in the robot.
    ROS_INFO_NAMED(name_, "Available Planning Groups:");
    std::copy(move_group_.getJointModelGroupNames().begin(), move_group_.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));
    // END_SUB_TUTORIAL
  }

  /**
  * Populates the plan with a path to the given joint space goal
  * Returns the success of planning
  **/
  bool planJointSpaceGoal( const std::vector<double> joint_group_positions, moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    // Under some circumstances the robot may be unable to move as the plan's start state
    // isn't where the actual robot is. These can be used to update this
    /* move_group_.setStartState(*move_group_.getCurrentState()); */
    /* start_state.setFromIK(joint_model_group_); */

    // BEGIN_SUB_TUTORIAL plan_joint_space
    // To plan to the goal we must set move_group's target to our goal
    move_group_.setJointValueTarget(joint_group_positions);
    // move_group can now plan to this goal
    bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // END_SUB_TUTORIAL'

    ROS_INFO_NAMED("tutorial", "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");
    return success;
  }

  /**
  * Populates the plan with a path to the pose goal
  * Returns the success of planning
  **/
  bool planPoseGoal(geometry_msgs::Pose pose, moveit::planning_interface::MoveGroupInterface::Plan& plan, bool resetStartState=false)
  {
    // Someitmes it can be useful to plan a path that doesn't start where the current robot is. If that is done it is important
    // to reset the planning start state back to where the robot is when done. This is also used when an object is attached.
    if (resetStartState)
      move_group_.setStartState(*move_group_.getCurrentState());

    // BEGIN_SUB_TUTORIAL plan_pose_goal
    // Setting the target for a pose goal is similar to a joint space goal
    move_group_.setPoseTarget(pose);
    // Planning to the goal, and then moving is identical to the joint space goal.
    //
    // END_SUB_TUTORIAL
    bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan (pose space goal) %s", success ? "" : "FAILED");
    return success;
  }

  /**
  * Adds the given planning constraints
  **/
  void addConstraint(moveit_msgs::Constraints test_constraints)
  {
    // BEGIN_SUB_TUTORIAL add_constraint
    // Add it to move_group
    move_group_.setPathConstraints(test_constraints);
    // Planning with constraints can be slow because every sample must call an inverse kinematics
    // solver. Lets increase the planning time from the default 5 seconds to be sure the planner
    // has enough time to succeed.
    move_group_.setPlanningTime(10.0);
    // END_SUB_TUTORIAL
  }

  /**
  * Removes planning constraints
  **/
  void removeConstraints()
  {
    // BEGIN_SUB_TUTORIAL remove_constraint
    // After moving it is often useful to revert back to the default settings.
    move_group_.clearPathConstraints();
    move_group_.setPlanningTime(5.0);
    // The demo should show two paths to compare constrained plan with an unconstrained plan.
    // END_SUB_TUTORIAL
  }

  /**
  * Populates the plan with a cartesian path following the given points in order
  **/
  double planCartesianPath(std::vector<geometry_msgs::Pose> target_poses, moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    // BEGIN_SUB_TUTORIAL cartesian_plan
    //
    // Cartesian motions are frequently needed to be slower for actions such as approach and
    // retreat grasp motions. Here we demonstrate how to reduce the speed of the robot arm via
    // a scaling factor of the maxiumum speed of each joint. Note this is not the speed of the
    // end effector point.
    move_group_.setMaxVelocityScalingFactor(0.1);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue.
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_.computeCartesianPath(target_poses, eef_step, jump_threshold, trajectory);

    // While we didn't use the planner to create this trajectory it still can be added to a plan
    plan.trajectory_ = trajectory;
    // Moving the robot with this plan is the same as before.
    //
    // When finished remember to reset the scalingfactor to return to normal speeds.
    move_group_.setMaxVelocityScalingFactor(1);

    ROS_INFO_NAMED("tutorial", "Visualizing plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    // END_SUB_TUTORIAL
  }

  /**
  * Adds the given object to the world
  * shape_msgs::SolidPrimitive primitive - The shape
  * geometry_msgs::Pose box_pose - The location
  * std::string name - The object ID
  **/
  void createObject(shape_msgs::SolidPrimitive primitive, geometry_msgs::Pose box_pose, std::string name)
  {
    // BEGIN_SUB_TUTORIAL add_obj
    // Create a collisionObject.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_.getPlanningFrame();
    collision_object.id = name;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add it to our storage.
    collision_objects_.push_back(collision_object);
    object_ids_.push_back(collision_object.id);

    // Now, let's add the collision object into the world.
    planning_scene_interface_.addCollisionObjects(collision_objects_);
    // Once added new plans will avoid the object.
    // END_SUB_TUTORIAL
  }

  /**
  * Removes all objects from the world
  **/
  void removeObjects()
  {
    // BEGIN_SUB_TUTORIAL remove_obj
    //
    // To remove the object from the planning scene.
    planning_scene_interface_.removeCollisionObjects(object_ids_);
    // To remove from our storage.
    object_ids_.clear();
    collision_objects_.clear();
    // END_SUB_TUTORIAL
  }

  /**
  * Attaches a object to the robot
  * std::string id - ID of the object to attach
  **/
  void attachObject(std::string id)
  {
    // BEGIN_SUB_TUTORIAL attach_obj
    // Attaching and detaching objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // move_group_ provides convenient interfaces for attaching and detaching collision objects.
    //
    // To attach an object.
    move_group_.attachObject(id);
    ros::spinOnce();
    ros::Duration(0.5).sleep(); // sleep for half a second
    ros::spinOnce();
    ros::Duration(0.5).sleep(); // sleep for half a second
    // Planning and moving with an attached object is the same as without.
    //
    // END_SUB_TUTORIAL
  }

  /**
  * detaches a given object from the robot
  * std::string ID of the object to detach
  **/
  void detachObject(std::string id)
  {
    // BEGIN_SUB_TUTORIAL detach_obj
    // To detach an object.
    move_group_.detachObject(id);
    ros::Duration(0.5).sleep(); // sleep for half a second
    // END_SUB_TUTORIAL

    // RViz
  }

  /**
  * Moves the robot according to the given plan
  **/
  void move(moveit::planning_interface::MoveGroupInterface::Plan plan)
  {
    // BEGIN_SUB_TUTORIAL movement
    // Moving the robot
    // ^^^^^^^^^^^^^^^^
    //
    // We can now use the plan we just created with movegroup to move the robot.
    move_group_.execute(plan);
    /* move_group_move(); // calls plan and execute at the same time */
    // END_SUB_TUTORIAL
  }

  /**
  * Gets the join states in vector form
  **/
  std::vector<double> getCurrentJointState()
  {
    // BEGIN_SUB_TUTORIAL get_joint_space
    //
    // Planning to a Joint-space Goal
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // One of the easiest ways to move the robot is by adjusting the joint angles. To start, we'll
    // create an pointer that references the current robot's state. RobotState is the object that
    // contains all the current position/velocity/acceleration data.
    std::vector<double> joint_group_positions;
    moveit::core::RobotStatePtr current_state = move_group_.getCurrentState();
    // Next get the current set of joint values for the group.
    current_state->copyJointGroupPositions(joint_model_group_, joint_group_positions);
    // END_SUB_TUTORIAL
    return joint_group_positions;
  }

private:
  // BEGIN_SUB_TUTORIAL setup
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an
  // object called the :joint_model_group:`JointModelGroup`. Throughout MoveIt the terms "planning group" and
  // "joint model group" are used interchangably.
  inline static const std::string planning_group_ = "panda_arm";
  // The :move_group_interface:`MoveGroupInterface` class can be easily setup
  // using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_=
      moveit::planning_interface::MoveGroupInterface(planning_group_);
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group_ =
      move_group_.getCurrentState()->getJointModelGroup(planning_group_);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  // These vectors hold additional information about the objects added to the "virtual world".
  std::vector<moveit_msgs::CollisionObject> collision_objects_;
  std::vector<std::string> object_ids_;

  // The package :moveit_visual_tools:`MoveItVisualTools` provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection
  // of a script.
  moveit_visual_tools::MoveItVisualTools visual_tools_ = moveit_visual_tools::MoveItVisualTools("panda_link0");
  // END_SUB_TUTORIAL

  // A few other pieces of data are stored in the class
  const std::string name_ = "tutorial";
  Eigen::Isometry3d text_pose_ = Eigen::Isometry3d::Identity();
};

int main(int argc, char** argv)
{
  bool success;
  // ROS startup
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Create and setup the MoveGroupInterfaceTutuorial instance
  MoveGroupInterfaceTutorial move_group_interface_tutorial;
  move_group_interface_tutorial.setupVisualization();
  move_group_interface_tutorial.printBasicInfo();

  // Create the plan we will be using
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  // This is a vector of locations for moveing to multiple targets.
  std::vector<geometry_msgs::Pose> target_poses;

  // Plan and move to Joint space goal
  move_group_interface_tutorial.publishToRViz("Joint Space Goal");

  std::vector<double> joint_group_positions = move_group_interface_tutorial.getCurrentJointState();
  // BEGIN_SUB_TUTORIAL modify_joint_space
  // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the
  joint_group_positions[0] = -1.0;  // radians
  // END_SUB_TUTORIAL

  // Plan
  success = move_group_interface_tutorial.planJointSpaceGoal(joint_group_positions, plan);
  move_group_interface_tutorial.publishToRViz("Joint Space Goal", plan);
  move_group_interface_tutorial.move(plan);

  move_group_interface_tutorial.publishToRViz("Pose Goal");
  // BEGIN_SUB_TUTORIAL pose_goal
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Instead of changing the joint angles a Pose goal chooses a final location and orientation
  // to end at. The pose contains a quaternion orientation and an xyz location.

  geometry_msgs::Pose pose;

  pose.orientation.w = 0.0;
  pose.orientation.x = 0.383;
  pose.orientation.y = -0.9240;
  pose.orientation.z = -0.0;
  pose.position.x = 0.28;
  pose.position.y = -0.2;
  pose.position.z = 0.5;

  // END_SUB_TUTORIAL
  target_poses.push_back( pose );

  success = move_group_interface_tutorial.planPoseGoal(target_poses.at(0), plan);
  move_group_interface_tutorial.publishToRViz("Pose Goal", plan, target_poses);
  move_group_interface_tutorial.move(plan);

  target_poses.pop_back();
  // Plan to a point with a constrained and an unconstrained path
  pose.position.x = -0.38;
  pose.position.y = -0.49;
  pose.position.z = 0.38;
  target_poses.push_back( pose );

  move_group_interface_tutorial.publishToRViz("Constrained Path");
  // BEGIN_SUB_TUTORIAL constrained_path
  //
  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints are created for a link on the robot.
  // Let's specify a path constraint for our group on the end effector.
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
  // END_SUB_TUTORIAL

  move_group_interface_tutorial.addConstraint(test_constraints);
  success = move_group_interface_tutorial.planPoseGoal(target_poses.at(0), plan);
  move_group_interface_tutorial.publishToRViz("Constrained Path", plan, target_poses);
  move_group_interface_tutorial.removeConstraints();
  success = move_group_interface_tutorial.planPoseGoal(target_poses.at(0), plan);
  move_group_interface_tutorial.publishPath(plan);
  target_poses.pop_back();
  move_group_interface_tutorial.publishToRViz("Cartesian Path");
  // BEGIN_SUB_TUTORIAL cartesian
  //
  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  //
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note The initial pose does not
  // need to be added to the waypoint list but adding it can help with visualizations
  //
  // Create current location
  pose.orientation.w = 0.0;
  pose.orientation.x = 0.383;
  pose.orientation.y = -0.9240;
  pose.orientation.z = -0.0;
  pose.position.x = 0.28;
  pose.position.y = -0.2;
  pose.position.z = 0.5;

  // This pose and subsequent changes are then added to a vector of poses.
  target_poses.push_back(pose);

  pose.position.z -= 0.2;
  target_poses.push_back(pose);  // down

  pose.position.y -= 0.2;
  target_poses.push_back(pose);  // right

  pose.position.z += 0.2;
  pose.position.y += 0.2;
  pose.position.x -= 0.2;
  target_poses.push_back(pose);  // up, left, and back
  // END_SUB_TUTORIAL

  double percent = move_group_interface_tutorial.planCartesianPath(target_poses, plan);
  move_group_interface_tutorial.publishToRViz("Cartesian Path", plan, target_poses);
  move_group_interface_tutorial.move(plan);
  target_poses.clear();

  move_group_interface_tutorial.publishToRViz("Path with Obstacle");
  // BEGIN_SUB_TUTORIAL create_obj
  //
  // Adding or Removing Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
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
  // END_SUB_TUTORIAL

  move_group_interface_tutorial.createObject(primitive, box_pose, "obstacle");

  pose.orientation.w = 0.707;
  pose.orientation.x = 0.000;
  pose.orientation.y = 0.000;
  pose.orientation.z = 0.707;
  pose.position.x = 0.5;
  pose.position.y = -0.2;
  pose.position.z = 0.6;
  target_poses.push_back(pose);

  success = move_group_interface_tutorial.planPoseGoal(target_poses.at(0), plan);
  move_group_interface_tutorial.publishToRViz("Path with Obstacle", plan, target_poses);
  move_group_interface_tutorial.move(plan);
  move_group_interface_tutorial.removeObjects();

  move_group_interface_tutorial.publishToRViz("Path with Payload");

  // Create payload
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5;
  box_pose.position.y = -0.2;
  box_pose.position.z = 0.7;

  std::string payload_name = "payload";
  move_group_interface_tutorial.createObject(primitive, box_pose, payload_name);
  move_group_interface_tutorial.publishToRViz("Path with Payload");
  // Attach payload
  move_group_interface_tutorial.attachObject(payload_name);
  move_group_interface_tutorial.publishToRViz("Path with Payload");

  // Plan movement
  target_poses.at(0).position.y += 0.2;
  target_poses.at(0).position.z += 0.2;
  success = move_group_interface_tutorial.planPoseGoal(target_poses.at(0), plan, true);
  move_group_interface_tutorial.publishToRViz("Path with Payload", plan, target_poses);

  // Move
  move_group_interface_tutorial.move(plan);
  // Detach and remove payload
  move_group_interface_tutorial.publishToRViz("Path with Payload", plan, target_poses);
  move_group_interface_tutorial.detachObject(payload_name);
  move_group_interface_tutorial.removeObjects();

  ros::shutdown();
  return 0;
}

// BEGIN_TUTORIAL
//
// CALL_SUB_TUTORIAL setup
// CALL_SUB_TUTORIAL viz_setup
// CALL_SUB_TUTORIAL visualization
// CALL_SUB_TUTORIAL basic_info
// CALL_SUB_TUTORIAL get_joint_space
// CALL_SUB_TUTORIAL modify_joint_space
// CALL_SUB_TUTORIAL plan_joint_space
// CALL_SUB_TUTORIAL movement
// CALL_SUB_TUTORIAL pose_goal
// CALL_SUB_TUTORIAL plan_pose_goal
// CALL_SUB_TUTORIAL constrained_path
// CALL_SUB_TUTORIAL add_constraint
// CALL_SUB_TUTORIAL remove_constraint
// CALL_SUB_TUTORIAL cartesian
// CALL_SUB_TUTORIAL cartesian_plan
// CALL_SUB_TUTORIAL create_obj
// CALL_SUB_TUTORIAL add_obj
// CALL_SUB_TUTORIAL remove_obj
// CALL_SUB_TUTORIAL attach_obj
// CALL_SUB_TUTORIAL detach_obj
//
// END_TUTORIAL
