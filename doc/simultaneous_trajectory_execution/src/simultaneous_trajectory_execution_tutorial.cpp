/* Author: Cristian C. Beltran-Hernandez */

#include <ros/ros.h>

#include <memory>

#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <stdlib.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simultaneous_trajectory_execution_move_group");

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  // Let's start by creating planning groups for each robot arm.
  // The panda dual arm environment has two planning groups defined as `panda_1` and `panda_2`
  moveit::planning_interface::MoveGroupInterface panda_1_group("panda_1");
  moveit::planning_interface::MoveGroupInterface panda_2_group("panda_2");

  // Now, let's define a target pose for `panda_1`
  geometry_msgs::PoseStamped panda_1_target_pose;
  panda_1_target_pose.header.frame_id = "base";
  panda_1_target_pose.pose.position.x = 0.450;
  panda_1_target_pose.pose.position.y = -0.50;
  panda_1_target_pose.pose.position.z = 1.600;
  panda_1_target_pose.pose.orientation.x = 0.993436;
  panda_1_target_pose.pose.orientation.y = 3.5161e-05;
  panda_1_target_pose.pose.orientation.z = 0.114386;
  panda_1_target_pose.pose.orientation.w = 2.77577e-05;

  // And one for `panda_2`
  geometry_msgs::PoseStamped panda_2_target_pose;
  panda_2_target_pose.header.frame_id = "base";
  panda_2_target_pose.pose.position.x = 0.450;
  panda_2_target_pose.pose.position.y = 0.40;
  panda_2_target_pose.pose.position.z = 1.600;
  panda_2_target_pose.pose.orientation.x = 0.993434;
  panda_2_target_pose.pose.orientation.y = -7.54803e-06;
  panda_2_target_pose.pose.orientation.z = 0.114403;
  panda_2_target_pose.pose.orientation.w = 3.67256e-05;

  // Planning
  // ^^^^^^^^
  // Let's plan a trajectory for `panda_1` using the previously defined target pose.
  panda_1_group.clearPoseTargets();
  panda_1_group.setStartStateToCurrentState();
  panda_1_group.setPoseTarget(panda_1_target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan panda_1_plan;
  bool success1 = (panda_1_group.plan(panda_1_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success1)
  {
    ROS_INFO("Plan with Panda 1 did not succeeded");
  }

  // Same for `panda_2`.
  panda_2_group.clearPoseTargets();
  panda_2_group.setStartStateToCurrentState();
  panda_2_group.setPoseTarget(panda_2_target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan panda_2_plan;
  bool success2 = (panda_2_group.plan(panda_2_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success2)
  {
    ROS_INFO("Plan with Panda 2 did not succeeded");
  }

  // Simultaneous Execution
  // ^^^^^^^^^^^^^^^^^^^^^^
  // Finally, let's execute both plans asynchronously to have them run simultaneously.
  panda_1_group.asyncExecute(panda_1_plan);
  panda_2_group.asyncExecute(panda_2_plan);
  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
