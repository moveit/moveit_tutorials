/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik, LLC.
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
 *   * Neither the name of PickNik nor the names of its
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

/* Author: Omid Heidari
   Desc: This file is a test for using trajopt in MoveIt. The goal is to make different types of constraints in
   MotionPlanRequest and visualize the result calculated using the trajopt planner.
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "moveit/planning_interface/planning_request.h"
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv)
{
  const std::string NODE_NAME = "trajopt_example";
  ros::init(argc, argv, NODE_NAME);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  const std::string PLANNING_GROUP = "panda_arm";
  const std::string ROBOT_DESCRIPTION = "robot_description";
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

  // Create a planing scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();

  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  // Create a RobotState to keep track of the current robot pose and planning group
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
  robot_state->setToDefaultValues();
  robot_state->update();

  // Create JointModelGroup
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  const std::vector<std::string>& joint_names = joint_model_group->getActiveJointModelNames();
  const std::vector<std::string>& link_model_names = joint_model_group->getLinkModelNames();
  ROS_INFO_NAMED(NODE_NAME, "end effector name %s\n", link_model_names.back().c_str());

  // Set the planner
  std::string planner_plugin_name = "trajopt_interface/TrajOptPlanner";
  node_handle.setParam("planning_plugin", planner_plugin_name);

  // Create pipeline
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

  // Current state
  std::vector<double> current_joint_values = { 0, 0, 0, -1.5, 0, 0.6, 0.9 };
  robot_state->setJointGroupPositions(joint_model_group, current_joint_values);

  geometry_msgs::Pose pose_msg_current;
  const Eigen::Isometry3d& end_effector_transform_current =
      robot_state->getGlobalLinkTransform(link_model_names.back());
  pose_msg_current = tf2::toMsg(end_effector_transform_current);

  // Create response and request
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  // Set start state
  // ======================================================================================
  // panda_arm joint limits:
  //   -2.8973  2.8973
  //   -1.7628  1.7628
  //   -2.8973  2.8973
  //   -3.0718 -0.0698
  //   -2.8973  2.8973
  //   -0.0175  3.7525
  //   -2.8973  2.8973

  std::vector<double> start_joint_values = { 0.4, 0.3, 0.5, -0.55, 0.88, 1.0, -0.075 };
  robot_state->setJointGroupPositions(joint_model_group, start_joint_values);
  robot_state->update();

  req.start_state.joint_state.name = joint_names;
  req.start_state.joint_state.position = start_joint_values;
  req.goal_constraints.clear();
  req.group_name = PLANNING_GROUP;

  geometry_msgs::Pose pose_msg_start;
  const Eigen::Isometry3d& end_effector_transform_start = robot_state->getGlobalLinkTransform(link_model_names.back());
  pose_msg_start = tf2::toMsg(end_effector_transform_start);

  // Set the goal state
  // ========================================================================================
  std::vector<double> goal_joint_values = { 0.8, 0.7, 1, -1.3, 1.9, 2.2, -0.1 };
  robot_state->setJointGroupPositions(joint_model_group, goal_joint_values);
  robot_state->update();
  moveit_msgs::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(*robot_state, joint_model_group);
  req.goal_constraints.push_back(joint_goal);
  req.goal_constraints[0].name = "goal_pos";
  // Set joint tolerance
  std::vector<moveit_msgs::JointConstraint> goal_joint_constraint = req.goal_constraints[0].joint_constraints;
  for (std::size_t x = 0; x < goal_joint_constraint.size(); ++x)
  {
    ROS_INFO_STREAM_NAMED(NODE_NAME, " ======================================= joint position at goal: "
                                         << goal_joint_constraint[x].position);
    req.goal_constraints[0].joint_constraints[x].tolerance_above = 0.001;
    req.goal_constraints[0].joint_constraints[x].tolerance_below = 0.001;
  }

  geometry_msgs::Pose pose_msg_goal;
  const Eigen::Isometry3d& end_effector_transform_goal = robot_state->getGlobalLinkTransform(link_model_names.back());
  pose_msg_goal = tf2::toMsg(end_effector_transform_goal);

  // Reference Trajectory. The type should be defined in the yaml file.
  // ========================================================================================
  // type: STATIONARY
  // No need to pass any trajectory. The current joint values will be replicated for all timesteps

  // type: JOINT_INTERPOLATED
  // The joint values at a specified state. Could be the goal state or one of the goals when having multiple goal states
  // The first index (points[0]) is used to set the values of the joints at the specified state as follows:
  // req.reference_trajectories.resize(1);
  // req.reference_trajectories[0].joint_trajectory.resize(1);
  // req.reference_trajectories[0].joint_trajectory[0].joint_names = joint_names;
  // req.reference_trajectories[0].joint_trajectory[0].points.resize(1);
  // req.reference_trajectories[0].joint_trajectory[0].points[0].positions = goal_joint_values;

  // type: GIVEN_TRAJ
  // For this example, we give an interpolated trajectory
  int const N_STEPS = 20;  // number of steps
  int const N_DOF = 7;     // number of degrees of freedom

  // We need one reference trajectory with one joint_trajectory
  req.reference_trajectories.resize(1);
  req.reference_trajectories[0].joint_trajectory.resize(1);
  // trajectory includes both the start and end points (N_STEPS + 1)
  req.reference_trajectories[0].joint_trajectory[0].points.resize(N_STEPS + 1);
  req.reference_trajectories[0].joint_trajectory[0].joint_names = joint_names;
  req.reference_trajectories[0].joint_trajectory[0].points[0].positions = current_joint_values;

  std::vector<double> joint_values = current_joint_values;
  // Increment joint values at each step
  for (std::size_t step_index = 1; step_index <= N_STEPS; ++step_index)
  {
    double increment;
    step_index <= 10 ? (increment = 0.05) : (increment = 0.044);
    for (int dof_index = 0; dof_index < N_DOF; ++dof_index)
    {
      joint_values[dof_index] = joint_values[dof_index] + increment;
    }
    req.reference_trajectories[0].joint_trajectory[0].joint_names = joint_names;
    req.reference_trajectories[0].joint_trajectory[0].points[step_index].positions = joint_values;
  }

  // Visualization
  // ========================================================================================
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, psm);
  visual_tools.loadRobotStatePub("/display_robot_state");
  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();  // clear all old markers
  visual_tools.trigger();

  /* Remote control is an introspection tool that allows users to step through a high level script
     via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

  /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
  visual_tools.trigger();

  /* We can also use visual_tools to wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Solve the problem
  // ========================================================================================
  // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
  // representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    planning_pipeline->generatePlan(lscene, req, res);
  }
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR_STREAM_NAMED(NODE_NAME, "Could not compute plan successfully");
    return 0;
  }

  visual_tools.prompt("Press 'next' to visualize the result");

  // Visualize the result
  // ========================================================================================
  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  for (size_t timestep_index = 0; timestep_index < response.trajectory.joint_trajectory.points.size(); ++timestep_index)
  {
    std::stringstream joint_values_stream;
    for (double position : response.trajectory.joint_trajectory.points[timestep_index].positions)
    {
      joint_values_stream << position << " ";
    }
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "step: " << timestep_index << " joints positions: " << joint_values_stream.str());
  }

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);

  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);

  visual_tools.publishAxisLabeled(pose_msg_current, "current");
  visual_tools.publishText(text_pose, "current pose", rvt::WHITE, rvt::XLARGE);

  visual_tools.publishAxisLabeled(pose_msg_start, "start");
  visual_tools.publishText(text_pose, "start pose", rvt::BLUE, rvt::XLARGE);

  visual_tools.publishAxisLabeled(pose_msg_goal, "goal");
  visual_tools.publishText(text_pose, "goal pose", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' to finish demo \n");
}
