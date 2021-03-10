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
   Desc: This is an example of how to load and use LERP motion planner.
*/

#include <ros/ros.h>

#include <pluginlib/class_loader.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
  const std::string NODE_NAME = "lerp_example";
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

  // Create a RobotState and to keep track of the current robot pose and planning group
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
  robot_state->setToDefaultValues();
  robot_state->update();

  // Create JointModelGroup
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  const std::vector<std::string>& link_model_names = joint_model_group->getLinkModelNames();
  ROS_INFO_NAMED(NODE_NAME, "end effector name %s\n", link_model_names.back().c_str());

  // Set the planner
  std::string planner_plugin_name = "lerp_interface/LERPPlanner";
  node_handle.setParam("planning_plugin", planner_plugin_name);

  // Create pipeline
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

  // ================================ Set the start and goal joint state
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name = PLANNING_GROUP;

  // Get the joint values of the start state and set them in request.start_state
  std::vector<double> start_joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, start_joint_values);
  req.start_state.joint_state.position = start_joint_values;

  // Goal constraint
  std::vector<double> goal_joint_values = { 0.8, 0.7, 1, 1.3, 1.9, 2.2, 3 };
  robot_state->setJointGroupPositions(joint_model_group, goal_joint_values);
  robot_state->update();
  moveit_msgs::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(*robot_state, joint_model_group);
  req.goal_constraints.clear();
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

  // ================================ Visualization tools
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, psm);
  visual_tools.loadRobotStatePub("/display_robot_state");
  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();  // clear all old markers
  visual_tools.trigger();
  visual_tools.loadRemoteControl();

  /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

  /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
  visual_tools.trigger();

  /* We can also use visual_tools to wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // ================================ planning context
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

  visual_tools.prompt("Press 'next' to visualzie the result");

  // ================================ Visualize the trajectory
  //  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  // visual_tools.trigger();

  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  moveit_msgs::RobotTrajectory solution_traj = response.trajectory;
  int number_of_steps = solution_traj.joint_trajectory.points.size();
  ROS_DEBUG_NAMED(NODE_NAME, "number of timesteps in the solution trajectory: %i", number_of_steps);

  for (int step_num = 0; step_num < number_of_steps; ++step_num)
  {
    std::vector<double> solution_positions;
    solution_positions = solution_traj.joint_trajectory.points[step_num].positions;
    std::stringstream sst;
    for (double solution_position : solution_positions)
    {
      sst << solution_position << " ";
    }
    ROS_INFO_STREAM_NAMED(NODE_NAME, sst.str());
  }

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);

  /* We can also use visual_tools to wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
}
