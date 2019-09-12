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
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <lerp_interface/lerp_planning_context.h>

int main(int argc, char** argv)
{
  const std::string NODE_NAME = "lerp_example";
  ros::init(argc, argv, NODE_NAME);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  const std::string PLANNING_GROUP = "panda_arm";
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader("robot_description"));
  robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();

  // Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  int dof = joint_names.size();

  // Create a planing scene monitor that
  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(planning_scene, robot_model_loader));
  psm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  psm->startStateMonitor();
  psm->startSceneMonitor();

  while (!psm->getStateMonitor()->haveCompleteState() && ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE_NAMED(1, NODE_NAME, "Waiting for complete state from topic ");
  }

  // Load a planner
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name = "lerp_interface/LERPPlanner";
  node_handle.setParam("planning_plugin", planner_plugin_name);

  // Make sure the planner is loaded
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
    ROS_INFO_STREAM_NAMED(NODE_NAME, " ===>>> planner_plugin_name:" << planner_plugin_name.c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
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

  // ================================ Set the start and goal joint state
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.trigger();
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name = PLANNING_GROUP;

  // Get the joint values of the start state and set them in request.start_state
  std::vector<double> start_joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, start_joint_values);
  req.start_state.joint_state.position = start_joint_values;

  // Goal constraint
  robot_state::RobotState goal_state(robot_model);
  std::vector<double> joint_values = { 0.8, 0.7, 1, 1.3, 1.9, 2.2, 3 };
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // ================================ planning context
  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);
  moveit_msgs::RobotTrajectory solution_traj = response.trajectory;

  int number_of_steps = solution_traj.joint_trajectory.points.size();
  printf("numner of timesteps in the solution trajectory: %i", number_of_steps);

  for (int w = 0; w < number_of_steps; ++w)
  {
    std::vector<double> vec;
    vec = solution_traj.joint_trajectory.points[w].positions;
    std::stringstream sst;
    for (int i = 0; i < vec.size(); ++i)
    {
      sst << vec[i] << " ";
    }
    ROS_INFO_STREAM_NAMED(NODE_NAME, sst.str());
  }

  // ================================ Visualize the trajectory
  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);

  /* We can also use visual_tools to wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
}
