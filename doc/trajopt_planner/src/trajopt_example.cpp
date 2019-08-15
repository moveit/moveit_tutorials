#include <ros/ros.h>
#include <ros/console.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

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

/* Author: Omid Heidari
   Desc: This file is a test for using trajopt in MoveIt. The goal is to make different types of constraints in
   MotionPlanRequest and visualize the result calculated by using trajopt planner.
*/

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
  robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();

  // Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group
  robot_state::RobotStatePtr current_state(new robot_state::RobotState(robot_model));
  current_state->setToDefaultValues();

  const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);
  const std::vector<std::string>& joint_names = joint_model_group->getActiveJointModelNames();

  // With the planning scene we create a planing scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));
  psm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  psm->startStateMonitor();
  psm->startSceneMonitor();

  planning_scene::PlanningScenePtr planning_scene = psm->getPlanningScene();

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
  robot_state::RobotStatePtr start_state(new robot_state::RobotState(robot_model));
  start_state->setJointGroupPositions(joint_model_group, start_joint_values);
  start_state->update();

  req.start_state.joint_state.name = joint_names;
  req.start_state.joint_state.position = start_joint_values;
  req.goal_constraints.clear();
  req.group_name = PLANNING_GROUP;

  // Set the middle and goal state and joints tolerance
  // ========================================================================================
  robot_state::RobotStatePtr middle_state(new robot_state::RobotState(robot_model));
  std::vector<double> middle_joint_values = { 0.5, 0.4, 0.65, -0.75, 1.05, 1.25, -0.15 };
  middle_state->setJointGroupPositions(joint_model_group, middle_joint_values);
  middle_state->update();
  moveit_msgs::Constraints joint_middle =
      kinematic_constraints::constructGoalConstraints(*middle_state, joint_model_group);
  req.goal_constraints.push_back(joint_middle);
  req.goal_constraints[0].name = "middle_pos";
  // Set joint tolerance
  std::vector<moveit_msgs::JointConstraint> middle_joint_constraint = req.goal_constraints[0].joint_constraints;
  for (std::size_t x = 0; x < middle_joint_constraint.size(); ++x)
  {
    ROS_INFO_STREAM_NAMED(NODE_NAME, " ======================================= joint position at goal: " <<
                           middle_joint_constraint[x].position);
    req.goal_constraints[0].joint_constraints[x].tolerance_above = 0.001;
    req.goal_constraints[0].joint_constraints[x].tolerance_below = 0.001;
  }

  robot_state::RobotStatePtr goal_state(new robot_state::RobotState(robot_model));
  std::vector<double> goal_joint_values = { 0.8, 0.7, 1, -1.3, 1.9, 2.2, -0.1 };
  goal_state->setJointGroupPositions(joint_model_group, goal_joint_values);
  goal_state->update();
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(*goal_state, joint_model_group);
  req.goal_constraints.push_back(joint_goal);
  req.goal_constraints[1].name = "goal_pos";
  // Set joint tolerance
  std::vector<moveit_msgs::JointConstraint> goal_joint_constraint = req.goal_constraints[1].joint_constraints;
  for (std::size_t x = 0; x < goal_joint_constraint.size(); ++x)
  {
    ROS_INFO_STREAM_NAMED(NODE_NAME ," ======================================= joint position at goal: " << goal_joint_constraint[x].position);
    req.goal_constraints[1].joint_constraints[x].tolerance_above = 0.001;
    req.goal_constraints[1].joint_constraints[x].tolerance_below = 0.001;
  }

  // Load planner
  // ======================================================================================
  while (!psm->getStateMonitor()->haveCompleteState() && ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE_NAMED(1, NODE_NAME, "Waiting for complete state from topic ");
  }
  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;

  std::string planner_plugin_name = "trajopt_interface/TrajOptPlanner";
  node_handle.setParam("planning_plugin", planner_plugin_name);

  // Make sure the planner plugin is loaded
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM_NAMED(NODE_NAME, "Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
    ROS_INFO_STREAM_NAMED(NODE_NAME, "planner_plugin_name: " << planner_plugin_name);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM_NAMED(NODE_NAME, "Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM_NAMED(NODE_NAME, "Could not initialize planner instance");
    ROS_INFO_STREAM_NAMED(NODE_NAME ,"Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM_NAMED(NODE_NAME, "Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
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

  // Creat planning context
  // ========================================================================================
  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR_NAMED(NODE_NAME, "Could not compute plan successfully");
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

  for (int timestep_index = 0; timestep_index < response.trajectory.joint_trajectory.points.size(); ++timestep_index)
  {
    for (int joint_index = 0;
         joint_index < response.trajectory.joint_trajectory.points[timestep_index].positions.size(); ++joint_index)
    {
      std::cout << response.trajectory.joint_trajectory.points[timestep_index].positions[joint_index] << "  ";
    }
    std::cout << std::endl;
  }

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);

  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);

  const std::vector<std::string>& link_model_names = joint_model_group->getLinkModelNames();
  ROS_INFO_NAMED(NODE_NAME, "end effector name %s\n", link_model_names.back().c_str());

  const Eigen::Affine3d& end_effector_transform_current = current_state->getGlobalLinkTransform(link_model_names.back());
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = robot_model->getModelFrame();
  pose_msg.pose = tf2::toMsg(end_effector_transform_current);

  visual_tools.publishAxisLabeled(pose_msg.pose, "current");
  visual_tools.publishText(text_pose, "current pose", rvt::WHITE, rvt::XLARGE);

  const Eigen::Affine3d& end_effector_transform_start = start_state->getGlobalLinkTransform(link_model_names.back());
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = robot_model->getModelFrame();
  pose_msg.pose = tf2::toMsg(end_effector_transform_start);

  visual_tools.publishAxisLabeled(pose_msg.pose, "start");
  visual_tools.publishText(text_pose, "start pose", rvt::BLUE, rvt::XLARGE);

  const Eigen::Affine3d& end_effector_transform_middle = middle_state->getGlobalLinkTransform(link_model_names.back());
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = robot_model->getModelFrame();
  pose_msg.pose = tf2::toMsg(end_effector_transform_middle);

  visual_tools.publishAxisLabeled(pose_msg.pose, "middle");
  visual_tools.publishText(text_pose, "middle pose", rvt::RED, rvt::XLARGE);

  const Eigen::Affine3d& end_effector_transform_goal = goal_state->getGlobalLinkTransform(link_model_names.back());
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = robot_model->getModelFrame();
  pose_msg.pose = tf2::toMsg(end_effector_transform_goal);

  visual_tools.publishAxisLabeled(pose_msg.pose, "goal");
  visual_tools.publishText(text_pose, "goal pose", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' to finish demo \n");
}