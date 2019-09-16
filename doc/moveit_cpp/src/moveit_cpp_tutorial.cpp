#include <ros/ros.h>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
//
#include <geometry_msgs/PointStamped.h>
//
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_cpp_tutorial");
  ros::NodeHandle nh("/moveit_cpp_tutorial");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";
  static const std::string LOGNAME = "moveit_cpp_tutorial";

  ros::Duration(5.0).sleep();

  ROS_INFO_STREAM_NAMED(LOGNAME, "Starting MoveIt Tutorials...");

  auto moveit_cpp_ptr = std::make_shared<moveit::planning_interface::MoveitCpp>(nh);

  auto planning_components =
      std::make_shared<moveit::planning_interface::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto robot_start_state = planning_components->getStartState();
  auto jmg_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

  //  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_ptr);
  //  planning_scene->getCurrentStateNonConst().setToDefaultValues(jmg, "ready");

  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0", rvt::RVIZ_MARKER_TOPIC,
                                                      moveit_cpp_ptr->getPlanningSceneMonitor());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveItCpp Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  /*
   * Setting Start State #1
   */
  planning_components->setStartStateToCurrentState();

  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "panda_link0";
  target_pose.pose.orientation.w = 1.0;
  target_pose.pose.position.x = 0.28;
  target_pose.pose.position.y = -0.2;
  target_pose.pose.position.z = 0.5;
  planning_components->setGoal(target_pose, "panda_hand");

  if (planning_components->plan())
  {
    auto plan_soln = planning_components->getLastPlanSolution();
    visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform("panda_hand"), "start_pose");
    visual_tools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishAxisLabeled(target_pose.pose, "target_pose");
    visual_tools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan_soln->trajectory, jmg_ptr);
    visual_tools.trigger();
    auto state = moveit_cpp_ptr->getCurrentState();
    std::vector<double> joints;
    moveit_cpp_ptr->getCurrentState()->copyJointGroupPositions(PLANNING_GROUP, joints);
    for (auto j : joints)
      ROS_ERROR_STREAM("Joint: " << j << " ");
    // planning_components->execute();
    moveit_cpp_ptr->getCurrentState()->copyJointGroupPositions(PLANNING_GROUP, joints);
    for (auto j : joints)
      ROS_ERROR_STREAM("Joint: " << j << " ");
  }

  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  auto start_state = *(moveit_cpp_ptr->getCurrentState());
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.55;
  start_pose.position.y = 0.0;
  start_pose.position.z = 0.6;

  ROS_ERROR_STREAM("RES: " << start_state.setFromIK(jmg_ptr, start_pose, 5.0));

  planning_components->setStartState(start_state);
  // Same as the last goal
  if (planning_components->plan())
  {
    auto plan_soln = planning_components->getLastPlanSolution();
    robot_state::RobotState robot_state(robot_model_ptr);
    moveit::core::robotStateMsgToRobotState(plan_soln->start_state, robot_state);

    visual_tools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("panda_hand"), "start_pose");
    visual_tools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishAxisLabeled(target_pose.pose, "target_pose");
    visual_tools.publishTrajectoryLine(plan_soln->trajectory, jmg_ptr);
    visual_tools.trigger();
    // planning_components->execute();
  }

  //  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  //  auto target_state = *robot_state;
  //  geometry_msgs::Pose target_pose;
  //  target_pose.orientation.w = 1.0;
  //  target_pose.position.x = 0.55;
  //  target_pose.position.y = -0.05;
  //  target_pose.position.z = 0.8;

  //  target_state.setFromIK(jmg, target_pose);

  //  planning_components->setGoal(target_state);

  /*
   * Setting Goal #1
   */

  // planning_components->setGoal("ready");

  /*
   * Setting Goal #2
   */

  /*
   * Setting Goal #3
   */

  // planning_components->execute(true);
  // visual_tools.publishTrajectoryLine(planning_components->, jmg);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' to end the demo");

  // Shutdown
  ROS_INFO_STREAM_NAMED(LOGNAME, "Shutting down.");
  ros::waitForShutdown();
}
