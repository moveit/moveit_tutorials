/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Acorn Pooley, Michael Lautman, Jens Petit */

#include <ros/ros.h>
#include "interactivity/interactive_robot.h"
#include "interactivity/pose_string.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/utils/robot_model_test_utils.h>

auto g_planning_scene = std::unique_ptr<planning_scene::PlanningScene>();
auto g_marker_array_publisher = std::unique_ptr<ros::Publisher>();
shapes::ShapePtr g_world_cube_shape;
visualization_msgs::MarkerArray g_collision_points;

const double BOX_SIZE = 0.1;

/** \brief Prints information to the console. */
void printHelp()
{
  ROS_INFO("#####################################################");
  ROS_INFO("RViz setup for interactive robot");
  ROS_INFO("----------");
  ROS_INFO("  Global options:");
  ROS_INFO("    FixedFrame = /panda_link0");
  ROS_INFO("  Add a RobotState display:");
  ROS_INFO("    RobotDescription = robot_description");
  ROS_INFO("    RobotStateTopic  = interactive_robot_state");
  ROS_INFO("  Add a Marker display:");
  ROS_INFO("    MarkerTopic = interactive_robot_markers");
  ROS_INFO("  Add an InteractiveMarker display:");
  ROS_INFO("    UpdateTopic = interactive_robot_imarkers/update");
  ROS_INFO("  Add a MarkerArray display:");
  ROS_INFO("    MarkerTopic = interactive_robot_marray");
  ROS_INFO("#####################################################");
}

void publishMarkers(visualization_msgs::MarkerArray& markers)
{
  // delete old markers
  if (!g_collision_points.markers.empty())
  {
    for (auto& marker : g_collision_points.markers)
      marker.action = visualization_msgs::Marker::DELETE;

    g_marker_array_publisher->publish(g_collision_points);
  }

  // move new markers into g_collision_points
  std::swap(g_collision_points.markers, markers.markers);

  // draw new markers (if there are any)
  if (!g_collision_points.markers.empty())
    g_marker_array_publisher->publish(g_collision_points);
}

void computeCollisionContactPoints(InteractiveRobot& robot)
{
  // move the world geometry in the collision world
  Eigen::Isometry3d world_cube_pose;
  double world_cube_size;
  robot.getWorldGeometry(world_cube_pose, world_cube_size);
  g_planning_scene->getWorldNonConst()->moveShapeInObject("world_cube", g_world_cube_shape, world_cube_pose);

  collision_detection::CollisionRequest c_req;
  collision_detection::CollisionResult c_res;
  c_req.group_name = robot.getGroupName();
  c_req.contacts = true;
  c_req.max_contacts = 100;
  c_req.max_contacts_per_pair = 5;
  c_req.verbose = false;

  g_planning_scene->checkCollision(c_req, c_res, *robot.robotState());

  if (c_res.collision)
  {
    ROS_INFO("COLLIDING contact_point_count=%d", (int)c_res.contact_count);
    if (c_res.contact_count > 0)
    {
      std_msgs::ColorRGBA color;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 1.0;
      color.a = 0.5;
      visualization_msgs::MarkerArray markers;

      /* Get the contact ponts and display them as markers */
      collision_detection::getCollisionMarkersFromContacts(markers, "panda_link0", c_res.contacts, color,
                                                           ros::Duration(),  // remain until deleted
                                                           0.01);            // radius
      publishMarkers(markers);
    }
  }
  else
  {
    ROS_INFO("Not colliding");

    // delete the old collision point markers
    visualization_msgs::MarkerArray empty_marker_array;
    publishMarkers(empty_marker_array);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bullet_collision_tutorial");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  ros::Publisher robot_state_publisher_2(node_handle.advertise<moveit_msgs::DisplayRobotState>("robot_state_before", 1));
  ros::Publisher marker_publisher(node_handle.advertise<visualization_msgs::Marker>("interactive_robot_markers", 1));
  visual_tools.setRobotStateTopic("interactive_robot_state");

  {
    // BEGIN_TUTORIAL
    // The code starts with creating an interactive robot and a new planning scene.
    InteractiveRobot interactive_robot("robot_description", "bullet_collision_tutorial/interactive_robot_state");
    g_planning_scene = std::make_unique<planning_scene::PlanningScene>(interactive_robot.robotModel());

    // Changing the collision detector to Bullet
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // The active collision detector is set from the planning scene using the specific collision detector allocator for
    // Bullet. The second argument indicates that Bullet will be the exclusive collision detection algorithm; the
    // default FCL will not be available anymore. Having one exclusive collision detection algorithm helps performance
    // a bit and is much more common.
    g_planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(),
                                                 true /* exclusive */);
    // For understanding the interactive interactive_robot, please refer to the Visualizing Collisions tutorial.
    // CALL_SUB_TUTORIAL CCD
    // CALL_SUB_TUTORIAL CCD_2
    // END_TUTORIAL

    Eigen::Isometry3d world_cube_pose;
    double world_cube_size;
    interactive_robot.getWorldGeometry(world_cube_pose, world_cube_size);
    g_world_cube_shape = std::make_shared<shapes::Box>(world_cube_size, world_cube_size, world_cube_size);
    g_planning_scene->getWorldNonConst()->addToObject("world_cube", g_world_cube_shape, world_cube_pose);

    // Create a marker array publisher for publishing contact points
    g_marker_array_publisher = std::make_unique<ros::Publisher>(
        node_handle.advertise<visualization_msgs::MarkerArray>("interactive_robot_marray", 100));

    interactive_robot.setUserCallback(computeCollisionContactPoints);

    printHelp();

    visual_tools.loadRemoteControl();
    visual_tools.prompt(
        "Press 'next' in the RvizVisualToolsGui window to start the continuous collision detection demo.");
    ROS_INFO("Shutting down the interactive interactive_robot...");

    // remove all collision markers
    if (!g_collision_points.markers.empty())
    {
      for (auto& marker : g_collision_points.markers)
        marker.action = visualization_msgs::Marker::DELETE;

      g_marker_array_publisher->publish(g_collision_points);
    }

    visual_tools.deleteAllMarkers();
  }

  // BEGIN_SUB_TUTORIAL CCD
  //
  // Continuous Collision Detection
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // For the CCD demonstration, the Panda robot is loaded again and with it a new planning scene created. Bullet is
  // again set as the active collision detector.
  robot_model::RobotModelPtr robot_model = moveit::core::loadTestingRobotModel("panda");
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(),
                                             /* exclusive = */ true);

  // The box is added and the robot brought into its position.
  Eigen::Isometry3d box_pose{ Eigen::Isometry3d::Identity() };
  box_pose.translation().x() = 0.43;
  box_pose.translation().y() = 0;
  box_pose.translation().z() = 0.55;

  auto box = std::make_shared<shapes::Box>(BOX_SIZE, BOX_SIZE, BOX_SIZE);
  planning_scene->getWorldNonConst()->addToObject("box", box, box_pose);

  robot_state::RobotState& state = planning_scene->getCurrentStateNonConst();
  state.setToDefaultValues();

  double joint2 = -0.785;
  double joint4 = -2.356;
  double joint6 = 1.571;
  double joint7 = 0.785;
  state.setJointPositions("panda_joint2", &joint2);
  state.setJointPositions("panda_joint4", &joint4);
  state.setJointPositions("panda_joint6", &joint6);
  state.setJointPositions("panda_joint7", &joint7);
  state.update();

  robot_state::RobotState state_before(state);

  // Finally, a collision check is performed and the result printed to the terminal.
  collision_detection::CollisionResult res;
  collision_detection::CollisionRequest req;
  req.contacts = true;
  planning_scene->checkCollision(req, res);
  ROS_INFO_STREAM_NAMED("bullet_tutorial", (res.collision ? "In collision." : "Not in collision."));
  // This code is repeated for the second robot configuration.
  // END_SUB_TUTORIAL

  visualization_msgs::Marker marker_delete;
  marker_delete.header.stamp = ros::Time::now();
  marker_delete.ns = "world_box";
  marker_delete.id = 0;
  marker_delete.action = visualization_msgs::Marker::DELETE;
  marker_publisher.publish(marker_delete);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "/panda_link0";
  marker.header.stamp = ros::Time::now();
  marker.ns = "world_cube";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = BOX_SIZE;
  marker.scale.y = BOX_SIZE;
  marker.scale.z = BOX_SIZE;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.4f;
  marker.lifetime = ros::Duration();
  marker.pose = tf2::toMsg(box_pose);
  marker_publisher.publish(marker);

  moveit_msgs::DisplayRobotState msg;
  robot_state::robotStateToRobotStateMsg(state, msg.state);
  visual_tools.publishRobotState(state);

  visual_tools.prompt("Press 'next' for second robot state.");

  double joint_2{ 0.05 };
  double joint_4{ -1.6 };
  state.setJointPositions("panda_joint2", &joint_2);
  state.setJointPositions("panda_joint4", &joint_4);
  state.update();
  visual_tools.publishRobotState(state);

  res.clear();
  planning_scene->checkCollision(req, res);
  ROS_INFO_STREAM_NAMED("bullet_tutorial", (res.collision ? "In collision." : "Not in collision."));

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to perform a CCD check.");

  // BEGIN_SUB_TUTORIAL CCD_2
  // For the CCD check, we display both robot states at the same time.
  moveit_msgs::DisplayRobotState msg_state_before;
  robot_state::robotStateToRobotStateMsg(state_before, msg_state_before.state);
  robot_state_publisher_2.publish(msg_state_before);
  // Now a continuous collision check using the two different robot states can be performed. As the planning scene does
  // not yet contain any direct functions to do CCD, we have to access the collision environment and perform the check.
  res.clear();
  planning_scene->getCollisionEnv()->checkRobotCollision(req, res, state, state_before);
  ROS_INFO_STREAM_NAMED("bullet_tutorial", (res.collision ? "In collision." : "Not in collision."));
  // Note that the terminal output displays "In collision.".
  // END_SUB_TUTORIAL
  visualization_msgs::MarkerArray markers;
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 0.0;
  color.b = 1.0;
  color.a = 0.5;
  /* Get the contact ponts and display them as markers */
  collision_detection::getCollisionMarkersFromContacts(markers, "panda_link0", res.contacts, color, ros::Duration(),
                                                       0.01);
  publishMarkers(markers);

  visual_tools.prompt("Press 'next' to end tutorial.");
  ros::shutdown();
  return 0;
}
