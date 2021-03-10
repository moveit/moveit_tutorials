/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Sachin Chitta */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  /* First put an object into the scene*/
  /* Advertise the collision object message publisher*/
  ros::Publisher collision_object_publisher =
      node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  while (collision_object_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }
  /* Define the object message */
  moveit_msgs::CollisionObject object;
  /* The header must contain a valid TF frame */
  object.header.frame_id = "r_wrist_roll_link";
  /* The id of the object */
  object.id = "box";

  /* A default pose */
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);

  /* An attach operation requires an ADD */
  object.operation = attached_object.object.ADD;

  /* Publish and sleep (to view the visualized results) */
  collision_object_publisher.publish(object);
  ros::WallDuration sleep_time(1.0);
  sleep_time.sleep();

  /* CHECK IF A STATE IS VALID */
  /* PUT THE OBJECT IN THE ENVIRONMENT */
  ROS_INFO("Putting the object back into the environment");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  sleep_time.sleep();

  /* Load the robot model */
  robot_model_loader::RDFLoader robot_model_loader("robot_description");
  /* Get a shared pointer to the model and construct a state */
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  robot_state::RobotState current_state(kinematic_model);
  current_state.getJointStateGroup("panda_arm")->setToRandomValues();

  /* Construct a robot state message */
  moveit_msgs::RobotState robot_state;
  robot_state::robotStateToRobotStateMsg(current_state, robot_state);

  /* Construct the service request */
  moveit_msgs::GetStateValidity::Request get_state_validity_request;
  moveit_msgs::GetStateValidity::Response get_state_validity_response;
  get_state_validity_request.robot_state = robot_state;
  get_state_validity_request.group_name = "panda_arm";

  /* Service client for checking state validity */
  ros::ServiceClient service_client = node_handle.serviceClient<moveit_msgs::GetStateValidity>("/check_state_validity");

  /* Publisher for display */
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>("/display_robot_state", 1);
  moveit_msgs::DisplayRobotState display_state;

  for (std::size_t i = 0; i < 20; ++i)
  {
    /* Make the service call */
    service_client.call(get_state_validity_request, get_state_validity_response);
    if (get_state_validity_response.valid)
      ROS_INFO("State %d was valid", (int)i);
    else
      ROS_ERROR("State %d was invalid", (int)i);

    /* Visualize the state */
    display_state.state = robot_state;
    display_publisher.publish(display_state);

    /* Generate a new state and put it into the request */
    current_state.getJointStateGroup("panda_arm")->setToRandomValues();
    robot_state::robotStateToRobotStateMsg(current_state, robot_state);
    get_state_validity_request.robot_state = robot_state;
    sleep_time.sleep();
  }
}
