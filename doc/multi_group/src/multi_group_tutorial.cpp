/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Utkarsh Rai
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
 *   * Neither the name of Utkarsh Rai nor the names of its
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

/* Author: Utkarsh Rai */

#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_group_tutorial");
  ros::NodeHandle nh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string movegroup_name, ee1_link, ee2_link;
  int counter = 1;
  // BEGIN_TUTORIAL
  // Setup
  // ^^^^^
  //
  // Setting up the dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  // The parameters ee1_link and ee2_link are used to specify the end effector for the two different arms.
  // The parameter movegroup_name specifies the planning group.
  // We have included both the arms under a combined planning group "both_bots" for moving the two arms in sync, as discussed above.
  nh.param<std::string>("move_group", movegroup_name, "both_bots");
  nh.param<std::string>("ee_link1", ee1_link, "panda_1_link8" );
  nh.param<std::string>("ee_link2", ee2_link, "panda_2_link8" );

  // We choose the rate at which this node should run. For our case it's 0.2 Hz(5 seconds)
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.2); // 0.2 Hz = 5 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

  ros::Duration(1.0).sleep(); // sleep 1 second

  // The :planning_interface:`MoveGroupInterface` class is setup for the
  // combined planning group of the arms.
  moveit::planning_interface::MoveGroupInterface move_group(movegroup_name);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Configure planner
  move_group.setPlanningTime(0.5);
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setEndEffectorLink(ee1_link);
  moveit::planning_interface::MoveItErrorCode success_plan = moveit_msgs::MoveItErrorCodes::FAILURE,
  motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  // Motion Planning
  // ^^^^^^^^^^^^^^^^^^
  // Send the arms to the home position, if they are not there yet.
  move_group.setStartStateToCurrentState();
  move_group.setNamedTarget("home_all");
  success_plan = move_group.plan(my_plan);

  if (success_plan == moveit_msgs::MoveItErrorCodes::SUCCESS) {
    motion_done = move_group.execute(my_plan);
  }
  else
  {
    ROS_WARN("Something went wrong moving the robots to home position.");
  }

  ros::Duration(2.0).sleep(); // 2 seconds

  // Specify the target pose and orientation of the end-effectors.

  geometry_msgs::PoseStamped target1, target2;

  target1.pose.position.x = 0.8;
  target1.pose.position.y = 0.3;
  target1.pose.position.z = 0.7;
  target1.pose.orientation.w = 1;
  target1.pose.orientation.x = 0;
  target1.pose.orientation.y = 1;
  target1.pose.orientation.z = 0;

  target2.pose.position.x = 0.6;
  target2.pose.position.y = -0.3;
  target2.pose.position.z = 0.7;
  target2.pose.orientation.w = 1;
  target2.pose.orientation.x = 0;
  target2.pose.orientation.y = 1;
  target2.pose.orientation.z = 0;

  // The  ``setPoseTarget(target1, ee1_link)`` sets the target pose for the corresponding end-effector (ee1_link in this case).
  // We alternate between the two target positions for the end-effectors in each iteration.

  while (ros::ok()) {

    if(counter == 0) {
      move_group.setPoseTarget(target1, ee1_link);
      move_group.setPoseTarget(target2, ee2_link);

    } else {
      move_group.setPoseTarget(target2, ee1_link);
      move_group.setPoseTarget(target1, ee2_link);
    }

    success_plan = move_group.plan(my_plan);

    if (success_plan == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        motion_done = move_group.execute(my_plan);
      }
      if (motion_done) {
        loop_rate_->sleep();
      }

    counter = (counter +1 ) % 2;
  }

  ros::shutdown();

  // END_TUTORIAL

  return 0;
}