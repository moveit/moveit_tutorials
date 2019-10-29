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

/* Author: Acorn Pooley, Michael Lautman */

// This code goes with the interactivity tutorial

#include <ros/ros.h>
#include "interactivity/interactive_robot.h"
#include "interactivity/pose_string.h"

void help()
{
  ROS_INFO("#####################################################");
  ROS_INFO("RVIZ SETUP");
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
  ROS_INFO("#####################################################");
}

void userCallback(InteractiveRobot& robot)
{
  ROS_INFO_STREAM("Robot position: " << PoseString(robot.robotState()->getGlobalLinkTransform("panda_link8")));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactivity_tutorial");
  ros::NodeHandle nh;

  InteractiveRobot robot;

  robot.setUserCallback(userCallback);

  help();

  ros::spin();

  ros::shutdown();
  return 0;
}
