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

/* Author: Acorn Pooley */

#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

// PI
#include <boost/math/constants/constants.hpp>

// This code is described in the RobotStateDisplay tutorial here:

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_display_tutorial");

  /* Needed for ROS_INFO commands to work */
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* Load the robot model */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  /* Get a shared pointer to the model */
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  /* Get the configuration for the joints in the right arm of the Panda*/
  const robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

  /* PUBLISH RANDOM ARM POSITIONS */
  ros::NodeHandle nh;
  ros::Publisher robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>("tutorial_robot_state", 1);

  /* loop at 1 Hz */
  ros::Rate loop_rate(1);

  for (int cnt = 0; cnt < 5 && ros::ok(); cnt++)
  {
    kinematic_state->setToRandomPositions(joint_model_group);

    /* get a robot state message describing the pose in kinematic_state */
    moveit_msgs::DisplayRobotState msg;
    robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);

    /* send the message to the RobotState display */
    robot_state_publisher.publish(msg);

    /* let ROS send the message, then wait a while */
    ros::spinOnce();
    loop_rate.sleep();
  }

  /* POSITION END EFFECTOR AT SPECIFIC LOCATIONS */

  /* Find the default pose for the end effector */
  kinematic_state->setToDefaultValues();

  const Eigen::Isometry3d end_effector_default_pose = kinematic_state->getGlobalLinkTransform("r_wrist_roll_link");

  const double PI = boost::math::constants::pi<double>();
  const double RADIUS = 0.1;

  for (double angle = 0; angle <= 2 * PI && ros::ok(); angle += 2 * PI / 20)
  {
    /* calculate a position for the end effector */
    Eigen::Isometry3d end_effector_pose =
        Eigen::Translation3d(RADIUS * cos(angle), RADIUS * sin(angle), 0.0) * end_effector_default_pose;

    ROS_INFO_STREAM("End effector position:\n" << end_effector_pose.translation());

    /* use IK to get joint angles satisfyuing the calculated position */
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_pose, 10, 0.1);
    if (!found_ik)
    {
      ROS_INFO_STREAM("Could not solve IK for pose\n" << end_effector_pose.translation());
      continue;
    }

    /* get a robot state message describing the pose in kinematic_state */
    moveit_msgs::DisplayRobotState msg;
    robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);

    /* send the message to the RobotState display */
    robot_state_publisher.publish(msg);

    /* let ROS send the message, then wait a while */
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();
  return 0;
}
