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

/* Author: Acorn Pooley */

// This code goes with the interactivity tutorial

#ifndef MOVEIT_TUTORIALS_INTERACTIVITY_SRC_INTERACTIVE_ROBOT_H_
#define MOVEIT_TUTORIALS_INTERACTIVITY_SRC_INTERACTIVE_ROBOT_H_

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include "imarker.h"

/** Keeps track of the state of the robot and the world.
 * Updates the state when interactive markers are manipulated.
 * Publishes the state to rviz. */
class InteractiveRobot
{
public:
  InteractiveRobot(const std::string& robot_description = "robot_description",
                   const std::string& robot_topic = "interactive_robot_state",
                   const std::string& marker_topic = "interactive_robot_markers",
                   const std::string& imarker_topic = "interactive_robot_imarkers");
  ~InteractiveRobot();

  /** set which group to manipulate */
  void setGroup(const std::string& name);
  const std::string& getGroupName() const;

  /** Set the pose of the group we are manipulating */
  bool setGroupPose(const Eigen::Isometry3d& pose);

  /** set pose of the world object */
  void setWorldObjectPose(const Eigen::Isometry3d& pose);

  /** set a callback to call when updates occur */
  void setUserCallback(boost::function<void(InteractiveRobot& robot)> callback)
  {
    user_callback_ = callback;
  }

  /** access RobotModel */
  robot_model::RobotModelPtr& robotModel()
  {
    return robot_model_;
  }
  /** access RobotState */
  robot_state::RobotStatePtr& robotState()
  {
    return robot_state_;
  }

  /** return size and pose of world object cube */
  void getWorldGeometry(Eigen::Isometry3d& pose, double& size);

  /** exception thrown when a problem occurs */
  class RobotLoadException : std::exception
  {
  };

  /** hook for user data.  Unused by the InteractiveRobot class.
   * initialized to 0 */
  void* user_data_;

private:
  /* Indicate that the world or the robot has changed and
   * the new state needs to be updated and published to rviz */
  void scheduleUpdate();

  /* set the callback timer to fire if needed.
   * Return true if callback should happen immediately. */
  bool setCallbackTimer(bool new_update_request);

  /* update the world and robot state and publish to rviz */
  void updateCallback(const ros::TimerEvent& e);

  /* functions to calculate new state and publish to rviz */
  void updateAll();
  void publishRobotState();
  void publishWorldState();

  /* callback called when marker moves.  Moves right hand to new marker pose. */
  static void movedRobotMarkerCallback(InteractiveRobot* robot,
                                       const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /* callback called when marker moves.  Moves world object to new pose. */
  static void movedWorldMarkerCallback(InteractiveRobot* robot,
                                       const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /* marker publishers */
  ros::NodeHandle nh_;
  ros::Publisher robot_state_publisher_;
  ros::Publisher world_state_publisher_;
  interactive_markers::InteractiveMarkerServer interactive_marker_server_;
  IMarker* imarker_robot_;
  IMarker* imarker_world_;

  /* robot info */
  robot_model_loader::RobotModelLoader rm_loader_;
  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;

  /* info about joint group we are manipulating */
  const robot_model::JointModelGroup* group_;
  Eigen::Isometry3d desired_group_end_link_pose_;

  /* world info */
  Eigen::Isometry3d desired_world_object_pose_;
  static const double WORLD_BOX_SIZE_;
  static const Eigen::Isometry3d DEFAULT_WORLD_OBJECT_POSE_;

  /* user callback function */
  boost::function<void(InteractiveRobot& robot)> user_callback_;

  /* timer info for rate limiting */
  ros::Timer publish_timer_;
  ros::Time init_time_;
  ros::Time last_callback_time_;
  ros::Duration average_callback_duration_;
  static const ros::Duration min_delay_;
  int schedule_request_count_;
};

#endif  // MOVEIT_TUTORIALS_INTERACTIVITY_SRC_INTERACTIVE_ROBOT_H_
