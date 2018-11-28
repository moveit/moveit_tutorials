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

#include "interactivity/pose_string.h"

// tf2
#include <tf2_eigen/tf2_eigen.h>

#include <iostream>
#include <iomanip>

/** return a string describing a pose (position and quaternion) */
std::string PoseString(const geometry_msgs::Pose& pose)
{
  std::stringstream ss;
  ss << "p(";
  ss << std::setprecision(3);
  ss << std::setiosflags(std::ios::fixed);
  ss << std::setw(7) << pose.position.x << ", ";
  ss << std::setw(7) << pose.position.y << ", ";
  ss << std::setw(7) << pose.position.z << ") q(";
  ss << std::setw(7) << pose.orientation.x << ", ";
  ss << std::setw(7) << pose.orientation.y << ", ";
  ss << std::setw(7) << pose.orientation.z << ", ";
  ss << std::setw(7) << pose.orientation.w << ")";

  return std::string(ss.str());
}

/** return a string describing a pose
 * Assumes the pose is a rotation and a translation
 */
std::string PoseString(const Eigen::Isometry3d& pose)
{
  return PoseString(tf2::toMsg(pose));
}
