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

#ifndef MOVEIT_TUTORIALS_INTERACTIVITY_SRC_IMARKER_H_
#define MOVEIT_TUTORIALS_INTERACTIVITY_SRC_IMARKER_H_

#include <interactive_markers/interactive_marker_server.h>
#include <Eigen/Geometry>

/* Interactive marker.
 *
 * Constructing creates an interactive marker with either 3 or 6 DOF controls.
 *
 * name - name of marker and text of marker description
 * pose - starting position of marker
 * callback - called when marker is moved
 * dof - POS = 3DOF positional, ORIENT = 3DOF orientation, BOTH = 6DOF
 */
class IMarker
{
public:
  /* degrees of freedom (3d position, 3d orientation, or both) */
  enum Dof
  {
    BOTH,
    POS,
    ORIENT
  };

  /** create an interactive marker at the origin */
  IMarker(interactive_markers::InteractiveMarkerServer& server, const std::string& name,
          const std::string& frame_id = "/panda_link0",
          boost::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> callback = printFeedback,
          Dof dof = BOTH)
    : imarker_()
  {
    initialize(server, name, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0), frame_id, callback, dof);
  }

  /** create an interactive marker with an initial pose */
  IMarker(interactive_markers::InteractiveMarkerServer& server, const std::string& name, const Eigen::Isometry3d& pose,
          const std::string& frame_id = "/panda_link0",
          boost::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> callback = printFeedback,
          Dof dof = BOTH)
    : imarker_()
  {
    Eigen::Quaterniond q(pose.linear());
    Eigen::Vector3d p = pose.translation();

    initialize(server, name, p, q, frame_id, callback, dof);
  }

  /** create an interactive marker with an initial pose */
  IMarker(interactive_markers::InteractiveMarkerServer& server, const std::string& name,
          const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
          const std::string& frame_id = "/panda_link0",
          boost::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> callback = printFeedback,
          Dof dof = BOTH)
    : imarker_()
  {
    initialize(server, name, position, orientation, frame_id, callback, dof);
  }

  /** create an interactive marker with an initial position */
  IMarker(interactive_markers::InteractiveMarkerServer& server, const std::string& name,
          const Eigen::Vector3d& position, const std::string& frame_id = "/panda_link0",
          boost::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> callback = printFeedback,
          Dof dof = BOTH)
    : imarker_()
  {
    initialize(server, name, position, Eigen::Quaterniond(1, 0, 0, 0), frame_id, callback, dof);
  }

  /** move marker to new pose */
  void move(const Eigen::Isometry3d& pose);

  /** default callback which just prints new position and orientation */
  static void printFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);

private:
  /* called by constructors */
  void initialize(interactive_markers::InteractiveMarkerServer& server, const std::string& name,
                  const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const std::string& frame_id,
                  boost::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> callback,
                  Dof dof);

  enum Axis
  {
    X,
    Y,
    Z
  };
  visualization_msgs::Marker makeAxisCyl(Axis axis);
  visualization_msgs::Marker makeBall();
  void makeBallControl();
  void makeAxisControl();

  visualization_msgs::InteractiveMarker imarker_;
  interactive_markers::InteractiveMarkerServer* server_;
};

#endif  // MOVEIT_TUTORIALS_INTERACTIVITY_SRC_IMARKER_H_
