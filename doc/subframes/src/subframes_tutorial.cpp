/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, who?
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

/* Author: Felix von Drigalski*/

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

bool moveToCartPose(geometry_msgs::PoseStamped pose, moveit::planning_interface::MoveGroupInterface& group,
                    std::string end_effector_link)
{
  group.clearPoseTargets();
  group.setEndEffectorLink(end_effector_link);
  group.setStartStateToCurrentState();
  group.setPoseTarget(pose);

  ROS_INFO_STREAM("Planning motion to pose:");
  ROS_INFO_STREAM(pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  if (group.plan(myplan) && group.execute(myplan))
    return true;

  ROS_WARN("Failed to perform motion.");
  return false;
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // Create two objects
  double z_offset_box = .25;
  moveit_msgs::CollisionObject box;
  box.header.frame_id = "panda_hand";
  box.id = "box";
  box.primitives.resize(1);
  box.primitive_poses.resize(1);
  box.primitives[0].type = box.primitives[0].BOX;
  box.primitives[0].dimensions.resize(3);
  box.primitives[0].dimensions[0] = 0.05;
  box.primitives[0].dimensions[1] = 0.1;
  box.primitives[0].dimensions[2] = 0.02;
  box.primitive_poses[0].position.z = 0.0 + z_offset_box;
  box.subframe_poses.resize(5);
  box.subframe_names.resize(5);

  box.subframe_names[0] = "bottom";
  box.subframe_poses[0].position.y = -.05;
  box.subframe_poses[0].position.z = 0.0 + z_offset_box;
  tf2::Quaternion orientation;
  orientation.setRPY((90.0 / 180.0 * M_PI), 0, 0);
  box.subframe_poses[0].orientation = tf2::toMsg(orientation);

  box.subframe_names[1] = "top";
  box.subframe_poses[1].position.y = .05;
  box.subframe_poses[1].position.z = 0.0 + z_offset_box;
  orientation.setRPY(-(90.0 / 180.0 * M_PI), 0, 0);
  box.subframe_poses[1].orientation = tf2::toMsg(orientation);

  box.subframe_names[2] = "corner_1";
  box.subframe_poses[2].position.x = -.025;
  box.subframe_poses[2].position.y = -.05;
  box.subframe_poses[2].position.z = -.01 + z_offset_box;
  orientation.setRPY((90.0 / 180.0 * M_PI), 0, 0);
  box.subframe_poses[2].orientation = tf2::toMsg(orientation);

  box.subframe_names[3] = "corner_2";
  box.subframe_poses[3].position.x = .025;
  box.subframe_poses[3].position.y = -.05;
  box.subframe_poses[3].position.z = -.01 + z_offset_box;
  orientation.setRPY((90.0 / 180.0 * M_PI), 0, 0);
  box.subframe_poses[3].orientation = tf2::toMsg(orientation);

  box.subframe_names[4] = "side";
  box.subframe_poses[4].position.x = .0;
  box.subframe_poses[4].position.y = .0;
  box.subframe_poses[4].position.z = -.01 + z_offset_box;
  orientation.setRPY(0, (180.0 / 180.0 * M_PI), 0);
  box.subframe_poses[4].orientation = tf2::toMsg(orientation);

  moveit_msgs::CollisionObject cylinder;
  // Spawning on the fingers causes collisions, and it's not easy to allow the collisions
  // through the planning scene interface. TODO(felixvd)
  double z_offset_cylinder = .12;
  cylinder.header.frame_id = "panda_hand";
  cylinder.id = "cylinder";
  cylinder.primitives.resize(1);
  cylinder.primitive_poses.resize(1);
  cylinder.primitives[0].type = box.primitives[0].CYLINDER;
  cylinder.primitives[0].dimensions.resize(2);
  cylinder.primitives[0].dimensions[0] = 0.06;   // height (along x)
  cylinder.primitives[0].dimensions[1] = 0.005;  // radius
  cylinder.primitive_poses[0].position.x = 0.0;
  cylinder.primitive_poses[0].position.y = 0.0;
  cylinder.primitive_poses[0].position.z = 0.0 + z_offset_cylinder;
  orientation.setRPY(0, (90.0 / 180.0 * M_PI), 0);
  cylinder.primitive_poses[0].orientation = tf2::toMsg(orientation);

  cylinder.subframe_poses.resize(1);
  cylinder.subframe_names.resize(1);
  cylinder.subframe_names[0] = "tip";
  cylinder.subframe_poses[0].position.x = 0.03;
  cylinder.subframe_poses[0].position.y = 0.0;
  cylinder.subframe_poses[0].position.z = 0.0 + z_offset_cylinder;
  orientation.setRPY(0, (90.0 / 180.0 * M_PI), 0);
  cylinder.subframe_poses[0].orientation = tf2::toMsg(orientation);

  // Publish each object
  moveit_msgs::CollisionObject co;
  co = box;
  co.operation = moveit_msgs::CollisionObject::ADD;
  planning_scene_interface.applyCollisionObject(co);
  co = cylinder;
  co.operation = moveit_msgs::CollisionObject::ADD;
  planning_scene_interface.applyCollisionObject(co);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_subframes");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(10.0);

  // Prepare scene (spawn objects and attach cylinder to robot)
  addCollisionObjects(planning_scene_interface);
  moveit_msgs::AttachedCollisionObject att_coll_object;
  att_coll_object.object.id = "cylinder";
  att_coll_object.link_name = "panda_hand";
  att_coll_object.object.operation = att_coll_object.object.ADD;
  ROS_INFO_STREAM("Attaching cylinder to robot.");
  planning_scene_interface.applyAttachedCollisionObject(att_coll_object);

  //---
  int c;
  tf2::Quaternion orientation, orientation2, orientation3;
  geometry_msgs::PoseStamped ps, ps_0;
  ps.header.frame_id = "panda_link0";
  ps.pose.position.y = -.4;
  ps.pose.position.z = .3;
  orientation.setRPY(0, (-20.0 / 180.0 * M_PI), 0);
  ps.pose.orientation = tf2::toMsg(orientation);

  while (ros::ok())
  {
    ROS_INFO("Press a key to start testing stuff. \n0 to exit"
             "\n1 to spawn box and cylinder."
             "\n2 to attach cylinder to the gripper"
             "\n3 to move to start pose "
             "\n4 to move cylinder tip to box bottom \n5 to move cylinder tip to box top"
             "\n6 to move cylinder tip to box corner 1 \n7 to move cylinder tip to box corner 2"
             "\n8 to move cylinder tip to side of box"
             "\n10 to remove box and cylinder from the scene."
             "\n11 to move to a certain point in space with end effector (for visualizing the subframe pose)"
             "\n12 to move to a certain point in space with cylinder/tip ");
    std::cin >> c;
    if (c == 0)
    {
      return true;
    }
    else if (c == 1)
    {
      ROS_INFO_STREAM("Respawning test box and cylinder.");
      addCollisionObjects(planning_scene_interface);
    }
    else if (c == 2)
    {
      moveit_msgs::AttachedCollisionObject att_coll_object;
      att_coll_object.object.id = "cylinder";
      att_coll_object.link_name = "panda_hand";
      att_coll_object.object.operation = att_coll_object.object.ADD;
      ROS_INFO_STREAM("Attaching cylinder to robot.");
      planning_scene_interface.applyAttachedCollisionObject(att_coll_object);
    }
    else if (c == 3)
    {
      // Go to neutral home pose
      group.clearPoseTargets();
      group.setNamedTarget("ready");
      group.move();
    }
    else if (c == 4)
    {
      ROS_INFO_STREAM("Moving to bottom of box with cylinder tip");
      ps_0.header.frame_id = "box/bottom";
      orientation.setRPY(0, (180.0 / 180.0 * M_PI), 0);
      // Use a second rotation to make the transformation easier to follow.
      // orientation2.setRPY(0, 0, -(90.0/180.0 *M_PI));
      // TODO(felixvd): Why is the line above not producing the pose I expected from the line below? Am I being stupid
      // or is there a bug?
      orientation2.setRPY(-(90.0 / 180.0 * M_PI), 0, 0);
      orientation3 = orientation * orientation2;
      ps_0.pose.orientation = tf2::toMsg(orientation3);
      ps_0.pose.position.z = 0.01;
      moveToCartPose(ps_0, group, "cylinder/tip");
    }
    else if (c == 5)
    {
      ROS_INFO_STREAM("Moving to top of box with cylinder tip");
      ps_0.header.frame_id = "box/top";
      orientation.setRPY(0, (180.0 / 180.0 * M_PI), 0);
      orientation2.setRPY(-(90.0 / 180.0 * M_PI), 0, 0);
      orientation3 = orientation2 * orientation;
      ps_0.pose.orientation = tf2::toMsg(orientation3);
      ps_0.pose.position.z = 0.01;
      moveToCartPose(ps_0, group, "cylinder/tip");
    }
    else if (c == 6)
    {
      ps_0.header.frame_id = "box/corner_1";
      orientation.setRPY(0, (180.0 / 180.0 * M_PI), 0);
      orientation2.setRPY(-(90.0 / 180.0 * M_PI), 0, 0);
      orientation3 = orientation * orientation2;
      ps_0.pose.orientation = tf2::toMsg(orientation3);
      ps_0.pose.position.z = 0.01;
      moveToCartPose(ps_0, group, "cylinder/tip");
    }
    else if (c == 7)
    {
      ps_0.header.frame_id = "box/corner_2";
      orientation.setRPY(0, (180.0 / 180.0 * M_PI), 0);
      orientation2.setRPY(-(90.0 / 180.0 * M_PI), 0, 0);
      orientation3 = orientation * orientation2;
      ps_0.pose.orientation = tf2::toMsg(orientation3);
      ps_0.pose.position.z = 0.01;
      moveToCartPose(ps_0, group, "cylinder/tip");
    }
    else if (c == 8)
    {
      ps_0.header.frame_id = "box/side";
      orientation.setRPY(0, (180.0 / 180.0 * M_PI), 0);
      orientation2.setRPY(-(90.0 / 180.0 * M_PI), 0, 0);
      orientation3 = orientation * orientation2;
      ps_0.pose.orientation = tf2::toMsg(orientation3);
      ps_0.pose.position.z = 0.01;
      moveToCartPose(ps_0, group, "cylinder/tip");
    }
    else if (c == 10)
    {
      try
      {
        ROS_INFO_STREAM("Removing box and cylinder.");
        moveit_msgs::AttachedCollisionObject att_coll_object;
        att_coll_object.object.id = "box";
        att_coll_object.object.operation = att_coll_object.object.REMOVE;
        planning_scene_interface.applyAttachedCollisionObject(att_coll_object);

        att_coll_object.object.id = "cylinder";
        att_coll_object.object.operation = att_coll_object.object.REMOVE;
        planning_scene_interface.applyAttachedCollisionObject(att_coll_object);

        moveit_msgs::CollisionObject co;
        co.id = "box";
        co.operation = moveit_msgs::CollisionObject::REMOVE;
        planning_scene_interface.applyCollisionObject(co);

        co.id = "cylinder";
        co.operation = moveit_msgs::CollisionObject::REMOVE;
      }
      catch (const std::exception& exc)
      {
        ROS_WARN_STREAM(exc.what());
      }
    }
    else if (c == 11)
    {
      ROS_INFO_STREAM("Moving to a pose using robot tip");
      moveToCartPose(ps, group, "panda_hand");
    }
    else if (c == 12)
    {
      ROS_INFO_STREAM("Moving to a pose with cylinder tip");
      moveToCartPose(ps, group, "cylinder/tip");
    }
    else
    {
      ROS_INFO("Could not read input. Quitting.");
      break;
    }
  }

  ros::waitForShutdown();
  return 0;
}
