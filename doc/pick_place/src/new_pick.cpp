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

/* Author: Ioan Sucan */

#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <eigen_conversions/eigen_msg.h>
// #include <Eigen/Geometry>
// #include <geometry_msgs/Pose.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  //  NEED TO SET THIS ABHI!!!
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::Grasp> grasps;
  // double i= 0.05;
  geometry_msgs::PoseStamped p;
  // while (i <= 0.13)
  // {
  //   std::cout<<i<<"\n";
  
  p.header.frame_id = "panda_link0";
  p.pose.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(-1.5707963267948966, -0.7853981633974483, -1.5707963267948966);
  // p.orientation.z = -0.383;
  // p.orientation.w = 0.924;
  // there is additional 0.165 to the base of the fingers
  p.pose.position.x = 0.415;
  p.pose.position.y = 0;
  p.pose.position.z = 0.5;

  // distance b/w hand and finger_start is 0.058; 0.442

  moveit_msgs::Grasp g;
  g.grasp_pose = p;
  g.pre_grasp_approach.direction.vector.x = 1.0;
  g.pre_grasp_approach.direction.header.frame_id = "panda_link0";
  g.pre_grasp_approach.min_distance = 0.095;
  g.pre_grasp_approach.desired_distance = 0.115;
  // g.pre_grasp_approach.min_distance = i - 0.01;
  // g.pre_grasp_approach.desired_distance = i;

  g.post_grasp_retreat.direction.header.frame_id = "panda_link0";
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.25;

  openGripper(g.pre_grasp_posture);

  closedGripper(g.grasp_posture);

  grasps.push_back(g);
  // i += 0.01;
  // }
  move_group.setSupportSurfaceName("table1");
  move_group.pick("cylinder", grasps);

  // move_group.setPoseTarget(p.pose);
  // move_group.move();
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;

  geometry_msgs::PoseStamped p;
  // while (i <= 0.13)
  // {
  //   std::cout<<i<<"\n";
  
  p.header.frame_id = "panda_link0";
  // This rotates the current state using the following rollpitchyaw values
  p.pose.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(0, 0, 1.5707963267948966);
  // p.orientation.z = -0.383;
  // p.pose.orientation.w = 1;
  // there is additional 0.165 to the base of the fingers
  p.pose.position.x = 0;
  p.pose.position.y = 0.415;
  p.pose.position.z = 0.5;

  // geometry_msgs::PoseStamped p;
  // p.header.frame_id = "base_footprint";
  // p.pose.position.x = 0.7;
  // p.pose.position.y = 0.0;
  // p.pose.position.z = 0.5;
  // p.pose.orientation.x = 0;
  // p.pose.orientation.y = 0;
  // p.pose.orientation.z = 0;
  // p.pose.orientation.w = 1;
  moveit_msgs::PlaceLocation g;
  g.place_pose = p;

  g.pre_place_approach.direction.vector.z = -1.0;
  g.pre_place_approach.direction.header.frame_id = "panda_link0";
  g.pre_place_approach.min_distance = 0.095;
  g.pre_place_approach.desired_distance = 0.115;
  // g.pre_grasp_approach.min_distance = i - 0.01;
  // g.pre_grasp_approach.desired_distance = i;

  g.post_place_retreat.direction.header.frame_id = "panda_link0";
  g.post_place_retreat.direction.vector.x = -1.0;
  g.post_place_retreat.min_distance = 0.1;
  g.post_place_retreat.desired_distance = 0.25;




  // g.pre_place_approach.direction.vector.z = -1.0;

  // g.post_place_retreat.direction.vector.x = -1.0;

  // g.post_place_retreat.direction.header.frame_id = "base_footprint";
  // g.pre_place_approach.direction.header.frame_id = "r_wrist_roll_link";

  // g.pre_place_approach.min_distance = 0.1;
  // g.pre_place_approach.desired_distance = 0.2;

  // g.post_place_retreat.min_distance = 0.1;
  // g.post_place_retreat.desired_distance = 0.25;

  openGripper(g.post_place_posture);

  loc.push_back(g);
  // group.setSupportSurfaceName("table2");

  // add path constraints
  // moveit_msgs::Constraints constr;
  // constr.orientation_constraints.resize(1);
  // moveit_msgs::OrientationConstraint& ocm = constr.orientation_constraints[0];
  // ocm.link_name = "r_wrist_roll_link";
  // ocm.header.frame_id = p.header.frame_id;
  // ocm.orientation.x = 0.0;
  // ocm.orientation.y = 0.0;
  // ocm.orientation.z = 0.0;
  // ocm.orientation.w = 1.0;
  // ocm.absolute_x_axis_tolerance = 0.2;
  // ocm.absolute_y_axis_tolerance = 0.2;
  // ocm.absolute_z_axis_tolerance = M_PI;
  // ocm.weight = 1.0;
  //  group.setPathConstraints(constr);
  // group.setPlannerId("RRTConnectkConfigDefault");

  group.place("cylinder", loc);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_co1 = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(45.0);

  moveit_msgs::CollisionObject collision_object;

  // remove table
  collision_object.id = "table1";
  collision_object.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co1.publish(collision_object);

  // // add table
  // collision_object.operation = moveit_msgs::CollisionObject::ADD;
  // collision_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.1;
  // collision_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.3;
  // collision_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.2;
  // collision_object.primitive_poses[0].position.x = 1.5;
  // collision_object.primitive_poses[0].position.y = 0;
  // collision_object.primitive_poses[0].position.z = 1.2;
  // pub_co.publish(collision_object);



  shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.CYLINDER;
  // primitive.dimensions.resize(2);
  // // height of cylinder
  // primitive.dimensions[0] = 0.1;
  // // radius of cylinder
  // primitive.dimensions[1] = 0.02;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  // height of cylinder
  primitive.dimensions[0] = 0.2;
  // radius of cylinder
  primitive.dimensions[1] = 0.6;
  primitive.dimensions[2] = 0.4;

  // Define a pose for the cylinder (specified relative to frame_id)
  geometry_msgs::Pose cylinder_pose;
  // cylinder_pose.orientation.w = 1;
  // Set the position of cylinder
  cylinder_pose.position.x = 0.5;
  cylinder_pose.position.y = 0;
  cylinder_pose.position.z = 0.2;

  // Add cylinder as collision object
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(cylinder_pose);
  collision_object.operation = collision_object.ADD;
  pub_co1.publish(collision_object);








  // remove table
  // collision_object.id = "table2";
  // collision_object.operation = moveit_msgs::CollisionObject::REMOVE;
  // pub_co.publish(collision_object);

  // // add table
  // collision_object.operation = moveit_msgs::CollisionObject::ADD;
  // collision_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
  // collision_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
  // collision_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
  // collision_object.primitive_poses[0].position.x = 0.7;
  // collision_object.primitive_poses[0].position.y = -0.2;
  // collision_object.primitive_poses[0].position.z = 0.175;
  // pub_co.publish(collision_object);

  collision_object.header.frame_id = "panda_link0";
  collision_object.id = "cylinder";

  collision_object.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(collision_object);

  moveit_msgs::AttachedCollisionObject aco;
  // aco.link_name = "panda_link0";
  aco.object = collision_object;
  pub_aco.publish(aco);

  // moveit_msgs::AttachedCollisionObject aco;
  // aco.object = collision_object;
  // pub_aco.publish(aco);

  // Define a cylinder which will be added to the world.
  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.CYLINDER;
  // primitive.dimensions.resize(2);
  // // height of cylinder
  // primitive.dimensions[0] = 0.1;
  // // radius of cylinder
  // primitive.dimensions[1] = 0.02;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  // height of cylinder
  primitive.dimensions[0] = 0.02;
  // radius of cylinder
  primitive.dimensions[1] = 0.02;
  primitive.dimensions[2] = 0.2;

  // Define a pose for the cylinder (specified relative to frame_id)
  // geometry_msgs::Pose cylinder_pose;
  // cylinder_pose.orientation.w = 0;
  // Set the position of cylinder
  cylinder_pose.position.x = 0.5;
  cylinder_pose.position.y = 0;
  cylinder_pose.position.z = 0.5;

  // Add cylinder as collision object
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(cylinder_pose);
  collision_object.operation = collision_object.ADD;
  pub_co.publish(collision_object);
  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.push_back(collision_object);
  // planning_scene_interface.applyCollisionObjects(collision_objects);

  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();

  pick(group);

  ros::WallDuration(1.0).sleep();

  place(group);

  ros::waitForShutdown();
  return 0;
}
