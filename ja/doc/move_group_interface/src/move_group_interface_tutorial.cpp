/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // セットアップ
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 
  // MoveItでは "planning groups" と呼ばれるロボットの関節情報を操作し，その操作の情報は `JointModelGroup` に格納します．
  // MoveItではこの "planning group" と "joint model group" を交互に使用していきます．
  static const std::string PLANNING_GROUP = "panda_arm";
  
  // :move_group_interface:`MoveGroupInterface` クラスは
  // プランニングに使用したい"planning group"の名前を設定するだけで簡単に使用することができます．

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // :planning_scene_interface:`PlanningSceneInterface` クラスによりプランニングを行う仮想環境へ障害物を追加したり，削除したりすることができます．
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // アクセス回数の多くなる"planning group"は，生ポインタを利用することで，実行速度パフォーマンスを向上させます．
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // 表示
  // ^^^^^^^^^^^^^
  //
  // "MoveItVisualTools"パッケージの利用により，障害物等の物体，ロボット，そして生成軌道等，多くのRVizへの表示機能を使うができるようになり，動作を一つずつ確認するといったデバッグツールとしての利用も可能になります．
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // 遠隔操作により，RViz内のキーボードショートカット等を利用した便利なスクリプトを利用可能になります．
  visual_tools.loadRemoteControl();

  // RVizでは多くのマーカを利用可能ですが，本デモではテキスト，円柱，球のマーカを使用します．
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // 逐次Publishを行うことで，RVizでの表示に使うメッセージの数を減らします．
  visual_tools.trigger();

  // 基本的な情報の取得
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // ロボットの座標系の名前を取得することができます．
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // また，対象グループのエンドエフェクタの名前の取得も可能です．
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // ロボットのグループ一覧の取得もできます．
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
    std::ostream_iterator<std::string>(std::cout, ", "));

  // デモを開始する
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // 目標姿勢までプランニングする
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 設定したエンドエフェクタの姿勢まで，指定のグループのモーションプランニングを行うことができます．
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  // これでプランナを呼び出してプランニングを行い，その結果を表示することができます．
  // ここではプランニングを行っているのみであり，"move_group"に実際にロボットを動かすように指示はしていないことに注意してください．
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // プランニング結果の表示
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // RViz上にプランニングの結果を線とマーカで表示することができます．
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 実際に目標姿勢まで動かす
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // 指定した姿勢まで動かすのはこれまでの手順とほとんど同じですが，
  // ここでは"move()"関数を使用します．
  // これまでの手順で指定した目標姿勢はまだ有効なので，
  // ロボットはこれに向かって動こうとします．
  // 実際にロボットを動かすには，軌道の実行に成功した際に"success"と返すようなコントローラを有効にする必要があります．
  // ここまでのチュートリアルではそのコントローラの設定は行っていないため，ここでは"move()"関数はコメントアウトしています．

  /* 実際にロボットを動かす場合には，下記のコメントアウトを外してください． */
  /* move_group.move(); */

  // 関節値指定での目標姿勢へのプランニング
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // 関節値で目標姿勢を設定して，動かしてみましょう．
  // これまで目標姿勢を指定していた部分を書き換えていきます．
  //
  // まず，ロボットの現在の状態を参照するためのポインタを生成します．
  // "RobotState"から現在の位置/速度/加速度の情報を取得することができます．
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // 続いて，指定のグループの関節角度を取得します．
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // 関節角度を一つ変更して目標姿勢を変更したら，プランニングを行い，その結果を表示してみましょう．
  joint_group_positions[0] = -1.0;  // ラジアン指定
  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // RVizで結果を表示する．
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 拘束条件付のプランニング
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // 拘束条件設定は，ロボットのリンク名を指定することで簡単に行なえます．
  // 指定のグループで実際に拘束条件と目標姿勢を設定してみましょう．
  // まず，拘束条件を設定します．
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // 次に，設定した拘束条件をグループに設定します．
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  // 目標姿勢はこれまでに使用したものを利用します．
  // ただし，現在の状態が拘束条件を満たしている必要があるので，
  // 開始姿勢を変更していることに注意してください．
  robot_state::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  // 新しく設定した開始姿勢から，
  // これまでに指定している目標姿勢まで動かしてみましょう．
  move_group.setPoseTarget(target_pose1);

  // 毎サンプルで逆運動学を解く必要があるため，拘束条件付でのプランニングには時間がかかります．
  // そこで，プランニングの制限時間を5秒から10秒に伸ばし，プランナが解を出せるように設定を変更しておきましょう．
  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // RVizで結果を表示する．
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // 拘束条件付のパスプランニングが完了したら，その拘束条件を削除するのを忘れないでください．
  move_group.clearPathConstraints();

  // 直動動作
  // ^^^^^^^^^^^^^^^^^^^^^^
  // エンドエフェクタの通過する軌道をウェイポイントのリストとして指定することで，直動動作のプランニングを行うことができます．
  // ただし，開始姿勢は拘束条件付プランニングの際に設定したものを使用します．
  // 開始姿勢はウェイポイントに追加する必要はありませんが，
  // 結果を確認する際に便利なので追加しています．
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // 下げる

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // 右へ

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // 上げて左へ

  // 直動動作は物体把持の際に利用されることがおおく，往々にしてゆっくりとした動作が必要になるかと思います．
  // そこでここでは，各関節の最大速度を元に速度を落としています．
  // ただし，この速度というのは，エンドエフェクタの速度ではないことに注意してください．
  move_group.setMaxVelocityScalingFactor(0.1);

  // 直動動作を1cmに区切りたいため，直動動作への変換最大ステップ（ `eef_step` ）を0.01に設定しています．
  // `jump_threshold` を0.0に設定しているのは，各ステップを飛ばさないように設定するためです．
  // **警告：ジャンプ閾値を無効にすると，
  // 冗長関節を持つロボットの場合は予測不可能な動作を起こし，
  // 危険な動作が起きる可能性があるので十分注意してください．**
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // 結果をRVizに表示する．
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 衝突物体の追加/削除とロボットへの設置/取り外し
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // 衝突物体のROSメッセージを定義します．
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // idを使って物体を識別します．
  collision_object.id = "box1";

  // 環境に追加する箱を定義します．
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  // 箱の姿勢を定義します．（"frame_id"に設定された座標系での値です．）
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // 実際に環境に衝突物体を追加してみましょう．
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // 状態をテキストでRVizに表示します．
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // "MoveGroup"に設定した物体のメッセージが届き，処理されるのを待ちます．
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // この状態でパスプランニングを行うと，物体を回避するような軌道が生成されます．
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.w = 1.0;
  another_pose.position.x = 0.4;
  another_pose.position.y = -0.4;
  another_pose.position.z = 0.9;
  move_group.setPoseTarget(another_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // RVizで結果を表示する．
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // 次に，ロボットに物体を取り付けてみましょう．
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group.attachObject(collision_object.id);

  // RVizで状態をテキストとして表示する．
  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

/* "MoveGroup"に設定した物体のメッセージが届き，処理されるのを待ちます．*/ 
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
    "robot");

  // 続いて，ロボットから物体を取り外してみましょう．
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group.detachObject(collision_object.id);

  // RVizで状態をテキストとして表示する．
  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* "MoveGroup"に設定した物体のメッセージが届き，処理されるのを待ちます．*/
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
    "robot");

  // 最後に，物体を環境から削除してみましょう．
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // RVizで状態をテキストとして表示する．
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* "MoveGroup"に設定した物体のメッセージが届き，処理されるのを待ちます. */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
