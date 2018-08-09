// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Grasp
#include <moveit_grasps/grasp_generator.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_grasps/grasp_plan.h>

// robot specific properties
#include <moveit_grasps/grasp_data.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_grasps
{
class GraspPlannerTest
{
public:
  // BEGIN_SUB_TUTORIAL tester_class_constructor
  GraspPlannerTest() : nh_("~")
  {
    // Get arm info from param server
    const std::string parent_name = "grasp_planner_test";  // for namespacing logging messages
    
    // TODO(@ridhwanluthra): Take these from the parameter server
    // rosparam_shortcuts::get(parent_name, nh_, "planning_group_name", planning_group_name_);
    // rosparam_shortcuts::get(parent_name, nh_, "ee_group_name", ee_group_name_);

    ee_group_name_ = "hand";
    planning_group_name_ = "panda_arm";

    ROS_INFO_STREAM_NAMED(parent_name, "End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED(parent_name, "Planning Group: " << planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // BEGIN_SUB_TUTORIAL planning_scene_monitor
    // Initialise a planning scene monitor
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    /* Check if the planning scene is loaded */
    if (planning_scene_monitor_->getPlanningScene())
    {
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "grasping_planning_scene");
      planning_scene_monitor_->getPlanningScene()->setName("grasping_planning_scene");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(parent_name, "Planning scene not configured");
      return;
    }
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL init_move_group
    // Initialize move group interface. |br|
    // This impliments functinality to call the pick pipeline.
    move_group_.reset(new moveit::planning_interface::MoveGroupInterface("panda_arm"));
    // END_SUB_TUTORIAL

    const robot_model::RobotModelConstPtr robot_model = planning_scene_monitor_->getRobotModel();
    // get the joint model group for the main robot arm
    arm_jmg_ = robot_model->getJointModelGroup(planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // BEGIN_SUB_TUTORIAL init_visual_tools
    // Load the visualization Tools for publishing to Rviz |br|
    // They are critical in debugging your application by visualizing various aspects of the grasp.
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(robot_model->getModelFrame(), "/rviz_visual_tools",
                                                                   planning_scene_monitor_));
    /* Load various useful components */
    visual_tools_->loadTrajectoryPub();
    visual_tools_->loadRobotStatePub();
    visual_tools_->loadSharedRobotState();
    visual_tools_->getSharedRobotState()->setToDefaultValues();
    visual_tools_->publishRobotState(visual_tools_->getSharedRobotState());
    robot_state::RobotStatePtr kinematic_state = visual_tools_->getSharedRobotState();
    // END_SUB_TUTORIAL

    // ---------------------------------------------------------------------------------------------
    // BEGIN_SUB_TUTORIAL grasp_data
    // Load grasp data specific to our robot. |br|
    // This is a crucial Step for any application using any part of this package. GraspData class is responsible
    // for loading all the parameters related to the robot and grasping.
    grasp_data_.reset(new GraspData(nh_, ee_group_name_, visual_tools_->getRobotModel()));
    // END_SUB_TUTORIAL

    // ---------------------------------------------------------------------------------------------
    // Clear out old collision objects
    visual_tools_->removeAllCollisionObjects();

    // ---------------------------------------------------------------------------------------------
    // BEGIN_SUB_TUTORIAL grasp_generator
    // Load grasp generator |br|
    // This class is important for generating all the possible grasps. The grasps generated here may not be viable.
    grasp_generator_.reset(new moveit_grasps::GraspGenerator(visual_tools_));
    // END_SUB_TUTORIAL

    // ---------------------------------------------------------------------------------------------
    // BEGIN_SUB_TUTORIAL grasp_planner
    // Load grasp planner |br|
    // This impliments the plan_grasps serivce used by the function call |code_start| moveGroupInterface::planGraspAndPick();\ |code_end|
    grasp_planner_.reset(new moveit_grasps::GraspPlanner(nh_));
    // END_SUB_TUTORIAL

    // ---------------------------------------------------------------------------------------------
    // Clear Markers
    visual_tools_->deleteAllMarkers();
    Eigen::Affine3d world_cs = Eigen::Affine3d::Identity();
    visual_tools_->publishAxis(world_cs);
  }

  bool cuboidTest()
  {
    geometry_msgs::Pose object_pose;
    object_pose.position.x = 0.5;
    object_pose.position.y = 0.0;
    object_pose.position.z = 0.5;
    double depth = 0.03;
    double width = 0.03;
    double height = 0.15;

    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);
    visual_tools_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);
    visual_tools_->trigger();

    // Generate set of grasps for one object
    ROS_INFO_STREAM_NAMED("test", "Generating cuboid grasps");
    // std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
    grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), depth, width, height, grasp_data_,
                                     grasp_candidates_);
    grasp_planner_->setCandidateGrasps(grasp_candidates_);
  }

  bool cylinderTest()
  {
    geometry_msgs::Pose object_pose;
    object_pose.position.x = 0.5;
    object_pose.position.y = 0.0;
    object_pose.position.z = 0.5;
    double radius = 0.015;
    double height = 0.15;

    // visual_tools_->publishCollisionCylinder(object_pose, "test_cylinder", radius, height, rviz_visual_tools::TRANSLUCENT_DARK);
    visual_tools_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);
    visual_tools_->trigger();

    // Generate set of grasps for one object
    ROS_INFO_STREAM_NAMED("test", "Generating cuboid grasps");
    std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
    grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), radius, height, grasp_data_,
                                     grasp_candidates);
  }

  void addTable()
  {
    // Add the first table where the cube will be kept.
    moveit_msgs::CollisionObject table_object;
    table_object.id = "table1";
    table_object.header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    table_object.primitives.resize(1);
    table_object.primitives[0].type = table_object.primitives[0].BOX;
    table_object.primitives[0].dimensions.resize(3);
    table_object.primitives[0].dimensions[0] = 0.2;
    table_object.primitives[0].dimensions[1] = 0.4;
    table_object.primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    table_object.primitive_poses.resize(1);
    table_object.primitive_poses[0].position.x = 0.5;
    table_object.primitive_poses[0].position.y = 0;
    table_object.primitive_poses[0].position.z = 0.2;
    // END_SUB_TUTORIAL

    table_object.operation = table_object.ADD;
    collision_objects.push_back(table_object);
  }

  void addCuboid()
  {
    moveit_msgs::CollisionObject cuboid;
    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    cuboid.header.frame_id = "panda_link0";
    cuboid.id = "object";

    /* Define the primitive and its dimensions. */
    cuboid.primitives.resize(1);
    cuboid.primitives[0].type = cuboid.primitives[0].BOX;
    cuboid.primitives[0].dimensions.resize(3);
    cuboid.primitives[0].dimensions[0] = 0.02;
    cuboid.primitives[0].dimensions[1] = 0.02;
    cuboid.primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
    cuboid.primitive_poses.resize(1);
    cuboid.primitive_poses[0].position.x = 0.5;
    cuboid.primitive_poses[0].position.y = 0;
    cuboid.primitive_poses[0].position.z = 0.5;
    // END_SUB_TUTORIAL

    cuboid.operation = cuboid.ADD;
    collision_objects.push_back(cuboid);
  }

  void addCylinder()
  {
    moveit_msgs::CollisionObject cylinder;
    cylinder.header.frame_id = "panda_link0";
    cylinder.id = "object";

    // Define a cylinder which will be added to the world.
    cylinder.primitives.resize(1);
    cylinder.primitives[0].type = cylinder.primitives[0].CYLINDER;
    cylinder.primitives[0].dimensions.resize(2);
    /* Setting height of cylinder. */
    cylinder.primitives[0].dimensions[0] = 0.2;
    /* Setting radius of cylinder. */
    cylinder.primitives[0].dimensions[1] = 0.01;

    // Define a pose for the cylinder (specified relative to frame_id).
    // Setting the position of cylinder.
    cylinder.primitive_poses.resize(1);
    cylinder.primitive_poses[0].position.x = 0.5;
    cylinder.primitive_poses[0].position.y = 0.;
    cylinder.primitive_poses[0].position.z = 0.5;

    // Add cylinder as collision object
    cylinder.operation = cylinder.ADD;
    collision_objects.push_back(cylinder);
  }

  void addCollisionObjects()
  {
    addTable();
    addCuboid();
    // addCylinder();
    planning_scene_interface_.applyCollisionObjects(collision_objects);
  }

  void planGraspsAndPick(std::string object_name)
  {
    move_group_->setSupportSurfaceName("table1");
    move_group_->setPlanningTime(45.0);
    move_group_->planGraspsAndPick(object_name);
  }

  // void visualizeGrasps()
  // {
  //   grasp_generator_->visualizeAnimatedGrasps(grasp_candidates_, arm_jmg_, 1)
  // }

private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp generator
  moveit_grasps::GraspGeneratorPtr grasp_generator_;

  // Grasp Planner
  moveit_grasps::GraspPlannerPtr grasp_planner_;

  // Grasp candidate
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates_;

  // Tool for visualizing things in Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // data for generating grasps
  moveit_grasps::GraspDataPtr grasp_data_;

  // Shared planning scene (load once for everything)
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // planning_scene_interface (make sure this works)
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // create move group 
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  // move_group_.setPlanningTime(45.0);

  // Arm
  const robot_model::JointModelGroup* arm_jmg_;

  // collision objects
  std::vector<moveit_msgs::CollisionObject> collision_objects;

  // which baxter arm are we using
  std::string ee_group_name_;
  std::string planning_group_name_;

};  // end of class

}  // namespace

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "grasp_generation_test");

  moveit_grasps::GraspPlannerTest tester;

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // moveit::planning_interface::MoveGroupInterface group("panda_arm");
  // group.setPlanningTime(45.0);

  tester.addCollisionObjects();

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  tester.cuboidTest();
  // calling service
  std::cout << "make a call to service" << std::endl;
  tester.planGraspsAndPick("object");

  // pick(group);

  // ros::WallDuration(1.0).sleep();

  // place(group);

  ros::waitForShutdown();
  return 0;
  

  // // Seed random
  // // srand(ros::Time::now().toSec());

  // // Benchmark time
  // ros::Time start_time;
  // start_time = ros::Time::now();

  // // Run Tests
  // moveit_grasps::GraspGeneratorTest tester;
  // tester.cuboidTest();
  // // tester.cylinderTest();

  // // Benchmark time
  // double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  // ROS_INFO_STREAM_NAMED("", "Total time: " << duration);
  // std::cout << duration << "\t" << num_tests << std::endl;

  // ros::Duration(1.0).sleep();  // let rviz markers finish publishing

  // return 0;
}

// BEGIN_TUTORIAL
// Workflow For Using The Package
// ------------------------------
// Step 1 - Loading all the components
// CALL_SUB_TUTORIAL planning_scene_monitor
// CALL_SUB_TUTORIAL init_move_group
// CALL_SUB_TUTORIAL init_visual_tools
// CALL_SUB_TUTORIAL grasp_data
// CALL_SUB_TUTORIAL grasp_generator
// CALL_SUB_TUTORIAL grasp_planner
// END_TUTORIAL