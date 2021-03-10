#include <ros/ros.h>
#include <moveit/task_constructor/stage.h>

using namespace moveit::task_constructor;

class MyPropagatingEitherWay : public PropagatingEitherWay
{
public:
  MyPropagatingEitherWay(const std::string& name) : PropagatingEitherWay(name)
  {
  }

  void computeForward(const InterfaceState& from) override
  {
    // do computation
    // ...
    // package that into a planning scene pointer
    planning_scene::PlanningScenePtr ptr;

    // send the solution forward to the next stage
    sendForward(from, InterfaceState(ptr), SubTrajectory());
  }

  void computeBackward(const InterfaceState& to) override
  {
    // do computation
    // ...
    // package that into a planning scene pointer
    planning_scene::PlanningScenePtr ptr;

    // send the solution backward to the previous stage
    sendBackward(InterfaceState(ptr), to, SubTrajectory());
  }

  void init(const moveit::core::RobotModelConstPtr& robot_model) override
  {
    PropagatingEitherWay::init(robot_model);
    robot_model_ = robot_model;
  }

private:
  moveit::core::RobotModelConstPtr robot_model_;
};

class MyGenerator : public Generator
{
public:
  MyGenerator(const std::string& name = "myGenerator") : Generator(name)
  {
  }

  void compute() override
  {
    compute_count++;

    // do computation
    // ...
    // package that into a planning scene pointer
    planning_scene::PlanningScenePtr ptr = std::make_shared<planning_scene::PlanningScene>(robot_model_);

    // spawn an interface state with the solution to be provided
    // at both ends of the stage
    spawn(InterfaceState(ptr), 0.0);
  }

  bool canCompute() const override
  {
    return compute_count < 1;
  }

  void init(const moveit::core::RobotModelConstPtr& robot_model) override
  {
    Generator::init(robot_model);
    robot_model_ = robot_model;
    compute_count = 0;
  }

private:
  unsigned short compute_count;
  moveit::core::RobotModelConstPtr robot_model_;
};

class MyMonitoringGenerator : public MonitoringGenerator
{
public:
  MyMonitoringGenerator(const std::string& name = "myMonitoringGenerator") : MonitoringGenerator(name)
  {
  }

  void onNewSolution(const SolutionBase& s) override
  {
    // Perform the following computation with the solution s, that the
    // the monitored stage just computed.
    // ...
  }

  void compute() override
  {
    compute_count++;

    // do computation
    // ...
    // package that into a planning scene pointer
    planning_scene::PlanningScenePtr ptr = std::make_shared<planning_scene::PlanningScene>(robot_model_);

    // spawn an interface state with the solution to be provided
    // at both ends of the stage
    spawn(InterfaceState(ptr), 0.0);
  }

  bool canCompute() const override
  {
    return compute_count < 1;
  }

  void init(const moveit::core::RobotModelConstPtr& robot_model) override
  {
    Generator::init(robot_model);
    robot_model_ = robot_model;
    compute_count = 0;
  }

private:
  unsigned short compute_count;
  moveit::core::RobotModelConstPtr robot_model_;
};

class MyConnecting : public Connecting
{
public:
  MyConnecting(const std::string& name) : Connecting(name){};

  void compute(const InterfaceState& from, const InterfaceState& to) override
  {
    // do computation
    // ...
    // package that into a Solution to link between the two states
    SolutionBasePtr s;

    // connect the two states using the solution above
    connect(from, to, s);
  }

  void init(const moveit::core::RobotModelConstPtr& robot_model) override
  {
    Connecting::init(robot_model);
    robot_model_ = robot_model;
  }

private:
  moveit::core::RobotModelConstPtr robot_model_;
};

int main(int argc, char** argv)
{
  const std::string node_name = "extension_tutorial";
  ros::init(argc, argv, node_name);
  ros::NodeHandle n;

  return 0;
}