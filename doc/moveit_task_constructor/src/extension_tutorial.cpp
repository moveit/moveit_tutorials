#include <ros/ros.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
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

  // Property Acces

  // BEGIN_TUTORIAL
  // Multiple properties can be grouped into a ``PropertyMap`` class, which groups them as
  // ``(name, property)`` key-value pairs.
  // Through the ``PropertyMap``, given a key, you have access to either the ``boost::any``
  // value or the entire corresponding ``Property`` object.
  // Every stage has a property map as a member, which can be accessed by using the
  // ``properties()`` function.
  //
  // Basic Operations with Properties
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Lets define and inspect some properties for our custom generator stage,
  // which we implemented in section `Programmatic Extension`_. First, we need to create
  // an instance of a stage. In this case, we chose to store it in a smart pointer, making it
  // easy for us to keep only one instance at a time of that stage in our code.
  auto g = std::make_unique<MyGenerator>("myGenerator");
  // The ``properties()`` function grants access to the property map of the stage.
  PropertyMap pm = g->properties();
  // Write and Declare Properties
  // ++++++++++++++++++++++++++++
  // A property has two values, a current and a default value. Use the following methods to modify
  // these values utilizing key-value notation:
  /* Declare a property for future use */
  g->properties().declare(std::string("Foo"), std::string("Foo Description"));

  /* Set both, the current and the default value */
  g->properties().set("Foo", "Bar");
  g->setProperty("Foo", "Bar");

  /* Set the current value only */
  g->properties().setCurrent("Foo", std::string("Bar"));
  g->properties().property("Foo").setCurrentValue(std::string("Bar"));  // only if Foo is already defined

  /* Set the default value only */
  g->properties().property("Foo").setDefaultValue(std::string("Bar"));  // only if Foo is already defined
  // You can also provide a description after the value has already been initialized
  g->properties().property("Foo").setDescription("FooBar Description!");
  // Alternatively to the above mentioned ways, you may initiate a property based on an
  // already existing one.
  /* configure already initialized properties */
  g->properties().declare(std::string("Foo2"), std::string("Foo2 Description"));
  g->properties().property("Foo2").configureInitFrom(Stage::MANUAL, "Foo");
  // Retrieve Properties
  // +++++++++++++++++++
  // To extract a property from the property map of a stage, you can use the ``property()`` function that
  // we implicitly already introduced in the code of the previous section `Write and Declare Properties`_.
  Property foo = g->properties().property("Foo");
  // First, lets check general meta data of the property:
  /* Check if the property is defined */
  bool def = foo.defined();

  /* Retrieve the description text */
  const std::string des = foo.description();

  /* Get the data type of the stored value */
  std::string typ = foo.typeName();
  // Next, lets retrieve the actual data that the property holds:
  /* current value */
  const boost::any curVal = foo.value();

  /* default value */
  const boost::any defVal = foo.defaultValue();

  // Instead of getting the value in its ``boost::any`` form, the property class provides a function to serialize
  // it into a string like so:
  std::string s = foo.serialize();

  // The Property Map
  // ^^^^^^^^^^^^^^^^
  // The property map allows us to check the metadata from a global scope:
  /* Check if property is declared */
  bool hp = g->properties().hasProperty("Foo");
  // You can also access the properties with an iterator:
  for (auto& elem : g->properties())
  {
    std::cout << "---" << elem.first << " ; " << elem.second.serialize() << " ; " << elem.second.description()
              << std::endl;
  }
  /* Output for our previously declared properties:
    Foo ; Bar ; FooBar Description!
    forwarded_properties ;  ; set of interface properties to forward
    marker_ns ; generator ; marker namespace
    timeout ;  ; timeout per run (s)
  */
  // Notice, that the output also shows, additional to our specified ``Foo`` property,
  // key-value pairs that are standard to all stages. You can duplicate the
  // configuration of an existing stage into another stage
  // using `Property Initializer Sources`.
  //
  // As an example scenario, lets create two generators and connect them together.
  // Using the property Initializer sources, you can specify from which generator the
  // connect stage will get its properties.
  // In the following example, we tell the connect stage to inherit its properties from
  // the parent stage, i.e. the first generator in the task hierarchy.
  /* Create a new task */
  Task t;
  t.loadRobotModel();
  /* Create a new stage stage to connect the generator to the following stages and set it to inherit the properties of the previous stage*/
  auto c = std::make_unique<stages::Connect>(
      "connect", stages::Connect::GroupPlannerVector{ { "panda_arm", std::make_shared<solvers::PipelinePlanner>() } });
  c->properties().configureInitFrom(Stage::PropertyInitializerSource::PARENT);
  /* Create a second generator stage. We will use this one down below in the next code section. */
  auto g2 = std::make_unique<MyGenerator>("generate!");
  // You can also load a set of properties from another property map into an unrelated stage like so:
  /* Perform initialization of still undefined properties */
  PropertyMap init = PropertyMap();
  g->properties().exposeTo(init, { "Foo", "Foo2" });
  g->properties().performInitFrom(Stage::PropertyInitializerSource::MANUAL, init);

  /* Add our generator and connecting stages to the task hierarchy */
  t.add(std::move(g));
  t.add(std::move(c));
  t.add(std::move(g2));
  // END_TUTORIAL

  try
  {
    t.plan();
  }
  catch (const moveit::task_constructor::InitStageException& e)
  {
    std::cerr << "planning failed with exception" << std::endl << e << t;
  }
  ros::waitForShutdown();

  return 0;
}