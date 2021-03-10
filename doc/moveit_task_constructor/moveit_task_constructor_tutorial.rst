MoveIt Task Constructor
=======================

.. image:: images/mtc_example.png
   :width: 700px

The Task Constructor framework provides a flexible and transparent way to define and plan actions that consist of multiple interdependent subtasks.
It draws on the planning capabilities of MoveIt to solve individual subproblems in black-box planning stages.
A common interface, based on MoveIt's PlanningScene is used to pass solution hypotheses between stages.
The framework enables the hierarchical organization of basic stages using containers, allowing for sequential as well as parallel compositions.
For more details, please refer to the associated `ICRA 2019 publication`_.

.. _ICRA 2019 publication: https://pub.uni-bielefeld.de/download/2918864/2933599/paper.pdf

Getting Started
---------------

If you have not already done so, make sure you have completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

Installing MoveIt Task Constructor
----------------------------------

Install From Source
^^^^^^^^^^^^^^^^^^^

Go into your catkin workspace and initialize wstool if necessary (assuming `~/ws_moveit` as workspace path): ::

  cd ~/ws_moveit/src
  git clone https://github.com/ros-planning/moveit_task_constructor.git

Install missing packages with rosdep: ::

  rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO

Build the workspace: ::

  catkin build

Running the Demo
----------------

The MoveIt Task Constructor package contains several basic examples and a pick-and-place demo.
For all demos you should launch the basic environment: ::

  roslaunch moveit_task_constructor_demo demo.launch

Subsequently, you can run the individual demos: ::

  rosrun moveit_task_constructor_demo cartesian
  rosrun moveit_task_constructor_demo modular
  roslaunch moveit_task_constructor_demo pickplace.launch

.. tip::

  To debug your application code and
  generate a visual task representation,
  add a new panel to your `RViZ` display.
  If you use the provided launch file, this is already pre-configured.

  .. figure:: images/panel_1.png
    :width: 370px
    :alt: add mtc rviz panel_1
    :align: center

  Select the `Motion Planning Tasks` plugin from the
  ``moveit_task_constructor_visualization`` package.

  .. figure:: images/panel_2.png
    :width: 370px
    :alt: add mtc rviz panel_2
    :align: center

The `Motion Planning Tasks` panel outlines the hierarchical stage structure of the tasks.
When you select a particular stage, the list of successful and failed solutions will be
shown in the right-most window. Selecting one of those solutions will start its visualization.

.. image:: images/mtc_show_stages.gif
   :width: 700px

.. _basic_concepts:
Basic Concepts
--------------

The fundamental idea of MTC is that complex motion planning problems can be composed into a set of simpler subproblems.
The top-level planning problem is specified as a **Task** while all subproblems are specified by **Stages**.
Stages can be arranged in any arbitrary order and hierarchy only limited by the individual stages types.
The order in which stages can be arranged is restricted by the direction in which results are passed.
There are three possible stages relating to the result flow: generator, propagator, and connector stages:

**Generators** compute their results independently of their neighbor stages and pass them in both directions, backwards and forwards.
An example is an IK sampler for geometric poses where approaching and departing motions (neighbor stages) depend on the solution.

**Propagators** receive the result of one neighbor stage, solve a subproblem and then propagate their result to the neighbor on the opposite site.
Depending on the implementation, propagating stages can pass solutions forward, backward or in both directions separately.
An example is a stage that computes a Cartesian path based on either a start or a goal state.

**Connectors** do not propagate any results, but rather attempt to bridge the gap between the resulting states of both neighbors.
An example is the computation of a free-motion plan from one given state to another.

Additional to the order types, there are different hierarchy types allowing to encapsulate subordinate stages.
Stages without subordinate stages are called **primitive stages**, higher-level stages are called **container stages**.
There are three container types:

**Wrappers** encapsulate a single subordinate stage and modify or filter the results.
For example, a filter stage that only accepts solutions of its child stage that satisfy a certain constraint can be realized as a wrapper.
Another standard use of this type includes the IK wrapper stage, which generates inverse kinematics solutions based on planning scenes annotated with a pose target property.

**Serial Containers** hold a sequence of subordinate stages and only consider end-to-end solutions as results.
An example is a picking motion that consists of a sequence of coherent steps.

**Parallel Containers** combine set of subordinate stages and can be used for passing the best of alternative results, running fallback solvers or for merging multiple independent solutions.
Examples are running alternative planners for a free-motion plan, picking objects with the right hand or with the left hand as a fallback, or moving the arm and opening the gripper at the same time.

.. image:: images/mtc_stage_types.png
   :width: 700px

Stages not only support solving motion planning problems.
They can also be used for all kinds of state transitions, as for instance modifying the planning scene.
Combined with the possibility of using class inheritance it is possible to construct very complex behavior while only relying on a well-structured set of primitive stages.

.. tip::

  In general, it is a good habit to encapsulate
  application-specific `MoveIt Task Constructor` code within a ros-node.

Programmatic Extension
----------------------

You may find that the available stages do not suffice the needs of your application.
Given this situation, you have the option to derive your own classes from core
stage types and implement custom functionality. Your choice of core class determines
the structure, i.e. the `flow path` of incoming and outgoing data with respect to
your new stage.

Overview of core classes
^^^^^^^^^^^^^^^^^^^^^^^^

The section :ref:`basic_concepts` in this tutorial provides an overview of the stage types
together with result propagation directions that are available for programmatic extension.
Remember: Core classes define result flow.
For example, to specify bi-directional result propagation, you might derive from the
``Generator`` class.
You will find this pattern if you take a look at the ``CurrentState`` stage,
which `generates` the current state of the planning scene and forwards it to both interfaces.

When deriving from one of the core classes, you need to implement your new computation in
the provided virtual functions. This ensures that the `MTC` backend can call your code
in the right place when traversing through the task hierarchy.
The following table provides an overview of these functions as well as giving more,
already implemented, examples of programmatic extension stages.

+------------------------+-----------------------+-------------------------+
| Core Class             | Virtual Functions     | Example Stages          |
+========================+=======================+=========================+
| Generator              | | ``compute``         | | *CurrentState*        |
|                        | | ``canCompute``      | | *FixedState*          |
+------------------------+-----------------------+-------------------------+
| Monitoring Generator   | | ``compute``         | | *FixedCartesianPoses* |
|                        | | ``canCompute``      | | *GeneratePickPose*    |
|                        | | ``onNewSolution``   | | *GeneratePlacePose*   |
+------------------------+-----------------------+-------------------------+
| Connecting             | | ``compute``         | | *Connect*             |
+------------------------+-----------------------+-------------------------+
| Propagating Either Way | | ``computeForward``  | | *MoveRelative*        |
|                        | | ``computeBackward`` | | *MoveTo*              |
|                        |                       | | *ModifyPlanningScene* |
+------------------------+-----------------------+-------------------------+

.. note::
  **compute()**
    The functionality of a stage is encapsulated in
    the ``compute()`` function.
    ``Interface`` classes are utilized as input and outputs of a stage.

  **canCompute()**
    To be able to gain control of the execution of the compute function, ``canCompute()`` acts as a guard.

Implementation
^^^^^^^^^^^^^^

In case you just want to insert the stage and check for the functions later, it is valid
to leave the function body with no contents, i.e.:

.. code-block:: c++

  void compute() override {};

Additionally, you may define a custom constructor for your derived stage class and forward the arguments like so:

.. code-block:: c++

  class MyGenerator : public Generator{
    public:
      MyGenerator(const std::string& name) : Generator(name) {};

Solutions of stages are propagated via ``InterfaceStates``.
An interface state can only propagate planning scene instances.
The next section presents code examples for all the core classes that you may use as a
basic implementation reference.

Generator
+++++++++
A template for a ``Generator`` stage is listed below.
As stated above the ``compute()`` function assembles a planning scene and spawns an interface, whilst the
``canCompute()`` function acts as an execution guard.

.. literalinclude:: src/extension_tutorial.cpp
  :language: cpp
  :lines: 48-84

The example above additionally implements a mechanism to call the ``compute`` function only once.
Create the stage:

.. code-block:: c++

  auto g = std::make_unique<MyGenerator>("myGenerator");

Monitoring Generator
++++++++++++++++++++
The ``MonitoringGenerator`` processes a solution that the monitored stage just computed.

.. literalinclude:: src/extension_tutorial.cpp
  :language: cpp
  :lines: 86-129

Create the stage:

.. code-block:: c++

  auto m = std::make_unique<MyMonitoringGenerator>("myMonitoringGenerator");

PropagatingEitherWay
++++++++++++++++++++

The ``PropagatingEitherWay`` stage forwards solutions between interface states of previous and
following stages in a forward and backward manner.

.. literalinclude:: src/extension_tutorial.cpp
  :language: cpp
  :lines: 9-46

Create the stage:

.. code-block:: c++

  auto pr = std::make_unique<MyPropagatingEitherWay>("myPropagatingEitherWay");

Connecting
++++++++++

The Connecting stage computes a solution between interface states
from the previous and following stages.

.. literalinclude:: src/extension_tutorial.cpp
  :language: cpp
  :lines: 131-155

Create the stage:

.. code-block:: c++

  auto c = std::make_unique<MyConnecting>("myConnecting");

Stage Configuration
-------------------

Stages are configured with properties.
A property is a ``boost::any`` value stored inside the `Property` class, which provides the following
wrapper-like functionality:

- Maintain default and current values with get-/set-/reset-to-default functions.
- Provide a description.
- Serialization into strings.
- Get the ``boost::any`` value or the entire `Property` object for a given key.
- Check for type and if-defined status of the property.
- Initialization from an already existing property.

.. tutorial-formatter:: ./src/extension_tutorial.cpp

Notice, that when we initialize our properties from an external domain, we need to provide a property initializer source flag.
You can access these flags through the ``Stage::PropertyInitializerSource::`` scope.
They define a priority hierarchy in which initializations are carried out:

  ``MANUAL > DEFAULT > INTERFACE > PARENT``

Initialization of properties is carried out during planning of the entire
task hierarchy. You can therefore specify a priority hierarchy on a per-property-basis from where the stage should get
the inforation for its properties.

E.g. use the ``MANUAL`` flag if you want to explicitly configure a (set) of properties
from a property map. ``MANUAL`` takes precedence over all the other flags.
As another example, you can use ``INTERFACE`` and ``PARENT`` flags to let the stage be initialized
by its successor or predecessor.

To summarize, the property map allows you to:

- Declare properties for future use without providing values yet.
- Expose a subset of the properties to another property map.
- Reset all properties.
- Check if a key-value pair is present.
- Initialize still undefined properties using SourceFlags.
- Iterate over the property map.
- Forward property maps through the task hierarchy using established interfaces for planning.
