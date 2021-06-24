Planning Scene Monitor
==================================

The :planning_scene_monitor:`PlanningSceneMonitor` is the recommended interface for maintaining an up-to-date planning scene. The relationship between :moveit_core:`RobotState`, :planning_scene_monitor:`CurrentStateMonitor`, :planning_scene:`PlanningScene`, :planning_scene_monitor:`PlanningSceneMonitor`, and :planning_interface:`PlanningSceneInterface` can be really confusing at first. This tutorial aims to make clear these key concepts.

RobotState
----------
The :moveit_core:`RobotState` is a snapshot of a robot. It contains the :moveit_core:`RobotModel` and a set of joint values.

CurrentStateMonitor
-------------------
The :planning_scene_monitor:`CurrentStateMonitor` (CSM) can be thought of as a ROS wrapper for the RobotState. It subscribes to a provided topic for :sensor_msgs:`JointState` messages that provide up-to-date sensor values for single degree of freedom actuators, such as revolute or prismatic joints, and updates its internal RobotState with those joint values. In addition to the single degree of freedom joints, a robot can have joints with multiple degrees of freedom such as floating and planar joints. To maintain up-to-date transform information for links and other frames attached with multiple-degree-of-freedom joints, the CSM stores a TF2 :tf2:`Buffer` that uses a TF2 :tf2:`TransformListener` to set their transforms in its internal data.

PlanningScene
-------------
The :planning_scene:`PlanningScene` is a snapshot of the world that includes both the RobotState and any number of collision objects. The Planning Scene can be used for collision checking as well as getting information about the environment.

PlanningSceneMonitor
--------------------
The :planning_scene_monitor:`PlanningSceneMonitor` wraps a PlanningScene with ROS interfaces for keeping the PlanningScene up to date. To access the PlanningSceneMonitor's underlying PlanningScene, use the provided :planning_scene_monitor:`LockedPlanningSceneRW` and :planning_scene_monitor:`LockedPlanningSceneRO` classes.

The PlanningSceneMonitor has the following objects, which have their own ROS interfaces for keeping sub-components of the planning scene up to date:

 * A :planning_scene_monitor:`CurrentStateMonitor` for tracking updates to the RobotState via a ``robot_state_subscriber_`` and a ``tf_buffer_``, as well as a planning scene subscriber for listening to planning scene diffs from other publishers.
 * An OccupancyMapMonitor for tracking updates to an OccupancyMap via ROS topics and services.

The PlanningSceneMonitor has the following subscribers:

 * ``collision_object_subscriber_`` - Listens to a provided topic for :moveit_msgs:`CollisionObject` messages that might add, remove, or modify collision objects in the planning scene and passes them into its monitored planning scene
 * ``planning_scene_world_subscriber_`` - Listens to a provided topic for :moveit_msgs:`PlanningSceneWorld` messages that may contain collision object information and/or octomap information. This is useful for keeping planning scene monitors in sync
 * ``attached_collision_object_subscriber_`` - Listens on a provided topic for :moveit_msgs:`AttachedCollisionObject` messages that determine the attaching/detaching of objects to links in the robot state.

The PlanningSceneMonitor has the following services:

 * ``get_scene_service_`` - Which is an optional service to get the full planning scene state.

The PlanningSceneMonitor is initialized with:

 * ``startSceneMonitor`` - Which starts the ``planning_scene_subscriber_``,
 * ``startWorldGeometryMonitor`` - Which starts the ``collision_object_subscriber_``, the ``planning_scene_world_subscriber_``, and the OccupancyMapMonitor,
 * ``startStateMonitor`` - Which starts the CurrentStateMonitor and the ``attached_collision_object_subscriber_``,
 * ``startPublishingPlanningScene`` - Which starts another thread for publishing the entire planning scene on a provided topic for other PlanningSceneMonitors to subscribe to, and
 * ``providePlanningSceneService`` - Which starts the ``get_scene_service_``.

PlanningSceneInterface
----------------------
The :planning_interface:`PlanningSceneInterface` is a useful class for publishing updates to a MoveGroup's :planning_scene_monitor:`PlanningSceneMonitor` through a C++ API without creating your own subscribers and service clients. It may not work without MoveGroup or MoveItCpp.
