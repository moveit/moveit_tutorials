URDF and SRDF
======================

URDF
----
MoveIt starts with a URDF (Unified Robot Description Format), the native format for describing robots in ROS. In this tutorial, you will find resources for the URDF, important tips and also a list of MoveIt specific requirements.

URDF Resources
^^^^^^^^^^^^^^

* `URDF ROS Wiki Page <http://www.ros.org/wiki/urdf>`_ - The URDF ROS Wiki page is the source of most information about the URDF.
* `URDF Tutorials <http://www.ros.org/wiki/urdf/Tutorials>`_ - Tutorials for working with the URDF.
* `SOLIDWORKS URDF Plugin <http://www.ros.org/wiki/sw_urdf_exporter>`_ - A plugin that lets you generate a URDF directly from a SOLIDWORKS model.

Important Tips
^^^^^^^^^^^^^^
This section contains a set of tips on making sure that the URDF that you generate can be used with MoveIt. Make sure you go through all these tips before starting to use MoveIt with your robot.

Special Characters in Joint Names
"""""""""""""""""""""""""""""""""
Joint names should not contain any of the following special characters: -,[,],(,),

We hope to be able to get rid of these restrictions on the joint names soon.

Safety Limits
"""""""""""""
Some URDFs have safety limits set in addition to the joint limits of the robot. Here's an example of the safety controller specified for the Panda head pan joint: ::

   <safety_controller k_position="100" k_velocity="1.5" soft_lower_limit="-2.857" soft_upper_limit="2.857"/>

The "soft_lower_limit" field and the "soft_upper_limit" field specify the joint position limits for this joint. MoveIt will compare these limits to the hard limits for the joint specified in the URDF and choose the limits that are more conservative.

.. note:: If the "soft_lower_limit" and the "soft_upper_limit" in the safety_controller are set to 0.0, your joint will be unable to move. MoveIt relies on you to specify the correct robot model.

Collision Checking
""""""""""""""""""
MoveIt uses the meshes specified in the URDF for collision checking. The URDF allows you to specify two sets of meshes separately for visualization and collision checking. In general, the visualization meshes can be detailed and pretty, but the collision meshes should be much less detailed. The number of triangles in a mesh affects the amount of time it takes to collision check a robot link. The number of triangles in the whole robot should be on the order of a few thousand.

Test your URDF
""""""""""""""
It is very important to test your URDF out and make sure things are ok. The ROS URDF packages provide a check_urdf tool. To verify your URDF using the check_urdf tool, follow the instructions `here <http://wiki.ros.org/urdf#Verification>`_.

URDF Examples
^^^^^^^^^^^^^
There are lots of URDFs available for robots using ROS.

* `URDF Examples <http://www.ros.org/wiki/urdf/Examples>`_ - A list of URDFs from the ROS community.


SRDF
----

The SRDF or Semantic Robot Description Format complement the URDF and specifies joint groups, default robot configurations, additional collision checking information, and additional transforms that may be needed to completely specify the robot's pose. The recommended way to generate a SRDF is using the MoveIt Setup Assistant.

Virtual Joints
^^^^^^^^^^^^^^
The URDF contains information only about the physical joints on the robot. Often, additional joints need to be defined to specify the pose of the root link on the robot with respect to a world coordinate system. In such cases, a virtual joint is used to specify this connection. E.g., a mobile robot like the PR2 that moves around in the plane is specified using a planar virtual joint that attaches the world coordinate frame to the frame of the robot. A fixed robot (like an industrial manipulator) should be attached to the world using a fixed joint.

Passive Joints
^^^^^^^^^^^^^^
Passive joints are unactuated joints on a robot, e.g. passive casters on a differential drive robots. They are specified separately in the SRDF to make sure that different components in the motion planning or control pipelines know that the joints cannot be directly controlled. If your robot has unactuated casters, they should be specified as passive casters.

Groups
^^^^^^
A 'Group' (sometimes called 'JointGroup' or 'Planning Group') is a central concept in MoveIt. MoveIt always acts on a particular group. MoveIt will only consider moving the joints in the group that it is planning for -- other joints are left stationary. (A motion plan where all joints in the robot may move can be achieved by creating a group containing all joints.) A group is simply a collection of joints and links. Each group can be specified in one of several different ways:

Collection of Joints
""""""""""""""""""""
A group can be specified as a collection of joints. All the child links of each joint are automatically included in the group.

Collection of Links
"""""""""""""""""""
A group can also be specified as a collection of links. All the parent joints of the links are also included in the group.

Serial Chain
""""""""""""
A serial chain is specified using the base link and the tip link. The tip link in a chain is the child link of the last joint in the chain. The base link in a chain is the parent link for the first joint in the chain.

Collection of Sub-Groups
""""""""""""""""""""""""
A group can also be a collection of groups. E.g., you can define left_arm and right_arm as two groups and then define a new group called both_arms that includes these two groups.

End-Effectors
^^^^^^^^^^^^^
Certain groups in a robot can be given a special designation as an end-effector. An end-effector is typically connected to another group (like an arm) through a fixed joint. Note that when specifying groups that are end-effectors, it's important to make sure that there are no common links between the end-effector and the parent group it is connected to.

Self-Collisions
^^^^^^^^^^^^^^^
The Default Self-Collision Matrix Generator (part of Setup Assistant) searches for pairs of links on the robot that can safely be disabled from collision checking, decreasing motion planning processing time. These pairs of links are disabled when they are always in collision, never in collision, in collision in the robot's default position or when the links are adjacent to each other on the kinematic chain. The sampling density specifies how many random robot positions to check for self collision. Higher densities require more computation time while lower densities have a higher possibility of disabling pairs that should not be disabled. The default value is 10,000 collision checks. Collision checking is done in parallel to decrease processing time.

Robot Poses
^^^^^^^^^^^
The SRDF can also store fixed configurations of the robot. A typical example of the SRDF in this case is in defining a HOME position for a manipulator. The configuration is stored with a string id, which can be used to recover the configuration later.

SRDF Documentation
^^^^^^^^^^^^^^^^^^
For information about the syntax for the SRDF, read more details on the `ROS SRDF Wiki page <http://www.ros.org/wiki/srdf>`_.
