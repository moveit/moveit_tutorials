Subframes
============================

Subframes are frames that are defined on `CollisionObjects <http://docs.ros.org/api/moveit_msgs/html/msg/CollisionObject.html>`_. 
They can be used to define points of interest on objects that you place in the scene, such as
the opening of a bottle, the the tip of a screwdriver, or the head of a screw.
They can be used for planning and to write robot instructions such as "*pick up the bottle, then 
move the opening under the spout of the tap*", or "*pick up the screwdriver, then place it above 
the head of the screw*". 

Writing code that focuses on the object that the robot manipulates is not only
more readable, but also more robust and portable between robots. This tutorial shows you how to 
define subframes on collision objects, publish them to the planning scene and use them to plan motions,
so you can do things like this:

.. image:: subframes_tutorial_cylinder_demo.gif
   :width: 450px
   :align: center

In this animation, the robot moves the tip of the cylinder to different positions on the box.

Running The Demo
----------------
After having completed the steps in `Getting Started <../getting_started/getting_started.html>`_, open two terminals. In the first terminal, execute this command to load up a panda, and wait for everything to finish loading: ::

    roslaunch panda_moveit_config demo.launch

In the second terminal run the tutorial: ::

    rosrun moveit_tutorials subframes_tutorial

In this terminal you should be able to enter numbers from 1-12 to send commands, and to see how the robot and the scene react.


The Code
---------------
The code for this example can be seen :codedir:`here <subframes>` in the moveit_tutorials GitHub project and is explained in detail below.

The code spawns a box and a cylinder in the planning scene, attaches the cylinder to the 
robot, and then lets you send motion commands via the command line. It also defines two 
convenience functions for sending a motion command, and for publishing the objects.

.. |br| raw:: html

   <br />

.. tutorial-formatter:: ./src/subframes_tutorial.cpp

Technical notes
---------------
Subframes are not known to TF, so they cannot be used outside of MoveIt planning requests. 
If you need the transformation to a subframe, you can obtain it from the ``PlanningScene``'s 
``CollisionRobot`` using the ``getFrameTransform`` function. This returns an ``Eigen::Isometry3d`` object, 
from which you can extract translation and quaternion (see `here <https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html>`).
The translation and quaternion can then be used to create the Transform and register it in your `TFListener`.

Visualizing subframes
---------------------
There is currently no visualization for subframes, but pull requests for this feature are welcome!

Troubleshooting
---------------
For older moveit_config packages that you have not generated yourself recently, the planning adapter
required for subframes might not be configured, and the subframe link might not be found. To fix this for your
moveit_config package, open the ``ompl_planning_pipeline.launch`` file in the ``<robot_moveit_config>/launch``
folder of your robot. For the Panda robot it is :panda_codedir:`this <launch/ompl_planning_pipeline.launch.xml>` file. 
Edit this launch file, find the lines where ``<arg name="planning_adapters">`` is mentioned and insert ``default_planner_request_adapters/ResolveConstraintFrames`` after
the line ``default_planner_request_adapters/FixStartStatePathConstraints``.

