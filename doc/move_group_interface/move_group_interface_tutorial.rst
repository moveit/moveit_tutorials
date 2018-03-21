Move Group Interface Tutorial
==================================

In MoveIt!, the simplest user interface is through the :move_group_interface:`MoveGroup` class. It provides easy to use functionality for most operations that a user may want to carry out, specifically setting joint or pose goals, creating motion plans, moving the robot, adding objects into the environment and attaching/detaching objects from the robot. This interface communicates over ROS topics, services, and actions to the `MoveGroup Node <http://docs.ros.org/indigo/api/moveit_ros_move_group/html/annotated.html>`_.

.. image:: move_group_interface_tutorial_start_screen.png

Prerequisites
-------------
If you haven't already done so, make sure you've completed the steps in `Prerequisites
<../prerequisites/prerequisites.html>`_.

The entire code
---------------
The entire code can be seen :codedir:`here in the MoveIt! Github project<move_group_interface/src/move_group_interface_tutorial.cpp>`.

.. tutorial-formatter:: ./src/move_group_interface_tutorial.cpp

The launch file
---------------
The entire launch file is :codedir:`here<move_group_interface/launch/move_group_interface_tutorial.launch>` on github. All the code in this tutorial can be run from the moveit_tutorials package that you have as part of your MoveIt! setup.

Running the Code
----------------
Open two shells and make sure you have re-sourced the setup files in both of them::

  source ~/ws_moveit/devel/setup.bash

In the first shell start Rviz and wait for everything to finish loading::

  roslaunch panda_moveit_config demo.launch

In the second shell, run the launch file::

  roslaunch moveit_tutorials move_group_interface_tutorial.launch

After a short moment, the Rviz window should appear and look similar to the one at the top of this page. To progress through each demo step either press the **Next** button in the **RvizVisualToolsGui** pannel at the bottom of the screen or select **Key Tool** in the **Tools** pannel at the top of the screen and then press **N** on your keyboard while Rviz is focused.

Expected Output
---------------
Watch the `YouTube video demo <https://youtu.be/_5siHkFQPBQ>`_ for expected output. In Rviz, we should be able to see the following:

 1. The robot moves its arm to the pose goal to its front.
 2. The robot moves its arm to the joint goal at its side.
 3. The robot moves its arm back to a new pose goal while maintaining the end-effector level.
 4. The robot moves its arm along the desired cartesian path (a triangle up+forward, left, down+back).
 5. A box object is added into the environment to the right of the arm.
    |B|

 6. The robot moves its arm to the pose goal, avoiding collision with the box.
 7. The object is attached to the wrist (its color will change to purple/orange/green).
 8. The object is detached from the wrist (its color will change back to green).
 9. The object is removed from the environment.

.. |B| image:: ./move_group_interface_tutorial_robot_with_box.png
