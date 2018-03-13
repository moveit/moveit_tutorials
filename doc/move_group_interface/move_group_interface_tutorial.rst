Move Group Interface Tutorial
==================================

In MoveIt!, the simplest user interface is through the :move_group_interface:`MoveGroup` class. It provides easy to use functionality for most operations that a user may want to carry out, specifically setting joint or pose goals, creating motion plans, moving the robot, adding objects into the environment and attaching/detaching objects from the robot. This interface communicates over ROS topics, services, and actions to the `MoveGroup Node <http://docs.ros.org/indigo/api/moveit_ros_move_group/html/annotated.html>`_.

.. image:: move_group_interface_tutorial_start_screen.png

Watch the `YouTube video demo <https://youtu.be/4FSmZRQh37Q>`_


Prerequisites
^^^^^^^^^^^^^
If you haven't already done so, make sure you've completed the steps in `Prerequisites
<../prerequisites/prerequisites.html>`_.

Start Rviz and MoveGroup node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Make sure you have re-sourced the setup files::

  source ~/ws_moveit/devel/setup.bash

Start Rviz and wait for everything to finish loading::

  roslaunch panda_moveit_config demo.launch

Running the demo
^^^^^^^^^^^^^^^^

In a new terminal window, run the :codedir:`move_group_interface_tutorial.launch<move_group_interface/launch/move_group_interface_tutorial.launch>` roslaunch file::

  roslaunch moveit_tutorials move_group_interface_tutorial.launch

After a short moment, the Rviz window should appear and look similar to the one at the top of this page. Press the **Next** button at the bottom of the screen or press 'N' on your keyboard while Rviz is focused to progress through each demo step.

Expected Output
^^^^^^^^^^^^^^^

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

Explaining the Demo
^^^^^^^^^^^^^^^^^^^
The entire code is located in the moveit_tutorials github repo under the subfolder :codedir:`move_group_interface<move_group_interface>`. Next we step through the code piece by piece to explain its functionality.

.. tutorial-formatter:: ./src/move_group_interface_tutorial.cpp
