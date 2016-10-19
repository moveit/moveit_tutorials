Move Group Interface Tutorial
==================================

In MoveIt!, the primary user interface is through the :move_group_interface:`MoveGroup` class. It provides easy to use functionality for most operations that a user may want to carry out, specifically setting joint or pose goals, creating motion plans, moving the robot, adding objects into the environment and attaching/detaching objects from the robot.

.. tutorial-formatter:: ../move_group_interface_tutorial.cpp

The entire code
^^^^^^^^^^^^^^^
The entire code can be seen :codedir:`here in the moveit_pr2 github project<planning>`.

Compiling the code
^^^^^^^^^^^^^^^^^^
Follow the `instructions for compiling code from source <http://moveit.ros.org/install/>`_.

The launch file
^^^^^^^^^^^^^^^
The entire launch file is `here <https://github.com/ros-planning/moveit_tutorials/tree/indigo-devel/doc/pr2_tutorials/planning/launch/move_group_interface_tutorial.launch>`_ on github. All the code in this tutorial can be compiled and run from the moveit_tutorials package
that you have as part of your MoveIt! setup.

Running the code
^^^^^^^^^^^^^^^^

Roslaunch the launch file to run the code directly from moveit_tutorials::

 roslaunch moveit_tutorials move_group_interface_tutorial.launch

After a short moment, the Rviz window should appear:

.. image:: move_group_interface_tutorial_start_screen.png

The *Motion Planning* section in the bottom right part of the window can be closed to get a better view of the robot.


Expected Output
^^^^^^^^^^^^^^^

In Rviz, we should be able to see the following (there will be a delay of 5-10 seconds between each step):

 1. The robot moves its right arm to the pose goal to its right front.
 2. The robot repeats the same motion from 1.
 3. The robot moves its right arm to the joint goal at its right side.
 4. The robot moves its right arm back to a new pose goal while maintaining the end-effector level.
 5. The robot moves its right arm along the desired cartesian path (a triangle up+forward, left, down+back).
 6. A box object is added into the environment to the right of the right arm.
    |B|

 7. The robot moves its right arm to the pose goal, avoiding collision with the box.
 8. The object is attached to the wrist (its color will change to purple/orange/green).
 9. The object is detached from the wrist (its color will change back to green).
 10. The object is removed from the environment.

.. |B| image:: ./move_group_interface_tutorial_robot_with_box.png
