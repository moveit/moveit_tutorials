Move Group Python Interface Tutorial
================================================

In MoveIt!, the primary user interface is through the RobotCommander class. It
provides functionality for most operations that a user may want to carry out,
specifically setting joint or pose goals, creating motion plans, moving the
robot, adding objects into the environment and attaching/detaching objects from
the robot.

.. tutorial-formatter:: ../move_group_python_interface_tutorial.py

The entire code
^^^^^^^^^^^^^^^
The entire code can be seen :codedir:`here in the moveit_pr2 github project<planning/scripts/move_group_python_interface_tutorial.py>`.

The launch file
^^^^^^^^^^^^^^^
The entire launch file is `here <https://github.com/ros-planning/moveit_tutorials/tree/kinetic-devel/doc/pr2_tutorials/planning/launch/move_group_python_interface_tutorial.launch>`_
on github. All the code in this tutorial can be run from the
moveit_tutorials package that you have as part of your MoveIt! setup.

Running the code
^^^^^^^^^^^^^^^^
Make sure your python file is executable::
 
 roscd moveit_tutorials/doc/pr2_tutorials/planning/scripts/
 chmod +x move_group_python_interface_tutorial.py

Launch the moveit! demo interface::

 roslaunch pr2_moveit_config demo.launch

Now run the python code directly using rosrun::

 rosrun moveit_tutorials move_group_python_interface_tutorial.py

Expected Output
^^^^^^^^^^^^^^^

In Rviz, we should be able to see the following (there will be a delay of 5-10 seconds between each step):

 1. The robot moves its left arm to the pose goal in front of it (plan1)
 2. The robot again moves its left arm to the same goal (plan1 again)
 3. The robot moves its left arm to the joint goal to the side,
 4. The robot moves its left arm along the desired cartesian path.
