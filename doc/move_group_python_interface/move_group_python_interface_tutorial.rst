Move Group Python Interface
================================================

In MoveIt!, the simplest user interface is through the RobotCommander class. It
provides functionality for most operations that a user may want to carry out,
specifically setting joint or pose goals, creating motion plans, moving the
robot, adding objects into the environment and attaching/detaching objects from
the robot.

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

The entire code
---------------
The entire code can be seen :codedir:`here in the MoveIt! GitHub project<move_group_python_interface/scripts/move_group_python_interface_tutorial.py>`.

.. tutorial-formatter:: ./scripts/move_group_python_interface_tutorial.py

The launch file
---------------
The entire launch file is :codedir:`here<move_group_python_interface/launch/move_group_python_interface_tutorial.launch>`
on GitHub. All the code in this tutorial can be run from the
moveit_tutorials package that you have as part of your MoveIt! setup.

Running the code
----------------
Make sure your python file is executable: ::

 roscd moveit_tutorials/doc/move_group_python_interface/scripts/
 chmod +x move_group_python_interface_tutorial.py

Start RViz and MoveGroup node
-----------------------------
Open two shells and make sure you have re-sourced the setup files in both shells: ::

  source ~/ws_moveit/devel/setup.bash

Start RViz and wait for everything to finish loading in the first shell: ::

  roslaunch panda_moveit_config demo.launch

Now run the python code directly in the other shell using rosrun: ::

 rosrun moveit_tutorials move_group_python_interface_tutorial.py

Expected Output
---------------

Watch the `YouTube video demo <https://youtu.be/8JWexAhqfsA>`_ for expected output. In RViz, we should be able to see the following:

Press *<enter>* in the shell where you ran the ``rosrun`` command between each step
 1. The robot plans and moves its arm to the joint goal.
 2. The robot plans a path to a pose goal.
 3. The robot displays the plan to the same goal again.
 4. The robot moves its arm to the joint goal to the side.
 5. The robot moves its arm along the desired Cartesian path.
