Planning Scene Python
==================================

The :moveit_python:`core.planning_scene.PlanningScene` class provides the main interface that you will use
for collision checking and constraint checking. In this tutorial, we
will explore the Python interface to this class.

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

The entire code
---------------
The entire code can be seen :codedir:`here in the MoveIt GitHub project<planning_scene>`.

.. tutorial-formatter:: ./src/planning_scene_tutorial.py

Running the code
----------------
Open two shells. Start RViz and wait for everything to finish loading in the first shell. You need to first clone `panda_moveit_config <http://github.com/ros-planning/panda_moveit_config>`_ in your catkin workspace and build: ::

  roslaunch panda_moveit_config demo.launch

Roslaunch the launch file to run the code directly from moveit_tutorials: ::

 rosrun moveit_tutorials planning_scene_tutorial.py

Expected Output
---------------

The output should look something like this, though we are using random
joint values so some things may be different. ::

 [ INFO] [1615403438.643254533]: Loading robot model 'panda'...
 [INFO] [1615403438.700939]: Test 1: Current state is in self collision
 [INFO] [1615403438.704516]: Contact between panda_hand and panda_link5
 [INFO] [1615403438.706369]: Contact between panda_link5 and panda_link7
 [INFO] [1615403459.455112]: Test 2: Current state is in self collision
 [INFO] [1615403459.461933]: Test 3: Current state is not in self collision
 [INFO] [1615403459.470156]: Test 4: Current state is valid
 [INFO] [1615403459.474089]: Test 6: Current state is not in self collision
 [INFO] [1615403459.479456]: Test 7: Current state is not in self collision
 [INFO] [1615403459.488881]: Test 8: Random state is not constrained
 [INFO] [1615403459.496180]: Test 9: Random state is not constrained
 [INFO] [1615403459.498652]: Test 10: Random state is not constrained
 [INFO] [1615403459.503490]: Test 12: Random state is not valid

**Note:** Don't worry if your output has different ROS console format. You can customize your ROS console logger by following `this blog post <http://dav.ee/blog/notes/archives/898>`_.