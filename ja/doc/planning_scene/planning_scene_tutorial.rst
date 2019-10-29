Planning Scene
==================================

The :planning_scene:`PlanningScene` class provides the main interface that you will use
for collision checking and constraint checking. In this tutorial, we
will explore the C++ interface to this class.

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

The entire code
---------------
The entire code can be seen :codedir:`here in the MoveIt GitHub project<planning_scene>`.

.. tutorial-formatter:: ./src/planning_scene_tutorial.cpp

The launch file
---------------
The entire launch file is :codedir:`here <planning_scene/launch/planning_scene_tutorial.launch>` on GitHub. All the code in this tutorial can be compiled and run from the moveit_tutorials package.

Running the code
----------------
Roslaunch the launch file to run the code directly from moveit_tutorials: ::

 roslaunch moveit_tutorials planning_scene_tutorial.launch

Expected Output
---------------

The output should look something like this, though we are using random
joint values so some things may be different. ::

 ros.moveit_tutorials: Test 1: Current state is not in self collision
 ros.moveit_tutorials: Test 2: Current state is not in self collision
 ros.moveit_tutorials: Test 3: Current state is not in self collision
 ros.moveit_tutorials: Test 4: Current state is valid
 ros.moveit_tutorials: Test 5: Current state is in self collision
 ros.moveit_tutorials: Contact between: panda_leftfinger and panda_link1
 ros.moveit_tutorials: Contact between: panda_link1 and panda_rightfinger
 ros.moveit_tutorials: Test 6: Current state is not in self collision
 ros.moveit_tutorials: Test 7: Current state is not in self collision
 ros.moveit_tutorials: Test 8: Random state is not constrained
 ros.moveit_tutorials: Test 9: Random state is not constrained
 ros.moveit_tutorials: Test 10: Random state is not constrained
 ros.moveit_tutorials: Test 11: Random state is feasible
 ros.moveit_tutorials: Test 12: Random state is not valid

**Note:** Don't worry if your output has different ROS console format. You can customize your ROS console logger by following `this blog post <http://dav.ee/blog/notes/archives/898>`_.
