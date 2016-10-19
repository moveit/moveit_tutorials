Planning Scene Tutorial
==================================

The :planning_scene:`PlanningScene` class provides the main interface that you will use
for collision checking and constraint checking. In this tutorial, we
will explore the C++ interface to this class.

.. tutorial-formatter:: ../planning_scene_tutorial.cpp

The entire code
^^^^^^^^^^^^^^^
The entire code can be seen :codedir:`here in the moveit_pr2 github project<planning>`.

Compiling the code
^^^^^^^^^^^^^^^^^^
Follow the `instructions for compiling code from source <http://moveit.ros.org/install/>`_.

The launch file
^^^^^^^^^^^^^^^
The entire launch file is `here <https://github.com/ros-planning/moveit_tutorials/tree/kinetic-devel/doc/pr2_tutorials/planning/launch/planning_scene_tutorial.launch>`_ on github. All the code in this
tutorial can be compiled and run from the moveit_tutorials package
that you have as part of your MoveIt! setup.

Running the code
^^^^^^^^^^^^^^^^

Roslaunch the launch file to run the code directly from moveit_tutorials::

 roslaunch moveit_tutorials planning_scene_tutorial.launch

Expected Output
^^^^^^^^^^^^^^^

The output should look something like this, though we are using random
joint values so some things may be different::

 [ INFO] [1385487628.853237681]: Test 1: Current state is not in self collision
 [ INFO] [1385487628.857680844]: Test 2: Current state is in self collision
 [ INFO] [1385487628.861798756]: Test 3: Current state is not in self collision
 [ INFO] [1385487628.861876838]: Current state is not valid
 [ INFO] [1385487628.866177315]: Test 4: Current state is in self collision
 [ INFO] [1385487628.866228020]: Contact between: l_shoulder_pan_link and r_forearm_link
 [ INFO] [1385487628.866259030]: Contact between: l_shoulder_pan_link and r_shoulder_lift_link
 [ INFO] [1385487628.866305963]: Contact between: l_shoulder_pan_link and r_shoulder_pan_link
 [ INFO] [1385487628.866331036]: Contact between: l_shoulder_pan_link and r_upper_arm_link
 [ INFO] [1385487628.866358135]: Contact between: l_shoulder_pan_link and r_upper_arm_roll_link
 [ INFO] [1385487628.870629418]: Test 5: Current state is not in self collision
 [ INFO] [1385487628.877406467]: Test 6: Current state is not in self collision
 [ INFO] [1385487628.879610797]: Test 7: Random state is not constrained
 [ INFO] [1385487628.880027331]: Test 8: Random state is not constrained
 [ INFO] [1385487628.880315077]: Test 9: Random state is not constrained
 [ INFO] [1385487628.880377445]: Test 10: Random state is feasible
 [ INFO] [1385487628.887157707]: Test 10: Random state is not valid
