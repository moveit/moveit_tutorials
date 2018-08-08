Visualizing Collisions
======================
.. image:: collisions.png
   :width: 700px

This section walks you through C++ example code that allows you to visualize collision contact points between the robot, itself, and the world as you move and interact with the robot’s arm in RViz.

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

Running the Code
----------------
Roslaunch the launch file to run the code directly from moveit_tutorials: ::

 roslaunch moveit_tutorials visualizing_collisions_tutorial.launch

You should now see the Panda robot with 2 interactive markers which you can drag around.

.. image:: visualizing_collisions_tutorial_screen.png
   :width: 700px

Classes
-------
The code for this tutorial is mainly in the :code:`InteractiveRobot` class which we will walk through below. The :code:`InteractiveRobot` class maintains a :code:`RobotModel`, a :code:`RobotState`, and information about 'the world' (in this case "the world" is a single yellow cube).

The :code:`InteractiveRobot` class uses the :code:`IMarker` class which maintains an interactive marker. This tutorial does not cover the implementation of the IMarker class (:codedir:`imarker.cpp <interactivity/src/imarker.cpp>`), but most of the code is copied from the `basic_controls <http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers:%20Getting%20Started#basic_controls>`_ tutorial and you can read more there about interactive markers if you are interested.

Interacting
-----------
In RViz you will see two sets of Red/Green/Blue interactive marker arrows. Drag these around with the mouse.
Move the right arm so it is in contact with the left arm. You will see magenta spheres marking the contact points.
If you do not see the magenta spheres be sure that you added the MarkerArray display with interactive_robot_marray topic as described above. Also be sure to set RobotAlpha to 0.3 (or some other value less than 1) so the robot is transparent and the spheres can be seen.
Move the right arm so it is in contact with the yellow cube (you may also move the yellow cube). You will see magenta spheres marking the contact points.

Relevant Code
-------------
The entire code can be seen :codedir:`here <visualizing_collisions>` in the moveit_tutorials GitHub project. Libraries used can be found :codedir:`here <interactivity>`. A lot of information necessary for understanding how this demo works is left out to keep this tutorial focused on collision contacts. To understand this demo fully, it is highly recommended that you read through the source code.

.. tutorial-formatter:: ./src/visualizing_collisions_tutorial.cpp

Launch file
-----------
The entire launch file is  :codedir:`here <visualizing_collisions>` on GitHub. All the code in this tutorial can be compiled and run from the moveit_tutorials package.
