MoveItCpp Tutorial
==================================

Introduction
------------
MoveItCpp is a new high level interface, a unified C++ API that does not require the use of ROS Actions, Services, and Messages to access the core MoveIt functionality, and an alternative (not a full replacement) for the existing `MoveGroup API <../move_group_interface/move_group_interface_tutorial.html>`_, we recommend this interface for advanced users needing more realtime control or for industry applications. This interface has been developed at PickNik Robotics by necessity for our many commercial applications.

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

Running the Code
----------------

Open a shell, run the launch file: ::

  roslaunch moveit_tutorials moveit_cpp_tutorial.launch

**Note:** This tutorial uses the **RvizVisualToolsGui** panel to step through the demo. To add this panel to RViz, follow the instructions in the `Visualization Tutorial <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html#rviz-visual-tools>`_.

After a short moment, the RViz window should appear and look similar to the one at the top of this page. To progress through each demo step either press the **Next** button in the **RvizVisualToolsGui** panel at the bottom of the screen or select **Key Tool** in the **Tools** panel at the top of the screen and then press **N** on your keyboard while RViz is focused.

The Entire Code
---------------
The entire code can be seen :codedir:`here in the MoveIt GitHub project<moveit_cpp/src/moveit_cpp_tutorial.cpp >`. Next we step through the code piece by piece to explain its functionality.

.. tutorial-formatter:: ./src/moveit_cpp_tutorial.cpp

The Launch File
---------------
The entire launch file is :codedir:`here<moveit_cpp/launch/moveit_cpp_tutorial.launch>` on GitHub. All the code in this tutorial can be run from the **moveit_tutorials** package that you have as part of your MoveIt setup.