Grasp Generator Tutorial
============================

TODO(@ridhwanluthra): verify all links in entire tutorial 

MoveIt! by default gives the user the capability to automatically generate grasps for primitive shapes like cylinder and cuboids.
This means that once you have added a primitive object to the planning scene you can go ahead and call the pick pipeline which will generate possible grasps, score them and then pick that object.

TODO(@ridhwanluthra): Add a working video

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

Using this functionality requires an additional package to be installed. Just clone the |code_start| moveit_grasps\ |code_end| package and build. ::

  git clone https://github.com/ros-planning/moveit_grasps

Configuration
-------------

In order for MoveIt! to generate efficient grasps their are a number of parameters related to grasp that you need to set.
A sample config file for the `panda robot <https://github.com/Ridhwanluthra/moveit_grasps/blob/generalisation/config_robot/panda.yaml>`_ can be found in the |code_start| `moveit_grasps <https://github.com/ros-planning/moveit_grasps>`_\ |code_end| package. Look at comments with the default parameters to understand what all parameters need to be set. ::

  base_link: 'panda_link0'
  # =====================================================
  hand:
      planning_group: 'panda_arm'
      end_effector_name: 'hand'  #ee group name

      # actuated joints in end effector
      joints : ['panda_finger_joint1', 'panda_finger_joint2']

      # open position of the gripper
      pregrasp_posture : [0.04, 0.04]
      pregrasp_time_from_start : 4.0

      # close position of the gripper
      grasp_posture : [0.0, 0.0]
      grasp_time_from_start : 8.0

      # Distance from wrist joint to palm of end effector [x, y, z, r, p, y]
      # Rotation from wrist joint to std end effector orientation
      # z-axis pointing toward object to grasp
      # x-axis perp. to movement of grippers
      # y-axis parallel to movement of grippers
      grasp_pose_to_eef_transform :  [0, 0, 0.165, 0, 0, -1.5707963267948966]

      # length of grippers (distance from finger tip to inner palm)
      finger_to_palm_depth : 0.06

      # width of gripper fingers
      gripper_finger_width: 0.08
      
      # max object width that can fit between fingers
      max_grasp_width : 0.03

      # grasp resolution parameters (angle is in degrees)
      grasp_resolution : 0.05
      grasp_depth_resolution : 0.025
      grasp_min_depth : 0.03 #0.015
      angle_resolution : 20

      # grasp approach and retreat parameters
      # direction vector wrt end effector link
      approach_direction: [1, 0, 0]
      # this is in addition to the finger_to_palm_depth
      approach_distance_desired: 0.109
      # direction vector wrt end effector link
      retreat_direction: [0, 0, -1]
      # this is in addition to the finger_to_palm_depth
      retreat_distance_desired: 0.3
      # this is really just MIN LIFT DISTANCE
      lift_distance_desired: 0.015
      # minimum padding on each side of the object on approach
      grasp_padding_on_approach: 0.010

      # Distance between fingers, in meters
      max_finger_width: 0.08
      min_finger_width: 0

Running The Code
----------------

Open two terminals. In the first terminal start RViz and wait for everything to finish loading: ::

  roslaunch panda_moveit_config demo.launch

In the second terminal run the grasp generator demo: ::

  roslaunch moveit_tutorials grasp_generator_demo.launch

You should see something similar to the video at the beginning of this tutorial.

Relevant Code
-------------
The entire code can be seen :codedir:`here <grasp_generator>` in the moveit_tutorials GitHub project. The details regarding addition and handling of collision object have been omitted in this tutorial as they are well documented in the `pick and place tutorial. <http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html>`_

.. |br| raw:: html

   <br />

.. |code_start| raw:: html

   <code>

.. |code_end| raw:: html

   </code>

.. tutorial-formatter:: ./src/grasp_generator_demo.cpp
