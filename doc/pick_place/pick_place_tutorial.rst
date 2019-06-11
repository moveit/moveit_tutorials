Pick and Place Tutorial
============================

In MoveIt, grasping is done using the MoveGroup interface. In order to grasp an object we need to create ``moveit_msgs::Grasp`` msg which will allow defining the various poses and postures involved in a grasping operation.
Watch this video to see the output of this tutorial:

.. raw:: html

    <div style="position: relative; padding-bottom: 5%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700px" height="400px" src="https://www.youtube.com/embed/QBJPxx_63Bs?rel=0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
    </div>

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

Running The Demo
----------------
Open two terminals. In the first terminal start RViz and wait for everything to finish loading: ::

    roslaunch panda_moveit_config demo.launch

In the second terminal run the pick and place tutorial: ::

    rosrun moveit_tutorials pick_place_tutorial

You should see something similar to the video at the beginning of this tutorial.

Understanding ``moveit_msgs::Grasp``
------------------------------------
For complete documentation refer to `moveit_msgs/Grasp.msg. <http://docs.ros.org/melodic/api/moveit_msgs/html/msg/Grasp.html>`_

The relevant fields of the message are:-

* ``trajectory_msgs/JointTrajectory pre_grasp_posture`` - This defines the trajectory position of the joints in the end effector group before we go in for the grasp.
* ``trajectory_msgs/JointTrajectory grasp_posture`` - This defines the trajectory position of the joints in the end effector group for grasping the object.
* ``geometry_msgs/PoseStamped grasp_pose`` - Pose of the end effector in which it should attempt grasping.
* ``moveit_msgs/GripperTranslation pre_grasp_approach`` - This is used to define the direction from which to approach the object and the distance to travel.
* ``moveit_msgs/GripperTranslation post_grasp_retreat`` - This is used to define the direction in which to move once the object is grasped and the distance to travel.
* ``moveit_msgs/GripperTranslation post_place_retreat`` - This is used to define the direction in which to move once the object is placed at some location and the distance to travel.

The Entire Code
---------------
The entire code can be seen :codedir:`here <pick_place>` in the moveit_tutorials GitHub project.

.. |br| raw:: html

   <br />

.. tutorial-formatter:: ./src/pick_place_tutorial.cpp
