Mesh Filter with UR5 and Kinect
===============================

MoveIt's mesh filter functionality removes your robot's geometry from a depth image! If your robot's arm is in your depth sensor's view, the pixels associated with the arm are subtracted from the depth image.

This is accomplished by giving the original depth image, the robot's transforms (``\tf``) and the robot's URDF as inputs.
The filter then publishes a modified depth image which does not contain the pixels that overlaps with the current robot state.

.. image:: MeshFilter.gif

Getting Started
---------------

* Follow the instructions for :moveit_website:`installing MoveIt<install>`
  first if you have not already done that.

* Install the franka_panda package for ROS `sudo apt-get install ros-$ROS_DISTRO-franka-ros`
* Install `Gazebo <http://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install>`_. You need to install ros-${ROS_DISTRO}-gazebo-plugins too.


Running the Demo
-----------------

Roslaunch the launch filis a depth image masked to remove pixels that overlaps with the robot state.e to run the code directly from moveit_tutorials: ::

 roslaunch moveit_tutorials start_panda.launch

 roslaunch moveit_tutorials start_mesh_filter.launch

The above command opens a Panda arm with Kinect sensor in Gazebo and Rviz

Topic ``/filtered/depth`` is a depth image masked to remove pixels that overlap with the robot state.

.. image:: Filtered_Depth.png

Topic ``/model/depth`` is a depth image masked to show only pixels that overlap with the robot state.

.. image:: Model_Depth.png

Check out the mesh filter code `here <https://github.com/moveit/moveit/blob/master/moveit_ros/perception/mesh_filter/src/depth_self_filter_nodelet.cpp>`_

References
----------
`Understanding ROS Nodelets <https://medium.com/@waleedmansoor/understanding-ros-nodelets-c43a11c8169e>`_
