Mesh Filter with UR5 and Kinect Sensor
======================================

The mesh filter takes in a point cloud, tf of robot and a URDF and published a modified point cloud with the point cloud subtracted that overlaps with the current robot state.

Getting Started 
---------------
Clone the universal_robot repository into your workspace 
<https://github.com/ros-industrial/universal_robot>
We require UR urdf files from the above repository

Running the Demo
+++++++++++++++++
Roslaunch the launch file to run the code directly from moveit_tutorials: ::

 roslaunch moveit_tutorials mesh_filter.launch


How to add sensor to arm 
-------------------------
Include sensor plugin in ur5_sensor.gazebo and build on top of base urdf using links and joints in ur5_sensor.urdf.xacro
This tutorial adds a Kinect but any other sensor can be added (e.g Hokuyo Lidar)

What is in the mesh_filter launch file?
---------------------------------------
mesh_filter launch file contains the following commands :

 roslaunch moveit_tutorials test_gazebo.launch
 rosrun topic_tools relay /camera/depth/image_raw /depth
 rosrun topic_tools relay /camera/color/camera_info /camera_info
 rosrun nodelet nodelet manager __name:=nodelet_manager
 rosrun nodelet nodelet load mesh_filter/DepthSelfFiltering nodelet_manager

Mesh Filter reference code - <https://github.com/ros-planning/moveit/blob/master/moveit_ros/perception/mesh_filter/src/depth_self_filter_nodelet.cpp>

What is a nodelet?
-------------------
