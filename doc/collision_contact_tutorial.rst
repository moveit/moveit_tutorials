Collision Contact Tutorial
==========================

This section walks you through C++ code which allows you to see collision contact points between the robot, itself, and the world as you move and interact with the robot’s arm in Rviz.


Code
----

Classes
^^^^^^^

The code for this tutorial is mainly in the **InteractiveRobot** class which we will walk through below. The **InteractiveRobot** class maintains a RobotModel, a RobotState, and information about "the world" (in this case "the world" is a single yellow cube).

The **InteractiveRobot** class uses the **IMarker** class which maintains an interactive marker. This tutorial does not cover the implementation of the IMarker class (`imarker.cpp <https://github.com/ros-planning/moveit_tutorials/blob/a0f75a9df840ac3dfd0eec337cea104a4c111747/doc/pr2_tutorials/interactivity/src/imarker.cpp>`_), but most of the code is copied from the `basic_controls <http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers:%20Getting%20Started#basic_controls>`_ tutorial and you can read more there about interactive markers if you are interested.


Code Description
^^^^^^^^^^^^^^^^

We will walk through the code in the order that it is run in the program, starting with the **main()** function in **collision_contact_tutorial.cpp**: 
::

 int main(int argc, char **argv)
 {
 ros::init (argc, argv, "acorn_play");
 ros::NodeHandle nh;
 
 InteractiveRobot robot;

After the standard ROS initialization we create an instance of the **InteractiveRobot** class. 
Next we implement a planning scene: ::

 g_planning_scene = new planning_scene::PlanningScene();
 g_planning_scene->configure(robot.robotModel());

Among other things the **PlanningScene** maintains collision information for the robot and the world. 
We have to tell the **PlanningScene** about the world geometry: ::

 Eigen::Affine3d world_cube_pose;
 double world_cube_size;
 robot.getWorldGeometry(world_cube_pose, world_cube_size);
 g_world_cube_shape.reset(new shapes::Box(world_cube_size, world_cube_size, world_cube_size));
 g_planning_scene->getCollisionWorld()->addToObject("world_cube", g_world_cube_shape, world_cube_pose);

**getWorldGeometry()** gets the size and pose of the cube from the **InteractiveRobot** class. **g_world_cube_shape** is a shared pointer to a new box shape describing the cube. ::

 g_marker_array_publisher = new ros::Publisher(nh.advertise<visualization_msgs::MarkerArray>("interactive_robot_marray",100));
     
The **g_marker_array_publisher** is used to publish collision contact points for display in Rviz.
::

 robot.setUserCallback(userCallback);
 ros::spin();
   
Here we set the callback, which will be called when the interactive markers are manipulated, and then enter the ros::spin() infinite loop.
The rest of the main function is just cleanup: ::

 delete g_planning_scene;
 delete g_marker_array_publisher;;
   
 ros::shutdown(); 
 return 0;
 }

The interesting work all happens in the callback function: ::

 void userCallback(InteractiveRobot& robot)
 {
 Eigen::Affine3d world_cube_pose;
 double world_cube_size;
 robot.getWorldGeometry(world_cube_pose, world_cube_size);
 g_planning_scene->getCollisionWorld()->moveShapeInObject("world_cube", g_world_cube_shape, world_cube_pose);

Here we tell the **CollisionWorld** the new location of the yellow world cube.
Next we prepare to check for collisions: ::

 collision_detection::CollisionRequest c_req;
 collision_detection::CollisionResult c_res;
 c_req.group_name = robot.getGroupName();  // "right_arm"
 c_req.contacts = true;
 c_req.max_contacts = 100;
 c_req.max_contacts_per_pair = 5;
 c_req.verbose = false;

* group_name (set above to "right_arm") indicates which part of the robot to check for collisions. (Remove this line to check the entire robot instead of just the right arm.)
* We ask for up to 100 collision points (up to 5 from each pair of colliding links/objects).
* Verbose can be set to true to print extra debug info.

Then actually run the collision check: ::

 g_planning_scene->checkCollision(c_req, c_res, *robot.robotState());

This checks for collisions between the "right_arm" and the world, as well as between the "right_arm" and the rest of the robot. 
If a collision occurred (c_res.collision is true) then we display the collision points: ::

 if (c_res.collision)
 {
 ROS_INFO("COLLIDING contact_point_count=%d",(int)c_res.contact_count);
     
 if (c_res.contact_count > 0)
  {
 std_msgs::ColorRGBA color;
 color.r = 1.0;
 color.g = 0.0;
 color.b = 1.0;
 color.a = 0.5;
 visualization_msgs::MarkerArray markers;
 collision_detection::getCollisionMarkersFromContacts(markers,
                                                      "base_footprint",
                                                       c_res.contacts,
                                                       color,
                                                       ros::Duration(), // remain until deleted
                                                       0.01);           // radius

**getCollisionMarkersFromContacts()** is a helper function that adds the collision contact points into a MarkerArray message. If you want to use the contact points for something other than displaying them you can iterate through **c_res.contacts** which is a std::map of contact points. Look at the implementation of getCollisionMarkersFromContacts() in `collision_tools.cpp <https://github.com/ros-planning/moveit/blob/kinetic-devel/moveit_core/collision_detection/src/collision_tools.cpp>`_ for how.
And finally we publish the markers to Rviz: ::

 publishMarkers(markers);
 }
   
If no collision occurred we erase any collision contact point markers that we may have placed there last time the callback was called: 
::

 else
 {
 ROS_INFO("Not colliding");
     
 // delete the old collision point markers
 visualization_msgs::MarkerArray empty_marker_array;
 publishMarkers(empty_marker_array);
 }
   
The **publishMarkers()** function deletes any old markers and then adds new ones: ::

 void publishMarkers(visualization_msgs::MarkerArray& markers)
 {
 // delete old markers
 if (g_collision_points.markers.size())
 {
  for (int i=0; i<g_collision_points.markers.size(); i++)
 g_collision_points.markers[i].action = visualization_msgs::Marker::DELETE;

 g_marker_array_publisher->publish(g_collision_points);
 }
   
 // move new markers into g_collision_points
 std::swap(g_collision_points.markers, markers.markers);
   
 // draw new markers (if there are any)
  if (g_collision_points.markers.size())
 g_marker_array_publisher->publish(g_collision_points);
   }

The entire code
^^^^^^^^^^^^^^^

The entire code can be seen `here <https://github.com/ros-planning/moveit_tutorials/tree/kinetic-devel/doc/pr2_tutorials/interactivity>`_ in the moveit_tutorials Github project.

Running
-------

Launch file
^^^^^^^^^^^

A launch file is located here. It loads the URDF and SRDF parameters for the PR2 robot, launches Rviz, and runs the collision_contact_tutorial program described above. If moveit_tutorials is in your ROS_PACKAGE_PATH then launch it by typing: 
::

 roslaunch moveit_tutorials collision_contact_tutorial.launch
     
Rviz setup
^^^^^^^^^^

When Rviz starts up you will have to add some displays to see the objects your code is publishing. This is done in the "Displays" panel in rviz. 

* Under GlobalOptions set FixedFrame to /base_footprint.
* Cick Add and (under moveit_ros_visualization) add a RobotState display. 

  * Set the RobotState::RobotDescription to robot_description
  
  * Set the RobotState::RobotStateTopic to interactive_robot_state
  
  * Set the RobotState::RobotAlpha to 0.3 (to make the robot transparent and see the collision points)
  
* Click Add and (under Rviz) add a Marker display. 

  * Set the Marker::MarkerTopic to interactive_robot_markers
  
* Click Add and (under Rviz) add a InteractiveMarkers display. 

  * Set the Marker::UpdateTopic to interactive_robot_imarkers/update
  
* Click Add and (under Rviz) add a MarkerArray display. 

  * Set the Marker::UpdateTopic to interactive_robot_marray.

You should now see the PR2 robot with 2 interactive markers which you can drag around. 

.. image:: collision_contact_tutorial_screen.png

Interacting
^^^^^^^^^^^

In Rviz you will see two sets of Red/Green/Blue interactive marker arrows. Drag these around with the mouse. 
Move the right arm so it is in contact with the left arm. You will see magenta spheres marking the contact points. 
If you do not see the magenta spheres be sure that you added the MarkerArray display with interactive_robot_marray topic as described above. Also be sure to set RobotAlpha to 0.3 (or some other value less than 1) so the robot is transparent and the spheres can be seen. 
Move the right arm so it is in contact with the yellow cube (you may also move the yellow cube). You will see magenta spheres marking the contact points. 
