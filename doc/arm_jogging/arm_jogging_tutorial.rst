Real-Time Arm Jogging
=====================

This tutorial shows how to send real-time jogging commands to a ROS-enabled robot. Some nice features of the jogger are singularity handling and collision checking that prevents operator from breaking the robot.

.. raw:: html

        <object width="480" height="385"><embed
        src="http://www.youtube.com/v/8sOucNloJeI&hl=en_US&fs=1&rel=0"
        type="application/x-shockwave-flash" allowscriptaccess="always"
        allowfullscreen="true" width="480"
        height="385"></embed></object>

Robot Requirements
------------------
The jogger streams an array of position or velocity commands to the robot controller. This is compatible with ros\_control ``position_controllers/JointGroupPositionControllers`` or ``velocity_controllers/JointGroupVelocityControllers``. You can check if these controllers are available for your robot by searching for the controller config file (typically named ``controllers.yaml``).

Jogging may work on other robots that have a different control scheme but there is no guarantee. It has been tested heavily on UR robots using the [ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver). The jogger currently does not limit joint jerk so may not be compatible with most heavy industrial robots.

The jogger can publish ``trajectory_msgs/JointTrajectory`` or ``std_msgs/Float64MultiArray`` message types. This is configured in a yaml file (see ``config/sia5_simulated_config.yaml`` for an example).

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

You can test the jogger with this `Gazebo simulator <https://github.com/UTNuclearRoboticsPublic/motoman_project>`_. Gazebo is necessary because it provides ros\_control controllers. Clone the repo into your catkin workspace. Install dependencies:

.. code-block:: bash

    sudo apt install ros-melodic-control* ros-melodic-gazebo-ros-control* ros-melodic-joint-state-controller ros-melodic-position-controllers ros-melodic-joint-trajectory-controller

Then build the workspace with ``catkin build`` and re-source your setup files (e.g. ``source ~/catkin_ws/devel/setup.bash``).

Launch the Gazebo simulation:

.. code-block:: bash

  roslaunch motoman_gazebo sia5_gazebo_nishida_lab.launch

  roslaunch motoman_moveit sia5_gazebo_nishida_lab_moveit_planning_execution.launch

Move the arm to a non-singular (non zero joint values) configuration then launch the jogger:

.. code-block:: bash

  roslaunch moveit_experimental jog_arm_server.launch

You can publish example jogging commands with:

.. code-block:: bash

	rostopic pub -r 100 /jog_arm_server/delta_jog_cmds geometry_msgs/TwistStamped "header: auto
	twist:
	  linear:
	    x: 0.0
	    y: -1.0
	    z: 1.0
	  angular:
	    x: 0.0
	    y: 0.0
	    z: 0.0"

Settings
--------
User-configurable settings of the jog node are well-documented in ``config/sia5_simulated_config.yaml``.

Configuring Control Devices (Gamepads, Joysticks, etc)
------------------------------------------------------
The ``jog_arm/config`` folder contains two examples of converting `SpaceNavigator <https://www.amazon.com/s/ref=nb_sb_noss_2?url=search-alias%3Daps&field-keywords=spacenavigator>`_ 3D mouse commands to jog commands. ``spacenav_teleop_tools.launch`` loads a config file then publishes commands to the jogger on the ``spacenav/joy topic``. It is easy to create your own config file for a particular joystick or gamepad. We welcome pull requests of config files for new controllers.

``spacenav_cpp.launch`` launches a C++ node that does the same thing but with less latency. We do not plan to accept C++ pull requests for more controller types because there is a lot of overhead involved in supporting them.


Integration Testing
-------------------
There is a Python integration test in ``test/integration``. Run it by:

.. code-block:: bash

  roscd moveit_experimental
  catkin run_tests --this
