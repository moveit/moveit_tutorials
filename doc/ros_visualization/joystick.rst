Joystick Control Teleoperation
==========================================

Run
---

Startup regular MoveIt! planning node with Rviz (for example demo.launch)

Make sure you have the dependencies installed::

    sudo apt-get install ros-indigo-joy

In the Motion Planning plugin of Rviz, enable "Allow External Comm." checkbox in the "Planning" tab. Enable the 'Query Goal State' robot display in the MoveIt! Motion Planning Plugins's 'Planning Request' section.

Now launch the joystick control launch file specific to your robot. If you are missing this file, first re-run the MoveIt! Setup Assistant using the latest version of the Setup Assistant::

    roslaunch YOURROBOT_moveit_config joystick_control.launch

The script defaults to using ``/dev/input/js0`` for your game controller port. To customize, you can also use, for example::

    roslaunch YOURROBOT_moveit_config joystick_control.launch dev:=/dev/input/js1

This script can read four types of joysticks:

1. XBox360 Controller via USB
2. PS3 Controller via USB
3. PS3 Controller via Bluetooth (Please use ps3joy package at `http://wiki.ros.org/ps3joy <http://wiki.ros.org/ps3joy>`_)
4. Arctic USB Wireless `Gamepad <https://www.arctic.ac/eu_en/usb-wireless-gamepad.html>`_

Joystick Command Mappings
-------------------------

=====================   ==================   ===================== ==================
Command                 PS3 Controller       Xbox Controller       Arctic Controller
=====================   ==================   ===================== ==================
+-x/y                   left analog stick    left analog stick     left analog stick
+-z                     L2/R2                LT/RT                 L2/R2
+-yaw                   L1/R1                LB/RB                 L1/R1
+-roll                  left/right           left/right            left/right
+-pitch                 up/down              up/down               up/down
change planning group   select/start         Y/A                   9/10
change end effector     triangle/cross       back/start            1/3
plan                    square               X                     4
execute                 circle               B                     2
=====================   ==================   ===================== ==================

Debugging
---------

Add "Pose" to rviz Displays and subscribe to ``/joy_pose`` in order to see the output from joystick.

Note that only planning groups that have IK solvers for all their End Effector parent groups will work.
