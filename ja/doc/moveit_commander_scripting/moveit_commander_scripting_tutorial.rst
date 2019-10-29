MoveIt Commander Scripting
===========================
.. image:: moveit_commander_scripting.png
   :width: 700px

The `moveit_commander <http://wiki.ros.org/moveit_commander>`_ Python package offers wrappers for the functionality provided in MoveIt. Simple interfaces are available for motion planning, computation of Cartesian paths, and pick and place. The ``moveit_commander`` package also includes a command line interface, ``moveit_commander_cmdline.py``.

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

Starting RViz and the Command Line Tool
---------------------------------------
Open two shells. Start RViz and wait for everything to finish loading in the first shell: ::

  roslaunch panda_moveit_config demo.launch

Now initiate the ``moveit_commander`` interface in another shell: ::

 rosrun moveit_commander moveit_commander_cmdline.py

Using the MoveIt Commander Command Line Tool
---------------------------------------------
The command below will start a command line interface tool that allows you to connect to a running instance of the move_group node. The first command you should type is: ::

 use <group name>

This will connect to the move_group node for the group name you specified (in the Panda, for instance, you could connect to ``panda_arm``). You can now execute commands on that group.
This command, ``current``, will show you the current state of your group: ::

 current

To record that state under a specific name you can simply type: ::

 rec c

This will remember the current joint values of the robot group under the name ``c``. Matlab-like syntax is available for modifying joint values. The code above copies the joint values of c into a new variable named goal. We then modify the first joint of ``goal`` to ``0.2``. You may need to use a different value instead of ``0.2`` (it needs to be within your allowed bounds and not cause collisions). The ``go`` command plans a motion and executes it.

To get the robot to move, you could type, for example: ::

 goal = c
 goal[0] = 0.2
 go goal


Instead of calling ``go`` you could also type: ::

 goal[0] = 0.2
 goal[1] = 0.2
 plan goal
 execute

This is slightly inefficient, but the advantage is that the ``plan`` command allows you to visualize the computed motion plan in RViz before you actually issue the execute command.

For a list of supported commands, you can type ``help``. To exit the ``moveit_commander`` interface you can type ``quit``.
