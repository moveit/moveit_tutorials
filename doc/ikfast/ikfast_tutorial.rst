IKFast Kinematics Solver
========================

.. image:: openrave_panda.png
   :width: 700px

In this section, we will walk through configuring an IKFast plugin for MoveIt

What is IKFast?
---------------

*From Wikipedia:*
IKFast, the Robot Kinematics Compiler, is a powerful inverse kinematics solver provided within Rosen Diankov's OpenRAVE motion planning software. Unlike most inverse kinematics solvers, IKFast can analytically solve the kinematics equations of any complex kinematics chain, and generate language-specific files (like C++) for later use. The end result is extremely stable solutions that can run as fast as 5 microseconds on recent processors

MoveIt IKFast
---------------

MoveIt IKFast is a tool that generates a IKFast kinematics plugin for MoveIt using OpenRAVE generated cpp files.
This tutorial will step you through setting up your robot to utilize the power of IKFast. MoveIt IKFast is tested on ROS Melodic with Catkin using OpenRAVE 0.8 with a 6DOF and 7DOF robot arm manipulator.
While it works in theory, currently the IKFast plugin generator tool does not work with >7 degree of freedom arms.

Getting Started
-----------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

You should have MoveIt configuration package for your robot that was created by using the `Setup Assistant <../setup_assistant/setup_assistant_tutorial.html>`_

Installing OpenRAVE on Ubuntu 16.04 is tricky. Here are 2 blog posts that give slightly different recipes for installing OpenRAVE.

 * `Stéphane Caron's Installing OpenRAVE on Ubuntu 16.04 <https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html>`_
 * `Francisco Suárez-Ruiz's Robotics Workstation Setup in Ubuntu 16.04 <https://fsuarez6.github.io/blog/workstation-setup-xenial>`_

Make sure you have these programs installed: ::

 sudo apt-get install cmake g++ git ipython minizip python-dev python-h5py python-numpy python-scipy qt4-dev-tools

You may also need the following libraries: ::

 sudo apt-get install libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev libbullet-dev libfaac-dev libglew-dev libgsm1-dev liblapack-dev liblog4cxx-dev libmpfr-dev libode-dev libogg-dev libpcrecpp0v5 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev libswscale-dev libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev

To enable the OpenRAVE viewer you may also need to install OpenSceneGraph-3.4 from source: ::

 sudo apt-get install libcairo2-dev libjasper-dev libpoppler-glib-dev libsdl2-dev libtiff5-dev libxrandr-dev
 git clone https://github.com/openscenegraph/OpenSceneGraph.git --branch OpenSceneGraph-3.4
 cd OpenSceneGraph
 mkdir build; cd build
 cmake .. -DDESIRED_QT_VERSION=4
 make -j$(nproc)
 sudo make install

For IkFast to work correctly, you *must* have the correct version of sympy installed: ::

 pip install --upgrade --user sympy==0.7.1

You should *not* have mpmath installed: ::

 sudo apt remove python-mpmath

MoveIt IKFast Installation
---------------------------
Install the MoveIt IKFast package either from debs or from source.

**Binary Install**: ::

 sudo apt-get install ros-melodic-moveit-kinematics

**Source**

Inside your catkin workspace: ::

 git clone https://github.com/ros-planning/moveit.git

OpenRAVE Installation
----------------------

**Binary Install (only Indigo / Ubuntu 14.04)**: ::

 sudo apt-get install ros-indigo-openrave

Note: you have to set: ::

 export PYTHONPATH=$PYTHONPATH:`openrave-config --python-dir`

**Source Install**: ::

 git clone --branch latest_stable https://github.com/rdiankov/openrave.git
 cd openrave && mkdir build && cd build
 cmake -DODE_USE_MULTITHREAD=ON -DOSG_DIR=/usr/local/lib64/ ..
 make -j$(nproc)
 sudo make install

Working commit numbers 5cfc7444... confirmed for Ubuntu 14.04 and 9c79ea26... confirmed for Ubuntu 16.04, according to Stéphane Caron.

**Please report your results with this on** `this GitHub repository. <https://github.com/ros-planning/moveit_tutorials>`_


Create Collada File For Use With OpenRAVE
-----------------------------------------

Parameters
^^^^^^^^^^

 * *MYROBOT_NAME* - name of robot as in your URDF
 * *PLANNING_GROUP* - name of the planning group you would like to use this solver for, as referenced in your SRDF and kinematics.yaml
 * *MOVEIT_IK_PLUGIN_PKG* - name of the new package you just created
 * *IKFAST_OUTPUT_PATH* - file path to the location of your generated IKFast output.cpp file

To make using this tutorial copy/paste friendly, set a MYROBOT_NAME environment variable with the name of your robot: ::

 export MYROBOT_NAME="panda_arm"

First you will need robot description file that is in `Collada or OpenRAVE <http://openrave.org/docs/latest_stable/collada_robot_extensions/>`_ robot format.

If your robot is not in this format we recommend you create a ROS `URDF <http://www.ros.org/wiki/urdf/Tutorials/Create%20your%20own%20urdf%20file>`_ file.

If your robot is in `xacro <http://wiki.ros.org/xacro/>`_ format you can convert it to urdf using the following command: ::

 rosrun xacro xacro --inorder -o "$MYROBOT_NAME".urdf "$MYROBOT_NAME".urdf.xacro

Once you have your robot in URDF format, you can convert it to Collada (.dae) file using the following command: ::

 rosrun collada_urdf urdf_to_collada "$MYROBOT_NAME".urdf "$MYROBOT_NAME".dae

Often floating point issues arise in converting a URDF file to Collada file, so a script has been created to round all the numbers down to x decimal places in your .dae file. Its probably best if you skip this step initially and see if IKFast can generate a solution with your default values, but if the generator takes longer than, say, an hour, try the following: ::

    export IKFAST_PRECISION="5"
    cp "$MYROBOT_NAME".dae "$MYROBOT_NAME".backup.dae  # create a backup of your full precision dae.
    rosrun moveit_kinematics round_collada_numbers.py "$MYROBOT_NAME".dae "$MYROBOT_NAME".dae "$IKFAST_PRECISION"

From experience we recommend 5 decimal places, but if the OpenRAVE IKFast generator takes to long to find a solution, lowering the number of decimal places should help.

To see the links in your newly generated Collada file

You may need to install package **libsoqt4-dev** to have the display working: ::

 openrave-robot.py "$MYROBOT_NAME".dae --info links

This is useful if you have a 7-dof arm and you need to fill in a --freeindex parameter, discussed later.

To test your newly generated Collada file in OpenRAVE: ::

 openrave "$MYROBOT_NAME".dae

You should see your robot.

.. image:: openrave_panda.png
   :width: 700px

Create IKFast Solution CPP File
-------------------------------
Once you have a numerically rounded Collada file its time to generate the C++ .h header file that contains the analytical IK solution for your robot.

Select IK Type
^^^^^^^^^^^^^^
You need to choose which sort of IK you want. See `this page <http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types>`_ for more info.  The most common IK type is *transform6d*.

Choose Planning Group
^^^^^^^^^^^^^^^^^^^^^
If your robot has more than one arm or "planning group" that you want to generate an IKFast solution for, choose one to generate first. The following instructions will assume you have chosen one <planning_group_name> that you will create a plugin for. Once you have verified that the plugin works, repeat the following instructions for any other planning groups you have. For example, you might have 2 planning groups: ::

 <planning_group_name> = "left_arm"
 <planning_group_name> = "right_arm"

To make it easy to use copy/paste for the rest of this tutorial. Set a PLANNING_GROUP environment variable. eg: ::

 export PLANNING_GROUP="panda_arm"

Identify Link Numbers
^^^^^^^^^^^^^^^^^^^^^
You also need the link index numbers for the *base_link* and *end_link* between which the IK will be calculated. You can count the number of links by viewing a list of links in your model: ::

 openrave-robot.py "$MYROBOT_NAME".dae --info links

A typical 6-DOF manipulator should have 6 arm links + a dummy base_link as required by ROS specifications.  If no extra links are present in the model, this gives: *baselink=0* and *eelink=6*.  Often, an additional tool_link will be provided to position the grasp/tool frame, giving *eelink=7*.

The manipulator below also has another dummy mounting_link, giving *baselink=1* and *eelink=8*.

=============  ======  ===========
name           index   parents
=============  ======  ===========
panda_link0    0
panda_link1    1       panda_link0
panda_link2    2       panda_link1
panda_link3    3       panda_link2
panda_link4    4       panda_link3
panda_link5    5       panda_link4
panda_link6    6       panda_link5
panda_link7    7       panda_link6
panda_link8    8       panda_link7
=============  ======  ===========

Set the base link and EEF link to the desired index::

 export BASE_LINK="0"
 export EEF_LINK="8"

If you have a 7 DOF arm you will need to specify a free joint. Selecting the correct free joint for a 7 DOF robot can have significant impact on performance of your kinematics plugin. We suggest experimenting with different choices for the free joint ::

 export FREE_INDEX="1"

Generate IK Solver
^^^^^^^^^^^^^^^^^^

To generate the IK solution between the manipulator's base and tool frames for a 6DOF arm, use the following command format. We recommend you name the output ikfast61\_"$PLANNING_GROUP".cpp: ::

 export IKFAST_OUTPUT_PATH=`pwd`/ikfast61_"$PLANNING_GROUP".cpp

For a 6DOF arm: ::

 python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot="$MYROBOT_NAME".dae --iktype=transform6d --baselink="$BASE_LINK" --eelink="$EEF_LINK" --savefile="$IKFAST_OUTPUT_PATH"

For a 7 dof arm, you will need to specify a free link: ::

 python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot="$MYROBOT_NAME".dae --iktype=transform6d --baselink="$BASE_LINK" --eelink="$EEF_LINK" --freeindex="$FREE_INDEX" --savefile="$IKFAST_OUTPUT_PATH"

The speed and success of this process will depend on the complexity of your robot. A typical 6 DOF manipulator with 3 intersecting axis at the base or wrist will take only a few minutes to generate the IK.

**Known issue**
--freeindex argument is known to have a bug that it cannot handle tree index correctly.
Say --baselink=2 --eelink=16 and links index from 3 to 9 is not related to current planning group chain. In that case --freeindex will expect index 2 as link 2, but index 3 as link 10 ... and index 9 as link 16.

You should consult the OpenRAVE mailing list and ROS Answers for information about 5 and 7 DOF manipulators.

Create Plugin
-------------

Create the package that will contain the IK plugin. We recommend you name the package "$MYROBOT_NAME"_ikfast_"$PLANNING_GROUP"_plugin.: ::

 export MOVEIT_IK_PLUGIN_PKG="$MYROBOT_NAME"_ikfast_"$PLANNING_GROUP"_plugin
 cd ~/catkin_ws/src
 catkin_create_pkg "$MOVEIT_IK_PLUGIN_PKG"

Build your workspace so the new package is detected (can be 'roscd'): ::

 catkin build

Create the plugin source code: ::

 rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" "$IKFAST_OUTPUT_PATH"

Or without ROS: ::

 python /path/to/create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" "$IKFAST_OUTPUT_PATH"

Usage
-----
The IKFast plugin should function identically to the default KDL IK Solver, but with greatly increased performance. The MoveIt configuration file is automatically edited by the moveit_ikfast script but you can switch between the KDL and IKFast solvers using the *kinematics_solver* parameter in the robot's kinematics.yaml file: ::

 rosed "$MYROBOT_NAME"_moveit_config kinematics.yaml

Edit these parts: ::

 <planning_group>:
   kinematics_solver: <myrobot_name>_<planning_group>_kinematics/IKFastKinematicsPlugin
 -INSTEAD OF-
   kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin

Test the Plugin
^^^^^^^^^^^^^^^

Use the MoveIt RViz Motion Planning Plugin and use the interactive markers to see if correct IK Solutions are found.

Updating the Plugin
-------------------

If any future changes occur with MoveIt or IKFast, you might need to re-generate this plugin using our scripts. To allow you to easily do this, a bash script is automatically created in the root of your IKFast package, named *update_ikfast_plugin.sh*. This does the same thing you did manually earlier, but uses the IKFast solution header file that is copied into the ROS package.
