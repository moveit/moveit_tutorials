Gazebo Simulation Integration
=============================

`Gazebo <http://gazebosim.org/>`_ is the most popular robotics simulator in the ROS ecosystem, so is naturally a good fit to integrate with MoveIt.

The `MoveIt Setup Assistant <../setup_assistant/setup_assistant_tutorial.html>`_ helps setup your robot to work with Gazebo, but there are still additional steps required to successfully run MoveIt in Gazebo.

----------------------------
After MoveIt Setup Assistant
----------------------------
This tutorial assumes that the robot is set up with `MoveIt Setup Assistant <../setup_assistant/setup_assistant_tutorial.html>`_,
so it is crucial to follow that document first. To the best of our knowledge, official FRANKA EMIKA Panda repository doesn't particularly consider Gazebo simulation
such that the necessary components to properly simulate the robot in Gazebo are missing with respect to :code:`urdf` and :code:`xacro` files. This is a rare incident, since most other robots
have those components out of the box. *If you have a custom robot, which already works well in Gazebo, you can skip the steps until Step-6.* Fortunately, there is already a good solution offered in `the blog post <https://erdalpekel.de/?p=55>`_ to this problem. For the sake of completion though,
the procedure outlined in there will be repeated (with improvements) in here as well for preparing the robot for Gazebo simulation.

**Note that these steps assume that you have cloned the** `franka_ros repository <https://github.com/frankaemika/franka_ros>`_ **from the source**:

1. Fix the robot to the world coordinate system
2. Add damping to the joint specifications
3. Add inertia matrices and masses to the links
4. Add friction and colorize the links
5. Configure gazebo_ros_control, transmissions and actuators
6. Adjust auto-generated ros_controllers.yaml
7. Adjust auto-generated ros_controllers.launch


1. Fix the robot to the world coordinate system
-----------------------------------------------
Open the :code:`franka_description/robots/panda_arm_hand.urdf.xacro` file and change the line 5 with:

.. code-block:: xml

    <xacro:panda_arm xyz="0 0 0" rpy="0 0 0" connected_to="world"/>

It alone doesn't fix the problem, since now we need to provide a link with name :code:`world`. Add the following line to
:code:`franka_description/robots/panda_arm_hand.urdf.xacro`:

.. code-block:: xml

    <link name="world" />

at line 10 just before the macro calls. Additionally, we should rename the fixed joint to :code:`virtual_joint` to properly match
the SRDF specification created in previous tutorial. Open :code:`franka_description/robots/panda_arm.xacro` file and replace
:code:`${arm_id}_joint_${connected_to}` with :code:`virtual_joint` at line 5.

2. Add damping to the joint specifications
------------------------------------------
In this step, we need to add:

.. code-block:: xml

    <dynamics damping="1.0"/>

to every joint (except :code:`${arm_id}_joint8}` joint) in :code:`franka_description/robots/panda_arm.xacro` file.


3. Add inertia matrices and masses to the links
-----------------------------------------------
Inertia matrices are required for Gazebo physics engine to work properly. We don't require exact numbers for pedagogic reasons,
yet it is perfectly suitable to go after them with MeshLab as explained at `relevant Gazebo documentation page <http://gazebosim.org/tutorials?tut=inertia&cat=build_robot>`_
in great depth. Add following snippet to the links from :code:`${arm_id}_link0` to :code:`${arm_id}_link8` in :code:`franka_description/robots/panda_arm.xacro`:

.. code-block:: xml

    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <mass value="3.06" />
        <inertia ixx="0.3" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.3" />
    </inertial>


Similarly add following snippet to the :code:`${ns}_hand` link in :code:`franka_description/robots/hand.xacro` file:

.. code-block:: xml

    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.68"/>
        <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>

Finally add following inertia block to :code:`finger` links:

.. code-block:: xml

    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.01"/>
        <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>

As previously mentioned, these values come from the referred blog post. It is explicitly advised to have a look in there to grasp the matter in-depth.

4. Add friction and colorize the links
--------------------------------------
In order to have a nice illustration of the robot in Gazebo simulation we need to colorize the links.
Moreover friction forces are added in order to have realistic dynamics. You can ignore them at all or change their values to experiment with.
Since the focus is MoveIt in this tutorial, we will just use the values from the provided solution.

This step is a bit tedious to do manually, so the ultimate :code:`xacro` file is provided entirely in below:

.. code-block:: xml

    <?xml version='1.0' encoding='utf-8'?>
    <!-- panda.gazebo.xacro -->
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
        <xacro:macro name="panda_gazebo" params="arm_id">
            <xacro:macro name="arm_gazebo" params="link">
                <gazebo reference="${link}">
                    <material>Gazebo/White</material>
                    <mu1>0.2</mu1>
                    <mu2>0.2</mu2>
                </gazebo>
            </xacro:macro>
            <xacro:macro name="hand_gazebo" params="link">
                <gazebo reference="${link}">
                    <material>Gazebo/Grey</material>
                    <mu1>0.2</mu1>
                    <mu2>0.2</mu2>
                </gazebo>
            </xacro:macro>
            <xacro:arm_gazebo link="${arm_id}_link0"/>
            <xacro:arm_gazebo link="${arm_id}_link1"/>
            <xacro:arm_gazebo link="${arm_id}_link2"/>
            <xacro:arm_gazebo link="${arm_id}_link3"/>
            <xacro:arm_gazebo link="${arm_id}_link4"/>
            <xacro:arm_gazebo link="${arm_id}_link5"/>
            <xacro:arm_gazebo link="${arm_id}_link6"/>
            <xacro:hand_gazebo link="${arm_id}_link7"/>
            <xacro:hand_gazebo link="${arm_id}_link8"/>
            <xacro:hand_gazebo link="${arm_id}_hand"/>
            <xacro:hand_gazebo link="${arm_id}_rightfinger"/>
            <xacro:hand_gazebo link="${arm_id}_leftfinger"/>
        </xacro:macro>
    </robot>

The filename is specified as an inline comment, but let's be pedantic. It should be named as :code:`panda.gazebo.xacro` and placed next
to the other xacro files.

Then add the following block to the end of :code:`franka_description/robots/panda_arm_hand_urdf.xacro` file:

.. code-block:: xml

    <xacro:include filename="$(find franka_description)/robots/panda.gazebo.xacro"/>
    <xacro:panda_gazebo arm_id="panda"/>

5. Configure gazebo_ros_control, transmissions and actuators
------------------------------------------------------------

This is necessary for the robot to move in Gazebo. ROS Control is a highly capable robot-agnostic stack, providing interfaces
to control theoretically any type of robot. :code:`gazebo_ros_control` enables the ROS control to be used in Gazebo.
See `its document <http://gazebosim.org/tutorials/?tut=ros_control>`_ for full details.


Along with the transmissions and actuators, which are the crucial components for joints to be able to move in Gazebo,
the plugin specification will be handled in a new file, :code:`panda.control.xacro`. As before, I will provide the full content now:

.. code-block:: xml

    <?xml version="1.0"?>
    <!-- panda.control.xacro -->
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
        <xacro:macro name="panda_control" params="arm_id">
            <xacro:macro name="arm_control" params="transmission joint motor">
                <transmission name="${transmission}">
                    <type>transmission_interface/SimpleTransmission</type>
                    <joint name="${joint}">
                        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
                    </joint>
                    <actuator name="${motor}">
                        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
                        <mechanicalReduction>1</mechanicalReduction>
                    </actuator>
                </transmission>
            </xacro:macro>
            <xacro:arm_control transmission="${arm_id}_tran_1" joint="${arm_id}_joint1" motor="${arm_id}_motor_1"/>
            <xacro:arm_control transmission="${arm_id}_tran_2" joint="${arm_id}_joint2" motor="${arm_id}_motor_2"/>
            <xacro:arm_control transmission="${arm_id}_tran_3" joint="${arm_id}_joint3" motor="${arm_id}_motor_3"/>
            <xacro:arm_control transmission="${arm_id}_tran_4" joint="${arm_id}_joint4" motor="${arm_id}_motor_4"/>
            <xacro:arm_control transmission="${arm_id}_tran_5" joint="${arm_id}_joint5" motor="${arm_id}_motor_5"/>
            <xacro:arm_control transmission="${arm_id}_tran_6" joint="${arm_id}_joint6" motor="${arm_id}_motor_6"/>
            <xacro:arm_control transmission="${arm_id}_tran_7" joint="${arm_id}_joint7" motor="${arm_id}_motor_7"/>
            <xacro:arm_control transmission="${arm_id}_leftfinger" joint="${arm_id}_finger_joint1" motor="${arm_id}_finger_joint1"/>
            <xacro:arm_control transmission="${arm_id}_rightfinger" joint="${arm_id}_finger_joint2" motor="${arm_id}_finger_joint2"/>
            <gazebo>
                <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so"/>
            </gazebo>
        </xacro:macro>
    </robot>

Again, this file should be placed next to other xacro files in :code:`franka_description` package.
Similarly add the following line to the end of :code:`franka_description/robots/panda_arm_hand_urdf.xacro` file:

.. code-block:: xml

    <xacro:include filename="$(find franka_description)/robots/panda.control.xacro"/>
    <xacro:panda_control arm_id="panda"/>

6. Adjust auto-generated ros_controllers.yaml
---------------------------------------------

Thankfully the blog post used as the source for this tutorial provides perfectly tuned gains both for hand and arm controllers.
In addition to them, all the necessary control configurations can be grouped in auto-generated :code:`ros_controllers.yaml` file.
Just copy the following snippet and overwrite :code:`panda_moveit_config/config/ros_controllers.yaml` with it:

.. code-block:: xml

    # MoveIt-specific simulation settings
    moveit_sim_hw_interface:
        joint_model_group: controllers_initial_group_
        joint_model_group_pose: controllers_initial_pose_
    # Settings for ros_control control loop
    generic_hw_control_loop:
        loop_hz: 300
        cycle_time_error_threshold: 0.01
    # Settings for ros_control hardware interface
    hardware_interface:
        joints:
            - panda_joint1
            - panda_joint2
            - panda_joint3
            - panda_joint4
            - panda_joint5
            - panda_joint6
            - panda_joint7
            - panda_finger_joint1
        sim_control_mode: 1  # 0: position, 1: velocity
    # Publish all joint states
    # Creates the /joint_states topic necessary in ROS
    joint_state_controller:
        type: joint_state_controller/JointStateController
        publish_rate: 50
    panda_arm_controller:
        type: effort_controllers/JointTrajectoryController
        joints:
            - panda_joint1
            - panda_joint2
            - panda_joint3
            - panda_joint4
            - panda_joint5
            - panda_joint6
            - panda_joint7
        gains:
            panda_joint1: { p: 12000, d: 50, i: 0.0, i_clamp: 10000 }
            panda_joint2: { p: 30000, d: 100, i: 0.02, i_clamp: 10000 }
            panda_joint3: { p: 18000, d: 50, i: 0.01, i_clamp: 1 }
            panda_joint4: { p: 18000, d: 70, i: 0.01, i_clamp: 10000 }
            panda_joint5: { p: 12000, d: 70, i: 0.01, i_clamp: 1 }
            panda_joint6: { p: 7000, d: 50, i: 0.01, i_clamp: 1 }
            panda_joint7: { p: 2000, d: 20, i: 0.0, i_clamp: 1 }

        constraints:
            goal_time: 2.0
        state_publish_rate: 25

    panda_hand_controller:
        type: effort_controllers/JointTrajectoryController
        joints:
            - panda_finger_joint1
            - panda_finger_joint2

        gains:
            panda_finger_joint1: { p: 5, d: 3.0, i: 0, i_clamp: 1 }
            panda_finger_joint2: { p: 5, d: 1.0, i: 0, i_clamp: 1 }

        state_publish_rate: 25

    controller_list:
        - name: panda_arm_controller
          action_ns: follow_joint_trajectory
          type: FollowJointTrajectory
          default: true
          joints:
            - panda_joint1
            - panda_joint2
            - panda_joint3
            - panda_joint4
            - panda_joint5
            - panda_joint6
            - panda_joint7
        - name: panda_hand_controller
          action_ns: follow_joint_trajectory
          type: FollowJointTrajectory
          default: true
          joints:
            - panda_finger_joint1
            - panda_finger_joint2


7. Adjust auto-generated :code:`ros_controllers.launch` in the :code:`panda_moveit_config` package.
---------------------------------------------------------------------------------------------------

Fill the :code:`args` in line 9 with:

.. code-block:: xml

    joint_state_controller panda_hand_controller panda_arm_controller

-------------------------------

8. Change the way :code:`robot_description` is loaded to Parameter Server.
--------------------------------------------------------------------------
Open auto-generated :code:`gazebo.launch` in the :code:`panda_moveit_config` package. Find the line starting
with :code:`<param name="robot_description" textfile="$(arg urdf_path)" />` and replace it with:

.. code-block:: xml

    <param name="robot_description" command="$(find xacro)/xacro '$(find franka_description)/robots/panda_arm_hand.urdf.xacro'"/>


With this adjustment we are using :code:`xacro` executable that compiles :code:`xacro` files into URDF files.

-------------------------------

At last purely Gazebo way of using the panda robot is ready! In order to be able to control the robot via a simpler
GUI, install `rqt_joint_trajectory_controller <http://wiki.ros.org/rqt_joint_trajectory_controller>`_.

In terminal-1:

.. code-block:: xml

    roslaunch panda_moveit_config gazebo.launch

In terminal-2:

.. code-block:: xml

    rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller


.. figure:: pure-gazebo.gif
   :width: 700px

   Panda arm control in Gazebo simulation.

If you happen to find all these steps too tedious (you cannot be blamed for that), just clone `the franka_ros fork <https://github.com/tahsinkose/franka_ros>`_, that is created
particularly for this tutorial with the final versions of the files mentioned in the previous steps.
The changes made thus far in auto-generated :code:`panda_moveit_config` package are `in this repository <https://github.com/tahsinkose/panda_moveit_config>`_.
At the end, both repositories will have the updated and directly usable versions.

-------------------------------

Now it is time to integrate MoveIt to this work. Open :code:`panda_moveit_config/launch/demo_gazebo.launch` file
and replace line 61 with:

.. code-block:: xml

    <arg name="rviz_config" value="$(find panda_moveit_config)/launch/moveit.rviz"/>

This will allow us to use juicy Motion Planning display of MoveIt in Rviz. There is a final minor issue in `demo_gazebo.launch` file. Remove
line 31 from that file, which contains unused :code:`urdf_path` argument. After that, launch:

.. code-block:: xml

    roslaunch panda_moveit_config demo_gazebo.launch

.. figure:: moveit-gazebo.gif
   :width: 700px

   Panda arm controlled via MoveIt in Gazebo simulation.


We have successfully integrated MoveIt and Gazebo ultimately. MoveIt Setup Assistant already does
many work under the hood, but it still misses some parts to provide a proper Gazebo integration. After following
this tutorial you should be able to reproduce this locally for any robot. In case you don't want to be
bothered with all the details, `franka_ros <https://github.com/tahsinkose/franka_ros>`_ and `panda_moveit_config <https://github.com/tahsinkose/panda_moveit_config>`_
forks provide a ready-made work.
