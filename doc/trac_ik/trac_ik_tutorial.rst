TRAC-IK Kinematics Solver
=========================

`TRAC-IK <https://bitbucket.org/traclabs/trac_ik>`_ is an inverse kinematics solver developed by TRACLabs that combines two IK implementations via threading to achieve more reliable solutions than common available open source IK solvers. From their documentation:

  (TRAC-IK) provides an alternative Inverse Kinematics solver to the popular inverse Jacobian methods in KDL. Specifically, KDL's convergence algorithms are based on Newton's method, which does not work well in the presence of joint limits --- common for many robotic platforms. TRAC-IK concurrently runs two IK implementations. One is a simple extension to KDL's Newton-based convergence algorithm that detects and mitigates local minima due to joint limits by random jumps. The second is an SQP (Sequential Quadratic Programming) nonlinear optimization approach which uses quasi-Newton methods that better handle joint limits. By default, the IK search returns immediately when either of these algorithms converges to an answer. Secondary constraints of distance and manipulability are also provided in order to receive back the "best" IK solution.

The package `trac_ik_kinematics_plugin <https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_kinematics_plugin/>`_ provides a KinematicsBase MoveIt interface that can replace the default KDL solver. Currently mimic joints are *not* supported.

Install
-------

As of v1.5.1, **trac_ik** is part of the ROS Kinetic/Melodic binaries: ::

  sudo apt-get install ros-melodic-trac-ik-kinematics-plugin

Binaries for Noetic are not yet available. You can track the progress `with this ticket <https://bitbucket.org/traclabs/trac_ik/issues/67/ros-noetic-support>`_.

Usage
-----

- Install **trac_ik_kinematics_plugin** and **trac_ik_lib package** or add to your catkin workspace.
- Find the MoveIt `kinematics.yaml <../kinematics_configuration/kinematics_configuration_tutorial.html>`_ file created for your robot.
- Replace ``kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin`` (or similar) with ``kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin``
- Set parameters as desired:

  - **kinematics\_solver\_timeout** (timeout in seconds, e.g., 0.005) and **position\_only\_ik** **ARE** supported.
  - **solve\_type** can be Speed, Distance, Manipulation1, Manipulation2 (see trac\_ik\_lib documentation for details).  Default is Speed.
  - **kinematics\_solver\_attempts** parameter is unneeded: unlike KDL, TRAC-IK solver already restarts when it gets stuck
  - **kinematics\_solver\_search\_resolution** is not applicable here.
  - Note: The Cartesian error distance used to determine a valid solution is **1e-5**, as that is what is hard-coded into MoveIt's KDL plugin.
