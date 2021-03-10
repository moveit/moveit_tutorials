Custom Constraint Samplers
==========================

.. note:: This tutorial is a stub. If you have the time to help us improve it, please do!

Overview
--------
Some planning problems require more complex or custom constraint
samplers for more difficult planning problems. This document explains
how to create a custom motion planning constraint sampler for use
with MoveIt.

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

Creating a constraint sampler
-----------------------------

* Create a ``ROBOT_moveit_plugins`` package and within that a sub-folder for your ``ROBOT_constraint_sampler`` plugin. Modify the template provided by ``ROBOT_moveit_plugins/ROBOT_moveit_constraint_sampler_plugin``
* In your ``ROBOT_moveit_config/launch/move_group.launch`` file, within the ``<node name="move_group">``, add the parameter: ::

  <param name="constraint_samplers" value="ROBOT_moveit_constraint_sampler/ROBOTConstraintSamplerAllocator"/>

* Now when you launch move_group, it should default to your new constraint sampler.
