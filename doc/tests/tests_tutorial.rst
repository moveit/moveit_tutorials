Integration/Unit Tests
======================

How to test changes to MoveIt on various robots, including unit and integration tests.

**Note:** *This is a stub tutorial, to be expanded upon in the future*

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

Integration Tests
-----------------

Python-based integration tests are available for testing higher-level move_group functionality in MoveIt. To run: ::

 rostest moveit_ros_planning_interface python_move_group.test


Running Unit Tests
------------------

To run unit tests locally on the entire MoveIt catkin workspace using catkin-tools: ::

  catkin run_tests -iv

To run a test for just 1 package, `cd` to the package's directory and run::

  catkin run_tests --no-deps --this -iv

To ignore most of the log/print output of the tests::

  catkin run_tests --no-status --summarize --no-deps --this

To run a single test, you can execute the compiled file, e.g. from your catkin workspace::

  ./devel/lib/moveit_core/test_planning_scene

Test Robots
-----------

**Panda**

From the package `panda_moveit_config <https://github.com/ros-planning/panda_moveit_config>`_: ::

  roslaunch panda_moveit_config demo.launch

**Fanuc M-10iA**

From the package `moveit_resources <https://github.com/ros-planning/moveit_resources>`_: ::

  roslaunch moveit_resources demo.launch


Writing Unit Tests
------------------

The file below explains how a test is structured, so you can write your own. The entire test file, with includes, can be seen :codedir:`here <tests>` in the moveit_tutorials GitHub project.

MoveIt uses Google Test as a testing framework.

.. |br| raw:: html

   <br />

.. tutorial-formatter:: ./test/tests.cpp


Code Coverage
-------------

Test coverage measures the lines of code that are executed while running the test suite.
To accumulate statistics and create a html coverage report, build the code without optimization
and run the special `_coverage` target::

  sudo apt install ros-noetic-code-coverage
  catkin config --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
  catkin build
  catkin build moveit_core -v --no-deps --catkin-make-args moveit_core_coverage

The output will print where the coverage report is located and it looks similar to the following image:

.. image:: code_coverage_example.png
    :width: 300px
    :align: center
    :alt: example code coverage output
