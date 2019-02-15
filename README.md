# MoveIt! Tutorials

[https://ros-planning.github.io/moveit_tutorials/](https://ros-planning.github.io/moveit_tutorials/)

This is the primary documentation for the MoveIt! project. We strongly encourage you to help improve MoveIt!'s documentation. Please consider reading the guidelines below for writing the best documentation and tutorials. However, if you are uncomfortable with any of the approaches, simply adding documentation text to your pull requests is better than nothing.

These tutorials use the [reStructuredText](http://www.sphinx-doc.org/en/stable/rest.html) format commonly used in the Sphinx "Python Documentation Generator". This unfortunately differs from the common Markdown format, but its advantage is that it supports embedding code directly from source files for inline code tutorials.

This repository is currently built automatically by two systems. Travis builds the documentation for Melodic and ROS Build Farm builds the documentation for Kinetic and other stable versions:
- [![Travis Status](https://travis-ci.org/ros-planning/moveit_tutorials.svg?branch=master)](https://travis-ci.org/ros-planning/moveit_tutorials) [Github Pages + Travis](https://ros-planning.github.io/moveit_tutorials/): latest
- ROS Melodic Build Farm: TODO, unreleased
- [![ROS Kinetic Build Farm Status](http://build.ros.org/buildStatus/icon?job=Kdoc__moveit_tutorials__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdoc__moveit_tutorials__ubuntu_xenial_amd64/) [ROS Kinetic Build Farm](http://docs.ros.org/kinetic/api/moveit_tutorials/html/)
- [![ROS Indigo Build Status](http://build.ros.org/buildStatus/icon?job=Idoc__moveit_tutorials__ubuntu_trusty_amd64)](http://build.ros.org/job/Idoc__moveit_tutorials__ubuntu_trusty_amd64/) [ROS Indigo Build Farm](http://docs.ros.org/indigo/api/moveit_tutorials/html/)

## Versions

- ``indigo-devel`` usage is discouraged
- ``kinetic-devel`` stable
- ``melodic-devel`` latest, changes should target this branch

## Build Locally

If you want to test the tutorials by generating the html pages locally on your machine, run in the root of the package:

    ./build_locally.sh

Then open ``LOCAL_PACKAGE_PATH/build/html/index.html`` in your web browser.

## ROS Build Farm Deployment

For deploying documentation changes to the web, [Section 3 of rosdoc_lite wiki](http://wiki.ros.org/rosdoc_lite) says that "rosdoc_lite is automatically run for packages in repositories that have rosinstall files listed in the rosdistro repository." This is done about once every 24 hours, [overnight](http://wiki.ros.org/rosdistro/Tutorials/Indexing%20Your%20ROS%20Repository%20for%20Documentation%20Generation).

## Contributing

We rely on the community to keep these tutorials up to date and bug free. If you find an issue with the tutorials please [open an issue on GitHub](https://github.com/ros-planning/moveit_tutorials/issues/new) or open a PR with proposed changes.

### Formatting and Style

**Code Formatting**

* These tutorials use the same [style guidelines](http://moveit.ros.org/documentation/contributing/code/) as the MoveIt! project. When modifying or adding to these tutorials, it is required that code is auto formatted using [clang-format](http://moveit.ros.org/documentation/contributing/code/).
* Tutorials should exemplify best coding practices. If a contribution wouldn't pass review in the MoveIt! project, then it shouldn't pass review in the tutorials.
* Relevant code should be included and explained using the ``.. tutorial-formatter::`` tag.
* Irrelevant code should be excluded from the generated html using the ``BEGIN_TUTORIAL``, ``END_TUTORIAL``, ``BEGIN_SUB_TUTORIAL``, and ``END_SUB_TUTORIAL`` tags.
* Whenever possible, links should be created using the ``extlinks`` dictionary defined in ``conf.py``.
* All demo code should be runnable from within the ``moveit_tutorials`` package.
* Python code should be run using ``rosrun``.

**Style**

* Each tutorial should be focused on teaching the user one feature or interface within MoveIt!.
* Tutorials should flow from show to tell with videos and demos at the beginning followed by explanations.
* New tutorials should match the formatting, style and flow of existing tutorials whenever possible.

### Directory Structure

* Each tutorial should live in its own subdirectory within the `./doc/ <>` directory.
* Add your tutorial to `index.rst` in the root directory.
* Tutorials should use the following directory structure omitting unnecessary files and subdirectories:

```
moveit_tutorials/doc/
└── <tutorial_name>/
    ├── <tutorial_name>_tutorial.rst
    ├── CMakeLists.txt
    ├── package.xml
    ├── setup.py
    ├── images/
    │   └── <tutorial_name>_<image_description>.png
    ├── include/
    │   └── <tutorial_name>/
    │       └── <include_header>.h                      # Any custom C++ library header files
    ├── launch/
    │   └── <tutorial_name>_tutorial.launch
    ├── src/
    │   ├── <tutorial_name>_tutorial.cpp                # Main C++ executable
    │   ├── <include_source>.cpp                        # Custom C++ library source files
    │   └── <tutorial_name>/
    │       ├── __init__.py
    │       ├── <tutorial_name>_tutorial.py             # Main Python executable
    │       └── <python_library>.py                     # Custom Python libraries
    └── test/                                           # Ideally tutorials have their own integration tests
        ├── <tutorial_name>_tutorial.test               # Launch file for tests
        ├── <tutorial_name>_tutorial_test.py            # Python tests for tutorial
        └── <tutorial_name>_tutorial_test.cpp           # C++ tests for tutorial
```
