# MoveIt Tutorials

[MoveIt 1 Live tutorials](https://ros-planning.github.io/moveit_tutorials/)

*These are the tutorials for MoveIt 1, for MoveIt 2 see [MoveIt 2 Tutorials](https://github.com/ros-planning/moveit2_tutorials/)*

This is the primary documentation for the MoveIt project. We strongly encourage you to help improve MoveIt's documentation. Please consider reading the guidelines below for writing the best documentation and tutorials. However, if you are uncomfortable with any of the approaches, simply adding documentation text to your pull requests is better than nothing.

These tutorials use the [reStructuredText](http://www.sphinx-doc.org/en/stable/rest.html) format commonly used in the Sphinx "Python Documentation Generator". This unfortunately differs from the common Markdown format, but its advantage is that it supports embedding code directly from source files for inline code tutorials.

All content in this repository is open source and released under the [BSD License v3](https://opensource.org/licenses/BSD-3-Clause). Each individual source code file should contain a copy of the license.

This repository is currently built automatically by two systems. Github Actions builds the documentation for Noetic, and ROS Build Farm builds the documentation for older versions:

- [ROS Noetic](https://ros-planning.github.io/moveit_tutorials/): [![CI](https://github.com/ros-planning/moveit_tutorials/actions/workflows/ci.yaml/badge.svg?branch=master)](https://github.com/ros-planning/moveit_tutorials/actions/workflows/ci.yaml?query=branch%3Amaster) [![Deploy](https://github.com/ros-planning/moveit_tutorials/actions/workflows/deploy.yaml/badge.svg?branch=master)](https://github.com/ros-planning/moveit_tutorials/actions/workflows/deploy.yaml?query=branch%3Amaster) [![Formatting](https://github.com/ros-planning/moveit_tutorials/actions/workflows/format.yaml/badge.svg?branch=master)](https://github.com/ros-planning/moveit_tutorials/actions/workflows/format.yaml?query=branch%3Amaster)
- [ROS Melodic](http://docs.ros.org/melodic/api/moveit_tutorials/html/): [![build farm](http://build.ros.org/buildStatus/icon?job=Mdoc__moveit_tutorials__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdoc__moveit_tutorials__ubuntu_bionic_amd64/)
- [ROS Kinetic](http://docs.ros.org/kinetic/api/moveit_tutorials/html/): [![build farm](http://build.ros.org/buildStatus/icon?job=Kdoc__moveit_tutorials__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdoc__moveit_tutorials__ubuntu_xenial_amd64/)

## Versions

- ``indigo-devel`` usage is discouraged
- ``kinetic-devel`` stable
- ``melodic-devel`` stable
- ``master`` latest, changes should target this branch

## Build Locally

If you want to test the tutorials by generating the html pages locally on your machine, you can use the ``build_locally`` script.
Run in the root of the moveit_tutorials package:

    export ROS_DISTRO=kinetic # 16.04
    export ROS_DISTRO=melodic # 18.04
    export ROS_DISTRO=noetic  # 20.04

    source /opt/ros/$ROS_DISTRO/setup.bash
    ./build_locally.sh

The local website ``<LOCAL_PACKAGE_PATH>/build/html/index.html`` should automatically open in your web browser.

## ROS Build Farm Deployment

For deploying documentation changes to the web, [Section 3 of rosdoc_lite wiki](http://wiki.ros.org/rosdoc_lite) says that "rosdoc_lite is automatically run for packages in repositories that have rosinstall files listed in the rosdistro repository." This is done about once every 24 hours, [overnight](http://wiki.ros.org/rosdistro/Tutorials/Indexing%20Your%20ROS%20Repository%20for%20Documentation%20Generation).

## Contributing

We rely on the community to keep these tutorials up to date and bug free. If you find an issue with the tutorials please [open an issue on GitHub](https://github.com/ros-planning/moveit_tutorials/issues/new) or open a PR with proposed changes.

### Formatting and Style

**Code Formatting**

* These tutorials use the same [style guidelines](http://moveit.ros.org/documentation/contributing/code/) as the MoveIt project. When modifying or adding to these tutorials, it is required that code is auto formatted using [clang-format](http://moveit.ros.org/documentation/contributing/code/). To check and apply our style guidelines we use [pre-commit](https://pre-commit.com/).
* Tutorials should exemplify best coding practices. If a contribution wouldn't pass review in the MoveIt project, then it shouldn't pass review in the tutorials.
* Relevant code should be included and explained using the ``.. tutorial-formatter::`` tag.
* Irrelevant code should be excluded from the generated html using the ``BEGIN_TUTORIAL``, ``END_TUTORIAL``, ``BEGIN_SUB_TUTORIAL``, and ``END_SUB_TUTORIAL`` tags.
* Whenever possible, links should be created using the ``extlinks`` dictionary defined in ``conf.py``.
* All demo code should be runnable from within the ``moveit_tutorials`` package.
* Python code should be run using ``rosrun``.

**Style**

* Each tutorial should be focused on teaching the user one feature or interface within MoveIt.
* Tutorials should flow from show to tell with videos and demos at the beginning followed by explanations.
* New tutorials should match the formatting, style and flow of existing tutorials whenever possible.

**pre-commit**

pre-commit is a tool that is used in ``moveit_tutorials`` to check and apply style guidelines automatically. To install pre-commit into your system:

    pip3 install pre-commit

In you catkin workspace, under ``moveit_tutorials`` directory you can install the git hooks like this:

    cd $CATKIN_WS/src/moveit_tutorials && pre-commit install

With this pre-commit will automatically run and check a list of styling including clang-format, end of files and trailing whitespaces whenever you run ``git commit``. To run pre-commit any time other than ``git commit`` you can use the following command:

    cd $CATKIN_WS/src/moveit_tutorials && pre-commit run -a

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

### Including Images and Videos
#### Images
The standard way to include an image in reStructuredText is
```
.. image:: filename.png
   :width: 700px
```

This assumes that `filename.png` is in the same folder as the source `.rst` file. Images linked in this way will automatically be copied to the appropriate folder in the build.

[External Documentation](https://sublime-and-sphinx-guide.readthedocs.io/en/latest/images.html)

Do **not** include animated gifs as the file format leads to very large files. Use a video format like `webm` and see the section on local video below.

#### YouTube and other External Video
You can embed video with raw html, like in this example from the Pick and Place Tutorial.
```
.. raw:: html

    <div style="position: relative; padding-bottom: 5%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700px" height="400px" src="https://www.youtube.com/embed/QBJPxx_63Bs?rel=0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
    </div>
```
This includes [Youtube's suggested embed html](https://support.google.com/youtube/answer/171780?hl=en).

#### Local Video
To embed a video that is included in this repository, you also will use raw html, like this example from the Quickstart in RViz tutorial.

```
.. raw:: html

    <video width="700px" nocontrols="true" autoplay="true" loop="true">
        <source src="../../_static/rviz_joints_nullspace.webm" type="video/webm">
        The joints moving while the end effector stays still
    </video>
```

Note that the video file is in the `_static` folder instead of the same folder.

[External Documentation on &lt;video&gt; tag](https://developer.mozilla.org/en-US/docs/Web/HTML/Element/video)
