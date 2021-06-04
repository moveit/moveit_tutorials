import sys, os

sys.path += [os.path.abspath("_scripts")]

extensions = ["sphinx.ext.extlinks", "tutorialformatter"]

# The master toctree document.
master_doc = "index"

# The suffix of source filenames.
source_suffix = ".rst"

project = u"moveit_tutorials"

# The version info for the project you're documenting, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.
#
# The short X.Y version.
version = "Noetic"
# The full version, including alpha/beta/rc tags.
release = "Noetic"

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = "sphinx"

# Name of the style used to generate the html documentation
html_theme = "sphinx_rtd_theme"
html_theme_path = [
    "_themes",
]

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]

html_context = {
    "display_github": True,
    "github_user": "ros-planning",
    "github_repo": "moveit_tutorials",
    "github_version": "master",
    "conf_py_path": "",
    "source_suffix": source_suffix,
    "css_files": ["_static/override.css"],
    "favicon": "favicon.ico"
    #  "logo": "logo.png"
}

# Add any paths that contain custom themes here, relative to this directory.

# Links
ros_distro = "noetic"
extlinks = {
    "codedir": (
        "https://github.com/"
        + html_context["github_user"]
        + "/moveit_tutorials/blob/"
        + html_context["github_version"]
        + "/doc/%s",
        "",
    ),
    "moveit_codedir": (
        "https://github.com/"
        + html_context["github_user"]
        + "/moveit/blob/"
        + html_context["github_version"]
        + "/%s",
        "",
    ),
    "panda_codedir": (
        "https://github.com/"
        + html_context["github_user"]
        + "/panda_moveit_config/blob/"
        + "melodic-devel"
        + "/%s",
        "",
    ),  # TODO(dlu): use ros_distro when noetic-devel branch is available
    "rosdocs": ("http://docs.ros.org/" + ros_distro + "/api/%s", ""),
    "moveit_core": (
        "http://docs.ros.org/"
        + ros_distro
        + "/api/moveit_core/html/cpp/classmoveit_1_1core_1_1%s.html",
        "",
    ),
    "planning_scene": (
        "http://docs.ros.org/"
        + ros_distro
        + "/api/moveit_core/html/cpp/classplanning__scene_1_1%s.html",
        "",
    ),
    "planning_scene_monitor": (
        "http://docs.ros.org/"
        + ros_distro
        + "/api/moveit_ros_planning/html/classplanning__scene__monitor_1_1%s.html",
        "",
    ),
    "collision_detection_struct": (
        "http://docs.ros.org/"
        + ros_distro
        + "/api/moveit_core/html/cpp/structcollision__detection_1_1%s.html",
        "",
    ),
    "collision_detection_class": (
        "http://docs.ros.org/"
        + ros_distro
        + "/api/moveit_core/html/cpp/classcollision__detection_1_1%s.html",
        "",
    ),
    "kinematic_constraints": (
        "http://docs.ros.org/"
        + ros_distro
        + "/api/moveit_core/html/cpp/classkinematic__constraints_1_1%s.html",
        "",
    ),
    "moveit_core_files": (
        "http://docs.ros.org/" + ros_distro + "/api/moveit_core/html/cpp/%s.html",
        "",
    ),
    "moveit_website": ("http://moveit.ros.org/%s/", ""),
    "locked_planning_scene": (
        "http://docs.ros.org/"
        + ros_distro
        + "/api/moveit_ros_planning/html/namespaceplanning__scene__monitor.html",
        "",
    ),
    "planning_interface": (
        "http://docs.ros.org/"
        + ros_distro
        + "/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1%s.html",
        "",
    ),
    "sensor_msgs": (
        "http://docs.ros.org/" + ros_distro + "/api/sensor_msgs/html/msg/%s.html",
        "",
    ),
    "moveit_msgs": (
        "http://docs.ros.org/" + ros_distro + "/api/moveit_msgs/html/msg/%s.html",
        "",
    ),
    "tf2": (
        "http://docs.ros.org/"
        + ros_distro
        + "/api/tf2_ros/html/c++/classtf2__ros_1_1%s.html",
        "",
    ),
}

# Output file base name for HTML help builder.
htmlhelp_basename = "MoveItDocumentation"
