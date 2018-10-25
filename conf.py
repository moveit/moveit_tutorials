import sys, os

sys.path += [ os.path.abspath( '_scripts' )]

extensions = [ 'sphinx.ext.extlinks',
               'tutorialformatter' ]

# The master toctree document.
master_doc = 'index'

# The suffix of source filenames.
source_suffix = '.rst'

project = u'moveit_tutorials'

# The version info for the project you're documenting, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.
#
# The short X.Y version.
version = 'Kinetic'
# The full version, including alpha/beta/rc tags.
release = 'Kinetic'

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

# Name of the style used to generate the html documentation
html_theme = 'sphinx_rtd_theme'
html_theme_path = ['_themes',]

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

html_context = {
    "display_github": True,
    "github_user": "ros-planning",
    "github_repo": "moveit_tutorials",
    "github_version": "melodic-devel",
    "conf_py_path": "",
    "source_suffix": source_suffix,
    "css_files": ['_static/override.css'],
    "favicon": "favicon.ico"
#  "logo": "logo.png"
}

# Experimental



# Add any paths that contain custom themes here, relative to this directory.

# Links
extlinks = {'codedir': ('https://github.com/' + html_context["github_user"] + '/moveit_tutorials/tree/' + html_context["github_version"] + '/doc/%s', ''),
            'moveit_codedir': ('https://github.com/' + html_context["github_user"] + '/moveit/blob/' + html_context["github_version"] + '/%s', ''),
            'moveit_core': ('http://docs.ros.org/melodic/api/moveit_core/html/classmoveit_1_1core_1_1%s.html', ''),
            'planning_scene': ('http://docs.ros.org/melodic/api/moveit_core/html/classplanning__scene_1_1%s.html', ''),
            'planning_scene_interface': ('http://docs.ros.org/melodic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1%s.html', ''),
            'planning_scene_monitor': ('http://docs.ros.org/melodic/api/moveit_ros_planning/html/classplanning__scene__monitor_1_1%s.html', ''),
            'collision_detection_struct': ('http://docs.ros.org/melodic/api/moveit_core/html/structcollision__detection_1_1%s.html', ''),
            'collision_detection_class': ('http://docs.ros.org/melodic/api/moveit_core/html/classcollision__detection_1_1%s.html', ''),
            'kinematic_constraints': ('http://docs.ros.org/melodic/api/moveit_core/html/classkinematic__constraints_1_1%s.html', ''),
            'moveit_core_files': ('http://docs.ros.org/melodic/api/moveit_core/html/%s.html', ''),
            'move_group_interface': ('http://docs.ros.org/melodic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1%s.html', ''),
            'moveit_website': ('http://moveit.ros.org/%s/', '')}

# Output file base name for HTML help builder.
htmlhelp_basename = 'MoveItDocumentation'
