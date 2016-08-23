cmake_minimum_required(VERSION 2.8.3)
project(moveit_tutorials)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    cmake_modules
    geometric_shapes
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)
find_package(Eigen REQUIRED)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(doc/pr2_tutorials/kinematics)
add_subdirectory(doc/pr2_tutorials/planning)
add_subdirectory(doc/pr2_tutorials/state_display)
add_subdirectory(doc/pr2_tutorials/interactivity)
add_subdirectory(doc/pr2_tutorials/pick_place)
