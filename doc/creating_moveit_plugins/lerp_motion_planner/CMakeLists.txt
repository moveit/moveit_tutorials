include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/lerp_interface.cpp
  src/lerp_planning_context.cpp
)

target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
set_target_properties(${PROJECT_NAME}  PROPERTIES  VERSION  "${${PROJECT_NAME}_VERSION}")

add_executable(lerp_example  src/lerp_example.cpp)
target_link_libraries(lerp_example
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Lerp planning plugin
add_library(moveit_lerp_planner_plugin src/lerp_planner_manager.cpp)
set_target_properties(moveit_lerp_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(moveit_lerp_planner_plugin ${PROJECT_NAME}  ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    moveit_lerp_planner_plugin
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Mark roslaunch files for installation
install(
  DIRECTORY
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES lerp_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc/creating_moveit_plugins/lerp_motion_planner/)
