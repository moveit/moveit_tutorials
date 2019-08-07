add_executable(test_lerp src/test_lerp.cpp)
target_link_libraries(test_lerp ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_library(
  moveit_lerp_planner_plugin
  src/lerp_planner_manager.cpp
  src/lerp_planning_context.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


set_target_properties(
  moveit_lerp_planner_plugin  PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}"
)

target_link_libraries(
  moveit_lerp_planner_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# LERP planning plugin
add_library(moveit_lerp_planner_plugin src/lerp_planner_manager.cpp)
set_target_properties(moveit_lerp_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(moveit_lerp_planner_plugin ${PROJECT_NAME} ${catkin_LIBRARIES})



install(TARGETS
  moveit_lerp_planner_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES lerp_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
