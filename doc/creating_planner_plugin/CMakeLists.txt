# EmptyPlan planning plugin
add_library(moveit_emptyplan_planner_plugin src/emptyplan_planner_manager.cpp)
set_target_properties(moveit_emptyplan_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

install(TARGETS
  moveit_emptyplan_planner_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES emptyplan_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
