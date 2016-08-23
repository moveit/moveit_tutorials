add_executable(planning_scene_tutorial src/planning_scene_tutorial.cpp)
target_link_libraries(planning_scene_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_scene_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planning_scene_ros_api_tutorial src/planning_scene_ros_api_tutorial.cpp)
target_link_libraries(planning_scene_ros_api_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_scene_ros_api_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(motion_planning_api_tutorial src/motion_planning_api_tutorial.cpp)
target_link_libraries(motion_planning_api_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS motion_planning_api_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planning_pipeline_tutorial src/planning_pipeline_tutorial.cpp)
target_link_libraries(planning_pipeline_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_pipeline_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS
  scripts/move_group_python_interface_tutorial.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
