add_executable(planning_scene_tutorial src/planning_scene_tutorial.cpp)
target_link_libraries(planning_scene_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_scene_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
