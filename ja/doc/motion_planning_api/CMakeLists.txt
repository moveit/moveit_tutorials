add_executable(motion_planning_api_tutorial src/motion_planning_api_tutorial.cpp)
target_link_libraries(motion_planning_api_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS motion_planning_api_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
