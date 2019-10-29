add_executable(ros_api_tutorial src/ros_api_tutorial.cpp)
target_link_libraries(ros_api_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ros_api_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
