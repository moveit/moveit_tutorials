add_executable(ros_api_tutorial src/ros_api_tutorial.cpp)
target_link_libraries(ros_api_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
