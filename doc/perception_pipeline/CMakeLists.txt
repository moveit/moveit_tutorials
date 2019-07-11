add_executable(cylinder_segment src/cylinder_segment.cpp)
target_link_libraries(cylinder_segment ${catkin_LIBRARIES})

add_executable(bag_publisher_maintain_time src/bag_publisher_maintain_time.cpp)
target_link_libraries(bag_publisher_maintain_time ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
TARGETS
	bag_publisher_maintain_time
	cylinder_segment
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
