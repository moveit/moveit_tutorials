add_executable(filtered_depth_subscriber_node src/filtered_depth_subscriber.cpp)
target_link_libraries(filtered_depth_subscriber_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS filtered_depth_subscriber_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})