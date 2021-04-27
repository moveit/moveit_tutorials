add_executable(detect_and_add_cylinder_collision_object_demo src/detect_and_add_cylinder_collision_object_demo.cpp)
target_link_libraries(detect_and_add_cylinder_collision_object_demo ${catkin_LIBRARIES})

add_executable(bag_publisher_maintain_time src/bag_publisher_maintain_time.cpp)
target_link_libraries(bag_publisher_maintain_time ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
TARGETS
    bag_publisher_maintain_time
    detect_and_add_cylinder_collision_object_demo
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY bags DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc/perception_pipeline/)
