add_executable(moveit_task_constructot_tutorial src/moveit_task_constructor_tutorial.cpp)
target_link_libraries(moveit_task_constructor_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS moveit_task_constructor_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
