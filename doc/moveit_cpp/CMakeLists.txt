add_executable(moveit_cpp_tutorial src/moveit_cpp_tutorial.cpp)
target_link_libraries(moveit_cpp_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS moveit_cpp_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
