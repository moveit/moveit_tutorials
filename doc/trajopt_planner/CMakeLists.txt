add_executable(test_trajopt src/test_trajopt.cpp)
target_link_libraries(test_trajopt  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS test_trajopt DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
