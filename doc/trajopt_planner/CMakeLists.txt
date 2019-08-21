add_executable(trajopt_example src/trajopt_example.cpp)
target_link_libraries(trajopt_example  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS trajopt_example DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
