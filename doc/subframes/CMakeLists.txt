add_executable(subframes_tutorial src/subframes_tutorial.cpp)
target_link_libraries(subframes_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS subframes_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
