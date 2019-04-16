add_executable(named_frames_tutorial src/named_frames_tutorial.cpp)
target_link_libraries(named_frames_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS named_frames_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
