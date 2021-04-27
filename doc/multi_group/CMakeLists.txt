add_executable(multi_group_tutorial src/multi_group_tutorial.cpp)
target_link_libraries(multi_group_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS multi_group_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})