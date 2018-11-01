catkin_add_gtest(tests_tutorial src/tests.cpp)
target_link_libraries(tests_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS tests_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
