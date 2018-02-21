add_executable(kinematic_model_tutorial src/kinematic_model_tutorial.cpp)
target_link_libraries(kinematic_model_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS kinematic_model_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
