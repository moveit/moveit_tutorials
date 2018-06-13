add_executable(pick_place_tutorial src/pick_place_tutorial.cpp)
target_link_libraries(pick_place_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_place_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(new_pick src/new_pick.cpp)
target_link_libraries(new_pick ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS new_pick DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})