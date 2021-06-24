add_executable(bullet_collision_checker_tutorial
    src/bullet_collision_checker_tutorial.cpp)

target_link_libraries(bullet_collision_checker_tutorial
    ${interactive_markers_LIBRARIES}
    interactivity_utils
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})

install(TARGETS bullet_collision_checker_tutorial
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
