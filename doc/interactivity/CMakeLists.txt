add_executable(interactivity_tutorial
  src/interactivity_tutorial.cpp
  src/interactive_robot.cpp
  src/imarker.cpp
  src/pose_string.cpp
)
target_link_libraries(interactivity_tutorial
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  interactive_markers
)

add_executable(collision_contact_tutorial
  src/collision_contact_tutorial.cpp
  src/interactive_robot.cpp
  src/imarker.cpp
  src/pose_string.cpp
)
target_link_libraries(collision_contact_tutorial
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  interactive_markers
)

add_executable(attached_body_tutorial
  src/attached_body_tutorial.cpp
  src/interactive_robot.cpp
  src/imarker.cpp
  src/pose_string.cpp
)
target_link_libraries(attached_body_tutorial
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  interactive_markers
)

install(
  TARGETS
  interactivity_tutorial
  collision_contact_tutorial
  attached_body_tutorial
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
