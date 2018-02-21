add_executable(collision_contact_tutorial
  src/collision_contact_tutorial.cpp
)

target_link_libraries(
  collision_contact_tutorial
  interactive_markers
  interactivity_utils
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS
  collision_contact_tutorial
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
