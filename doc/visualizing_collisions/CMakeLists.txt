add_executable(visualizing_collisions_tutorial
  src/visualizing_collisions_tutorial.cpp
)

target_link_libraries(visualizing_collisions_tutorial
  ${interactive_markers_LIBRARIES}
  interactivity_utils
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS
  visualizing_collisions_tutorial
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
