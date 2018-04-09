set(INTERACTIVITY_LIB_NAME interactivity_utils)

add_library(${INTERACTIVITY_LIB_NAME}
  src/interactive_robot.cpp
  src/imarker.cpp
  src/pose_string.cpp
)

# add_executable(interactivity_tutorial
  # src/interactivity_tutorial.cpp
  # src/interactive_robot.cpp
  # src/imarker.cpp
  # src/pose_string.cpp
# )
# target_link_libraries(interactivity_tutorial
  # ${catkin_LIBRARIES}
  # ${Boost_LIBRARIES}
  # interactive_markers
# )
#
# add_executable(attached_body_tutorial
  # src/attached_body_tutorial.cpp
  # src/interactive_robot.cpp
  # src/imarker.cpp
  # src/pose_string.cpp
# )

target_link_libraries(
  ${INTERACTIVITY_LIB_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS
    ${INTERACTIVITY_LIB_NAME}
  LIBRARY
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
