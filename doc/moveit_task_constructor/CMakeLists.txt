add_executable(extension  src/extension_tutorial.cpp)
target_link_libraries(extension
  ${catkin_LIBRARIES}
)

# Executables
install(
  TARGETS
    extension
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)