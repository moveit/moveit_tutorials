add_library(controller_manager_example src/moveit_controller_manager_example.cpp)
target_link_libraries(controller_manager_example ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS controller_manager_example DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Need to use non-standard install location to find plugin description both in devel and install space
install(FILES moveit_controller_manager_example_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc/controller_configuration)
