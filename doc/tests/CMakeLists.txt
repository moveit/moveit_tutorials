if(CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})

  catkin_add_gtest(tests_tutorial test/tests.cpp)
  target_link_libraries(tests_tutorial
    ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()
