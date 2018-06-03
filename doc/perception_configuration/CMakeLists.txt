add_executable(cluster_extraction src/cluster_extraction.cpp)
target_link_libraries(cluster_extraction ${catkin_LIBRARIES})

add_executable(remove_floor src/remove_floor.cpp)
target_link_libraries(remove_floor ${catkin_LIBRARIES})

add_executable(cylinder_segment src/cylinder_segment.cpp)
target_link_libraries(cylinder_segment ${catkin_LIBRARIES})