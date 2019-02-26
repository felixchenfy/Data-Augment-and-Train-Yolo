
add_executable(node_detect_object_from_cloud node_detect_object_from_cloud.cpp )
target_link_libraries( node_detect_object_from_cloud
    my_pcl my_basics detect_object_from_cloud
    ${catkin_LIBRARIES} 
)
