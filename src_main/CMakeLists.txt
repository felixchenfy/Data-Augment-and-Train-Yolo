
add_executable(n2_detect_object_from_pc n2_detect_object_from_pc.cpp )
target_link_libraries( n2_detect_object_from_pc
    my_pcl my_basics
    ${catkin_LIBRARIES} 
)
