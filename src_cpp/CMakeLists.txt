

add_library(my_pcl SHARED
    my_pcl/pcl_commons.cpp
    my_pcl/pcl_io.cpp
    my_pcl/pcl_visualization.cpp
    my_pcl/pcl_filters.cpp
    my_pcl/pcl_advanced.cpp
)

add_library(my_basics SHARED
    my_basics/basics.cpp
    my_basics/eigen_funcs.cpp
)

target_link_libraries( my_basics
    ${THIRD_PARTY_LIBS} 
)


target_link_libraries( my_pcl
    ${THIRD_PARTY_LIBS} 
    my_basics
)

# ----------------------- scripts -----------------------

add_library(detect_object_from_cloud SHARED
    detect_object_from_cloud.cpp
)
target_link_libraries( detect_object_from_cloud
    ${THIRD_PARTY_LIBS} 
    my_basics my_pcl
    ${catkin_LIBRARIES} 
)





