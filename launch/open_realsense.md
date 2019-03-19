$ roslaunch realsense2_camera rs_camera.launch filters:=pointcloud align_depth:=true

$ roslaunch realsense2_camera rs_camera.launch

$ roslaunch realsense2_camera rs_camera.launch filters:=colorizer align_depth:=true

$ roslaunch realsense2_camera rs_camera.launch align_depth:=true

* If using filters:=pointcloud, depth image will be filled with colored. So don't use it.