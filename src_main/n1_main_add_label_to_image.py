#!/usr/bin/env python
# -*- coding: utf-8 -*-


# ROS
import rospy, math
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import PointCloud2
import sensor_msgs

# Common
import numpy as np
import copy
import cv2
from matplotlib import pyplot as plt
import open3d
import time
import os, sys
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"
from collections import deque


# my libriries
sys.path.append(PYTHON_FILE_PATH + "../src_python")
from lib_ros_topic import ColorImageSubscriber, DepthImageSubscriber
from detect_object_from_rgbd import ObjectDetectorFromRGBD
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"

def draw_images(color, depth):
    depth_in_color = cv2.cvtColor((depth/10).astype(np.uint8),cv2.COLOR_GRAY2RGB)
    cv2.imshow("rgb + depth", np.hstack([color, depth_in_color]))
    q = cv2.waitKey(1)
    return chr(q)

if __name__=="__main__":
    
    # Init node
    node_name='detect_object_from_pc'
    rospy.init_node(node_name)
    rospy.sleep(0.3)
    
    # Image subscriber
    sub_color = ColorImageSubscriber(rospy.get_param("topic_color_image"))
    sub_depth = DepthImageSubscriber(rospy.get_param("topic_depth_image"))

    # Object detector
    detector = ObjectDetectorFromRGBD(
            "filename_camera_info",
            "topic_point_cloud",
            "topic_point_cloud_objects",
            "topic_num_objects",
            "topic_objects_on_image",)

    # Process
    cnt = 0
    while not rospy.is_shutdown():
        rospy.sleep(0.01)
        if(sub_color.isReceiveImage() and sub_depth.isReceiveImage):
            try:
                color, t1 = sub_color.get_image()
                depth, t2 = sub_depth.get_image()
            except:
                continue
            cnt+=1

            print "\n======================================"
            print "Node 1: Publishes {:02d}th cloud to .cpp node for detecting objects".format(cnt)

            # Input color and depth image; Receive point clouds and bounding box of each object
            obj_clouds = detector.getObjectsCloudsFromRgbd(color, depth)
            obj_bboxes, img_disp = detector.getObjetctsBboxFromClouds(obj_clouds, color)

            if 0: # DEBUG: save to file
                for i in range(len(obj_clouds)):
                    filename="/home/feiyu/baxterws/src/winter_prj/auto_collect/data/pcobj"+str(i)+".pcd"
                    open3d.write_point_cloud(filename, obj_clouds[i])
                
    # plt.show()
    cv2.destroyAllWindows()
    print "Node stops"


    

    