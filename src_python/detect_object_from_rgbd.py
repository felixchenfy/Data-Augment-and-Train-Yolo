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
from lib_cloud import *
from lib_ros_topic import ColorImageSubscriber, DepthImageSubscriber, CloudPublisher, ImagePublisher
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromRosToOpen3d
from lib_cam_calib import drawBoxToImage
from lib_geo_trans import cam2pixel
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"

class ObjectDetectorFromRGBD(object):

    def __init__(self,
            filename_camera_info,
            topic_point_cloud,
            topic_point_cloud_objects,
            topic_num_objects,
            topic_objects_on_image,
        ):
        
        # Camera info
        self.camera_intrinsic = read_pinhole_camera_intrinsic(rospy.get_param(filename_camera_info))
    
        # Use sub/pub to communicate with the .cpp file which does the job of "object detection from cloud" 
        self.pub_cloud = CloudPublisher(rospy.get_param(topic_point_cloud))
        self.sub_objects = ObjectsCloudsSubscriber(rospy.get_param(topic_num_objects),rospy.get_param(topic_point_cloud_objects))
        self.pub_image_with_bbox = ImagePublisher(rospy.get_param(topic_objects_on_image), img_format="color")

        # Params
        self.max_wait_time = 0.5

    def getObjectsCloudsFromRgbd(self, color, depth):
        obj_clouds = list()

        # Generate cloud data from color and depth images
        cloud = rgbd2cloud(color, depth, self.camera_intrinsic, img_format="cv2")
        cloud = self.filtCloud(cloud)

        # Send cloud to .cpp node for detect object
        self.sub_objects.reset()
        self.pub_cloud.publish(cloud, cloud_format="open3d")
        while(self.sub_objects.num_objects is None): # wait until receives the result from .cpp node
            rospy.sleep(0.001)
            

        # Receive detection results, which are a list of clouds
        cnt_processed=0
        start = time.time()
        while(((not self.sub_objects.flag_receive_all)) and (not rospy.is_shutdown())): # wait until processed all obj clouds
            if len(self.sub_objects.clouds_buff)>0: # A new cloud has been received
                cnt_processed+=1
                cloud_ros_format = self.sub_objects.pop_cloud() # pop cloud from buff
                obj_clouds.append(convertCloudFromRosToOpen3d((cloud_ros_format))) # save to results

            # Deal with error: Set up max waiting time
            rospy.sleep(0.001)
            if time.time()-start>self.max_wait_time:
                break

        # Check error
        num_unreceived = self.sub_objects.num_objects - cnt_processed
        if num_unreceived!=0:
            print "my Warning: might miss a number of {:02d} objects".format(num_unreceived)
        
        # Return
        return obj_clouds

    def getObjetctsBboxFromClouds(self, obj_clouds, color):
        img_disp = color.copy()
        bbox_finder = Clouds2Bbox(self.camera_intrinsic)
        obj_bboxes = list()
        # Input cloud; Get bounding box on image
        for i in range(len(obj_clouds)): 
            bbox = bbox_finder.computeCloudsBboxOnImage(obj_clouds[i], img_disp=img_disp)
            obj_bboxes.append(bbox)
            drawBoxToImage(img_disp, [bbox.x0, bbox.y0], [bbox.x1, bbox.y1], color='r',line_width=4)

        if 1:
            self.pub_image_with_bbox.publish(img_disp) # publish for displaying
        return obj_bboxes, img_disp

    def filtCloud(self, cloud):
        # filt by downsample + range 
        cloud = voxel_down_sample(cloud, 0.005)
        cloud = filtCloudByRange(cloud, zmin=0.2, zmax=0.5)

        # Statistical outlier removal
        cl,ind = statistical_outlier_removal(cloud,
            nb_neighbors=20, std_ratio=2.0)
        cloud = select_down_sample(cloud, ind, invert=False)

        # Radius outlier removal
        cl,ind = radius_outlier_removal(cloud,
            nb_points=16, radius=0.05)
        cloud = select_down_sample(cloud, ind, invert=False)
        return cloud


class ObjectsCloudsSubscriber(object):
    def __init__(self, topic_num_objects, topic_point_cloud_objects):
        self.num_objects=None;
        self.num_objects_to_receive=None;
        self.clouds_buff=deque()
        self.flag_receive_all=None
        self.sub_num_objects = rospy.Subscriber(topic_num_objects, Int32, self._call_back_sub_num_objects)
        self.sub_objects = rospy.Subscriber(topic_point_cloud_objects, PointCloud2, self._call_back_sub_objects)
        # self.sub_objects = rospy.Subscriber(topic_point_cloud_objects, PointCloud2, self._call_back_sub_objects,rospy.TransportHints().tcpNoDelay())
    
    def reset(self):
        self.num_objects=None;
        self.num_objects_to_receive=None;
        # print "before clear: clouds_buff.size()=" , len(self.clouds_buff)
        self.clouds_buff=deque()
        self.flag_receive_all=None
    
    def _call_back_sub_num_objects(self, num_objects):
        self.num_objects=num_objects.data
        self.num_objects_to_receive=num_objects.data
        self.flag_receive_all=False

    def _call_back_sub_objects(self, cloud_ros_format):
        self.clouds_buff.append(cloud_ros_format)

    def pop_cloud(self):
        cloud_ros_format = self.clouds_buff.popleft()
        self.num_objects_to_receive-=1
        if self.num_objects_to_receive==0:
            self.flag_receive_all=True
        return cloud_ros_format

class Bbox(object):
    def __init__(self, x0, y0, x1, y1):
        self.x0 = x0
        self.y0 = y0
        self.x1 = x1
        self.y1 = y1
        self.list_float2int = lambda arr : [int(math.floor(i)) for i in arr]

    def convertToInt(self):
        self.x0 = int(math.floor(self.x0))
        self.y0 = int(math.floor(self.y0))
        self.x1 = int(math.floor(self.x1))
        self.y1 = int(math.floor(self.y1))

class BboxOfPoints(object):
    def __init__(self):
        inf=999999
        self.xmin=inf
        self.xmax=-inf
        self.ymin=inf
        self.ymax=-inf
    
    def addNewPoint(self, xy):
        x=xy[0]
        y=xy[1]
        self.xmin=min(self.xmin, x)
        self.ymin=min(self.ymin, y)
        self.xmax=max(self.xmax, x)
        self.ymax=max(self.ymax, y)
    
    def returnBbox(self):
        return Bbox(self.xmin, self.ymin, self.xmax, self.ymax)
        


class Clouds2Bbox(object):
    
    def __init__(self, camera_intrinsic):
        self.K = camera_intrinsic.intrinsic_matrix
        self.width = camera_intrinsic.width
        self.height = camera_intrinsic.height
        rgb=['r','g','b']
        colormap={'b':[255,0,0],'g':[0,255,0],'r':[0,0,255]}
        self.getColor=lambda idx:colormap[rgb[idx]]

    def computeCloudsBboxOnImage(self, cloud, color_idx=2, img_disp=None):
        color = np.array(self.getColor(color_idx))
        pc_points, _ = getCloudContents(cloud)
        N = pc_points.shape[0]
        bbox_finder = BboxOfPoints()
        for i in range(N):
            uv = cam2pixel(pc_points[i:i+1,:].transpose(), self.K)
            bbox_finder.addNewPoint(uv)
            if img_disp is not None: # Add color to the image
                x, y = int(uv[0]), int(uv[1])
                r=5
                try:
                    img_disp[y-r:y+r,x-r:x+r]=color
                except:
                    None
        bbox = bbox_finder.returnBbox()        
        bbox.convertToInt()
        return bbox
