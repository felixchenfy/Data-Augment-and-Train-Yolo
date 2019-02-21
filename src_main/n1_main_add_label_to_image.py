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

# ==================================================================================================

def filtCloud(cloud):
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

def draw_images(color, depth):
    depth_in_color = cv2.cvtColor((depth/10).astype(np.uint8),cv2.COLOR_GRAY2RGB)
    cv2.imshow("rgb + depth", np.hstack([color, depth_in_color]))
    q = cv2.waitKey(1)
    return chr(q)

class ObjectsCloudsSubscriber(object):
    def __init__(self, topic_num_objects, topic_point_cloud_objects):
        self.num_objects=None;
        self.num_objects_to_receive=None;
        self.clouds_buff=deque()
        self.flag_receive_all=False
        self.sub_num_objects = rospy.Subscriber(topic_num_objects, Int32, self._call_back_sub_num_objects)
        self.sub_objects = rospy.Subscriber(topic_point_cloud_objects, PointCloud2, self._call_back_sub_objects)

    
    def _call_back_sub_num_objects(self, num_objects):
        self.num_objects=num_objects.data
        self.num_objects_to_receive=num_objects.data
        self.clouds_buff.clear()
        self.flag_receive_all=False
        print "\nNode 1 starts subscribing {:d} objects' point cloud: ".format(self.num_objects)

    def _call_back_sub_objects(self, ros_cloud):
        self.clouds_buff.append(ros_cloud)
        self.num_objects_to_receive-=1
        # print "{:d} objects remained to receive".format(self.num_objects_to_receive)
        # print "{:d} objects remained to process".format(len(self.clouds_buff))
        if self.num_objects_to_receive==0:
            self.flag_receive_all=True

class ExternalBboxFinder(object):
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
        p0=[self.xmin, self.ymin]
        p1=[self.xmax, self.ymax]
        return p0, p1


class PointCloudsProcessor(object):
    
    def __init__(self, camera_intrinsic):
        self.K = camera_intrinsic.intrinsic_matrix
        self.width = camera_intrinsic.width
        self.height = camera_intrinsic.height
        rgb=['r','g','b']
        colormap={'b':[255,0,0],'g':[0,255,0],'r':[0,0,255]}
        self.getColor=lambda idx:colormap[rgb[idx]]
        self.list_float2int = lambda arr : [int(math.floor(i)) for i in arr]

    def computeCloudsBboxOnImage(self, cloud, color_idx=2, img_disp=None):
        color = np.array(self.getColor(color_idx))
        pc_points, _ = getCloudContents(cloud)
        N = pc_points.shape[0]
        bbox_finder = ExternalBboxFinder()
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
        p0, p1 = bbox_finder.returnBbox()        
        return self.list_float2int(p0), self.list_float2int(p1)

# ==================================================================================================
if __name__=="__main__":
    
    # Init node
    node_name='detect_object_from_pc'
    rospy.init_node(node_name)
    rospy.sleep(0.5)
    bridge = CvBridge()
    
    # Params
    MAX_PROC_TIME_PER_FRAME = rospy.get_param("~MAX_PROC_TIME_PER_FRAME")

    # Camera info
    camera_intrinsic = read_pinhole_camera_intrinsic(rospy.get_param("file_camera_info"))
    
    # Image subscriber
    sub_color = ColorImageSubscriber(rospy.get_param("topic_color_image"))
    sub_depth = DepthImageSubscriber(rospy.get_param("topic_depth_image"))
    pub_cloud = CloudPublisher(rospy.get_param("topic_point_cloud"))
    sub_objects = ObjectsCloudsSubscriber(rospy.get_param("topic_num_objects"),rospy.get_param("topic_point_cloud_objects"))
    pub_objects_on_image = ImagePublisher(rospy.get_param("topic_objects_on_image"), img_format="color")

    # Process
    prc=PointCloudsProcessor(camera_intrinsic)
    cnt = 0
    while not rospy.is_shutdown():
        if(sub_color.isReceiveImage() and sub_depth.isReceiveImage):
            try:
                color, t1 = sub_color.get_image()
                depth, t2 = sub_depth.get_image()
            except:
                continue
            cnt+=1
            img_disp = color.copy()
            # key = draw_images(color, depth)

            # Send data to Node 2 for processing
            cloud = rgbd2cloud(color, depth, camera_intrinsic, img_format="cv2")
            cloud = filtCloud(cloud)
            pub_cloud.publish(cloud, cloud_format="open3d")
            print "\n======================================"
            print "Node 1: Publish a cloud " + str(cnt)

            # Receive each object's point cloud and project them onto image
            cnt_processed=0
            start = time.time()
            while(((not sub_objects.flag_receive_all) or cnt_processed<sub_objects.num_objects) and (not rospy.is_shutdown())): # wait until processed all clouds
                if len(sub_objects.clouds_buff)>0: # A new cloud has been received
                    cnt_processed+=1
                    print "Node 1: processing object " + str(cnt_processed)
                    cloud_ros_format = sub_objects.clouds_buff.popleft()
                    cloud = convertCloudFromRosToOpen3d(cloud_ros_format)
                    if 0: # DEBUG: save to file
                        filename="/home/feiyu/baxterws/src/winter_prj/auto_collect/data/pcobj"+str(cnt_processed)+".pcd"
                        open3d.write_point_cloud(filename,cloud)
                    
                    # Main processing
                    p0, p1 = prc.computeCloudsBboxOnImage(cloud, img_disp=img_disp)
                    drawBoxToImage(img_disp, p0, p1, color='r',line_width=4)

                rospy.sleep(0.001)
                if time.time()-start>MAX_PROC_TIME_PER_FRAME:
                    break
            if cnt_processed==sub_objects.num_objects:
                print "Node 1: ALL OBJECTS BEEN PROCESSED. COOL !!!"
                pub_objects_on_image.publish(img_disp)
            else:
                print "Node 1: miss some clouds?"

        # End one frame
        rospy.sleep(0.01)

    plt.show()
    cv2.destroyAllWindows()
    print "Node stops"
    # rospy.spin()


    

    