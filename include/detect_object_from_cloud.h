// This is for ROS node of "node_detect_object_from_cloud.cpp"

#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"

#include "my_basics/basics.h"
#include "my_pcl/pcl_visualization.h"
#include "my_pcl/pcl_commons.h"
#include "my_pcl/pcl_filters.h"
#include "my_pcl/pcl_advanced.h"
#include "my_pcl/pcl_io.h"

using namespace std;
using namespace pcl;
typedef PointCloud<PointXYZRGB>::Ptr CloudPtr;

class ObjectDetector
{
  public:
    ObjectDetector();
    void callback_sub_pointcloud(const sensor_msgs::PointCloud2 &ros_cloud);
    void pubPclCloudToTopic(ros::Publisher &pub, CloudPtr pcl_cloud);
    int numCloud() { return (int)buff_cloud_src.size(); }
    bool hasCloud() { return numCloud() > 0; }
    CloudPtr popCloud();
    vector<CloudPtr> detect_objects_from_cloud(CloudPtr cloud_src);
    void publishResults(vector<CloudPtr> cloud_clusters);

  private:
    // ============================= Variables =============================
    string topic_point_cloud, topic_point_cloud_clustering_res, topic_point_cloud_objects, topic_num_objects;
    string folder_data;
    queue<CloudPtr> buff_cloud_src; // When the node sub cloud from topic, save it to here

    // ===================== Subscriber and Publisher ======================
    ros::Subscriber sub_pointcloud;
    ros::Publisher pub_pc_clustering_result;
    ros::Publisher pub_pc_objects;
    ros::Publisher pub_num_objects;

    // ============================= Parameters =============================
    // Filter: plane segmentation
    float plane_distance_threshold;
    int plane_max_iterations;
    int num_planes;
    float ratio_of_rest_points;

    // Filter: divide cloud into clusters
    bool flag_do_clustering;
    double cluster_tolerance;
    int min_cluster_size, max_cluster_size, max_num_objects;
};
