
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

// ============================= Variables =============================
string topic_point_cloud, topic_point_cloud_clustering_res, topic_point_cloud_objects, topic_num_objects;
string folder_data;
queue<PointCloud<PointXYZRGB>::Ptr> buff_cloud_src; // When sub cloud from topic, save it to the buff first,
PointCloud<PointXYZRGB>::Ptr cloud_src(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_clustering_res(new PointCloud<PointXYZRGB>);
vector<PointCloud<PointXYZRGB>::Ptr> cloud_clusters;

// ============================= Parameters =============================
// Filter: plane segmentation
float plane_distance_threshold;
int plane_max_iterations;
int num_planes;
float ratio_of_rest_points = -1; // disabled

// Filter: divide cloud into clusters
bool flag_do_clustering;
double cluster_tolerance;
int min_cluster_size, max_cluster_size, max_num_objects;

// ============================= Functions =============================
#define NH_GET_PARAM(param_name, returned_val)                              \
    if (!nh.getParam(param_name, returned_val))                             \
    {                                                                       \
        cout << "Error in reading ROS param named: " << param_name << endl; \
        assert(0);                                                          \
    }

void callback_sub_pointcloud(const sensor_msgs::PointCloud2 &ros_cloud)
{
    static int cnt = 0;
    PointCloud<PointXYZRGB>::Ptr tmp(new PointCloud<PointXYZRGB>);
    fromROSMsg(ros_cloud, *tmp);
    buff_cloud_src.push(tmp);
    printf("\nNode 2 has subscribed %dth cloud. Ready to detect object...\n", ++cnt);
}
void pubPclCloudToTopic(ros::Publisher &pub, PointCloud<PointXYZRGB>::Ptr pcl_cloud)
{
    sensor_msgs::PointCloud2 ros_cloud_to_pub;
    pcl::toROSMsg(*pcl_cloud, ros_cloud_to_pub);
    ros_cloud_to_pub.header.frame_id = "base";
    pub.publish(ros_cloud_to_pub);
}
void initAllROSParams()
{
    {
        ros::NodeHandle nh;
        NH_GET_PARAM("topic_point_cloud", topic_point_cloud);
        NH_GET_PARAM("topic_point_cloud_clustering_res", topic_point_cloud_clustering_res);
        NH_GET_PARAM("topic_point_cloud_objects", topic_point_cloud_objects);
        NH_GET_PARAM("topic_num_objects", topic_num_objects);
        NH_GET_PARAM("folder_data", folder_data);
        
    }
    {
        ros::NodeHandle nh("~");

        // -- Segment plane
        NH_GET_PARAM("plane_distance_threshold", plane_distance_threshold)
        NH_GET_PARAM("plane_max_iterations", plane_max_iterations)
        NH_GET_PARAM("num_planes", num_planes)

        // -- Clustering
        NH_GET_PARAM("flag_do_clustering", flag_do_clustering)
        NH_GET_PARAM("cluster_tolerance", cluster_tolerance)
        NH_GET_PARAM("min_cluster_size", min_cluster_size)
        NH_GET_PARAM("max_cluster_size", max_cluster_size)
        NH_GET_PARAM("max_num_objects", max_num_objects)
    }
}
// ============================= Main =============================
void process_cloud(); // Input cloud_src; Output cloud_clustering_res
void main_loop()
{
    // Subscriber and Publisher
    ros::NodeHandle nh;
    ros::Subscriber sub_pointcloud = nh.subscribe(topic_point_cloud, 5, callback_sub_pointcloud);
    ros::Publisher pub_pc_clustering_result = nh.advertise<sensor_msgs::PointCloud2>(topic_point_cloud_clustering_res, 5);
    ros::Publisher pub_pc_objects = nh.advertise<sensor_msgs::PointCloud2>(topic_point_cloud_objects, 5);
    ros::Publisher pub_num_objects = nh.advertise<std_msgs::Int32>(topic_num_objects, 5);

    // Process
    int cnt_cloud = 0;
    while (ros::ok())
    {
        if (buff_cloud_src.size() > 0)
        {

            // Pop out data
            cnt_cloud += 1;
            cloud_src = buff_cloud_src.front();
            buff_cloud_src.pop();

            // Process data
            process_cloud();

            // Save to file
            // string suffix = my_basics::int2str(cnt_cloud, 5) + ".pcd";
            // string f0 = folder_data + "pcsrc" + suffix;
            // my_pcl::write_point_cloud(f0, cloud_src); // src cloud
            // string f1 = folder_data + "pcres" + suffix;
            // my_pcl::write_point_cloud(f1, cloud_clustering_res); // res cloud

            // -- Publish
            
            // Number of objects
            std_msgs::Int32 _num_objects;
            _num_objects.data=(int)cloud_clusters.size();
            pub_num_objects.publish(_num_objects);

            // Object clustering result
            pubPclCloudToTopic(pub_pc_clustering_result, cloud_clustering_res);
            for(auto cloud:cloud_clusters){
                pubPclCloudToTopic(pub_pc_objects, cloud);
                ros::Duration(0.05).sleep();
            }
            
        }
        ros::spinOnce(); // In python, sub is running in different thread. In C++, same thread. So need this.
        ros::Duration(0.01).sleep();
    }
}

int main(int argc, char **argv)
{
    string node_name = "n2_detect_object_from_pc";
    ros::init(argc, argv, node_name);

    // init parameters
    initAllROSParams();

    // Call main
    main_loop();

    // End
    ROS_INFO("Node2 stops");
    return 0;
}
void process_cloud()
{
    cloud_clusters.clear(); // This is 1st output
    // cloud_clustering_res // This is 2nd output

    PointCloud<PointXYZRGB>::Ptr cloud_no_plane(new PointCloud<PointXYZRGB>);
    pcl::copyPointCloud(*cloud_src, *cloud_no_plane);

    // ---------------------------------------------
    // -- Remove planes
    PointCloud<PointXYZRGB>::Ptr plane(new PointCloud<PointXYZRGB>);
    int num_removed_planes = my_pcl::removePlanes(
        cloud_no_plane, plane,
        plane_distance_threshold, plane_max_iterations,
        num_planes, ratio_of_rest_points);

    // -- Clustering: Divide the remaining point cloud into different clusters
    while (1)
    {
        // -- Do clustering
        vector<PointIndices> clusters_indices;
        clusters_indices = my_pcl::divideIntoClusters(
            cloud_no_plane, cluster_tolerance, min_cluster_size, max_cluster_size);
        if (clusters_indices.size() > max_num_objects)
            clusters_indices.resize(max_num_objects);
        int num_object = clusters_indices.size();
        if (num_object == 0)
        {
            pcl::copyPointCloud(*cloud_src, *cloud_clustering_res);
            break;
        }
        // -- Extract sub clouds
        cloud_clusters = my_pcl::extractSubCloudsByIndices(cloud_no_plane, clusters_indices);

        // -- add colors
        for (PointXYZRGB &p : cloud_no_plane->points)
            my_pcl::setPointColor(p, 255, 255, 255); // Set all points' color to white

        for (PointXYZRGB &p : plane->points)
            my_pcl::setPointColor(p, 255, 255, 255); // Set all points' color to white

        vector<vector<unsigned char>> colors({{255, 0, 0}, {0, 255, 0}, {0, 0, 255}});
        assert(max_num_objects <= colors.size());
        int i = 0;

        for (std::vector<PointIndices>::const_iterator it = clusters_indices.begin(); it != clusters_indices.end(); ++it)
        { // iterate clusters_indices
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            { // iterate each point index
                my_pcl::setPointColor(cloud_no_plane->points[*pit], colors[i][0], colors[i][1], colors[i][1]);
            }
            i++;
        }

        // copy data
        pcl::copyPointCloud(*cloud_no_plane, *cloud_clustering_res);
        *cloud_clustering_res += *plane;
        break;
    }

    // Print result
    cout << "size of cloud_src: ";
    my_pcl::printCloudSize(cloud_src);

    cout << "size of the " << cloud_clusters.size() << " objects: ";
    for (auto cloud : cloud_clusters)
        cout << cloud->width * cloud->height << ", ";

    cout << "\nsize of cloud_clustering_res: ";
    my_pcl::printCloudSize(cloud_clustering_res);
}