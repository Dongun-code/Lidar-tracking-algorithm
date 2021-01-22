#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "util.hpp"
// #include "kalman_hand.hpp"

ros::Publisher pub, pub2, pub3;

// double cluster_value, centroid_distance, leaf_size;
double cluster_value = 0.5;
double centroid_distance = 1.6;
double leaf_size = 0.3;


class frameTracker
{

public:

    // std::vector<std::vector<pcl::PointXYZI>> compareVector;
    pcl::PointCloud<pcl::PointXYZI> outCloud;
    lidarUtil util;



    void Callback(const sensor_msgs::PointCloud2& msg)
    {

        //set variable and convert ros msg

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new  pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr projection_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(msg, *cloud);


        //Point roi & Point projection

        output_cloud = util.point_roi(cloud);
        projection_cloud = util.point_projection(output_cloud);

            // std::cout<<"Input: "<<cloud->points.size()<<" ( "<<pcl::getFieldsList(*cloud)<<")"<<std::endl;
        //  voxel
        
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(projection_cloud);
        sor.setLeafSize(0.3f,0.3f,0.3f);
        sor.setLeafSize(leaf_size,leaf_size,leaf_size);
        sor.filter(*projection_cloud);

        // std::cout << "Output : " << cloud_filtered->points.size () << " (" << pcl::getFieldsList (*cloud_filtered) <<")"<< std::endl;

        //Create  object
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(projection_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(cluster_value);
        ec.setMinClusterSize(15);
        ec.setMaxClusterSize(300);
        ec.setSearchMethod(tree);
        ec.setInputCloud(projection_cloud);
        ec.extract(cluster_indices);

        std::vector<pcl::PointCloud<pcl::PointXYZI>> TotalCloud; 
        pcl::PointCloud<pcl::PointXYZI> final_cloud; 


        for( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZI> tempcloud; 

            for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                pcl::PointXYZI pt = projection_cloud->points[*pit];
                pcl::PointXYZI pt2;

                pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
                // pt2.intensity = (float)(j+1);

                tempcloud.push_back(pt2);
                final_cloud.push_back(pt2);
 
            }

            TotalCloud.push_back(tempcloud);

        }

        util.selectNumber(TotalCloud, centroid_distance);


        pcl::PCLPointCloud2 cloud_clustered;
        pcl::toPCLPointCloud2(final_cloud, cloud_clustered);
        sensor_msgs::PointCloud2 output_clustered; 
        pcl_conversions::fromPCL(cloud_clustered, output_clustered);
        output_clustered.header.frame_id = "velodyne";
        pub.publish(output_clustered);
        final_cloud.clear();


        pcl::PCLPointCloud2 center_cloud;
        pcl::toPCLPointCloud2(util.outCloud, center_cloud);
        sensor_msgs::PointCloud2 center; 
        pcl_conversions::fromPCL(center_cloud, center);
        center.header.frame_id = "velodyne";
        pub2.publish(center); 
        util.outCloud.clear();

        pcl::PCLPointCloud2 kalman_predict;
        pcl::toPCLPointCloud2(util.outKalman, kalman_predict);
        sensor_msgs::PointCloud2 kalman_center; 
        pcl_conversions::fromPCL(kalman_predict, kalman_center);
        kalman_center.header.frame_id = "velodyne";
        pub3.publish(kalman_center); 
        util.outKalman.clear();

    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv,"tracking_frame");
    ros::NodeHandle nh;

    nh.getParam("cluster_value_",cluster_value);
    nh.getParam("centroid_distance_",centroid_distance);
    nh.getParam("voxel_leaf_size_", leaf_size);

    frameTracker tracker;
    ros::Subscriber sub = nh.subscribe("/Front_velo/velodyne_points",1, &frameTracker::Callback, &tracker);

    pub = nh.advertise<sensor_msgs::PointCloud2>("tracking", 1);

    pub2 = nh.advertise<sensor_msgs::PointCloud2>("center", 1);

    pub3 = nh.advertise<sensor_msgs::PointCloud2>("Kalman_predict", 1);

    ros::spin();

}