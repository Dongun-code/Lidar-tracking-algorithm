#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include "util.hpp"
#include <cmath>
#include <typeinfo>
#include <cstdlib>
#include <ctime>

ros::Publisher pub, pub2;


class frameTracker
{

public:

    pcl::PointCloud<pcl::PointXYZI> centroidCloud; 
    // pcl::PointCloud<pcl::PointXYZI>finalCloud; 
    pcl::PointCloud<pcl::PointXYZI> compareCloud;
    std::vector<std::vector<pcl::PointXYZI>> compareVector;
    pcl::PointCloud<pcl::PointXYZI> outCloud;
    int centroidNum = 0;
    std::vector<float> colorNumber = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
    int mode = 0;

    // pcl::PointCloud<pcl::PointXYZI>
    void selectNumber(std::vector<pcl::PointCloud<pcl::PointXYZI>> vec)
    {

        std::cout<<"--------------------compareVector"<<compareVector.size()<<std::endl;
        // pcl::PointCloud<pcl::PointXYZI>::Ptr tempPoint(new pcl::PointCloud<pcl::PointXYZI>);
        // std:;vector<pcl::PointCloud<pcl::PointXYZI>> tempVector;
        std::vector<pcl::PointXYZI> tempVector;
        std::vector<pcl::PointCloud<pcl::PointXYZI>>::iterator cloud;
        pcl::PointCloud<pcl::PointXYZI> out_cloud;
        float intensity = 0;
        
        for(cloud = vec.begin(); cloud != vec.end(); cloud++)
        {
            pcl::PointXYZI outpoint;
            Eigen::Vector4f centroid;
            std::cout<<"this is test"<<std::endl;
            float tempSave = 0;
            int random_num = std::rand() % 15;

            // tempPoint->push_back(cloud);
            pcl::compute3DCentroid(vec[0], centroid);

            // std::cout<<"center point:"<<centroid.size()<<std::endl;
            outpoint.x = centroid[0], outpoint.y = centroid[1], outpoint.z = centroid[2], outpoint.intensity = random_num;
            // tempSave = colorNumber.front();
            // colorNumber.erase(colorNumber.begin());
            // std::cout<<colorNumber.size()<<std::endl;

            if(compareVector.size() != 0)
            {
                std::vector<std::vector<pcl::PointXYZI>>::iterator pt;
                pt = compareVector.begin();
                // for(pt = compareVec)
                for(int i = 0; i < pt->size(); i++)
                {

                    pcl::PointXYZI tempP;
                    tempP.x = pt->at(i).x, tempP.y = pt->at(i).y, tempP.z = pt->at(i).z;
                    double add_x = tempP.x - outpoint.x;
                    double add_y = tempP.y - outpoint.y;
                    double distance = sqrt(pow(add_x,2)+ pow(add_y, 2));

                    // std::cout<<"distance:"<<distance<<std::endl;

                    if(distance <= 1.5)
                    {
                        std::cout<<"Intensity change!"<<pt->at(i).intensity<<std::endl;
                        intensity = pt->at(i).intensity;
                        outpoint.intensity = intensity;

                    }

                }

            }
            out_cloud.push_back(outpoint);
            outCloud.push_back(outpoint);
            tempVector.push_back(outpoint);

        }

        compareVector.clear();
        compareVector.push_back(tempVector);
        std::cout<<"out cloud size"<<out_cloud.size()<<std::endl;


        // return out_cloud;

    }

    // void setIntensity(pcl::PointCloud<pcl::PointXYZI> cloud, float num)
    // {

    //     for(int i = 0; i<= cloud.size(); i++)
    //     {   
    //         pcl::PointXYZI tempP;
    //         tempP.x = cloud.points[i].x, tempP.y = cloud.points[i].y, tempP.z = cloud.points[i].z, tempP.intensity = num;
    //         finalCloud.push_back(tempP);
    //     }

    // }


    void Callback(const sensor_msgs::PointCloud2& msg)
    {
        lidarUtil util;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new  pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr projection_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(msg, *cloud);



        output_cloud = util.point_roi(cloud);
        projection_cloud = util.point_projection(output_cloud);

            // std::cout<<"Input: "<<cloud->points.size()<<" ( "<<pcl::getFieldsList(*cloud)<<")"<<std::endl;
        //  voxel
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(projection_cloud);
        sor.setLeafSize(0.3f,0.3f,0.3f);
        sor.filter(*projection_cloud);

        // std::cout << "Output : " << cloud_filtered->points.size () << " (" << pcl::getFieldsList (*cloud_filtered) <<")"<< std::endl;

        //Create  object
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(projection_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.6);
        ec.setMinClusterSize(15);
        ec.setMaxClusterSize(300);
        ec.setSearchMethod(tree);
        ec.setInputCloud(projection_cloud);
        ec.extract(cluster_indices);

        std::vector<pcl::PointCloud<pcl::PointXYZI>> TotalCloud; 
        pcl::PointCloud<pcl::PointXYZI> centerPoint; 
        pcl::PointCloud<pcl::PointXYZI> final_cloud; 
        std::vector<pcl::PointCloud<pcl::PointXYZI>> totalvector;
        int j = 0;

        for( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZI> tempcloud; 

            for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                pcl::PointXYZI pt = projection_cloud->points[*pit];
                pcl::PointXYZI pt2;
                // std::cout<<"value:"<<pt<<std::endl;
                // std::cout<<"------------------"<<std::endl;
                pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
                // pt2.intensity = (float)(j+1);

                tempcloud.push_back(pt2);
                final_cloud.push_back(pt2);
 
            }

            TotalCloud.push_back(tempcloud);

        }

        // std::pair<pcl::PointCloud<pcl::PointXYZI>, float> result = selectNumber(TotalCloud);
        selectNumber(TotalCloud);

        pcl::PCLPointCloud2 cloud_clustered;
        pcl::toPCLPointCloud2(final_cloud, cloud_clustered);
        sensor_msgs::PointCloud2 output_clustered; 
        pcl_conversions::fromPCL(cloud_clustered, output_clustered);
        output_clustered.header.frame_id = "velodyne";
        pub.publish(output_clustered); 


        pcl::PCLPointCloud2 center_cloud;
        pcl::toPCLPointCloud2(outCloud, center_cloud);
        sensor_msgs::PointCloud2 center; 
        pcl_conversions::fromPCL(center_cloud, center);
        center.header.frame_id = "velodyne";
        pub2.publish(center); 
        outCloud.clear();

        // mode++;

        // if(mode == 2)
        // {
        //     std::cout<<"clear!!!!"<<std::endl;
        //     centroidCloud.clear();
        //     mode = 0;
        // }

    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv,"tracking_frame");
    ros::NodeHandle nh;

    frameTracker tracker;
    ros::Subscriber sub = nh.subscribe("/Front_velo/velodyne_points",1, &frameTracker::Callback,&tracker);

    pub = nh.advertise<sensor_msgs::PointCloud2>("tracking", 1);

    pub2 = nh.advertise<sensor_msgs::PointCloud2>("center", 1);

    ros::spin();

}