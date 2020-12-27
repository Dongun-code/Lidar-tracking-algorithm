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
    pcl::PointCloud<pcl::PointXYZI>finalCloud; 
    pcl::PointCloud<pcl::PointXYZI> compareCloud;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> compareVector;
    int centroidNum = 0;
    std::vector<float> colorNumber = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
    int mode = 0;

    // std::pair<pcl::PointXYZI, float> selectNumber(pcl::PointCloud<pcl::PointXYZI> cloud, float intensity_)
    // {
    //     // pcl::PointCloud<pcl::PointXYZI>::Ptr tempPoint(new pcl::PointCloud<pcl::PointXYZI>);
    //     pcl::PointXYZI outpoint;
    //     Eigen::Vector4f centroid;
    //     float intensity = 0;
    //     // tempPoint->push_back(cloud);
    //     pcl::compute3DCentroid(cloud, centroid);

    //     // std::cout<<"center point:"<<centroid.size()<<std::endl;

    //     outpoint.x = centroid[0], outpoint.y = centroid[1], outpoint.z = centroid[2], outpoint.intensity = intensity_;
    //     if(compareCloud.size() != 0)
    //     {
    //         for(int i = 0; i<= compareCloud.size(); i++)
    //         {
    //             pcl::PointXYZ tempP;
    //             tempP.x = compareCloud.points[i].x, tempP.y = compareCloud.points[i].y, tempP.z = compareCloud.points[i].z;
    //             double add_x = tempP.x - outpoint.x;
    //             double add_y = tempP.y - outpoint.y;
    //             // double add_z = tempP.z - outpoint.z; 
    //             // double distance = sqrt(pow(add_x,2)+ pow(add_y, 2) + pow(add_z, 2));
    //             double distance = sqrt(pow(add_x,2)+ pow(add_y, 2));

    //             // std::cout<<"distance:"<<distance<<std::endl;
    //             if(distance <= 5)
    //             {
    //                 // std::cout<<"Intensity change!"<<centroidCloud.points[i].intensity<<std::endl;
    //                 intensity = compareCloud.points[i].intensity;
    //             }

    //         }

    //         deletePrecentroid();
    //     }

    //     // pre_value.push_back(outpoint);
    //     return {outpoint, intensity};

    // }


    std::pair<pcl::PointXYZI, float> selectNumber(pcl::PointCloud<pcl::PointXYZI> cloud, int intensity_)
    {
        std::cout<<"--------------------compareVector"<<compareVector.size()<<std::endl;
        // pcl::PointCloud<pcl::PointXYZI>::Ptr tempPoint(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointXYZI outpoint;
        Eigen::Vector4f centroid;
        int intensity = 0;
        float tempSave = 0;
        std::srand(static_cast<unsigned int>(std::time(0)));
        int random_num = std::rand() % 50;

        // tempPoint->push_back(cloud);
        pcl::compute3DCentroid(cloud, centroid);

        // std::cout<<"center point:"<<centroid.size()<<std::endl;
        outpoint.x = centroid[0], outpoint.y = centroid[1], outpoint.z = centroid[2], outpoint.intensity = random_num;
        // tempSave = colorNumber.front();
        // colorNumber.erase(colorNumber.begin());
        // std::cout<<colorNumber.size()<<std::endl;

        if(compareVector.size() != 0)
        {
            // for( std::vector<pcl::PointCloud<pcl::PointXYZI>>:: iterator pt = compareVector.begin(); pt != compareVector.end(); ++pt)
            // std::cout<<compareVector.back().size()<<std::endl;
            for(int i = 0; i<= compareVector.front().points.size(); i++)
            {
                // std::cout<<point<<std::endl;
                // std::cout<<compareVector.size()<<std::endl;
                // std::cout<<"--------------------"<<std::endl;
                pcl::PointXYZ tempP;
                tempP.x = compareVector.front().points[i].x, tempP.y = compareVector.front().points[i].y, tempP.z = compareVector.front().points[i].z;
                double add_x = tempP.x - outpoint.x;
                double add_y = tempP.y - outpoint.y;
                // double add_z = tempP.z - outpoint.z; 
                // double distance = sqrt(pow(add_x,2)+ pow(add_y, 2) + pow(add_z, 2));
                double distance = sqrt(pow(add_x,2)+ pow(add_y, 2));
                std::cout<<"distance:"<<distance<<std::endl;

                if(distance <= 1.5)
                {
                    std::cout<<"Intensity change!"<<(int)compareVector.front().points[i].intensity<<std::endl;
                    intensity = (int)compareVector.front().points[i].intensity;
                    outpoint.intensity = (int)compareVector.front().points[i].intensity;
                    // colorNumber.push_back(tempSave);

                }

            }

        
            compareVector.erase(compareVector.begin());
            // compareVector.clear();
        }

        std::cout<<"--------------------"<<std::endl;
        // pre_value.push_back(outpoint);
        return {outpoint, intensity};

    }

    void setIntensity(pcl::PointCloud<pcl::PointXYZI> cloud, float num)
    {

        for(int i = 0; i<= cloud.size(); i++)
        {   
            pcl::PointXYZI tempP;
            tempP.x = cloud.points[i].x, tempP.y = cloud.points[i].y, tempP.z = cloud.points[i].z, tempP.intensity = num;
            finalCloud.push_back(tempP);
        }

    }

    void test()
    {
        std::cout<<compareVector.size()<<std::endl;
        for(int i = 0; i < compareVector.size(); i++)
        {
            for(int j = 0; j <compareVector[i].points.size(); j++)
            {
                std::cout<<"vector size test:"<<compareVector[i].points[j].x<<", "<<compareVector[i].points[j].y<<std::endl;
            }
        }
            // std::cout<<"vector size test:"<<compareVector[i].points.size()<<std::endl;

    }


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

        // std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
        // for(int i = 0 ; i< cluster_indices.size(); i++)
        // {

        //     // std::cout<< "cluster:"<<cluster_indices[i]<<std::endl;
        // }

        pcl::PointCloud<pcl::PointXYZI>TotalCloud; 
        pcl::PointCloud<pcl::PointXYZI> centerPoint; 
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
                TotalCloud.push_back(pt2);
                tempcloud.push_back(pt2);
            }

            // std::cout<<"cloud"<<tempcloud.size()<<std::endl;
            // centroidCloud.push_back(selectNumber(tempcloud));
            std::pair<pcl::PointXYZI, float> result = selectNumber(tempcloud, (j+1));
            centerPoint.push_back(result.first);
            std::cout<<"centerpoint:"<<centerPoint.size()<<std::endl;
            compareVector.push_back(centerPoint);
            // centroidCloud.push_back(result.first);
            // compareVector.push_back(centroidCloud);

            // centroidCloud.clear();
            setIntensity(tempcloud,result.second);
            j++;
        }
        // compareVector.push_back(centerPoint);
        // test();
        // std::cout<<"compare Vector value"<<compareVector.size()<<std::endl;
        // std::cout<<"comareVecotr value:"<<compareVector.back().points[0].x<<","<<compareVector.back().points[0].y<<","<<compareVector.back().points[0].z<<std::endl;
        // std::cout<<"comareVecotr value:"<<compareVector.front().points[0].x<<","<<compareVector.front().points[0].y<<","<<compareVector.front().points[0].z<<std::endl;
        // std::cout<<"centroidCloud:"<<centroidCloud.size()<<std::endl;

        pcl::PCLPointCloud2 cloud_clustered;
        pcl::toPCLPointCloud2(finalCloud, cloud_clustered);
        finalCloud.clear();
        sensor_msgs::PointCloud2 output_clustered; 
        pcl_conversions::fromPCL(cloud_clustered, output_clustered);
        output_clustered.header.frame_id = "velodyne";
        pub.publish(output_clustered); 


        pcl::PCLPointCloud2 center_cloud;
        pcl::toPCLPointCloud2(centerPoint, center_cloud);
        sensor_msgs::PointCloud2 center; 
        pcl_conversions::fromPCL(center_cloud, center);
        center.header.frame_id = "velodyne";
        pub2.publish(center); 

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