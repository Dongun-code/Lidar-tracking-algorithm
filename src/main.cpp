#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "util.hpp"
#include "frame_tracking/pointInformation.h"
#include "frame_tracking/pointInformationarray.h"
// #include "kalman_hand.hpp"

ros::Publisher pub, pub2, pub3, pub_vis, pub_information, pub_axis_test;

// double cluster_value, centroid_distance, leaf_size;
double cluster_value = 0.5;
double centroid_distance = 1.6;
double leaf_size = 0.3;


class frameTracker
{
public:
    lidarUtil util;
    pcl::PointCloud<pcl::PointXYZI> outCloud;
    pcl::PointCloud<pcl::PointXYZI> showMaxp;
    pcl::PointCloud<pcl::PointXYZI> showMinp;
    std::vector<float> area_vector;


    void extractArea(std::vector<pcl::PointCloud<pcl::PointXYZI>>& vec) {

        for(int i = 0; i < vec.size(); i++ ) {
            
            Eigen::Vector4f Max;
            Eigen::Vector4f Min;
            Eigen::Vector4f center;
            pcl::PointXYZI temp;
            pcl::PointXYZI temp2;

            pcl::getMinMax3D(vec[i], Min, Max);

            temp.x = Max[0], temp.y = Max[1], temp.z = 0;
            temp2.x = Min[0], temp2.y = Min[1], temp2.z = 0;
            center[0] = Min[0], center[1] = Max[1], center[2] = 0;

            //  compute obstacle Area
            float pre_length1 = pow((Max[0] - center[0]) + (Max[1] - center[1]), 2);
            float pre_length2 = pow((center[0] - Min[0]) + (center[1] - Min[1]), 2);

            float length1 = std::sqrt(pre_length1);
            float length2 = std::sqrt(pre_length2);

            float Area = length1 * length2;

            area_vector.push_back(Area);
        }
    }


    // void drawMarker(pcl::PointCloud<pcl::PointXYZI>& cloud , pcl::PointXYZI& centroid, double distance, float area, int id) {

    //     Eigen::Vector4f center;
    //     Eigen::Vector4f min;
    //     Eigen::Vector4f max;

    //     center << centroid.x, centroid.y, 0, 0;        
    //     pcl::getMinMax3D(cloud, min, max);
    //     // std::cout<<"min"<<min<<"Max:"<<max<<std::endl;
    //     pub_vis.publish(util.mark_centroid(pcl_conversions::fromPCL(cloud.header), center, min, max, "velodyne", distance, id, area, 0, 255, 0));
        
    // }


    void publishClusterInformation(pcl::PointXYZI& centroid_, float distance_) {

        frame_tracking::pointInformation data;
        frame_tracking::pointInformationarray msg;

        data.x = centroid_.x, data.y = centroid_.y, data.distance = distance_;
        msg.points.push_back(data);

        pub_information.publish(msg);
    }


    void Callback(const sensor_msgs::PointCloud2& msg)
    {
        //set variable and convert ros msg
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new  pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr projection_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        visualization_msgs::MarkerArray arr;
        
        // msg -> 연산을 위한 pointcloud로 변환
        pcl::fromROSMsg(msg, *cloud);


        //Point roi & Point projection
        // 영역 지정하여 지면 제거 및 관심 영역 제한하여 추출
        output_cloud = util.point_roi(cloud);
        // 3차원 데이터 2차원 데이터로 projection 진행
        projection_cloud = util.point_projection(output_cloud);

            // std::cout<<"Input: "<<cloud->points.size()<<" ( "<<pcl::getFieldsList(*cloud)<<")"<<std::endl;

        //  voxel화 적용 
        // Voxel화는 연산 효율을 위해 포인트들을 줄이고 포인트들을 Voxel화 시키기 위해 사용
        // Voxel이 무엇인지 모르겠으면 인터넷으로 찾오보기
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(projection_cloud);
        sor.setLeafSize(0.3f,0.3f,0.3f);
        sor.setLeafSize(leaf_size,leaf_size,leaf_size);
        sor.filter(*projection_cloud);

        // 그냥 pointcloud에 있는 point 개수 확인하기 위한 코드
        // PointCloud는 Vector같은 존재이고 내부에 존재하는 points는 vector에 push_back된 원소라고 생각하면됨
        // 그리고 PointCloud 자체는 Class 인스턴스라고 생각하면됨
        // std::cout << "Output : " << cloud_filtered->points.size () << " (" << pcl::getFieldsList (*cloud_filtered) <<")"<< std::endl;

        // Create  object
        // CLuster 할때 사용할 Method 설정
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(projection_cloud);

       // * cluster로 추출된 point에 해당하는 index  저장하기 위한 vector
        std::vector<pcl::PointIndices> cluster_indices;

        //  EuclideanCluster를 사용하여 포인터 클러스터 추출

        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        // cluster_value값 넣기
        ec.setClusterTolerance(cluster_value);
        // set min max는 클러스터로 인정하는 최소 최대 포인터 집합개수를 의미
        ec.setMinClusterSize(15);
        ec.setMaxClusterSize(300);
        // Method 넣기
        ec.setSearchMethod(tree);
        // pointcloud 넣고 추출하여 cluster_indices에 저장
        ec.setInputCloud(projection_cloud);
        ec.extract(cluster_indices);

        std::vector<pcl::PointCloud<pcl::PointXYZI>> TotalCloud; 
        std::vector<float> intensityVector;
        pcl::PointCloud<pcl::PointXYZI> final_cloud; 
        
        // * 위에서 추출한 index를 활용하여 
        // index에 해당하는 point값들을 추출하여 새롭게 만들어진 pointcloud에 저장하는 방법
        // 이건 구현자 맘대로 개발하면됨
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

        // 추출된 cluster간에 비교하는 함수
        // 이전 프레임과 현재 프레임에 존재하는 cluster를 비교하여 같은 클러스터인지 비교하는 기능
        intensityVector = util.selectNumber(TotalCloud, centroid_distance);
        // 각 클러스터 간의 거리를 추출하는 함수
        util.extractDistance();
        // 각 클러스터의 넓이를 구하는 함수
        extractArea(TotalCloud);


        // 밑의 코드들은 그냥 pointcloud2 msg로 변환해서 publish해주는 함수
        // 그냥 따라하면됨
        pcl::PCLPointCloud2 cloud_clustered;
        pcl::toPCLPointCloud2(util.paintColor(TotalCloud, intensityVector), cloud_clustered);
        sensor_msgs::PointCloud2 output_clustered; 
        pcl_conversions::fromPCL(cloud_clustered, output_clustered);
        output_clustered.header.frame_id = "velodyne";
        pub.publish(output_clustered);


        pcl::PCLPointCloud2 center_cloud;
        pcl::toPCLPointCloud2(util.outCloud, center_cloud);
        sensor_msgs::PointCloud2 center; 
        pcl_conversions::fromPCL(center_cloud, center);
        center.header.frame_id = "velodyne";
        pub2.publish(center); 


        pcl::PCLPointCloud2 kalman_predict;
        pcl::toPCLPointCloud2(util.outKalman, kalman_predict);
        sensor_msgs::PointCloud2 kalman_center; 
        pcl_conversions::fromPCL(kalman_predict, kalman_center);
        kalman_center.header.frame_id = "velodyne";
        pub3.publish(kalman_center); 


        // 여기는 marker로 표현하려고 만든거여서 딱히 쓸필요 없음
        // std::pair<std::vector<pcl::PointCloud<pcl::PointXYZI>>, std::vector<double>> point_distance_data;

        // point_distance_data =  util.exceptPoint(TotalCloud, util.distanceVector);
        // arr = util.visualFunction(point_distance_data.first, point_distance_data.second, area_vector);
        // arr = util.visualFunction(TotalCloud , util.outCloud, util.distanceVector, area_vector);
        // pub_vis.publish(arr);

        //  variable clear
        // 메모리 너무 많이 쓰지 않게 각 변수들 초기화 해주는 곳
        util.outCloud.clear();
        util.outKalman.clear();
        util.distanceVector.clear();
        showMaxp.clear();
        showMinp.clear();
        area_vector.clear();
        final_cloud.clear();

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
    ros::Subscriber sub = nh.subscribe("/Combined_points",1, &frameTracker::Callback, &tracker);

    pub = nh.advertise<sensor_msgs::PointCloud2>("tracking", 1);

    pub2 = nh.advertise<sensor_msgs::PointCloud2>("center", 1);

    pub3 = nh.advertise<sensor_msgs::PointCloud2>("Kalman_predict", 1);

    pub_vis = nh.advertise<visualization_msgs::MarkerArray>("Area_maker_array", 1);

    pub_information = nh.advertise<frame_tracking::pointInformationarray>("axis_distance_msg", 1);

    ros::spin();

}