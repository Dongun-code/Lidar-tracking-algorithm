#ifndef UTIL_HPP
#define UTIL_HPP

// #include <ros/ros.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
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
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <utility>
// #include "kalman_hand.hpp"

class KalmanFilter_hand {

public:

    std::vector<pcl::PointXYZI> preMean;
    float T = 0.1;

    KalmanFilter_hand() { }

    ~KalmanFilter_hand() { }


    pcl::PointXYZI predict(const pcl::PointXYZI& pre_state, const pcl::PointXYZI& cur_state) {

        cv::KalmanFilter KFT( 4,2,0);
        pcl::PointXYZI outpoint;
        // cv::KalmanFilter KF( 4,2,0);
        KFT.transitionMatrix = (cv::Mat_<float>(4,4) << 1,0 , T, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0,
                                            0, 0, 0, 1);
        // preMean = centroid;
        KFT.measurementMatrix = cv::Mat::zeros(2, 4, CV_32F);
        KFT.measurementMatrix.at<float>(0) = 1.0f;
        KFT.measurementMatrix.at<float>(5) = 1.0f;        

        cv::setIdentity(KFT.processNoiseCov, cv::Scalar::all(0.01));
        cv::setIdentity(KFT.measurementNoiseCov, cv::Scalar::all(0.1));
        cv::setIdentity(KFT.errorCovPost, cv::Scalar::all(5));

        KFT.statePre.at<float>(0) = pre_state.x;
        KFT.statePre.at<float>(1) = pre_state.y;
        KFT.statePre.at<float>(2) = 0;
        KFT.statePre.at<float>(3) = 0;

        // cv::Mat_<float> measure(4,1); 
        cv::Mat measure(2, 1, CV_32F);
        measure.at<float>(0) = cur_state.x;
        measure.at<float>(1) = cur_state.y;
        // measure << cur_state.x, cur_state.y;
        // std::cout<<"check measure"<< measure<<std::endl;
        KFT.predict();
        auto estimated = KFT.correct(measure);
        std::cout<<estimated<<std::endl;
        outpoint.x = estimated.at<float>(0) , outpoint.y = estimated.at<float>(1), outpoint.z = 0;

        return outpoint;

    }

};



class lidarUtil
{

private:
    std::vector<pcl::PointXYZI> pre_value;

public:

    std::vector<std::vector<pcl::PointXYZI>> compareVector;
    std::vector<std::vector<pcl::PointXYZI>> kalmanVector;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> outMinMax;
    pcl::PointCloud<pcl::PointXYZI> outCloud;
    pcl::PointCloud<pcl::PointXYZI> outKalman;
    std::vector<double> distanceVector;

    lidarUtil() {};

    pcl::PointCloud<pcl::PointXYZI>::Ptr segmentPoint(std::vector<pcl::PointXYZI>& point)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr outPoint(new pcl::PointCloud<pcl::PointXYZI>);
        float j = 0;
        for(std::vector<pcl::PointXYZI>::iterator it = point.begin(); it != point.end(); ++ it)
        {
            it->intensity = (float)(j+1);
            outPoint->push_back(*it);
            j++;
        }

        return outPoint;
    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr point_projection(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point)
    {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

        coefficients->values.resize(4);
        coefficients->values[0] = coefficients->values[1] = 0;
        coefficients->values[2] = 1.0;
        coefficients->values[3] = 0;
        
        pcl::ProjectInliers<pcl::PointXYZI> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(point);
        proj.setModelCoefficients(coefficients);
        proj.filter(*cloud_projected);

        return cloud_projected;    
    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr point_roi(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        // pcl_conversions::toPCL(msgs, cloud);
        pcl::PassThrough<pcl::PointXYZI> pass;
        // pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(point);    
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-15, 15);
        pass.filter(*point);
        pass.setInputCloud(point);   
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.2,1.5);
        pass.filter(*point);
        pass.setInputCloud(point);   
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-100,100);
        pass.filter(*out_cloud);

        return out_cloud;
    }

    // visualization_msgs::Marker mark_centroid(std_msgs::Header header, Eigen::Vector4f centroid, Eigen::Vector4f min, Eigen::Vector4f max, std::string ns , double distance_,float area, int id, float r, float g, float b)
    // {
    //     uint32_t shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = "velodyne";
    //     marker.header.stamp = ros::Time();

    //     marker.ns = ns;
    //     marker.id = id;
    //     marker.type = shape;
    //     marker.action = visualization_msgs::Marker::ADD;
        
    //     marker.pose.position.x = centroid[0];
    //     marker.pose.position.y = centroid[1];
    //     marker.pose.position.z = centroid[2];
    //     marker.pose.orientation.x = 0.0;
    //     marker.pose.orientation.y = 0.0;
    //     marker.pose.orientation.z = 0.0;
    //     marker.pose.orientation.w = 1.0;

    //     // std::string distance_text = "";

    //     marker.text = "Area";
    //     marker.scale.x = (max[0]-min[0]);
    //     marker.scale.y = (max[1]-min[1]);
    //     marker.scale.z = 1.0;
    //     // marker.scale.x = 1;
    //     // marker.scale.y = 2;
    //     // marker.scale.z = 1;
        
    //     // if (marker.scale.x ==0)
    //     //     marker.scale.x=0.1;

    //     // if (marker.scale.y ==0)
    //     //     marker.scale.y=0.1;

    //     // if (marker.scale.z ==0)
    //     //     marker.scale.z=0.1;
        
    //     // marker.color.r = 0.0f;
    //     // marker.color.g = 1.0f;
    //     // marker.color.b = 0.0f;
    //     marker.color.a = 1;

    //     marker.lifetime = ros::Duration(0.2);
    //     return marker;
    // }

    // visualization_msgs::MarkerArray visualFunction(std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud , pcl::PointCloud<pcl::PointXYZI> center, std::vector<double> distance, std::vector<float> area) {

    //     visualization_msgs::MarkerArray markerArr;

    //     for(int i = 0; i< cloud.size(); i++) {

    //         Eigen::Vector4f min;
    //         Eigen::Vector4f max;
    //         Eigen::Vector4f centroid;

    //         pcl::getMinMax3D(cloud[i], min, max);
    //         centroid<< center.at(i).x , center.at(i).y,0,0;

    //         visualization_msgs::Marker marker;

    //         marker.header.frame_id = "/velodyne";
    //         marker.header.stamp = ros::Time::now();
    //         marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    //         marker.action = visualization_msgs::Marker::ADD;
    //         marker.color.r = 0.0f;
    //         marker.color.g = 1.0f;
    //         marker.color.b = 0.0f;
    //         marker.color.a = 1.0;
    //         marker.id = i;
    //         marker.scale.z = 1.0;
    //         marker.pose.orientation.w = 1.0;
    //         // marker.text = "Area";
    //         marker.text = std::to_string(area.at(i));
    //         marker.pose.position.x = centroid[0];
    //         marker.pose.position.y = centroid[1];
    //         marker.lifetime = ros::Duration(0.2);
    //         markerArr.markers.push_back(marker);
    //     }
    //         return markerArr;
    // }

    visualization_msgs::MarkerArray visualFunction(std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud , std::vector<double> distance, std::vector<float> area) {

        visualization_msgs::MarkerArray markerArr;

        for(int i = 0; i< cloud.size(); i++) {

            Eigen::Vector4f min;
            Eigen::Vector4f max;
            Eigen::Vector4f centroid;

            pcl::getMinMax3D(cloud[i], min, max);
            pcl::compute3DCentroid(cloud[i], centroid);

            visualization_msgs::Marker marker;

            marker.header.frame_id = "/velodyne";
            marker.header.stamp = ros::Time::now();
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
            marker.id = i;
            marker.scale.z = 1.0;
            marker.pose.orientation.w = 1.0;
            // marker.text = "Area";
            // std::string st = "Area : ";
            // std::string st2 = " Distance : "
            // std::string area = std::to_string(area.at(i));
            // std::string distance = std::to_string(distance.at(i));
            // std::string total = st+area+st2+distance;
            // marker.text = std::to_string(area.at(i));
            marker.text = std::to_string(area.at(i));
            marker.pose.position.x = centroid[0];
            marker.pose.position.y = centroid[1];
            marker.lifetime = ros::Duration(0.2);
            markerArr.markers.push_back(marker);
        }


        return markerArr;
    }




    visualization_msgs::Marker mark_centroid(std_msgs::Header header, Eigen::Vector4f centroid, Eigen::Vector4f min, Eigen::Vector4f max, std::string ns , double distance_,float area, int id, float r, float g, float b)
    {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "/velodyne";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        
        // marker.scale.x = 3.0;
        // marker.scale.y = 3.0;
        marker.scale.z = 1.0;
        marker.pose.orientation.w = 1.0;
        // marker.text = "Area";
        marker.text = std::to_string(area);
        marker.pose.position.x = centroid[0];
        marker.pose.position.y = centroid[1];

        marker.lifetime = ros::Duration(0.2);
        return marker;
    }



    std::vector<float> selectNumber(std::vector<pcl::PointCloud<pcl::PointXYZI>>& vec, double thresold)
    {
        
        KalmanFilter_hand KF;
        // std::cout<<"--------------------compareVector"<<compareVector.size()<<std::endl;
        std::vector<pcl::PointXYZI> tempVector;
        std::vector<pcl::PointXYZI> tempKalman;
        std::vector<pcl::PointCloud<pcl::PointXYZI>>::iterator cloud;
        pcl::PointCloud<pcl::PointXYZI> out_cloud;

        std::vector<float> intensityVector;

        for(int cl = 0; cl < vec.size(); cl++)
        {

            pcl::PointXYZI outpoint;
            pcl::PointXYZI new_outpoint;
            pcl::PointXYZI kalmanPoint;
            pcl::PointXYZI total_point;
            Eigen::Vector4f centroid;
            float tempSave = 0;
            float intensity = std::rand() % 30;

            pcl::compute3DCentroid(vec[cl], centroid);
            

            outpoint.x = centroid[0], outpoint.y = centroid[1], outpoint.z = centroid[2], outpoint.intensity = intensity;
            total_point.x = centroid[0], total_point.y = centroid[1], total_point.z = centroid[2], total_point.intensity = intensity;
            
            if(compareVector.size() != 0)
            {
                std::vector<std::vector<pcl::PointXYZI>>::iterator pt;
                pt = compareVector.begin();

                for(int i = 0; i < pt->size(); i++)
                {

                    pcl::PointXYZI tempP;
                    tempP.x = pt->at(i).x, tempP.y = pt->at(i).y, tempP.z = pt->at(i).z;
                    double add_x = tempP.x - outpoint.x;
                    double add_y = tempP.y - outpoint.y;
                    double distance = sqrt(pow(add_x,2)+ pow(add_y, 2));

                    std::cout<<"distance:"<<distance<<std::endl;

                    if(distance <= thresold)
                    {
                        new_outpoint = KF.predict(tempP, outpoint);
                        outpoint.x = new_outpoint.x, outpoint.y = new_outpoint.y, new_outpoint.z = 0;
                        std::cout<<"Intensity change!"<<pt->at(i).intensity<<std::endl;
                        intensity = pt->at(i).intensity;
                        outpoint.intensity = intensity;
                        new_outpoint.intensity = intensity;
                        outKalman.push_back(new_outpoint);
                        outMinMax.push_back(vec[cl]);
                        // drawMarker(vec[cl], new_outpoint, intensity);
                        // kalmanPoint.intensity = intensity;
                        // std::cout<<"kalman : "<<kalmanPoint<<std::endl;
                    }

                }
            }

            out_cloud.push_back(outpoint);
            outCloud.push_back(outpoint);
            // outKalman.push_back(kalmanPoint);
            tempVector.push_back(outpoint);
            // tempKalman.push_back(kalmanPoint);
            intensityVector.push_back(intensity);

        }

        compareVector.clear();
        compareVector.push_back(tempVector);

        return intensityVector;
    }

    pcl::PointCloud<pcl::PointXYZI> paintColor(std::vector<pcl::PointCloud<pcl::PointXYZI>>& vec, std::vector<float> intensity) {

        pcl::PointCloud<pcl::PointXYZI> clusterCloud;
        
        for(int i = 0; i < vec.size(); i++) {

            auto point = vec.at(i);
            
            for(int pt = 0; pt <point.size(); pt++ ) {
                pcl::PointXYZI tempCloud;
                tempCloud.x = point.points[pt].x, tempCloud.y = point.points[pt].y, tempCloud.z = point.points[pt].z;
                tempCloud.intensity = intensity.at(i);
                clusterCloud.push_back(tempCloud);
            }
        }
        
        return clusterCloud;
    }

    float extractDistance() {

        for( auto point : outCloud) {

            double distance = pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2);
            distance = sqrt(distance);

            distanceVector.push_back(distance);
        }
    }

    std::pair<std::vector<pcl::PointCloud<pcl::PointXYZI>>, std::vector<double>> exceptPoint(std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud , std::vector<double> distance) {

        std::vector<pcl::PointCloud<pcl::PointXYZI>> outcloud;
        std::vector<double> outdistance;

        for(int i = 0; i < cloud.size(); i++) {
            
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(cloud[i], centroid);


            if(!((centroid[0] < 0.3) && (centroid[0] > -4.7) && (centroid[1] < 0.3) && (centroid[1] > - 3.2))) {

                outcloud.push_back(cloud[i]);
                outdistance.push_back(distance[i]);
            }

        }

        return std::make_pair(outcloud, outdistance);
    }

};

#endif