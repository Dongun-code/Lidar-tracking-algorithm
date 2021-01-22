#ifndef UTIL_HPP
#define UTIL_HPP

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

        cv::setIdentity(KFT.processNoiseCov, cv::Scalar::all(1e-4));
        cv::setIdentity(KFT.measurementNoiseCov, cv::Scalar::all(1e-1));
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
    pcl::PointCloud<pcl::PointXYZI> outCloud;
    pcl::PointCloud<pcl::PointXYZI> outKalman;
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
        pass.setFilterLimits(-3, 10);
        pass.filter(*point);
        pass.setInputCloud(point);   
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-1.45,0.7);
        pass.filter(*point);
        pass.setInputCloud(point);   
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-5,1000);
        pass.filter(*out_cloud);

        return out_cloud;
    }



    // pcl::PointXYZI selectNumber(pcl::PointCloud<pcl::PointXYZI> cloud, int mode)
    // {
    //     // pcl::PointCloud<pcl::PointXYZI>::Ptr tempPoint(new pcl::PointCloud<pcl::PointXYZI>);
    //     pcl::PointXYZI outpoint;
    //     Eigen::Vector4f centroid;
    //     // tempPoint->push_back(cloud);
    //     pcl::compute3DCentroid(cloud, centroid);

    //     std::cout<<"center point:"<<centroid<<std::endl;

    //     outpoint.x = centroid[0], outpoint.y = centroid[1], outpoint.z = centroid[2], outpoint.intensity = 5;
    //     pre_value.push_back(outpoint);
    //     std::cout<<"pre_value:"<<pre_value.size()<<std::endl;
    //     std::cout<<"pre_value:"<<pre_value.back()<<std::endl;
    //     std::cout<<"pre_value:"<<pre_value.front()<<std::endl;

    //     return outpoint;

    // }

    void selectNumber(std::vector<pcl::PointCloud<pcl::PointXYZI>> vec, double thresold)
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
            pcl::PointXYZI kalmanPoint;
            Eigen::Vector4f centroid;
            float tempSave = 0;
            float intensity = std::rand() % 15;

            pcl::compute3DCentroid(vec[cl], centroid);

            outpoint.x = centroid[0], outpoint.y = centroid[1], outpoint.z = centroid[2], outpoint.intensity = intensity;

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
                        kalmanPoint = KF.predict(tempP, outpoint);
                        std::cout<<"Intensity change!"<<pt->at(i).intensity<<std::endl;
                        intensity = pt->at(i).intensity;
                        outpoint.intensity = intensity;
                        kalmanPoint.intensity = intensity;
                        std::cout<<"kalman : "<<kalmanPoint<<std::endl;
                    }

                    if(distance <= thresold)
                    {
                        kalmanPoint = KF.predict(tempP, outpoint);
                        std::cout<<"Intensity change!"<<pt->at(i).intensity<<std::endl;
                        intensity = pt->at(i).intensity;
                        outpoint.intensity = intensity;
                        kalmanPoint.intensity = intensity;
                        std::cout<<"kalman : "<<kalmanPoint<<std::endl;
                    }

                }
                // intensityVector.push_back(intensity);
            }
            out_cloud.push_back(outpoint);
            outCloud.push_back(outpoint);
            outKalman.push_back(kalmanPoint);
            tempVector.push_back(outpoint);
            tempKalman.push_back(kalmanPoint);

        }
        // setIntensity(vec, intensityVector);
        compareVector.clear();
        kalmanVector.clear();
        compareVector.push_back(tempVector);
        kalmanVector.push_back(tempKalman);

    }



};

#endif