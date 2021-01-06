#ifndef ADD_H
#define ADD_H

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

class lidarUtil
{
private:
    std::vector<pcl::PointXYZI> pre_value;


public:
    std::vector<std::vector<pcl::PointXYZI>> compareVector;
    pcl::PointCloud<pcl::PointXYZI> outCloud;
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

        std::cout<<"--------------------compareVector"<<compareVector.size()<<std::endl;
        std::vector<pcl::PointXYZI> tempVector;
        std::vector<pcl::PointCloud<pcl::PointXYZI>>::iterator cloud;
        pcl::PointCloud<pcl::PointXYZI> out_cloud;
        std::vector<float> intensityVector;


        for(int cl = 0; cl < vec.size(); cl++)
        {
            pcl::PointXYZI outpoint;
            Eigen::Vector4f centroid;
            std::cout<<"this is test"<<std::endl;
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
                        std::cout<<"Intensity change!"<<pt->at(i).intensity<<std::endl;
                        intensity = pt->at(i).intensity;
                        outpoint.intensity = intensity;

                    }

                }


                // intensityVector.push_back(intensity);
            }
            out_cloud.push_back(outpoint);
            outCloud.push_back(outpoint);
            tempVector.push_back(outpoint);

        }
        // setIntensity(vec, intensityVector);
        compareVector.clear();
        compareVector.push_back(tempVector);

    }



};

#endif