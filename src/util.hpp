#ifndef ADD_H
#define ADD_H

#include <iostream>
#include <pcl/point_types.h>

class lidarUtil
{
private:
    std::vector<pcl::PointXYZI> pre_value;


public:
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



    pcl::PointXYZI selectNumber(pcl::PointCloud<pcl::PointXYZI> cloud, int mode)
    {
        // pcl::PointCloud<pcl::PointXYZI>::Ptr tempPoint(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointXYZI outpoint;
        Eigen::Vector4f centroid;
        // tempPoint->push_back(cloud);
        pcl::compute3DCentroid(cloud, centroid);

        std::cout<<"center point:"<<centroid<<std::endl;

        outpoint.x = centroid[0], outpoint.y = centroid[1], outpoint.z = centroid[2], outpoint.intensity = 5;
        pre_value.push_back(outpoint);
        std::cout<<"pre_value:"<<pre_value.size()<<std::endl;
        std::cout<<"pre_value:"<<pre_value.back()<<std::endl;
        std::cout<<"pre_value:"<<pre_value.front()<<std::endl;

        return outpoint;

    }



};

#endif