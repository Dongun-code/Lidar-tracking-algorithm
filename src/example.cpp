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

ros::Publisher pub, pub2;

pcl::PointCloud<pcl::PointXYZI>::Ptr point_roi(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl_conversions::toPCL(msgs, cloud);
    pcl::PassThrough<pcl::PointXYZI> pass;
    // pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(point);    
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-10, 10);
    pass.filter(*point);
    pass.setInputCloud(point);   
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.55,0.5);
    pass.filter(*point);
    pass.setInputCloud(point);   
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-5,1000);
    pass.filter(*out_cloud);

    return out_cloud;
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

pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_point(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr output (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(point);
    sor.setLeafSize(0.3f,0.3f,0.3f);
    sor.filter(*output);

    return output;
}

std::vector<pcl::PointXYZI> pointCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point)
{
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(point);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(15);
    ec.setMaxClusterSize(150);
    ec.setSearchMethod(tree);
    ec.setInputCloud(point);
    ec.extract(cluster_indices);

    // std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
    // for(int i = 0 ; i< cluster_indices.size(); i++)
    // {

    //     std::cout<< "cluster:"<<cluster_indices[i]<<std::endl;
    // }
    // pcl::PointCloud<pcl::PointXYZI> TotalCloud; 
    std::vector<pcl::PointXYZI> TotalCloud;

    int j = 0;

    for( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        // int j = 0;
        // std::cout<<"one"<<std::endl;
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            pcl::PointCloud<pcl::PointXYZ> temp_point;
            pcl::PointXYZI pt = point->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            // pt2.intensity = (float)(1);
            // temp_point.push_back(pt2);
            // pt2.intensity = (float)(j+1);
            // centroidCompute()

            TotalCloud.push_back(pt2);
            
        }
        // std::cout<<"test:"<<TotalCloud<<std::endl;

    }

    return TotalCloud;
}


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

pcl::PointXYZI selectNumber(pcl::PointCloud<pcl::PointXYZI> cloud)
{
    // pcl::PointCloud<pcl::PointXYZI>::Ptr tempPoint(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI outpoint;
    Eigen::Vector4f centroid;
    // tempPoint->push_back(cloud);
    pcl::compute3DCentroid(cloud, centroid);

    std::cout<<"center point:"<<centroid<<std::endl;

    outpoint.x = centroid[0], outpoint.y = centroid[1], outpoint.z = centroid[2], outpoint.intensity = 5;

    return outpoint;

}


void Callback(const sensor_msgs::PointCloud2& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new  pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr projection_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msg, *cloud);

    output_cloud = point_roi(cloud);
    projection_cloud = point_projection(output_cloud);

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
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(15);
    ec.setMaxClusterSize(300);
    ec.setSearchMethod(tree);
    ec.setInputCloud(projection_cloud);
    ec.extract(cluster_indices);

    // std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
    for(int i = 0 ; i< cluster_indices.size(); i++)
    {

        // std::cout<< "cluster:"<<cluster_indices[i]<<std::endl;
    }
    pcl::PointCloud<pcl::PointXYZI>TotalCloud; 
    pcl::PointCloud<pcl::PointXYZI> centroidCloud; 
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
            pt2.intensity = (float)(j+1);
            TotalCloud.push_back(pt2);
            tempcloud.push_back(pt2);


        }
        std::cout<<"cloud"<<tempcloud.size()<<std::endl;
        centroidCloud.push_back(selectNumber(tempcloud));
        
        j++;
    }

    pcl::PCLPointCloud2 cloud_clustered;
    pcl::toPCLPointCloud2(TotalCloud, cloud_clustered);
    sensor_msgs::PointCloud2 output_clustered; 
    pcl_conversions::fromPCL(cloud_clustered, output_clustered);
    output_clustered.header.frame_id = "velodyne";
    pub.publish(output_clustered); 


    pcl::PCLPointCloud2 center_cloud;
    pcl::toPCLPointCloud2(centroidCloud, center_cloud);
    sensor_msgs::PointCloud2 center; 
    pcl_conversions::fromPCL(center_cloud, center);
    center.header.frame_id = "velodyne";
    pub2.publish(center); 
}



int main(int argc, char** argv)
{
    ros::init(argc, argv,"tracking_frame");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/Front_velo/velodyne_points",1, Callback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("tracking",1);

    pub2 = nh.advertise<sensor_msgs::PointCloud2>("center",1);

    ros::spin();

}