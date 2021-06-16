#ifndef FUNCTIONS_H
#define FUNCTIONS_H


#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include "pcl_conversions/pcl_conversions.h"

class functions
{
public:
    //transformPCL: Returns pointcloud transformed from input frame to output frame
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, std::string inputFrame, 
        std::string outputFrame, tf::TransformListener* tfListener);

    static pcl::PointCloud<pcl::PointXYZ>::Ptr transformPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, std::string inputFrame, 
        std::string outputFrame, tf::TransformListener* tfListener);
};

class conversions
{
public:
    //fromPCLtoPC2: Conversion from sensor_msgs::Pointcloud2 to pcl::PointCloud
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr fromPC2toPCLRGB(sensor_msgs::PointCloud2 rosPc2);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr fromPC2toPCL(sensor_msgs::PointCloud2 rosPc2);
};

#endif 