#include "pointcloud_tools/functions.h"
#include <iostream>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr functions::transformPCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, std::string inputFrame, 
    std::string outputFrame, tf::TransformListener* tfListener)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    tf::StampedTransform transform;
    try{
        tfListener->lookupTransform(outputFrame, inputFrame,
                            ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        return inputCloud;
    }

    pcl_ros::transformPointCloud(*inputCloud, *outputCloud, transform);
    return outputCloud; 
}

pcl::PointCloud<pcl::PointXYZ>::Ptr functions::transformPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, std::string inputFrame, 
    std::string outputFrame, tf::TransformListener* tfListener)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZ>);
    tf::StampedTransform transform;
    try{
        tfListener->lookupTransform(outputFrame, inputFrame,
                            ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        return inputCloud;
    }

    pcl_ros::transformPointCloud(*inputCloud, *outputCloud, transform);
    return outputCloud; 
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr conversions::fromPC2toPCLRGB(sensor_msgs::PointCloud2 rosPc2)
{
    pcl::PCLPointCloud2 pclPc2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_conversions::toPCL(rosPc2,pclPc2);
    pcl::fromPCLPointCloud2(pclPc2,*pclCloud);
    return pclCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr conversions::fromPC2toPCL(sensor_msgs::PointCloud2 rosPc2)
{
    pcl::PCLPointCloud2 pclPc2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(rosPc2,pclPc2);
    pcl::fromPCLPointCloud2(pclPc2,*pclCloud);
    return pclCloud;
}
