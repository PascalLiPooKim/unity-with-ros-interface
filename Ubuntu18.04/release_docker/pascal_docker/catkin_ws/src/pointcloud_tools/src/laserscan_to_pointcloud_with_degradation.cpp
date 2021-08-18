/**
 * Script to collect data from LiDAR on Husky and convert the raw data into pointcloud.
 * The script also simulates deteriorating visual LiDAR.
 */


#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "laser_geometry/laser_geometry.h"
#include <tf/transform_listener.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_ros/transforms.h>
#include "pointcloud_tools/functions.h"

// Add additional libraries for KdTree
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>

#include <algorithm>
#include "std_msgs/Float32.h"
#include <math.h>  

#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>

#include <std_msgs/Bool.h>
#include <cstdlib> 
#include <ctime> 


bool degradeVisual = false;

class LaserToPointcloud{
    private:
    tf::TransformListener* tfListener;
    std::string scanSubTopic;
    std::string pclPubTopic;
    ros::Subscriber scanSub;
    ros::Publisher pclPub;

    ros::Subscriber degradeSub;

    sensor_msgs::PointCloud2 pc2Msg;
    int updateRate;
    std::string frame;
    std_msgs::Header outputHeader;


    public:
    LaserToPointcloud(ros::NodeHandle *nh){
        tfListener = new tf::TransformListener();
        nh->param<std::string>("pcl_topic", pclPubTopic, "/scan/pointcloud");
        nh->param<std::string>("scan_topic", scanSubTopic, "/scan");
	    nh->param<int>("update_rate", updateRate, 30);
        nh->getParam("frame", frame);
        scanSub = nh->subscribe(scanSubTopic, 1, &LaserToPointcloud::ScanCallback, this);
        pclPub = nh->advertise<sensor_msgs::PointCloud2>(pclPubTopic, 1);

        // Subscriber to message sent from keyboard key pressed
        degradeSub = nh->subscribe("/degrade_pointcloud", 1, &LaserToPointcloud::degradeCallback, this);
        

    }

    // Callback function to store bool state
    void degradeCallback(const std_msgs::Bool& msg){
        ros::Rate loop_rate(updateRate);
        degradeVisual = msg.data;

        loop_rate.sleep();
    }



    void ScanCallback(const sensor_msgs::LaserScan& msg){
	ros::Rate loop_rate(updateRate);
        laser_geometry::LaserProjection projector;
        projector.projectLaser(msg, pc2Msg);
        
        // Convert pc2 message to pcl
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutput = conversions::fromPC2toPCL(pc2Msg);
        outputHeader = msg.header;

        // Convert from sensor frame to Husky base frame
        if(frame.length() > 0){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTransformed (new pcl::PointCloud<pcl::PointXYZ>);
            cloudTransformed = functions::transformPCL(cloudOutput, outputHeader.frame_id, frame, tfListener);
            outputHeader.frame_id = frame;
            cloudOutput = cloudTransformed;
        }

        pcl_conversions::toPCL(outputHeader, cloudOutput->header);

        std::srand(static_cast<unsigned int>(std::time(nullptr)));

        // Degrade LiDAR pointcloud
        if (degradeVisual){
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::RandomSample<pcl::PointXYZ> random;
            random.setInputCloud (cloudOutput);
            random.setSample(30);
            random.filter (*filteredCloud);

            float delay = ((double) std::rand() / (RAND_MAX)) - 0.5;
            ros::Duration(delay).sleep();
            pclPub.publish(filteredCloud);
        }
        else{
            pclPub.publish(cloudOutput);
        }
        

	    loop_rate.sleep();
    };
};


int main(int argc, char** argv){
    ros::init(argc, argv, "LaserToPointcloud");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("Node initialised");
    ROS_INFO_STREAM("KdTree version");
    LaserToPointcloud laserToPointcloud = LaserToPointcloud(&nh);
    ros::spin();
    return 0;
}
