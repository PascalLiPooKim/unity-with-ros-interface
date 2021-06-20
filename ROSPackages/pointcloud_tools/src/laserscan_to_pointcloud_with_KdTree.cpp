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

class LaserToPointcloud{
    private:
    tf::TransformListener* tfListener;
    std::string scanSubTopic;
    std::string pclPubTopic;
    ros::Subscriber scanSub;
    ros::Publisher pclPub;

    sensor_msgs::PointCloud2 pc2Msg;
    int updateRate;
    std::string frame;
    std_msgs::Header outputHeader;

    // Variables required for KdTree
    pcl::PointXYZ rightSearchPoint;
    pcl::PointXYZ leftSearchPoint;

    ros::Publisher rightAudioPub;


    public:
    LaserToPointcloud(ros::NodeHandle *nh){
        tfListener = new tf::TransformListener();
        nh->param<std::string>("pcl_topic", pclPubTopic, "/scan/pointcloud");
        nh->param<std::string>("scan_topic", scanSubTopic, "/scan");
	    nh->param<int>("update_rate", updateRate, 30);
        nh->getParam("frame", frame);
        scanSub = nh->subscribe(scanSubTopic, 1, &LaserToPointcloud::ScanCallback, this);
        pclPub = nh->advertise<sensor_msgs::PointCloud2>(pclPubTopic, 1);

        rightAudioPub = nh->advertise<std_msgs::Float32>("minRightDistance", 1);

    }



    void ScanCallback(const sensor_msgs::LaserScan& msg){
	ros::Rate loop_rate(updateRate);
        laser_geometry::LaserProjection projector;
        projector.projectLaser(msg, pc2Msg);
        
        //Converting pc2 message to pcl
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutput = conversions::fromPC2toPCL(pc2Msg);
        outputHeader = msg.header;


        if(frame.length() > 0){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTransformed (new pcl::PointCloud<pcl::PointXYZ>);
            cloudTransformed = functions::transformPCL(cloudOutput, outputHeader.frame_id, frame, tfListener);
            outputHeader.frame_id = frame;
            cloudOutput = cloudTransformed;
        }

        pcl_conversions::toPCL(outputHeader, cloudOutput->header);
        pclPub.publish(cloudOutput);

        // Apply KdTree on final cloud output

        FindClosestPoints(cloudOutput);
        

	    loop_rate.sleep();
    };

    void FindClosestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutput){

        // Fixed virtual points around the Husky

        // <xacro:property name="base_x_size" value="0.98740000" />
        // <xacro:property name="base_y_size" value="0.57090000" />
        // <xacro:property name="base_z_size" value="0.24750000" />

        rightSearchPoint.x = 0.4937f + 0.05f;
        rightSearchPoint.y = 0.0f;
        rightSearchPoint.z = 0.0f;

        
        leftSearchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
        leftSearchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
        leftSearchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloudOutput);

        // K nearest neighbor search
        // int K = 10;

        // std::vector<int> pointIdxKNNSearch(K);
        // std::vector<float> pointKNNSquaredDistance(K);

        // Neighbors within radius search
        float radius = 0.2f;

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        std_msgs::Float32 rightMsgDistance;

        if (kdtree.radiusSearch(rightSearchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {   
            // kdtree.nearestKSearch(rightSearchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);
            // float closestPointDistance = *min_element(pointRadiusSquaredDistance.begin(),
            //                                             pointRadiusSquaredDistance.end());

            // float closestPointDistance = pointRadiusSquaredDistance[0];
            
            rightMsgDistance.data = sqrt(pointRadiusSquaredDistance[0]);
        }
        else{
            rightMsgDistance.data = 0.0f;
        }

        rightAudioPub.publish(rightMsgDistance);
    }
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
