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
    pcl::PointXYZ frontSearchPoint;
    pcl::PointXYZ backSearchPoint;

    ros::Publisher rightAudioPub;
    ros::Publisher leftAudioPub;
    ros::Publisher frontAudioPub;
    ros::Publisher backAudioPub;


    public:
    LaserToPointcloud(ros::NodeHandle *nh){
        tfListener = new tf::TransformListener();
        nh->param<std::string>("pcl_topic", pclPubTopic, "/scan/pointcloud");
        nh->param<std::string>("scan_topic", scanSubTopic, "/scan");
	    nh->param<int>("update_rate", updateRate, 30);
        nh->getParam("frame", frame);
        scanSub = nh->subscribe(scanSubTopic, 1, &LaserToPointcloud::ScanCallback, this);
        pclPub = nh->advertise<sensor_msgs::PointCloud2>(pclPubTopic, 1);

        rightAudioPub = nh->advertise<std_msgs::Float32>("/audio/minRightDistance", 1);
        leftAudioPub = nh->advertise<std_msgs::Float32>("/audio/minLeftDistance", 1);
        frontAudioPub = nh->advertise<std_msgs::Float32>("/audio/minFrontDistance", 1);
        backAudioPub = nh->advertise<std_msgs::Float32>("/audio/minBackDistance", 1);
        

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

        rightSearchPoint.x = 0.0f;
        rightSearchPoint.y = -0.2f;
        rightSearchPoint.z = 0.0f;
        
        leftSearchPoint.x = 0.0f;
        leftSearchPoint.y = 0.2f;
        leftSearchPoint.z = 0.0f;

        frontSearchPoint.x = 0.55f;
        frontSearchPoint.y = 0.0f;
        frontSearchPoint.z = 0.0f;

        backSearchPoint.x = -0.55f;
        backSearchPoint.y = 0.0f;
        backSearchPoint.z = 0.0f;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloudOutput);

        // K nearest neighbor search
        int K = 5;

        std::vector<int> rightPointIdxKNNSearch(K);
        std::vector<float> rightPointKNNSquaredDistance(K);

        std::vector<int> leftPointIdxKNNSearch(K);
        std::vector<float> leftPointKNNSquaredDistance(K);

        std::vector<int> frontPointIdxKNNSearch(K);
        std::vector<float> frontPointKNNSquaredDistance(K);

        std::vector<int> backPointIdxKNNSearch(K);
        std::vector<float> backPointKNNSquaredDistance(K);

        // Neighbors within radius search
        // float radius = 0.75f;

        // std::vector<int> pointIdxRadiusSearch;
        // std::vector<float> pointRadiusSquaredDistance;

        std_msgs::Float32 rightMsgDistance;
        std_msgs::Float32 leftMsgDistance;
        std_msgs::Float32 frontMsgDistance;
        std_msgs::Float32 backMsgDistance;

        // if (kdtree.radiusSearch(rightSearchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        // {   
        //     // kdtree.nearestKSearch(rightSearchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);
        //     // float closestPointDistance = *min_element(pointRadiusSquaredDistance.begin(),
        //     //                                             pointRadiusSquaredDistance.end());

        //     // float closestPointDistance = pointRadiusSquaredDistance[0];
        //     std::cout << true << std::endl;
        //     rightMsgDistance.data = sqrt(pointRadiusSquaredDistance[0]);
        // }
        // else{
        //     rightMsgDistance.data = 0.0f;
        // }

        // On the right of Husky
        if (kdtree.nearestKSearch(rightSearchPoint, K, rightPointIdxKNNSearch, rightPointKNNSquaredDistance) > 0)
        {   
            // kdtree.nearestKSearch(rightSearchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);
            // float closestPointDistance = *min_element(pointRadiusSquaredDistance.begin(),
            //                                             pointRadiusSquaredDistance.end());

            // float closestPointDistance = pointRadiusSquaredDistance[0];
            // std::cout << true << std::endl;
            rightMsgDistance.data = rightPointKNNSquaredDistance[0];
        }
        else{
            rightMsgDistance.data = 0.0f;
        }

        rightAudioPub.publish(rightMsgDistance);


        // On the left of Husky
        if (kdtree.nearestKSearch(leftSearchPoint, K, leftPointIdxKNNSearch, leftPointKNNSquaredDistance) > 0)
        {   
            leftMsgDistance.data = leftPointKNNSquaredDistance[0];
        }
        else{
            leftMsgDistance.data = 0.0f;
        }

        leftAudioPub.publish(leftMsgDistance);

        // In front of Husky
        if (kdtree.nearestKSearch(frontSearchPoint, K, frontPointIdxKNNSearch, frontPointKNNSquaredDistance) > 0)
        {   
            frontMsgDistance.data = frontPointKNNSquaredDistance[0];
        }
        else{
            frontMsgDistance.data = 0.0f;
        }
        frontAudioPub.publish(frontMsgDistance);

        // At the back of Husky
        // if (kdtree.nearestKSearch(backSearchPoint, K, backPointIdxKNNSearch, backPointKNNSquaredDistance) > 0)
        // {   
        //     backMsgDistance.data = backPointKNNSquaredDistance[0];
        // }
        // else{
        //     backMsgDistance.data = 0.0f;
        // }
        // backAudioPub.publish(backMsgDistance);
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
