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
    pcl::PointXYZ frontSearchPoint;
    pcl::PointXYZ rightFrontSearchPoint;
    pcl::PointXYZ rightBackSearchPoint;
    pcl::PointXYZ leftFrontSearchPoint;
    pcl::PointXYZ leftBackSearchPoint;
    
    // pcl::PointXYZ backSearchPoint;

    ros::Publisher rightFrontAudioPub;
    ros::Publisher rightBackAudioPub;
    ros::Publisher leftFrontAudioPub;
    ros::Publisher leftBackAudioPub;
    ros::Publisher frontAudioPub;
    // ros::Publisher backAudioPub;

    ros::Publisher frontPclPub;
    ros::Publisher leftFrontPclPub;
    ros::Publisher leftBackPclPub;
    ros::Publisher rightFrontPclPub;
    ros::Publisher rightBackPclPub;


    public:
    LaserToPointcloud(ros::NodeHandle *nh){
        tfListener = new tf::TransformListener();
        nh->param<std::string>("pcl_topic", pclPubTopic, "/scan/pointcloud");
        nh->param<std::string>("scan_topic", scanSubTopic, "/scan");
	    nh->param<int>("update_rate", updateRate, 30);
        nh->getParam("frame", frame);
        scanSub = nh->subscribe(scanSubTopic, 1, &LaserToPointcloud::ScanCallback, this);
        pclPub = nh->advertise<sensor_msgs::PointCloud2>(pclPubTopic, 1);
        

        rightFrontAudioPub = nh->advertise<std_msgs::Float32>("/audio/minRightFrontDistance", 1);
        rightBackAudioPub = nh->advertise<std_msgs::Float32>("/audio/minRightBackDistance", 1);
        leftFrontAudioPub = nh->advertise<std_msgs::Float32>("/audio/minLeftFrontDistance", 1);
        leftBackAudioPub = nh->advertise<std_msgs::Float32>("/audio/minLeftBackDistance", 1);
        frontAudioPub = nh->advertise<std_msgs::Float32>("/audio/minFrontDistance", 1);
        //backAudioPub = nh->advertise<std_msgs::Float32>("/audio/minBackDistance", 1);

        frontPclPub = nh->advertise<sensor_msgs::PointCloud2>("/frontFilteredPcl", 1);
        leftFrontPclPub = nh->advertise<sensor_msgs::PointCloud2>("/leftFrontFilteredPcl", 1);
        leftBackPclPub = nh->advertise<sensor_msgs::PointCloud2>("/leftBackFilteredPcl", 1);
        rightFrontPclPub = nh->advertise<sensor_msgs::PointCloud2>("/rightFrontFilteredPcl", 1);
        rightBackPclPub = nh->advertise<sensor_msgs::PointCloud2>("/rightBackFilteredPcl", 1);
        

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

        rightFrontSearchPoint.x = 0.45f; //-0.15 -> - 0.2
        rightFrontSearchPoint.y = -0.25f;
        rightFrontSearchPoint.z = 0.0f;

        rightBackSearchPoint.x = -0.45f; //-0.15 -> - 0.2
        rightBackSearchPoint.y = -0.25f;
        rightBackSearchPoint.z = 0.0f;
        
        leftFrontSearchPoint.x = 0.45f;
        leftFrontSearchPoint.y = 0.25f; //0.2
        leftFrontSearchPoint.z = 0.0f;

        leftBackSearchPoint.x = -0.45f;
        leftBackSearchPoint.y = 0.25f; //0.2
        leftBackSearchPoint.z = 0.0f;

        frontSearchPoint.x = 0.5f; //0.55
        frontSearchPoint.y = 0.0f;
        frontSearchPoint.z = 0.0f;

        // backSearchPoint.x = -0.55f;
        // backSearchPoint.y = 0.0f;
        // backSearchPoint.z = 0.0f;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        if (cloudOutput->size() > 0){
		    kdtree.setInputCloud(cloudOutput);
	    }


        pcl::KdTreeFLANN<pcl::PointXYZ> frontKdtree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr frontFilteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
        passThroughFilter(cloudOutput, frontFilteredCloud, "y", -0.5f, 0.5f); // 0.35 --> 0.5

        pcl::KdTreeFLANN<pcl::PointXYZ> rightFrontKdtree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr rightFrontFilteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
        passThroughFilter(cloudOutput, rightFrontFilteredCloud, 
        "x", 0.0f, 0.6f, "y", -1.55f, -0.25f);
        // -0.5    0.6 --> 0.7

        pcl::KdTreeFLANN<pcl::PointXYZ> rightBackKdtree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr rightBackFilteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
        passThroughFilter(cloudOutput, rightBackFilteredCloud, 
        "x", -0.6f, 0.0f, "y", -1.55f, -0.25f);

        pcl::KdTreeFLANN<pcl::PointXYZ> leftFrontKdtree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr leftFrontFilteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
        passThroughFilter(cloudOutput, leftFrontFilteredCloud, "x", 0.0f, 0.6f, "y", 0.25f, 1.55f);

        pcl::KdTreeFLANN<pcl::PointXYZ> leftBackKdtree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr leftBackFilteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
        passThroughFilter(cloudOutput, leftBackFilteredCloud, "x", -0.6f, 0.0f, "y", 0.25f, 1.55f);
                                                                    
        
        

        // K nearest neighbor search
        int K = 5;

        std::vector<int> rightFrontPointIdxKNNSearch(K);
        std::vector<float> rightFrontPointKNNSquaredDistance(K);

        std::vector<int> rightBackPointIdxKNNSearch(K);
        std::vector<float> rightBackPointKNNSquaredDistance(K);

        std::vector<int> leftFrontPointIdxKNNSearch(K);
        std::vector<float> leftFrontPointKNNSquaredDistance(K);

        std::vector<int> leftBackPointIdxKNNSearch(K);
        std::vector<float> leftBackPointKNNSquaredDistance(K);

        std::vector<int> frontPointIdxKNNSearch(K);
        std::vector<float> frontPointKNNSquaredDistance(K);

        // std::vector<int> backPointIdxKNNSearch(K);
        // std::vector<float> backPointKNNSquaredDistance(K);

        // Neighbors within radius search
        // float radius = 0.3f;

        // std::vector<int> rightPointIdxRadiusSearch;
        // std::vector<float> rightPointRadiusSquaredDistance;

        std_msgs::Float32 rightFrontMsgDistance;
        std_msgs::Float32 rightBackMsgDistance;
        std_msgs::Float32 leftFrontMsgDistance;
        std_msgs::Float32 leftBackMsgDistance;
        std_msgs::Float32 frontMsgDistance;
        // std_msgs::Float32 backMsgDistance;

        // if (kdtree.radiusSearch(rightFrontSearchPoint, radius, rightPointIdxRadiusSearch, 
        //     rightPointRadiusSquaredDistance) > 2)
        // {   
        //     // kdtree.nearestKSearch(rightFrontSearchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);
        //     // float closestPointDistance = *min_element(pointRadiusSquaredDistance.begin(),
        //     //                                             pointRadiusSquaredDistance.end());

        //     // float closestPointDistance = pointRadiusSquaredDistance[0];
        //     // std::cout << true << std::endl;
        //     rightFrontMsgDistance.data = rightPointRadiusSquaredDistance[0];
        // }
        // else{
        //     rightFrontMsgDistance.data = 0.0f;
        // }

        // On the right of Husky
        if(rightFrontFilteredCloud->size() > 0)
        {
            rightFrontKdtree.setInputCloud(rightFrontFilteredCloud);
            if (rightFrontKdtree.nearestKSearch(rightFrontSearchPoint, K, rightFrontPointIdxKNNSearch, 
                rightFrontPointKNNSquaredDistance) > 0)
            {   
                rightFrontMsgDistance.data = rightFrontPointKNNSquaredDistance[0];
            }
            else
            {
                rightFrontMsgDistance.data = 0.0f;
            }

        }
        else
        {
            rightFrontMsgDistance.data = 0.0f;
        }
        // if (kdtree.nearestKSearch(rightFrontSearchPoint, K, rightFrontPointIdxKNNSearch, 
        //     rightFrontPointKNNSquaredDistance) > 2)
        // {   
        //     // kdtree.nearestKSearch(rightFrontSearchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);
        //     // float closestPointDistance = *min_element(pointRadiusSquaredDistance.begin(),
        //     //                                             pointRadiusSquaredDistance.end());

        //     // float closestPointDistance = pointRadiusSquaredDistance[0];
        //     // std::cout << true << std::endl;
        //     rightFrontMsgDistance.data = rightFrontPointKNNSquaredDistance[0];
        // }
        // else{
        //     rightFrontMsgDistance.data = 0.0f;
        // }

        rightFrontAudioPub.publish(rightFrontMsgDistance);
        rightFrontPclPub.publish(rightFrontFilteredCloud);


        if(rightBackFilteredCloud->size() > 0)
        {
            rightBackKdtree.setInputCloud(rightBackFilteredCloud);
            if (rightBackKdtree.nearestKSearch(rightBackSearchPoint, K, rightBackPointIdxKNNSearch, 
                rightBackPointKNNSquaredDistance) > 0)
            {   
                rightBackMsgDistance.data = rightBackPointKNNSquaredDistance[0];
            }
            else
            {
                rightBackMsgDistance.data = 0.0f;
            }

        }
        else
        {
            rightBackMsgDistance.data = 0.0f;
        }

        rightBackAudioPub.publish(rightBackMsgDistance);
        rightBackPclPub.publish(rightBackFilteredCloud);


        // On the left of Husky
        if(leftFrontFilteredCloud->size() > 0)
        {
            leftFrontKdtree.setInputCloud(leftFrontFilteredCloud);
            if (leftFrontKdtree.nearestKSearch(leftFrontSearchPoint, K, leftFrontPointIdxKNNSearch, 
                leftFrontPointKNNSquaredDistance) > 0)
            {   
                leftFrontMsgDistance.data = leftFrontPointKNNSquaredDistance[0];
            }
            else
            {
                leftFrontMsgDistance.data = 0.0f;
            }

        }
        else
        {
            leftFrontMsgDistance.data = 0.0f;
        }

        // if (kdtree.nearestKSearch(leftFrontSearchPoint, K, leftFrontPointIdxKNNSearch, leftFrontPointKNNSquaredDistance) > 2)
        // {   
        //     leftFrontMsgDistance.data = leftFrontPointKNNSquaredDistance[0];
        // }
        // else{
        //     leftFrontMsgDistance.data = 0.0f;
        // }

        leftFrontAudioPub.publish(leftFrontMsgDistance);
        leftFrontPclPub.publish(leftFrontFilteredCloud);


        if(leftBackFilteredCloud->size() > 0)
        {
            leftBackKdtree.setInputCloud(leftBackFilteredCloud);
            if (leftBackKdtree.nearestKSearch(leftBackSearchPoint, K, leftBackPointIdxKNNSearch, 
                leftBackPointKNNSquaredDistance) > 0)
            {   
                leftBackMsgDistance.data = leftBackPointKNNSquaredDistance[0];
            }
            else
            {
                leftBackMsgDistance.data = 0.0f;
            }

        }
        else
        {
            leftBackMsgDistance.data = 0.0f;
        }

        leftBackAudioPub.publish(leftBackMsgDistance);
        leftBackPclPub.publish(leftBackFilteredCloud);
        

        // In front of Husky

        if(frontFilteredCloud->size() > 0){

            frontKdtree.setInputCloud(frontFilteredCloud);

            if (frontKdtree.nearestKSearch(frontSearchPoint, K, frontPointIdxKNNSearch, 
            frontPointKNNSquaredDistance) > 0)
            {   
                frontMsgDistance.data = frontPointKNNSquaredDistance[0];
            }
            else{
                frontMsgDistance.data = 0.0f;
                }
            }
        else{
            frontMsgDistance.data = 0.0f;
        }




        // if (kdtree.nearestKSearch(frontSearchPoint, K, frontPointIdxKNNSearch, frontPointKNNSquaredDistance) > 2)
        // {   
        //     frontMsgDistance.data = frontPointKNNSquaredDistance[0];
        // }
        // else{
        //     frontMsgDistance.data = 0.0f;
        // }

        frontAudioPub.publish(frontMsgDistance);
        frontPclPub.publish(frontFilteredCloud);

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

    void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud, std::string axis, 
    float min, float max)
    {
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud (cloud);
            pass.setFilterFieldName (axis);
            // min and max values in y axis to keep
            pass.setFilterLimits (min, max);
            pass.filter (*filteredCloud);
            
    }

    void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud, std::string xAxis, 
    float xMin, float xMax, std::string yAxis, float yMin, float yMax)
    {

        pcl::PassThrough<pcl::PointXYZ> yPass;
        yPass.setInputCloud (cloud);
        yPass.setFilterFieldName (yAxis);
        // min and max values in y axis to keep
        yPass.setFilterLimits (yMin, yMax);
        yPass.filter (*filteredCloud);

        pcl::PassThrough<pcl::PointXYZ> xPass;
        xPass.setInputCloud (filteredCloud);
        xPass.setFilterFieldName (xAxis);
            // min and max values in y axis to keep
        xPass.setFilterLimits (xMin, xMax);
        xPass.filter (*filteredCloud);
        
        

        // pcl::PassThrough<pcl::PointXYZ> xPass;
        // xPass.setInputCloud (cloud);
        // xPass.setFilterFieldName (xAxis);
        //     // min and max values in y axis to keep
        // xPass.setFilterLimits (xMin, xMax);
        // xPass.filter (*filteredCloud);
        
        // pcl::PassThrough<pcl::PointXYZ> yPass;
        // yPass.setInputCloud (filteredCloud);
        // yPass.setFilterFieldName (yAxis);
        // // min and max values in y axis to keep
        // yPass.setFilterLimits (yMin, yMax);
        // yPass.filter (*filteredCloud);
            
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "LaserToPointcloud");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("Node initialised");
    ROS_INFO_STREAM("KdTree version 2");
    LaserToPointcloud laserToPointcloud = LaserToPointcloud(&nh);
    ros::spin();
    return 0;
}
