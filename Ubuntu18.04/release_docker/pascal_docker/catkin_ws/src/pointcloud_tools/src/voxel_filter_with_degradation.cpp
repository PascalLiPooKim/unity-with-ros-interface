
/**
 * Script to collect data from RGBD camera and publish the pointcloud.
 * Depeding on the keyboard key pressed, the visual information can be degraded.
 */

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"

#include <pcl/filters/voxel_grid.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

#include "pointcloud_tools/functions.h"

#include <std_msgs/Bool.h>
#include <cstdlib> 
#include <ctime> 

bool degradeVisual = false;

class VoxelFilter{
    private:
    std::string pclSubTopic;
    std::string pclFilteredPubTopic;
    std::string frame;
    ros::Subscriber pclSub;
    ros::Publisher pclFilteredPub;

    tf::TransformListener* tfListener;
    std_msgs::Header outputHeader;
    double voxelSize;
    int updateRate;

    ros::Subscriber degradeSub;

    public:
    VoxelFilter(ros::NodeHandle *nh){
        tfListener = new tf::TransformListener();

        nh->param<std::string>("pcl_topic", pclSubTopic, "/realsense/depth/color/points");
        nh->param<std::string>("pcl_filtered_topic", pclFilteredPubTopic, "/realsense/depth/color/points/filtered");
        nh->getParam("voxel_size", voxelSize);
        nh->getParam("update_rate", updateRate);
        nh->getParam("frame", frame);

        pclSub = nh->subscribe(pclSubTopic, 1, &VoxelFilter::PclCallback, this);
        pclFilteredPub = nh->advertise<sensor_msgs::PointCloud2>(pclFilteredPubTopic, 1);
        degradeSub = nh->subscribe("/degrade_pointcloud", 2, &VoxelFilter::degradeCallback, this);
    };

    // Callback function to store state of message
    void degradeCallback(const std_msgs::Bool& msg){
        ros::Rate loop_rate(updateRate);
        degradeVisual = msg.data;

        loop_rate.sleep();
    }

    void PclCallback(const sensor_msgs::PointCloud2& msg){
        ros::Rate loop_rate(updateRate);
        outputHeader = msg.header;  

        //Converting pc2 message to pcl
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = conversions::fromPC2toPCLRGB(msg);

        //Filtering cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> voxelFilter;
        voxelFilter.setInputCloud (cloud);


        // Publish degraded or normal vision depending on message
        if (degradeVisual){

            float randomVoxelSize;
            std::srand(static_cast<unsigned int>(std::time(nullptr))); 
            for (int i=0; i < 10; ++i){
                randomVoxelSize = ((double) std::rand() / (RAND_MAX)) + 0.2;
            }
            voxelFilter.setLeafSize ((float)randomVoxelSize, (float)randomVoxelSize, (float)randomVoxelSize);
            float delay = ((double) std::rand() / (RAND_MAX)) - 0.5;

            ros::Duration(delay).sleep();
        }
        else{
            voxelFilter.setLeafSize ((float)voxelSize, (float)voxelSize, (float)voxelSize);
        }
        
        voxelFilter.filter (*cloudFiltered);

        //If output frame specified transforming into frame and update header
        if(frame.length() >0){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTransformed (new pcl::PointCloud<pcl::PointXYZRGB>);
            cloudTransformed = functions::transformPCL(cloudFiltered, outputHeader.frame_id, frame, tfListener);
            outputHeader.frame_id = frame;
            cloudFiltered = cloudTransformed;
        }

        //Formatting output message and publishing
        pcl_conversions::toPCL(outputHeader, cloudFiltered->header);
        pclFilteredPub.publish(cloudFiltered);
        loop_rate.sleep();
    };
};

int main(int argc, char** argv){
    ros::init(argc, argv, "VoxelFilter");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("Node initialised");
    VoxelFilter voxelFilter = VoxelFilter(&nh);
    ros::spin();
    return 0;
}
