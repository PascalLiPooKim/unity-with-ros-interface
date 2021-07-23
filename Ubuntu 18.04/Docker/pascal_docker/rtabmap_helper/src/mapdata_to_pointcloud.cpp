#include <ros/ros.h>
#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv_bridge/cv_bridge.h"

#include "rtabmap/core/util3d.h"
#include "rtabmap/core/Compression.h"
#include "rtabmap/core/Signature.h"
#include <rtabmap/core/Graph.h>

#include "rtabmap_ros/MsgConversion.h"
#include "rtabmap_ros/MapData.h"
#include "rtabmap_ros/NodeData.h"
#include "rtabmap_ros/MapGraph.h"

#include <rtabmap_helper/PointCloudMapData.h>
#include <rtabmap_helper/PointCloudNodeData.h>

#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/common/io.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_ros/transforms.h>
#include <algorithm>

class MapdataToPointcloud{

    private:
    tf::TransformListener* tfListener;
    ros::Subscriber map_data_sub;
    ros::Publisher pcl_pub;
    ros::Publisher pcl_mapdata_pub;
    std::string pcl_mapdata_topic;
    std::string pcl_data_topic;
    double voxelsize;
    std_msgs::Header header;
    float fx;
    float fy;
    float cx;
    float cy;
    float fx_inv;
    float fy_inv;

    std::map<int, pcl::PointCloud<pcl::PointXYZRGB>> cloud_dict;
    ros::Publisher pcl_map_debug;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_aggregate;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_aggregate_icp;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_alligned;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out;
    bool isIcpReady = false;
    union RGBValue
    {
        struct
        {
        unsigned char Blue;
        unsigned char Green;
        unsigned char Red;
        unsigned char Alpha;
        };
        float float_value;
        std::uint32_t long_value;
    };

    public:
    MapdataToPointcloud(ros::NodeHandle *nh){
        tfListener = new tf::TransformListener;
        nh->param<std::string>("pcl_mapdata_topic", pcl_mapdata_topic, "/rtabmap_helper/pcl_mapdata");
        nh->param<std::string>("pcl_data_topic", pcl_data_topic, "/rtabmap_helper/pcl_frame");
        nh->param<double>("voxelsize", voxelsize, 0.05d);

        ROS_INFO_STREAM("PointCloud data topic set to: " << pcl_data_topic);
        ROS_INFO_STREAM("PointCloud map data topic set to: " << pcl_mapdata_topic);
        ROS_INFO("Voxelsize set to: %.2f", (float)voxelsize);

        map_data_sub = nh->subscribe("/rtabmap/mapData", 100, &MapdataToPointcloud::mapdata_callback, this);
        pcl_pub = nh->advertise<sensor_msgs::PointCloud2>(pcl_data_topic, 100);
        pcl_mapdata_pub = nh->advertise<rtabmap_helper::PointCloudMapData>(pcl_mapdata_topic, 100);
        pcl_map_debug = nh->advertise<sensor_msgs::PointCloud2>("/rtabmap_helper/pcl_debug", 100);
    }

    void print4x4Matrix (const Eigen::Matrix4d & matrix)
    {
        printf ("Rotation matrix :\n");
        printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
        printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
        printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
        printf ("Translation vector :\n");
        printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
    }


    void mapdata_callback(const rtabmap_ros::MapData& msg){
        header = msg.header;
        //Uncompress images
        cv::Mat rgb_image_compressed = cv::Mat(msg.nodes[0].image);
        cv::Mat depth_image_compressed = cv::Mat(msg.nodes[0].depth);
        cv::Mat rgb_image;
        cv::Mat depth_image;
        rgb_image = rtabmap::uncompressImage(rgb_image_compressed);
        depth_image = rtabmap::uncompressImage(depth_image_compressed);

        //Assign camera variables from message
        fx = msg.nodes[0].fx[0];
        fy = msg.nodes[0].fy[0];
        cx = msg.nodes[0].cx[0];
        cy = msg.nodes[0].cy[0];
        fx_inv = 1.0f / fx;
        fy_inv = 1.0f / fy;

        //Obtain pointcloud from rgb and depth image
        if(depth_image.size().width != 0 || depth_image.size().height != 0 
            || rgb_image.size().width != 0 || rgb_image.size().height != 0){
        ROS_INFO("Received new frame [id: %d]", msg.nodes[0].id);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rtab = cloudRGBFromRtabmap(msg.nodes[0]);
        pcl_conversions::toPCL(msg.header, cloud_rtab->header);
        
        //Apply voxelfilter to downsample pointcloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        VoxelFilter(cloud_rtab, cloud_filtered, (float)voxelsize);
        cloud_dict.insert(std::make_pair(msg.nodes[0].id, *cloud_filtered));

        //Transforming cloud into correct position for frame publisher
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
        int index;
        for (int i = 0; i < msg.graph.posesId.size(); i++){
            if(msg.graph.posesId[i] == msg.nodes[0].id){
                index = i;
            }
        }

        Eigen::Vector3f map_position = Eigen::Vector3f(msg.graph.poses[index].position.x, msg.graph.poses[index].position.y, msg.graph.poses[index].position.z);
        Eigen::Quaternionf map_rotation = Eigen::Quaternionf(msg.graph.poses[index].orientation.w, msg.graph.poses[index].orientation.x, 
            msg.graph.poses[index].orientation.y, msg.graph.poses[index].orientation.z);
            
        pcl::transformPointCloud(*cloud_filtered, *cloud_transformed, map_position, map_rotation);
        
        //Generating custom message and assigning variables
        rtabmap_helper::PointCloudMapData pcl_mapdata;
        rtabmap_helper::PointCloudNodeData pcl_nodedata;
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*cloud_filtered, ros_cloud);

        //Filtering poses
        std::map<int, rtabmap::Transform> poses;
	    for(unsigned int i=0; i<msg.graph.posesId.size() && i<msg.graph.poses.size(); ++i){
		    poses.insert(std::make_pair(msg.graph.posesId[i], rtabmap_ros::transformFromPoseMsg(msg.graph.poses[i])));
	    }

        poses = rtabmap::graph::radiusPosesFiltering(poses,0.0f, 30.0f*CV_PI/180.0);
        std::vector<int> posesId;
        std::vector<geometry_msgs::Pose> ros_poses;

        for (std::map<int, rtabmap::Transform>::iterator it=poses.begin(); it != poses.end(); ++it){
            geometry_msgs::Pose ros_pose;
            rtabmap_ros::transformToPoseMsg(it->second, ros_pose);
            posesId.push_back(it->first);
            ros_poses.push_back(ros_pose);
        }

        // //Constructing aggregate pointcloud
        // pcl::PointCloud<pcl::PointXYZRGB> cloud_aggregate;
        // for(unsigned int i=0; i<posesId.size() && i<ros_poses.size(); ++i){
        //     if(cloud_dict.find(posesId[i])!=cloud_dict.end()){
        //         pcl::PointCloud<pcl::PointXYZRGB> cloud_step;
        //         pcl::PointCloud<pcl::PointXYZRGB> cloud_step_transformed;

        //         cloud_step = cloud_dict.find(posesId[i])->second;

        //         Eigen::Vector3f map_position = Eigen::Vector3f(ros_poses[i].position.x, ros_poses[i].position.y, ros_poses[i].position.z);
        //         Eigen::Quaternionf map_rotation = Eigen::Quaternionf(ros_poses[i].orientation.w, ros_poses[i].orientation.x, 
        //             ros_poses[i].orientation.y, ros_poses[i].orientation.z);

        //         pcl::transformPointCloud(cloud_step, cloud_step_transformed, map_position, map_rotation);
        //         cloud_aggregate += cloud_step_transformed;
        //     }
        // }
        // pcl_conversions::toPCL(msg.header, cloud_aggregate.header);
        
        //Constructing ROS messages
        pcl_nodedata.id = msg.nodes[0].id;
        pcl_nodedata.mapId = msg.nodes[0].mapId;
        pcl_nodedata.weight = msg.nodes[0].weight;
        pcl_nodedata.stamp = msg.nodes[0].stamp;
        pcl_nodedata.label = msg.nodes[0].label;
        pcl_nodedata.nodeData = ros_cloud;
        pcl_nodedata.localTransform = msg.nodes[0].localTransform;
        
        pcl_mapdata.graph = msg.graph;
        pcl_mapdata.graph.posesId = posesId;
        pcl_mapdata.graph.poses = ros_poses;
        pcl_mapdata.header = header;
        pcl_mapdata.nodes.push_back(pcl_nodedata);

        //Publishing Messages
        pcl_pub.publish(cloud_transformed);
        pcl_mapdata_pub.publish(pcl_mapdata);
        //pcl_map_debug.publish(cloud_aggregate);
        }
        else{
            ROS_INFO("Recieved empty frame, Skipping.");
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToXYZRGBPointCloud(cv::Mat &image, cv::Mat &depth){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl_conversions::toPCL(header, cloud->header);
        cloud->width = image.size().width;
        cloud->height = image.size().height;
        cloud->points.resize (cloud->height * cloud->width);

        //Getting data buffer from images
        uint8_t* rgb_buffer = (uint8_t*)image.data;
        float* depth_buffer = (float*)depth.data;

        //Variables for depth loop
        unsigned step = cloud->width / depth.size().width;
        unsigned skip = cloud->width - (depth.size().width * step);
        unsigned value_idx = 0;
        unsigned point_idx = 0;

        //Depth image -> assigning positions of points
        for (unsigned v = 0; v < depth.size().height; ++v, point_idx += skip){
            for (unsigned u = 0; u < depth.size().width; ++u, ++value_idx, point_idx += step){
                pcl::PointXYZRGB& pt = (*cloud)[point_idx];
                if(!isnan(depth_buffer[point_idx])){
                    pt.z = depth_buffer[value_idx];
                    pt.x = (static_cast<float> (u) - cx) * pt.z * fx_inv;
                    pt.y = (static_cast<float> (v) - cy) * pt.z * fy_inv;
                }
                else
                {
                    pt.z = 0.0f;
                    pt.y = 0.0f;
                    pt.x = 0.0f;
                }
            }
        }

        //Variables for RGB loop
        step = cloud->width / depth.size().width;
        skip = cloud->width - (depth.size().width * step);  
        value_idx = 0;
        point_idx = 0;
        RGBValue color;
        color.Alpha = 0xff;

        //Colour image -> assigning colours of points
        for (unsigned yIdx = 0; yIdx < image.size().height; ++yIdx, point_idx += skip){
            for (unsigned xIdx = 0; xIdx < image.size().width; ++xIdx, point_idx += step, value_idx += 3){
                pcl::PointXYZRGB& pt = (*cloud)[point_idx];

                color.Blue  = rgb_buffer[value_idx];
                color.Green = rgb_buffer[value_idx + 1];
                color.Red   = rgb_buffer[value_idx + 2];

                pt.rgba = color.long_value;
            }
        }

        return cloud;
    }

            // if(isIcpReady){
        //     ROS_INFO("Applying ICP");

        //     pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aggregate_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
        //     pcl::copyPointCloud(cloud_aggregate_icp, *cloud_aggregate_ptr);

        //     // Set the input source and target
        //     icp.setInputSource (cloud_filtered_transformed);
        //     icp.setInputTarget (cloud_aggregate_ptr);
        //     icp.setMaxCorrespondenceDistance (0.05);
        //     icp.setRANSACOutlierRejectionThreshold(0.10);
        //     // Set the maximum number of iterations (criterion 1)
        //     icp.setMaximumIterations (500);
        //     // Set the transformation epsilon (criterion 2)
        //     icp.setTransformationEpsilon (1e-9);
        //     // Set the euclidean distance difference epsilon (criterion 3)
        //     icp.setEuclideanFitnessEpsilon (1);
        //     icp.align (cloud_alligned);
        //     if (icp.hasConverged ()){
        //         ROS_INFO("ICP Converged");
        //         Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();
        //         print4x4Matrix (transformation_matrix.cast<double>());
        //     }
        //     pcl::PointCloud<pcl::PointXYZRGB> transformed_target_cloud;
        //     Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();
        //     pcl::transformPointCloud(*cloud_filtered_transformed, transformed_target_cloud, transformation_matrix);
        //     cloud_aggregate_icp += transformed_target_cloud;

        //     sensor_msgs::PointCloud2 ros_aggregate_icp;
        //     cloud_aggregate_icp.header.frame_id = "/map";
        //     pcl::toROSMsg(cloud_aggregate_icp, ros_aggregate_icp);
        //     pcl_map_debug_icp.publish(ros_aggregate_icp);
        // }
        // else{
        //     ROS_INFO("cloud_aggregate has no points; Creating");
        //     cloud_aggregate_icp += *cloud_filtered_transformed;
        //     isIcpReady = true;
        // }

    pcl::PointCloud<pcl::PointXYZRGB> cloudICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_target){
         pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
            // Set the input source and target
            icp.setInputSource (input_src);
            icp.setInputTarget (input_target);
            icp.setMaxCorrespondenceDistance (0.05);
            icp.setRANSACOutlierRejectionThreshold(0.10);
            // Set the maximum number of iterations (criterion 1)
            icp.setMaximumIterations (500);
            // Set the transformation epsilon (criterion 2)
            icp.setTransformationEpsilon (1e-9);
            // Set the euclidean distance difference epsilon (criterion 3)
            icp.setEuclideanFitnessEpsilon (1);
            icp.align (cloud_alligned);
            pcl::PointCloud<pcl::PointXYZRGB> transformed_target;
            if (icp.hasConverged ()){
                Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();
                pcl::PointCloud<pcl::PointXYZRGB> transformed_target_cloud;
                pcl::transformPointCloud(*input_target, transformed_target, transformation_matrix);
            }
            return transformed_target;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGBFromRtabmap(rtabmap_ros::NodeData node){
        rtabmap::Signature s = rtabmap_ros::nodeDataFromROS(node);
        cv::Mat image, depth;
        rtabmap::LaserScan scan;

        s.sensorData().uncompressData(true?&image:0, true?&depth:0, false?&scan:0);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        pcl::IndicesPtr validIndices(new std::vector<int>);

        cloud = rtabmap::util3d::cloudRGBFromSensorData(
                s.sensorData(),
                1,
                999.0f,
                0.5f,
                validIndices.get());
        ROS_INFO_STREAM(cloud->header.frame_id);
        return cloud;
    }

    void VoxelFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output, float voxel_size){
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud (cloud_input);
        sor.setLeafSize (voxel_size, voxel_size, voxel_size);
        sor.filter (*cloud_output);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input, std::string frameid_input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output, std::string frameid_output){
        tf::StampedTransform transform;
        try{
            tfListener->lookupTransform(frameid_input, frameid_output,
                                ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s","No Transform Found");
            return cloud_input;
        }

        pcl_ros::transformPointCloud(*cloud_input, *cloud_output, transform);
        return cloud_output;    
    }
};

int main(int argc, char **argv){
    ROS_INFO("Started Node");
    ros::init(argc, argv, "rtabmap_helper");
    ros::NodeHandle nh;
    MapdataToPointcloud mdtpc = MapdataToPointcloud(&nh);
    ros::spin();
}
