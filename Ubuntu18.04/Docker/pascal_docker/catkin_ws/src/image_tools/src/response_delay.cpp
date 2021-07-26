#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <iostream>
#include <math.h>
#include <chrono>
#include <ros/ros.h>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Point.h"

class LatencyTool{
    private:
    ros::Publisher imagePub;
    ros::Subscriber commandSub;
    cv_bridge::CvImage imageMsg;
    int frameR = 255;
    int frameG = 0;
    int frameB = 0;
    int outWidth;
    int outHeight;
    int updaterate;
    bool shouldVisualise;

    public: 
    LatencyTool(ros::NodeHandle *nh){
        nh->param<int>("out_width", outWidth, 640);
        nh->param<int>("out_height", outHeight, 480);
        nh->param<int>("updaterate", updaterate, 30);
        nh->param<bool>("visualise", shouldVisualise, false);
        ros::Rate loop_rate(updaterate);
        commandSub = nh->subscribe("/latency/command", 100, &LatencyTool::LatencyCommandCallback, this);
        imagePub = nh->advertise<sensor_msgs::Image>("latency/image", 100);

        while (ros::ok()){
            cv::Mat image = cv::Mat::zeros(outHeight, outWidth, CV_8UC3);
            for( int i = 0; i < image.rows; i++ ) {
                for( int j = 0; j < image.cols; j++ ) {
                    image.at<cv::Vec3b>(i,j)[0] = frameB;
                    image.at<cv::Vec3b>(i,j)[1] = frameG;
                    image.at<cv::Vec3b>(i,j)[2] = frameR;
                }
            }
            if(shouldVisualise){
                cv::imshow("Image", image);
                cv::waitKey(1);
            }
            ros::spinOnce();
            imageMsg.encoding = sensor_msgs::image_encodings::BGR8;
            imageMsg.image = image;
            imagePub.publish(imageMsg.toImageMsg());
            loop_rate.sleep();
        }
    }

    void LatencyCommandCallback(const geometry_msgs::Point& msg){
        frameR = msg.x;
        frameG = msg.y;
        frameB = msg.z;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Latency_tool");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Node initialised");
    LatencyTool latencyTool = LatencyTool(&nh);
    return 0;
}