#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
    cv::namedWindow("test");
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, std::string("bgr8"));
    cv::imshow("test", cv_ptr->image);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    while(ros::ok)
    {
        ros::init(argc, argv, "cameraview");
        ros::NodeHandle n;
        ros::Subscriber imgSub = n.subscribe("/drone_camera", 100, imageCallback);
        ros::spin();
    }
    return 0;
}
