#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <iostream>

class lineDetect {
    private:
    float avgAngleArray[5];
    int counter = 0;

    public:
    ros::Subscriber imgSub;
    ros::Publisher anglePub;

    lineDetect(ros::NodeHandle *nh){
        std::cout << "Constructed Class" << "\n";
        imgSub = nh->subscribe("/image_test", 100, &lineDetect::imageCallback, this);
        anglePub = nh->advertise<std_msgs::Float64>("/avg_angle", 100); 
    }

    void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg){
        float avgAngleFrame = 0;
        cv::namedWindow("test");
        cv::Mat dst, cdst, cdstP;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, std::string("bgr8"));

        //Edge Detection
        cv::Canny(cv_ptr->image, dst,50, 200, 3);

        // Copy edges to the images that will display the results in BGR
        cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);

        // Standard Hough Line Transform
        std::vector<cv::Vec2f> lines; // will hold the results of the detection
        HoughLines(dst, lines, 1, CV_PI/180, 50, 0, 0 ); // runs the actual detection
        // Draw the lines
        for( size_t i = 0; i < lines.size(); i++ ){
            float rho = lines[i][0], theta = lines[i][1];
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            line( cdst, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);
            double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x);
            if(angle<0) {
                angle+=CV_PI;
            }
            avgAngleFrame += (float)angle - 1.57079633;
        }
        
        avgAngleFrame = avgAngleFrame/lines.size();
        float avgAngle = averageAngle(avgAngleFrame);
        std_msgs::Float64 anglemsg;
        anglemsg.data = averageAngle(avgAngleFrame);
        anglePub.publish(anglemsg);

        cv::imshow("CannyDetection", dst);
        cv::waitKey(1);
    }

    float averageAngle(float avgAngleFrame){ 
        avgAngleArray[counter] = avgAngleFrame;
        counter += 1;
        if(counter>4){
            counter=0;
        }
        float avgAngle;
        int i;
        for(i=0; i<5; ++i){
            avgAngle += avgAngleArray[i];
        }
        return avgAngle / 5; 
    }
};

int main(int argc, char** argv)
{
    std::cout << "Starting node" << "\n";
    ros::init(argc, argv, "cameraview");
    ros::NodeHandle n;
    lineDetect ld = lineDetect(&n);
    std::cout << "Node Set Up" << "\n";
    ros::spin();
    
}
