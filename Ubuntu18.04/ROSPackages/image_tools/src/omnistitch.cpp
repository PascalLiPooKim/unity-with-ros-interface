#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <iostream>
#include <math.h>
#include <chrono>
#include <ros/ros.h>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
using namespace cv;
using namespace std;

class Omnicamera {
    private:
    int src_x;
    int src_y;
    int out_x;
    int out_y;
    int buf;
    int updaterate;
    double fov;
    cv_bridge::CvImagePtr left_fisheye;
    cv_bridge::CvImagePtr right_fisheye;
    bool shouldVisualise;
    std_msgs::Header imageHeader;
    
    std::string left_image_topic;
    std::string right_image_topic;

    ros::Subscriber left_image_sub;
    ros::Subscriber right_image_sub;
    ros::Publisher omni_image_pub;

    public:
    Omnicamera(ros::NodeHandle *nh){
        //Setting projection parameters
        nh->param<int>("src_width", src_x, 960);
        nh->param<int>("src_height", src_y, 960);
        nh->param<int>("out_width", out_x, 1920);
        nh->param<int>("out_height", out_y, 960);
        nh->param<int>("buffer", buf, 50);
        nh->param<double>("fov", fov, 220.00d);
        nh->param<bool>("visualise", shouldVisualise, false);

        nh->param<int>("update_rate", updaterate, 10);
        //Setting ROS parameters and creating publishers/subscribers
        nh->param<std::string>("left_image_topic", left_image_topic, "/omnicam/front/image_raw");
        nh->param<std::string>("right_image_topic", right_image_topic, "/omnicam/back/image_raw");
        ros::Rate loop_rate(updaterate);

        left_image_sub = nh->subscribe(left_image_topic, 100, &Omnicamera::left_image_callback, this);
        right_image_sub = nh->subscribe(right_image_topic, 100, &Omnicamera::right_image_callback, this);
        omni_image_pub = nh->advertise<sensor_msgs::Image>("/omnicam/stitched/image_raw", 100);
        
        ROS_INFO_STREAM("Beginning projection calculation");

        //Generating src -> out map and vector of out image indices
        std::vector<int> map = map_calculation(src_x, src_y, out_x, out_y, fov);
        int* map_p = map.data();

        std::vector<int> index = index_calculation(out_x, out_y);
        int* index_p = index.data();

        ROS_INFO_STREAM("Ended projection calculation, waiting for images");

        while (ros::ok()){
            if(left_fisheye!= NULL && right_fisheye!= NULL){

                cv::Mat leftproj = map_project(left_fisheye->image, out_x, out_y, map_p, index_p);
                cv::Mat rightproj = map_project(right_fisheye->image, out_x, out_y, map_p, index_p);

                Mat n_leftproj(out_y,out_x, CV_8UC3);
                leftproj(Rect(n_leftproj.cols/2, 0, n_leftproj.cols/2, n_leftproj.rows)).copyTo(n_leftproj(Rect(0, 0, n_leftproj.cols/2, n_leftproj.rows)));
                leftproj(Rect(0, 0, n_leftproj.cols/2, n_leftproj.rows)).copyTo(n_leftproj(Rect(n_leftproj.cols/2, 0, n_leftproj.cols/2, n_leftproj.rows)));

                cv::Mat stitch_img = stitch(n_leftproj, rightproj, buf);
                if(shouldVisualise){
                    visualise(stitch_img, "Stitched");
                }

                cv_bridge::CvImage out_msg;
                out_msg.header  = imageHeader;
                out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                out_msg.image = stitch_img;
                omni_image_pub.publish(out_msg.toImageMsg());
                ROS_INFO_STREAM("Stitching new images");
            }
            else{
                ROS_INFO_STREAM("NUll images");
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void left_image_callback(const sensor_msgs::Image& msg){
        imageHeader = msg.header;
        try
        {
            left_fisheye = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void right_image_callback(const sensor_msgs::Image& msg){
        try
        { 
            right_fisheye = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    template <typename T>
    std::vector<T> linspace(T a, T b, size_t N)
    {
        //Equivalent of np.linspace
        T h = (b - a) / static_cast<T>(N-1);
        std::vector<T> xs(N);
        typename std::vector<T>::iterator x;
        T val;
        for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
            *x = val;
        return xs;
    }

    void visualise(cv::Mat img, string windowstring)
    {
        //Visualise image using opencv
        namedWindow(windowstring); // Create a window

        cv::imshow(windowstring, img); // Show our image inside the created window.

        cv::waitKey(1);

        //destroyWindow(windowstring); //destroy the created window
    }

    std::vector<int> map_calculation(int src_x, int src_y, int out_x, int out_y, double fov)
    {
    //Poject function outputting the mapping from the output image to input image
    //Ouput: Mat of the same size as the projected output image containing index of equivalent source image pixels
    std::vector<int> map((int)2*out_y*out_x+2*out_y+2);
        for(int i = 0; i < out_y; i++)
            for(int j = 0; j < out_x; j++)
            {
                double normy = (i - out_y / 2.) / (out_y / 2.);
                double normx = (j - out_x / 2.) / (out_x / 2.);

                double lon = normx * M_PI;
                double lat = (normy * M_PI) / 2.;

                double Px = cos(lat) * cos(lon);
                double Py = cos(lat) * sin(lon);
                double Pz = sin(lat);

                double app = (fov * M_PI) / 180.;
                double r = (2 * atan2(sqrt(Px * Px + Pz * Pz), Py)) / app;
                double theta = atan2(Pz, Px);

                double x = r * cos(theta);
                double y = r * sin(theta);

                int fish_normx = (int) ((src_x / 2.) * x + (src_x / 2.));
                int fish_normy = (int) ((src_y / 2.) * y + (src_y / 2.));

                if (fish_normx > src_x or fish_normy > src_y)
                    continue;
                if (fish_normx < 0 or fish_normy < 0)
                    continue;

                map[2*out_x*i+2*j+0] = fish_normx;
                map[2*out_x*i+2*j+1] = fish_normy;
            }

        return map;
    }

    std::vector<int> index_calculation(int x, int y)
    {
        std::vector<int> out_index((int)2*y*x+2*y+2);
        for(int i = 0; i < y; i++)
            for(int j = 0; j < x; j++)
            {
                out_index[2*x*i+2*j+0] = j;
                out_index[2*x*i+2*j+1] = i;
            }
        return out_index;
    }

    cv::Mat map_project(cv::Mat src_img, int out_x, int out_y, int* map_pointer, int* index_pointer)
    {
        Mat out_img(out_y,out_x, CV_8UC3);
        int limit = ((int)2*out_y*out_x+2*out_y+2)/2;
        unsigned char *src_data = src_img.data;
        int src_step = src_img.step1();
        int src_channels = src_img.channels();

        unsigned char *out_data = out_img.data;
        int out_step = out_img.step1();
        int out_channels = out_img.channels();

        for(int i = 0; i < limit; i++)
        {
            out_data[out_step*index_pointer[(2*i) + 1] + out_channels*index_pointer[2*i] + 0] = src_data[src_step*map_pointer[(2*i) + 1] + src_channels*map_pointer[2*i] + 0];
            out_data[out_step*index_pointer[(2*i) + 1] + out_channels*index_pointer[2*i] + 1] = src_data[src_step*map_pointer[(2*i) + 1] + src_channels*map_pointer[2*i] + 1];
            out_data[out_step*index_pointer[(2*i) + 1] + out_channels*index_pointer[2*i] + 2] = src_data[src_step*map_pointer[(2*i) + 1] + src_channels*map_pointer[2*i] + 2];
            //out_img.at<cv::Vec3b>(index_pointer[(2*i) + 1], index_pointer[2*i]) = src_img.at<cv::Vec3b>(map_pointer[(2*i) + 1], map_pointer[2*i]);
        }
        flip(out_img, out_img, 1);
        return out_img;
    }

    std::vector<double> sig_gen(int buf)
    {
        std::vector<int> linvector = linspace(-4, 4, buf);
        std::vector<double> sigvector(buf);
        std::vector<double> invsigvector(buf);

        for(int i=0; i<linvector.size(); i++)
        {
            double sigi = (1/(1+ exp(-linvector.at(i))));
            sigvector.at(i) = sigi;
        }

        return sigvector;
    }

    std::vector<double> inv_sig_gen(int buf)
    {
        std::vector<int> linvector = linspace(-4, 4, buf);
        std::vector<double> invsigvector(buf);

        for(int i=0; i<linvector.size(); i++)
        {
            double invsigi = 1-(1/(1+ exp(-linvector.at(i))));
            invsigvector.at(i) = invsigi;
        }

        return invsigvector;
    }

    cv::Mat stitch(const Mat leftimg, const Mat rightimg, const int buf)
    {
        //Generatingout_index[ind_x] sigmoid and inv_sigmoid vectors
        std::vector<double> sig = sig_gen(2*buf+1);
        std::vector<double> invsig = inv_sig_gen(2*buf+1);

        //Creating output image and copying over non-sticted areas of original
        Mat out_img(leftimg.rows, leftimg.cols, CV_8UC3);
        rightimg(Rect(buf, 0 ,out_img.cols/2-2*buf, out_img.rows)).copyTo(out_img(Rect(buf, 0 ,out_img.cols/2-2*buf, out_img.rows)));
        leftimg(Rect(out_img.cols/2+buf, 0 ,out_img.cols/2-2*buf, out_img.rows)).copyTo(out_img(Rect(out_img.cols/2+buf, 0 ,out_img.cols/2-2*buf, out_img.rows)));

        //Generating mats for image sorting
        Mat dst;
        for(int i=0; i<2*buf+1; i++)
        {
            //Middle stitch
            cv::addWeighted(leftimg(Rect(out_img.cols/2+buf-i, 0, 1, out_img.rows)), invsig.at(i), rightimg(Rect(out_img.cols/2+buf-i, 0, 1, out_img.rows)), sig.at(i), 0, dst);
            dst.copyTo(out_img(Rect(out_img.cols/2+buf-i, 0, 1, out_img.rows)));

            if(i<buf+1)
            {
                //Left side stitch
                cv::addWeighted(leftimg(Rect(buf-i, 0, 1, out_img.rows)), sig.at(i), rightimg(Rect(buf-i, 0, 1, out_img.rows)), invsig.at(i), 0, dst);
                dst.copyTo(out_img(Rect(buf-i, 0, 1, out_img.rows)));

                //Right side stitch
                cv::addWeighted(leftimg(Rect(out_img.cols-(i+1), 0, 1, out_img.rows)), invsig.at(i), rightimg(Rect(out_img.cols-(i+1), 0, 1, out_img.rows)), sig.at(i), 0, dst);
                dst.copyTo(out_img(Rect(out_img.cols-(i+1), 0, 1, out_img.rows)));
            }
        }

        return out_img;
    }
};


int main(int argc, char** argv)
{
    //TODO: Set up ros, Create new class for subscribers and publishers and move all needed functions
    //into new class

//     //Setting constants
//     int src_x = 640;
//     int src_y = 640;
//     int out_x = 1920;
//     int out_y = 960;
//     int buf = 10;
//     int limit = ((int)2*out_y*out_x+2*out_y+2)/2;
//     double fov = 202.5;

//     //Generating src -> out map and vector of out image indices
//     std::vector<int> map = map_calculation(src_x, src_y, out_x, out_y, fov);
//     int* map_p = map.data();

//     std::vector<int> index = index_calculation(out_x, out_y);
//     int* index_p = index.data();

//     Mat frame = imread("/home/pi/Scripts/test.jpg", IMREAD_COLOR);

//     //Starting timer


//     //Splitting and rotating left and right image
//     Mat left = frame(Rect(0, 0, frame.cols/2, frame.rows-80));
//     Mat right = frame(Rect(frame.cols/2, 0, frame.cols/2, frame.rows-80));
//     rot90(left, 1);
//     rot90(right, 2);
    
//     auto start = std::chrono::system_clock::now();

//     //Applying projection
//     cv::Mat leftproj = map_project(left, out_x, out_y, map_p, index_p);
//     cv::Mat rightproj = map_project(right, out_x, out_y, map_p, index_p);
    
    
//     //Stopping timer
//     auto end = std::chrono::system_clock::now();
//     auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//     std::cout << elapsed.count() << '\n';

//     //Testing out Parralisation
// //    cv::Mat leftproj(out_y, out_x, CV_8UC3);
// //    cv::Mat rightproj(out_y, out_x, CV_8UC3);

// //    Parallel_project parallel_left(leftproj, left, map_p, index_p);
// //    cv::parallel_for_(cv::Range(0, ((int)2*out_y*out_x+2*out_y+2)/2), parallel_left);

// //    Parallel_project parallel_right(rightproj, right, map_p, index_p);
// //    cv::parallel_for_(cv::Range(0, ((int)2*out_y*out_x+2*out_y+2)/2), parallel_right);

//     //Flipping
//     Mat n_leftproj(out_y,out_x, CV_8UC3);
//     leftproj(Rect(n_leftproj.cols/2, 0, n_leftproj.cols/2, n_leftproj.rows)).copyTo(n_leftproj(Rect(0, 0, n_leftproj.cols/2, n_leftproj.rows)));
//     leftproj(Rect(0, 0, n_leftproj.cols/2, n_leftproj.rows)).copyTo(n_leftproj(Rect(n_leftproj.cols/2, 0, n_leftproj.cols/2, n_leftproj.rows)));

//     Mat stitch_img = stitch(n_leftproj, rightproj, buf);;

//     //Visualising image
//     visualise(stitch_img, "Stitched");
    ros::init(argc, argv, "omnicamera");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("Node initialised");
    Omnicamera omnicamera = Omnicamera(&nh);
    return 0;
}
