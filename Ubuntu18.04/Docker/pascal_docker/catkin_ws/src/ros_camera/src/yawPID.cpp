#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>


class yawPID{
    private:
    float error_prior = 0;
    float integral_prior = 0;
    float kp;
    float ki;
    float kd;
    float bias = 0;
    float setPoint = 0;
    float t = 0.01;
    

    public:
    ros::Subscriber boolSub;
    ros::Subscriber angleSub;
    ros::Publisher posePub;
    bool enablePID = false;
    struct Quaternion
    {
        double w, x, y, z;
    };

    yawPID(ros::NodeHandle *nh){
        std::cout << "Constructed Class" << "\n";
        boolSub = nh->subscribe("/pid", 100, &yawPID::boolCallback, this);
        angleSub = nh->subscribe("/avg_angle", 100, &yawPID::angleCallback, this);
        posePub = nh->advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);
    }

    Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
    {
        // Abbreviations for the various angular functions
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

    void boolCallback(std_msgs::Bool msg){
        enablePID = msg.data;
    }

    void angleCallback(std_msgs::Float64 msg){
        float angle = msg.data;
        float poseInput = PID(angle);
        constructPose(poseInput);
        ros::Duration(t).sleep();
    }

    float PID(float angle){
        float error = setPoint - angle;
        float i = error * t;
        float d = (error - error_prior)/t;
        float output = kp*error + ki*i + kd*d + bias;
        error_prior = error;
        integral_prior = i;
        return output;
    }

    void constructPose(float poseInput)
    {
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = 50;
        poseMsg.pose.position.y = 3;
        poseMsg.pose.position.z = 0;
        Quaternion rotation = ToQuaternion(poseInput, 0, 0);
        poseMsg.pose.orientation.x = rotation.x;
        poseMsg.pose.orientation.y = rotation.x;
        poseMsg.pose.orientation.z = rotation.z;
        poseMsg.pose.orientation.w = rotation.w;
        if(enablePID){
            posePub.publish(poseMsg);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yawPID");
    ros::NodeHandle n;
    yawPID pid = yawPID(&n);
    ros::spin();
}