#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>




int kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;
  
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  
  ch = getchar();
  
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  
  return 0;
}


int main (int argc, char** argv)
{
  /** 
    * Initialize ROS 
    *
    * The ros::init() function needs to see argc and argv so that it can
    * perform any ROS arguments and name remapping that were provided at the
    * command line. For programmatic remappings you can use a different version
    * of init() which takes remappings directly, but for most command-line
    * programs, passing argc and argv is the easiest way to do it. The third
    * argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
  ros::init (argc, argv, "kbhit_to_unity");
  
  /**
    * NodeHandle is the main access point to communications with the ROS
    * system. The first NodeHandle constructed will fully initialize this node,
    * and the last NodeHandle destructed will close down the node.
    */
  ros::NodeHandle nh ("~");


  ros::Publisher button_pub =
    nh.advertise<std_msgs::Int32>("/stop_timer", 1);

  ros::Publisher degrade_pub =
    nh.advertise<std_msgs::Bool>("/degrade_pointcloud", 1);

    ros::Rate r (100);
  while (ros::ok ())
  {
    
    
    // check if a key is pressed without blocking the while
    if (kbhit()) 
    {
      int ch = getchar();
      std_msgs::Int32 button;
      std_msgs::Bool degrade;

      if (ch == 's') // if h is pressed
      {
        button.data = 1;
        std::cout<<"Stop timer and counter"<<std::endl;
        button_pub.publish(button);
      } 
      else if (ch == 'r'){
        button.data = 0;
        std::cout<<"Resume timer and counter"<<std::endl;
        button_pub.publish(button);
      }
      else if (ch == 'd'){
        degrade.data = true;
        std::cout<<"Degrade point cloud"<<std::endl;
        degrade_pub.publish(degrade);
      }
      else if (ch == 'u'){
        degrade.data = false;
        degrade_pub.publish(degrade);
      }
    }
      

    ros::spinOnce();
    r.sleep();
  }
    
  return (0);
}