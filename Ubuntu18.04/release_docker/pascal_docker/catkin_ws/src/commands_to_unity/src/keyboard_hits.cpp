/**
 * Script to stop timer and keystroke counter in Unity. Furthermore, script is used to degrade
 * visual information when corresponding key is pressed.
 * Codes based on test_ce3.cpp of CW3 of COMP0129(20/21)
 */




#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>



// Function to check if any keyboard key was pressed
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
    
    
    // Check if a key is pressed without blocking the while and publish booleans to functions in Unity
    if (kbhit()) 
    {
      int ch = getchar();
      std_msgs::Int32 button;
      std_msgs::Bool degrade;

      if (ch == 's') // if s is pressed
      {
        button.data = 1;
        std::cout << std::endl;
        std::cout << "[ INFO] [" << ros::Time::now() << "]: Stop timer and counter" << std::endl;
        button_pub.publish(button);
      } 
      else if (ch == 'r'){ // if r is pressed
        button.data = 0;
        std::cout << std::endl;
        std::cout << "[ INFO] [" << ros::Time::now() << "]: Resume timer and counter" << std::endl;
        button_pub.publish(button);
      }
      else if (ch == 'd'){ // if d is pressed
        degrade.data = true;
        std::cout << std::endl;
        std::cout << "[ INFO] [" << ros::Time::now() << "]: Degrade RGBD and LiDAR visual feedback" << std::endl;
        degrade_pub.publish(degrade);
      }
      else if (ch == 'u'){ // if u is pressed
        degrade.data = false;
        std::cout << std::endl;
        std::cout << "[ INFO] [" << ros::Time::now() << "]: Deactivate vision degradation" << std::endl;
        degrade_pub.publish(degrade);
      }
    }
      

    ros::spinOnce();
    r.sleep();
  }
    
  return (0);
}