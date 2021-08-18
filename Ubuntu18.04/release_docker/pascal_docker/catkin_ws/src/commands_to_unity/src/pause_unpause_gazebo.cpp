/**
 * Script used to pause and unpause Gazebo simulation. When paused, the simulation in
 * Unity too will be paused and a cube will be spawned to block the operator camera.
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

#include "std_srvs/Empty.h"



// https://answers.ros.org/question/319774/ros-c-pause-unpause-gazebo/
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
  ros::init (argc, argv, "pause_unpause_gazebo");
  
  /**
    * NodeHandle is the main access point to communications with the ROS
    * system. The first NodeHandle constructed will fully initialize this node,
    * and the last NodeHandle destructed will close down the node.
    */
  ros::NodeHandle nh ("~");


  ros::Rate r (100);

  char keyPressed;

  // Services to pause and resume Gazebo simulation
  ros::ServiceClient pauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  ros::ServiceClient unpauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
 
  std_srvs::Empty pauseSrv;
  std_srvs::Empty unpauseSrv;


  ros::Publisher pause_sim =
    nh.advertise<std_msgs::Bool>("/pause_gazebo", 1);

    std_msgs::Bool pause;
    pause.data = false;
 
  while (ros::ok ())
  {
    std::cin >> keyPressed; // wait for key to be pressed
    if (keyPressed == 'p'){ // if p is pressed
        pauseGazebo.call(pauseSrv);
        std::cout << "[ INFO] [" << ros::Time::now() << "]: Pause simulation" << std::endl;
        pause.data = true;
    }
    else if (keyPressed == 'r'){ // if r is pressed
        unpauseGazebo.call(unpauseSrv);
        std::cout << "[ INFO] [" << ros::Time::now() << "]: Unpause simulation" << std::endl;
        pause.data = false;
        
    }
    else {
        std::cout << "[ INFO] [" << ros::Time::now() << "]: Undefined key" << std::endl;
    }

    pause_sim.publish(pause);

    r.sleep();

  }

    return (0);
}