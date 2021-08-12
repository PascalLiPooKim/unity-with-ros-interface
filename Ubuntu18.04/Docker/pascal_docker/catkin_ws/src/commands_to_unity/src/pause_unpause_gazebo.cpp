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
    std::cin >> keyPressed;
    if (keyPressed == 'p'){
        pauseGazebo.call(pauseSrv);
        std::cout << "[ INFO] [" << ros::Time::now() << "]: Pause simulation" << std::endl;
        pause.data = true;
        // pause_sim.publish(pause);
    }
    else if (keyPressed == 'r'){
        unpauseGazebo.call(unpauseSrv);
        std::cout << "[ INFO] [" << ros::Time::now() << "]: Unpause simulation" << std::endl;
        pause.data = false;
        
    }
    else {
        std::cout << "[ INFO] [" << ros::Time::now() << "]: Undefined key" << std::endl;
    }

    pause_sim.publish(pause);

    // ros::spinOnce();
    r.sleep();

  }


//     if (kbhit()) 
//     {
//     int keyPressed = getchar();
//     std_msgs::Bool pause;
//     std_srvs::Empty pauseSrv;
//     std_srvs::Empty unpauseSrv;


//     std::cout << "[ INFO] [" << ros::Time::now() << "]: Keypressed" << std::endl;
//     if (keyPressed == 'p') // if h is pressed
//     {
//         pauseGazebo.call(pauseSrv);
//         pause.data = true;
//         std::cout << "[ INFO] [" << ros::Time::now() << "]: Pause simulation" << std::endl;
//         pause_sim.publish(pause);

        
//     } 
//     else if (keyPressed == 'r'){
//         unpauseGazebo.call(unpauseSrv);
//         pause.data = false;
//         std::cout << "[ INFO] [" << ros::Time::now() << "]: Unpause simulation" << std::endl;
//         pause_sim.publish(pause);

//     }
     
//     }
      

//     ros::spinOnce();
//     r.sleep();
// }

    return (0);
}