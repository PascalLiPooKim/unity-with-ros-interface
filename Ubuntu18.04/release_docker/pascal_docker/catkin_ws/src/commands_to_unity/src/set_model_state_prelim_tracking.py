#!/usr/bin/env python

"""
Script for spawning objects around Husky for preliminary experiment
(Blind Test).
"""
import rospy
import rospkg
import time
from gazebo_msgs.msg import ModelState
import numpy as np
import rosbag

# Global publisher for all models
pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)

# Function to make model perform trajectories and record them (not being used)
def pose_publisher_prelim_tracking(model_name, x, y, iter, r, s):

    pose_msg = ModelState()
    pose_msg.model_name = model_name
    
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pub.publish(pose_msg)

    time.sleep(2)

    i = 0

    rate = rospy.Rate(r)

    rospack = rospkg.RosPack()
    path = rospack.get_path("commands_to_unity")

    bag = rosbag.Bag(path + "/bags/model_traj.bag", 'w')
    
    try:

        for i in range(iter):
            pose_msg.pose.position.y -= s/r
            print(pose_msg.pose.position.x, pose_msg.pose.position.y)
            pub.publish(pose_msg)
            rate.sleep()


        for i in range(iter):
            pose_msg.pose.position.x -= s/r
            bag.write("model_traj", pose_msg.pose.position)
            print(pose_msg.pose.position.x, pose_msg.pose.position.y)
            pub.publish(pose_msg)
            rate.sleep()

        for i in range(iter):
            pose_msg.pose.position.y += s/r
            bag.write("model_traj", pose_msg.pose.position)
            print(pose_msg.pose.position.x, pose_msg.pose.position.y)
            pub.publish(pose_msg)
            rate.sleep()


        for i in range(iter):
            pose_msg.pose.position.x += s/r
            bag.write("model_traj", pose_msg.pose.position)
            print(pose_msg.pose.position.x, pose_msg.pose.position.y)
            pub.publish(pose_msg)
            rate.sleep()


        for i in range(iter):
            pose_msg.pose.position.y -= s/r
            pose_msg.pose.position.x -= s/r
            bag.write("model_traj", pose_msg.pose.position)
            print(pose_msg.pose.position.x, pose_msg.pose.position.y)
            pub.publish(pose_msg)
            rate.sleep()

    finally:
        bag.close()

    
# Function to publish new coordinate of models to be spawned
def pose_publisher_prelim_static(model_name, x, y):
    rate = rospy.Rate(30)
    pose_msg = ModelState()
    pose_msg.model_name = model_name
    
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pub.publish(pose_msg)
    rate.sleep()

# Function to run functions to spawn objects when keyboard numerical key is pressed
def move_models_near_husky():

    while True:

        input_key = input()

        if input_key == 2:
            pose_publisher_prelim_static("Construction Barrel", 1.5, 0.0)
            time.sleep(1)

        elif input_key == 1:
            pose_publisher_prelim_static("r2", 0.0, 1.5)
            time.sleep(1)

        elif input_key == 3:
            pose_publisher_prelim_static("Dumpster", 0.0, -2.0)
            time.sleep(1)


        elif input_key == 4:
            move_model_around_clockwise(-2.0, 1.0, 2.0, 50)
            

        elif input_key == 5:
            move_model_around_anticlockwise(-2.0, -1.0, 2.0, 50)


        elif input_key == 8:
            pose_publisher_prelim_static("Construction Barrel", 20.0, 20.0)
            time.sleep(1)


        elif input_key == 7:
            pose_publisher_prelim_static("r2", 20.0, 18.0)
            time.sleep(1)

        elif input_key == 9:
            pose_publisher_prelim_static("Dumpster", 20.0, 15.0)
            time.sleep(1)


        elif input_key == 0:
            break

        else:
            print("Invalid Key pressed")

# Make model moves clockwise
def move_model_around_clockwise(init_x, init_y, s, r):
    pose_msg = ModelState()
    pose_msg.model_name = "mars_rover"
    
    pose_msg.pose.position.x = init_x
    pose_msg.pose.position.y = init_y

    pub.publish(pose_msg)

    rate = rospy.Rate(r)

    for i in range(100):
        pose_msg.pose.position.x += s/r
        print(pose_msg.pose.position.x, pose_msg.pose.position.y)
        pub.publish(pose_msg)
        rate.sleep()

    for i in range(50):
        pose_msg.pose.position.y -= s/r
        print(pose_msg.pose.position.x, pose_msg.pose.position.y)
        pub.publish(pose_msg)
        rate.sleep()


    for i in range(100):
        pose_msg.pose.position.x -= s/r
        print(pose_msg.pose.position.x, pose_msg.pose.position.y)
        pub.publish(pose_msg)
        rate.sleep()

# Make model moves anti-clockwise
def move_model_around_anticlockwise(init_x, init_y, s, r):
    pose_msg = ModelState()
    pose_msg.model_name = "mars_rover"
    
    pose_msg.pose.position.x = init_x
    pose_msg.pose.position.y = init_y

    pub.publish(pose_msg)

    rate = rospy.Rate(r)

    for i in range(100):
        pose_msg.pose.position.x += s/r
        print(pose_msg.pose.position.x, pose_msg.pose.position.y)
        pub.publish(pose_msg)
        rate.sleep()

    for i in range(50):
        pose_msg.pose.position.y += s/r
        print(pose_msg.pose.position.x, pose_msg.pose.position.y)
        pub.publish(pose_msg)
        rate.sleep()


    for i in range(100):
        pose_msg.pose.position.x -= s/r
        print(pose_msg.pose.position.x, pose_msg.pose.position.y)
        pub.publish(pose_msg)
        rate.sleep()



    


if __name__ == '__main__':
      rospy.init_node('pose_publisher')
      try:
        time.sleep(1)
        move_models_near_husky()
      except rospy.ROSInterruptException:
          pass
