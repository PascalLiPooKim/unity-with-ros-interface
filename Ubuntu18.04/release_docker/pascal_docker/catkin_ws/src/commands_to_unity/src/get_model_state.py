#!/usr/bin/env python

"""
Script to publish distance between goal and Husky. The publish message is used for Text-to-Speech in Windows.
(Currently not being used)
"""

# Import libraries
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from gazebo_msgs.srv import GetModelState
import time
import numpy as np


# Subscribe to ground truth state of Husky in Gazebo
def pub_goal_position():
   
    rospy.Subscriber("/ground_truth/state", Odometry, callback)


# Callback function to publish distance between goal position and Husky
def callback(data):
    position = data.pose.pose.position
    husky_x = position.x
    husky_y = position.y

    pub = rospy.Publisher("/tts/goal_dist", Float32, queue_size=1)
    model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    model_pose = model_state("r2", "world")

    point = Point()
    point.x = model_pose.pose.position.x
    point.y = model_pose.pose.position.y
    point.z = model_pose.pose.position.z

    xy_goal_pos = np.array([point.x, point.y])
    xy_husky_pos = np.array([husky_x, husky_y])


    rate = rospy.Rate(100)
    dist = np.linalg.norm(xy_goal_pos - xy_husky_pos)
    dist_msg = Float32()
    dist_msg.data = dist

    pub.publish(dist_msg)

    rate.sleep()

    


if __name__ == '__main__':
    rospy.init_node('pub_model_state')
    pub_goal_position()
    rospy.spin()
