#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from gazebo_msgs.srv import GetModelState
import time
import numpy as np


# husky_x = 0
# husky_y = 0



def pub_goal_position():
    # model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    
    # pub = rospy.Publisher("/tts/goal_location", Point, queue_size=1)
    rospy.Subscriber("/ground_truth/state", Odometry, callback)

    # model_pose = model_state("r2", "world")

    # point = Point()
    # point.x = model_pose.pose.position.x
    # point.y = model_pose.pose.position.y
    # point.z = model_pose.pose.position.z

    # xy_goal_pos = np.array([point.x, point.y])
    # xy_husky_pos = np.array([husky_x, husky_y])



    # # while not rospy.is_shutdown():
    # dist = np.linalg.norm(xy_goal_pos - xy_husky_pos)
    # # print(dist)
    # pub.publish(point)
    # time.sleep(1)

    # rospy.spin()

def callback(data):
    position = data.pose.pose.position
    husky_x = position.x
    husky_y = position.y

    # pub = rospy.Publisher("/tts/goal_location", Point, queue_size=10)
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
    # while not rospy.is_shutdown():
    dist = np.linalg.norm(xy_goal_pos - xy_husky_pos)
    # print(dist)
    dist_msg = Float32()
    dist_msg.data = dist

    pub.publish(dist_msg)
    #print(xy_goal_pos, xy_husky_pos, dist)

    rate.sleep()

    


if __name__ == '__main__':
    rospy.init_node('pub_model_state')
    pub_goal_position()
    rospy.spin()
